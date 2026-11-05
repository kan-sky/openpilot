import json
import subprocess
from types import SimpleNamespace as NS

from openpilot.selfdrive.navdy.diagnostics import LinkDiagnostics


def recorder():
  emitted = []
  return LinkDiagnostics(lambda name, **event: emitted.append((name, event))), emitted


def receiver(clears=205, session=39):
  return {'wireless': {'session': session, 'clears': clears, 'owner': 'bluetooth', 'usbUpdates': 0},
          'displayPower': {'state': 'offroad', 'fresh': True, 'invalidations': 220, 'generation': 525},
          'modelNavigation': {'privateRoute': [[1, 2]]}, 'navigationFeedback': {'privateDestination': 'SECRET'}}


def feedback(diagnostics, response):
  diagnostics.feedback(response, transport='bluetooth', received_at=20, elapsed_ms=55, wireless={'rttMs': 60})


def test_historical_counters_are_baseline_not_a_new_failure():
  diagnostics, emitted = recorder()
  feedback(diagnostics, receiver())
  for _ in range(100):
    feedback(diagnostics, receiver())
  assert len(emitted) == 1
  assert emitted[0][1]['kind'] == 'receiver_baseline'
  assert diagnostics.snapshot()['lastFailure'] is None


def test_new_clear_and_session_change_are_preserved_without_claiming_cause():
  diagnostics, emitted = recorder()
  feedback(diagnostics, receiver())
  feedback(diagnostics, receiver(clears=206))
  feedback(diagnostics, receiver(clears=0, session=1))
  assert [event['kind'] for _, event in emitted] == ['receiver_baseline', 'receiver_changed', 'receiver_changed']
  event = emitted[1][1]
  assert event['previous']['wireless']['clears'] == 205
  assert event['current']['wireless']['clears'] == 206
  assert event['receivedAt'] == 20
  assert event['roundTripMs'] == 55
  assert event['batchLatestWireless']['rttMs'] == 60
  assert diagnostics.snapshot()['lastFailure'] is None


def test_no_events_from_sequence_or_latency_alone_or_private_feedback():
  diagnostics, emitted = recorder()
  response = receiver()
  feedback(diagnostics, response)
  response['wireless']['receivedSeq'] = 9000
  response['wireless']['appliedSeq'] = 9000
  response['displayPower']['offroadMs'] = 999999
  feedback(diagnostics, response)
  assert len(emitted) == 1
  assert diagnostics.receiver['wireless']['receivedSeq'] == 9000
  assert diagnostics.receiver['wireless']['appliedSeq'] == 9000
  assert 'SECRET' not in json.dumps(diagnostics.snapshot())
  assert 'privateRoute' not in json.dumps(diagnostics.snapshot())


def test_failure_survives_recovery_and_bounded_ring_eviction():
  diagnostics, _ = recorder()
  feedback(diagnostics, receiver())
  diagnostics.failure(phase='exchange', transport='bluetooth', error=TimeoutError('Navdy wireless v2 response deadline'),
                      connected_at=10, wireless={'rttMs': 900, 'peer': 'PRIVATE', 'ackAgeMs': float('inf')})
  failure = diagnostics.snapshot()['lastFailure']
  feedback(diagnostics, receiver(clears=207, session=40))
  for _ in range(40):
    diagnostics.record('transport_open', transport='bluetooth', connectedAt=30)
  snapshot = diagnostics.snapshot()
  assert len(snapshot['events']) == 16
  assert all(event['runId'] == snapshot['runId'] for event in snapshot['events'])
  assert snapshot['lastFailure'] == failure
  assert failure['receiver']['wireless']['session'] == 39
  assert failure['phase'] == 'exchange'
  assert failure['errorType'] == 'TimeoutError'
  assert failure['wireless'] == {'rttMs': 900}
  snapshot['lastFailure']['reason'] = 'mutated'
  assert diagnostics.snapshot()['lastFailure']['reason'] == 'Navdy wireless v2 response deadline'
  json.dumps(diagnostics.snapshot(), allow_nan=False)


def test_logging_failure_cannot_escape_into_display_transport():
  def failed(*args, **kwargs):
    raise OSError('logging unavailable')
  diagnostics = LinkDiagnostics(failed)
  feedback(diagnostics, receiver())
  diagnostics.failure(phase='status', transport='usb', error=OSError('disk'), connected_at=0, wireless=None)
  assert diagnostics.snapshot()['lastFailure']['phase'] == 'status'


def test_missing_receiver_diagnostics_do_not_invent_reset():
  diagnostics, emitted = recorder()
  feedback(diagnostics, receiver())
  feedback(diagnostics, {'navigationFeedback': None})
  assert len(emitted) == 1


def test_no_observation_has_bounded_counts_and_original_receipt_without_per_ack_events(monkeypatch):
  from openpilot.selfdrive.navdy import diagnostics as module
  from openpilot.selfdrive.navdy.tests.test_navigation import packet
  monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
  diagnostics, emitted = recorder()
  local = []
  diagnostics.journal = NS(submit=lambda row: local.append(row), stats=dict)
  for now in range(10, 40):
    diagnostics.feedback({'navigationFeedback': None, 'modelNavigation': None}, transport='bluetooth',
                         received_at=now, elapsed_ms=123., wireless={'peer': 'PRIVATE'})
  feedback(diagnostics, packet())
  feedback(diagnostics, {'navigationFeedback': None, 'private': 'PRIVATE'})
  snapshot = diagnostics.snapshot()
  expected = {'noObservation': {'count': 30, 'lastReceivedAt': 39},
              'observation': {'count': 1, 'lastReceivedAt': 20},
              'incomplete': {'count': 1, 'lastReceivedAt': 20}}
  assert snapshot['feedback'] == expected
  assert snapshot['lastFailure'] is None and snapshot['events'] == []
  assert not emitted and not local  # No new cloudlog or per-ACK disk cadence.
  snapshot['feedback']['noObservation']['count'] = 0
  assert diagnostics.snapshot()['feedback'] == expected
  diagnostics.periodic(connected=True, transport='bluetooth', bluetooth_only=True, retry_in=0, wireless=None)
  assert local[-1]['kind'] == 'periodic_status'
  assert local[-1]['feedback'] == expected
  assert 'PRIVATE' not in json.dumps(local)


def test_feedback_stats_saturate_and_reject_invalid_diagnostic_timestamps():
  from openpilot.selfdrive.navdy.diagnostics import COUNTER_MAX
  diagnostics, _ = recorder()
  diagnostics.feedback_stats['noObservation']['count'] = COUNTER_MAX - 1
  values = [21., 20., -1., True, 'PRIVATE', float('nan'), float('inf')]
  for value in values:
    diagnostics.feedback({'navigationFeedback': None, 'modelNavigation': None}, transport='bluetooth',
                         received_at=value, elapsed_ms=0., wireless=None)
  assert diagnostics.snapshot()['feedback']['noObservation'] == {'count': COUNTER_MAX, 'lastReceivedAt': 21.}
  json.dumps(diagnostics.snapshot(), allow_nan=False)


def test_usb_heartbeat_counter_does_not_evict_failure_history():
  diagnostics, emitted = recorder()
  for count in range(100):
    response = receiver()
    response['wireless']['usbUpdates'] = count
    feedback(diagnostics, response)
  assert len(emitted) == 1
  assert diagnostics.receiver['wireless']['usbUpdates'] == 99


def test_exception_commands_paths_addresses_and_data_are_not_persisted():
  diagnostics, emitted = recorder()
  errors = [subprocess.TimeoutExpired(['tool', 'PRIVATE'], 5),
            subprocess.CalledProcessError(1, ['tool', 'PRIVATE']),
            PermissionError(13, 'PRIVATE', '/PRIVATE'), ValueError('PRIVATE'), ConnectionError('PRIVATE')]
  for error in errors:
    diagnostics.failure(phase='connect', transport='bluetooth', error=error, connected_at=0, wireless=None)
  assert 'PRIVATE' not in json.dumps(emitted)
  assert 'PRIVATE' not in json.dumps(diagnostics.snapshot())


def test_source_state_changes_are_distinct_from_radio_failures():
  diagnostics, emitted = recorder()
  sources = {key: {'usable': True, 'reason': ''}
             for key in ('deviceState', 'pandaStates', 'carState', 'selfdriveState')}
  payload = {'displayPower': 'onroad', 'ambientState': 'driving'}
  diagnostics.sources(sources, payload)
  diagnostics.sources(sources, payload)
  sources['pandaStates'] = {'usable': False, 'reason': 'stale'}
  diagnostics.sources(sources, {'displayPower': 'unknown', 'ambientState': 'disconnected'})
  assert len(emitted) == 2
  assert emitted[1][1]['kind'] == 'source_changed'
  assert emitted[1][1]['state']['missing'] == {'pandaStates': 'stale'}
  assert diagnostics.snapshot()['lastFailure'] is None
  diagnostics.sources(sources, payload, can_timeout=True)
  assert emitted[-1][1]['state']['missing']['carState'] == 'can_timeout'


def test_periodic_status_has_bounded_phase_timings_and_receiver_sequences(monkeypatch):
  from openpilot.selfdrive.navdy import diagnostics as module
  now = [100.]
  monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
  diagnostics, emitted = recorder()
  local = []
  diagnostics.journal = NS(submit=lambda row: local.append(row), stats=dict)
  response = receiver()
  response['wireless'].update(receivedSeq=123, appliedSeq=122)
  feedback(diagnostics, response)
  diagnostics.begin_phase('exchange')
  now[0] += .2
  diagnostics.exchanged(2)
  diagnostics.end_phase()
  for _ in range(100):
    diagnostics.periodic(connected=True, transport='bluetooth', bluetooth_only=True, retry_in=0, wireless={'sequence': 124})
  statuses = [row for row in local if row['kind'] == 'periodic_status']
  assert len(statuses) == 1
  assert not any(row['kind'] == 'periodic_status' for _, row in emitted)
  assert statuses[0]['receiver']['wireless']['receivedSeq'] == 123
  assert statuses[0]['receiver']['wireless']['appliedSeq'] == 122
  assert statuses[0]['exchanges'] == 1 and statuses[0]['replies'] == 2
  assert abs(statuses[0]['timings']['exchange']['maxMs'] - 200) < 1e-6
  assert diagnostics.timings == {}
  now[0] += 5
  diagnostics.periodic(connected=False, transport='bluetooth', bluetooth_only=True, retry_in=3, wireless=None)
  assert local[-1]['kind'] == 'periodic_status'
  assert local[-1]['timings'] == {}
  assert local[-1]['receiverAt'] == 20


def test_new_local_events_never_change_cloudlog_cadence_or_history():
  diagnostics, emitted = recorder()
  diagnostics.record('transport_open', transport='bluetooth')
  original = diagnostics.snapshot()['events']
  for _ in range(100):
    for kind in ('bridge_started', 'connect_attempt', 'connect_failed', 'transport_closed', 'retry_scheduled', 'periodic_status'):
      diagnostics.record(kind)
    diagnostics.record('transport_open', local_only=True, transport='usb')
  assert len(emitted) == 1
  assert diagnostics.snapshot()['events'] == original


def test_custom_exception_names_are_not_serialized():
  error = type('PRIVATE', (ValueError,), {})('PRIVATE')
  diagnostics, emitted = recorder()
  diagnostics.failure(phase='exchange', transport='bluetooth', error=error, connected_at=0, wireless=None)
  assert 'PRIVATE' not in json.dumps(emitted)


def test_journal_errors_are_visible_outside_the_failed_disk_sink():
  diagnostics, emitted = recorder()
  stats = {'queued': 0, 'written': 4, 'dropped': 6, 'storageErrors': 2, 'disabled': False}
  diagnostics.journal = NS(stats=lambda: stats)
  snapshot = diagnostics.snapshot()
  assert snapshot['journal'] == stats
  stats['storageErrors'] = 3
  assert snapshot['journal']['storageErrors'] == 2
  diagnostics.journal = NS(stats=lambda: (_ for _ in ()).throw(OSError('private')))
  assert diagnostics.snapshot()['journal'] == {}
  assert not emitted
