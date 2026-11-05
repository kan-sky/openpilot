import json
import threading
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bridge, journal
from openpilot.selfdrive.navdy.tests.test_bridge_reconnect import run_wireless_bridge
from openpilot.selfdrive.navdy.tests.test_journal import records


def broken(*_args, **_kwargs):
  raise OSError('PRIVATE logging failure')


def link_scenario(link, state):
  if link.attempt == 1 and state.now >= 102.6:
    state.freeze_sources = True
    raise ConnectionError('PRIVATE address')
  return state.reply()


def behavior(state):
  return (state.attempts, state.sent, state.delays, state.closes, state.feedback_writes,
          [(row.get('connected'), row.get('extendedFeedback'), row.get('transport')) for row in state.statuses])


@pytest.mark.parametrize('fault', ['sink', 'disk', 'full_queue'])
def test_diagnostic_faults_leave_wireless_timing_ttl_and_reconnect_unchanged(monkeypatch, tmp_path, fault):
  reference = tmp_path / 'reference'
  reference.mkdir()
  with monkeypatch.context() as patch:
    patch.setattr(bridge, 'LocalJournal', lambda: None)
    baseline = run_wireless_bridge(patch, reference, link_scenario, duration=12)

  target = tmp_path / fault
  target.mkdir()
  release = threading.Event()
  sink = None
  try:
    with monkeypatch.context() as patch:
      if fault == 'sink':
        sink = NS(submit=broken, stats=broken, close=broken)
      else:
        if fault == 'disk':
          patch.setattr(journal.JournalFiles, 'write', broken)
        else:
          patch.setattr(journal, 'QUEUE_SIZE', 1)
          patch.setattr(journal.JournalFiles, 'write', lambda *_: release.wait(3))
        sink = journal.LocalJournal(target.resolve() / 'journal')
      patch.setattr(bridge, 'LocalJournal', lambda: sink)
      observed = run_wireless_bridge(patch, target, link_scenario, duration=12)
      assert behavior(observed) == behavior(baseline)
      assert observed.delays == [.2]
      assert not observed.sent[-1][2]['active']  # Source TTL still expires normally.
      if fault == 'disk':
        assert sink.storage_errors >= 1
      if fault == 'full_queue':
        assert sink.dropped >= 1
  finally:
    release.set()
    if isinstance(sink, journal.LocalJournal):
      sink.close()
      sink.thread.join(2)
      assert not sink.thread.is_alive()


def test_shutdown_cleans_link_and_journal_even_when_status_write_fails(monkeypatch, tmp_path):
  captured = []
  real_write = bridge.atomic_json

  def write(path, value):
    if value.get('reason') == 'bridge_stopped':
      raise OSError('PRIVATE shutdown status failure')
    real_write(path, value)

  sink = NS(submit=lambda value: captured.append(value), stats=dict, close=lambda: captured.append('closed'))
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: sink)
  monkeypatch.setattr(bridge, 'atomic_json', write)
  with pytest.raises(OSError, match='shutdown status failure'):
    run_wireless_bridge(monkeypatch, tmp_path, lambda _link, state: state.reply())
  assert captured[-1] == 'closed'
  assert captured[-2]['kind'] == 'bridge_stopped'
  assert not (tmp_path / 'feedback.json').exists()


def test_startup_failure_still_stops_journal_and_resources(monkeypatch, tmp_path):
  from openpilot import cereal
  closed = []
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: None), raising=False)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=broken, close=lambda: closed.append('door')))
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(close=lambda: closed.append('probe')))
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: NS(submit=lambda _: None, close=lambda: closed.append('journal')))
  with pytest.raises(OSError):
    bridge.run(feedback=tmp_path / 'feedback.json', status=tmp_path / 'status.json', duration=1)
  assert closed == ['door', 'probe', 'journal']
  assert json.loads((tmp_path / 'status.json').read_text())['reason'] == 'bridge_stopped'


def test_real_bridge_journal_keeps_receiver_sequences_without_private_feedback(monkeypatch, tmp_path):
  sink = journal.LocalJournal(tmp_path.resolve() / 'journal')
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: sink)

  def exchange(_link, state):
    sequence = len(state.sent)
    return [({'wireless': {'owner': 'bluetooth', 'session': 1, 'receivedSeq': sequence, 'appliedSeq': sequence - 1,
                           'peer': 'PRIVATE'},
              'displayPower': {'state': 'onroad', 'fresh': True, 'generation': 1, 'invalidations': 0},
              'music': 'PRIVATE', 'location': 'PRIVATE'}, state.now, 10.)]

  run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=12)
  rows = records(sink.files.directory)
  statuses = [row for row in rows if row['kind'] == 'periodic_status']
  assert len(statuses) == 3
  assert [row['at'] for row in statuses] == [100., 105., 110.]
  assert statuses[-1]['receiver']['wireless']['receivedSeq'] == 50
  assert statuses[-1]['receiver']['wireless']['appliedSeq'] == 49
  assert 'PRIVATE' not in json.dumps(rows)
  assert rows[-1]['kind'] == 'bridge_stopped'
  assert not sink.thread.is_alive()


def test_heartbeat_size_supports_overnight_quiet_link_retention():
  # Full five-phase/context sample, with long numeric timestamps/counters. This
  # is a retention budget, not a guarantee during repeated lifecycle failures.
  sample = dict.fromkeys(('at', 'wallTime', 'connectedAt', 'receiverAt', 'sourceAt', 'eventSeq'), 123456.123456)
  sample.update(kind='periodic_status', runId='a' * 32, transport='bluetooth', connected=True, bluetoothOnly=True,
                retryInSeconds=0, exchanges=25, replies=25,
                timings={phase: {'count': 25, 'totalMs': 1234.123456, 'maxMs': 123.123456} for phase in journal.PHASES},
                sourceState={'power': 'onroad', 'ambient': 'onroad', 'missing': {}},
                receiver={'wireless': {'session': 99999, 'owner': 'bluetooth', 'clears': 99999, 'usbUpdates': 99999,
                                       'receivedSeq': 999999, 'appliedSeq': 999998},
                          'power': {'state': 'onroad', 'fresh': True, 'generation': 999999, 'invalidations': 99999}},
                wireless=dict.fromkeys(journal.METRICS, 123456.123456),
                journal={'queued': 1, 'written': 99999, 'dropped': 99999, 'storageErrors': 99999, 'disabled': False})
  size = len(json.dumps(journal.sanitize(sample), separators=(',', ':')).encode()) + 128
  print(f'heartbeat_budget_bytes={size} quiet_retention_hours={journal.FILE_BYTES * journal.FILE_COUNT / size * 5 / 3600:.2f}')
  assert size < 1800
  assert journal.FILE_BYTES * journal.FILE_COUNT / size * 5 > 12 * 3600
