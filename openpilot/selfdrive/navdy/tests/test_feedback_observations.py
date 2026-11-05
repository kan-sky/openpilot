import copy
import json
import socket
from types import SimpleNamespace as NS
import zlib

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy.bluetooth_pipeline import ResponseReader, SnapshotPipeline
from openpilot.selfdrive.navdy.feedback import feedback_shape, is_no_observation
from openpilot.selfdrive.navdy.navigation import FeedbackTracker, NavigationReader
from openpilot.selfdrive.navdy.tests.test_bridge_reconnect import run_wireless_bridge
from openpilot.selfdrive.navdy.tests.test_bridge import Messages
from openpilot.selfdrive.navdy.tests.test_model_navigation import packet as model_packet, write_cache, write_feedback
from openpilot.selfdrive.navdy.tests.test_navigation import packet


def discard():
  # Exact v167 WirelessOwner.discard contract, not a normal ACK with null guidance.
  return {'navigationFeedback': None, 'modelNavigation': None}


@pytest.fixture
def wire_feedback():
  """Real NVQ2/NVA2 sockets and parser, with deterministic ACK arrival time."""
  left, right = socket.socketpair()
  pipeline = None

  def reply(value, state, *, compressed, status=1):
    nonlocal pipeline
    captured_ns = round(state.now * 1e9)
    if pipeline is None:
      pipeline = SnapshotPipeline(left, captured_ns, reader_factory=lambda conn: ResponseReader(conn, threaded=False))
    payload = state.sent[-1][2]
    assert pipeline.update(payload, captured_ns=captured_ns) == []
    head = bt.REQUEST.unpack(bt.receive(right, bt.REQUEST.size, state.now + 1.))
    request = bt.receive(right, head[5], state.now + 1.)
    assert json.loads(bt.unpack_payload(request, head[4], head[7], head[6])) == payload
    raw = value if isinstance(value, bytes) else json.dumps(value, separators=(',', ':')).encode()
    encoded = zlib.compress(raw) if compressed else raw
    frame = bt.RESPONSE.pack(b'NVA2', head[1], status, head[1] + 1, 527900 if status == 1 else 100000,
                             len(raw), len(encoded), zlib.crc32(raw), int(compressed)) + encoded
    # Split the actual byte stream: only full ACK receipt may start its clock.
    right.sendall(frame[:7])
    pipeline.reader._receive_available()
    assert pipeline.reader.frames.empty()
    state.now = round(state.now + .04, 9)
    right.sendall(frame[7:])
    pipeline.reader._receive_available()
    received = state.now
    state.now = round(state.now + .02, 9)
    replies = pipeline._read()
    assert len(replies) == 1 and replies[0][1] == received and replies[0][2] == 40.
    assert not pipeline.pending
    assert pipeline.diagnostics()['displayStatus'] == status
    return replies

  try:
    yield reply
  finally:
    if pipeline is not None:
      pipeline.close()
    left.close()
    right.close()


def observe_file_reader(monkeypatch, tmp_path):
  reader = NavigationReader(tmp_path / 'feedback.json')
  samples = []
  original_write = bridge.atomic_json

  def write(path, value):
    original_write(path, value)
    if path.name == 'status.json' and value.get('connected'):
      now = value['updatedAt']
      samples.append(NS(now=now, navigation=reader.read(now), model=reader.model_read(now),
                        received=reader.tracker.received_at, model_received=reader.model_tracker.received,
                        model_deadline=reader.model_tracker.deadline, token=reader.model_route_token))

  monkeypatch.setattr(bridge, 'atomic_json', write)
  return reader, samples


@pytest.mark.parametrize('compressed', [False, True], ids=['plain', 'zlib'])
@pytest.mark.parametrize('normal_status', [0, 1, 3])
@pytest.mark.parametrize('clear', ['camera', 'guidance', 'both'])
def test_wire_discard_retains_observation_but_real_clear_reaches_reader(
    monkeypatch, tmp_path, wire_feedback, compressed, normal_status, clear):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  _, samples = observe_file_reader(monkeypatch, tmp_path)
  cleared = {**packet(2), 'modelNavigation': None}
  if clear in ('camera', 'both'):
    cleared.update(cameraSpeedKph=0, cameraDistance='')
  if clear in ('guidance', 'both'):
    cleared['navigationFeedback'] = None

  def exchange(_link, state):
    index = len(state.sent) - 1
    value = {**packet(), 'modelNavigation': model_packet()} if index == 0 else cleared if index == 3 else discard()
    raw = b'{"navigationFeedback":null,"modelNavigation":null}' if is_no_observation(value) else value
    replies = wire_feedback(raw, state, compressed=compressed, status=normal_status if index in (0, 3) else 1)
    assert is_no_observation(replies[0][0]) is (index not in (0, 3))
    if index not in (0, 3):
      assert replies[0][0] == discard()  # Parser/age_feedback have not added any keys.
    return replies

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=1.)
  assert not state.failures and len(samples) == 5
  assert [row['receivedAt'] for row in state.feedback_writes] == [100.04, 100.64]
  first = samples[0]
  assert first.navigation.speed.sdi_present and first.navigation.current.present and first.model is not None
  assert first.model_deadline == pytest.approx(101.9)  # Original source age and 40 ms RTT, not the poll time.
  for sample in samples[1:3]:
    assert sample.navigation == first.navigation and sample.model == first.model
    assert (sample.received, sample.model_received, sample.model_deadline, sample.token) == (
      first.received, first.model_received, first.model_deadline, first.token)
  for sample in samples[3:]:
    assert sample.received == 100.64
    assert sample.navigation.speed.sdi_present is (clear == 'guidance')
    assert sample.navigation.current.present is (clear == 'camera')
    assert sample.model is None
  counters = state.statuses[-2]['diagnostics']['feedback']
  assert counters['noObservation'] == {'count': 3, 'lastReceivedAt': 100.84}
  assert counters['observation']['count'] == 2 and counters['incomplete']['count'] == 0


@pytest.mark.parametrize('compressed', [False, True], ids=['plain', 'zlib'])
@pytest.mark.parametrize('order', ['normal-discard', 'discard-normal', 'discard-discard'])
def test_wire_batch_keeps_last_normal_and_never_writes_discard(monkeypatch, tmp_path, wire_feedback, compressed, order):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)

  def exchange(_link, state):
    replies = []
    for kind in order.split('-'):
      replies.extend(wire_feedback(discard() if kind == 'discard' else packet(), state, compressed=compressed))
    return replies

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.15)
  assert not state.failures
  if order == 'discard-discard':
    assert state.feedback_writes == []
  else:
    assert len(state.feedback_writes) == 1
    row = state.feedback_writes[0]
    assert row['receivedAt'] == (100.04 if order == 'normal-discard' else 100.1)
    assert row['feedback']['cameraSpeedKph'] == 50
    assert row['feedback']['navigationFeedback']['ageMs'] == 40.
    assert row['feedback']['navigationFeedback']['validForMs'] == 2460


@pytest.mark.parametrize('compressed', [False, True], ids=['plain', 'zlib'])
def test_wire_discard_cannot_extend_reader_or_bridge_model_deadline(monkeypatch, tmp_path, wire_feedback, compressed):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  _, samples = observe_file_reader(monkeypatch, tmp_path)
  tracker = bridge.ModelNavigationTracker()
  monkeypatch.setattr(bridge, 'ModelNavigationTracker', lambda: tracker)
  model_times = []

  def exchange(_link, state):
    if len(state.sent) > 1:
      model_times.append((tracker.received, tracker.deadline))
    value = {**packet(), 'modelNavigation': model_packet()} if len(state.sent) == 1 else discard()
    return wire_feedback(value, state, compressed=compressed)

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=3.)
  assert not state.failures and len(state.feedback_writes) == 1
  assert model_times and all(received == 100.04 and deadline == pytest.approx(101.9) for received, deadline in model_times)
  for sample in samples:
    assert sample.received == 100.04
    assert (sample.navigation is not None) is (sample.now < 102.04)
    assert (sample.model is not None) is (sample.now < 101.9)
    assert sample.model_received == 100.04 and sample.model_deadline == pytest.approx(101.9)
  assert tracker.current is None and tracker.value is None


@pytest.mark.parametrize('compressed', [False, True], ids=['plain', 'zlib'])
@pytest.mark.parametrize('invalid', [b'{"navigationFeedback":null}', b'{"navigationFeedback":null,"modelNavigation":{}}',
                                    b'{"navigationFeedback":null,"modelNavigation":null,"cameraDistance":[]}'])
def test_wire_incomplete_response_is_not_preserved_as_no_observation(monkeypatch, tmp_path, wire_feedback, compressed, invalid):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  _, samples = observe_file_reader(monkeypatch, tmp_path)

  def exchange(_link, state):
    value = {**packet(), 'modelNavigation': model_packet()} if len(state.sent) == 1 else invalid
    replies = wire_feedback(value, state, compressed=compressed)
    assert not is_no_observation(replies[0][0])
    return replies

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.4)
  assert not state.failures and len(state.feedback_writes) == 2
  assert samples[0].navigation.current.present and samples[0].model is not None
  assert samples[1].navigation is None and samples[1].model is None
  counters = state.statuses[-2]['diagnostics']['feedback']
  assert counters['noObservation']['count'] == 0 and counters['incomplete']['count'] == 1


@pytest.mark.parametrize('elapsed', [-1., float('nan'), float('inf'), 2000., 2001., 3000.])
@pytest.mark.parametrize('model', [model_packet(), {}], ids=['model', 'malformed-model'])
def test_aging_model_cannot_hide_real_null_guidance_from_file_reader(monkeypatch, tmp_path, elapsed, model):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  _, samples = observe_file_reader(monkeypatch, tmp_path)

  def exchange(_link, state):
    value = {**packet(), 'modelNavigation': model_packet()} if len(state.sent) == 1 else {
      'navigationFeedback': None, 'modelNavigation': copy.deepcopy(model)}
    return [(value, state.now, 0. if len(state.sent) == 1 else elapsed)]

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.4)
  assert not state.failures and len(state.feedback_writes) == 2
  cleared = state.feedback_writes[-1]['feedback']
  assert 'navigationFeedback' in cleared and cleared['navigationFeedback'] is None
  assert not is_no_observation(cleared)
  assert samples[0].navigation.current.present and samples[0].model is not None
  assert samples[1].navigation is None and samples[1].model is None and samples[1].received == 100.2
  counters = state.statuses[-2]['diagnostics']['feedback']
  assert counters['noObservation']['count'] == 0 and counters['incomplete']['count'] == 1


@pytest.mark.parametrize('elapsed', [-1., float('nan'), float('inf'), 2000., 2001., 3000.])
def test_exact_discard_with_bad_elapsed_never_ages_or_refreshes_model(monkeypatch, tmp_path, elapsed):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  _, samples = observe_file_reader(monkeypatch, tmp_path)
  tracker = bridge.ModelNavigationTracker()
  monkeypatch.setattr(bridge, 'ModelNavigationTracker', lambda: tracker)

  def exchange(_link, state):
    first = len(state.sent) == 1
    value = {**packet(), 'modelNavigation': model_packet()} if first else discard()
    return [(value, state.now, 0. if first else elapsed)]

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.4)
  assert not state.failures and len(state.feedback_writes) == 1
  assert tracker.received == 100. and tracker.deadline == pytest.approx(101.9)
  assert tracker.is_fresh(100.2)
  assert samples[1].navigation.speed == samples[0].navigation.speed
  assert samples[1].navigation.current == samples[0].navigation.current
  assert samples[1].navigation.traffic.remain_sec < samples[0].navigation.traffic.remain_sec
  assert samples[1].model == samples[0].model
  assert samples[1].received == samples[1].model_received == 100.
  assert samples[1].model_deadline == pytest.approx(101.9)
  counters = state.statuses[-2]['diagnostics']['feedback']
  assert counters['noObservation']['count'] == 1 and counters['observation']['count'] == 1


@pytest.mark.parametrize('value', [None, [], {}, {'navigationFeedback': None}, {'modelNavigation': None},
                                 {'navigationFeedback': 0, 'modelNavigation': None},
                                 {'navigationFeedback': None, 'modelNavigation': False},
                                 {'navigationFeedback': [], 'modelNavigation': {}},
                                 {**discard(), 'unknown': None}, {**discard(), 'roadPreview': None},
                                 {**discard(), 'cameraSource': None}, {**discard(), 'wireless': {}}, packet()])
def test_only_exact_two_null_contract_is_no_observation(value):
  assert not is_no_observation(value)
  assert feedback_shape(value) != 'noObservation'
  assert is_no_observation(discard())
  assert feedback_shape(discard()) == 'noObservation'


def test_discard_does_not_clear_camera_or_renew_observation():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10., 10.)
  before = tracker.snapshot(10.)
  components = copy.deepcopy(tracker.components)
  assert not tracker.accept(discard(), 10.2, 10.2)
  assert tracker.received_at == 10.
  assert tracker.components == components
  assert tracker.snapshot(10.2).speed == before.speed
  assert tracker.snapshot(10.2).current.present
  tracker.accept(packet(), 10.4, 10.4)
  assert tracker.snapshot(10.4).speed == before.speed


def test_continuous_discard_cannot_extend_link_ttl():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10., 10.)
  for offset in range(1, 31):
    now = 10. + offset / 10.
    assert not tracker.accept(discard(), now, now)
    assert (tracker.snapshot(now) is not None) is (now < 12.)
    assert tracker.received_at == 10.


def test_discard_preserves_short_producer_deadline():
  tracker = FeedbackTracker()
  value = packet()
  value['navigationFeedback']['validForMs'] = 100
  tracker.accept(value, 10., 10.)
  tracker.accept(discard(), 10.05, 10.05)
  assert tracker.snapshot(10.05).current.present
  tracker.accept(discard(), 10.1, 10.1)
  state = tracker.snapshot(10.1)
  assert state.speed.sdi_present
  assert not state.current.present and not state.traffic.present


def test_discard_cannot_extend_four_second_source_ttl():
  tracker = FeedbackTracker()
  value = packet()
  value['navigationFeedback']['validForMs'] = 4000
  tracker.accept(value, 10., 10.)
  tracker.accept(value, 13., 13.)  # Same producer sequence must not renew it.
  tracker.accept(discard(), 13.9, 13.9)
  assert tracker.snapshot(13.9).current.present
  tracker.accept(discard(), 14., 14.)
  state = tracker.snapshot(14.)
  assert state.speed.sdi_present
  assert state.speed.road_limit_kph is None and not state.current.present


@pytest.mark.parametrize('clear', ['camera', 'guidance', 'both'])
def test_actual_observation_clears_immediately(clear):
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10., 10.)
  value = packet(sequence=2)
  value['modelNavigation'] = None
  if clear in ('camera', 'both'):
    value.update(cameraSpeedKph=0, cameraDistance='')
  if clear in ('guidance', 'both'):
    value['navigationFeedback'] = None
  assert tracker.accept(value, 10.2, 10.2)
  state = tracker.snapshot(10.2)
  assert state.speed.sdi_present is (clear == 'guidance')
  assert state.current.present is (clear == 'camera')


@pytest.mark.parametrize('value', [
  {}, {'navigationFeedback': None}, {'modelNavigation': None},
  {'navigationFeedback': False, 'modelNavigation': None},
  {'navigationFeedback': None, 'modelNavigation': {}},
  {'navigationFeedback': None, 'modelNavigation': None, 'roadPreview': None},
  {'navigationFeedback': None, 'modelNavigation': None, 'cameraSpeedKph': 0},
  {'navigationFeedback': None, 'modelNavigation': None, 'wireless': {}},
])
def test_incomplete_or_clear_is_not_silently_held_as_discard(value):
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10., 10.)
  assert tracker.accept(value, 10.2, 10.2)
  assert tracker.camera is None
  assert tracker.received_at == 10.2
  # Existing malformed-component validation is unchanged; this reply must not
  # preserve the previous camera merely because another field is incomplete.
  if value.get('navigationFeedback') is None:
    assert tracker.snapshot(10.2) is None


@pytest.mark.parametrize('order', ['normal-discard', 'discard-normal', 'discard-discard', 'normal-clear-discard'])
def test_bridge_batch_preserves_last_observation_and_original_receipt(monkeypatch, tmp_path, order):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  value = packet()
  normal = (value, 99.96, 0.)
  none = (discard(), 99.99, 10.)
  clear = ({**value, 'cameraSpeedKph': 0, 'cameraDistance': '', 'navigationFeedback': None}, 99.98, 0.)
  replies = {'normal-discard': [normal, none], 'discard-normal': [none, normal],
             'discard-discard': [none, none], 'normal-clear-discard': [normal, clear, none]}[order]
  state = run_wireless_bridge(monkeypatch, tmp_path, lambda _link, _state: copy.deepcopy(replies), duration=.1)
  assert not state.failures
  if order == 'discard-discard':
    assert state.feedback_writes == []
  else:
    assert len(state.feedback_writes) == 1
    expected = clear if order == 'normal-clear-discard' else normal
    assert state.feedback_writes[0]['receivedAt'] == expected[1]
    assert state.feedback_writes[0]['feedback'] == expected[0]


def test_bridge_incomplete_last_reply_is_not_replaced_by_prior_good_data(monkeypatch, tmp_path):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  value = {'navigationFeedback': None, 'modelNavigation': None, 'cameraDistance': []}
  replies = [(packet(), 99.98, 0.), (value, 99.99, 0.), (discard(), 100., 0.)]
  state = run_wireless_bridge(monkeypatch, tmp_path, lambda _link, _state: copy.deepcopy(replies), duration=.1)
  assert state.feedback_writes[0]['feedback'] == value
  assert state.feedback_writes[0]['receivedAt'] == 99.99


@pytest.mark.parametrize('remove', ['cameraSpeedKph', 'cameraSource', 'cameraType', 'cameraDistance'])
def test_missing_camera_key_is_not_no_observation(remove):
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10., 10.)
  value = {**packet(2), 'modelNavigation': None}
  value.pop(remove)
  assert not is_no_observation(value)
  assert tracker.accept(value, 10.1, 10.1)
  assert not tracker.snapshot(10.1).speed.sdi_present


@pytest.mark.parametrize('elapsed', [-1., float('nan'), 2001.])
def test_bad_model_timing_cannot_turn_legacy_clear_into_discard(monkeypatch, tmp_path, elapsed):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  replies = [(packet(), 99.98, 0.), ({'navigationFeedback': None}, 99.99, elapsed)]
  state = run_wireless_bridge(monkeypatch, tmp_path, lambda _link, _state: copy.deepcopy(replies), duration=.1)
  assert state.feedback_writes[0]['feedback'] == {'navigationFeedback': None}
  assert not is_no_observation(state.feedback_writes[0]['feedback'])


@pytest.mark.parametrize('stamp', [99.8, 100., float('nan'), float('inf'), 102., True])
def test_bridge_cannot_republish_old_or_invalid_receipt(monkeypatch, tmp_path, stamp):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  def exchange(_link, state):
    return [(packet(), 100., 0.)] if state.now == 100. else [(packet(2), stamp, 0.)]
  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.4)
  assert len(state.feedback_writes) == 1
  assert state.feedback_writes[0]['receivedAt'] == 100.


@pytest.mark.parametrize('keys', [
  ('cameraSource', 'cameraType', 'cameraSpeedKph', 'cameraDistance'),
  ('cameraSource', 'cameraType', 'cameraSpeedKph', 'cameraDistance', 'roadPreview'),
  ('cameraSource', 'cameraType', 'cameraSpeedKph', 'cameraDistance', 'roadPreview', 'navigationFeedback'),
])
def test_legacy_observations_preserve_fields_and_camera(monkeypatch, tmp_path, keys):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  original = packet()
  value = {key: original[key] for key in keys}
  state = run_wireless_bridge(monkeypatch, tmp_path, lambda _link, _state: [(copy.deepcopy(value), 100., 0.)], duration=.1)
  assert state.feedback_writes[0]['feedback'] == value
  tracker = FeedbackTracker()
  tracker.accept(state.feedback_writes[0]['feedback'], 100., 100.)
  assert tracker.snapshot(100.).speed.sdi_speed_limit_kph == 50
  assert tracker.extended is ('navigationFeedback' in keys)


@pytest.mark.parametrize('transport_name', ['usb', 'jetson'])
def test_wired_and_relay_normal_clear_responses_are_unchanged(monkeypatch, tmp_path, transport_name):
  from openpilot import cereal
  clock = NS(now=100.)
  sm = Messages()
  sm.update = lambda _: None
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'sleep', lambda delay: setattr(clock, 'now', round(clock.now + delay, 9)))
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=lambda: None, close=lambda: None, door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda _now: None, close=lambda: None))
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: (None, False))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: ['192.0.2.1'])
  replies = [packet(), {**packet(), 'cameraSpeedKph': 0, 'cameraDistance': '', 'navigationFeedback': None},
             {'navigationFeedback': None}]
  writes = []
  feedback = tmp_path / 'feedback.json'
  original_write = bridge.atomic_json
  def write(path, value):
    if path == feedback:
      writes.append(value)
    original_write(path, value)
  monkeypatch.setattr(bridge, 'atomic_json', write)

  class Transport:
    def connect(self):
      pass
    def close(self):
      pass
    def exchange(self, payload):
      return copy.deepcopy(replies[len(writes)])

  class MissingUsb(Transport):
    def connect(self):
      raise ConnectionError('No cable')

  monkeypatch.setattr(bridge, 'AdbTransport', Transport if transport_name == 'usb' else MissingUsb)
  monkeypatch.setattr(bridge, 'SshTransport', lambda _: Transport())
  bridge.run(feedback=feedback, status=tmp_path / 'status.json', duration=.5)
  assert [row['feedback'] for row in writes] == replies
  assert [row['receivedAt'] for row in writes] == [100., 100.2, 100.4]


def test_status_one_with_observation_must_not_be_skipped(monkeypatch, tmp_path):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  def exchange(link, state):
    link.last_exchange = {'displayStatus': 1, 'receiverAgeMs': 527.9}
    return [(packet(), state.now, 0.)]
  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=.1)
  assert len(state.feedback_writes) == 1
  assert state.feedback_writes[0]['feedback']['cameraSpeedKph'] == 50


def test_bridge_discards_do_not_reset_or_freshen_model(monkeypatch, tmp_path):
  monkeypatch.setattr(bridge, 'LocalJournal', lambda: None)
  tracker = bridge.ModelNavigationTracker()
  monkeypatch.setattr(bridge, 'ModelNavigationTracker', lambda: tracker)
  observations = []
  original_accept = tracker.accept
  def accept(value, received, now):
    observations.append((value, received))
    original_accept(value, received, now)
  monkeypatch.setattr(tracker, 'accept', accept)

  def exchange(_link, state):
    first = state.now == 100.
    if not first:
      assert tracker.received == 99.96
      assert tracker.deadline == pytest.approx(101.86)
      assert tracker.is_fresh(state.now) is (state.now < 101.86)
    return ([({**packet(), 'modelNavigation': model_packet()}, 99.96, 0.)] if first else []) + [
      (discard(), state.now, 0.)]

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=3.)
  assert len(state.feedback_writes) == len(observations) == 1
  assert state.feedback_writes[0]['modelRouteCacheToken'] is not None
  assert all(row['connected'] for row in state.statuses[:-1])
  assert tracker.current is None  # Original model deadline still expires.


def test_reader_discard_preserves_receipts_and_model_cache_until_deadline(tmp_path):
  path = tmp_path / 'feedback.json'
  write_cache(tmp_path / 'feedback-model-route.json')
  write_feedback(path, model_packet(), 10.)
  reader = NavigationReader(path)
  assert reader.model_read(10.) is not None
  original = (reader.model_tracker.received, reader.model_tracker.deadline, reader.model_route_token)
  for now in (10.2, 10.4, 11.8, 11.91):
    path.write_text(json.dumps({'receivedAt': now, 'feedback': discard(), 'modelRouteCacheToken': None}))
    assert (reader.model_read(now) is not None) is (now < 11.9)
    assert (reader.model_tracker.received, reader.model_tracker.deadline, reader.model_route_token) == original


def test_reader_real_null_model_is_not_discard(tmp_path):
  path = tmp_path / 'feedback.json'
  write_cache(tmp_path / 'feedback-model-route.json')
  write_feedback(path, model_packet(), 10.)
  reader = NavigationReader(path)
  assert reader.model_read(10.) is not None
  # Match the real writer's inode replacement even on coarse-timestamp filesystems.
  bridge.atomic_json(path, {'receivedAt': 10.2, 'feedback': {**packet(), 'modelNavigation': None}})
  assert reader.model_read(10.2) is None
  assert reader.model_tracker.received == 10.2
