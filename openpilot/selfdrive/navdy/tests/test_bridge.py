import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.tests.test_display import car, model, selfdrive


class Messages:
  def __init__(self):
    self.seen = dict.fromkeys(bridge.SERVICES, True)
    self.alive = dict.fromkeys(bridge.SERVICES, True)
    self.valid = dict.fromkeys(bridge.SERVICES, True)
    self.recv_time = dict.fromkeys(bridge.SERVICES, 100.0)
    self.logMonoTime = dict.fromkeys(bridge.SERVICES, 100_000_000_000)
    self.data = dict.fromkeys(bridge.SERVICES, None)
    self.data.update(deviceState=NS(started=True), carState=car(), selfdriveState=selfdrive(), modelV2=model(),
                     pandaStates=[], carControl=NS(latActive=True, longActive=True), carOutput=NS(actuatorsOutput=NS(accel=-0.5)))

  def __getitem__(self, service):
    return self.data[service]

  def delay(self, service, seconds):
    self.recv_time[service] = 100.0 - seconds
    self.logMonoTime[service] = round((100.0 - seconds) * 1e9)
    self.alive[service] = False


@pytest.fixture
def sm(monkeypatch):
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: 100.0)
  return Messages()


@pytest.fixture(autouse=True)
def no_hardware_door_thread(monkeypatch):
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=lambda: None, close=lambda: None,
                                                     door_open=False, gm=True, error=''))


def test_offroad_without_car_messages_keeps_ambient_heartbeat(sm):
  sm.data['deviceState'].started = False
  for service in ('carState', 'selfdriveState'):
    sm.delay(service, 10)
  payload = bridge.live_payload(sm, 1, door_open=True)
  assert payload['ambientState'] == 'offroad_door'
  assert payload['displayPower'] == 'offroad'
  assert not payload['active'] and payload['vEgoKph'] == 0
  assert payload['doorOpen']
  assert bridge.live_payload(sm, 2, door_open=False)['ambientState'] == 'offroad'


@pytest.mark.parametrize('condition,expected', [('offroad', True), ('onroad', True), ('ignition', True),
                                              ('stale_device', False), ('stale_panda', False), ('invalid_panda', False),
                                              ('empty_panda', False), ('unknown_panda', False)])
def test_usb_recovery_allows_onroad_but_requires_fresh_known_state(sm, condition, expected):
  sm.update = lambda _: None
  sm.data['deviceState'].started = condition == 'onroad'
  sm.data['pandaStates'] = [NS(pandaType='tres', ignitionLine=False, ignitionCan=False)]
  if condition == 'ignition':
    sm.data['pandaStates'][0].ignitionLine = True
  elif condition == 'empty_panda':
    sm.data['pandaStates'] = []
  elif condition == 'unknown_panda':
    sm.data['pandaStates'][0].pandaType = 'unknown'
  elif condition == 'stale_device':
    sm.delay('deviceState', 2)
  elif condition == 'stale_panda':
    sm.delay('pandaStates', 2)
  elif condition == 'invalid_panda':
    sm.valid['pandaStates'] = False
  assert bridge.usb_recovery_allowed(sm) is expected


@pytest.mark.parametrize('service', ['deviceState', 'pandaStates'])
def test_unknown_power_cannot_request_sleep(sm, service):
  sm.data['deviceState'].started = False
  sm.delay(service, 2)
  payload = bridge.live_payload(sm, 1)
  assert payload['displayPower'] == 'unknown'
  assert payload['ambientState'] == 'disconnected'


def test_ignition_before_onroad_wakes_without_fabricating_active_telemetry(sm):
  sm.data['deviceState'].started = False
  sm.data['pandaStates'] = [NS(ignitionLine=True, ignitionCan=False)]
  payload = bridge.live_payload(sm, 1)
  assert payload['displayPower'] == 'onroad' and payload['onroad']
  assert not payload['active'] and payload['vEgoKph'] == 0


def test_ambient_override_is_carried_without_changing_display_state(sm):
  command = {'mode': 'profile', 'id': 'test'}
  payload = bridge.live_payload(sm, 1, ambient_override=command)
  assert payload['ambientOverride'] == command
  assert payload['active']


@pytest.mark.parametrize('service', ['carState', 'selfdriveState'])
def test_single_delayed_frame_does_not_clear_entire_display(sm, service):
  expected = bridge.live_payload(sm, 1)
  sm.delay(service, 0.15)
  assert bridge.live_payload(sm, 1) == expected


def test_zero_crossing_preserves_gear_but_not_stale_or_invalid_data(sm):
  sm.data['carState'].vEgo = -3.39526e-22
  sm.data['carState'].standstill = True
  result = bridge.live_payload(sm, 1)
  assert result['gear'] == 'drive'
  assert result['active'] and result['ambientState'] == 'onroad'
  sm.data['carState'].gearShifter = 'reverse'
  result = bridge.live_payload(sm, 2)
  assert result['gear'] == 'reverse' and result['ambientState'] == 'reverse'
  assert not result['active']
  sm.delay('carState', 0.6)
  result = bridge.live_payload(sm, 3)
  assert result['gear'] == 'unknown' and not result['active']
  sm.recv_time['carState'] = 100.0
  sm.logMonoTime['carState'] = 100_000_000_000
  sm.data['carState'].canValid = False
  result = bridge.live_payload(sm, 4)
  assert result['gear'] == 'unknown' and not result['active']


@pytest.mark.parametrize('age', [0.2, 0.4, 0.499])
def test_display_grace_is_bounded_by_receipt_age(sm, age):
  sm.delay('selfdriveState', age)
  assert bridge.live_payload(sm, 1)['active']


@pytest.mark.parametrize('service', ['carState', 'selfdriveState'])
@pytest.mark.parametrize('age', [0.5, 0.6, 1.0, 5.0])
def test_expired_critical_service_clears_display(sm, service, age):
  sm.delay(service, age)
  result = bridge.live_payload(sm, 1)
  assert not result['active']
  assert result['vEgoKph'] == 0
  assert result['setSpeedKph'] == 0
  assert result['navVehicles'] == []
  assert 'navPathLeft' not in result


@pytest.mark.parametrize('service', ['deviceState', 'carState', 'selfdriveState'])
def test_explicit_invalid_or_unseen_input_is_not_held(sm, service):
  sm.valid[service] = False
  assert not bridge.live_payload(sm, 1)['active']
  sm.valid[service] = True
  sm.seen[service] = False
  assert not bridge.live_payload(sm, 1)['active']


@pytest.mark.parametrize('update', ['disengage', 'offroad', 'reverse', 'canInvalid', 'canTimeout'])
def test_explicit_state_changes_take_effect_immediately(sm, update):
  assert bridge.live_payload(sm, 1)['active']
  if update == 'disengage':
    sm.data['selfdriveState'] = selfdrive(enabled=False, active=False)
  elif update == 'offroad':
    sm.data['deviceState'].started = False
  elif update == 'reverse':
    sm.data['carState'].gearShifter = 'reverse'
  elif update == 'canInvalid':
    sm.data['carState'].canValid = False
  else:
    sm.data['carState'].canTimeout = True
  assert not bridge.live_payload(sm, 2)['active']


def test_new_alert_is_not_debounced(sm):
  bridge.live_payload(sm, 1)
  sm.data['selfdriveState'] = selfdrive(active=False, enabled=False, alertText1='Take control',
                                      alertText2='Immediately', alertStatus='critical', alertSize='full')
  result = bridge.live_payload(sm, 2)
  assert not result['active']
  assert result['alertText1'] == 'Take control'
  assert result['alertText2'] == 'Immediately'
  assert result['alertStatus'] == 'critical'


@pytest.mark.parametrize('age,active', [(0.5, True), (1.0, True), (1.499, True), (1.5, False)])
def test_two_hz_device_state_has_its_own_deadline(sm, age, active):
  sm.delay('deviceState', age)
  assert bridge.live_payload(sm, 1)['active'] is active


@pytest.mark.parametrize('service', ['carState', 'selfdriveState'])
def test_delayed_queued_message_cannot_refresh_display(sm, service):
  sm.logMonoTime[service] = 99_000_000_000
  assert not bridge.live_payload(sm, 1)['active']


def test_stale_receipt_is_not_refreshed_by_publication_timestamp(sm):
  sm.recv_time['selfdriveState'] = 99.0
  assert not bridge.live_payload(sm, 1)['active']


@pytest.mark.parametrize('timestamp', [0, 101_000_000_000])
def test_missing_or_future_publication_time_is_not_fresh(sm, timestamp):
  sm.logMonoTime['selfdriveState'] = timestamp
  assert not bridge.live_payload(sm, 1)['active']


def test_stale_geometry_is_explicitly_cleared_at_hud(sm):
  assert bridge.live_payload(sm, 1)['navPathLeft']
  sm.delay('modelV2', 0.6)
  result = bridge.live_payload(sm, 2)
  assert result['active']
  assert result['navPathLeft'] == []
  assert result['navPathRight'] == []
  assert result['navVehicles'] == []


def test_recovery_requires_new_fresh_data(sm):
  sm.delay('selfdriveState', 0.6)
  assert not bridge.live_payload(sm, 1)['active']
  sm.alive['selfdriveState'] = True
  assert not bridge.live_payload(sm, 2)['active']
  sm.recv_time['selfdriveState'] = 100.0
  sm.logMonoTime['selfdriveState'] = 100_000_000_000
  assert bridge.live_payload(sm, 3)['active']


@pytest.mark.parametrize('received', [101.0, float('nan'), float('inf')])
def test_bad_receipt_time_is_rejected_and_diagnostics_are_json_safe(sm, received):
  sm.recv_time['selfdriveState'] = received
  sources = bridge.display_sources(sm)
  assert sources['selfdriveState']['reason'] == 'invalid_timestamp'
  json.dumps(sources, allow_nan=False)
  assert not bridge.live_payload(sm, 1, sources=sources)['active']


def run_fake_bridge(monkeypatch, tmp_path, *, connect_delay=0, invalidate=False, engaged=True, disconnect=False):
  from openpilot import cereal

  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  clock = NS(now=100.0)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))
  sm = Messages()
  sm.data['selfdriveState'] = selfdrive(active=engaged, enabled=engaged)

  def update(timeout):
    sm.recv_time = dict.fromkeys(bridge.SERVICES, clock.now)
    sm.logMonoTime = dict.fromkeys(bridge.SERVICES, int(clock.now * 1e9))
    sm.valid['selfdriveState'] = not (invalidate and 0.19 < clock.now - 100 < 0.39)

  sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda services: sm), raising=False)
  sent = []
  statuses = []
  connections = []

  class Transport:
    def connect(self):
      connections.append(clock.now)
      clock.now += connect_delay

    def exchange(self, payload):
      sent.append(payload)
      if disconnect and len(sent) == 1:
        raise ConnectionError('Navdy Bluetooth disconnected')
      return {'navigationFeedback': None}

    def close(self):
      pass

  monkeypatch.setattr(bridge, 'AdbTransport', Transport)
  atomic_json = bridge.atomic_json

  def capture(path, value):
    if path.name == 'status.json' and value.get('connected'):
      statuses.append(value)
    atomic_json(path, value)

  monkeypatch.setattr(bridge, 'atomic_json', capture)
  bridge.run(feedback=tmp_path / 'feedback.json', status=tmp_path / 'status.json',
             bluetooth_config=tmp_path / 'unconfigured-bluetooth.json',
             duration=2.6 if disconnect else 0.6)
  assert json.loads((tmp_path / 'status.json').read_text())['reason'] == 'bridge_stopped'
  assert not (tmp_path / 'feedback.json').exists()
  return sent, statuses, connections


def test_link_failure_remains_visible_after_success_and_shutdown(monkeypatch, tmp_path):
  _, statuses, _ = run_fake_bridge(monkeypatch, tmp_path, disconnect=True)
  assert statuses
  for status in statuses:
    failure = status['diagnostics']['lastFailure']
    assert failure['phase'] == 'exchange'
    assert failure['reason'] == 'Navdy Bluetooth disconnected'
    assert status['connected'] and status['reason'] == ''
  stopped = json.loads((tmp_path / 'status.json').read_text())
  assert stopped['diagnostics']['lastFailure']['reason'] == 'Navdy Bluetooth disconnected'


def test_model_route_cache_precedes_feedback_and_rotates_after_clear(monkeypatch, tmp_path):
  from openpilot import cereal
  from openpilot.selfdrive.navdy.tests.test_model_navigation import packet

  clock = NS(now=100.0)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: int(clock.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  sm = Messages()
  def update(_):
    sm.recv_time = dict.fromkeys(bridge.SERVICES, clock.now)
    sm.logMonoTime = dict.fromkeys(bridge.SERVICES, int(clock.now * 1e9))
  sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)

  first = packet()
  heartbeat = packet(sequence=2)
  heartbeat.pop('routeChunk')
  clear = dict(packet(sequence=3), active=False)
  replacement = packet([(38., 128.), (38.001, 128.001)], sequence=4)
  responses = [first, heartbeat, clear, replacement]
  exchanges = []
  class Transport:
    def connect(self):
      pass
    def exchange(self, payload):
      exchanges.append(payload)
      assert len(exchanges) <= len(responses)
      return {'modelNavigation': responses[len(exchanges) - 1]}
    def close(self):
      pass
  monkeypatch.setattr(bridge, 'AdbTransport', Transport)

  feedback = tmp_path / 'feedback.json'
  sidecar = tmp_path / 'feedback-model-route.json'
  events, caches = [], []
  original = bridge.atomic_json
  def capture(path, value):
    if path == feedback:
      token = value['modelRouteCacheToken']
      # Check the committed sidecar before the matching feedback is published.
      if token is None:
        assert not sidecar.exists()
      else:
        cached = json.loads(sidecar.read_text())
        assert cached['bridgeCacheToken'] == token
        caches.append(cached)
      events.append(('feedback', token))
    original(path, value)
    if path == sidecar:
      events.append(('sidecar', value['bridgeCacheToken']))
  monkeypatch.setattr(bridge, 'atomic_json', capture)
  bridge.run(feedback=feedback, status=tmp_path / 'status.json',
             bluetooth_config=tmp_path / 'unconfigured-bluetooth.json', duration=0.7)

  assert len(exchanges) == 4
  first_token, next_token = caches[0]['bridgeCacheToken'], caches[-1]['bridgeCacheToken']
  assert first_token != next_token
  assert events == [('sidecar', first_token), ('feedback', first_token), ('feedback', first_token),
                    ('feedback', None), ('sidecar', next_token), ('feedback', next_token)]
  assert caches[0] == caches[1]
  assert caches[0]['sha256'] == first['routeChunk']['sha256']
  assert caches[-1]['sha256'] == replacement['routeChunk']['sha256'] != caches[0]['sha256']
  assert caches[-1]['polyline6'] == replacement['routeChunk']['data']
  assert not sidecar.exists() and not feedback.exists()


def test_messages_are_refreshed_after_slow_connect(monkeypatch, tmp_path):
  sent, statuses, _ = run_fake_bridge(monkeypatch, tmp_path, connect_delay=2)
  assert len(sent) == 1
  assert sent[0]['active']
  assert statuses[0]['display']['sources']['selfdriveState']['ageSeconds'] == 0


def test_dropouts_are_diagnosed_separately_from_usb_connection(monkeypatch, tmp_path):
  sent, statuses, _ = run_fake_bridge(monkeypatch, tmp_path, invalidate=True)
  assert [p['active'] for p in sent] == [True, False, True]
  assert all(s['connected'] for s in statuses)
  assert statuses[0]['lastDisplayDropout'] is None
  assert statuses[1]['display']['sources']['selfdriveState']['reason'] == 'invalid'
  assert statuses[-1]['lastDisplayDropout']['sources'] == {'selfdriveState': 'invalid'}


def test_disengaged_telemetry_dropouts_are_also_recorded(monkeypatch, tmp_path):
  sent, statuses, _ = run_fake_bridge(monkeypatch, tmp_path, invalidate=True, engaged=False)
  assert not any(p['active'] for p in sent)
  assert [p['vEgoKph'] for p in sent] == [75.6, 0, 75.6]
  assert statuses[-1]['lastDisplayDropout']['sources'] == {'selfdriveState': 'invalid'}


def test_reconnect_uses_new_messages(monkeypatch, tmp_path):
  sent, statuses, connections = run_fake_bridge(monkeypatch, tmp_path, disconnect=True)
  assert len(connections) == 2
  assert sent[-1]['active']
  assert statuses[-1]['display']['sources']['selfdriveState']['ageSeconds'] == 0


@pytest.mark.parametrize('hosts,wireless', [([], True), (['192.0.2.1', '192.0.2.2'], True),
                                         (['192.0.2.1'], False)])
def test_usb_disconnect_falls_back_and_reattach_takes_priority(monkeypatch, tmp_path, hosts, wireless):
  from openpilot import cereal
  clock = NS(now=100.0)
  sm = Messages()
  sent, statuses = [], []
  current = NS(writer=None, bluetooth_closed=False)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: int(clock.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))

  def update(_):
    sm.recv_time = dict.fromkeys(bridge.SERVICES, clock.now)
    sm.logMonoTime = dict.fromkeys(bridge.SERVICES, int(clock.now*1e9))
  sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: ('54:ED:A3:00:48:23' if wireless else None, False))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: hosts)
  monkeypatch.setattr(bridge, 'SshTransport', lambda _: pytest.fail('Wireless handover tried a stale Jetson relay'))

  def usb_missing():
    return 100.1 < clock.now < 103.2 or 103.5 < clock.now < 106.6

  class Usb:
    def connect(self):
      if usb_missing():
        raise ConnectionError('Cable removed')
    def exchange(self, payload):
      if usb_missing():
        raise ConnectionError('Cable removed')
      if clock.now >= 103.2:
        assert current.bluetooth_closed
      assert current.writer in (None, self)
      current.writer = self
      sent.append(('usb', clock.now, payload['seq']))
      return {'navigationFeedback': None}
    def close(self):
      if current.writer is self:current.writer = None

  class Bluetooth:
    def __init__(self, peer):self.peer = peer
    def connect(self):
      current.bluetooth_closed = False
      clock.now += .3
    def exchange(self, payload, *, captured_ns=None):
      assert current.writer in (None, self)
      if wireless:
        assert captured_ns == int(clock.now * 1e9)
      current.writer = self
      sent.append(('bluetooth' if wireless else 'jetson', clock.now, payload['seq']))
      return {'navigationFeedback': None}
    def exchange_batch(self, payload, *, captured_ns):
      return [(self.exchange(payload, captured_ns=captured_ns), clock.now, 0.)]
    def close(self):
      current.bluetooth_closed = True
      if current.writer is self:current.writer = None

  class Probe:
    def __init__(self, factory):self.factory = factory; self.passed = False
    def poll(self, now):
      if usb_missing() or self.passed:return None
      self.passed = True
      candidate = self.factory()
      candidate.connect()
      return candidate
    def close(self):pass

  monkeypatch.setattr(bridge, 'AdbTransport', Usb)
  monkeypatch.setattr(bridge, 'BluetoothTransport', Bluetooth)
  if not wireless:
    monkeypatch.setattr(bridge, 'SshTransport', Bluetooth)
  monkeypatch.setattr(bridge, 'UsbProbe', Probe)
  original = bridge.atomic_json
  def capture(path, value):
    if path.name == 'status.json':statuses.append(value)
    original(path, value)
  monkeypatch.setattr(bridge, 'atomic_json', capture)
  bridge.run(feedback=tmp_path/'feedback.json', status=tmp_path/'status.json', duration=7)
  transports = [name for name, _, _ in sent]
  assert transports[0] == transports[-1] == 'usb'
  fallback = 'bluetooth' if wireless else 'jetson'
  assert fallback in transports
  transitions = [name for i, name in enumerate(transports) if i == 0 or name != transports[i-1]]
  assert transitions == ['usb', fallback, 'usb', fallback, 'usb']
  assert all(s['display']['sources']['selfdriveState']['ageSeconds'] == 0 for s in statuses if s.get('connected'))
  assert any(s.get('lastHandover', {}).get('to') == 'usb' for s in statuses if s.get('lastHandover'))
  assert current.writer is None and not (tmp_path/'feedback.json').exists()


@pytest.mark.parametrize('peer', [None, '54:ED:A3:00:48:23'])
def test_jetson_remains_available_without_working_bluetooth(monkeypatch, tmp_path, peer):
  from openpilot import cereal
  clock = NS(now=100.0)
  sm = Messages()
  sm.update = lambda _: None
  attempted = []
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: (peer, False))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: ['192.0.2.1'])

  class MissingUsb:
    def connect(self):
      raise ConnectionError('No USB')
    def close(self):
      pass

  class MissingBluetooth(MissingUsb):
    def __init__(self, target):
      assert target == peer
    def connect(self):
      attempted.append('bluetooth')
      raise ConnectionError('Wireless unavailable')

  class Jetson:
    def __init__(self, host):
      assert host == '192.0.2.1'
      attempted.append('jetson')
    def connect(self):
      pass
    def exchange(self, payload):
      return {'navigationFeedback': None}
    def close(self):
      pass

  monkeypatch.setattr(bridge, 'AdbTransport', MissingUsb)
  monkeypatch.setattr(bridge, 'BluetoothTransport', MissingBluetooth)
  monkeypatch.setattr(bridge, 'SshTransport', Jetson)
  bridge.run(feedback=tmp_path/'feedback.json', status=tmp_path/'status.json', duration=3)
  assert attempted == (['bluetooth', 'jetson'] if peer else ['jetson'])


def test_jetson_connection_bootstraps_only_inactive_state():
  transport = bridge.SshTransport.__new__(bridge.SshTransport)
  sent = []
  transport.exchange = lambda payload: sent.append(payload)
  transport.connect()
  assert len(sent) == 1
  assert sent[0]['schema'] == 'navdy.openpilot.v1'
  assert not sent[0]['active'] and not sent[0]['onroad']
  assert 'displayPower' not in sent[0]
  assert sent[0]['vEgoKph'] == 0 and sent[0]['navVehicles'] == []


@pytest.mark.parametrize('reconnect_fails', [False, True])
def test_healthy_bluetooth_retries_itself_before_unavailable_jetson(monkeypatch, tmp_path, reconnect_fails):
  from openpilot import cereal
  clock = NS(now=100.0)
  sm = Messages()
  attempts = []
  state = NS(failed=False)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: int(clock.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))
  sm.update = lambda _: None
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: ('54:ED:A3:00:48:23', False))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: ['192.0.2.1'])
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda now: None, close=lambda: None))

  class MissingUsb:
    def connect(self):
      raise ConnectionError('No USB')
    def close(self):
      pass

  class Wireless:
    def __init__(self, peer):
      self.peer = peer
    def connect(self):
      attempts.append('bluetooth')
      if reconnect_fails and len(attempts) == 2:
        raise ConnectionError('Radio unavailable during retry')
    def exchange(self, payload, *, captured_ns):
      if clock.now >= 102.5 and not state.failed:
        state.failed = True
        raise TimeoutError('Temporary radio interruption after healthy session')
      return {'navigationFeedback': None}
    def exchange_batch(self, payload, *, captured_ns):
      return [(self.exchange(payload, captured_ns=captured_ns), clock.now, 0.)]
    def close(self):
      pass

  class MissingJetson(MissingUsb):
    def __init__(self, host):
      attempts.append('jetson')

  monkeypatch.setattr(bridge, 'AdbTransport', MissingUsb)
  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)
  monkeypatch.setattr(bridge, 'SshTransport', MissingJetson)
  bridge.run(feedback=tmp_path/'feedback.json', status=tmp_path/'status.json', duration=10)
  assert state.failed
  assert attempts == (['bluetooth', 'bluetooth', 'jetson'] if reconnect_fails else ['bluetooth', 'bluetooth'])


@pytest.mark.parametrize('scenario', ['only', 'retry', 'missing_peer', 'enable_live', 'disable_live',
                                      'corrupt_live', 'disable_peer_live'])
def test_bluetooth_only_keeps_usb_for_diagnostics(monkeypatch, tmp_path, scenario):
  from openpilot import cereal
  clock = NS(now=100.0, changed=False, failed=False)
  sm = Messages()
  connections, sent, statuses, probes = [], [], [], []
  config = tmp_path / 'bluetooth.json'
  value = {'enabled': scenario != 'missing_peer', 'peer': '54:ED:A3:00:48:23',
           'bluetooth_only': scenario != 'enable_live'}
  config.write_text(json.dumps(value))
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: int(clock.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))

  def update(_):
    sm.recv_time = dict.fromkeys(bridge.SERVICES, clock.now)
    sm.logMonoTime = dict.fromkeys(bridge.SERVICES, int(clock.now * 1e9))
    if clock.now >= 104.9 and not clock.changed:
      clock.changed = True
      if scenario == 'enable_live':
        value['bluetooth_only'] = True
      elif scenario == 'disable_live':
        value['bluetooth_only'] = False
      elif scenario == 'disable_peer_live':
        value['enabled'] = False
      config.write_text('invalid json' if scenario == 'corrupt_live' else json.dumps(value))

  sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: pytest.fail('Bluetooth-only must not use Jetson'))

  class Usb:
    def connect(self):
      connections.append('usb')
    def exchange(self, payload):
      sent.append('usb')
      return {}
    def close(self):
      pass

  class Wireless:
    def __init__(self, peer):
      self.peer = peer
    def connect(self):
      connections.append('bluetooth')
    def exchange(self, payload, *, captured_ns):
      if scenario == 'retry' and not clock.failed:
        clock.failed = True
        raise ConnectionError('Intentional wireless failure')
      assert payload['active']
      sent.append('bluetooth')
      return {}
    def exchange_batch(self, payload, *, captured_ns):
      return [(self.exchange(payload, captured_ns=captured_ns), clock.now, 0.)]
    def close(self):
      pass

  class Probe:
    def __init__(self, factory):
      pass
    def poll(self, now):
      probes.append(now)
      return None
    def close(self):
      pass

  original = bridge.atomic_json
  def capture(path, value):
    if path.name == 'status.json':
      statuses.append(value)
    original(path, value)

  monkeypatch.setattr(bridge, 'AdbTransport', Usb)
  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)
  monkeypatch.setattr(bridge, 'UsbProbe', Probe)
  monkeypatch.setattr(bridge, 'atomic_json', capture)
  bridge.run(feedback=tmp_path / 'feedback.json', status=tmp_path / 'status.json',
             bluetooth_config=config, duration=7.5)
  assert not probes
  if scenario == 'enable_live':
    assert connections == ['usb', 'bluetooth'] and sent[-1] == 'bluetooth'
  elif scenario == 'disable_live':
    assert connections == ['bluetooth', 'usb'] and sent[-1] == 'usb'
  elif scenario == 'missing_peer':
    assert not connections and not sent
  else:
    assert connections == ['bluetooth'] * (2 if scenario == 'retry' else 1)
    assert set(sent) == {'bluetooth'}
  if scenario in ('missing_peer', 'corrupt_live', 'disable_peer_live'):
    assert not statuses[-2]['connected']
    assert statuses[-2]['transportPolicy'] == 'bluetooth_only'
  assert statuses[-1]['reason'] == 'bridge_stopped'


def test_pipeline_empty_polls_do_not_republish_or_rejuvenate_navigation(monkeypatch, tmp_path):
  from openpilot import cereal
  clock = NS(now=100.)
  sm = Messages()
  sm.update = lambda _: None
  writes, ages, accepts = [], [], []
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: round(clock.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(clock, 'now', clock.now + seconds))
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: ('54:ED:A3:00:48:23', True))
  monkeypatch.setattr(bridge, 'age_model_feedback', lambda value, elapsed: ages.append(elapsed))
  tracker = bridge.ModelNavigationTracker()
  original_accept = tracker.accept
  def accept(value, received, now):
    accepts.append((value, received, now))
    original_accept(value, received, now)
  tracker.accept = accept
  monkeypatch.setattr(bridge, 'ModelNavigationTracker', lambda: tracker)

  class Wireless:
    def __init__(self, peer):
      self.peer, self.calls = peer, 0
    def connect(self):pass
    def close(self):pass
    def exchange_batch(self, payload, *, captured_ns):
      self.calls += 1
      if self.calls == 3:
        return [({'navigationFeedback': {'sequence': 1}}, clock.now - .02, 420.),
                ({'navigationFeedback': {'sequence': 2}}, clock.now - .01, 210.)]
      return []

  original_write = bridge.atomic_json
  def write(path, value):
    if path.name == 'feedback.json':
      writes.append(value)
    original_write(path, value)
  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)
  monkeypatch.setattr(bridge, 'atomic_json', write)
  bridge.run(feedback=tmp_path/'feedback.json', status=tmp_path/'status.json', duration=1.1)
  assert ages == [420., 210.] and len(accepts) == 2
  assert len(writes) == 1
  assert writes[0]['feedback']['navigationFeedback']['sequence'] == 2
  assert writes[0]['receivedAt'] == pytest.approx(100.39)
