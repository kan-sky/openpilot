import json
import socket
import subprocess
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy.tests.test_bluetooth import PEER

PHASES = ['bluetoothctl', 'sdp', 'socket_create', 'security_request', 'socket', 'security', 'hello', 'pipeline']


@pytest.fixture
def radio(monkeypatch):
  from openpilot.selfdrive.navdy import bluetooth_pipeline

  state = NS(now=100., paired=True, channel=3, uuid=bt.SERVICE_UUID, security=2, chunk_size=bt.HELLO.size,
             hello=bt.HELLO.pack(b'NVH2', 0, bt.TTL_US, bt.MAX_BYTES),
             delays={}, errors={}, overruns=set(), calls=[], commands=[], sockets=[], clients=[], pipelines=[])
  monkeypatch.setattr(bt.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(bt.time, 'monotonic_ns', lambda: round(state.now * 1e9))
  monkeypatch.setattr(socket, 'AF_BLUETOOTH', 31, raising=False)
  monkeypatch.setattr(socket, 'BTPROTO_RFCOMM', 3, raising=False)

  def consume(phase, timeout=None):
    state.calls.append({'phase': phase, 'at': state.now, 'timeout': timeout})
    delay = state.delays.get(phase, 0.)
    if isinstance(delay, list):
      delay = delay.pop(0) if delay else 0.
    if timeout is not None and delay > timeout and phase not in state.overruns:
      state.now += timeout
      raise TimeoutError('PRIVATE ' + PEER)
    state.now += delay
    if phase in state.errors:
      raise state.errors[phase]

  def command(args, **kwargs):
    state.commands.append(args)
    assert args[:2] in (['bluetoothctl', 'info'], ['sdptool', 'browse'])
    assert args[2] == PEER
    timeout = kwargs.pop('timeout')
    assert kwargs == {'capture_output': True, 'text': True, 'check': True}
    assert 0 < timeout <= (3 if args[0] == 'bluetoothctl' else 5)
    try:
      consume('bluetoothctl' if args[0] == 'bluetoothctl' else 'sdp', timeout)
    except TimeoutError:
      raise subprocess.TimeoutExpired(args, timeout, output='PRIVATE ' + PEER) from None
    if args[0] == 'bluetoothctl':
      return NS(stdout='Paired: yes' if state.paired else 'Paired: no')
    return NS(stdout=f'Service Name: Navdy\n{state.uuid}\nChannel: {state.channel}')

  class RadioSocket:
    def __init__(self, *args):
      assert args == (socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
      consume('socket_create')
      self.hello = state.hello
      self.closed = False
      self.target = None
      self.security_requested = False
      self.security_checked = False
      self.timeout = None
      state.sockets.append(self)

    def setsockopt(self, *args):
      assert args == (274, 4, bytes([2, 0]))
      consume('security_request')
      self.security_requested = True

    def settimeout(self, timeout):
      assert timeout > 0
      self.timeout = timeout

    def connect(self, target):
      assert self.security_requested and 0 < self.timeout <= 5
      self.target = target
      consume('socket', self.timeout)

    def getsockopt(self, *args):
      assert args == (274, 4, 2)
      consume('security')
      self.security_checked = True
      return bytes([state.security, 0])

    def recv(self, size):
      assert self.security_checked and state.security >= 2 and 0 < self.timeout <= 2
      consume('hello', self.timeout)
      count = min(size, state.chunk_size)
      result, self.hello = self.hello[:count], self.hello[count:]
      return result

    def sendall(self, _):
      pytest.fail('Setup must not transmit application data')

    def close(self):
      if not self.closed:
        self.closed = True
        consume('close')

  def pipeline(conn, received_ns):
    assert conn.security_checked and not conn.hello
    assert received_ns == round(state.now * 1e9)
    consume('pipeline')
    result = NS(closed=False)
    result.close = lambda: setattr(result, 'closed', True)
    state.pipelines.append(result)
    return result

  monkeypatch.setattr(bluetooth_pipeline, 'SnapshotPipeline', pipeline)

  def client(peer=PEER):
    link = bt.BluetoothTransport(peer, command=command, socket_factory=RadioSocket)
    state.clients.append(link)
    return link

  state.client = client
  yield state
  for link in state.clients:
    link.close()
  assert all(sock.closed for sock in state.sockets)
  assert all(pipeline.closed for pipeline in state.pipelines)


def setup_calls(radio):
  return [call for call in radio.calls if call['phase'] != 'close']


@pytest.mark.parametrize('same_transport', [False, True])
def test_each_connection_discovers_current_uuid_channel_before_opening_socket(radio, same_transport):
  first = radio.client()
  first.connect()
  first.close()
  radio.channel = 7
  second = first if same_transport else radio.client()
  second.connect()
  assert [sock.target for sock in radio.sockets] == [(PEER, 3), (PEER, 7)]
  assert [call['phase'] for call in setup_calls(radio)] == PHASES * 2
  assert radio.commands == [['bluetoothctl', 'info', PEER], ['sdptool', 'browse', PEER]] * 2


def test_missing_service_uuid_never_opens_a_socket(radio):
  radio.uuid = 'phone-service'
  with pytest.raises(ConnectionError, match='not advertised'):
    radio.client().connect()
  assert [call['phase'] for call in setup_calls(radio)] == ['bluetoothctl', 'sdp']
  assert not radio.sockets


def test_successful_phase_diagnostics_are_precise_and_address_free(radio):
  radio.delays.update(bluetoothctl=.25, sdp=.5, socket=.75, hello=.125)
  link = radio.client()
  link.connect()
  steps = link.connect_diagnostics['steps']
  assert [step['phase'] for step in steps] == PHASES
  assert [step['elapsedMs'] for step in steps] == [250., 500., 0., 0., 750., 0., 125., 0.]
  assert all(step['ok'] for step in steps)
  assert link.connect_diagnostics['deadlineExpired'] is False
  assert PEER not in json.dumps(link.connect_diagnostics)


@pytest.mark.parametrize('phase', PHASES)
def test_failed_phase_closes_without_any_in_call_retry(radio, phase):
  radio.errors[phase] = TimeoutError('PRIVATE ' + PEER)
  link = radio.client()
  with pytest.raises((TimeoutError, subprocess.TimeoutExpired)):
    link.connect()
  assert link.conn is None and link.pipeline is None
  assert [call['phase'] for call in setup_calls(radio)] == PHASES[:PHASES.index(phase) + 1]
  assert link.connect_diagnostics['steps'][-1]['phase'] == phase
  assert link.connect_diagnostics['steps'][-1]['ok'] is False
  assert PEER not in json.dumps(link.connect_diagnostics)


@pytest.mark.parametrize('fault,phase', [('pairing', 'bluetoothctl'), ('security', 'security'), ('hello', 'hello')])
def test_pairing_security_and_hello_still_fail_closed(radio, fault, phase):
  if fault == 'pairing':
    radio.paired = False
  elif fault == 'security':
    radio.security = 1
  else:
    radio.hello = bt.HELLO.pack(b'BAD!', 0, bt.TTL_US, bt.MAX_BYTES)
  link = radio.client()
  with pytest.raises((ConnectionError, ValueError)):
    link.connect()
  assert link.conn is None and not radio.pipelines
  assert [call['phase'] for call in setup_calls(radio)] == PHASES[:PHASES.index(phase) + 1]
  assert link.connect_diagnostics['steps'][-1]['ok'] is False


@pytest.mark.parametrize('phase', PHASES)
def test_os_or_scheduler_overrun_cannot_start_a_later_phase(radio, phase):
  radio.delays[phase] = bt.CONNECT_TIMEOUT
  radio.overruns.add(phase)
  link = radio.client()
  with pytest.raises(TimeoutError):
    link.connect()
  assert radio.now == 115.
  assert link.conn is None and link.pipeline is None
  assert link.connect_diagnostics['deadlineExpired'] is True
  assert link.connect_diagnostics['steps'][-1]['ok'] is False
  assert [call['phase'] for call in setup_calls(radio)] == PHASES[:PHASES.index(phase) + 1]
  assert all(call['at'] < 115. for call in setup_calls(radio))


@pytest.mark.parametrize('method,phase', [('_check_pairing', 'bluetoothctl'), ('_discover_channel', 'sdp'),
                                       ('_open_socket', 'socket_create'), ('_connect_socket', 'socket')])
def test_expiry_between_phases_does_not_start_the_next_operation(radio, method, phase):
  link = radio.client()
  original = getattr(link, method)

  def delayed(*args):
    radio.now += bt.CONNECT_TIMEOUT
    return original(*args)

  setattr(link, method, delayed)
  with pytest.raises(TimeoutError, match='connection deadline'):
    link.connect()
  assert link.connect_diagnostics['deadlineExpired'] is True
  assert [call['phase'] for call in setup_calls(radio)] == PHASES[:PHASES.index(phase)]
  assert all(call['at'] < 115. for call in setup_calls(radio))


@pytest.mark.parametrize('method,phase', [('_check_pairing', 'bluetoothctl'), ('_discover_channel', 'sdp'),
                                       ('_connect_socket', 'socket')])
def test_command_and_socket_timeouts_are_clamped_to_remaining_budget(radio, method, phase):
  link = radio.client()
  original = getattr(link, method)

  def delayed(*args):
    radio.now += 14.5
    return original(*args)

  setattr(link, method, delayed)
  link.connect()
  assert next(call for call in radio.calls if call['phase'] == phase)['timeout'] == .5
  assert [call['timeout'] for call in radio.calls if call['phase'] == 'hello'] == [.5]
  assert radio.now == 114.5


def test_initial_socket_cleanup_is_charged_to_total_budget(radio):
  link = radio.client()
  link.connect()
  radio.calls.clear()
  radio.delays['close'] = 15.
  with pytest.raises(TimeoutError, match='connection deadline'):
    link.connect()
  assert link.conn is None and not setup_calls(radio)
  assert link.connect_diagnostics == {'deadlineExpired': True, 'steps': []}


@pytest.mark.parametrize('chunk_delay,expected_timeouts,success', [
  (.2, [1., .8, .6, .4], True),
  (.25, [1., .75, .5, .25], False),
  (.4, [1., .6, .2], False),
])
def test_partial_hello_reads_share_the_remaining_overall_deadline(radio, chunk_delay, expected_timeouts, success):
  radio.delays.update(bluetoothctl=3., sdp=5., socket=5., security=1., hello=chunk_delay)
  radio.chunk_size = 4
  link = radio.client()
  if success:
    link.connect()
    assert radio.now == pytest.approx(114.8)
  else:
    with pytest.raises(TimeoutError):
      link.connect()
    assert radio.now == 115.
    assert link.conn is None and not radio.pipelines
    assert link.connect_diagnostics['deadlineExpired'] is True
  assert [call['timeout'] for call in radio.calls if call['phase'] == 'hello'] == pytest.approx(expected_timeouts)
  assert all(call['at'] < 115. for call in setup_calls(radio))


def test_partial_hello_keeps_two_second_phase_limit_when_overall_budget_remains(radio):
  radio.delays['hello'] = .6
  radio.chunk_size = 4
  link = radio.client()
  with pytest.raises(TimeoutError):
    link.connect()
  assert radio.now == 102. and not radio.pipelines
  assert link.connect_diagnostics['deadlineExpired'] is False
  assert [call['timeout'] for call in radio.calls if call['phase'] == 'hello'] == pytest.approx([2., 1.4, .8, .2])


def test_new_explicit_connection_gets_new_budget_and_fresh_discovery(radio):
  link = radio.client()
  radio.delays['security'] = 15.
  with pytest.raises(TimeoutError):
    link.connect()
  radio.delays.clear()
  radio.channel = 7
  link.connect()
  assert link._connect_deadline == 130.
  assert radio.commands == [['bluetoothctl', 'info', PEER], ['sdptool', 'browse', PEER]] * 2
  assert radio.sockets[-1].target == (PEER, 7)


@pytest.mark.parametrize('bluetooth_only', [False, True])
@pytest.mark.parametrize('phase', ['bluetoothctl', 'sdp', 'socket', 'security', 'hello', 'pipeline'])
@pytest.mark.parametrize('failure_mode', ['private_error', 'budget', 'success'])
def test_bridge_persists_address_free_failure_phase_and_deadline_status(radio, monkeypatch, tmp_path, phase, bluetooth_only, failure_mode):
  from openpilot import cereal
  from openpilot.selfdrive.navdy import bridge
  from openpilot.selfdrive.navdy.tests.test_bridge import Messages
  from openpilot.selfdrive.navdy.tests.test_journal import records

  sm = Messages()
  sm.update = lambda _: None
  statuses = []
  real_write = bridge.atomic_json
  if failure_mode == 'budget':
    radio.delays[phase] = 15.
    radio.overruns.add(phase)
  elif failure_mode == 'private_error':
    radio.errors[phase] = TimeoutError('PRIVATE ' + PEER)

  def write(path, value):
    if path.name == 'status.json':
      statuses.append(value)
    real_write(path, value)

  class MissingUsb:
    def connect(self):
      raise ConnectionError('No USB')

    def close(self):
      pass

  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(radio, 'now', round(radio.now + seconds, 9)))
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=lambda: None, close=lambda: None,
                                                     door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: (PEER, bluetooth_only))
  monkeypatch.setattr(bridge, 'BluetoothTransport', radio.client)
  monkeypatch.setattr(bridge, 'jetson_candidates', list)
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda _: None, close=lambda: None))
  monkeypatch.setattr(bridge, 'AdbTransport', MissingUsb)
  monkeypatch.setattr(bridge, 'atomic_json', write)
  monkeypatch.setattr(bt.BluetoothTransport, 'exchange_batch', lambda *args, **kwargs: [])
  bridge.run(feedback=tmp_path / 'feedback.json', status=tmp_path / 'status.json', duration=1.)
  if failure_mode == 'success':
    assert any(row.get('connected') for row in statuses)
    assert len(radio.pipelines) == 1
  else:
    failure = next(row for row in statuses if 'retryInSeconds' in row)
    assert failure['bluetoothConnect']['steps'][-1]['phase'] == phase
    assert failure['bluetoothConnect']['steps'][-1]['ok'] is False
    assert failure['bluetoothConnect']['deadlineExpired'] is (failure_mode == 'budget')
    assert failure['retryInSeconds'] == 2.
    assert statuses[-1]['bluetoothConnect'] == failure['bluetoothConnect']
    assert all(pipeline.closed for pipeline in radio.pipelines)
  # The final status replaces the failure/success status; the journal retains it.
  assert statuses[-1]['reason'] == 'bridge_stopped'
  saved = records(tmp_path / 'diagnostics')
  timed = [row for row in saved if row.get('connectTimings')]
  assert len(timed) == 1
  assert timed[0]['kind'] == ('transport_open' if failure_mode == 'success' else 'connect_failed')
  assert timed[0]['transport'] == 'bluetooth'
  assert timed[0]['connectTimings'] == bridge.connect_timings(statuses[-1]['bluetoothConnect'])
  assert set(timed[0]['connectTimings']) <= set(PHASES)
  assert statuses[-1]['diagnostics']['journal']['dropped'] == 0
  assert PEER not in json.dumps(saved) and 'PRIVATE' not in json.dumps(saved)
  assert PEER not in json.dumps(statuses) and 'PRIVATE' not in json.dumps(statuses)
