"""Lifecycle regressions with synthetic CAN, no native IPC or worker threads."""
from contextlib import contextmanager
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import ambient, bridge


class Clock:
  def __init__(self):
    self.now = 100.0


class BoundedStop:
  def __init__(self, clock, ticks=8):
    self.clock = clock
    self.ticks = ticks
    self.waits = []
    self.stopped = False

  def wait(self, timeout):
    assert timeout > 0, 'Worker retries must not busy-spin'
    self.waits.append(timeout)
    self.clock.now += timeout
    if len(self.waits) > self.ticks:
      self.set()
    return self.stopped

  def is_set(self):
    return self.stopped

  def set(self):
    self.stopped = True


class DoorParser:
  """Match CANParser's per-update samples and cached values, not DBC decoding."""
  def __init__(self, message, signal, address, bus):
    self.message, self.signal = message, signal
    self.address, self.bus = address, bus
    self.vl = {message: {signal: 0}}
    self.vl_all = {message: {signal: []}}
    self.ts_nanos = {message: {signal: 0}}

  def update(self, frames):
    self.vl_all[self.message][self.signal] = []
    for timestamp, messages in frames:
      for address, data, bus in messages:
        if (address, bus) == (self.address, self.bus):
          self.vl[self.message][self.signal] = data[0]
          self.vl_all[self.message][self.signal].append(data[0])
          self.ts_nanos[self.message][self.signal] = timestamp


def parsers():
  return [DoorParser('Door_Open_Switch_Status_LS', 'DrDoorOpenSwAct', 1, 1),
          DoorParser('BCMDoorBeltStatus', 'FrontLeftDoor', 2, 0)]


class Messages:
  def __init__(self, clock, cached_door=None):
    self.clock = clock
    self.data = dict.fromkeys(bridge.SERVICES)
    self.seen = dict.fromkeys(bridge.SERVICES, False)
    self.valid = dict.fromkeys(bridge.SERVICES, True)
    self.alive = dict.fromkeys(bridge.SERVICES, True)
    self.updated = dict.fromkeys(bridge.SERVICES, False)
    self.recv_time = dict.fromkeys(bridge.SERVICES, clock.now - 0.05)
    self.logMonoTime = dict.fromkeys(bridge.SERVICES, round((clock.now - 0.05) * 1e9))
    self.data.update(deviceState=NS(started=False), pandaStates=[])
    self.seen.update(deviceState=True, pandaStates=True)
    if cached_door is not None:
      self.seen['carState'] = True
      self.data['carState'] = NS(canValid=True, canTimeout=False, doorOpen=cached_door,
                                 gearShifter='park', vEgo=0.0)

  def __getitem__(self, name):
    return self.data[name]

  def update(self, _timeout):
    self.updated = dict.fromkeys(bridge.SERVICES, False)
    for name in ('deviceState', 'pandaStates'):
      self.recv_time[name] = self.clock.now
      self.logMonoTime[name] = round(self.clock.now * 1e9)
      self.updated[name] = True


@contextmanager
def gm_params(_raw):
  yield NS(brand='gm')


@pytest.fixture
def harness(monkeypatch, tmp_path):
  clock = Clock()
  messages = Messages(clock)
  monitor = ambient.DoorMonitor(tmp_path / 'OffroadDoorOpen')
  monitor.stop = BoundedStop(clock)
  can_socket = NS(close=lambda: None)
  messaging = NS(SubMaster=lambda _services: messages,
                 sub_sock=lambda *_args, **_kwargs: can_socket,
                 recv_one_or_none=lambda _socket: None)
  cereal = ModuleType('openpilot.cereal')
  cereal.car = NS(CarParams=NS(from_bytes=gm_params))
  cereal.messaging = messaging
  params = ModuleType('openpilot.common.params')
  params.Params = lambda: NS(get=lambda _name: b'gm')
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', cereal)
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', params)
  monkeypatch.setattr(ambient.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(ambient.time, 'monotonic_ns', lambda: round(clock.now * 1e9))
  monkeypatch.setattr(ambient, 'build_door_parsers', parsers)
  return NS(clock=clock, messages=messages, monitor=monitor, messaging=messaging, socket=can_socket)


def can_event(timestamp, door_open):
  return NS(valid=True, logMonoTime=round(timestamp * 1e9),
            can=[NS(address=2, dat=bytes([door_open]), src=0)])


@pytest.mark.parametrize('reverse_parser_order', [False, True])
def test_ordered_batch_keeps_newer_close_across_buses(reverse_parser_order):
  readers = parsers()[::-1] if reverse_parser_order else parsers()
  frames = [(100_000_000_000, [(1, b'\x01', 1)]),
            (100_050_000_000, [(2, b'\x00', 0)])]
  assert ambient.updated_door_state(readers, frames) is False
  assert ambient.updated_door_state(readers, [(100_100_000_000, [])]) is None


def test_simultaneous_bus_samples_still_preserve_an_open_door():
  frames = [(100_000_000_000, [(1, b'\x01', 1), (2, b'\x00', 0)])]
  assert ambient.updated_door_state(parsers(), frames) is True


@pytest.mark.parametrize('cached_door', [False, True])
def test_cached_car_state_cannot_undo_newer_offroad_can(harness, cached_door):
  h = harness
  h.messages.data['carState'] = NS(canValid=True, canTimeout=False, doorOpen=cached_door)
  h.messages.seen['carState'] = True
  h.monitor.remember(cached_door)
  events = [can_event(h.clock.now, not cached_door)]
  h.messaging.recv_one_or_none = lambda _socket: events.pop(0) if events else None

  h.monitor._listen()

  assert h.clock.now - h.messages.recv_time['carState'] > 0.5
  assert h.monitor.error == ''
  assert h.monitor.door_open is not cached_door
  assert h.monitor.path.read_bytes() == (b'0' if cached_door else b'1')


@pytest.mark.parametrize('cached_door', [False, True])
def test_offroad_payload_uses_monitor_not_cached_car_state(harness, cached_door):
  messages = Messages(harness.clock, cached_door=cached_door)
  payload = bridge.live_payload(messages, 1, door_open=not cached_door)
  assert payload['displayPower'] == 'offroad'
  assert payload['doorOpen'] is not cached_door
  assert payload['ambientState'] == ('offroad' if cached_door else 'offroad_door')
  assert not payload['active']


@pytest.mark.parametrize('publication_offset,expected_open', [(0.0005, True), (10.0, False), (-2.0, False)],
                         ids=['arrives-during-drain', 'future-clock-rejected', 'stale-rejected'])
def test_receive_time_freshness_includes_arrivals_during_drain(harness, publication_offset, expected_open):
  h = harness
  delivered = False

  def receive(_socket):
    nonlocal delivered
    if delivered:
      return None
    delivered = True
    event = can_event(h.clock.now + publication_offset, True)
    h.clock.now += 0.001
    return event

  h.messaging.recv_one_or_none = receive
  h.monitor._listen()

  assert delivered
  assert h.monitor.error == ''
  assert h.monitor.door_open is expected_open


def test_transient_socket_initialization_failure_retries_without_a_real_thread(harness):
  h = harness
  attempts = []

  def subscribe(*_args, **_kwargs):
    attempts.append(h.clock.now)
    if len(attempts) == 1:
      raise OSError('transient CAN socket initialization failure')
    return h.socket

  events = []

  def receive(_socket):
    if events:
      return None
    events.append(True)
    return can_event(h.clock.now, True)

  h.messaging.sub_sock = subscribe
  h.messaging.recv_one_or_none = receive
  h.monitor._run()

  assert len(attempts) == 2
  assert attempts[1] > attempts[0], 'Startup failure recovery needs a backoff'
  assert h.monitor.error == ''
  assert h.monitor.door_open is True
  assert h.monitor.stop.is_set()


def test_persistent_initialization_failure_has_backoff_and_obeys_stop(harness):
  h = harness
  attempts = []

  def subscribe(*_args, **_kwargs):
    attempts.append(h.clock.now)
    if len(attempts) > h.monitor.stop.ticks + 1:
      pytest.fail('Unbounded startup retry loop')
    raise OSError('CAN socket unavailable')

  h.messaging.sub_sock = subscribe
  h.monitor._run()

  assert len(attempts) >= 2
  assert all(later > earlier for earlier, later in zip(attempts, attempts[1:]))
  assert h.monitor.stop.is_set()
  assert 'CAN socket unavailable' in h.monitor.error
