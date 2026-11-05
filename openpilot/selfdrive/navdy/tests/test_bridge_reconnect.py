import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.tests.test_bridge import Messages


def healthy_session(reconnect, start=100.):
  for offset in (0., .5, 1., 1.5, 2.):
    reconnect.acknowledge(start + offset, start + offset)


@pytest.mark.parametrize('arrivals', [[], [100.], [100., 100.5, 101., 101.5],
                                    [100., 102.], [100., 100., 100., 100.]])
def test_uptime_sparse_or_duplicate_acks_cannot_earn_fast_retry(arrivals):
  reconnect = bridge.BluetoothReconnect()
  for received in arrivals:
    reconnect.acknowledge(received, received)
  assert not reconnect.take_fast_retry(102.1)


@pytest.mark.parametrize('now,allowed', [(102., True), (104., True), (104.001, False), (101.9, False)])
def test_fast_retry_requires_recent_healthy_ack_history(now, allowed):
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  assert reconnect.take_fast_retry(now) is allowed


@pytest.mark.parametrize('received', [float('nan'), float('inf'), 103., 99.])
def test_invalid_ack_time_discards_health(received):
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  reconnect.acknowledge(received, 102.)
  assert not reconnect.take_fast_retry(102.)


def test_new_session_must_requalify_without_resetting_fast_retry_cooldown():
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  assert reconnect.take_fast_retry(102.)
  assert not reconnect.take_fast_retry(102.)
  reconnect.reset_session()
  assert not reconnect.healthy(102.)
  healthy_session(reconnect, 128.)
  assert not reconnect.take_fast_retry(131.999)
  assert reconnect.take_fast_retry(132.)
  reconnect.reset_session()
  reconnect.acknowledge(164., 164.)
  assert not reconnect.take_fast_retry(164.)


def run_wireless_bridge(monkeypatch, tmp_path, exchange, *, connect=None, duration=5., bluetooth_only=True, usb_at=None):
  from openpilot import cereal

  state = NS(now=100., attempts=[], sent=[], statuses=[], feedback_writes=[], usb_attempts=[], closes=[],
             feedback=tmp_path / 'feedback.json', sm=Messages(), freeze_sources=False)
  state.reply = lambda: [({'navigationFeedback': None}, state.now, 0.)]
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: round(state.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(state, 'now', round(state.now + seconds, 9)))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=lambda: None, close=lambda: None,
                                                     door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: ('54:ED:A3:00:48:23', bluetooth_only))
  monkeypatch.setattr(bridge, 'jetson_candidates', list)

  def update(_):
    if not state.freeze_sources:
      state.sm.recv_time = dict.fromkeys(bridge.SERVICES, state.now)
      state.sm.logMonoTime = dict.fromkeys(bridge.SERVICES, round(state.now * 1e9))
      state.sm.data['carState'].vEgo = state.sm.data['carState'].vEgoCluster = state.now - 100.

  state.sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: state.sm), raising=False)

  class Wireless:
    def __init__(self, peer):
      self.peer = peer

    def connect(self):
      self.attempt = len(state.attempts) + 1
      state.attempts.append(state.now)
      if connect is not None:
        connect(self, state)
      self.connected_at = state.now

    def exchange_batch(self, payload, *, captured_ns):
      assert captured_ns == round(state.now * 1e9)
      state.sent.append((self.attempt, state.now, payload))
      return exchange(self, state)

    def close(self):
      state.closes.append(self.attempt)

  class Usb:
    def connect(self):
      assert not bluetooth_only
      state.usb_attempts.append(state.now)
      if usb_at is None or state.now < usb_at:
        raise ConnectionError('No cable')

    def exchange(self, payload):
      return {'navigationFeedback': None}

    def close(self):
      pass

  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)
  monkeypatch.setattr(bridge, 'AdbTransport', Usb)
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda now: None, close=lambda: None))
  original_write = bridge.atomic_json

  def write(path, value):
    if path.name == 'status.json':
      state.statuses.append(value)
      if not value.get('connected'):
        assert not state.feedback.exists()
    elif path == state.feedback:
      state.feedback_writes.append(value)
    original_write(path, value)

  monkeypatch.setattr(bridge, 'atomic_json', write)
  bridge.run(feedback=state.feedback, status=tmp_path / 'status.json', duration=duration)
  assert not state.feedback.exists()
  assert json.loads((tmp_path / 'status.json').read_text())['reason'] == 'bridge_stopped'
  state.failures = [s for s in state.statuses if 'retryInSeconds' in s]
  state.delays = [s['retryInSeconds'] for s in state.failures]
  return state


@pytest.mark.parametrize('fault,delay', [(ConnectionError, .2), (TimeoutError, .2), (OSError, .2), (ValueError, 2.)])
def test_healthy_exchange_loss_gets_one_fast_retry_but_protocol_errors_do_not(monkeypatch, tmp_path, fault, delay):
  def exchange(link, state):
    if link.attempt == 1 and state.now >= 102.6:
      raise fault('Link interrupted')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange)
  assert state.delays == [delay]
  assert state.attempts == pytest.approx([100., 102.6 + delay])
  assert state.failures[0]['diagnostics']['lastFailure']['phase'] == 'exchange'
  assert state.closes == [1, 2]


@pytest.mark.parametrize('mode', ['empty', 'one_ack', 'short', 'old_ack', 'sparse'])
def test_no_fast_retry_for_unproven_or_silent_sessions(monkeypatch, tmp_path, mode):
  def exchange(link, state):
    elapsed = state.now - link.connected_at
    if elapsed >= (1. if mode == 'short' else 3.):
      raise TimeoutError('No healthy session')
    if mode == 'short' or (mode == 'one_ack' and elapsed == 0):
      return state.reply()
    if mode == 'old_ack':
      return [({}, state.now - 2.1, 0.)]
    if mode == 'sparse' and elapsed in (0., 2.):
      return state.reply()
    return []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=4.)
  assert state.delays == [2.]


@pytest.mark.parametrize('healthy_first', [False, True])
def test_failed_connects_and_empty_polls_keep_bounded_backoff(monkeypatch, tmp_path, healthy_first):
  def connect(link, state):
    if not healthy_first or link.attempt > 2:
      raise ConnectionError('Peer unavailable')

  def exchange(link, state):
    if state.now - link.connected_at >= 2.6:
      raise TimeoutError('Radio lost')
    return state.reply() if link.attempt == 1 else []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, connect=connect, duration=35.)
  assert state.delays == ([.2, 4., 8., 10., 10.] if healthy_first else [2., 4., 8., 10., 10., 10.])
  for failure, attempt in zip(state.failures, state.attempts[1:], strict=False):
    assert attempt - failure['updatedAt'] == pytest.approx(failure['retryInSeconds'])


def test_repeated_healthy_flaps_cannot_rearm_fast_retry_inside_cooldown(monkeypatch, tmp_path):
  def exchange(link, state):
    if state.now - link.connected_at >= 2.6:
      raise ConnectionError('Flapping peer')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=42.)
  fast = [s['updatedAt'] for s in state.failures if s['retryInSeconds'] == .2]
  assert len(fast) == 2
  assert fast[1] - fast[0] >= bridge.BLUETOOTH_FAST_RETRY_COOLDOWN
  assert all(s['retryInSeconds'] == 2. for s in state.failures if s['updatedAt'] not in fast)


@pytest.mark.parametrize('freeze_sources', [False, True])
def test_fast_reconnect_refreshes_payload_without_replaying_feedback(monkeypatch, tmp_path, freeze_sources):
  def connect(link, state):
    if link.attempt == 2:
      assert not state.feedback.exists()
      state.now += 1.

  def exchange(link, state):
    if link.attempt == 1 and state.now >= 102.6:
      state.freeze_sources = freeze_sources
      raise ConnectionError('Transient reset')
    return state.reply() if link.attempt == 1 else []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, connect=connect)
  assert state.delays == [.2]
  first_reconnected = next(payload for attempt, _, payload in state.sent if attempt == 2)
  assert first_reconnected['active'] is not freeze_sources
  assert first_reconnected['vEgoKph'] == pytest.approx(0. if freeze_sources else round(3.8 * 3.6, 1))
  assert state.feedback_writes[-1]['receivedAt'] == pytest.approx(102.4)
  assert all(s['extendedFeedback'] is False for s in state.statuses if s.get('connectedAt') == 103.8)


def test_fast_wireless_retry_still_checks_usb_first(monkeypatch, tmp_path):
  def exchange(link, state):
    if state.now >= 102.6:
      raise ConnectionError('Wireless reset as cable arrives')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, bluetooth_only=False, usb_at=102.7)
  assert state.delays == [.2]
  assert state.attempts == [100.]
  assert state.usb_attempts == pytest.approx([100., 102.8])
  assert state.statuses[-2]['transport'] == 'usb'


@pytest.mark.parametrize('phase', ['payload', 'feedback', 'status'])
def test_non_exchange_errors_do_not_consume_healthy_fast_retry(monkeypatch, tmp_path, phase):
  failed = False

  def exchange(link, state):
    if link.attempt == 2 and state.now - link.connected_at >= 2.6:
      raise ConnectionError('Later transient radio failure')
    return state.reply()

  def fail_once():
    nonlocal failed
    if bridge.time.monotonic() >= 102.6 and not failed:
      failed = True
      raise OSError('Non-radio failure')

  if phase == 'payload':
    original_payload = bridge.live_payload

    def payload(*args, **kwargs):
      fail_once()
      return original_payload(*args, **kwargs)

    monkeypatch.setattr(bridge, 'live_payload', payload)
  elif phase == 'feedback':
    original_age = bridge.age_model_feedback

    def age(*args):
      fail_once()
      return original_age(*args)

    monkeypatch.setattr(bridge, 'age_model_feedback', age)
  else:
    original_write = bridge.atomic_json

    def write(path, value):
      if path.name == 'status.json' and value.get('connected'):
        fail_once()
      original_write(path, value)

    monkeypatch.setattr(bridge, 'atomic_json', write)

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=8.)
  assert state.delays == [2., .2]
  assert state.failures[0]['diagnostics']['lastFailure']['phase'] == phase
