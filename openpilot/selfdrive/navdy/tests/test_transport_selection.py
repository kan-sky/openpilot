import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.tests.test_bridge import Messages


def run_selection(monkeypatch, tmp_path, *, modes, cable=True, peer=True, jetson=False, fault=None, duration=12):
  from openpilot import cereal
  state = NS(now=100., connects=[], closes=[], sent=[], statuses=[], probes=[], failures=0)
  sm = Messages()
  feedback = tmp_path / 'feedback.json'
  config = tmp_path / 'bluetooth.json'
  original_config = json.dumps({'enabled': peer, 'peer': '54:ED:A3:00:48:23', 'bluetooth_only': True})
  config.write_text(original_config)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: round(state.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(state, 'now', round(state.now + seconds, 9)))
  monkeypatch.setattr(bridge, 'read_transport_mode', lambda: modes(state.now))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda: NS(start=lambda: None, close=lambda: None,
                                                      door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: ['192.0.2.1'] if jetson else [])

  def update(_):
    sm.recv_time = dict.fromkeys(bridge.SERVICES, state.now)
    sm.logMonoTime = dict.fromkeys(bridge.SERVICES, round(state.now * 1e9))
  sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: sm), raising=False)

  class Link:
    def __init__(self, kind):
      self.kind = kind
      self.peer = '54:ED:A3:00:48:23'
    def connect(self):
      state.connects.append((state.now, self.kind))
      if self.kind == 'usb' and not cable:
        raise ConnectionError('No USB cable')
      if fault == 'connect':
        raise ConnectionError('Test connect failure')
    def exchange(self, payload):
      if fault == 'once' and state.failures == 0:
        state.failures += 1
        raise ConnectionError('Test cable loss')
      state.sent.append((state.now, self.kind, payload))
      return {'navigationFeedback': {'fixture': self.kind}}
    def exchange_batch(self, payload, *, captured_ns):
      assert captured_ns == round(state.now * 1e9)
      return [(self.exchange(payload), state.now, 0.)]
    def close(self):
      state.closes.append((state.now, self.kind))

  monkeypatch.setattr(bridge, 'AdbTransport', lambda: Link('usb'))
  monkeypatch.setattr(bridge, 'BluetoothTransport', lambda _: Link('bluetooth'))
  monkeypatch.setattr(bridge, 'SshTransport', lambda _: Link('jetson'))
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda now: state.probes.append(now), close=lambda: None))
  original_write = bridge.atomic_json

  def write(path, value):
    if path.name == 'status.json':
      state.statuses.append(value)
      if not value['connected']:
        assert not feedback.exists(), 'Transport change/failure must clear stale navigation'
    original_write(path, value)
  monkeypatch.setattr(bridge, 'atomic_json', write)
  bridge.run(feedback=feedback, status=tmp_path / 'status.json', bluetooth_config=config, duration=duration)
  assert config.read_text() == original_config
  assert not feedback.exists()
  return state


@pytest.mark.parametrize('mode,expected', [(0, 'bluetooth'), (1, 'usb'), (2, 'bluetooth')])
def test_selected_transport_owns_every_sent_frame(monkeypatch, tmp_path, mode, expected):
  state = run_selection(monkeypatch, tmp_path, modes=lambda _: mode)
  assert {kind for _, kind, _ in state.sent} == {expected}
  assert len(state.sent) == 60
  assert not state.probes


@pytest.mark.parametrize('peer,jetson,expected', [(True, False, None), (False, False, None), (True, True, 'jetson')])
def test_wired_missing_cable_never_uses_bluetooth(monkeypatch, tmp_path, peer, jetson, expected):
  state = run_selection(monkeypatch, tmp_path, modes=lambda _: 1, cable=False, peer=peer, jetson=jetson)
  assert 'bluetooth' not in [kind for _, kind in state.connects]
  assert {kind for _, kind, _ in state.sent} == ({expected} if expected else set())
  statuses = state.statuses[:-1]
  assert all(s['transportPolicy'] == 'usb_only' for s in statuses)
  if expected is None:
    assert all(not s['connected'] for s in statuses)
    assert 'wired mode' in statuses[-1]['reason']


def test_wireless_missing_peer_never_uses_available_cable_or_jetson(monkeypatch, tmp_path):
  state = run_selection(monkeypatch, tmp_path, modes=lambda _: 2, peer=False, jetson=True)
  assert not state.connects and not state.sent and not state.probes


def test_bidirectional_live_switch_closes_old_link_and_clears_feedback(monkeypatch, tmp_path):
  state = run_selection(monkeypatch, tmp_path, modes=lambda now: 2 if now < 105 or now >= 110 else 1)
  assert [kind for _, kind in state.connects] == ['bluetooth', 'usb', 'bluetooth']
  assert [kind for _, kind in state.closes] == ['bluetooth', 'usb', 'bluetooth']
  assert all(kind == ('usb' if 105 <= at < 110 else 'bluetooth') for at, kind, _ in state.sent)
  assert [s['transportPolicy'] for s in state.statuses if s.get('reason') == 'connection_mode_changed'] == [
    'usb_only', 'bluetooth_only']


def test_wired_cable_error_retries_only_wired(monkeypatch, tmp_path):
  state = run_selection(monkeypatch, tmp_path, modes=lambda _: 1, fault='once')
  assert [kind for _, kind in state.connects] == ['usb', 'usb']
  assert any(s.get('retryInSeconds') == 2 for s in state.statuses)
  assert state.sent


def test_switch_during_failed_connect_backoff_is_not_delayed_by_old_policy(monkeypatch, tmp_path):
  state = run_selection(monkeypatch, tmp_path, modes=lambda now: 2 if now < 105 else 1, fault='connect')
  assert next(at for at, kind in state.connects if kind == 'usb') == pytest.approx(105.)


def test_corrupt_mode_cannot_fall_back_or_leave_old_feedback(monkeypatch, tmp_path):
  def mode(now):
    if now >= 105:
      raise ValueError('Invalid test mode')
    return 1
  state = run_selection(monkeypatch, tmp_path, modes=mode)
  assert [kind for _, kind in state.connects] == ['usb']
  assert all(at < 105 for at, _, _ in state.sent)
  assert state.statuses[-2]['transportPolicy'] == 'invalid'
