import json
import socket
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.navdy.transport import AdbTransport, LineReader, MAX_FRAME_BYTES, encode_frame


@pytest.fixture(autouse=True)
def no_real_adb_server(monkeypatch):
  def missing(_):
    raise ConnectionRefusedError()
  monkeypatch.setattr('openpilot.selfdrive.navdy.transport.existing_adb_devices', missing)


class FragmentSocket:
  def __init__(self, chunks):
    self.chunks = iter(chunks)

  def recv(self, size):
    return next(self.chunks, b'')


def test_fragmented_utf8_and_multiple_frames():
  frame = encode_frame({'navigationFeedback': {'title': '서울역'}})
  reader = LineReader()
  sock = FragmentSocket([frame[:4], frame[4:20], frame[20:] + b'{"roadPreview":null}\n'])
  assert reader.receive(sock)['navigationFeedback']['title'] == '서울역'
  assert reader.receive(sock) == {'roadPreview': None}


@pytest.mark.parametrize('frame', [b'[]\n', b'{"other":1}\n', b'{', b'x' * (MAX_FRAME_BYTES + 1)])
def test_invalid_response(frame):
  with pytest.raises((ValueError, ConnectionError)):
    LineReader().receive(FragmentSocket([frame]))


def test_socket_timeout_propagates():
  left, right = socket.socketpair()
  try:
    left.settimeout(0.01)
    right.sendall(b'{"roadPreview":')
    with pytest.raises(TimeoutError):
      LineReader().receive(left)
  finally:
    left.close()
    right.close()


def test_maximum_frame_followed_by_another_frame_is_not_oversized():
  frame = encode_frame({'roadPreview': 'x' * (MAX_FRAME_BYTES - len(encode_frame({'roadPreview': ''})))})
  assert len(frame) == MAX_FRAME_BYTES
  reader = LineReader()
  socket = FragmentSocket([frame[:-10], frame[-10:] + b'{"roadPreview":null}\n'])
  assert len(reader.receive(socket)['roadPreview']) > 32000
  assert reader.receive(socket) == {'roadPreview': None}


def test_extreme_nesting_is_a_recoverable_protocol_error():
  with pytest.raises(ValueError, match='nesting'):
    LineReader().receive(FragmentSocket([b'{"roadPreview":' + b'[' * 2000 + b'0' + b']' * 2000 + b'}\n']))


def test_frame_bound_and_nan():
  with pytest.raises(ValueError):
    encode_frame({'title': 'a' * MAX_FRAME_BYTES})
  with pytest.raises(ValueError):
    encode_frame({'speed': float('nan')})
  assert json.loads(encode_frame({'title': '이등병의 편지'}))['title'] == '이등병의 편지'


def test_discover_ignores_phones_unauthorized_and_network_adb():
  calls = []

  def run(cmd, **kwargs):
    calls.append(cmd)
    if cmd[-2:] == ['devices', '-l']:
      result = 'phone device\nnavdy device\nother unauthorized\n192.168.1.2:5555 device\n'
    else:
      result = 'package:/data/app/com.navdy.hud.app/base.apk' if 'navdy' in cmd else ''
    return SimpleNamespace(stdout=result)

  assert AdbTransport(adb='adb', command=run).discover() == 'navdy'
  assert all(cmd[1:3] == ['-P', '5038'] for cmd in calls)
  assert len(calls) == 3


def test_multiple_targets_fail_closed():
  def run(cmd, **kwargs):
    return SimpleNamespace(stdout='a device\nb device\n' if cmd[-2:] == ['devices', '-l'] else 'package:/base.apk')
  with pytest.raises(ConnectionError, match='Multiple'):
    AdbTransport(adb='adb', command=run).discover()


def test_only_discovery_allows_bounded_cold_server_startup():
  calls = []

  def run(cmd, **kwargs):
    calls.append((cmd, kwargs))
    return SimpleNamespace(stdout='navdy device\n' if cmd[-2:] == ['devices', '-l'] else 'package:/base.apk')

  transport = AdbTransport(adb='adb', command=run)
  assert transport.discover() == 'navdy'
  transport.run(['forward', 'tcp:0', 'tcp:8765'], serial='navdy')
  assert [options['timeout'] for _, options in calls] == [15, 3, 3]
  assert all(options['check'] for _, options in calls)
  assert all(cmd[1:3] == ['-P', '5038'] for cmd, _ in calls)


def test_cold_start_timeout_propagates_without_reset_or_retry():
  calls = []

  def run(cmd, **kwargs):
    calls.append(cmd)
    raise subprocess.TimeoutExpired(cmd, kwargs['timeout'])

  with pytest.raises(subprocess.TimeoutExpired) as failure:
    AdbTransport(adb='adb', command=run).discover()
  assert failure.value.timeout == 15
  assert calls == [['adb', '-P', '5038', 'devices', '-l']]


@pytest.mark.parametrize('owner', [5037, 5038])
def test_reuse_existing_owner_without_spawning_competing_server(owner):
  calls = []
  def inventory(port):
    if port != owner:
      raise ConnectionRefusedError()
    return 'navdy device usb:3-1\n'
  def run(cmd, **kw):
    calls.append(cmd)
    return SimpleNamespace(stdout='package:/base.apk')
  transport = AdbTransport(adb='adb', command=run, inventory=inventory)
  assert transport.discover() == 'navdy'
  assert transport.server_port == owner
  assert len(calls) == 1 and calls[0][1:3] == ['-P', str(owner)]
  assert 'devices' not in calls[0]


def test_default_server_owns_target_after_dedicated_server_lost_it():
  transport = AdbTransport(adb='adb', command=lambda *a, **kw: SimpleNamespace(stdout='package:/base.apk'),
                           inventory=lambda port: '' if port == 5038 else 'navdy device usb:3-1')
  assert transport.discover() == 'navdy'
  assert transport.server_port == 5037


def test_existing_offline_server_is_observed_not_replaced():
  seen = []
  def inventory(port):
    return 'navdy offline' if port == 5037 else ''
  transport = AdbTransport(adb='adb', command=lambda *a, **kw: pytest.fail('no new server'), inventory=inventory)
  transport.usb_health = SimpleNamespace(unavailable=lambda servers, link: seen.append(servers))
  with pytest.raises(ConnectionError, match='not found'):
    transport.discover()
  assert seen == [[(5038, ''), (5037, 'navdy offline')]]


def test_forward_cleanup_uses_selected_owner():
  calls = []
  transport = AdbTransport(adb='adb', command=lambda cmd, **kw: calls.append(cmd) or SimpleNamespace(stdout=''))
  transport.server_port, transport.serial, transport.port = 5037, 'navdy', 12345
  transport.close()
  assert calls == [['adb', '-P', '5037', '-s', 'navdy', 'forward', '--remove', 'tcp:12345']]


@pytest.mark.parametrize('error', [TimeoutError(), ValueError(), ConnectionResetError()])
def test_unresponsive_existing_server_does_not_start_a_competitor(error):
  def inventory(port):
    if port == 5038:
      raise ConnectionRefusedError()
    raise error
  transport = AdbTransport(adb='adb', command=lambda *a, **kw: pytest.fail('must not start server'), inventory=inventory)
  with pytest.raises(ConnectionError, match='inventory unavailable'):
    transport.discover()
