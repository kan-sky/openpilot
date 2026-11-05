import json
import socket
import threading
import time
from types import SimpleNamespace as NS
import zlib

import pytest

from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy.connection import UsbProbe
from openpilot.selfdrive.navdy.transport import parse_response

PEER = '54:ED:A3:00:48:23'
PAYLOAD = {'schema': 'navdy.openpilot.v1', 'seq': 1, 'active': False}


def test_disabled_config_is_backward_compatible(tmp_path):
  path = tmp_path / 'wireless.json'
  assert bt.configured_peer(path) is None
  path.write_text('{"enabled":false}')
  assert bt.configured_peer(path) is None
  path.write_text(json.dumps({'enabled': True, 'peer': PEER.lower()}))
  assert bt.configured_peer(path) == PEER
  assert bt.configured_connection(path) == (PEER, False)


@pytest.mark.parametrize('enabled', [True, False])
def test_bluetooth_only_is_explicit_and_survives_disabled_peer(tmp_path, enabled):
  path = tmp_path / 'wireless.json'
  path.write_text(json.dumps({'enabled': enabled, 'peer': PEER, 'bluetooth_only': True}))
  assert bt.configured_connection(path) == (PEER if enabled else None, True)


@pytest.mark.parametrize('value', [None, 0, 1, 'true', [], {}])
def test_invalid_bluetooth_only_policy(tmp_path, value):
  path = tmp_path / 'wireless.json'
  path.write_text(json.dumps({'enabled': True, 'peer': PEER, 'bluetooth_only': value}))
  with pytest.raises(ValueError):
    bt.configured_connection(path)


@pytest.mark.parametrize('value', [[], None, {'enabled': 1, 'peer': PEER}, {'enabled': True},
                                  {'enabled': True, 'peer': 'device-name'}, {'enabled': True, 'peer': PEER+'\n'}])
def test_invalid_config_fails_closed(tmp_path, value):
  path = tmp_path / 'wireless.json'
  path.write_text(json.dumps(value))
  with pytest.raises(ValueError):
    bt.configured_peer(path)


def test_config_size_bound(tmp_path):
  path = tmp_path / 'wireless.json'
  path.write_bytes(b' ' * 4097)
  with pytest.raises(ValueError):
    bt.configured_peer(path)


def test_sdp_only_selects_dedicated_service():
  text = f'Service Name: Phone\nChannel: 2\nService Name: Navdy wireless\nUUID: {bt.SERVICE_UUID}\nChannel: 7\n'
  assert bt.service_channel(text) == 7
  for bad in [text + text, text.replace(bt.SERVICE_UUID, 'other'), text.replace('Channel: 7', 'Channel: 31'),
              text + 'Channel: 8\n']:
    with pytest.raises((ValueError, ConnectionError)):
      bt.service_channel(bad)


@pytest.mark.parametrize('compressed', [False, True])
def test_codec_utf8_and_limit(compressed):
  raw = json.dumps({'navigationFeedback': '서울역'}, ensure_ascii=False).encode()
  wire = zlib.compress(raw, 1) if compressed else raw
  assert bt.unpack_payload(wire, len(raw), int(compressed), zlib.crc32(raw)) == raw
  with pytest.raises(ValueError):
    bt.unpack_payload(wire, len(raw), int(compressed), 1)
  with pytest.raises(ValueError):
    bt.unpack_payload(wire, len(raw)+1, int(compressed), zlib.crc32(raw))


@pytest.mark.parametrize('kind', ['bomb', 'truncated', 'trailing', 'flags', 'oversize'])
def test_bad_compression_is_bounded(kind):
  raw = b'x' * 64
  wire = zlib.compress(raw)
  size, flags = len(raw), 1
  if kind == 'bomb':wire = zlib.compress(b'x' * 1000000)
  if kind == 'truncated':wire = wire[:-2]
  if kind == 'trailing':wire += b'x'
  if kind == 'flags':flags = 2
  if kind == 'oversize':size = bt.MAX_BYTES + 1
  with pytest.raises(ValueError):
    bt.unpack_payload(wire, size, flags, zlib.crc32(raw))


@pytest.mark.parametrize('raw', [b'{"navigationFeedback":NaN}', b'{"navigationFeedback":1e999}',
                                 b'{"navigationFeedback":"\xff"}',
                                 b'[]', b'{"unknown":1}', b'{"roadPreview":'+b'['*33+b'0'+b']'*33+b'}'])
def test_bad_response_rejected(raw):
  with pytest.raises(ValueError):
    parse_response(raw)


def test_navigation_source_deadline_is_never_extended():
  packet = {'navigationFeedback': {'session': 'original', 'sequence': 12, 'ageMs': 100, 'validForMs': 200},
            'roadPreview': {'ageMs': 5, 'validForMs': 10}}
  aged = bt.age_feedback(packet, 50.2)
  assert aged['navigationFeedback'] == {'session': 'original', 'sequence': 12, 'ageMs': 150.2, 'validForMs': 149}
  assert aged['roadPreview']['validForMs'] == 0


def transaction(*, status=0, age=20000, ack_seq=0, lease=1, magic=b'NVA1', oversize=False, delay=0, corrupt=False):
  left, right = socket.socketpair()
  client = bt.BluetoothTransport(PEER)
  client.conn = left
  client.lease_received_ns = time.monotonic_ns()
  errors = []

  def server():
    try:
      head = bt.receive(right, bt.REQUEST.size, time.monotonic()+2)
      marker, sequence, received_lease, delta, size, wire_size, crc, flags = bt.REQUEST.unpack(head)
      wire = bt.receive(right, wire_size, time.monotonic()+2)
      assert marker == b'NVQ1' and sequence == received_lease == 0 and delta >= 0
      assert json.loads(bt.unpack_payload(wire, size, flags, crc)) == PAYLOAD
      raw = b'{"navigationFeedback":null,"roadPreview":null}'
      wire = zlib.compress(raw, 1)
      reply = bt.RESPONSE.pack(magic, ack_seq, status, lease, age, len(raw),
                               bt.MAX_BYTES+1 if oversize else len(wire), zlib.crc32(raw)+(1 if corrupt else 0), 1)
      if delay:time.sleep(delay)
      right.sendall(reply+wire)
    except (BrokenPipeError, ConnectionResetError):
      pass
    except Exception as error:
      errors.append(error)
    finally:
      right.close()

  thread = threading.Thread(target=server)
  thread.start()
  return client, thread, errors


def test_complete_transaction_returns_real_feedback():
  client, thread, errors = transaction()
  try:
    assert client.exchange(PAYLOAD, captured_ns=time.monotonic_ns()) == {'navigationFeedback': None, 'roadPreview': None}
    assert client.sequence == client.lease == 1
    assert client.last_exchange['receiverAgeMs'] == 20
  finally:
    client.close()
    thread.join(2)
  assert not thread.is_alive() and not errors


@pytest.mark.parametrize('fault', [dict(status=1), dict(status=2), dict(status=3), dict(age=-1), dict(age=bt.TTL_US),
                                   dict(ack_seq=1), dict(lease=0), dict(magic=b'BAD!'), dict(oversize=True),
                                   dict(corrupt=True), dict(delay=.8)])
def test_bad_or_expired_ack_closes_connection(fault):
  client, thread, errors = transaction(**fault)
  try:
    with pytest.raises((ValueError, ConnectionError, TimeoutError)):
      client.exchange(PAYLOAD, captured_ns=time.monotonic_ns())
    assert client.conn is None
  finally:
    client.close()
    thread.join(2)
  assert not thread.is_alive() and not errors


@pytest.mark.parametrize('delay_ns', [500_000_000, 1_000_000_000, -1])
def test_snapshot_age_is_measured_from_payload_creation(monkeypatch, delay_ns):
  client = bt.BluetoothTransport(PEER)
  client.conn = object()  # Any attempt to use the socket fails the test.
  client.lease_received_ns = 100_000_000_000
  captured = 101_000_000_000
  monkeypatch.setattr(bt.time, 'monotonic_ns', lambda: captured + delay_ns)
  with pytest.raises(ConnectionError, match='expired before transmission'):
    client.exchange(PAYLOAD, captured_ns=captured)


def test_connect_does_not_pair_unknown_peer(monkeypatch):
  monkeypatch.setattr(socket, 'AF_BLUETOOTH', 31, raising=False)
  calls = []

  def command(args, **kwargs):
    calls.append(args)
    return NS(stdout='Paired: no')

  client = bt.BluetoothTransport(PEER, command=command)
  with pytest.raises(ConnectionError, match='not paired'):
    client.connect()
  assert calls == [['bluetoothctl', 'info', PEER]]


def test_connect_checks_security_and_hello(monkeypatch):
  from openpilot.selfdrive.navdy import bluetooth_pipeline
  monkeypatch.setattr(bluetooth_pipeline, 'SnapshotPipeline', lambda *args: NS(close=lambda: None))
  monkeypatch.setattr(socket, 'AF_BLUETOOTH', 31, raising=False)
  monkeypatch.setattr(socket, 'BTPROTO_RFCOMM', 3, raising=False)
  class FakeSocket:
    def __init__(self, security=2, hello=None):
      self.security = security
      self.hello = hello or bt.HELLO.pack(b'NVH1', 0, bt.TTL_US, bt.MAX_BYTES)
      self.closed = False
    def setsockopt(self, *args):pass
    def settimeout(self, value):pass
    def connect(self, target):assert target == (PEER, 3)
    def getsockopt(self, *args):return bytes([self.security, 0])
    def recv(self, size):result, self.hello = self.hello[:size], self.hello[size:]; return result
    def close(self):self.closed = True
    def setblocking(self, value):assert value is False
  def command(args, **kwargs):
    return NS(stdout='Paired: yes' if args[0]=='bluetoothctl' else f'Service Name: Navdy\n{bt.SERVICE_UUID}\nChannel: 3')
  v2 = bt.HELLO.pack(b'NVH2', 0, bt.TTL_US, bt.MAX_BYTES)
  for security, hello in [(1, None), (2, bt.HELLO.pack(b'HLS0', 0, bt.TTL_US, bt.MAX_BYTES)), (2, None), (2, v2)]:
    sock = FakeSocket(security, hello)
    client = bt.BluetoothTransport(PEER, command=command, socket_factory=lambda *a: sock)
    if security == 2 and hello in (None, v2):
      client.connect()
      assert client.conn is sock
      assert (client.pipeline is not None) == (hello == v2)
    else:
      with pytest.raises((ConnectionError, ValueError)):
        client.connect()
      assert sock.closed
    client.close()


def test_v1_batch_fallback_delivers_one_reply_once():
  client, thread, errors = transaction()
  try:
    replies = client.exchange_batch(PAYLOAD, captured_ns=time.monotonic_ns())
    assert len(replies) == 1 and replies[0][0]['navigationFeedback'] is None
    assert replies[0][1] > 0 and replies[0][2] >= 0
  finally:
    client.close()
    thread.join(2)
  assert not errors


def test_usb_probe_never_writes_and_hands_off_once():
  connected, release = threading.Event(), threading.Event()
  class Usb:
    def __init__(self):self.closed = False
    def connect(self):connected.set(); release.wait(2)
    def close(self):self.closed = True
    def exchange(self, _):raise AssertionError('Probe must never send a snapshot')
  candidate = Usb()
  probe = UsbProbe(lambda: candidate)
  assert probe.poll(0) is None
  assert connected.wait(1)
  assert probe.poll(3) is None
  release.set()
  probe.thread.join(1)
  assert probe.poll(4) is candidate
  probe.close()
  assert not candidate.closed  # ownership passed to the display writer
  candidate.close()


def test_late_usb_probe_is_closed_on_shutdown():
  release = threading.Event()
  class Usb:
    closed = False
    def connect(self):release.wait(2)
    def close(self):self.closed = True
  candidate = Usb()
  probe = UsbProbe(lambda: candidate)
  probe.poll(0)
  probe.close()
  release.set()
  probe.thread.join(1)
  assert candidate.closed and probe.ready is None


def test_receive_total_deadline_not_per_byte():
  left, right = socket.socketpair()
  try:
    right.sendall(b'a')
    with pytest.raises(TimeoutError):
      bt.receive(left, 2, time.monotonic()+.02)
  finally:
    left.close()
    right.close()
