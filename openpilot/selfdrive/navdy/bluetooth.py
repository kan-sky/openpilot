"""Encrypted, explicitly paired Navdy full-state link. No pairing or radio resets."""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
from pathlib import Path
import re
import socket
import struct
import subprocess
import time
import zlib

from openpilot.selfdrive.navdy.transport import parse_response

CONFIG_PATH = Path('/data/navdy/bluetooth.json')
SERVICE_UUID = '9b07004c-c95d-4e82-bf6f-9af7679b4a21'
HELLO = struct.Struct('!4sIII')
REQUEST = struct.Struct('!4sIIqIIII')
RESPONSE = struct.Struct('!4sIIIqIIII')
MAX_BYTES = 32768
TTL_US = 500000
EXCHANGE_TIMEOUT = 0.65
CONNECT_TIMEOUT = 15.0
MAC_PATTERN = re.compile(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}')


def configured_connection(path=CONFIG_PATH):
  try:
    with path.open('rb') as stream:
      raw = stream.read(4097)
  except FileNotFoundError:
    return None, False
  if len(raw) > 4096:
    raise ValueError('Navdy Bluetooth config exceeds bound')
  value = json.loads(raw)
  if not isinstance(value, dict) or type(value.get('enabled')) is not bool:
    raise ValueError('Invalid Navdy Bluetooth configuration')
  bluetooth_only = value.get('bluetooth_only', False)
  if type(bluetooth_only) is not bool:
    raise ValueError('Navdy bluetooth_only must be a boolean')
  if not value['enabled']:
    return None, bluetooth_only
  peer = value.get('peer')
  if not isinstance(peer, str) or not MAC_PATTERN.fullmatch(peer):
    raise ValueError('An exact paired Navdy Bluetooth address is required')
  return peer.upper(), bluetooth_only


def configured_peer(path=CONFIG_PATH):
  return configured_connection(path)[0]


def service_channel(text):
  matches = []
  for block in re.split(r'(?=Service Name:)', text):
    if SERVICE_UUID not in block.lower():
      continue
    channels = re.findall(r'Channel:\s*(\d+)', block)
    if len(channels) != 1:
      raise ValueError('Ambiguous Navdy Bluetooth service')
    matches.append(int(channels[0]))
  if len(matches) != 1 or not 1 <= matches[0] <= 30:
    raise ConnectionError('Navdy wireless v1 service not advertised')
  return matches[0]


def receive(conn, size, deadline):
  chunks = bytearray()
  while len(chunks) < size:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
      raise TimeoutError('Navdy Bluetooth response deadline')
    conn.settimeout(remaining)
    chunk = conn.recv(size - len(chunks))
    if not chunk:
      raise ConnectionError('Navdy Bluetooth disconnected')
    chunks.extend(chunk)
  return bytes(chunks)


def unpack_payload(wire, raw_size, flags, checksum):
  if not 1 <= raw_size <= MAX_BYTES or not 1 <= len(wire) <= MAX_BYTES:
    raise ValueError('Navdy wireless frame exceeds bound')
  if flags == 0:
    raw = wire
  elif flags == 1:
    inflater = zlib.decompressobj()
    try:
      raw = inflater.decompress(wire, raw_size + 1)
    except zlib.error as error:
      raise ValueError('Invalid Navdy compressed response') from error
    if not inflater.eof or inflater.unused_data or inflater.unconsumed_tail:
      raise ValueError('Incomplete or trailing Navdy compressed response')
  else:
    raise ValueError('Unknown Navdy wireless encoding')
  if len(raw) != raw_size or zlib.crc32(raw) != checksum:
    raise ValueError('Navdy wireless size/checksum mismatch')
  return raw


def age_feedback(value, elapsed_ms):
  # Charge the full round trip as a conservative return-transit budget without
  # refreshing any source sequence or clock.
  for name in ('roadPreview', 'navigationFeedback'):
    component = value.get(name)
    if not isinstance(component, dict):
      continue
    age = component.get('ageMs')
    if isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age):
      component['ageMs'] = age + elapsed_ms
    remaining = component.get('validForMs')
    if type(remaining) is int:
      component['validForMs'] = max(0, remaining - math.ceil(elapsed_ms))
  return value


class BluetoothTransport:
  def __init__(self, peer, *, command=subprocess.run, socket_factory=socket.socket):
    if not MAC_PATTERN.fullmatch(peer):
      raise ValueError('Invalid Navdy Bluetooth address')
    self.peer = peer.upper()
    self.command = command
    self.socket_factory = socket_factory
    self.conn = None
    self.sequence = 0
    self.lease = 0
    self.lease_received_ns = 0
    self.last_exchange = None
    self.pipeline = None
    self.connect_diagnostics = None
    self._connect_deadline = 0.0

  def _connect_timeout(self, limit):
    remaining = self._connect_deadline - time.monotonic()
    if remaining <= 0:
      self.connect_diagnostics['deadlineExpired'] = True
      raise TimeoutError('Navdy Bluetooth connection deadline')
    return min(limit, remaining)

  @contextmanager
  def _connect_phase(self, phase):
    self._connect_timeout(CONNECT_TIMEOUT)
    started = time.monotonic()
    step = {'phase': phase, 'ok': False}
    self.connect_diagnostics['steps'].append(step)
    try:
      yield
      self._connect_timeout(CONNECT_TIMEOUT)
      step['ok'] = True
    finally:
      now = time.monotonic()
      step['elapsedMs'] = round(max(0., now - started) * 1000, 2)
      if now >= self._connect_deadline:
        self.connect_diagnostics['deadlineExpired'] = True

  def _check_pairing(self):
    with self._connect_phase('bluetoothctl'):
      info = self.command(['bluetoothctl', 'info', self.peer], capture_output=True, text=True,
                          timeout=self._connect_timeout(3), check=True).stdout
      if not re.search(r'^\s*Paired: yes\s*$', info, re.MULTILINE):
        raise ConnectionError('Configured Navdy is not paired; pairing is never automatic')

  def _discover_channel(self):
    with self._connect_phase('sdp'):
      sdp = self.command(['sdptool', 'browse', self.peer], capture_output=True, text=True,
                         timeout=self._connect_timeout(5), check=True).stdout
      channel = service_channel(sdp)
    return channel

  def _open_socket(self):
    with self._connect_phase('socket_create'):
      self.conn = self.socket_factory(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    with self._connect_phase('security_request'):
      self.conn.setsockopt(274, 4, bytes([2, 0]))  # SOL_BLUETOOTH, BT_SECURITY_MEDIUM

  def _connect_socket(self, channel):
    with self._connect_phase('socket'):
      self.conn.settimeout(self._connect_timeout(5))
      self.conn.connect((self.peer, channel))

  def connect(self):
    # One setup budget, including cleanup; OS/scheduler delays can still overrun.
    self._connect_deadline = time.monotonic() + CONNECT_TIMEOUT
    self.connect_diagnostics = {'deadlineExpired': False, 'steps': []}
    self.close()
    try:
      if not hasattr(socket, 'AF_BLUETOOTH'):
        raise OSError('Native Bluetooth RFCOMM is unavailable on this OS')
      self._check_pairing()
      # An old RFCOMM channel may now belong to the phone acceptor. Discover
      # this UUID every time, before opening any socket, even on reconnect.
      channel = self._discover_channel()
      self._open_socket()
      self._connect_socket(channel)
      with self._connect_phase('security'):
        if self.conn.getsockopt(274, 4, 2)[0] < 2:
          raise ConnectionError('Navdy requires an encrypted paired link')
      with self._connect_phase('hello'):
        hello_deadline = min(self._connect_deadline, time.monotonic() + 2)
        hello = receive(self.conn, HELLO.size, hello_deadline)
        if time.monotonic() >= hello_deadline:
          raise TimeoutError('Navdy Bluetooth response deadline')
        marker, lease, ttl, bound = HELLO.unpack(hello)
        if marker not in (b'NVH1', b'NVH2') or (lease, ttl, bound) != (0, TTL_US, MAX_BYTES):
          raise ValueError('Unsupported Navdy wireless handshake')
      self.lease, self.sequence = lease, 0
      self.lease_received_ns = time.monotonic_ns()
      if marker == b'NVH2':
        with self._connect_phase('pipeline'):
          from openpilot.selfdrive.navdy.bluetooth_pipeline import SnapshotPipeline
          self.pipeline = SnapshotPipeline(self.conn, self.lease_received_ns)
      self._connect_timeout(CONNECT_TIMEOUT)
    except Exception:
      self.close()
      raise

  def exchange_batch(self, payload, *, captured_ns):
    """Drain real replies once; an empty poll must never refresh navigation."""
    if self.pipeline is None:
      begin = time.monotonic()
      response = self.exchange(payload, captured_ns=captured_ns)
      received = time.monotonic()
      return [(response, received, (received - begin) * 1000)]
    try:
      replies = self.pipeline.update(payload, captured_ns=captured_ns)
      self.last_exchange = self.pipeline.diagnostics()
      return replies
    except Exception:
      self.close()
      raise

  def exchange(self, payload, *, captured_ns):
    if self.pipeline is not None:
      raise ValueError('Navdy wireless v2 requires exchange_batch')
    if self.conn is None:
      # Caller must connect first, then construct a fresh state snapshot.
      raise ConnectionError('Navdy wireless link is not connected')
    if type(captured_ns) is not int:
      raise ValueError('Navdy snapshot capture timestamp is required')
    if not isinstance(payload, dict) or payload.get('schema') != 'navdy.openpilot.v1':
      raise ValueError('Only Navdy display snapshots are supported')
    raw = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if not 1 <= len(raw) <= MAX_BYTES:
      raise ValueError('Navdy display snapshot exceeds bound')
    wire = zlib.compress(raw, 1)
    if len(wire) > MAX_BYTES:
      raise ValueError('Navdy compressed snapshot exceeds bound')
    delta_us = (captured_ns - self.lease_received_ns) // 1000
    if not 0 <= delta_us <= 10000000 or self.sequence > 2147483647:
      raise ConnectionError('Navdy wireless session lease expired')
    if not 0 <= time.monotonic_ns() - captured_ns < TTL_US * 1000:
      raise ConnectionError('Navdy snapshot expired before transmission')
    begin = time.monotonic()
    deadline = begin + EXCHANGE_TIMEOUT
    header = REQUEST.pack(b'NVQ1', self.sequence, self.lease, delta_us,
                          len(raw), len(wire), zlib.crc32(raw), 1)
    try:
      self.conn.settimeout(EXCHANGE_TIMEOUT)
      self.conn.sendall(header + wire)
      marker, sequence, status, lease, age, size, wire_size, crc, flags = RESPONSE.unpack(receive(self.conn, RESPONSE.size, deadline))
      if marker != b'NVA1' or sequence != self.sequence or lease != self.lease + 1:
        raise ValueError('Navdy wireless acknowledgement mismatch')
      if not 1 <= wire_size <= MAX_BYTES:
        raise ValueError('Navdy wireless response exceeds bound')
      response = unpack_payload(receive(self.conn, wire_size, deadline), size, flags, crc)
      received_ns = time.monotonic_ns()
      if status != 0 or not 0 <= age < TTL_US:
        raise ConnectionError(f'Navdy wireless state rejected (status={status})')
      value = parse_response(response)
      elapsed_ms = (time.monotonic() - begin) * 1000
      self.last_exchange = {'rttMs': round(elapsed_ms, 2), 'receiverAgeMs': round(age / 1000, 2), 'sequence': sequence}
      self.lease, self.lease_received_ns = lease, received_ns
      self.sequence += 1
      return age_feedback(value, elapsed_ms)
    except Exception:
      self.close()
      raise

  def close(self):
    if self.pipeline is not None:
      self.pipeline.close()
    if self.conn is not None:
      self.conn.close()
      self.conn = None
    self.pipeline = None
