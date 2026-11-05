"""Bounded full-state pipelining, independent of the receiver's 500 ms expiry."""
from collections import deque
import json
import queue
import select
import threading
import time
import zlib

from openpilot.selfdrive.navdy.bluetooth import MAX_BYTES, REQUEST, RESPONSE, TTL_US, age_feedback, unpack_payload
from openpilot.selfdrive.navdy.transport import parse_response

MAX_IN_FLIGHT = 2
MAX_WRITE_ATTEMPTS = 16
RESPONSE_TIMEOUT_NS = 1_500_000_000
LEASE_HISTORY = 8


class ResponseReader:
  """One bounded IO reader; timestamp full ACK receipt before the display tick."""
  def __init__(self, conn, *, threaded=True):
    self.conn = conn
    self.frames = queue.Queue(maxsize=MAX_IN_FLIGHT)
    self.rx = bytearray()
    self.error = None
    self.stop = threading.Event()
    self.thread = None
    if threaded:
      self.thread = threading.Thread(target=self._run, name='NavdyAckReader', daemon=True)
      self.thread.start()

  def _receive_available(self):
    for _ in range(MAX_IN_FLIGHT):
      while True:
        if self.stop.is_set():
          return
        required = RESPONSE.size
        if len(self.rx) >= RESPONSE.size:
          head = RESPONSE.unpack(self.rx[:RESPONSE.size])
          if head[0] != b'NVA2' or not 1 <= head[5] <= MAX_BYTES or not 1 <= head[6] <= MAX_BYTES:
            raise ValueError('Navdy wireless v2 response header/bound')
          required += head[6]
        if len(self.rx) == required and required > RESPONSE.size:
          break
        try:
          chunk = self.conn.recv(required - len(self.rx))
        except BlockingIOError:
          return
        if not chunk:
          raise ConnectionError('Navdy Bluetooth disconnected')
        self.rx.extend(chunk)
      received_ns = time.monotonic_ns()
      wire = bytes(self.rx[RESPONSE.size:required])
      self.rx.clear()
      try:
        self.frames.put_nowait((head, wire, received_ns))
      except queue.Full as error:
        raise ValueError('Navdy wireless v2 unsolicited response overflow') from error

  def _run(self):
    try:
      while not self.stop.is_set():
        if select.select([self.conn], [], [], .05)[0]:
          self._receive_available()
    except (OSError, ValueError) as error:
      self.error = error

  def pop(self):
    if self.thread is None:
      self._receive_available()
    if self.error is not None:
      raise self.error
    try:
      return self.frames.get_nowait()
    except queue.Empty:
      return None

  def close(self):
    self.stop.set()
    if self.thread is not None:
      self.thread.join(.2)


class SnapshotPipeline:
  def __init__(self, conn, hello_received_ns, *, reader_factory=ResponseReader):
    self.conn = conn
    self.conn.setblocking(False)
    self.leases = deque([(0, hello_received_ns)], maxlen=LEASE_HISTORY)
    self.pending = deque()
    self.sequence = 0
    self.reader = reader_factory(conn)
    self.tx = b''
    self.tx_captured_ns = 0
    self.last_ack = None
    self.last_received_ns = None
    self.skipped = 0
    self.partial_writes = 0
    self.blocked_writes = 0
    self.last_tx_ms = None

  def _read(self):
    replies = []
    # A poll consumes at most the two acknowledged requests. No unbounded drain
    # loop, response queue or re-publication of a cached reply.
    for _ in range(MAX_IN_FLIGHT):
      packet = self.reader.pop()
      if packet is None:
        break
      head, wire, received_ns = packet
      marker, sequence, status, lease, age, size, wire_size, crc, flags = head
      if (marker != b'NVA2' or not self.pending or sequence != self.pending[0][0]
          or lease != sequence + 1 or status not in (0, 1, 3)):
        raise ValueError('Navdy wireless v2 acknowledgement mismatch')
      raw = unpack_payload(wire, size, flags, crc)
      if not 0 <= age <= 10_000_000 or (status == 0 and age >= TTL_US):
        raise ValueError('Navdy wireless v2 invalid acknowledgement age')
      value = parse_response(raw)
      _, captured_ns = self.pending.popleft()
      elapsed_ms = (received_ns - captured_ns) / 1e6
      if not 0 <= received_ns - captured_ns < RESPONSE_TIMEOUT_NS:
        raise TimeoutError('Navdy wireless v2 response deadline')
      self.leases.append((lease, received_ns))
      self.last_received_ns = received_ns
      self.last_ack = {'sequence': sequence, 'rttMs': round(elapsed_ms, 2),
                       'receiverAgeMs': round(age / 1000, 2), 'displayStatus': status,
                       'ackPollDelayMs': round((time.monotonic_ns() - received_ns) / 1e6, 2)}
      # A stale display rejection still carries a new lease and independently
      # timed phone feedback. It is not a reason to disconnect a working radio.
      replies.append((age_feedback(value, elapsed_ms), received_ns / 1e9, elapsed_ms))
      if not self.pending:
        break
    return replies

  def _flush(self):
    # A short send can leave the socket writable. Finish it now instead of
    # adding a full display tick per fragment, but never spin on backpressure.
    for _ in range(MAX_WRITE_ATTEMPTS):
      if not self.tx:
        return
      if not 0 <= time.monotonic_ns() - self.tx_captured_ns < TTL_US * 1000:
        # Never complete a partially transmitted old frame or renew its age.
        raise TimeoutError('Navdy wireless v2 transmit deadline')
      try:
        sent = self.conn.send(self.tx)
      except BlockingIOError:
        self.blocked_writes += 1
        return
      if sent <= 0:
        raise ConnectionError('Navdy Bluetooth disconnected during write')
      if sent < len(self.tx):
        self.partial_writes += 1
      self.tx = self.tx[sent:]
      if not self.tx:
        self.last_tx_ms = round((time.monotonic_ns() - self.tx_captured_ns) / 1e6, 2)
        return

  def update(self, payload, *, captured_ns):
    if type(captured_ns) is not int:
      raise ValueError('Navdy snapshot capture timestamp is required')
    replies = self._read()
    now_ns = time.monotonic_ns()
    if self.pending and now_ns - self.pending[0][1] >= RESPONSE_TIMEOUT_NS:
      raise TimeoutError('Navdy wireless v2 response deadline')
    self._flush()
    if self.tx or len(self.pending) >= MAX_IN_FLIGHT:
      self.skipped += 1
      return replies
    if not 0 <= now_ns - captured_ns < TTL_US * 1000:
      raise ConnectionError('Navdy snapshot expired before transmission')
    if self.sequence >= 2147483647:
      raise ConnectionError('Navdy wireless session sequence exhausted')
    # ACKs can arrive during this poll, after this snapshot was captured. Use
    # the latest lease actually known BEFORE capture, never a negative delta.
    usable = [(lease, received) for lease, received in self.leases if received <= captured_ns]
    if not usable:
      raise ConnectionError('Navdy wireless snapshot predates its lease')
    lease, received = usable[-1]
    delta_us = (captured_ns - received) // 1000
    if delta_us > 10_000_000:
      raise ConnectionError('Navdy wireless session lease expired')
    if not isinstance(payload, dict) or payload.get('schema') != 'navdy.openpilot.v1':
      raise ValueError('Only Navdy display snapshots are supported')
    raw = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if not 1 <= len(raw) <= MAX_BYTES:
      raise ValueError('Navdy display snapshot exceeds bound')
    wire = zlib.compress(raw, 1)
    if len(wire) > MAX_BYTES:
      raise ValueError('Navdy compressed snapshot exceeds bound')
    self.tx = REQUEST.pack(b'NVQ2', self.sequence, lease, delta_us,
                           len(raw), len(wire), zlib.crc32(raw), 1) + wire
    self.tx_captured_ns = captured_ns
    self.pending.append((self.sequence, captured_ns))
    self.sequence += 1
    self._flush()
    return replies

  def diagnostics(self):
    now = time.monotonic_ns()
    return {'protocol': 2, **(self.last_ack or {}), 'inFlight': len(self.pending),
            'skippedSnapshots': self.skipped,
            'txPendingBytes': len(self.tx), 'lastTxMs': self.last_tx_ms,
            'txAgeMs': round((now - self.tx_captured_ns) / 1e6, 2) if self.tx else None,
            'partialWriteCount': self.partial_writes, 'blockedWriteCount': self.blocked_writes,
            'ackAgeMs': round((now - self.last_received_ns) / 1e6, 2) if self.last_received_ns is not None else None}

  def close(self):
    self.reader.close()
