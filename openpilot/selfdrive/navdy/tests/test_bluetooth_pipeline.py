import heapq
import json
import random
import socket
import time
from types import SimpleNamespace as NS
import zlib

import pytest

from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy import bluetooth_pipeline as pipe

PAYLOAD = {'schema': 'navdy.openpilot.v1', 'active': True}


class Wire:
  def __init__(self):
    self.input = bytearray()
    self.output = bytearray()
    self.write_limit = 100000
    self.read_limit = 100000
    self.closed = False
    self.send_calls = 0

  def setblocking(self, value):
    assert value is False

  def recv(self, size):
    if not self.input:
      if self.closed:
        return b''
      raise BlockingIOError()
    count = min(size, len(self.input), self.read_limit)
    value = self.input[:count]
    del self.input[:count]
    return value

  def send(self, value):
    self.send_calls += 1
    if not self.write_limit:
      raise BlockingIOError()
    count = min(len(value), self.write_limit)
    self.output.extend(value[:count])
    return count

  def close(self):
    self.closed = True

  def frames(self):
    data, result = self.output, []
    while data:
      head = bt.REQUEST.unpack(data[:bt.REQUEST.size])
      size = bt.REQUEST.size + head[5]
      raw = bt.unpack_payload(bytes(data[bt.REQUEST.size:size]), head[4], head[7], head[6])
      result.append((head, json.loads(raw)))
      data = data[size:]
    return result

  def ack(self, sequence, *, status=0, age=100000, lease=None, magic=b'NVA2', raw=None, crc=None, size=None):
    raw = raw or b'{"navigationFeedback":{"sequence":7,"ageMs":10,"validForMs":1000}}'
    wire = zlib.compress(raw)
    self.input.extend(bt.RESPONSE.pack(magic, sequence, status, sequence + 1 if lease is None else lease, age,
                                      len(raw), len(wire) if size is None else size,
                                      zlib.crc32(raw) if crc is None else crc, 1) + wire)


@pytest.fixture
def link(monkeypatch):
  clock = NS(ns=10_000_000_000)
  monkeypatch.setattr(pipe.time, 'monotonic_ns', lambda: clock.ns)
  wire = Wire()
  state = pipe.SnapshotPipeline(wire, clock.ns, reader_factory=lambda conn: pipe.ResponseReader(conn, threaded=False))
  return clock, wire, state


def tick(link, seconds=0., **payload):
  clock, _, state = link
  clock.ns += round(seconds * 1e9)
  return state.update({**PAYLOAD, **payload}, captured_ns=clock.ns)


def test_two_snapshots_do_not_wait_for_previous_ack_and_never_accumulate(link):
  _, wire, state = link
  assert tick(link, seq=0) == []
  assert tick(link, .2, seq=1) == []
  assert len(state.pending) == 2
  assert tick(link, .2, seq=2) == []
  assert len(wire.frames()) == 2
  wire.ack(0)
  wire.ack(1)
  replies = tick(link, .2, seq=3)
  assert len(replies) == 2
  assert [f[1]['seq'] for f in wire.frames()] == [0, 1, 3]
  assert all(f[0][0] == b'NVQ2' for f in wire.frames())
  assert state.skipped == 1 and len(state.pending) == 1
  assert tick(link, .1, seq=4) == []


def test_ack_age_charges_original_capture_and_preserves_source_sequence(link):
  tick(link)
  _, wire, _ = link
  wire.ack(0)
  response, received, elapsed = tick(link, .7)[0]
  assert elapsed == 700 and received == 10.7
  assert response['navigationFeedback'] == {'sequence': 7, 'ageMs': 710., 'validForMs': 300}


@pytest.mark.parametrize('status,age', [(1, 600000), (3, 100000)])
def test_expired_display_or_usb_owner_does_not_disconnect_healthy_link(link, status, age):
  _, wire, state = link
  tick(link)
  wire.ack(0, status=status, age=age)
  assert len(tick(link, .7)) == 1
  assert state.diagnostics()['displayStatus'] == status
  assert not wire.closed and state.sequence == 2


@pytest.mark.parametrize('fault', [{'magic': b'NVA1'}, {'status': 2}, {'status': 4}, {'age': -1},
                                  {'age': bt.TTL_US}, {'lease': 9}, {'crc': 0}, {'size': bt.MAX_BYTES + 1}])
def test_invalid_ack_is_not_tolerated(link, fault):
  _, wire, _ = link
  tick(link)
  wire.ack(0, **fault)
  with pytest.raises(ValueError):
    tick(link, .2)


def test_unsolicited_and_out_of_order_acks_fail_closed(link):
  _, wire, _ = link
  tick(link)
  tick(link, .1)
  wire.ack(1)
  with pytest.raises(ValueError, match='mismatch'):
    tick(link, .1)


def test_replayed_ack_cannot_renew_lease(link):
  _, wire, _ = link
  tick(link)
  wire.ack(0)
  tick(link, .1)
  wire.ack(0)
  with pytest.raises(ValueError, match='mismatch'):
    tick(link, .1)


def test_split_header_and_body_are_bounded(link):
  _, wire, state = link
  tick(link)
  wire.ack(0)
  remainder = bytes(wire.input[7:])
  del wire.input[7:]
  assert tick(link, .1) == [] and len(state.reader.rx) == 7
  wire.input.extend(remainder)
  wire.read_limit = 1
  assert len(tick(link, .1)) == 1 and not state.reader.rx


@pytest.mark.parametrize('with_ack', [False, True])
def test_silent_or_too_late_response_has_absolute_deadline(link, with_ack):
  _, wire, _ = link
  tick(link)
  tick(link, .2)
  if with_ack:
    wire.ack(0)
  with pytest.raises(TimeoutError, match='response deadline'):
    tick(link, 1.3)


def test_write_backpressure_has_one_buffer_and_original_expiry(link):
  _, wire, state = link
  wire.write_limit = 1
  tick(link)
  assert len(state.pending) == 1 and state.tx
  wire.write_limit = 0
  tick(link, .2)
  assert len(state.pending) == 1
  with pytest.raises(TimeoutError, match='transmit deadline'):
    tick(link, .3)


def test_partial_write_is_completed_without_duplicate_bytes(link):
  _, wire, _ = link
  wire.write_limit = 7
  tick(link, seq=1)
  wire.write_limit = 100000
  tick(link, .1, seq=2)
  assert [f[1]['seq'] for f in wire.frames()] == [1, 2]


def test_ack_arriving_after_capture_cannot_be_its_lease(link):
  clock, wire, state = link
  tick(link)
  wire.ack(0)
  captured = clock.ns + 100_000_000
  clock.ns = captured + 10_000_000
  state.update(PAYLOAD, captured_ns=captured)
  head, _ = wire.frames()[-1]
  assert head[2] == 0 and head[3] == 100000


def test_lease_history_and_inflight_memory_remain_bounded(link):
  _, wire, state = link
  tick(link)
  for seq in range(100):
    wire.ack(seq)
    tick(link, .2)
    wire.output.clear()
  assert len(state.leases) == pipe.LEASE_HISTORY
  assert len(state.pending) == 1 and not state.reader.rx and not state.tx


def test_wrapper_closes_and_discards_pipeline_after_protocol_error(link):
  _, wire, state = link
  tick(link)
  client = bt.BluetoothTransport('54:ED:A3:00:48:23')
  client.conn, client.pipeline = wire, state
  wire.ack(0, crc=0)
  with pytest.raises(ValueError):
    client.exchange_batch(PAYLOAD, captured_ns=link[0].ns)
  assert client.conn is client.pipeline is None and wire.closed


@pytest.mark.parametrize('value', [None, 1.2, True, '10'])
def test_capture_timestamp_type_required(link, value):
  with pytest.raises(ValueError):
    link[2].update(PAYLOAD, captured_ns=value)


def test_future_and_expired_snapshots_rejected(link):
  clock, _, state = link
  for stamp in (clock.ns + 1, clock.ns - 500_000_000):
    with pytest.raises(ConnectionError, match='expired'):
      state.update(PAYLOAD, captured_ns=stamp)


def test_real_reader_timestamps_arrival_not_next_display_tick(link):
  clock, wire, _ = link
  left, right = socket.socketpair()
  state = pipe.SnapshotPipeline(left, clock.ns)
  try:
    state.update(PAYLOAD, captured_ns=clock.ns)
    right.settimeout(1)
    assert right.recv(32768)
    clock.ns += 150_000_000
    wire.ack(0)
    right.sendall(bytes(wire.input))
    deadline = time.monotonic() + 1
    while state.reader.frames.empty() and time.monotonic() < deadline:
      time.sleep(.001)
    assert not state.reader.frames.empty()
    clock.ns += 150_000_000
    response, received, elapsed = state.update(PAYLOAD, captured_ns=clock.ns)[0]
    assert received == 10.15 and elapsed == 150.
    assert response['navigationFeedback']['ageMs'] == 160.
    assert state.leases[-1][1] == 10_150_000_000
  finally:
    state.close()
    left.close()
    right.close()
  assert not state.reader.thread.is_alive()


def test_receiver_eof_propagates_and_thread_exits(link):
  clock, _, _ = link
  left, right = socket.socketpair()
  state = pipe.SnapshotPipeline(left, clock.ns)
  try:
    right.close()
    state.reader.thread.join(1)
    with pytest.raises(ConnectionError):
      state.update(PAYLOAD, captured_ns=clock.ns)
  finally:
    state.close()
    left.close()


def test_writable_short_sends_do_not_wait_for_200ms_display_ticks(link):
  clock, wire, state = link
  wire.write_limit = 16
  tick(link, detail='fresh state with vehicles and lanes')
  # The socket remains writable. A short successful send is not EAGAIN.
  assert not state.tx
  assert len(wire.frames()) == 1
  assert 1 < wire.send_calls <= pipe.MAX_WRITE_ATTEMPTS
  assert state.pending[0][1] == clock.ns
  assert state.diagnostics()['lastTxMs'] == 0


def test_short_send_loop_is_bounded_and_backpressure_does_not_spin(link):
  _, wire, state = link
  wire.write_limit = 1
  tick(link)
  assert wire.send_calls == pipe.MAX_WRITE_ATTEMPTS and state.tx
  wire.write_limit = 0
  calls = wire.send_calls
  tick(link, .2)
  assert wire.send_calls == calls + 1
  assert state.diagnostics()['blockedWriteCount'] == 1
  assert state.diagnostics()['partialWriteCount'] == pipe.MAX_WRITE_ATTEMPTS
  assert state.diagnostics()['txPendingBytes'] == len(state.tx)
  assert state.diagnostics()['txAgeMs'] == 200


def test_short_send_rechecks_original_expiry_before_every_write(link):
  clock, wire, state = link
  wire.write_limit = 1
  original_send = wire.send

  def delayed_send(data):
    sent = original_send(data)
    clock.ns += 250_000_000
    return sent

  wire.send = delayed_send
  with pytest.raises(TimeoutError, match='transmit deadline'):
    tick(link)
  assert wire.send_calls == 2
  assert len(wire.output) == 2
  assert state.tx_captured_ns == 10_000_000_000


def test_partial_write_keeps_fifo_and_drops_superseded_unsent_snapshots(link):
  _, wire, state = link
  wire.write_limit = 0
  tick(link, seq=0)
  tick(link, .1, seq=1)
  assert len(state.pending) == 1 and not wire.output
  wire.write_limit = 16
  tick(link, .1, seq=2)
  assert [f[1]['seq'] for f in wire.frames()] == [0, 2]
  assert [f[0][1] for f in wire.frames()] == [0, 1]
  assert len(state.pending) == pipe.MAX_IN_FLIGHT
  assert state.diagnostics()['lastTxMs'] == 0
  tick(link, .1, seq=3)
  assert [f[1]['seq'] for f in wire.frames()] == [0, 2]
  wire.ack(0)
  wire.ack(1)
  tick(link, .1, seq=4)
  assert [f[1]['seq'] for f in wire.frames()] == [0, 2, 4]
  assert [f[0][1] for f in wire.frames()] == [0, 1, 2]


def test_ack_poll_delay_is_observable_but_never_added_to_lease_or_feedback_age(link):
  clock, wire, state = link
  tick(link)
  clock.ns += 75_000_000
  wire.ack(0)
  state.reader._receive_available()
  clock.ns += 125_000_000
  replies = tick(link)
  assert replies[0][1:] == (10.075, 75.)
  assert replies[0][0]['navigationFeedback']['ageMs'] == 85.
  assert state.last_ack['ackPollDelayMs'] == 125.
  assert state.leases[-1] == (1, 10_075_000_000)
  assert wire.frames()[-1][0][2:4] == (1, 125000)


def test_successful_short_sends_preserve_capture_time_and_wire_bytes(link):
  clock, wire, state = link
  captured = clock.ns
  clock.ns += 150_000_000
  wire.write_limit = 16
  state.update(PAYLOAD, captured_ns=captured)
  assert len(state.pending) == 1 and not state.tx
  assert state.pending[0] == (0, captured)
  head, payload = wire.frames()[0]
  raw = json.dumps(PAYLOAD, separators=(',', ':')).encode()
  expected = zlib.compress(raw, 1)
  assert bytes(wire.output) == bt.REQUEST.pack(b'NVQ2', 0, 0, 0, len(raw), len(expected), zlib.crc32(raw), 1) + expected
  assert payload == PAYLOAD and head[3] == 0
  assert state.diagnostics()['lastTxMs'] == 150.


@pytest.mark.parametrize('comma_ppm,navdy_ppm', [(-500, -500), (-500, 500), (500, -500), (500, 500)])
def test_delayed_pipeline_preserves_bounded_leases_and_conservative_age(link, comma_ppm, navdy_ppm):
  clock, wire, state = link
  rng = random.Random(0x24A5)
  epoch = clock.ns
  navdy_offset = rng.randrange(1_000_000_000, 100_000_000_000)
  events, server_leases, receipts, captures = [], {}, {0: epoch}, {}
  completed = 0
  last_ack_at = -1

  def comma_ns(physical_ns):
    return epoch + physical_ns * (1_000_000 + comma_ppm) // 1_000_000

  def navdy_us(physical_ns):
    return navdy_offset + physical_ns * (1_000_000 + navdy_ppm) // 1_000_000_000

  server_leases[0] = navdy_us(-50_000_000)

  def advance(physical_ns):
    nonlocal completed, last_ack_at
    while events and events[0][0] <= physical_ns:
      at, kind, seq, data = heapq.heappop(events)
      clock.ns = comma_ns(at)
      if kind == 'request':
        head = data
        assert head[1] == completed
        assert head[2] in server_leases
        assert receipts[head[2]] <= comma_ns(captures[seq])
        elapsed = navdy_us(at) - server_leases[head[2]]
        # Mirror WirelessProtocol.ageUs, including its clock-rate allowance.
        age = elapsed - head[3] + 2000 + max(0, elapsed // 1000)
        assert age >= (at - captures[seq]) / 1000
        server_leases[seq + 1] = navdy_us(at)
        while len(server_leases) > pipe.LEASE_HISTORY:
          server_leases.pop(next(iter(server_leases)))
        last_ack_at = max(last_ack_at + 1, at + rng.randrange(1, 240_000_001))
        heapq.heappush(events, (last_ack_at, 'ack', seq, age))
        completed += 1
      else:
        wire.ack(seq, age=data, status=1 if data >= bt.TTL_US else 0)
        state.reader._receive_available()
        receipts[seq + 1] = clock.ns
        assert state.reader.frames.qsize() <= pipe.MAX_IN_FLIGHT
    clock.ns = comma_ns(physical_ns)

  # A single RFCOMM stream preserves request and ACK order even when latency
  # varies. Receipt can occur during payload construction after capture.
  last_request_at = -1
  for tick_index in range(15000):
    physical_capture = tick_index * 200_000_000
    advance(physical_capture)
    captured_ns = clock.ns
    sent_at = physical_capture + rng.randrange(0, 5_000_001)
    advance(sent_at)
    wire.write_limit = rng.randrange(16, 129)
    state.update({**PAYLOAD, 'seq': tick_index}, captured_ns=captured_ns)
    assert len(state.pending) <= pipe.MAX_IN_FLIGHT
    assert len(state.leases) <= pipe.LEASE_HISTORY
    assert not state.tx
    for head, _ in wire.frames():
      if captures:
        assert physical_capture - captures[head[1] - 1] >= 200_000_000
      captures[head[1]] = physical_capture
      request_at = max(last_request_at + 1, sent_at + rng.randrange(1, 240_000_001))
      last_request_at = request_at
      heapq.heappush(events, (request_at, 'request', head[1], head))
    wire.output.clear()
    if state.sequence >= 10000:
      break
  assert state.sequence == 10000
  advance(physical_capture + 1_000_000_000)
  state._read()
  assert completed == 10000 and not state.pending
  assert state.reader.frames.empty() and not state.reader.rx


@pytest.mark.parametrize('partial_bytes', [7, bt.RESPONSE.size + 1])
def test_reader_thread_closes_during_fragmented_ack(link, partial_bytes):
  clock, wire, _ = link
  left, right = socket.socketpair()
  state = pipe.SnapshotPipeline(left, clock.ns)
  try:
    state.update(PAYLOAD, captured_ns=clock.ns)
    right.settimeout(1)
    assert right.recv(32768)
    wire.ack(0)
    right.sendall(bytes(wire.input[:partial_bytes]))
    deadline = time.monotonic() + 1
    while len(state.reader.rx) < partial_bytes and time.monotonic() < deadline:
      time.sleep(.001)
    assert len(state.reader.rx) == partial_bytes
    state.close()
    assert not state.reader.thread.is_alive()
    assert state.reader.frames.empty()
  finally:
    state.close()
    left.close()
    right.close()


def test_expired_partial_send_closes_before_reconnect_without_reusing_state(link):
  clock, wire, state = link
  client = bt.BluetoothTransport('54:ED:A3:00:48:23')
  client.conn, client.pipeline = wire, state
  wire.write_limit = 1
  client.exchange_batch(PAYLOAD, captured_ns=clock.ns)
  old_bytes = bytes(wire.output)
  clock.ns += 500_000_000
  with pytest.raises(TimeoutError, match='transmit deadline'):
    client.exchange_batch(PAYLOAD, captured_ns=clock.ns)
  assert client.conn is client.pipeline is None
  assert wire.closed and bytes(wire.output) == old_bytes
  new_wire = Wire()
  new_state = pipe.SnapshotPipeline(new_wire, clock.ns, reader_factory=lambda conn: pipe.ResponseReader(conn, threaded=False))
  client.conn, client.pipeline = new_wire, new_state
  try:
    assert new_state.reader is not state.reader
    client.exchange_batch(PAYLOAD, captured_ns=clock.ns)
    assert new_wire.frames()[0][0][1:4] == (0, 0, 0)
    assert new_state.diagnostics()['partialWriteCount'] == 0
    assert new_state.pending == pipe.deque([(0, clock.ns)])
  finally:
    client.close()
