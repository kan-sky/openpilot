#!/usr/bin/env python3
"""Latest-only compressed snapshot experiment. NEVER forwards to the real HUD."""
import argparse
import copy
import json
from pathlib import Path
import re
import socket
import struct
import threading
import time
import zlib

from client import check_offroad, percentile, receive

HELLO = struct.Struct('!4sIII')
REQUEST = struct.Struct('!4sIIqIIII')
RESPONSE = struct.Struct('!4sIIIqqII')
MAX_BYTES = 32768


class LatestState:
  """One unsent full-state snapshot; never use this for commands/events."""
  def __init__(self):
    self.condition = threading.Condition()
    self.pending = None
    self.produced = 0
    self.replaced = 0

  def put(self, value):
    with self.condition:
      self.replaced += self.pending is not None
      self.pending = value
      self.produced += 1
      self.condition.notify()

  def take(self, timeout):
    with self.condition:
      self.condition.wait_for(lambda: self.pending is not None, timeout)
      result, self.pending = self.pending, None
      return result


def encode(value, compressed):
  raw = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
  if not 1 <= len(raw) <= MAX_BYTES:
    raise ValueError('Snapshot exceeds bound')
  wire = zlib.compress(raw, 1) if compressed else raw
  if len(wire) > MAX_BYTES:
    raise ValueError('Compressed snapshot exceeds bound')
  return raw, wire


def decode(wire, raw_size, compressed):
  if not 1 <= raw_size <= MAX_BYTES or not 1 <= len(wire) <= MAX_BYTES:
    raise ValueError('Snapshot exceeds bound')
  if compressed:
    inflater = zlib.decompressobj()
    raw = inflater.decompress(wire, raw_size + 1)
    if not inflater.eof or inflater.unused_data or inflater.unconsumed_tail:
      raise ValueError('Incomplete or trailing compressed snapshot')
  else:
    raw = wire
  if len(raw) != raw_size:
    raise ValueError('Decompressed size mismatch')
  return raw


def measure(conn, fixture, *, seconds, hz, compressed, stall_every=0, stale_every=0, guard=check_offroad):
  guard()
  magic, lease, ttl_us, bound = HELLO.unpack(receive(conn, HELLO.size, time.monotonic() + 2))
  lease_received = time.monotonic_ns()
  if (magic, ttl_us, bound) != (b'HLS0', 500000, MAX_BYTES):
    raise ValueError('Wrong snapshot server')
  slot = LatestState()
  stop = threading.Event()
  started = time.monotonic()
  deadline = started + seconds
  producer_errors = []

  def produce():
    sequence = 0
    next_tick = started
    try:
      while not stop.is_set() and time.monotonic() < deadline:
        if stop.wait(max(0, next_tick - time.monotonic())) or time.monotonic() >= deadline:
          break
        value = copy.deepcopy(fixture)
        value['seq'] = sequence
        value['vEgoKph'] = round(60 + (sequence % 100) / 10, 1)
        slot.put((sequence, time.monotonic_ns(), value))
        sequence += 1
        next_tick += 1 / hz
        next_tick = max(next_tick, time.monotonic())
    except Exception as exc:
      producer_errors.append(str(exc))
      stop.set()

  producer = threading.Thread(target=produce, name='synthetic-snapshots')
  producer.start()
  rtts, ages, encode_ms, decode_ms, capture_to_ack = [], [], [], [], []
  fresh, stale, verified, local_expired, injected, expired_clears = 0, 0, 0, 0, 0, 0
  sent, received, raw_bytes, wire_bytes = 0, 0, 0, 0
  error = None
  try:
    while time.monotonic() < deadline:
      guard()
      if stall_every and verified and verified % stall_every == 0:
        stop.wait(0.8)
      if time.monotonic() >= deadline:
        break
      item = slot.take(min(0.2, max(0, deadline - time.monotonic())))
      if item is None:
        continue
      if time.monotonic() >= deadline:
        break
      sequence, captured, value = item
      if (time.monotonic_ns() - captured) / 1000 >= ttl_us:
        local_expired += 1
        continue
      intentional_stale = stale_every and (verified + 1) % stale_every == 0
      reported_capture = captured - (1000000000 if intentional_stale else 0)
      injected += bool(intentional_stale)
      encode_start = time.monotonic()
      raw, wire = encode(value, compressed)
      encode_ms.append((time.monotonic() - encode_start) * 1000)
      header = REQUEST.pack(b'HLS2', sequence, lease, (reported_capture - lease_received) // 1000,
                            len(raw), len(wire), zlib.crc32(raw), int(compressed))
      begin = time.monotonic()
      limit = begin + 2
      conn.settimeout(2)
      conn.sendall(header + wire)
      sent += len(header) + len(wire)
      fields = RESPONSE.unpack(receive(conn, RESPONSE.size, limit))
      marker, ack_seq, status, next_lease, age_us, unpack_us, wire_size, clears = fields
      if marker != b'HLA2' or ack_seq != sequence or next_lease != lease + 1 or wire_size != len(wire):
        raise ValueError('Snapshot acknowledgement mismatch')
      echo = receive(conn, wire_size, limit)
      lease_received = time.monotonic_ns()
      if decode(echo, len(raw), compressed) != raw:
        raise ValueError('Snapshot roundtrip changed bytes')
      received += RESPONSE.size + len(echo)
      if status not in (0, 1) or (intentional_stale and status != 1):
        raise ValueError('Freshness policy rejected sequence or accepted stale injection')
      if status == 0 and not 0 <= age_us < ttl_us:
        raise ValueError('Receiver applied expired state')
      lease = next_lease
      verified += 1
      fresh += status == 0
      stale += status == 1
      expired_clears = clears
      raw_bytes += len(raw)
      wire_bytes += len(wire)
      rtts.append((time.monotonic() - begin) * 1000)
      capture_to_ack.append((time.monotonic_ns() - captured) / 1000000)
      if not intentional_stale:
        ages.append(age_us / 1000)
      decode_ms.append(unpack_us / 1000)
  except (OSError, ValueError, RuntimeError, zlib.error) as exc:
    error = f'{type(exc).__name__}: {exc}'
  finally:
    stop.set()
    producer.join(timeout=2)
  if producer.is_alive() or producer_errors:
    error = f'Producer failed: {producer_errors}'
  elapsed = time.monotonic() - started
  def stats(values):
    return {'median': percentile(values, .5), 'p95': percentile(values, .95),
            'maximum': max(values, default=None)}
  return {'scope': 'Isolated latest-state sink, not HUD/nav integration', 'encoding': 'zlib1' if compressed else 'raw',
          'requested_seconds': seconds, 'elapsed_seconds': elapsed, 'requested_hz': hz,
          'produced': slot.produced, 'coalesced_unsent': slot.replaced, 'pending_at_end': int(slot.pending is not None),
          'max_pending_capacity': 1, 'verified': verified, 'fresh_applied': fresh, 'receiver_rejected': stale,
          'local_expired': local_expired, 'stale_injected': injected, 'receiver_expired_clears': expired_clears,
          'completed_hz': verified / elapsed, 'fresh_hz': fresh / elapsed,
          'rtt_ms': stats(rtts), 'capture_to_ack_ms': stats(capture_to_ack),
          'receiver_age_upper_ms': stats(ages), 'encode_ms': stats(encode_ms), 'decode_ms': stats(decode_ms),
          'mean_raw_bytes': raw_bytes / verified if verified else None,
          'mean_wire_bytes': wire_bytes / verified if verified else None,
          'sent_bytes': sent, 'received_bytes': received, 'error': error}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--peer', required=True)
  parser.add_argument('--channel', type=int, required=True)
  parser.add_argument('--fixture', type=Path, required=True)
  parser.add_argument('--seconds', type=float, default=60)
  parser.add_argument('--hz', type=float, default=5)
  parser.add_argument('--raw', action='store_true')
  parser.add_argument('--stall-every', type=int, default=0)
  parser.add_argument('--stale-every', type=int, default=0)
  args = parser.parse_args()
  if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.peer) or not 1 <= args.channel <= 30:
    parser.error('Exact paired target required')
  if not 1 <= args.seconds <= 300 or not 1 <= args.hz <= 20 or args.stall_every < 0 or args.stale_every < 0:
    parser.error('Trial bounds invalid')
  fixture = json.loads(args.fixture.read_text())
  if not isinstance(fixture, dict) or fixture.get('schema') != 'navdy.openpilot.v1':
    parser.error('Expected synthetic display fixture')
  encode(fixture, not args.raw)
  check_offroad()
  with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) as conn:
    conn.setsockopt(274, 4, bytes([2, 0]))
    conn.settimeout(15)
    connect_started = time.monotonic()
    conn.connect((args.peer, args.channel))
    connect_ms = (time.monotonic() - connect_started) * 1000
    if conn.getsockopt(274, 4, 2)[0] < 2:
      raise RuntimeError('Encrypted paired link required')
    result = measure(conn, fixture, seconds=args.seconds, hz=args.hz, compressed=not args.raw,
                     stall_every=args.stall_every, stale_every=args.stale_every)
    result.update(connect_ms=connect_ms, security_level=2)
    print(json.dumps(result, indent=2))
    if result['error'] or not result['verified']:
      raise SystemExit(1)


if __name__ == '__main__':
  main()
