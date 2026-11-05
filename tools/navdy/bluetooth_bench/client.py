#!/usr/bin/env python3
"""Bounded secure RFCOMM benchmark. No vehicle/nav messages or pairing actions."""
import argparse
import json
import math
from pathlib import Path
import re
import socket
import struct
import time
import zlib

HEADER = struct.Struct('!4sIIII')
MAX_BYTES = 32768
UUID = 'b910e2b5-1b15-4b3d-88a0-4d803ed0f7ce'


def pattern(sequence, size):
  result = bytearray(size)
  for i in range(size):
    sequence = (sequence * 1664525 + 1013904223) & 0xffffffff
    result[i] = sequence >> 24
  return bytes(result)


def receive(conn, size, deadline):
  result = bytearray()
  while len(result) < size:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
      raise TimeoutError('Frame deadline exceeded')
    conn.settimeout(remaining)
    chunk = conn.recv(size - len(result))
    if not chunk:
      raise ConnectionError('Peer closed benchmark socket')
    result.extend(chunk)
  return bytes(result)


def request_frame(sequence, tx, rx):
  if not 0 <= sequence < 2**31 or not HEADER.size <= tx <= MAX_BYTES or not HEADER.size <= rx <= MAX_BYTES:
    raise ValueError('Invalid frame size or sequence')
  body = pattern(sequence, tx - HEADER.size)
  return HEADER.pack(b'HLB1', sequence, len(body), rx - HEADER.size, zlib.crc32(body)) + body


def response_body(header, sequence, tx, rx):
  magic, actual_seq, size, request_size, crc = HEADER.unpack(header)
  if magic != b'HLR1' or actual_seq != sequence or size != rx - HEADER.size or request_size != tx - HEADER.size:
    raise ValueError('Response header/sequence mismatch')
  return size, crc


def check_offroad():
  for key in ('IsOnroad', 'IsEngaged'):
    if (Path('/data/params/d') / key).read_text().strip() != '0':
      raise RuntimeError('Benchmark is offroad-only')


def percentile(values, q):
  return sorted(values)[max(0, math.ceil(len(values) * q) - 1)] if values else None


def measure(conn, *, seconds, hz, tx, rx, guard=check_offroad):
  guard()
  started = time.monotonic()
  next_tick = started
  latencies = []
  missed_slots = 0
  attempts = 0
  error = None
  wire_tx = 0
  wire_rx = 0
  try:
    while time.monotonic() - started < seconds:
      if hz and time.monotonic() < next_tick:
        time.sleep(next_tick - time.monotonic())
      if time.monotonic() - started >= seconds:
        break
      guard()
      sequence = len(latencies)
      request = request_frame(sequence, tx, rx)
      expected = pattern(sequence ^ 0x5a5a5a5a, rx - HEADER.size)
      began = time.monotonic()
      deadline = began + 2
      attempts += 1
      conn.settimeout(2)
      conn.sendall(request)
      wire_tx += len(request)
      header = receive(conn, HEADER.size, deadline)
      size, crc = response_body(header, sequence, tx, rx)
      body = receive(conn, size, deadline)
      wire_rx += HEADER.size + len(body)
      if zlib.crc32(body) != crc or body != expected:
        raise ValueError('Corrupt response')
      latencies.append((time.monotonic() - began) * 1000)
      if hz:
        next_tick += 1 / hz
        now = time.monotonic()
        if now > next_tick:
          skipped = math.floor((now - next_tick) * hz) + 1
          missed_slots += skipped
          next_tick += skipped / hz
  except (OSError, ValueError, RuntimeError) as exc:
    error = f'{type(exc).__name__}: {exc}'
  elapsed = time.monotonic() - started
  return {
    'transport': 'bluetooth_rfcomm', 'uuid': UUID, 'requested_hz': hz,
    'requested_seconds': seconds, 'elapsed_seconds': elapsed,
    'tx_frame_bytes': tx, 'rx_frame_bytes': rx, 'attempts': attempts,
    'verified_exchanges': len(latencies), 'missed_schedule_slots': missed_slots,
    'rtt_p50_ms': percentile(latencies, .5), 'rtt_p95_ms': percentile(latencies, .95),
    'rtt_p99_ms': percentile(latencies, .99), 'rtt_max_ms': max(latencies, default=None),
    'rtt_over_200ms': sum(value > 200 for value in latencies),
    'completed_hz': len(latencies) / elapsed,
    'verified_payload_bytes_per_second': len(latencies) * (tx + rx) / elapsed,
    'sent_bytes': wire_tx, 'received_bytes': wire_rx, 'error': error,
    'scope': 'Synthetic bidirectional radio traffic, not HUD integration or road validation',
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--peer', required=True)
  parser.add_argument('--channel', required=True, type=int)
  parser.add_argument('--seconds', type=float, default=60)
  parser.add_argument('--hz', type=float, default=5, help='0 = bounded saturation trial')
  parser.add_argument('--tx', type=int, default=2409, help='Total request frame bytes')
  parser.add_argument('--rx', type=int, default=1932, help='Total response frame bytes')
  args = parser.parse_args()
  if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.peer) or not 1 <= args.channel <= 30:
    parser.error('Exact Bluetooth peer address and RFCOMM channel required')
  if not 1 <= args.seconds <= 300 or not 0 <= args.hz <= 20:
    parser.error('Use 1..300 seconds and 0..20 Hz')
  request_frame(0, args.tx, args.rx)
  check_offroad()
  with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) as conn:
    # Linux SOL_BLUETOOTH/BT_SECURITY: require encrypted, authenticated pairing.
    conn.setsockopt(274, 4, bytes([2, 0]))
    conn.settimeout(15)
    started = time.monotonic()
    conn.connect((args.peer, args.channel))
    connect_ms = (time.monotonic() - started) * 1000
    security = conn.getsockopt(274, 4, 2)
    if security[0] < 2:
      raise RuntimeError('Bluetooth link security below required level')
    result = measure(conn, seconds=args.seconds, hz=args.hz, tx=args.tx, rx=args.rx)
    result.update(connect_ms=connect_ms, security_level=security[0])
    print(json.dumps(result, indent=2))
    if result['error']:
      raise SystemExit(1)


if __name__ == '__main__':
  main()
