#!/usr/bin/env python3
"""Reject one intentionally corrupt benchmark frame, then verify recovery."""
import argparse
import json
import re
import socket
import time

from client import check_offroad, measure, request_frame


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--peer', required=True)
  parser.add_argument('--channel', type=int, required=True)
  args = parser.parse_args()
  if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.peer) or not 1 <= args.channel <= 30:
    parser.error('Exact known benchmark target required')

  def connect():
    check_offroad()
    conn = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    try:
      conn.setsockopt(274, 4, bytes([2, 0]))
      conn.settimeout(15)
      conn.connect((args.peer, args.channel))
      if conn.getsockopt(274, 4, 2)[0] < 2:
        raise RuntimeError('Insufficient Bluetooth security')
      return conn
    except BaseException:
      conn.close()
      raise

  with connect() as conn:
    frame = bytearray(request_frame(0, 128, 128))
    frame[-1] ^= 1
    conn.settimeout(2)
    conn.sendall(frame)
    try:
      closed = conn.recv(1) == b''
    except ConnectionResetError:
      closed = True
    if not closed:
      raise RuntimeError('Corrupt benchmark frame accepted')
  attempts = []
  recovered = None
  for delay in (0, 0.5, 1, 2):
    time.sleep(delay)
    started = time.monotonic()
    try:
      conn = connect()
    except OSError as exc:
      attempts.append({'delay_seconds': delay, 'connect_ms': (time.monotonic() - started) * 1000,
                       'error': str(exc)})
      continue
    attempts.append({'delay_seconds': delay, 'connect_ms': (time.monotonic() - started) * 1000,
                     'error': None})
    with conn:
      recovered = measure(conn, seconds=5, hz=5, tx=1200, rx=1000)
    break
  print(json.dumps({'intentional_corruption_rejected': closed, 'reconnect_attempts': attempts,
                    'recovery': recovered}, indent=2))
  if recovered is None:
    raise SystemExit(1)
  if recovered['error'] or not recovered['verified_exchanges']:
    raise SystemExit(1)


if __name__ == '__main__':
  main()
