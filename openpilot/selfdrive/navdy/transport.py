"""USB ADB socket transport. Also runnable on a paired Jetson over SSH stdio."""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import socket
import subprocess
import sys
import time

MAX_FRAME_BYTES = 32768
ADB_PORT = 5038
ADB_DISCOVERY_TIMEOUT = 15
ADB_COMMAND_TIMEOUT = 3
SERVICE = 'com.navdy.hud.app/.openpilot.OpenpilotStateService'


def existing_adb_devices(port):
  """Query only an existing local server; unlike `adb devices`, never start one."""
  deadline = time.monotonic() + 0.4
  with socket.create_connection(('127.0.0.1', port), timeout=0.4) as conn:
    def read(size):
      result = b''
      while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          raise TimeoutError('ADB inventory deadline')
        conn.settimeout(remaining)
        chunk = conn.recv(size - len(result))
        if not chunk:
          raise ConnectionError('ADB inventory closed')
        result += chunk
      return result

    request = b'host:devices-l'
    conn.sendall(f'{len(request):04x}'.encode() + request)
    if read(4) != b'OKAY':
      raise ConnectionError('ADB inventory rejected')
    size = int(read(4), 16)
    if size > 32768:
      raise ValueError('ADB inventory too large')
    return read(size).decode('utf-8')


def adb_devices(raw):
  return [(fields[0], fields[1]) for line in raw.splitlines()
          if len(fields := line.split()) >= 2 and re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', fields[0])
          and fields[1] in ('device', 'offline', 'unauthorized', 'no')]


def parse_response(raw):
  def invalid_constant(value):
    raise ValueError('Non-finite Navdy response')

  try:
    value = json.loads(raw.decode('utf-8'), parse_constant=invalid_constant)
  except RecursionError as error:
    raise ValueError('Navdy response nesting is too deep') from error
  if not isinstance(value, dict):
    raise ValueError('Navdy response is not an object')
  pending = [(value, 0)]
  while pending:
    item, depth = pending.pop()
    if depth > 32:
      raise ValueError('Navdy response nesting is too deep')
    if isinstance(item, dict):
      pending.extend((child, depth + 1) for child in item.values())
    elif isinstance(item, list):
      pending.extend((child, depth + 1) for child in item)
    elif isinstance(item, float) and not math.isfinite(item):
      raise ValueError('Non-finite Navdy response')
  if not any(key in value for key in ('roadPreview', 'navigationFeedback', 'cameraSpeedKph')):
    raise ValueError('Navdy service returned an unsupported response')
  return value


def encode_frame(payload):
  encoded = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8') + b'\n'
  if len(encoded) > MAX_FRAME_BYTES:
    raise ValueError('Navdy frame exceeds 32 KiB')
  return encoded


class LineReader:
  def __init__(self):
    self.buffer = b''

  def receive(self, conn):
    while b'\n' not in self.buffer:
      chunk = conn.recv(min(4096, MAX_FRAME_BYTES + 1 - len(self.buffer)))
      if not chunk:
        raise ConnectionError('Navdy closed its socket')
      self.buffer += chunk
      boundary = self.buffer.find(b'\n')
      if (boundary < 0 and len(self.buffer) > MAX_FRAME_BYTES) or boundary >= MAX_FRAME_BYTES:
        raise ValueError('Oversized Navdy response')
    line, self.buffer = self.buffer.split(b'\n', 1)
    return parse_response(line)


class AdbTransport:
  def __init__(self, adb=None, command=subprocess.run, inventory=None):
    self.adb = adb or shutil.which('adb')
    self.command = command
    self.inventory = inventory or existing_adb_devices
    self.server_port = ADB_PORT
    self.usb_health = None
    self.recovery_allowed = lambda: True
    self.recovery_fd = None
    self.serial = None
    self.port = None
    self.conn = None
    self.reader = LineReader()

  def run(self, args, *, serial=None):
    if not self.adb:
      raise FileNotFoundError('ADB is not installed')
    target = serial or self.serial
    cmd = [self.adb, '-P', str(self.server_port)]
    if target:
      cmd += ['-s', target]
    # Discovery can start the dedicated server; cold startup exceeds 3 s on AGNOS.
    timeout = ADB_DISCOVERY_TIMEOUT if args == ['devices', '-l'] else ADB_COMMAND_TIMEOUT
    result = self.command(cmd + args, capture_output=True, text=True, timeout=timeout, check=True)
    return result.stdout.strip()

  def discover(self):
    servers = []
    unknown_server = False
    for port in (ADB_PORT, 5037):
      try:
        servers.append((port, self.inventory(port)))
      except ConnectionRefusedError:
        pass
      except (OSError, ValueError):
        unknown_server = True
    if not servers:
      if unknown_server:
        raise ConnectionError('ADB inventory unavailable')
      self.server_port = ADB_PORT
      servers = [(ADB_PORT, self.run(['devices', '-l']))]
    candidates = []
    for port, raw in servers:
      self.server_port = port
      for serial, state in adb_devices(raw):
        if state != 'device':
          continue
        # Verify the installed HUD package; never start services on an arbitrary phone.
        try:
          package = self.run(['shell', 'pm', 'path', 'com.navdy.hud.app'], serial=serial)
          if package.startswith('package:'):
            candidates.append((serial, port))
        except (OSError, subprocess.SubprocessError):
          continue
    if len({serial for serial, _ in candidates}) == 1:
      serial, self.server_port = candidates[0]
      if self.usb_health is not None:
        self.usb_health.verified(serial)
      return serial
    if not candidates and not unknown_server and self.usb_health is not None:
      self.usb_health.unavailable(servers, self)
    raise ConnectionError('Navdy USB not found' if not candidates else 'Multiple Navdy USB devices')

  def connect(self):
    self.serial = self.discover()
    try:
      # Allocate our own loopback port, without replacing another bridge's mapping.
      port = self.run(['forward', 'tcp:0', 'tcp:8765'])
      if not port.isdigit() or not 1024 <= int(port) <= 65535:
        raise ConnectionError('ADB did not allocate a Navdy forward port')
      self.port = int(port)
      result = self.run(['shell', 'am', 'startservice', '-n', SERVICE])
      if 'Error' in result or 'Exception' in result:
        raise ConnectionError('Navdy service failed to start')
      self.conn = socket.create_connection(('127.0.0.1', self.port), timeout=0.8)
      self.conn.settimeout(0.8)
      self.reader = LineReader()
    except Exception:
      self.close()
      raise

  def exchange(self, payload):
    if self.conn is None:
      self.connect()
    self.conn.sendall(encode_frame(payload))
    return self.reader.receive(self.conn)

  def close(self):
    if self.conn is not None:
      self.conn.close()
      self.conn = None
    if self.port is not None:
      try:
        self.run(['forward', '--remove', f'tcp:{self.port}'])
      except (OSError, subprocess.SubprocessError):
        pass
    self.port = None
    self.serial = None


def relay_stdio():
  """No shell, privileged USB reconfiguration, Bluetooth reset or vehicle writes."""
  transport = AdbTransport()
  try:
    while True:
      line = sys.stdin.buffer.readline(MAX_FRAME_BYTES + 1)
      if not line:
        return
      if len(line) > MAX_FRAME_BYTES or not line.endswith(b'\n'):
        raise ValueError('Invalid relay frame')
      payload = json.loads(line)
      if not isinstance(payload, dict) or payload.get('schema') != 'navdy.openpilot.v1':
        raise ValueError('Unsupported display payload')
      sys.stdout.buffer.write(encode_frame(transport.exchange(payload)))
      sys.stdout.buffer.flush()
  finally:
    transport.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--stdio', action='store_true')
  if parser.parse_args().stdio:
    relay_stdio()
