"""Read NetworkManager secrets only in a low-priority, private USB worker."""
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

from wifi_protocol import HEADER, LIMIT, validate


def nm(*args):
  result = subprocess.run(['nmcli', '--escape', 'no', *args], capture_output=True,
                          text=True, timeout=2, env=dict(os.environ, LC_ALL='C'))
  if result.returncode:
    raise RuntimeError('NetworkManager query unavailable')
  return result.stdout.rstrip('\n')


def collect():
  active = set(nm('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines())
  rows = nm('-t', '-f', 'UUID,TYPE', 'connection', 'show').splitlines()
  ids = [row.split(':')[0] for row in rows if row.endswith(':802-11-wireless')]
  ids.sort(key=lambda uid: uid not in active)
  profiles = []
  for uid in ids[:8]:
    if not re.fullmatch('[0-9a-f-]{36}', uid):
      continue
    try:
      fields = nm('--show-secrets', '-g',
        '802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.hidden,802-11-wireless-security.key-mgmt,802-11-wireless-security.psk',
        'connection', 'show', 'uuid', uid).split('\n')
      # Empty trailing fields are meaningful; query output is normalized below.
      fields += [''] * (5 - len(fields))
      if len(fields) != 5:
        raise ValueError('Invalid Wi-Fi profile response')
      ssid, mode, hidden, security, password = fields
      if mode not in ('infrastructure', ''):
        continue  # Never import comma's access-point/tethering profile as a client.
      profile = dict(id=uid, ssid=ssid, hidden=hidden in ('yes', 'true'), active=uid in active,
                     security=security or 'open', password=password)
      validate(dict(version=1, owner='0' * 64, profiles=[profile]))
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired):
      # Preserve the current network if an old profile is unreadable; never
      # publish a fallback as active when the active client cannot be exported.
      if uid in active:
        raise RuntimeError('Active Wi-Fi profile unavailable') from None
      continue
    profiles.append(profile)
  if set(nm('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines()) != active:
    raise RuntimeError('Active network changed during Wi-Fi collection')
  return profiles


def road_state(raw):
  # Current typed Params returns bool; older installations returned bytes.
  if type(raw) is bool:
    return raw
  if raw in (b'0', '0'):
    return False
  if raw in (b'1', '1'):
    return True
  return None


def main(fd):
  sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
  from openpilot.common.params import Params
  params = Params()
  owner = hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()
  release = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/host_release.json'
  manifest = json.loads(release.read_text()) if release.is_file() else None
  channel = socket.socket(fileno=fd)
  channel.setblocking(False)
  profiles = None
  refresh = 0
  while True:
    now = time.monotonic()
    if now >= refresh:
      try:
        profiles = collect()
        refresh = now + 30
      except Exception:
        # Do not send an empty deletion set after an access/query failure.
        profiles = None
        refresh = now + 5
    if profiles is not None:
      raw = params.get('IsOnroad')
      onroad = road_state(raw)
      value = dict(version=1, owner=owner, profiles=profiles, onroad=onroad)
      if manifest is not None:
        value['jetson_release'] = manifest
      data = json.dumps(value, separators=(',', ':')).encode()
      if len(data) <= LIMIT:
        try:
          channel.send(HEADER.pack(time.monotonic()) + data)
        except BlockingIOError:
          pass
    time.sleep(2)


if __name__ == '__main__':
  main(int(sys.argv[1]))
