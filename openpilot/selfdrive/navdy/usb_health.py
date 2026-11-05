"""Navdy-only USB recovery, onroad or offroad. Never reset a host controller."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import select
import stat
import subprocess
import sys
import threading
import time

SYSFS = Path('/sys/bus/usb/devices')
PARAMS = Path('/data/params/d')
WAIT_SECONDS = 30.
RESET_COOLDOWN = 300.
MAX_RESETS = 2
USBDEVFS_RESET = 0x5514


def recovery_messages(sm, now):
  for service in ('deviceState', 'pandaStates'):
    if (not sm.valid[service] or not 0 <= now - sm.recv_time[service] < 1.5
        or not 0 <= now - sm.logMonoTime[service] / 1e9 < 1.5):
      return False
  pandas = sm['pandaStates']
  return bool(pandas) and all(str(p.pandaType) not in ('unknown', '0') for p in pandas)


def parked_messages(sm, now):
  return (recovery_messages(sm, now) and not sm['deviceState'].started
          and all(not p.ignitionLine and not p.ignitionCan for p in sm['pandaStates']))


class RecoveryGuard:
  """Each worker owns its subscriber; never consume the bridge's live messages."""
  messages_allowed = staticmethod(recovery_messages)

  def __init__(self):
    self.local = threading.local()

  def __call__(self):
    try:
      return self.check()
    except Exception:
      self.local.sm = None
      return False

  def check(self):
    from openpilot.cereal import messaging
    sm = getattr(self.local, 'sm', None)
    if sm is None:
      sm = self.local.sm = messaging.SubMaster(['deviceState', 'pandaStates'])
      deadline = time.monotonic() + 1.2
      while time.monotonic() < deadline and not all(sm.seen.values()):
        sm.update(100)
    sm.update(0)
    return self.messages_allowed(sm, time.monotonic())


class ParkedGuard(RecoveryGuard):
  """Stricter guard for installation checks, not runtime display recovery."""
  messages_allowed = staticmethod(parked_messages)


def navdy_devices(root=SYSFS):
  devices = []
  for path in root.glob('*'):
    try:
      if ((path / 'idVendor').read_text().strip(), (path / 'idProduct').read_text().strip(),
          (path / 'manufacturer').read_text().strip(), (path / 'product').read_text().strip()) != (
              '9886', '0001', 'Navdy', 'Display'):
        continue
      serial = (path / 'serial').read_text().strip()
      if not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', serial):
        continue
      bus, number = (int((path / key).read_text()) for key in ('busnum', 'devnum'))
      if not 1 <= bus <= 255 or not 1 <= number <= 127:
        continue
      devices.append({'path': path.name, 'serial': serial, 'bus': bus, 'number': number,
                      'identity': hashlib.sha256(serial.encode()).hexdigest()})
    except (OSError, ValueError):
      continue
  return devices


def reset_verified(path, identity, *, root=SYSFS, dev_root=Path('/dev/bus/usb'), params=PARAMS,
                   can_recover=None, offline=None, still_allowed=lambda: True):
  """Open the checked node before final checks, so address reuse cannot retarget the ioctl."""
  if not re.fullmatch(r'\d+-\d+(?:\.\d+)*', path):
    raise ValueError('Invalid USB device path')
  can_recover = can_recover or RecoveryGuard()
  offline = offline or offline_owners
  matches = [d for d in navdy_devices(root) if d['path'] == path and d['identity'] == identity]
  if (len(matches) != 1 or (params / 'IsOnroad').read_bytes().strip() not in (b'0', b'1') or not can_recover()
      or not offline(matches[0]['serial']) or not still_allowed()):
    raise ValueError('USB recovery identity or runtime guard failed')
  device = matches[0]
  node = dev_root / f"{device['bus']:03d}" / f"{device['number']:03d}"
  fd = os.open(node, os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC)
  try:
    info = os.fstat(fd)
    if (not stat.S_ISCHR(info.st_mode) or os.major(info.st_rdev) != 189
        or os.minor(info.st_rdev) != (device['bus'] - 1) * 128 + device['number'] - 1
        or device not in navdy_devices(root) or not offline(device['serial']) or not can_recover()
        or (params / 'IsOnroad').read_bytes().strip() not in (b'0', b'1') or not still_allowed()):
      raise ValueError('USB recovery device changed')
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
  finally:
    os.close(fd)


def offline_owners(serial, inventory=None):
  from openpilot.selfdrive.navdy.transport import adb_devices, existing_adb_devices
  inventory = inventory or existing_adb_devices
  states = []
  for port in (5038, 5037):
    try:
      states.extend((port, state) for target, state in adb_devices(inventory(port)) if target == serial)
    except ConnectionRefusedError:
      continue
    except (OSError, ValueError):
      return []
  return [port for port, _ in states] if states and all(state == 'offline' for _, state in states) else []


class UsbHealth:
  def __init__(self, state, can_recover, emit, *, inventory=navdy_devices, command=subprocess.run, clock=time.monotonic):
    self.state = Path(state)
    self.can_recover = can_recover
    self.emit = emit
    self.inventory = inventory
    self.command = command
    self.clock = clock
    self.failed_since = None
    self.identity = None
    self.verified_identity = None
    self.reconnected = False
    self.last_report = -float('inf')
    self.lock = threading.RLock()
    try:
      with self.saved_state() as (_, saved):
        self.verified_identity = saved.get('identity')
    except (OSError, ValueError):
      pass

  @contextmanager
  def saved_state(self):
    fd = os.open(self.state, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'r+') as handle:
      info = os.fstat(handle.fileno())
      if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_nlink != 1:
        raise ValueError('Invalid USB recovery state file')
      fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
      raw = handle.read(4097)
      saved = json.loads(raw) if raw else {'count': 0, 'last': 0.}
      if (not isinstance(saved, dict) or type(saved.get('count')) is not int or not 0 <= saved['count'] <= MAX_RESETS
          or type(saved.get('last')) not in (int, float) or not 0 <= saved['last'] <= self.clock()
          or ('identity' in saved and not re.fullmatch('[a-f0-9]{64}', str(saved['identity'])))):
        raise ValueError('Invalid USB recovery budget')
      yield handle, saved

  @staticmethod
  def save(handle, saved):
    handle.seek(0)
    handle.write(json.dumps(saved))
    handle.truncate()
    handle.flush()

  def verified(self, serial):
    with self.lock:
      devices = self.inventory()
      self.failed_since = None
      self.reconnected = False
      if len(devices) == 1 and devices[0]['serial'] == serial:
        self.verified_identity = devices[0]['identity']
        try:
          with self.saved_state() as (handle, saved):
            saved['identity'] = self.verified_identity
            self.save(handle, saved)
        except (OSError, ValueError):
          pass

  def record(self, action, now, count=0):
    if now - self.last_report >= 10 or action in ('reconnect', 'reset', 'reset_failed'):
      self.emit('usb_recovery', local_only=True, transport='usb', usbRecovery={
        'action': action, 'failureAge': max(0., now - (self.failed_since or now)), 'resetCount': count})
      self.last_report = now

  def unavailable(self, servers, transport):
    with self.lock:
      self._unavailable(servers, transport)

  def _unavailable(self, servers, transport):
    from openpilot.selfdrive.navdy.transport import adb_devices
    now = self.clock()
    devices = self.inventory()
    if len(devices) != 1:
      self.failed_since, self.identity = None, None
      self.record('absent_or_ambiguous', now)
      return
    device = devices[0]
    states = [(port, state) for port, raw in servers for serial, state in adb_devices(raw) if serial == device['serial']]
    if device['identity'] != self.identity:
      self.identity, self.failed_since, self.reconnected = device['identity'], now, False
    if self.failed_since is None:
      self.failed_since = now
    # Never reset an online device whose package/service query failed, or override authorization.
    if any(state != 'offline' for _, state in states):
      self.failed_since = now
      self.record('not_offline', now)
      return
    if not states:
      self.failed_since = now
      self.record('no_adb_owner', now)
      return
    if self.verified_identity != device['identity']:
      self.record('unverified', now)
      return
    if not transport.recovery_allowed() or not self.can_recover():
      self.record('guard_blocked', now)
      return
    elapsed = now - self.failed_since
    if elapsed < 10:
      self.record('waiting', now)
      return
    owners = offline_owners(device['serial'], transport.inventory)
    if owners != [port for port, _ in states]:
      self.failed_since = now
      self.record('changed', now)
      return
    if not self.can_recover() or not transport.recovery_allowed():
      self.record('guard_blocked', now)
      return
    if not self.reconnected and states:
      self.reconnected = True
      transport.server_port = owners[0]
      self.record('reconnect', now)
      try:
        transport.run(['reconnect'], serial=device['serial'])
      except (OSError, subprocess.SubprocessError):
        pass
      return
    if elapsed < WAIT_SECONDS:
      return
    # The shared lock and budget survive bridge restarts within this boot.
    try:
      with self.saved_state() as (handle, saved):
        count = saved['count']
        if count >= MAX_RESETS or (count and now - saved['last'] < RESET_COOLDOWN):
          self.record('budget', now, count)
          return
        if (offline_owners(device['serial'], transport.inventory) != owners
            or not self.can_recover() or self.inventory() != devices or not transport.recovery_allowed()):
          self.record('changed', now, count)
          return
        count += 1
        saved.update(count=count, last=now)
        self.save(handle, saved)
        self.record('reset', now, count)
        command = [sys.executable, '-m', 'openpilot.selfdrive.navdy.usb_health', '--reset', device['path'],
                   '--identity', device['identity']]
        cancel_fd = getattr(transport, 'recovery_fd', None)
        if cancel_fd is not None:
          command += ['--cancel-fd', str(cancel_fd)]
        result = self.command(command, pass_fds=() if cancel_fd is None else (cancel_fd,),
                              capture_output=True, timeout=7, check=False)
        if result.returncode:
          self.record('reset_failed', now, count)
    except (OSError, ValueError, subprocess.SubprocessError):
      self.record('reset_failed', now)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--reset', required=True)
  parser.add_argument('--identity', required=True)
  parser.add_argument('--cancel-fd', type=int)
  args = parser.parse_args()
  def allowed():
    from openpilot.selfdrive.navdy.bluetooth import configured_connection
    from openpilot.selfdrive.navdy.policy import read_transport_mode, transport_policy
    _, legacy_only = configured_connection()
    return (transport_policy(read_transport_mode(), legacy_only) not in ('bluetooth_only', 'invalid')
            and (args.cancel_fd is None or not select.select([args.cancel_fd], [], [], 0)[0]))
  reset_verified(args.reset, args.identity, still_allowed=allowed)
