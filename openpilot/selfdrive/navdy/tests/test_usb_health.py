import json
import os
from pathlib import Path
import stat
import subprocess
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import usb_health as usb


class RuntimeMessages:
  def __init__(self, started=False, ignition=False):
    self.valid = dict.fromkeys(('deviceState', 'pandaStates'), True)
    self.recv_time = dict.fromkeys(self.valid, 100.)
    self.logMonoTime = dict.fromkeys(self.valid, 100_000_000_000)
    self.seen = dict.fromkeys(self.valid, True)
    self.data = {'deviceState': NS(started=started),
                 'pandaStates': [NS(pandaType='cuatro', ignitionLine=ignition, ignitionCan=ignition)]}

  def __getitem__(self, name):
    return self.data[name]

  def update(self, timeout):
    pass


@pytest.mark.parametrize('started,ignition', [(False, False), (False, True), (True, True), (True, False)])
def test_runtime_recovery_and_installation_guards_have_separate_policies(monkeypatch, started, ignition):
  import sys
  sm = RuntimeMessages(started, ignition)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', NS(messaging=NS(SubMaster=lambda _: sm)))
  monkeypatch.setattr(usb.time, 'monotonic', lambda: 100.)
  assert usb.RecoveryGuard()()
  assert usb.ParkedGuard()() is (not started and not ignition)


@pytest.mark.parametrize('service', ['deviceState', 'pandaStates'])
@pytest.mark.parametrize('field', ['recv_time', 'logMonoTime'])
@pytest.mark.parametrize('age', [1.5, 5., -1., float('nan')])
def test_onroad_recovery_still_rejects_stale_or_invalid_time(service, field, age):
  sm = RuntimeMessages(True, True)
  getattr(sm, field)[service] = (100. - age) * (1e9 if field == 'logMonoTime' else 1.)
  assert not usb.recovery_messages(sm, 100.)


@pytest.fixture
def device(tmp_path):
  root = tmp_path / 'sysfs'
  path = root / '3-1'
  path.mkdir(parents=True)
  fields = {'idVendor': '9886', 'idProduct': '0001', 'manufacturer': 'Navdy', 'product': 'Display',
            'serial': 'TEST-NAVDY', 'busnum': '3', 'devnum': '2'}
  for name, value in fields.items():
    (path / name).write_text(value)
  return NS(root=root, path=path, info=usb.navdy_devices(root)[0])


@pytest.fixture
def recovery(tmp_path, device):
  state = NS(now=100., allowed=True, calls=[], events=[], devices=[device.info])
  state.transport = NS(server_port=5038, run=lambda args, **kw: state.calls.append(('adb', args, kw)),
                       recovery_allowed=lambda: True, inventory=lambda port: dict(state.servers).get(port, ''))
  state.health = usb.UsbHealth(tmp_path / 'budget.json', lambda: state.allowed,
                             lambda *args, **kw: state.events.append((args, kw)),
                             inventory=lambda: state.devices, clock=lambda: state.now,
                             command=lambda args, **kw: state.calls.append(('reset', args, kw)) or NS(returncode=0))
  state.servers = [(5037, 'TEST-NAVDY offline usb:3-1')]
  state.health.verified('TEST-NAVDY')
  state.fail = lambda: state.health.unavailable(state.servers, state.transport)
  return state


def test_stuck_offline_reconnects_owner_then_one_targeted_reset(recovery):
  r = recovery
  r.fail()
  r.now = 110.
  r.fail()
  assert r.calls == [('adb', ['reconnect'], {'serial': 'TEST-NAVDY'})]
  assert r.transport.server_port == 5037
  r.now = 129.9
  r.fail()
  assert len(r.calls) == 1
  r.now = 130.
  r.fail()
  assert len(r.calls) == 2 and r.calls[1][0] == 'reset'
  assert r.calls[1][1][-4:] == ['--reset', '3-1', '--identity', r.devices[0]['identity']]
  assert r.calls[1][2]['timeout'] == 7
  assert json.loads(r.health.state.read_text()) == {'count': 1, 'last': 130., 'identity': r.devices[0]['identity']}


def test_onroad_verified_offline_runs_recovery_without_ignition_transition(recovery):
  r = recovery
  sm = RuntimeMessages(True, True)
  r.health.can_recover = lambda: usb.recovery_messages(sm, r.now)
  for r.now in (100., 110., 130.):
    sm.recv_time = dict.fromkeys(sm.valid, r.now)
    sm.logMonoTime = dict.fromkeys(sm.valid, int(r.now * 1e9))
    r.fail()
  assert [c[0] for c in r.calls] == ['adb', 'reset']
  assert sm['deviceState'].started and sm['pandaStates'][0].ignitionLine


@pytest.mark.parametrize('states', ['device', 'unauthorized', 'no', 'missing'])
def test_online_unauthorized_or_unknown_owner_never_reset(recovery, states):
  r = recovery
  r.servers = [(5038, '' if states == 'missing' else 'TEST-NAVDY ' + states)]
  for r.now in (100., 120., 180., 500.):
    r.fail()
  assert not r.calls


def test_failed_runtime_guard_never_reconnects_or_resets(recovery):
  r = recovery
  r.allowed = False
  for r.now in (100., 120., 500.):
    r.fail()
  assert not r.calls


def test_unverified_device_never_reset(recovery):
  r = recovery
  r.health.verified_identity = None
  for r.now in (100., 120., 500.):
    r.fail()
  assert not r.calls


@pytest.mark.parametrize('devices', [[], [{}, {}]])
def test_missing_or_multiple_devices_never_reset(recovery, devices):
  recovery.devices = devices
  recovery.fail()
  assert not recovery.calls


def test_healthy_reconnection_resets_continuous_failure_timer_not_budget(recovery):
  r = recovery
  r.fail()
  r.now = 110.
  r.fail()
  r.now = 130.
  r.fail()
  r.health.verified('TEST-NAVDY')
  r.now = 140.
  r.fail()
  assert r.health.failed_since == 140.
  assert json.loads(r.health.state.read_text())['count'] == 1


def test_reset_budget_survives_new_monitor_and_stops_storm(recovery):
  r = recovery
  for r.now in (100., 110., 130., 200., 429., 430., 800., 2000.):
    r.fail()
  assert sum(c[0] == 'reset' for c in r.calls) == 2
  original = r.health
  r.health = usb.UsbHealth(original.state, lambda: True, original.emit, inventory=original.inventory,
                           command=original.command, clock=original.clock)
  assert r.health.verified_identity == r.devices[0]['identity']
  for r.now in (2100., 2110., 2140.):
    r.fail()
  assert sum(c[0] == 'reset' for c in r.calls) == 2


@pytest.mark.parametrize('raw', ['broken', '[]', '{"count":-1,"last":0}', '{"count":0,"last":NaN}',
                                  '{"count":0,"last":99999}', '{"count":true,"last":0}'])
def test_invalid_budget_fails_closed(recovery, raw):
  r = recovery
  r.health.state.write_text(raw)
  for r.now in (100., 110., 130.):
    r.fail()
  assert not any(c[0] == 'reset' for c in r.calls)


def test_budget_symlink_does_not_write_or_reset(recovery, tmp_path):
  r = recovery
  outside = tmp_path / 'outside'
  outside.write_text('keep')
  r.health.state.unlink()
  r.health.state.symlink_to(outside)
  for r.now in (100., 110., 130.):
    r.fail()
  assert outside.read_text() == 'keep'
  assert not any(c[0] == 'reset' for c in r.calls)


def test_failed_reset_consumes_budget(recovery):
  r = recovery
  def timeout(*args, **kwargs):
    r.calls.append(('reset',))
    raise subprocess.TimeoutExpired('bounded-reset', 5)
  r.health.command = timeout
  for r.now in (100., 110., 130., 140.):
    r.fail()
  assert sum(c[0] == 'reset' for c in r.calls) == 1
  assert json.loads(r.health.state.read_text())['count'] == 1


@pytest.mark.parametrize('field,value', [('idVendor', '18d1'), ('idProduct', '0002'), ('manufacturer', 'Other'),
                                       ('product', 'Phone'), ('serial', 'host:5555'), ('busnum', '0'), ('devnum', '128')])
def test_only_exact_navdy_identity_is_eligible(device, field, value):
  (device.path / field).write_text(value)
  assert not usb.navdy_devices(device.root)


@pytest.mark.parametrize('onroad,after_open', [('0', '0'), ('1', '1'), ('0', '1'), ('1', '0')])
def test_reset_opens_exact_device_and_checks_identity_again(monkeypatch, tmp_path, device, onroad, after_open):
  params = tmp_path / 'params'
  params.mkdir()
  (params / 'IsOnroad').write_text(onroad)
  calls = []
  real_fstat = usb.os.fstat
  def opened(path, flags):
    calls.append(('open', str(path)))
    (params / 'IsOnroad').write_text(after_open)
    return 99
  sm = RuntimeMessages(onroad == '1', onroad == '1')
  monkeypatch.setattr(usb, 'RecoveryGuard', lambda: lambda: usb.recovery_messages(sm, 100.))
  monkeypatch.setattr(usb.os, 'open', opened)
  monkeypatch.setattr(usb.os, 'fstat', lambda fd: NS(st_mode=stat.S_IFCHR, st_rdev=os.makedev(189, 257))
                      if fd == 99 else real_fstat(fd))
  monkeypatch.setattr(usb.os, 'close', lambda fd: calls.append(('close', fd)))
  monkeypatch.setattr(usb.fcntl, 'ioctl', lambda *args: calls.append(('reset', args)))
  usb.reset_verified('3-1', device.info['identity'], root=device.root, params=params,
                     offline=lambda _: [5038])
  assert calls == [('open', '/dev/bus/usb/003/002'), ('reset', (99, 0x5514, 0)), ('close', 99)]


@pytest.mark.parametrize('change', ['unknown_state', 'identity', 'node', 'stale_state', 'online', 'cancelled'])
def test_reset_race_fails_closed(monkeypatch, tmp_path, device, change):
  params = tmp_path / 'params'
  params.mkdir()
  (params / 'IsOnroad').write_text('0')
  real_fstat = usb.os.fstat
  state = NS(fresh=True, offline=True, allowed=True)
  def opened(*args):
    if change == 'unknown_state':
      (params / 'IsOnroad').write_text('invalid')
    elif change == 'identity':
      (device.path / 'serial').write_text('REPLACED')
    elif change == 'stale_state':
      state.fresh = False
    elif change == 'online':
      state.offline = False
    elif change == 'cancelled':
      state.allowed = False
    return 99
  monkeypatch.setattr(usb.os, 'open', opened)
  monkeypatch.setattr(usb.os, 'fstat', lambda fd: NS(st_mode=stat.S_IFCHR,
                                                  st_rdev=os.makedev(189, 258 if change == 'node' else 257))
                      if fd == 99 else real_fstat(fd))
  monkeypatch.setattr(usb.os, 'close', lambda fd: None)
  monkeypatch.setattr(usb.fcntl, 'ioctl', lambda *args: pytest.fail('must not reset'))
  with pytest.raises(ValueError):
    usb.reset_verified('3-1', device.info['identity'], root=device.root, params=params,
                       can_recover=lambda: state.fresh, offline=lambda _: [5038] if state.offline else [],
                       still_allowed=lambda: state.allowed)


@pytest.mark.parametrize('new', ['online', 'owner', 'unknown', 'stopped'])
def test_recovery_rechecks_latest_adb_and_probe_state(recovery, new):
  r = recovery
  for r.now in (100., 110.):
    r.fail()
  before = len(r.calls)
  if new == 'unknown':
    def unknown(_):
      raise TimeoutError('unavailable')
    r.transport.inventory = unknown
  elif new == 'stopped':
    r.transport.recovery_allowed = lambda: False
  else:
    r.transport.inventory = lambda port: 'TEST-NAVDY device' if new == 'online' else ('TEST-NAVDY offline' if port == 5038 else '')
  r.now = 140.
  r.fail()
  assert len(r.calls) == before


def test_verified_identity_survives_restart_while_offline(recovery):
  r = recovery
  old = r.health
  r.health = usb.UsbHealth(old.state, old.can_recover, old.emit, inventory=old.inventory,
                           clock=old.clock, command=old.command)
  for r.now in (100., 110., 130.):
    r.fail()
  assert sum(c[0] == 'reset' for c in r.calls) == 1


@pytest.mark.parametrize('during', ['reconnect', 'reset'])
def test_probe_cancelled_during_guard_cannot_dispatch(recovery, during):
  r = recovery
  r.fail()
  if during == 'reset':
    r.now = 110.
    r.fail()
  before = len(r.calls)
  def guard():
    r.transport.recovery_allowed = lambda: False
    return True
  r.health.can_recover = guard
  r.now = 140.
  r.fail()
  assert len(r.calls) == before


@pytest.mark.parametrize('failure', ['init', 'update'])
@pytest.mark.parametrize('guard_type', [usb.RecoveryGuard, usb.ParkedGuard])
def test_guard_ipc_failure_is_closed_and_discards_subscriber(monkeypatch, failure, guard_type):
  import sys
  def fail(*args):
    raise RuntimeError('IPC unavailable')
  fake = NS(seen={'deviceState':False}, update=fail)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', NS(messaging=NS(SubMaster=fail if failure == 'init' else lambda _:fake)))
  guard = guard_type()
  assert not guard()
  assert guard.local.sm is None


def test_probe_close_cancels_child_pipe_and_retries_do_not_leak():
  import select
  import threading
  from openpilot.selfdrive.navdy.connection import UsbProbe
  entered, release = threading.Event(), threading.Event()
  fds = []
  class Candidate:
    def connect(self):
      fds.append(os.dup(self.recovery_fd))
      entered.set()
      release.wait(2)
      raise ConnectionError('offline')
    def close(self): pass
  probe = UsbProbe(Candidate)
  try:
    probe.poll(0)
    assert entered.wait(1)
    probe.close()
    assert select.select([fds[0]], [], [], 0)[0]
    release.set()
    probe.thread.join(1)
    assert probe.cancel_read is None and probe.cancel_write is None
  finally:
    release.set()
    probe.close()
    probe.thread.join(2)
    for fd in fds: os.close(fd)
    fds.clear()
  for _ in range(3):
    probe = UsbProbe(Candidate)
    probe.poll(0)
    probe.thread.join(1)
    probe.poll(3)
    probe.thread.join(1)
    probe.close()
    assert probe.cancel_read is None and probe.cancel_write is None
    for fd in fds: os.close(fd)
    fds.clear()


def test_probe_pipe_failure_does_not_interrupt_fallback(monkeypatch):
  from openpilot.selfdrive.navdy import connection
  def fail(): raise OSError('out of descriptors')
  monkeypatch.setattr(connection.os, 'pipe', fail)
  probe = connection.UsbProbe(lambda: None)
  assert probe.poll(0) is None
  assert probe.thread is None
  probe.close()


def test_logs_do_not_contain_device_identifiers(recovery):
  r = recovery
  for r.now in (100., 110., 130.):
    r.fail()
  text = json.dumps(r.events)
  assert 'TEST-NAVDY' not in text and r.devices[0]['identity'] not in text
