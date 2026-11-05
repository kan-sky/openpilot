"""Device-free role recovery checks. No test opens SSH or invokes systemd."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import n16_usb_role as role

BOOT = '04f93ca8-5842-4181-9484-c5c6436ccfa0'
PIN = {'format': 1, 'model': role.MODEL, 'content_sha256': 'a' * 64, 'base_source_commit': role.BASE}


def write(path, value):
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(value))


@pytest.fixture(autouse=True)
def no_external_commands(monkeypatch, request):
  if request.module.__name__ != __name__:
    return
  def denied(*args, **kwargs):
    pytest.fail('Unmocked external command')
  monkeypatch.setattr(subprocess, 'Popen', denied)


@pytest.fixture
def case(tmp_path, monkeypatch):
  state = tmp_path / 'paired'
  state.mkdir(mode=0o700)
  pin = tmp_path / 'selection.json'
  write(pin, PIN)
  config = {'version': 1, 'fingerprint': 'SHA256:paired', 'host': '10.0.0.9'}
  monkeypatch.setattr(role.management, 'paired_config', lambda path: deepcopy(config))
  route = {'interface': 'usb1', 'source': role.SOURCE, 'device': '/sys/devices/usb4/4-1', 'bus': '4', 'number': '2'}
  snapshot = {'boot_id': BOOT, 'overlay': role.selection(pin), 'policy_sha256': role.POLICY_SHA256,
              'selected': 'hl-n16-' + 'a' * 64, 'observed_monotonic': 100.0}
  calls = []
  value = NS(state=state, pin=pin, config=config, route=route, snapshot=snapshot, calls=calls, error=None, on_snapshot=None)

  def remote(state, route, arguments, timeout):
    calls.append((arguments, timeout))
    if 'systemd-run' not in arguments:
      if value.on_snapshot:
        value.on_snapshot()
      return json.dumps(snapshot)
    # The reservation must already be durable when dispatch is attempted.
    assert json.loads((state / role.STATE_FILE).read_text())['attempts'][-1]['state'] == 'reserved'
    if value.error:
      raise value.error
    return ''

  value.remote = remote
  return value


def recover(case, safe=lambda: True, **overrides):
  args = {'state': case.state, 'selection_path': case.pin, 'idle': lambda: True,
          'route_check': lambda: deepcopy(case.route), 'remote': case.remote}
  args.update(overrides)
  return role.recover_once(safe, **args)


def actions(case):
  return [argv for argv, _ in case.calls if 'systemd-run' in argv]


def test_no_pin_does_nothing(case):
  case.pin.unlink()
  assert recover(case) == {'state': 'not-selected'}
  assert not case.calls and not list(case.state.iterdir())


@pytest.mark.parametrize('field,value', [('model', 'b' * 64), ('base_source_commit', 'b' * 40),
                                       ('content_sha256', 'A' * 64), ('content_sha256', ''), ('format', True), ('format', 2)])
def test_invalid_pin_fails_before_ssh(case, field, value):
  write(case.pin, {**PIN, field: value})
  with pytest.raises(ValueError):
    recover(case)
  assert not case.calls


def test_pin_symlink_rejected(case):
  case.pin.unlink()
  case.pin.symlink_to(case.state / 'missing')
  with pytest.raises(ValueError):
    recover(case)
  assert not case.calls


@pytest.mark.parametrize('safe,idle', [(False, True), (True, False), (1, True), (True, 1)])
def test_unsafe_or_active_never_contacts_device(case, safe, idle):
  assert recover(case, lambda: safe, idle=lambda: idle)['state'] == 'blocked'
  assert not case.calls


def test_inference_status_fail_closed(tmp_path):
  path = tmp_path / 'status'
  assert role.inference_idle(path)
  for data in ({'active': True}, {'active': 0}, {}, []):
    write(path, data)
    assert not role.inference_idle(path)
  write(path, {'active': False})
  assert role.inference_idle(path)
  path.write_text('broken')
  assert not role.inference_idle(path)
  path.unlink()
  path.symlink_to(tmp_path / 'absent')
  assert not role.inference_idle(path)


def test_dispatch_is_independent_bounded_and_burns_boot(case):
  result = recover(case)
  assert result['state'] == 'dispatched' and result['verified'] is False
  argv = actions(case)[0]
  assert argv[:3] == ['sudo', '-n', 'systemd-run']
  assert '--no-block' in argv and '--collect' in argv and '--wait' not in argv
  for prop in ('Type=oneshot', 'RuntimeMaxSec=60', 'TimeoutStartSec=60', 'TimeoutStopSec=5',
               'KillMode=control-group', 'SendSIGKILL=yes', 'Restart=no', 'UMask=0077'):
    assert '--property=' + prop in argv
  expected = json.loads(argv[-1])
  assert expected['boot_id'] == BOOT and expected['expires'] == 115
  assert expected['pin'] == role.selection(case.pin)
  assert case.calls[-1][1] == 10
  assert (case.state / role.STATE_FILE).stat().st_mode & 0o777 == 0o600
  assert recover(case)['state'] == 'already-attempted'
  assert len(actions(case)) == 1
  # Changing the explicit overlay does not buy another attempt on the same boot.
  write(case.pin, {**PIN, 'content_sha256': 'b' * 64})
  case.snapshot['overlay'] = role.selection(case.pin)
  assert recover(case)['state'] == 'already-attempted'
  assert len(actions(case)) == 1


@pytest.mark.parametrize('error', [RuntimeError('lost reply'), OSError('USB gone'), subprocess.TimeoutExpired('ssh', 10)])
def test_ambiguous_dispatch_never_retries(case, error):
  case.error = error
  assert recover(case)['state'] == 'dispatch-unknown'
  assert recover(case)['state'] == 'already-attempted'
  assert len(actions(case)) == 1


def test_each_new_remote_boot_gets_one_attempt_and_state_is_bounded(case):
  for i in range(12):
    case.snapshot['boot_id'] = f'04f93ca8-5842-4181-9484-{i:012x}'
    assert recover(case)['state'] == 'dispatched'
  attempts = json.loads((case.state / role.STATE_FILE).read_text())['attempts']
  assert len(attempts) == role.MAX_ATTEMPTS
  assert attempts[-1]['boot_id'] == case.snapshot['boot_id']


@pytest.mark.parametrize('values,consumed', [([True, False], False), ([True, True, False], True)])
def test_fresh_offroad_guard_revocation(case, values, consumed):
  values = iter(values)
  assert recover(case, lambda: next(values))['state'] == 'blocked'
  assert not actions(case)
  assert (case.state / role.STATE_FILE).exists() == consumed
  if consumed:
    assert recover(case)['state'] == 'already-attempted'


def test_active_inference_at_final_guard_blocks(case):
  values = iter([True, True, False])
  assert recover(case, idle=lambda: next(values))['state'] == 'blocked'
  assert not actions(case)


@pytest.mark.parametrize('change', ['pin', 'pair', 'route'])
def test_inputs_rechecked_after_remote_inspection(case, change):
  def changed():
    if change == 'pin':
      write(case.pin, {**PIN, 'content_sha256': 'b' * 64})
    elif change == 'pair':
      case.config['fingerprint'] = 'SHA256:different'
    else:
      case.route['number'] = '3'
  case.on_snapshot = changed
  assert recover(case)['state'] == 'blocked'
  assert not actions(case)


@pytest.mark.parametrize('key,value', [('boot_id', '../invalid'), ('policy_sha256', 'b' * 64), ('overlay', {}),
                                     ('selected', role.BASE), ('observed_monotonic', float('nan')),
                                     ('observed_monotonic', True), ('observed_monotonic', -1)])
def test_remote_snapshot_mismatch_never_dispatches(case, key, value):
  case.snapshot[key] = value
  with pytest.raises(ValueError):
    recover(case)
  assert not actions(case)


@pytest.mark.parametrize('value', [{'format': True, 'attempts': []}, {'format': 1, 'attempts': [{}]},
                                  {'format': 1, 'attempts': [None] * 9}])
def test_bad_journal_cannot_authorize(case, value):
  write(case.state / role.STATE_FILE, value)
  with pytest.raises(ValueError):
    recover(case)
  assert not actions(case)


def test_journal_and_lock_symlinks_rejected(case):
  journal = case.state / role.STATE_FILE
  journal.symlink_to(case.state / 'missing')
  with pytest.raises(ValueError):
    recover(case)
  journal.unlink()
  lock = case.state / 'n16-usb-role.lock'
  lock.unlink()
  lock.symlink_to(case.state / 'missing')
  with pytest.raises(OSError):
    recover(case)
  assert not case.calls


def test_concurrent_recovery_does_not_dispatch(case):
  with role.locked(case.state), pytest.raises(BlockingIOError):
    recover(case)
  assert not case.calls


@pytest.fixture
def usb(tmp_path):
  root = tmp_path / 'sys'
  hub = root / 'devices/platform/soc/usb4'
  device = hub / '4-1'
  interface = device / '4-1:1.5'
  interface.mkdir(parents=True)
  for parent, vendor, product in ((hub, '1d6b', '0003'), (device, '0955', '7020')):
    for key, value in {'idVendor': vendor, 'idProduct': product, 'busnum': '4', 'devnum': '2'}.items():
      (parent / key).write_text(value)
  driver = root / 'bus/usb/drivers/rndis_host'
  driver.mkdir(parents=True)
  (interface / 'driver').symlink_to(driver)
  net = root / 'class/net/usb1'
  net.mkdir(parents=True)
  (net / 'carrier').write_text('1')
  (net / 'device').symlink_to(interface)
  row = {'dst': role.PEER, 'dev': 'usb1', 'prefsrc': role.SOURCE}

  def command(argv, timeout):
    assert argv == ['ip', '-j', '-4', 'route', 'get', role.PEER] and timeout == 5
    return json.dumps([row])
  return NS(root=root, hub=hub, device=device, interface=interface, net=net, row=row, command=command)


def test_nearest_nvidia_usb_device_accepts_linux_root_hub(usb):
  result = role.usb_route(usb.command, usb.root)
  assert result['device'] == str(usb.device.resolve())
  assert result['interface'] == 'usb1' and result['source'] == role.SOURCE


@pytest.mark.parametrize('key,value', [('gateway', '192.168.55.2'), ('prefsrc', '192.168.1.1'),
                                     ('dst', '192.168.55.0/24'), ('type', 'local'), ('dev', '../usb1')])
def test_non_direct_route_rejected(usb, key, value):
  usb.row[key] = value
  with pytest.raises(ValueError):
    role.usb_route(usb.command, usb.root)


@pytest.mark.parametrize('kind', ['foreign', 'no-carrier', 'wrong-driver', 'missing-vendor', 'virtual'])
def test_non_nvidia_or_nonphysical_interface_rejected(usb, kind):
  if kind == 'foreign':
    (usb.device / 'idVendor').write_text('1234')
    (usb.hub / 'idVendor').write_text('0955')
    (usb.hub / 'idProduct').write_text('7020')
  elif kind == 'no-carrier':
    (usb.net / 'carrier').write_text('0')
  elif kind == 'wrong-driver':
    target = usb.root / 'bus/usb/drivers/unknown'
    target.mkdir()
    (usb.interface / 'driver').unlink()
    (usb.interface / 'driver').symlink_to(target)
  elif kind == 'missing-vendor':
    (usb.device / 'idVendor').unlink()
  else:
    (usb.net / 'device').unlink()
    (usb.net / 'device').symlink_to(usb.net)
  with pytest.raises(ValueError):
    role.usb_route(usb.command, usb.root)


def test_ssh_uses_existing_pin_and_only_direct_source(case, monkeypatch):
  calls = []
  monkeypatch.setattr(role.management, 'run', lambda argv, timeout: calls.append((argv, timeout)) or 'ok')
  assert role.ssh(case.state, case.route, ['sudo', '-n', 'python3', '-c', 'print("quoted")'], 10) == 'ok'
  argv, timeout = calls[0]
  assert timeout == 10 and argv[0] == 'ssh'
  for option in ('StrictHostKeyChecking=yes', 'UpdateHostKeys=no', 'IdentitiesOnly=yes',
                 'HostKeyAlias=hl-paired-jetson', 'GlobalKnownHostsFile=/dev/null', 'ForwardAgent=no'):
    assert option in argv
  assert argv[-4:-1] == ['-b', role.SOURCE, 'jetlink@' + role.PEER]
  assert shlex.split(argv[-1]) == ['sudo', '-n', 'python3', '-c', 'print("quoted")']


@pytest.fixture
def target(tmp_path, monkeypatch):
  fs = tmp_path.resolve()
  runtime = fs / 'opt/carrot-jetlink'
  current = runtime / 'releases/hl-n16-test'
  current.mkdir(parents=True)
  (runtime / 'current').symlink_to(current)
  (current / 'SOURCE_COMMIT').write_text(role.BASE)
  helper = current / 'tools/jetlink/n16_overlay.py'
  helper.parent.mkdir(parents=True)
  helper.write_text('def verify_files(root):\n  assert (root / "checked-file").read_text() == "intact"\n')
  (current / 'checked-file').write_text('intact')
  files = {'tools/jetlink/n16_overlay.py': hashlib.sha256(helper.read_bytes()).hexdigest()}
  pin = {key: value for key, value in PIN.items() if key != 'format'}
  pin['content_sha256'] = hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
  manifest = {**pin, 'kind': 'device-local-n16', 'format': 1, 'files': files}
  write(current / 'HL_N16_OVERLAY.json', manifest)
  policy = fs / 'usr/local/lib/carrot-jetlink/usbc_policy.py'
  policy.parent.mkdir(parents=True)
  policy.write_text('# synthetic hash-pinned policy, never executed\n')
  boot = fs / 'proc/sys/kernel/random/boot_id'
  boot.parent.mkdir(parents=True)
  boot.write_text(BOOT)
  compatible = fs / 'proc/device-tree/compatible'
  compatible.parent.mkdir(parents=True)
  compatible.write_bytes(b'nvidia,p3767-0001+p3768-0000\0nvidia,tegra234\0')
  write(fs / 'run/carrot-storage.json', {'state': 'protected'})
  monkeypatch.setattr(platform, 'machine', lambda: 'aarch64')
  monkeypatch.setattr(time, 'monotonic', lambda: 100.0)
  monkeypatch.setattr(sys, 'dont_write_bytecode', True)
  # Synthetic files stand in for root-owned installed files; retain real modes.
  real_stat = Path.stat
  def root_stat(path, *args, **kwargs):
    result = real_stat(path, *args, **kwargs)
    if path.is_relative_to(fs):
      fields = list(result)
      fields[4] = 0
      return os.stat_result(fields)
    return result
  monkeypatch.setattr(Path, 'stat', root_stat)
  services = {'carrot-jetlink.service': {'ActiveState': 'inactive', 'MainPID': '0', 'ControlPID': '0'},
              'carrot-jetlink-boot-gate.service': {'ActiveState': 'active', 'MainPID': '123', 'ControlPID': '0'}}
  def command(argv, text, timeout):
    assert argv[:2] == ['systemctl', 'show'] and text and timeout == 5
    return '\n'.join(key + '=' + value for key, value in services[argv[2]].items())
  monkeypatch.setattr(subprocess, 'check_output', command)
  expected = {'pin': pin, 'policy_sha256': hashlib.sha256(policy.read_bytes()).hexdigest(), 'boot_id': BOOT, 'expires': 115}
  return NS(fs=fs, runtime=runtime, current=current, helper=helper, policy=policy, boot=boot,
            compatible=compatible, expected=expected, services=services, manifest=manifest)


def test_remote_check_verifies_tree_without_gpu(target):
  value = role._remote_check(target.expected, target.fs)
  assert value['overlay'] == target.expected['pin'] and value['boot_id'] == BOOT
  assert not list(target.fs.rglob('__pycache__'))


@pytest.mark.parametrize('mutation', ['boot', 'expires', 'module', 'arch', 'storage', 'transaction', 'dangling-transaction',
                                    'base', 'manifest', 'helper', 'policy', 'world-writable', 'tree',
                                    'active', 'main-pid', 'control-pid', 'gate', 'gate-pid', 'boot-ready', 'outside'])
def test_remote_mismatch_refuses_role_action(target, monkeypatch, mutation):
  if mutation == 'boot':
    target.boot.write_text('14f93ca8-5842-4181-9484-c5c6436ccfa0')
  elif mutation == 'expires':
    target.expected['expires'] = 99
  elif mutation == 'module':
    target.compatible.write_bytes(b'nvidia,p3767-0000\0')
  elif mutation == 'arch':
    monkeypatch.setattr(platform, 'machine', lambda: 'x86_64')
  elif mutation == 'storage':
    write(target.fs / 'run/carrot-storage.json', {'state': 'unprotected'})
  elif mutation == 'transaction':
    write(target.runtime / 'HL_N16_TRANSACTION.json', {})
  elif mutation == 'dangling-transaction':
    (target.runtime / 'HL_N16_TRANSACTION.json').symlink_to('missing')
  elif mutation == 'base':
    (target.current / 'SOURCE_COMMIT').write_text('b' * 40)
  elif mutation == 'manifest':
    write(target.current / 'HL_N16_OVERLAY.json', {**target.manifest, 'content_sha256': 'b' * 64})
  elif mutation in ('helper', 'policy'):
    getattr(target, mutation).write_text('changed')
  elif mutation == 'world-writable':
    target.policy.chmod(0o666)
  elif mutation == 'tree':
    (target.current / 'checked-file').write_text('changed')
  elif mutation in ('active', 'main-pid', 'control-pid'):
    key, value = {'active': ('ActiveState', 'active'), 'main-pid': ('MainPID', '12'), 'control-pid': ('ControlPID', '12')}[mutation]
    target.services['carrot-jetlink.service'][key] = value
  elif mutation == 'gate':
    target.services['carrot-jetlink-boot-gate.service']['ActiveState'] = 'inactive'
  elif mutation == 'gate-pid':
    target.services['carrot-jetlink-boot-gate.service']['MainPID'] = '0'
  elif mutation == 'boot-ready':
    write(target.fs / 'run/carrot-jetlink-boot-ready.json', {'boot_id': BOOT})
  else:
    (target.runtime / 'current').unlink()
    (target.runtime / 'current').symlink_to(target.fs)
  with pytest.raises((ValueError, AssertionError)):
    role._remote_check(target.expected, target.fs)


@pytest.mark.parametrize('apply', [False, True])
def test_embedded_program_is_complete_and_compiles(apply):
  program = role.remote_program(apply)
  compile(program, '<remote>', 'exec')
  assert 'import n16_usb_role' not in program
  if apply:
    assert "'--mode', 'comma', '--parked-test'" in program
    assert 'O_EXCL' in program and 'O_NOFOLLOW' in program


def test_remote_apply_reserves_once_rechecks_then_execs(tmp_path, monkeypatch):
  events = []
  monkeypatch.setattr(role, '_remote_check', lambda expected: events.append('check'))
  real_open = os.open
  def local_marker(path, flags, mode):
    assert path == '/run/carrot-n16-usb-role-attempt.json'
    assert flags & os.O_EXCL and flags & os.O_NOFOLLOW and mode == 0o600
    events.append('reserve')
    return real_open(tmp_path / 'marker', flags, mode)
  monkeypatch.setattr(os, 'open', local_marker)
  monkeypatch.setattr(os, 'execv', lambda executable, argv: events.append((executable, argv)))
  role._remote_apply({'boot_id': BOOT})
  assert events[:3] == ['check', 'reserve', 'check']
  assert events[3] == ('/usr/bin/python3', ['/usr/bin/python3', '-B',
                       '/usr/local/lib/carrot-jetlink/usbc_policy.py', '--mode', 'comma', '--parked-test'])
  with pytest.raises(FileExistsError):
    role._remote_apply({'boot_id': BOOT})
  assert sum(isinstance(event, tuple) for event in events) == 1


@pytest.mark.parametrize('state,age,ready', [('ready', 0, True), ('ready', 2.99, True), ('ready', 3, False),
                                         ('ready', -1, False), ('loading', 0, False), ('ready', float('nan'), False)])
def test_only_fresh_ready_skips_role_probe(tmp_path, monkeypatch, state, age, ready):
  path = tmp_path / 'link.json'
  write(path, {'state': state, 'updated': 100 - age})
  monkeypatch.setattr(role.time, 'monotonic', lambda: 100)
  assert role.link_ready(path) is ready


def test_ready_keeps_previous_role_diagnostic_and_skips_probe(case, monkeypatch):
  path = case.state / 'n16-usb-role-status.json'
  write(path, {'state': 'dispatched', 'boot_id': BOOT})
  before = path.read_bytes()
  monkeypatch.setattr(role, 'link_ready', lambda: True)
  monkeypatch.setattr(role, 'recover_once', lambda *a, **k: pytest.fail('READY must skip SSH'))
  role.management.recover_n16_role(case.state, lambda: True)
  assert path.read_bytes() == before
  assert not (case.state / 'status.json').exists()


@pytest.mark.parametrize('failure', [RuntimeError('No .55 route'), LookupError('Optional probe failed')])
def test_role_error_is_separate_from_updater_status(case, monkeypatch, failure):
  write(case.state / 'status.json', {'state': 'succeeded', 'token': 'updater'})
  before = (case.state / 'status.json').read_bytes()
  monkeypatch.setattr(role, 'link_ready', lambda: False)
  def fail(*args, **kwargs):
    raise failure
  monkeypatch.setattr(role, 'recover_once', fail)
  role.management.recover_n16_role(case.state, lambda: True)
  assert (case.state / 'status.json').read_bytes() == before
  assert json.loads((case.state / 'n16-usb-role-status.json').read_text())['state'] == 'probe-error'


@pytest.mark.parametrize('scenario,expected', [('selected', [30, 60, 90]), ('no-pin', []), ('onroad', []),
                                            ('interrupted', [80]), ('pending', [])])
def test_daemon_stable_offroad_rate_and_selection_gates(case, monkeypatch, scenario, expected):
  client = role.management
  class Finished(BaseException):
    pass
  now, calls = [0.], []
  monkeypatch.setattr(client, 'STATE', case.state)
  monkeypatch.setattr(client, 'N16_SELECTION', case.pin)
  write(case.state / 'config.json', {'automatic': False})
  if scenario == 'no-pin':
    case.pin.unlink()
  if scenario == 'pending':
    write(case.state / 'job.json', {'token': 'existing'})
  def safe():
    return scenario != 'onroad' and not (scenario == 'interrupted' and 20 <= now[0] < 50)
  monkeypatch.setattr(client, 'Safety', lambda: safe)
  monkeypatch.setattr(client.time, 'monotonic', lambda: now[0])
  monkeypatch.setattr(role, 'link_ready', lambda: False)
  def failed_probe(guard, **kwargs):
    assert guard is safe and guard()
    assert kwargs['selection_path'] == case.pin and kwargs['state'] == case.state
    calls.append(now[0])
    raise RuntimeError('Normal USB has no .55 route')
  monkeypatch.setattr(role, 'recover_once', failed_probe)
  monkeypatch.setattr(client, 'sync', lambda *a, **k: pytest.fail('Role recovery must not trigger HUD/update installation'))
  def sleep(seconds):
    now[0] += seconds
    if now[0] > 90:
      raise Finished
  monkeypatch.setattr(client.time, 'sleep', sleep)
  with pytest.raises(Finished):
    client.daemon()
  assert calls == expected
  assert not (case.state / 'status.json').exists()


def test_role_diagnostic_write_failure_does_not_escape(case, monkeypatch):
  monkeypatch.setattr(role, 'link_ready', lambda: False)
  monkeypatch.setattr(role, 'recover_once', lambda *a, **k: {'state': 'blocked'})
  def fail(*args):
    raise OSError('Disk full')
  monkeypatch.setattr(role.management, 'private_json', fail)
  role.management.recover_n16_role(case.state, lambda: True)
