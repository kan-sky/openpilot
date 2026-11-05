"""Management lifecycle checks with virtual configfs, never real USB or SSH."""
import ast
import inspect
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.selfdrive.modeld.jetlink import daemon, mobile
import usb_management as usb


@pytest.mark.parametrize('reply,enabled', [('true', True), ('false', False), ('1', False), ('null', False)])
def test_boot_latch_uses_only_fixed_root_action(monkeypatch, reply, enabled):
  root = Mock(return_value='{"enabled_this_boot":' + reply + '}')
  monkeypatch.setattr(mobile, '_root', root)
  assert mobile.initialize_management() is enabled
  root.assert_called_once_with('setup_gadget.sh', '--latch-management')


@pytest.mark.parametrize('reply', ['bad json', '[]', '{}'])
def test_unreadable_boot_latch_leaves_ordinary_inference_available(monkeypatch, reply):
  monkeypatch.setattr(mobile, '_root', lambda *a: reply)
  assert mobile.initialize_management() is False
  monkeypatch.setattr(mobile, '_root', Mock(side_effect=mobile.MobileLinkError('root helper unavailable')))
  assert mobile.initialize_management() is False


def test_boot_latch_follows_exclusive_owner_lock_and_precedes_usb_setup():
  tree = ast.parse(inspect.getsource(daemon.main))
  calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
  latch = next(n for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'initialize_management')
  lock = next(n for n in calls if isinstance(n.func, ast.Attribute) and n.func.attr == 'flock')
  usb_calls = [n for n in calls if isinstance(n.func, ast.Name) and n.func.id in ('setup_gadget', 'CarrotTransport')]
  assert lock.lineno < latch.lineno and all(latch.lineno < n.lineno for n in usb_calls)


@pytest.mark.parametrize('mode', ['usb', 'android', 'auto', 'ios'])
def test_only_explicit_usb_can_request_management(monkeypatch, tmp_path, mode):
  calls = []
  monkeypatch.setattr(mobile, 'GADGET', tmp_path)
  monkeypatch.setattr(mobile, '_root', lambda *a: calls.append(a))
  mobile.setup_gadget(mode, management=True)
  assert calls == ([('setup_gadget.sh', '--management')] if mode == 'usb' else
                   [('setup_gadget.sh',)] if mode == 'android' else [('setup_mobile.sh', 'gadget')])


@pytest.mark.parametrize('initial,after_bind,network_failure', [
  (False, False, False), (True, False, False), (True, True, False), (True, True, True),
])
def test_actual_owner_setup_rechecks_offroad_after_binding(monkeypatch, tmp_path, initial, after_bind, network_failure):
  tree = ast.parse(inspect.getsource(daemon.main))
  block = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'owner is None')
  udcs = tmp_path / 'udcs'
  (udcs / 'controller').mkdir(parents=True)
  monkeypatch.setattr(mobile, 'GADGET', tmp_path / 'gadget')
  calls, now, bound = [], [100.], [False]
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: now[0])
  monkeypatch.setattr(daemon.time, 'sleep', lambda seconds: now.__setitem__(0, now[0] + seconds))
  def root(*args):
    calls.append(args)
    if args[-1] == '--activate-management':
      assert bound[0]
      if network_failure:
        raise mobile.MobileLinkError('no management network')
  monkeypatch.setattr(mobile, '_root', root)
  def owner(*args, **kwargs):
    calls.append('bound')
    bound[0] = True
    return NS(bound_udc='controller')
  scope = {'owner': None, 'TRANSPORT_MODE': 'usb', 'mobile_unavailable': False,
           'management_allowed': daemon.management_allowed,
           'management_safe': lambda: after_bind if bound[0] else initial,
           'setup_gadget': mobile.setup_gadget, 'CarrotTransport': owner, 'GADGET': 'virtual',
           'Path': lambda _: udcs, 'activate_management': mobile.activate_management}
  exec(compile(ast.Module(body=[block], type_ignores=[]), '<actual-owner-setup>', 'exec'), scope)
  assert scope['owner'].management is initial
  assert calls == [('setup_gadget.sh', *(['--management'] if initial else [])), 'bound',
                   *([('setup_gadget.sh', '--activate-management')] if initial and after_bind else [])]
  assert now[0] <= 101.001
  before = list(calls)
  exec(compile(ast.Module(body=[block], type_ignores=[]), '<existing-owner>', 'exec'), scope)
  assert calls == before  # Never hot-reconfigure an existing inference owner.


def test_guard_unavailable_or_failing_never_grants_management(monkeypatch):
  assert not daemon.management_allowed(None)
  assert not daemon.management_allowed(Mock(side_effect=OSError('state unavailable')))
  from tools.jetlink import manage_jetson
  monkeypatch.setattr(manage_jetson, 'Safety', Mock(side_effect=ImportError('native IPC missing')))
  assert daemon.management_safety() is None


@pytest.mark.parametrize('bound,reader_alive,cleanup_failure', [
  ('', False, False), ('controller', False, False), ('', True, False), ('', False, True),
])
def test_management_detach_requires_verified_native_owner_shutdown(monkeypatch, tmp_path, bound, reader_alive, cleanup_failure):
  owner = daemon.CarrotTransport.__new__(daemon.CarrotTransport)
  owner.gadget, owner.management = str(tmp_path), True
  owner._reader = NS(is_alive=lambda: reader_alive)
  (tmp_path / 'UDC').write_text(bound)
  calls = []
  monkeypatch.setattr(daemon.FfsTransport, 'close', lambda self: calls.append('native-close'))
  def detach():
    calls.append('detach-management')
    if cleanup_failure:
      raise mobile.MobileLinkError('cleanup failed')
  monkeypatch.setattr(daemon, 'deactivate_management', detach)
  error = daemon.close_resources(owner)
  assert bool(error) is (bool(bound) or reader_alive or cleanup_failure)
  assert calls == ['native-close', *([] if bound or reader_alive else ['detach-management'])]
  assert owner.management is bool(error)


@pytest.mark.parametrize('flag,action', [('--latch-management', 'boot'), ('--activate-management', 'net'),
                                      ('--detach-management', 'detach')])
def test_shell_dispatches_management_only_to_fixed_helper(tmp_path, flag, action):
  python = tmp_path / 'python3'
  python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
  python.chmod(0o700)
  script = Path(usb.__file__).with_name('setup_gadget.sh')
  result = subprocess.run(['/bin/bash', str(script), flag], env=dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.defpath),
                          capture_output=True, text=True)
  assert result.returncode == 0, result.stderr
  assert result.stdout.splitlines() == [str(script.with_name('usb_management.py')), action]


def test_unwritable_management_status_does_not_abort_operation(tmp_path, monkeypatch, capsys):
  monkeypatch.setattr(usb, 'STATUS', tmp_path / 'missing/status.json')
  usb.report('prepared')
  assert 'status unavailable' in capsys.readouterr().err


def test_gadget_script_checks_unbound_state_before_descriptor_writes():
  source = Path(usb.__file__).with_name('setup_gadget.sh').read_text()
  assert source.index('jetlink controller is still bound') < source.index('echo "$VID"')
  assert source.index('usb_management.py" gadget') > source.index('[[ -e "$FFS_MOUNT/ep0" ]]')
