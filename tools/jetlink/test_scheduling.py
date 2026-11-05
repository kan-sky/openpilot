import ast
import inspect
import sys
from contextlib import nullcontext
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.modeld.jetlink import daemon


def test_gadget_setup_precedes_functionfs_owner_and_mobile_listener():
  tree = ast.parse(inspect.getsource(daemon.main))
  main = tree.body[0]
  calls = {n.func.id: n for n in ast.walk(main) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
           and n.func.id in ('setup_gadget', 'CarrotTransport', 'MobileCable')}
  assert calls['setup_gadget'].lineno < calls['CarrotTransport'].lineno < calls['MobileCable'].lineno


@pytest.mark.parametrize('mode,unavailable,expected', [('usb', False, 'usb'), ('auto', True, 'usb'),
                                                   ('auto', False, 'auto'), ('android', False, 'android'), ('ios', False, 'ios')])
def test_actual_daemon_call_keeps_auto_fallback_bulk_only(mode, unavailable, expected):
  tree = ast.parse(inspect.getsource(daemon.main))
  call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'setup_gadget')
  assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'management' for t in n.targets))
  scope = {'TRANSPORT_MODE': mode, 'mobile_unavailable': unavailable, 'management_safe': object(),
           'management_allowed': lambda *a, **kw: True, 'setup_gadget': lambda mode, management: (mode, management)}
  exec(compile(ast.Module(body=[assignment], type_ignores=[]), '<management-gate>', 'exec'), scope)
  actual = eval(compile(ast.Expression(call), '<gadget-call>', 'eval'),
                scope)
  assert actual == (expected, mode == 'usb')


@pytest.mark.parametrize('onroad,big_online', [(False, False), (False, True), (True, False), (True, True)])
@pytest.mark.parametrize('vanished_thread', [False, True])
def test_onroad_policy_uses_low_priority_and_keeps_all_threads_off_core7(monkeypatch, onroad, big_online, vanished_thread):
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(get_bool=lambda key: onroad)))
  def path(value):
    if value == '/sys/devices/system/cpu/cpu7/online':
      assert onroad  # Offroad never needs to probe or wake the big cores.
      return NS(read_text=lambda: '1' if big_online else '0')
    assert value == '/proc/self/task'
    return NS(iterdir=lambda: [NS(name='10'), NS(name='11')])
  monkeypatch.setattr(daemon, 'Path', path)
  calls, affinities = [], []
  monkeypatch.setattr(daemon.os, 'SCHED_FIFO', 1, raising=False)
  monkeypatch.setattr(daemon.os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(daemon.os, 'sched_setscheduler', lambda pid, policy, priority: calls.append((pid, policy, priority)), raising=False)
  monkeypatch.setattr(daemon.os, 'sched_param', lambda value: value, raising=False)
  def affinity(tid, cores):
    affinities.append((tid, cores))
    if vanished_thread and tid == 10:
      raise ProcessLookupError('thread exited')
  monkeypatch.setattr(daemon.os, 'sched_setaffinity', affinity, raising=False)
  daemon.update_affinity()
  realtime = onroad and big_online
  assert calls == [(0, 1 if realtime else 0, 1 if realtime else 0)]
  assert affinities == [(10, {0, 1, 2, 3}), (11, {0, 1, 2, 3})]


@pytest.mark.parametrize('error', [ValueError, RuntimeError])
def test_client_disconnect_closes_ipc_without_overriding_road_policy(monkeypatch, error):
  from openpilot.selfdrive.modeld.jetlink.link import SPEC
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.common.runtime_diagnostics', NS(RuntimeDiagnostics=lambda *a: None))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(event=lambda *a: None)))
  connected, policies, closed = [True], [], []
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: 100.)
  monkeypatch.setattr(daemon, 'host_attached', lambda: connected[0])
  monkeypatch.setattr(daemon, 'publish', lambda *a, **kw: None)
  monkeypatch.setattr(daemon, 'update_affinity', lambda: policies.append('road-policy'))
  class Connection:
    def settimeout(self, seconds):
      pass
    def __enter__(self):
      return self
    def __exit__(self, *args):
      closed.append(True)
  def send(*args):
    connected[0] = False
    raise error('reply failed')
  monkeypatch.setattr(daemon, 'send', send)
  with pytest.raises(error) if error is RuntimeError else nullcontext():
    daemon._serve_local(NS(accept=lambda: (Connection(), None)), NS(spec=SPEC, last_state={}), {'protocol': 2}, None, None)
  assert policies == ['road-policy']
  assert closed == [True]
