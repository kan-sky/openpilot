import ast
import datetime
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.cereal import log
from openpilot.common.logging_extra import SwagLogger
from openpilot.system.hardware.automatic_shutdown import REQUEST_KEY, automatic_shutdown_reason, try_automatic_shutdown

PandaType = log.PandaState.PandaType
HarnessStatus = log.PandaState.HarnessStatus


class Messages:
  def __init__(self):
    self.data = {
      "deviceState": NS(started=False),
      "pandaStates": [NS(pandaType=PandaType.cuatro, ignitionLine=False, ignitionCan=False, harnessStatus=HarnessStatus.normal)],
      "peripheralState": NS(pandaType=PandaType.cuatro, voltage=11200),
    }
    self.seen = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)
    self.recv_time = dict.fromkeys(self.data, 100.)
    self.logMonoTime = dict.fromkeys(self.data, 100_000_000_000)
    self.update = Mock()

  def __getitem__(self, key):
    return self.data[key]


def request(reason="low_voltage", timestamp=100.):
  return {"reason": reason, "monotonicTime": timestamp}


@pytest.mark.parametrize("reason", ["low_voltage", "offroad_timeout"])
def test_fresh_offroad_request_can_poweroff(reason):
  sm, params, shutdown = Messages(), Mock(), Mock()
  params.get.return_value = request(reason)
  assert try_automatic_shutdown(sm, params, shutdown, clock=lambda: 100.) == reason
  sm.update.assert_called_once_with(0)
  assert params.get.call_count == 2
  params.get.assert_called_with(REQUEST_KEY)
  shutdown.assert_called_once_with()
  params.put_bool.assert_not_called()


@pytest.mark.parametrize("value", [None, {}, [], "", {"reason": "manual"}, request(timestamp=None),
                                    request(timestamp=True), request(timestamp=float("nan")),
                                    request(timestamp=float("inf")), request(timestamp=98.499), request(timestamp=100.001)])
def test_bad_expired_and_future_requests_do_not_poweroff(value):
  sm, params, shutdown = Messages(), Mock(), Mock()
  params.get.return_value = value
  assert try_automatic_shutdown(sm, params, shutdown, clock=lambda: 100.) is None
  shutdown.assert_not_called()


@pytest.mark.parametrize("service", ["deviceState", "pandaStates", "peripheralState"])
@pytest.mark.parametrize("field,value", [("seen", False), ("valid", False), ("alive", False),
                                        ("recv_time", 98.), ("recv_time", 101.),
                                        ("logMonoTime", 98_000_000_000), ("logMonoTime", 101_000_000_000)])
def test_unknown_or_stale_state_blocks_shutdown(service, field, value):
  sm = Messages()
  getattr(sm, field)[service] = value
  assert automatic_shutdown_reason(request(), sm, 100.) is None


@pytest.mark.parametrize("field", ["ignitionLine", "ignitionCan"])
@pytest.mark.parametrize("reason", ["low_voltage", "offroad_timeout"])
def test_any_panda_ignition_blocks_automatic_poweroff(field, reason):
  sm = Messages()
  sm.data["pandaStates"].append(NS(pandaType=PandaType.cuatro, ignitionLine=False, ignitionCan=False))
  setattr(sm["pandaStates"][1], field, True)
  assert automatic_shutdown_reason(request(reason), sm, 100.) is None


@pytest.mark.parametrize("reason", ["low_voltage", "offroad_timeout"])
def test_onroad_or_unknown_panda_blocks_automatic_poweroff(reason):
  sm = Messages()
  sm["deviceState"].started = True
  assert automatic_shutdown_reason(request(reason), sm, 100.) is None
  sm["deviceState"].started = False
  sm["pandaStates"][0].pandaType = PandaType.unknown
  assert automatic_shutdown_reason(request(reason), sm, 100.) is None
  sm.data["pandaStates"] = []
  assert automatic_shutdown_reason(request(reason), sm, 100.) is None


@pytest.mark.parametrize("voltage", [0, 11300, 14000])
def test_final_raw_voltage_recovery_blocks_low_voltage_not_timeout(voltage):
  sm = Messages()
  sm["peripheralState"].voltage = voltage
  assert automatic_shutdown_reason(request(), sm, 100.) is None
  assert automatic_shutdown_reason(request("offroad_timeout"), sm, 100.) == "offroad_timeout"


def test_bench_power_can_expire_but_not_trigger_vehicle_low_voltage():
  sm = Messages()
  sm["pandaStates"][0].harnessStatus = HarnessStatus.notConnected
  sm["peripheralState"].voltage = 5000
  assert automatic_shutdown_reason(request(), sm, 100.) is None
  assert automatic_shutdown_reason(request("offroad_timeout"), sm, 100.) == "offroad_timeout"


def test_actual_cereal_readers_and_ignition_transition():
  sm = Messages()
  panda = log.PandaState.new_message(pandaType="cuatro", harnessStatus="normal", ignitionLine=False, ignitionCan=False)
  sm.data["pandaStates"] = [panda.as_reader()]
  sm.data["deviceState"] = log.DeviceState.new_message(started=False).as_reader()
  sm.data["peripheralState"] = log.PeripheralState.new_message(pandaType="cuatro", voltage=11200).as_reader()
  assert automatic_shutdown_reason(request(), sm, 100.) == "low_voltage"
  panda.ignitionCan = True
  assert automatic_shutdown_reason(request(), sm, 100.) is None


@pytest.mark.parametrize("change", ["ignitionLine", "ignitionCan", "started", "recovery", "stale", "harness", "withdraw", "expired"])
def test_final_message_drain_cancels_pending_poweroff(change):
  sm, params, shutdown = Messages(), Mock(), Mock()
  params.get.return_value = request()
  clock = Mock(return_value=100.)

  def drain(_):
    if change in ("ignitionLine", "ignitionCan"):
      setattr(sm["pandaStates"][0], change, True)
    elif change == "started":
      sm["deviceState"].started = True
    elif change == "recovery":
      sm["peripheralState"].voltage = 12000
    elif change == "stale":
      sm.valid["pandaStates"] = False
    elif change == "harness":
      sm["pandaStates"][0].harnessStatus = HarnessStatus.notConnected
    elif change == "withdraw":
      params.get.return_value = None
    elif change == "expired":
      clock.return_value = 102.
  sm.update.side_effect = drain
  assert try_automatic_shutdown(sm, params, shutdown, clock=clock) is None
  shutdown.assert_not_called()


def test_manager_keeps_manual_requests_separate_and_checks_auto_before_cleanup():
  root = Path(__file__).resolve().parents[3]
  manager = ast.parse((root / "system/manager/manager.py").read_text())
  thread = next(n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name == "manager_thread")
  loops = [n for n in ast.walk(thread) if isinstance(n, ast.For) and isinstance(n.iter, ast.Tuple)]
  assert any([x.value for x in n.iter.elts] == ["DoUninstall", "DoShutdown", "DoReboot"] for n in loops)
  calls = [n for n in ast.walk(thread) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
  assert any(n.func.id == "try_automatic_shutdown" for n in calls)
  assert not any(n.func.id == "manager_cleanup" for n in calls)
  hardware = ast.parse((root / "system/hardware/hardwared.py").read_text())
  assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr.startswith("put")
                 and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == "DoShutdown"
                 for n in ast.walk(hardware))


@pytest.mark.parametrize("mode", ["automatic", "ignition_during_final_check", "manual_shutdown", "manual_reboot"])
def test_actual_manager_loop_uses_cancellable_auto_path(mode):
  root = Path(__file__).resolve().parents[3]
  tree = ast.parse((root / "system/manager/manager.py").read_text())
  thread = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "manager_thread")
  sm, shutdown = Messages(), Mock()
  sm.data["carParams"] = NS()
  sm.all_checks = lambda *_: False
  values = {REQUEST_KEY: request(), "DongleId": "isolated-test"}
  if mode == "manual_shutdown":
    values["DoShutdown"] = True
  if mode == "manual_reboot":
    values["DoReboot"] = True
  params = NS(get=lambda k: values.get(k), get_bool=lambda k: bool(values.get(k, False)),
              put=lambda k, v: values.update({k: v}), put_bool=lambda k, v: values.update({k: v}))
  polls = []

  def update(timeout):
    polls.append(timeout)
    if timeout == 0 and mode == "ignition_during_final_check":
      sm["pandaStates"][0].ignitionCan = True
    elif timeout == 1000 and polls.count(1000) == 2:
      # A second loop proves cancellation did not tear down managed processes.
      values["DoReboot"] = True
    assert polls.count(1000) <= 2
  sm.update = update
  globals_ = {
    "UpdateStatus": object, "Params": lambda: params, "log": log,
    "cloudlog": SwagLogger(), "os": NS(environ={}, getenv=lambda key, default=None: default),
    "UNREGISTERED_DONGLE_ID": "unregistered", "managed_processes": {},
    "messaging": NS(SubMaster=lambda *a, **kw: sm, PubMaster=lambda *_: Mock(),
                    new_message=lambda *a, **kw: NS(managerState=NS())),
    "XiaogeStartupGate": lambda: NS(update=lambda *_: True),
    "write_onroad_params": Mock(), "clear_ignition_on_params": Mock(),
    "ensure_running": Mock(), "time": NS(monotonic=lambda: 100.),
    "datetime": datetime, "HARDWARE": NS(shutdown=shutdown),
    "notify_automatic_shutdown": Mock(),
    "try_automatic_shutdown": lambda s, p, fn, **kw: try_automatic_shutdown(s, p, fn, clock=lambda: 100., **kw),
  }
  exec(compile(ast.Module(body=[thread], type_ignores=[]), "manager.py", "exec"), globals_)
  globals_["manager_thread"](NS(update=lambda *_: False))
  if mode == "automatic":
    shutdown.assert_called_once_with()
    assert polls == [1000, 0, 0]
    globals_["notify_automatic_shutdown"].assert_called_once_with("low_voltage", sm, params)
    assert values["LastManagerExitReason"].startswith("AutomaticShutdown low_voltage ")
    assert "DoShutdown" not in values
  elif mode == "ignition_during_final_check":
    globals_["notify_automatic_shutdown"].assert_not_called()
    shutdown.assert_not_called()
    assert polls == [1000, 0, 1000]
    assert values["LastManagerExitReason"].startswith("DoReboot ")
    assert globals_["ensure_running"].call_count == 3
  else:
    globals_["notify_automatic_shutdown"].assert_not_called()
    shutdown.assert_not_called()  # Manual poweroff/reboot stays in main(), after cleanup.
    assert polls == [1000]
    assert values["LastManagerExitReason"].startswith("DoShutdown " if mode == "manual_shutdown" else "DoReboot ")


@pytest.mark.parametrize("reason", ["low_voltage", "offroad_timeout"])
@pytest.mark.parametrize("fails", [False, True])
def test_notice_precedes_poweroff_but_failure_does_not_prevent_it(reason, fails):
  sm, params, shutdown = Messages(), Mock(), Mock()
  params.get.return_value = request(reason)
  order = []
  def notify(*args):
    order.append("notice")
    assert args == (reason, sm, params)
    if fails:
      raise RuntimeError("offline")
  shutdown.side_effect = lambda: order.append("shutdown")
  assert try_automatic_shutdown(sm, params, shutdown, clock=lambda: 100., notify=notify) == reason
  assert order == ["notice", "shutdown"]
  assert sm.update.call_count == 2


@pytest.mark.parametrize("change", ["ignition", "onroad", "recovery", "withdraw", "stale", "reason"])
def test_recheck_after_notice_cancels_shutdown(change):
  sm, params, shutdown = Messages(), Mock(), Mock()
  params.get.return_value = request()
  def notify(*_):
    if change == "ignition":
      sm["pandaStates"][0].ignitionCan = True
    elif change == "onroad":
      sm["deviceState"].started = True
    elif change == "recovery":
      sm["peripheralState"].voltage = 12000
    elif change == "withdraw":
      params.get.return_value = None
    elif change == "stale":
      sm.valid["pandaStates"] = False
    else:
      params.get.return_value = request("offroad_timeout")
  assert try_automatic_shutdown(sm, params, shutdown, clock=lambda: 100., notify=notify) is None
  shutdown.assert_not_called()


def test_invalid_request_does_not_notify():
  sm, params, shutdown, notify = Messages(), Mock(), Mock(), Mock()
  params.get.return_value = None
  assert try_automatic_shutdown(sm, params, shutdown, clock=lambda: 100., notify=notify) is None
  notify.assert_not_called()
