import __future__
import ast
import json
import os
import signal
from abc import ABC, abstractmethod
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.system import wayon_camera as camera


class Params:
  def __init__(self, **values):
    self.values = {"IsOffroad": True, **values}

  def get_bool(self, key):
    return self.values.get(key, False)


class Samples(dict):
  def __init__(self):
    super().__init__(deviceState=SimpleNamespace(started=False, thermalStatus="green", memoryUsagePercent=50),
                     pandaStates=[SimpleNamespace(ignitionLine=False, ignitionCan=False, controlsAllowed=False)])
    self.seen = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.recv_time = dict.fromkeys(self, 100.)
    self.logMonoTime = dict.fromkeys(self, 100_000_000_000)


def test_offroad_guard_never_relies_only_on_manager_params():
  sm = Samples()
  assert camera.camera_block_reason(Params(), sm, 101) is None
  for field in ("ignitionLine", "ignitionCan", "controlsAllowed"):
    setattr(sm["pandaStates"][0], field, True)
    assert camera.camera_block_reason(Params(), sm, 101) == "ignition"
    setattr(sm["pandaStates"][0], field, False)
  sm["deviceState"].started = True
  assert camera.camera_block_reason(Params(), sm, 101) == "ignition"


@pytest.mark.parametrize("key,value,reason", [("IsOnroad", True, "onroad"), ("IsOffroad", False, "onroad"),
                                             ("IsDriverViewEnabled", True, "driver_view")])
def test_camera_refuses_active_other_modes(key, value, reason):
  assert camera.camera_block_reason(Params(**{key: value}), Samples(), 101) == reason


def test_guard_rejects_stale_invalid_future_or_republished_state():
  for name in ("deviceState", "pandaStates"):
    for field, value in (("seen", False), ("valid", False), ("recv_time", 95),
                         ("recv_time", 103), ("logMonoTime", 95_000_000_000), ("logMonoTime", 103_000_000_000)):
      sm = Samples()
      getattr(sm, field)[name] = value
      assert camera.camera_block_reason(Params(), sm, 101) == "state_unavailable"


def test_thermal_and_memory_guards():
  sm = Samples()
  sm["deviceState"].thermalStatus = "red"
  assert camera.camera_block_reason(Params(), sm, 101) == "temperature"
  sm["deviceState"].thermalStatus = "green"
  for value in (92, 99, float("nan"), float("inf")):
    sm["deviceState"].memoryUsagePercent = value
    assert camera.camera_block_reason(Params(), sm, 101) == "low_memory"


def test_snapshot_accepts_current_carrot_frame_without_unused_valid_bit():
  client = SimpleNamespace(valid=False, timestamp_eof=100_000_000_000)
  assert camera.snapshot_frame_fresh(client, 100.1)
  assert not camera.snapshot_frame_fresh(client, 100.6)
  assert not camera.snapshot_frame_fresh(client, 99.9)
  for eof in (0, -1, float("nan"), float("inf")):
    client.timestamp_eof = eof
    assert not camera.snapshot_frame_fresh(client, 100.1)


def test_marker_expires_and_is_owned(tmp_path, monkeypatch):
  path = tmp_path / "camera"
  monkeypatch.setattr(camera.time, "monotonic", lambda: 100)
  camera.request_camera("panorama", path)
  assert path.stat().st_mode & 0o777 == 0o600
  assert camera.camera_request(path, 101)["mode"] == "panorama"
  assert camera.camera_request(path, 104) is None
  assert camera.camera_request(path, 99) is None
  camera.release_camera(path)
  assert not path.exists()
  path.write_text(json.dumps({"pid": os.getpid() + 1, "mode": "snapshot", "updated": 100}))
  camera.release_camera(path)
  assert path.exists()
  monkeypatch.setattr(camera.os, "kill", lambda *args: (_ for _ in ()).throw(ProcessLookupError()))
  assert camera.camera_request(path, 101) is None


@pytest.mark.parametrize("value", ["", "{", "[]", "null", '{"pid":"bad"}', '{"pid":1,"updated":100,"mode":"panorama"}'])
def test_bad_marker_is_harmless(tmp_path, value):
  path = tmp_path / "camera"
  path.write_text(value)
  assert camera.camera_request(path, 101) is None
  camera.release_camera(path)


def test_no_automatic_enrollment(tmp_path):
  path = tmp_path / "config"
  assert not camera.configured(path)
  for config in ({}, {"token":"wayon_test", "endpoint":"http://insecure"},
                 {"token":"wayon_test", "endpoint":"https://example.com", "enabled":False}):
    path.write_text(json.dumps(config))
    assert not camera.configured(path)
  path.write_text(json.dumps({"token":"wayon_test", "endpoint":"https://example.com"}))
  assert camera.configured(path)


def test_camera_handover_waits_for_previous_owner_without_changing_params():
  tree = ast.parse((Path(__file__).parent / "manager/process_config.py").read_text())
  functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name in ("driverview", "normal_camera", "remote_camera")]
  scope = {"Params": Params, "car": SimpleNamespace(CarParams=object),
           "wayon_camera_requested": lambda started, params, CP: not started and params.get_bool("IsOffroad"),
           "managed_processes": {name: SimpleNamespace(proc=None) for name in ("camerad", "wayon_camerad")}}
  exec(compile(ast.Module(body=functions, type_ignores=[]), "camera_predicates", "exec"), scope)
  params = Params()
  owners = scope["managed_processes"]
  assert scope["remote_camera"](False, params, None)
  assert not scope["normal_camera"](False, params, None)
  owners["wayon_camerad"].proc = object()
  assert not scope["remote_camera"](True, params, None)
  assert not scope["normal_camera"](True, params, None)
  owners["wayon_camerad"].proc = None
  assert scope["normal_camera"](True, params, None)
  owners["camerad"].proc = object()
  assert not scope["remote_camera"](False, params, None)
  owners["camerad"].proc = None
  params.values["IsDriverViewEnabled"] = True
  assert not scope["remote_camera"](False, params, None)
  assert scope["normal_camera"](False, params, None)
  assert params.values == {"IsOffroad": True, "IsDriverViewEnabled": True}


@pytest.fixture
def manager_lifecycle(monkeypatch):
  state = SimpleNamespace(now=100.0, children=[], signals=[], reports=[], joins=[], request={"mode": "panorama"})

  def advance(seconds):
    state.now += seconds

  class Child:
    def __init__(self, name, target, args):
      self.name, self.target, self.args = name, target, args
      self.pid = 1000 + len(state.children)
      self.code = None
      self.exit_at = None
      self.ignore_interrupt = False
      state.children.append(self)

    @property
    def exitcode(self):
      if self.exit_at is not None and state.now >= self.exit_at:
        self.code = 0
      return self.code

    def start(self):
      if self.name in ("camerad", "wayon_camerad"):
        assert not any(child is not self and child.name in ("camerad", "wayon_camerad") and child.is_alive()
                       for child in state.children), "both camera children are alive"

    def is_alive(self):
      return self.exitcode is None

    def join(self):
      state.joins.append(self.pid)
      assert self.exitcode is not None, "fake child did not exit after SIGKILL"

  def kill(pid, sig):
    child = next(child for child in state.children if child.pid == pid)
    state.signals.append((child.name, sig))
    if sig == signal.SIGKILL:
      child.code = -signal.SIGKILL
    elif sig == signal.SIGINT and not child.ignore_interrupt:
      child.exit_at = state.now + 0.2

  scope = {
    "ABC": ABC, "abstractmethod": abstractmethod, "Process": Child,
    "time": SimpleNamespace(monotonic=lambda: state.now, sleep=advance),
    "os": SimpleNamespace(path=os.path, kill=kill), "signal": signal, "BASEDIR": "/fake",
    "cloudlog": SimpleNamespace(info=lambda *args: None, error=lambda *args: None),
    "flush_crash_reports": lambda: None, "report_process_exit": lambda *args: state.reports.append(args),
    "nativelauncher": lambda *args: None, "WEBCAM": False,
    "wayon_camera_requested": camera.wayon_camera_requested,
    "wayon_panorama_requested": camera.wayon_panorama_requested,
  }
  monkeypatch.setattr(camera, "camera_request", lambda: state.request)
  manager_dir = Path(__file__).parent / "manager"
  # Execute the production lifecycle and registrations, without native imports or child execution.
  for filename, names in (
    ("process.py", {"join_process", "ManagerProcess", "NativeProcess", "ensure_running"}),
    ("process_config.py", {"driverview", "normal_camera", "remote_camera", "CameraProcess"}),
  ):
    tree = ast.parse((manager_dir / filename).read_text())
    nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    assert {node.name for node in nodes} == names
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(manager_dir / filename), "exec",
                 flags=__future__.annotations.compiler_flag), scope)
  registrations = next(node.value for node in tree.body if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Name) and target.id == "procs" for target in node.targets))
  names = {"wayon_encoderd", "wayon_camerad", "camerad"}
  calls = [node for node in registrations.elts if isinstance(node, ast.Call) and node.args[0].value in names]
  processes = eval(compile(ast.fix_missing_locations(ast.Expression(ast.List(elts=calls, ctx=ast.Load()))),
                           str(manager_dir / "process_config.py"), "eval"), scope)
  state.owners = scope["managed_processes"] = {process.name: process for process in processes}
  assert list(state.owners) == ["wayon_encoderd", "wayon_camerad", "camerad"]
  state.params = Params()
  state.advance = advance
  state.tick = lambda started=False: scope["ensure_running"](state.owners.values(), started, state.params, None)
  return state


def test_manager_offroad_start_is_exclusive_and_idempotent(manager_lifecycle):
  state = manager_lifecycle
  assert [process.name for process in state.tick()] == ["wayon_encoderd", "wayon_camerad"]
  assert state.owners["wayon_encoderd"].sigkill is True
  assert state.owners["camerad"].proc is None
  state.tick()
  assert [child.name for child in state.children] == ["wayon_encoderd", "wayon_camerad"]
  assert not state.signals
  assert state.params.values == {"IsOffroad": True}


@pytest.mark.parametrize("ignore_interrupt", [False, True])
def test_manager_onroad_waits_for_camera_exit_and_cleanup(manager_lifecycle, ignore_interrupt):
  state = manager_lifecycle
  state.tick()
  remote = state.owners["wayon_camerad"].proc
  remote.ignore_interrupt = ignore_interrupt
  assert state.tick(started=True) == []
  assert state.signals == [("wayon_encoderd", signal.SIGKILL), ("wayon_camerad", signal.SIGINT)]
  assert state.now == 100.0
  assert state.owners["wayon_camerad"].proc is remote
  assert state.owners["camerad"].proc is None
  state.advance(0.3)
  assert [process.name for process in state.tick(started=True)] == ["camerad"]
  assert state.owners["wayon_camerad"].proc is None
  assert state.owners["wayon_encoderd"].proc is None
  assert state.owners["camerad"].proc.is_alive()
  if ignore_interrupt:
    assert state.now >= 105.3
    assert state.signals[-1] == ("wayon_camerad", signal.SIGKILL)
    assert state.joins == [remote.pid]
  assert not state.reports


def test_manager_driver_view_flip_between_predicates_cannot_launch_two_cameras(manager_lifecycle):
  state = manager_lifecycle

  class FlippingParams(Params):
    reads = 0

    def get_bool(self, key):
      if key == "IsDriverViewEnabled":
        self.reads += 1
        self.values[key] = self.reads >= 2
      return super().get_bool(key)

  state.params = FlippingParams()
  assert [process.name for process in state.tick()] == ["wayon_encoderd", "wayon_camerad", "camerad"]
  assert state.owners["wayon_camerad"].proc.is_alive()
  assert state.owners["camerad"].proc is None
  state.request = None
  state.tick()
  assert state.owners["camerad"].proc is None
  state.advance(0.3)
  state.tick()
  assert state.owners["wayon_camerad"].proc is None
  assert state.owners["wayon_encoderd"].proc is None
  assert state.owners["camerad"].proc.is_alive()
  assert state.params.values == {"IsOffroad": True, "IsDriverViewEnabled": True}


@pytest.mark.parametrize("started", [False, True])
def test_manager_failed_remote_child_reaped_before_restart_or_handover(manager_lifecycle, started):
  state = manager_lifecycle
  state.tick()
  remote = state.owners["wayon_camerad"].proc
  remote.code = 7
  state.tick(started=started)
  assert state.reports == [("wayon_camerad", remote.pid, 7)]
  owner = state.owners["camerad" if started else "wayon_camerad"]
  assert owner.proc is not remote and owner.proc.is_alive()
  assert state.owners["wayon_camerad" if started else "camerad"].proc is None
  state.tick(started=started)
  assert len(state.reports) == 1


def test_manager_unreaped_normal_child_blocks_remote_until_next_tick(manager_lifecycle):
  state = manager_lifecycle
  state.tick(started=True)
  normal = state.owners["camerad"].proc
  normal.code = 9
  state.tick()
  assert state.owners["camerad"].proc is None
  assert state.owners["wayon_camerad"].proc is None
  assert state.reports == [("camerad", normal.pid, 9)]
  state.tick()
  assert state.owners["wayon_camerad"].proc.is_alive()
