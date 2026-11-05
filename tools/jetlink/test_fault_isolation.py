"""Failure isolation without publishing messages or touching vehicle controls."""
import ast
from collections import defaultdict
from functools import cache
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
SELFDRIVED = ROOT / 'openpilot/selfdrive/selfdrived/selfdrived.py'


def ignored_processes(device='mici', nvme=False):
  # Exercise the constructor's exact policy without opening IPC or real Params.
  tree = ast.parse(SELFDRIVED.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
  nodes = [n for n in init.body if 'self.ignored_processes' in ast.unparse(n)]
  context = {'self': NS(), 'HARDWARE': NS(get_device_type=lambda: device), 'os': NS(path=NS(exists=lambda _: nvme))}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SELFDRIVED), 'exec'), context)
  return context['self'].ignored_processes


@pytest.mark.parametrize('device,nvme,expected', [
  ('mici', False, {'jetson_maintenance', 'comma_remote', 'carrot_cluster', 'navdy_bridge', 'mapsd'}),
  ('tici', False, {'jetson_maintenance', 'comma_remote', 'carrot_cluster', 'navdy_bridge', 'mapsd'}),
  ('tici', True, {'jetson_maintenance', 'comma_remote', 'carrot_cluster', 'navdy_bridge', 'mapsd', 'loggerd'}),
])
def test_only_non_driving_helpers_are_added_to_existing_exception_policy(device, nvme, expected):
  assert ignored_processes(device, nvme) == expected


@cache
def validity_expression(function='fill_model_msg'):
  tree = ast.parse((ROOT / 'openpilot/selfdrive/modeld/modeld.py').read_text())
  call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == function)
  assert isinstance(call.args[-1], ast.Name) and call.args[-1].id == 'model_output_valid'
  assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'model_output_valid' for t in n.targets))
  return compile(ast.Expression(assignment.value), '<model-validity>', 'eval')


def output_valid(model, calibrated=True):
  return eval(validity_expression(), {'model': model, 'live_calib_seen': calibrated})


@pytest.mark.parametrize('function', ['fill_model_msg', 'fill_pose_msg'])
def test_published_model_and_pose_keep_calibration_gate(function):
  for calibrated in (False, True):
    scope = {'live_calib_seen': calibrated, 'model': NS()}
    assert eval(validity_expression(function), scope) is calibrated
    for valid in (False, True):
      scope['model'] = NS(output_valid=valid)
      assert eval(validity_expression(function), scope) is (calibrated and valid)


def failed_active_model(monkeypatch, fault):
  import numpy as np
  from openpilot.selfdrive.modeld.jetlink import model
  from test_hotplug_resources import joining_model
  m = joining_model(monkeypatch)
  m.active = True
  def fail(*args, **kwargs):
    raise ConnectionError('external inference disconnected')
  m.client = NS(infer=fail, close=lambda: None)
  m.small.reset = lambda: None
  monkeypatch.setattr(model, 'FAULT', fault)
  monkeypatch.setattr(model.time, 'monotonic', lambda: 100.)
  inputs = {'desire_pulse': np.zeros(8), 'traffic_convention': [1, 0], 'action_t': [.2, .3]}
  return m, inputs


def test_active_model_failure_retains_native_fallback_and_five_second_fault(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld.jetlink import link
  fault = tmp_path / 'fault'
  m, inputs = failed_active_model(monkeypatch, fault)
  monkeypatch.setattr(link, 'FAULT', fault)
  assert m.run({}, {}, inputs, False) == 'local'
  assert not m.active and m.client is None and m.next_join == 105.
  for now, expected in ((99.9, False), (100., True), (104.999, True), (105., False)):
    monkeypatch.setattr(link.time, 'monotonic', lambda: now)
    assert link.fault_active() is expected
    assert output_valid(m) is (now >= 105.)


def test_fault_marker_write_failure_must_not_abort_native_fallback(monkeypatch, tmp_path):
  m, inputs = failed_active_model(monkeypatch, tmp_path)  # Directory cannot be written as a marker.
  assert m.run({}, {}, inputs, False) == 'local'
  assert not m.output_valid and m.fault_until == 105.


def test_active_failure_must_remain_visible_when_fault_marker_cannot_be_read(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld.jetlink import link
  m, inputs = failed_active_model(monkeypatch, tmp_path / 'fault')
  assert m.run({}, {}, inputs, False) == 'local'
  monkeypatch.setattr(link, 'FAULT', tmp_path)
  assert link.fault_active() or not output_valid(m)


@pytest.mark.parametrize('active', [False, True])
def test_client_close_failure_cannot_abort_native_fallback_or_clear_fault(monkeypatch, tmp_path, active):
  m, inputs = failed_active_model(monkeypatch, tmp_path)
  m.active = active
  resets = []
  m.small.reset = lambda: resets.append(True)
  def close():
    raise OSError('socket cleanup failed')
  m.client.close = close
  assert m.run({}, {}, inputs, False) == 'local'
  assert m.client is None and not m.active
  assert not output_valid(m)
  assert resets == ([True] if active else [])


def test_first_inference_timeout_also_invalidates_late_local_result(monkeypatch, tmp_path):
  m, inputs = failed_active_model(monkeypatch, tmp_path / 'fault')
  m.active = False
  assert m.run({}, {}, inputs, False) == 'local'
  assert not m.output_valid and m.fault_until == 105.
  assert (tmp_path / 'fault').is_file()


def test_unwritable_status_cannot_abort_following_frames_or_clear_ipc_fault(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld.jetlink import model
  m, inputs = failed_active_model(monkeypatch, tmp_path)
  assert m.run({}, {}, inputs, False) == 'local'
  m.phase_enabled = False
  monkeypatch.setattr(model, 'MODEL_STATUS', tmp_path / 'missing/status.json')
  monkeypatch.setattr(model, 'state', lambda: {})
  monkeypatch.setattr(model, 'may_join', lambda *a: False)
  class Messages(dict):
    pass
  sm = Messages(carState=None, selfdriveState=None, carControl=None)
  sm.valid = sm.alive = dict.fromkeys(sm, True)
  sm.recv_time = {}
  m.update(sm, NS())
  assert m.next_status == 101. and not m.output_valid
  assert m.run({}, {}, inputs, False) == 'local'
  assert not output_valid(m)


@pytest.mark.parametrize('valid,dropped', [(False, 0), (True, 0), (True, 1)])
def test_real_model_pose_and_driving_messages_preserve_fault_and_camera_gates(valid, dropped):
  import numpy as np
  from openpilot.cereal import log
  from openpilot.selfdrive.modeld.jetlink.contracts import parse_outputs
  from openpilot.selfdrive.modeld.jetlink.link import SPEC
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, fill_pose_msg, PublishState
  output = parse_outputs(Parser(), SPEC, np.zeros(SPEC.output_nelem, np.float32))
  model, driving, pose = (log.Event.new_message() for _ in range(3))
  model.init('modelV2')
  driving.init('drivingModelData')
  pose.init('cameraOdometry')
  fill_model_msg(model, output, log.ModelDataV2.Action.new_message(), PublishState(), 1, 1, 1, 0., 1, .01, valid)
  fill_driving_model_data(driving, model)
  fill_pose_msg(pose, output, 1, dropped, 1, valid)
  for message, expected in ((model, valid), (driving, valid), (pose, valid and dropped == 0)):
    with log.Event.from_bytes(message.to_bytes()) as decoded:
      assert decoded.valid is expected


def make_selfdrived(monkeypatch):
  # These integration tests run on comma, where the native IPC module is built.
  # All messages remain in memory; no SubMaster, PubMaster, or Params is created.
  pytest.importorskip('msgq.ipc_pyx')
  from openpilot.cereal import car, log, messaging
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.events import Events

  class Messages(dict):
    def all_alive(self, services=None): return all(self.alive[s] for s in self if services is None or s in services)
    def all_freq_ok(self, services=None): return all(self.freq_ok[s] for s in self if services is None or s in services)
    def all_checks(self, services=None):
      return (self.all_alive(services) and self.all_freq_ok(services)
              and all(self.valid[s] for s in self if services is None or s in services))

  sm = Messages()
  for name in ('controlsState', 'deviceState', 'peripheralState', 'liveCalibration', 'modelV2',
               'carrotMan', 'carControl', 'longitudinalPlan', 'livePose', 'radarState', 'driverAssistance',
               'driverMonitoringState', 'liveParameters'):
    sm[name] = getattr(messaging.new_message(name), name)
  sm['pandaStates'] = []
  sm['managerState'] = NS(processes=[], rebootRequired=False)
  sm['controlsState'].lateralControlState.init('pidState')
  sm['deviceState'].freeSpacePercent = 90
  sm['liveCalibration'].calStatus = log.LiveCalibrationData.Status.calibrated
  sm.frame = 100
  sm.valid = defaultdict(lambda: True)
  sm.alive = defaultdict(lambda: True)
  sm.freq_ok = defaultdict(lambda: True)
  sm.updated = defaultdict(lambda: False)
  sm.recv_frame = defaultdict(lambda: 1)
  sm.recv_frame['alertDebug'] = 0
  sm.seen = defaultdict(lambda: False)
  monkeypatch.setattr(cloudlog, 'event', lambda *a, **kw: None)

  sd = object.__new__(SelfdriveD)
  sd.sm, sd.events = sm, Events()
  sd.CP = car.CarParams.new_message(notCar=True, pcmCruise=True)
  sd.CS_prev = car.CarState.new_message()
  sd.car_events = NS(update=lambda *a: Events())
  sd.params = NS(get_int=lambda _: 0, get_bool=lambda _: False)
  sd.initialized = sd.enabled = True
  sd.startup_event = sd.calibrated_pose = sd.not_running_prev = sd.logged_comm_issue = None
  sd.is_ldw_enabled = sd.excessive_actuation = False
  sd.mismatch_counter = sd.cruise_mismatch_counter = sd.last_steering_pressed_frame = 0
  sd.last_functional_fan_frame = sd.distance_traveled = 0
  sd.atc_type_last = ''
  sd._manager_frame, sd._not_running = -1, set()
  sd.big_model_loading = sd.big_model_active = sd.model_startup_complete = False
  sd.big_model_ready_t = float('-inf')
  sd.dm_disabled_prev = sd.update_reboot_alerted = False
  sd.cutin_audio_tracker = NS(update=lambda *a: False)
  sd.radar_only_warning = NS(update_live=lambda *a: False)
  sd.phone_forward_risk = NS(update_live=lambda *a, **kw: False)
  sd.car_state_time_s = 0.
  sd.sensor_packets = sd.camera_packets = []
  sd.gps_location_service = 'gpsLocationExternal'
  sd.ignored_processes = ignored_processes()
  sd.rk = NS(lagging=False)
  cs = car.CarState.new_message(canValid=True, standstill=True)
  return sd, cs


@pytest.mark.parametrize('failed,critical', [
  ({'comma_remote'}, False), ({'jetson_maintenance'}, False),
  ({'comma_remote', 'jetson_maintenance'}, False),
  ({'carrot_cluster'}, False), ({'carrot_cluster', 'modeld'}, True),
  ({'navdy_bridge'}, False), ({'navdy_bridge', 'modeld'}, True),
  ({'jetlinkd'}, True), ({'modeld'}, True), ({'controlsd'}, True), ({'pandad'}, True),
  ({'comma_remote', 'modeld'}, True),
])
def test_real_selfdrived_process_events_and_disengagement(monkeypatch, failed, critical):
  sd, cs = make_selfdrived(monkeypatch)
  from openpilot.cereal import log
  from openpilot.selfdrive.modeld.jetlink import link
  from openpilot.selfdrive.selfdrived.state import StateMachine, SOFT_DISABLE_TIME
  from openpilot.common.realtime import DT_CTRL
  monkeypatch.setattr(link, 'fault_active', lambda: False)
  sd.sm['managerState'].processes = [NS(name=name, running=False, shouldBeRunning=True) for name in failed]
  sd.update_events(cs)
  assert (log.OnroadEvent.EventName.processNotRunning in sd.events.names) is critical
  machine = StateMachine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  for _ in range(int(SOFT_DISABLE_TIME / DT_CTRL) + 2):
    machine.update(sd.events)
  assert machine.state == (log.SelfdriveState.OpenpilotState.disabled if critical else log.SelfdriveState.OpenpilotState.enabled)


def test_faulted_model_ipc_still_disengages_when_marker_cannot_be_read(monkeypatch, tmp_path):
  sd, cs = make_selfdrived(monkeypatch)
  from openpilot.cereal import log
  from openpilot.selfdrive.modeld.jetlink import link
  import openpilot.selfdrive.selfdrived.selfdrived as selfdrived
  from openpilot.selfdrive.selfdrived.state import StateMachine, SOFT_DISABLE_TIME
  from openpilot.common.realtime import DT_CTRL
  monkeypatch.setattr(link, 'FAULT', tmp_path)  # Unreadable as a file.
  assert not link.fault_active()
  m, inputs = failed_active_model(monkeypatch, tmp_path)
  assert m.run({}, {}, inputs, False) == 'local'
  now = [100.]
  monkeypatch.setattr(link.time, 'monotonic', lambda: now[0])
  monkeypatch.setattr(selfdrived, 'communication_snapshot', lambda *a: {})
  machine = StateMachine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  for step in range(int(SOFT_DISABLE_TIME / DT_CTRL) + 2):
    now[0] = 100. + step * DT_CTRL
    sd.sm.valid['modelV2'] = output_valid(m)
    sd.update_events(cs)
    assert log.OnroadEvent.EventName.commIssue in sd.events.names
    machine.update(sd.events)
  assert machine.state == log.SelfdriveState.OpenpilotState.disabled
  now[0] = m.fault_until
  sd.sm.valid['modelV2'] = output_valid(m)
  sd.update_events(cs)
  assert log.OnroadEvent.EventName.commIssue not in sd.events.names
  machine.update(sd.events)
  assert machine.state == log.SelfdriveState.OpenpilotState.disabled  # No automatic re-engagement.
