"""Wayon Traverse display parity. Real CAN codec/KF/schema; no device I/O."""
import ast
from collections import defaultdict
from pathlib import Path
import sys
from types import MethodType, ModuleType, SimpleNamespace as NS

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm import cluster_speed
from opendbc.car.gm.values import CAR, DBC, GMFlags


@pytest.mark.parametrize("raw,display", [(0, 0), (-2, -2), (24, 25), (25, 26), (28, 29),
                                       (36, 38), (56, 60), (66, 70), (95, 100), (97.5, 102.5), (154, 160)])
def test_wayon_display_mapping(raw, display):
  assert cluster_speed.gm_cluster_display_kph_from_raw_display_kph(raw) == pytest.approx(display)
  assert cluster_speed.gm_cluster_cruise_speed_from_raw_ms(raw / 3.6) * 3.6 == pytest.approx(display)


@pytest.mark.parametrize("display,raw", cluster_speed._CLUSTER_TO_RAW_DISPLAY_TABLE)
def test_every_wayon_inverse_sample(display, raw):
  assert cluster_speed.gm_raw_display_kph_from_cluster_display_kph(display) == pytest.approx(raw)
  assert cluster_speed.gm_raw_cruise_speed_from_cluster_ms(display / 3.6) * 3.6 == pytest.approx(raw)


class MemoryParams:
  def __init__(self, *args):
    self.values = {"CruiseSpeedUnit": 10, "CruiseSpeedUnitBasic": 1, "CruiseButtonLongDelay": 50,
                   "AutoNaviSpeedSafetyFactor": 100, "AutoGasTokSpeed": 30, "AutoGasCancelSpeed": 30,
                   "LongActuatorDelay": 50, "SpeedFromPCM": 2}

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_int_nonblocking(self, key, value):
    self.values[key] = int(value)

  def put_bool_nonblocking(self, key, value):
    self.values[key] = bool(value)


@pytest.fixture(scope="module")
def runtime():
  # Desktop checkout need not build Params or open a hardware/IPC connection.
  # Only I/O dependencies are replaced; parser, state, KF and controller run whole.
  with pytest.MonkeyPatch.context() as patch:
    params = ModuleType("openpilot.common.params")
    params.Params = MemoryParams
    patch.setitem(sys.modules, params.__name__, params)
    realtime = ModuleType("openpilot.common.realtime")
    realtime.DT_CTRL, realtime.DT_MDL = 0.01, 0.05
    patch.setitem(sys.modules, realtime.__name__, realtime)
    from opendbc.can import CANPacker, CANParser
    import opendbc.car.gm.carstate as state
    import opendbc.car.gm.carcontroller as controller
    import opendbc.car.gm.interface as interface
    import opendbc.car.interfaces as interfaces
    import openpilot.selfdrive.car.cruise as cruise
    for module in (state, controller, interface, interfaces, cruise):
      patch.setattr(module, "Params", MemoryParams)
    patch.setattr(cruise, "CommandReader", lambda *args: NS(read=lambda **kwargs: None, is_repeat=False))
    yield NS(state=state, controller=controller, interface=interface, cruise=cruise,
             CANPacker=CANPacker, CANParser=CANParser)


def build_cp(runtime, candidate=CAR.CHEVROLET_TRAVERSE, alpha=True):
  return runtime.interface.CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, True, False)


def test_traverse_preserves_installed_physical_baseline(runtime):
  cp = build_cp(runtime)
  assert CAR.CHEVROLET_TRAVERSE.config.specs.mass == 2090
  assert cp.mass == pytest.approx(2090 + 136)
  assert cp.centerToFront / cp.wheelbase == pytest.approx(.7)


def parser_state(runtime, candidate=CAR.CHEVROLET_TRAVERSE, flags=0):
  cp = build_cp(runtime, candidate)
  cp.flags |= int(flags)
  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  ci.CS.update(ci.can_parsers)  # Register lazily accessed messages before input.
  return ci, runtime.CANPacker(DBC[candidate][Bus.pt])


def frames(packer, raw_kph=95., set_kph=95., reverse=False):
  return [
    packer.make_can_msg("EBCMWheelSpdFront", 0, {"FLWheelSpd": raw_kph, "FRWheelSpd": raw_kph}),
    packer.make_can_msg("EBCMWheelSpdRear", 0, {"RLWheelSpd": raw_kph, "RRWheelSpd": raw_kph,
                                              "RLWheelDir": 2 if reverse else 0, "RRWheelDir": 2 if reverse else 0}),
    packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {"ACCSpeedSetpoint": set_kph}),
  ]


@pytest.mark.parametrize("flags", [0, GMFlags.SPEED_RELATED_MSG])
def test_real_can_interface_corrects_display_not_motion(runtime, flags):
  ci, packer = parser_state(runtime, flags=flags)
  ret = ci.update([(1_000_000_000, frames(packer))])
  assert ret.vEgo * 3.6 == pytest.approx(95, abs=.0311)
  assert ret.vEgoRaw == pytest.approx(ret.vEgo)
  assert ret.aEgo == pytest.approx(0)
  assert ret.wheelSpeeds.fl == pytest.approx(ret.vEgo)
  # Existing +/-0.5 km/h interface hysteresis is deliberately retained.
  assert round(ret.vEgoCluster * 3.6) == 100
  assert ret.cruiseState.speed * 3.6 == pytest.approx(95)
  assert ret.cruiseState.speedCluster * 3.6 == pytest.approx(100)
  assert not ret.standstill
  assert ret.vCluRatio == pytest.approx(.96 if flags else 0.)


def test_zero_and_reverse_do_not_gain_display_offset(runtime):
  ci, packer = parser_state(runtime)
  zero = ci.update([(1_000_000_000, frames(packer, 0, 0))])
  assert zero.vEgoCluster == zero.cruiseState.speedCluster == 0
  reverse = ci.update([(2_000_000_000, frames(packer, 30, 0, reverse=True))])
  assert reverse.vEgo < 0
  assert reverse.vEgoCluster <= 0
  assert reverse.cruiseState.speedCluster == 0


def test_other_camera_gm_display_unchanged(runtime):
  ci, packer = parser_state(runtime, CAR.CHEVROLET_BOLT_EUV)
  ret = ci.update([(1_000_000_000, frames(packer))])
  assert round(ret.vEgoCluster * 3.6) == 95
  assert ret.cruiseState.speedCluster == ret.cruiseState.speed


@pytest.mark.parametrize("candidate", [CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_VOLT])
def test_state_trajectory_changes_only_traverse_display_fields(runtime, candidate):
  changed, packer = parser_state(runtime, candidate)
  original, _ = parser_state(runtime, candidate)
  tree = ast.parse(Path(runtime.state.__file__).read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "CarState")
  update = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "update")
  update.body = [node for node in update.body if not (isinstance(node, ast.If) and
                 ast.unparse(node.test) == "self.CP.carFingerprint == CAR.CHEVROLET_TRAVERSE")]
  namespace = vars(runtime.state).copy()
  exec(compile(ast.Module(body=[update], type_ignores=[]), "before-display-port", "exec"), namespace)
  original.CS.update = MethodType(namespace["update"], original.CS)
  samples = [0.] * 10 + [i * .5 for i in range(240)] + [120. - i * .5 for i in range(240)] + [0.] * 100
  for tick, raw_speed in enumerate(samples):
    packets = [(1_000_000_000 + tick * 10_000_000, frames(packer, raw_speed, 95.))]
    actual, before = changed.update(packets).to_dict(), original.update(packets).to_dict()
    if candidate == CAR.CHEVROLET_TRAVERSE:
      actual.pop("vEgoCluster")
      before.pop("vEgoCluster")
      actual["cruiseState"].pop("speedCluster")
      before["cruiseState"].pop("speedCluster")
    assert actual == before


@pytest.mark.parametrize("pcm", [False, True])
@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("metric", [False, True])
@pytest.mark.parametrize("candidate", [CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_BOLT_EUV])
def test_carrot_cruise_publishes_stock_display_without_rescaling_target(runtime, pcm, mode, metric, candidate):
  cp = build_cp(runtime, candidate, alpha=not pcm)
  helper = runtime.cruise.VCruiseCarrot(cp)
  helper.params.values["SpeedFromPCM"] = mode
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 80.
  helper.cruise_state_available_last = True
  helper._gas_pressed_count = helper._brake_pressed_count = -2
  cs = structs.CarState(gearShifter="drive", vEgo=70/3.6, vEgoCluster=75/3.6,
                       cruiseState={"available": True, "enabled": True, "speed": 95/3.6, "speedCluster": 100/3.6})
  sm = {"carControl": structs.CarControl(enabled=True), "carrotMan": NS()}
  class Messages(dict):
    alive = dict.fromkeys(("carrotMan", "longitudinalPlan", "radarState", "drivingModelData"), False)
    valid = alive
    seen = alive
  helper.update_v_cruise(cs, Messages(sm), metric)
  stock_display = pcm and (candidate == CAR.CHEVROLET_TRAVERSE or mode == 1)
  assert helper.v_cruise_cluster_kph == pytest.approx(100 if stock_display else 80)
  assert helper.v_cruise_kph == pytest.approx(95 if pcm and mode == 1 else 80)


def controller_state():
  out = structs.CarState(vEgo=10., standstill=False)
  out.cruiseState.enabled = True
  return NS(out=out, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
            cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=0,
            pcm_acc_status=0, pscm_status=defaultdict(int))


@pytest.mark.parametrize("active", [False, True])
@pytest.mark.parametrize("accel", [-3., 0., 1.])
@pytest.mark.parametrize("set_kph", [0., 70., 100., 255.])
def test_dashboard_only_can_difference(runtime, monkeypatch, active, accel, set_kph):
  cp = build_cp(runtime)
  changed = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  original = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  cc = structs.CarControl(enabled=active, longActive=active)
  cc.actuators.accel = accel
  cc.actuators.longControlState = "pid" if active else "off"
  cc.hudControl.setSpeed = set_kph / 3.6
  state = controller_state()
  parser = runtime.CANParser(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt], [("ASCMActiveCruiseControlStatus", 25)], 0)
  baseline_parser = runtime.CANParser(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt], [("ASCMActiveCruiseControlStatus", 25)], 0)
  corrected_speed = runtime.controller.get_acc_dashboard_speed_kph
  for tick in range(100):
    now = 1_000_000_000 + tick * 10_000_000
    output, sends = changed.update(cc.as_reader(), state, now)
    monkeypatch.setattr(runtime.controller, "get_acc_dashboard_speed_kph", lambda cp, speed: speed)
    baseline, old_sends = original.update(cc.as_reader(), state, now)
    monkeypatch.setattr(runtime.controller, "get_acc_dashboard_speed_kph", corrected_speed)
    assert output.to_dict() == baseline.to_dict()
    assert [(a, d, b) for a, d, b in sends if a != 0x370] == [(a, d, b) for a, d, b in old_sends if a != 0x370]
    dashboards = [msg for msg in sends if msg[0] == 0x370]
    if dashboards:
      parser.update([(now, dashboards)])
      baseline_parser.update([(now, [msg for msg in old_sends if msg[0] == 0x370])])
      expected = cluster_speed.gm_raw_display_kph_from_cluster_display_kph(set_kph if set_kph < 252 else 0)
      assert parser.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] == pytest.approx(expected)
      actual = dict(parser.vl["ASCMActiveCruiseControlStatus"])
      before = dict(baseline_parser.vl["ASCMActiveCruiseControlStatus"])
      actual.pop("ACCSpeedSetpoint")
      before.pop("ACCSpeedSetpoint")
      assert actual == before


def test_non_traverse_dashboard_passthrough(runtime):
  for candidate in CAR:
    if candidate != CAR.CHEVROLET_TRAVERSE:
      assert runtime.controller.get_acc_dashboard_speed_kph(NS(carFingerprint=candidate), 100.) == 100.


def test_controlsd_hud_selects_set_not_transient_slowdown():
  # Execute the exact display-selection block without starting controls/modeld.
  root = Path(__file__).resolve().parents[5]
  tree = ast.parse((root / "openpilot/selfdrive/controls/controlsd.py").read_text())
  block = next(node for node in ast.walk(tree) if isinstance(node, ast.If) and
               ast.unparse(node.test) == "self.CP.carFingerprint == GM_CAR.CHEVROLET_TRAVERSE")
  code = compile(ast.Module(body=[block], type_ignores=[]), "controlsd-display", "exec")
  for pcm in (True, False):
    for mode in range(4):
      for x_state in (0, 3):
        hud = NS()
        namespace = dict(self=NS(CP=NS(carFingerprint=CAR.CHEVROLET_TRAVERSE, pcmCruise=pcm),
                                 params=NS(get_int=lambda key: mode)), CS=NS(vCruiseCluster=100),
                         lp=NS(xState=x_state), hudControl=hud, GM_CAR=CAR, CV=CV,
                         desired_kph=50, setSpeed=40 / 3.6)
        exec(code, namespace)
        assert hud.setSpeed * 3.6 == pytest.approx(100)


def test_controlsd_non_traverse_selection_unchanged():
  root = Path(__file__).resolve().parents[5]
  tree = ast.parse((root / "openpilot/selfdrive/controls/controlsd.py").read_text())
  block = next(node for node in ast.walk(tree) if isinstance(node, ast.If) and
               ast.unparse(node.test) == "self.CP.carFingerprint == GM_CAR.CHEVROLET_TRAVERSE")
  code = compile(ast.Module(body=[block], type_ignores=[]), "controlsd-display", "exec")
  old_code = compile(ast.Module(body=block.orelse, type_ignores=[]), "controlsd-before-display", "exec")
  for pcm in (True, False):
    for mode in range(4):
      for x_state in (0, 3):
        namespace = dict(self=NS(CP=NS(carFingerprint=CAR.CHEVROLET_BOLT_EUV, pcmCruise=pcm),
                                 params=NS(get_int=lambda key: mode)), CS=NS(vCruiseCluster=100),
                         lp=NS(xState=x_state), hudControl=NS(), GM_CAR=CAR, CV=CV,
                         desired_kph=50, setSpeed=40 / 3.6)
        exec(old_code, namespace)
        before = namespace["hudControl"].setSpeed
        exec(code, namespace)
        assert namespace["hudControl"].setSpeed == before
