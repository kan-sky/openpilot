"""Run production consumer methods with native solver/IPC boundaries isolated."""
import ast
import capnp
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from openpilot.common.tests.test_vehicle_dynamics import car_params, messages
from openpilot.common.vehicle_dynamics import VehicleDynamics, read_live_vehicle_dynamics
from openpilot.cereal import log

ROOT = Path(__file__).resolve().parents[3]


def method(path, class_name, method_name, namespace, *, stop_before=None):
  tree = ast.parse(path.read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  fn = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)
  if stop_before:
    # Execute the complete geometry prologue; no native MPC solver in this test.
    stop = next(i for i, node in enumerate(fn.body) if ast.unparse(node).startswith(stop_before))
    fn.body = fn.body[:stop]
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), namespace)
  return namespace[method_name]


def test_lateral_planner_refreshes_factors_and_holds_on_invalid_snapshot():
  cp = car_params()
  dynamics = VehicleDynamics.from_car_params(cp).scaled(2226, 2.149)
  sm = messages(cp, dynamics)
  planner = NS(CP=cp)
  update = method(ROOT / "selfdrive/controls/lib/lateral_planner.py", "LateralPlanner", "update",
                  {"read_live_vehicle_dynamics": read_live_vehicle_dynamics}, stop_before="self.readParams")
  update(planner, sm, None)
  assert planner.factor1 == cp.wheelbase - dynamics.centerToFront
  assert planner.factor2 == dynamics.centerToFront * dynamics.mass / (cp.wheelbase * dynamics.tireStiffnessRear)
  before = (planner.factor1, planner.factor2)
  sm.checks = False
  sm["liveParameters"].vehicleDynamics.centerToFront = 1.0
  update(planner, sm, None)
  assert (planner.factor1, planner.factor2) == before


def test_learner_updates_all_globals_without_resetting_learned_state():
  cp = car_params()
  base = VehicleDynamics.from_car_params(cp)
  target = base.scaled(2226, 2.149)
  writes = []
  history_resets = []
  state = np.arange(9, dtype=float)
  covariance = np.eye(9)
  learner = NS(dynamics=base, kf=NS(
    x=state, P=covariance, set_globals=lambda **kw: writes.append(kw),
    filter=NS(reset_rewind=lambda: history_resets.append(True)),
  ))
  update = method(ROOT / "selfdrive/locationd/paramsd.py", "VehicleParamsLearner", "update_dynamics", {})
  update(learner, target)
  update(learner, target)
  assert writes == [target.kalman_globals()]
  assert len(history_resets) == 1
  assert learner.kf.x is state and learner.kf.P is covariance
  assert learner.dynamics == target


def test_real_learner_message_contains_exact_physical_snapshot():
  states = NS(STEER_RATIO=0, STIFFNESS=1, ANGLE_OFFSET=2, ANGLE_OFFSET_FAST=3, ROAD_ROLL=4, YAW_RATE=5)
  dynamics = VehicleDynamics.from_car_params(car_params()).scaled(2226, 2.149)

  def new_message(service):
    msg = log.Event.new_message()
    msg.init(service)
    return msg

  ns = dict(np=np, capnp=capnp, States=states, messaging=NS(new_message=new_message),
            MAX_ANGLE_OFFSET_DELTA=1., ROLL_MAX_DELTA=.01, LOW_ACTIVE_SPEED=10.,
            OFFSET_MAX=10., OFFSET_LOWERED_MAX=8., ROLL_MAX=.17, ROLL_LOWERED_MAX=.14,
            ROLL_STD_MAX=.026, check_valid_with_hysteresis=lambda current, value, limit, lower: abs(value) < limit)
  get_msg = method(ROOT / "selfdrive/locationd/paramsd.py", "VehicleParamsLearner", "get_msg", ns)
  learner = NS(
    kf=NS(x=np.array([17.9, 1., 0., 0., 0., 0.]), P=np.eye(6) * .0001),
    dynamics=dynamics, avg_angle_offset=0., angle_offset=0., roll=0., active=False,
    avg_offset_valid=True, total_offset_valid=True, roll_valid=True, min_sr=8., max_sr=36.,
  )
  msg = get_msg(learner, True)
  with log.Event.from_bytes(msg.to_bytes()) as received:
    sm = messages(car_params())
    sm["liveParameters"] = received.liveParameters
    assert read_live_vehicle_dynamics(sm, car_params()) == dynamics
