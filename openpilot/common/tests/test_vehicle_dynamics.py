from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from opendbc.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from opendbc.car.vehicle_model import VehicleModel
from openpilot.cereal import car, log
from openpilot.common.vehicle_dynamics import (
  CENTER_RATIO_RATE_S, MASS_RATE_KG_S, LiveVehicleDynamics, VehicleDynamics,
  initialize_vehicle_specs, read_live_vehicle_dynamics, validate_vehicle_spec, vehicle_spec_defaults,
)


def car_params(mass=1955, ratio=0.4):
  mass += STD_CARGO_KG
  front = 3.07 * ratio
  stiffness = scale_tire_stiffness(mass, 3.07, front, 0.8)
  return car.CarParams.new_message(
    mass=mass, wheelbase=3.07, centerToFront=front, steerRatio=17.9,
    rotationalInertia=scale_rot_inertia(mass, 3.07),
    tireStiffnessFront=stiffness[0], tireStiffnessRear=stiffness[1],
  )


def messages(cp, dynamics=None):
  lp = log.LiveParametersData.new_message(valid=True)
  if dynamics is not None:
    lp.vehicleDynamics = dynamics.to_message()

  class Messages(dict):
    checks = True

    def all_checks(self, services):
      assert services == ["liveParameters"]
      return self.checks

  return Messages(liveParameters=lp)


@pytest.mark.parametrize("name,value", [
  ("VehicleMass", "2090"), ("VehicleMass", b"1955"),
  ("VehicleMass", 500), ("VehicleMass", 5000),
  ("VehicleCenterToFrontRatio", "0.70"), ("VehicleCenterToFrontRatio", 0.1),
  ("VehicleCenterToFrontRatio", 0.9), ("VehicleCenterToFrontRatio", 0.735),
])
def test_valid_settings(name, value):
  assert validate_vehicle_spec(name, value) == float(value)


@pytest.mark.parametrize("name", ["VehicleMass", "VehicleCenterToFrontRatio"])
@pytest.mark.parametrize("value", [None, "", True, False, "nan", "inf", "-inf", -1, 0, 1, 9, 10000, [], {}, 70.1])
def test_invalid_settings(name, value):
  with pytest.raises(ValueError):
    validate_vehicle_spec(name, value)


def test_unrelated_parameters_unchanged():
  assert validate_vehicle_spec("CustomSR", "137.2") == "137.2"


@pytest.mark.parametrize("mass,ratio", [(1955, 0.4), (2090, 0.7)])
def test_actual_defaults_and_legacy_migration_are_idempotent(mass, ratio):
  cp = car_params(mass, ratio)
  defaults = vehicle_spec_defaults(cp)
  assert defaults == {"VehicleMass": mass, "VehicleCenterToFrontRatio": ratio}
  stored = {"VehicleMass": 0, "VehicleCenterToFrontRatio": 0}
  writes = []

  def put(name, value):
    stored[name] = value
    writes.append((name, value))

  params = NS(get=stored.get, put=put)
  assert initialize_vehicle_specs(params, defaults) == defaults
  assert stored == defaults
  assert len(writes) == 2
  initialize_vehicle_specs(params, defaults)
  assert len(writes) == 2
  tuning = LiveVehicleDynamics(cp, params)
  for step in range(20):
    assert tuning.update(step / 10) == tuning.base
  stored.update(VehicleMass=2400, VehicleCenterToFrontRatio=65)
  assert initialize_vehicle_specs(params, defaults) == {"VehicleMass": 2400, "VehicleCenterToFrontRatio": 0.65}
  stored["VehicleCenterToFrontRatio"] = 0.735
  assert initialize_vehicle_specs(params, defaults)["VehicleCenterToFrontRatio"] == 0.735


def test_initialization_leaves_corruption_untouched_and_initializes_missing():
  stored = {"VehicleMass": "nan"}
  params = NS(get=stored.get, put=stored.__setitem__)
  assert initialize_vehicle_specs(params, vehicle_spec_defaults(car_params())) == {"VehicleCenterToFrontRatio": 0.4}
  assert stored["VehicleMass"] == "nan"


@pytest.mark.parametrize("mass,ratio", [(1955, 0.4), (2090, 0.7)])
def test_defaults_preserve_installed_car_params_exactly(mass, ratio):
  cp = car_params(mass, ratio)
  before = cp.to_dict()
  tuning = LiveVehicleDynamics(cp, NS(get=lambda name: b"0"))
  for step in range(100):
    assert tuning.update(step * 0.05) == VehicleDynamics.from_car_params(cp)
  assert cp.to_dict() == before


def test_live_update_ramps_recomputes_and_restores_without_restarting():
  cp = car_params()
  stored = {}
  tuning = LiveVehicleDynamics(cp, NS(get=stored.get))
  base = tuning.update(0)
  stored.update(VehicleMass=b"2090", VehicleCenterToFrontRatio=b"0.70")
  previous = base
  for step in range(1, 401):
    current = tuning.update(step * 0.05)
    assert abs(current.mass - previous.mass) <= MASS_RATE_KG_S * 0.05 + 1e-8
    assert abs(current.centerToFront - previous.centerToFront) <= cp.wheelbase * CENTER_RATIO_RATE_S * 0.05 + 1e-8
    previous = current
  assert current.mass == 2090 + STD_CARGO_KG
  assert current.centerToFront == pytest.approx(3.07 * 0.7)
  assert current.rotationalInertia == pytest.approx(scale_rot_inertia(current.mass, cp.wheelbase))
  front, rear = scale_tire_stiffness(current.mass, cp.wheelbase, current.centerToFront, 0.8)
  assert current.tireStiffnessFront == pytest.approx(front)
  assert current.tireStiffnessRear == pytest.approx(rear)
  stored.update(vehicle_spec_defaults(cp))
  for step in range(401, 801):
    current = tuning.update(step * 0.05)
  assert current == base
  assert VehicleDynamics.from_car_params(cp) == base


def test_corrupt_setting_retains_last_valid_target_and_independent_field_updates():
  cp = car_params()
  stored = {"VehicleMass": "2090", "VehicleCenterToFrontRatio": "0.70"}
  tuning = LiveVehicleDynamics(cp, NS(get=stored.get))
  tuning.update(0)
  stored.update(VehicleMass="nan", VehicleCenterToFrontRatio="0.60")
  tuning.update(1)
  assert tuning.target_mass == 2226
  assert tuning.target_front == pytest.approx(3.07 * 0.6)


def test_read_cadence_and_no_jumps_after_pause_or_backwards_clock():
  cp = car_params()
  reads = []
  tuning = LiveVehicleDynamics(cp, NS(get=lambda key: (reads.append(key), "0")[1]))
  tuning.update(0)
  tuning.update(0.49)
  assert len(reads) == 2
  tuning.update(0.5)
  assert len(reads) == 4
  tuning.params = NS(get=lambda key: "5000" if key == "VehicleMass" else "0.90")
  current = tuning.update(1000)
  assert current.mass - cp.mass == pytest.approx(10)
  assert current.centerToFront - cp.centerToFront == pytest.approx(cp.wheelbase * 0.002)
  assert tuning.update(999) == current
  assert tuning.update(float("nan")) == current


def test_not_car_never_reads_tuning():
  cp = car_params()
  cp.notCar = True
  tuning = LiveVehicleDynamics(cp, NS(get=lambda key: pytest.fail("notCar tuning read")))
  assert tuning.update(10) == tuning.base


def test_old_native_key_registry_does_not_crash_the_learner():
  class UnknownKeyName(Exception):
    pass

  def get(key):
    raise UnknownKeyName(key)

  tuning = LiveVehicleDynamics(car_params(), NS(get=get))
  assert tuning.update(0) == tuning.base
  assert tuning.update(1) == tuning.base


def test_real_capnp_message_roundtrip_and_vehicle_model_learned_values_preserved():
  cp = car_params()
  dynamics = VehicleDynamics.from_car_params(cp).scaled(2226, 3.07 * 0.7)
  sm = messages(cp, dynamics)
  with log.LiveParametersData.from_bytes(sm["liveParameters"].to_bytes()) as lp:
    sm["liveParameters"] = lp
    assert read_live_vehicle_dynamics(sm, cp) == dynamics
  vm = VehicleModel(cp)
  vm.update_params(0.85, 18.2)
  vm.update_dynamics(dynamics)
  assert vm.m == dynamics.mass
  assert vm.aF == dynamics.centerToFront
  assert vm.aR == pytest.approx(cp.wheelbase - dynamics.centerToFront)
  assert vm.cF == dynamics.tireStiffnessFront * 0.85
  assert vm.cR == dynamics.tireStiffnessRear * 0.85
  assert vm.j == dynamics.rotationalInertia
  assert vm.sR == 18.2


@pytest.mark.parametrize("field,value", [
  ("mass", float("nan")), ("rotationalInertia", 0), ("tireStiffnessRear", -1),
  ("tireStiffnessFront", float("inf")), ("centerToFront", 4), ("wheelbase", 2.5),
])
def test_invalid_live_snapshot_rejected(field, value):
  cp = car_params()
  sm = messages(cp, replace(VehicleDynamics.from_car_params(cp), **{field: value}))
  assert read_live_vehicle_dynamics(sm, cp) is None


def test_missing_stale_or_invalid_live_snapshot_leaves_consumers_unchanged():
  cp = car_params()
  sm = messages(cp)
  assert read_live_vehicle_dynamics(sm, cp) is None  # old schema/log
  sm["liveParameters"].vehicleDynamics = VehicleDynamics.from_car_params(cp).to_message()
  sm.checks = False
  assert read_live_vehicle_dynamics(sm, cp) is None
  sm.checks = True
  sm["liveParameters"].valid = False
  assert read_live_vehicle_dynamics(sm, cp) is None


def test_kalman_globals_include_all_dependent_constants():
  values = VehicleDynamics.from_car_params(car_params())
  assert values.kalman_globals() == dict(
    mass=values.mass, rotational_inertia=values.rotationalInertia,
    center_to_front=values.centerToFront, center_to_rear=values.wheelbase-values.centerToFront,
    stiffness_front=values.tireStiffnessFront, stiffness_rear=values.tireStiffnessRear,
  )
