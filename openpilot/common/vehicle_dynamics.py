"""Live vehicle-spec overrides, distributed by paramsd with its learned values."""
from dataclasses import asdict, dataclass
import math

from opendbc.car import STD_CARGO_KG


SPEC_LIMITS = {
  "VehicleMass": (500, 5000),
  "VehicleCenterToFrontRatio": (0.1, 0.9),
}
READ_INTERVAL_S = 0.5
MASS_RATE_KG_S = 100.0
CENTER_RATIO_RATE_S = 0.02


def validate_vehicle_spec(name, value):
  if name not in SPEC_LIMITS:
    return value
  low, high = SPEC_LIMITS[name]
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    number = math.nan
  integer = name == "VehicleMass"
  if isinstance(value, bool) or not math.isfinite(number) or not low <= number <= high or (integer and not number.is_integer()):
    raise ValueError(f"{name}: use {'an integer' if integer else 'a decimal'} from {low} to {high}")
  return int(number) if integer else number


def vehicle_spec_defaults(cp):
  """Return display/storage defaults from the unchanged installed CarParams."""
  if cp.notCar or not math.isfinite(cp.wheelbase) or cp.wheelbase <= 0:
    raise ValueError("vehicle specifications are unavailable")
  return {
    "VehicleMass": validate_vehicle_spec("VehicleMass", round(cp.mass - STD_CARGO_KG)),
    # CarParams uses float32; remove representation noise, not useful decimals.
    "VehicleCenterToFrontRatio": validate_vehicle_spec("VehicleCenterToFrontRatio", round(cp.centerToFront / cp.wheelbase, 6)),
  }


def resolve_stored_vehicle_spec(name, raw, defaults):
  """Migrate the old zero sentinel and integer-percent ratio on read."""
  if raw is None or raw in ("", b""):
    return defaults[name]
  number = float(raw)
  if number == 0 and not isinstance(raw, bool):
    return defaults[name]
  if name == "VehicleCenterToFrontRatio" and 10 <= number <= 90 and number.is_integer():
    return number / 100.0
  return validate_vehicle_spec(name, raw)


def initialize_vehicle_specs(params, defaults):
  """Persist actual values once, without replacing a valid custom setting."""
  values = {}
  for name in SPEC_LIMITS:
    try:
      raw = params.get(name)
      value = resolve_stored_vehicle_spec(name, raw, defaults)
      if raw is None or raw in ("", b"") or float(raw) != value:
        params.put(name, value)
      values[name] = value
    except (KeyError, TypeError, ValueError):
      # A corrupt value is not permission to change a running vehicle model.
      continue
  return values


@dataclass(frozen=True)
class VehicleDynamics:
  mass: float
  wheelbase: float
  centerToFront: float
  rotationalInertia: float
  tireStiffnessFront: float
  tireStiffnessRear: float

  @classmethod
  def from_car_params(cls, cp):
    return cls(**{key: float(getattr(cp, key)) for key in cls.__dataclass_fields__})

  def scaled(self, mass, center_to_front):
    if mass == self.mass and center_to_front == self.centerToFront:
      return self
    mass_scale = mass / self.mass
    return VehicleDynamics(
      mass, self.wheelbase, center_to_front,
      self.rotationalInertia * mass_scale,
      self.tireStiffnessFront * mass_scale * (self.wheelbase - center_to_front) / (self.wheelbase - self.centerToFront),
      self.tireStiffnessRear * mass_scale * center_to_front / self.centerToFront,
    )

  def to_message(self):
    return {"valid": True, **asdict(self)}

  def kalman_globals(self):
    return dict(mass=self.mass, rotational_inertia=self.rotationalInertia,
                center_to_front=self.centerToFront, center_to_rear=self.wheelbase - self.centerToFront,
                stiffness_front=self.tireStiffnessFront, stiffness_rear=self.tireStiffnessRear)


class LiveVehicleDynamics:
  """One publisher reads settings; consumers never independently poll them."""
  def __init__(self, cp, params):
    self.base = VehicleDynamics.from_car_params(cp)
    self.current = self.base
    self.params = params
    self.enabled = not cp.notCar
    self.target_mass = self.base.mass
    self.target_front = self.base.centerToFront
    self.last_read = None
    self.last_update = None
    self.defaults = vehicle_spec_defaults(cp) if self.enabled else {}

  def update(self, now):
    if not self.enabled or not math.isfinite(now):
      return self.current
    if self.last_read is None or now < self.last_read or now - self.last_read >= READ_INTERVAL_S:
      for name in SPEC_LIMITS:
        try:
          raw = self.params.get(name)
        except Exception:
          # A source update can precede rebuilding the native Params registry.
          # An unreadable/missing key must not stop the parameter learner.
          continue
        try:
          value = resolve_stored_vehicle_spec(name, raw, self.defaults)
        except (TypeError, ValueError):
          continue
        if name == "VehicleMass":
          self.target_mass = self.base.mass if value == self.defaults[name] else value + STD_CARGO_KG
        else:
          self.target_front = self.base.centerToFront if value == self.defaults[name] else self.base.wheelbase * value
      self.last_read = now

    # Bound both tuning steps and scheduler gaps; never replace the controller
    # or reset its PID state when a setting is edited.
    dt = 0.0 if self.last_update is None else max(0.0, min(now - self.last_update, 0.1))
    self.last_update = now
    mass = move_towards(self.current.mass, self.target_mass, MASS_RATE_KG_S * dt)
    front = move_towards(self.current.centerToFront, self.target_front, self.base.wheelbase * CENTER_RATIO_RATE_S * dt)
    self.current = self.base.scaled(mass, front)
    return self.current


def move_towards(current, target, limit):
  return current + max(-limit, min(target - current, limit))


def read_live_vehicle_dynamics(sm, cp):
  """Old recordings/mixed versions or invalid messages leave the last model intact."""
  if not sm.all_checks(["liveParameters"]):
    return None
  lp = sm["liveParameters"]
  if not lp.valid:
    return None
  msg = getattr(lp, "vehicleDynamics", None)
  if msg is None or not msg.valid:
    return None
  values = VehicleDynamics.from_car_params(msg)
  if (not all(math.isfinite(v) and v > 0 for v in asdict(values).values())
      or not math.isclose(values.wheelbase, cp.wheelbase, rel_tol=1e-6)
      or not 0 < values.centerToFront < values.wheelbase):
    return None
  return values
