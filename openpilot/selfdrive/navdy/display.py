"""Pure Carrot -> navdy.openpilot.v1 display mapping.

Projection and display conventions adapted from Wayon navdy_op_bridge.py at
b7660d1d403938c464d4f14f842449e3f0e64ebd (helpers, geometry and payload).
No transport, Params, clock, SP services, raw CAN or control writes. The caller
passes None for stale/invalid services and adds any transport timestamp.
All speed fields ending in Kph are km/h regardless of the vehicle's UI units.
"""
from __future__ import annotations

from itertools import islice
import math
from typing import Any

from openpilot.selfdrive.car.outside_air_temp import outside_air_temp_c
from openpilot.selfdrive.car.stock_lane_markings import stock_lane_codes
from openpilot.selfdrive.navdy.deceleration import badge_code

KPH_PER_MS = 3.6
MAX_MODEL_SAMPLES = 64
MAX_PROJECTED_POINTS = 10
MAX_DISTANCE_M = 80.0


def finite_float(value: Any, default: float = 0.0) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return number if math.isfinite(number) else default


def rounded(value: Any, digits: int = 1) -> float:
  return round(finite_float(value), digits)


def enum_text(value: Any) -> str:
  return str(value).split(".")[-1]


def _items(value: Any, count: int) -> list:
  return list(islice(value, count)) if value is not None else []


def _pair_text(left: bool, right: bool, both: str) -> str:
  return both if left and right else "left" if left else "right" if right else "off"


def resolve_operating_state(*, connected: bool, onroad: bool, door_open: bool = False,
                            gear: str = "unknown") -> str:
  if not connected:
    return "disconnected"
  if gear == "reverse":
    return "reverse"
  if onroad:
    return "onroad_door" if door_open else "onroad"
  return "offroad_door" if door_open else "offroad"


def _speed_kph(holder: Any, names: tuple[str, ...], scale: float = 1.0) -> float:
  for name in names:
    speed = finite_float(getattr(holder, name, 0.0)) * scale
    if 0.0 < speed < 255.0:
      return speed
  return 0.0


def set_speed_kph(car_state: Any, car_control: Any = None) -> float:
  # card.py publishes vCruise[Cluster] in km/h; CruiseState/HUDControl use m/s.
  return (_speed_kph(car_state, ("vCruiseCluster", "vCruise"))
          or _speed_kph(getattr(car_state, "cruiseState", None), ("speedCluster", "speed"), KPH_PER_MS)
          or _speed_kph(getattr(car_control, "hudControl", None), ("setSpeed",), KPH_PER_MS))


def _model_points(x_values: Any, y_values: Any) -> list[tuple[float, float]]:
  points = []
  for raw_x, raw_y in zip(_items(x_values, MAX_MODEL_SAMPLES), _items(y_values, MAX_MODEL_SAMPLES), strict=False):
    x, y = finite_float(raw_x, math.nan), finite_float(raw_y, math.nan)
    # Preserve sub-millimeter negative origin jitter at standstill, not gaps
    # in an established forward trajectory or invalid/nonfinite samples.
    if points and -0.001 <= x < 0.0 and points[-1][0] <= 0.001 and abs(y - points[-1][1]) <= 0.001:
      continue
    if not (math.isfinite(x) and math.isfinite(y) and 0.0 <= x <= 10000.0 and abs(y) <= 100.0):
      break
    if points and x <= points[-1][0]:
      # A stopping/turning trajectory can fold back at its far end. Keep
      # only this frame's unambiguous forward prefix, never sort the tail.
      break
    points.append((x, y))
  return points if len(points) >= 2 else []


def model_line_y_at_values(x_values: Any, y_values: Any, distance_m: float) -> float | None:
  points = _model_points(x_values, y_values)
  if len(points) < 2 or not points[0][0] <= distance_m <= points[-1][0]:
    return None
  for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False):
    if x0 <= distance_m <= x1:
      return y0 + (y1 - y0) * (distance_m - x0) / (x1 - x0)
  return None


def model_line_y_at(line: Any, distance_m: float) -> float | None:
  return model_line_y_at_values(getattr(line, "x", None), getattr(line, "y", None), distance_m)


def navdy_project_point(x_value: Any, y_value: Any, path_y_value: Any = None) -> list[float]:
  x, y = finite_float(x_value, math.nan), finite_float(y_value, math.nan)
  if not 0.0 <= x <= MAX_DISTANCE_M or not math.isfinite(y) or abs(y) > 100.0:
    return []
  distance = (x / MAX_DISTANCE_M) ** 0.65
  lateral_scale = 44.0 * (1.0 - distance) + 8.0 * distance
  if path_y_value is None:
    screen_x = 160.0 + y * lateral_scale
  else:
    path_y = finite_float(path_y_value, math.nan)
    if not math.isfinite(path_y) or abs(path_y) > 100.0:
      return []
    screen_x = 160.0 + path_y * max(lateral_scale, 17.0) + (y - path_y) * lateral_scale
  return [rounded(screen_x), rounded(96.0 - 88.0 * distance)]


def navdy_model_line(x_values: Any, y_values: Any, lateral_offset: float = 0.0,
                     path_x_values: Any = None, path_y_values: Any = None) -> list[float]:
  points = [(x, y) for x, y in _model_points(x_values, y_values) if x <= MAX_DISTANCE_M]
  if len(points) > MAX_PROJECTED_POINTS:
    last = len(points) - 1
    points = [points[round(i * last / (MAX_PROJECTED_POINTS - 1))] for i in range(MAX_PROJECTED_POINTS)]
  projected = []
  for x, y in points:
    path_y = model_line_y_at_values(path_x_values, path_y_values, x)
    projected.extend(navdy_project_point(x, y + lateral_offset, path_y))
  return projected


def _probability(values: list, index: int) -> float:
  return max(0.0, min(1.0, finite_float(values[index]))) if index < len(values) else 0.0


def _lane_type(code: Any, probability: Any) -> str:
  # Hyundai publishes color (0..3) * 10 + type (0..8); Xiaoge replaces
  # only the type with 0/1. Reject reserved codes instead of coercing them.
  if isinstance(code, bool) or not isinstance(code, int) or not 0 <= code <= 38 or code % 10 > 8:
    return "unknown"
  if isinstance(probability, bool) or not isinstance(probability, (int, float)) or not 0.3 < probability <= 1.0:
    return "unknown"
  # Match C4, including double/inner markings. The existing Navdy renderer
  # treats unknown as dashed; do not infer a style from geometry alone.
  return "dashed" if code % 10 == 0 else "solid"


def navdy_model_geometry(model: Any) -> dict[str, Any]:
  position = getattr(model, "position", None)
  lanes = _items(getattr(model, "laneLines", None), 4)
  if position is None or len(lanes) < 3:
    return {}
  path_x, path_y = getattr(position, "x", None), getattr(position, "y", None)

  def line_points(line: Any) -> list[float]:
    return navdy_model_line(getattr(line, "x", None), getattr(line, "y", None), 0.0, path_x, path_y)

  geometry = {
    "navPathLeft": navdy_model_line(path_x, path_y, -1.0, path_x, path_y),
    "navPathRight": navdy_model_line(path_x, path_y, 1.0, path_x, path_y),
    "navLaneLeft": line_points(lanes[1]),
    "navLaneRight": line_points(lanes[2]),
  }
  if not all(len(points) >= 4 for points in geometry.values()):
    return {}
  probs = _items(getattr(model, "laneLineProbs", None), 4)
  for index, side in enumerate(("FarLeft", "Left", "Right", "FarRight")):
    geometry[f"navLane{side}Prob"] = rounded(_probability(probs, index), 2)
  if len(lanes) >= 4:
    geometry["navLaneFarLeft"] = line_points(lanes[0])
    geometry["navLaneFarRight"] = line_points(lanes[3])
  edges = _items(getattr(model, "roadEdges", None), 2)
  stds = _items(getattr(model, "roadEdgeStds", None), 2)
  for index, side in enumerate(("Left", "Right")):
    if index >= len(edges) or index >= len(stds):
      continue
    std = finite_float(stds[index], math.inf)
    confidence = max(0.0, min(1.0, 1.0 - std)) if std >= 0.0 else 0.0
    points = line_points(edges[index])
    if confidence >= 0.5 and len(points) >= 4:
      geometry[f"navRoadEdge{side}"] = points
      geometry[f"navRoadEdge{side}Prob"] = rounded(confidence, 2)
  return geometry


def _lead_lane(model: Any, distance: float, y: float) -> str:
  lanes = _items(getattr(model, "laneLines", None), 4)
  probs = _items(getattr(model, "laneLineProbs", None), 4)
  boundaries = [model_line_y_at(line, distance) for line in lanes]
  for left, right, lane, threshold in ((1, 2, "center", 0.3), (0, 1, "left", 0.2), (2, 3, "right", 0.2)):
    if right >= len(boundaries) or min(_probability(probs, left), _probability(probs, right)) < threshold:
      continue
    low, high = boundaries[left], boundaries[right]
    if low is not None and high is not None and low < high and low <= y <= high:
      return lane
  return ""


def navdy_lead_geometry(model: Any, radar_state: Any, longitudinal_plan: Any = None) -> dict[str, Any]:
  """Only the two published, visible leads, never inferred raw radar tracks."""
  vehicles = []
  source = enum_text(getattr(longitudinal_plan, "longitudinalPlanSource", ""))
  for index, name in enumerate(("leadOne", "leadTwo")):
    lead = getattr(radar_state, name, None)
    if not bool(getattr(lead, "status", False)):
      continue
    distance = finite_float(getattr(lead, "dRel", None), math.nan)
    lateral = -finite_float(getattr(lead, "yRel", None), math.nan)
    relative_speed = finite_float(getattr(lead, "vRel", None), math.nan)
    if not 2.0 <= distance <= MAX_DISTANCE_M or not math.isfinite(relative_speed):
      continue
    path_y = model_line_y_at(getattr(model, "position", None), distance)
    lane = _lead_lane(model, distance, lateral)
    if not lane:
      continue
    # Lanes can extend beyond the time-based stopping trajectory. Match their
    # existing projection there, using the measured lateral coordinate directly.
    screen = navdy_project_point(distance, lateral, path_y)
    if not screen:
      continue
    track_id = int(finite_float(getattr(lead, "radarTrackId", -1), -1))
    if track_id >= 0 and any(vehicle["trackId"] == track_id for vehicle in vehicles):
      continue
    vehicle = {
      "trackId": track_id,
      "screenX": rounded(max(4.0, min(316.0, screen[0]))),
      "screenY": screen[1],
      "distanceM": rounded(distance),
      "relativeSpeedMps": rounded(relative_speed),
      "lateralM": rounded(lateral, 2),
      "lane": lane,
      "source": "radar" if bool(getattr(lead, "radar", False)) else "vision",
      "longitudinalLead": source == f"lead{index}",
      "confidence": rounded(max(0.0, min(1.0, finite_float(getattr(lead, "modelProb", 0.0)))), 2),
    }
    vehicles.append(vehicle)
  return {"navVehicles": sorted(vehicles, key=lambda vehicle: vehicle["distanceM"], reverse=True)}


def _longitudinal_actuator(active: bool, car_control: Any, car_output: Any) -> tuple[str, float]:
  # The GM-specific raw-brake/350 normalization is not portable across Carrot
  # ports. Use the returned SI acceleration, never the requested carControl one.
  if not active or not bool(getattr(car_control, "longActive", False)):
    return "none", 0.0
  accel = finite_float(getattr(getattr(car_output, "actuatorsOutput", None), "accel", None), math.nan)
  if not math.isfinite(accel):
    return "none", 0.0
  if accel > 0.05:
    return "accelerator", min(1.0, accel)
  if accel < -0.05:
    return "brake", min(1.0, -accel)
  return "coast", 0.0


def payload_from_messages(*, car_state: Any = None, selfdrive_state: Any = None,
                          controls_state: Any = None, car_control: Any = None,
                          car_output: Any = None, model: Any = None,
                          radar_state: Any = None, longitudinal_plan: Any = None,
                          carrot_man: Any = None, seq: int = 0, onroad: bool = False,
                          door_open: bool | None = None) -> dict[str, Any]:
  """Map fresh service structs without retaining state or synthesizing alerts.

  Only the display-only deceleration verdict uses carrot_man here; navigation
  feedback is still owned by the bridge separately.
  Alert strings are preserved verbatim, including hold/resume/critical alerts.
  """
  car_valid = (car_state is not None and bool(getattr(car_state, "canValid", False))
               and not bool(getattr(car_state, "canTimeout", False)))
  if onroad:
    # GM speed is signed and its filter can cross zero at rest. Neither is a
    # lost CAN connection; reverse display suppression is handled by gear below.
    car_valid = car_valid and abs(finite_float(getattr(car_state, "vEgo", None), math.inf)) < 100.0
  connected = car_valid and (not onroad or selfdrive_state is not None)
  gear = enum_text(getattr(car_state, "gearShifter", "unknown")).lower() if car_valid else "unknown"
  door = bool(getattr(car_state, "doorOpen", False) if door_open is None else door_open)
  driving = connected and bool(onroad) and gear != "reverse"
  enabled = driving and bool(getattr(selfdrive_state, "enabled", False))
  active = enabled and bool(getattr(selfdrive_state, "active", False))
  engageable = driving and bool(getattr(selfdrive_state, "engageable", False))
  state = enum_text(getattr(selfdrive_state, "state", "unknown")) if driving else "disabled"
  left = driving and bool(getattr(car_state, "leftBlinker", False))
  right = driving and bool(getattr(car_state, "rightBlinker", False))
  left_bsm = driving and bool(getattr(car_state, "leftBlindspot", False))
  right_bsm = driving and bool(getattr(car_state, "rightBlindspot", False))
  soft_hold = int(finite_float(getattr(car_state, "softHoldActive", 0))) if driving else 0
  hold = driving and (bool(getattr(car_state, "brakeHoldActive", False)) or soft_hold == 2)
  standstill = driving and bool(getattr(car_state, "standstill", False))
  stop_icon = driving and (state == "preEnabled" or (standstill and (enabled or hold)))
  speed = _speed_kph(car_state, ("vEgoCluster", "vEgo"), KPH_PER_MS) if driving else 0.0
  physical_acc = _speed_kph(getattr(car_state, "cruiseState", None), ("speedCluster", "speed"), KPH_PER_MS)
  actuator, actuator_level = _longitudinal_actuator(active, car_control, car_output)
  payload = {
    "schema": "navdy.openpilot.v1",
    "seq": max(0, min(2**53 - 1, int(finite_float(seq)))),
    "state": state,
    "enabled": enabled,
    "active": active,
    "engaged": active,
    "disengaged": not enabled,
    "engageable": engageable,
    "opAvailable": engageable,
    "onroad": bool(onroad),
    "doorOpen": door,
    "ambientState": resolve_operating_state(connected=connected, onroad=bool(onroad), door_open=door, gear=gear),
    "gear": gear,
    "standstill": stop_icon,
    "cruiseStandstill": stop_icon,
    "autoHoldActive": hold,
    "softHoldActive": soft_hold,
    "setSpeedKph": rounded(set_speed_kph(car_state, car_control)) if driving else 0.0,
    "actualAccSetKph": rounded(physical_acc) if driving else 0.0,
    "vEgoKph": rounded(speed),
    "speedUnit": "kph",
    "leftBlinker": left,
    "rightBlinker": right,
    "blinkers": _pair_text(left, right, "hazard"),
    "leftBlindspot": left_bsm,
    "rightBlindspot": right_bsm,
    "blindspot": _pair_text(left_bsm, right_bsm, "both"),
    "alertText1": str(getattr(selfdrive_state, "alertText1", "")),
    "alertText2": str(getattr(selfdrive_state, "alertText2", "")),
    "alertType": str(getattr(selfdrive_state, "alertType", "")),
    "alertStatus": enum_text(getattr(selfdrive_state, "alertStatus", "normal")),
    "alertSize": enum_text(getattr(selfdrive_state, "alertSize", "none")),
    "greenLightAlert": False,
    "leadDepartAlert": False,
    "longitudinalActuator": actuator,
    "longitudinalActuatorLevel": rounded(actuator_level, 2),
    "latActive": active and bool(getattr(car_control, "latActive", False)),
    "longActive": active and bool(getattr(car_control, "longActive", False)),
    "activeLaneLine": active and bool(getattr(controls_state, "activeLaneLine", False)),
    "navVehicles": [],
    "decel": 0,
  }
  outside_c = outside_air_temp_c(car_state) if car_valid else None
  if outside_c is not None:
    # Absent key keeps the HUD on its own OBD 0x46 reading.
    payload['outsideTempC'] = rounded(outside_c, 1)
  if payload['longActive'] and payload['alertSize'] == 'none' and payload['alertStatus'] == 'normal':
    payload['decel'] = badge_code(car_state, selfdrive_state, car_control, longitudinal_plan, carrot_man)
  if active:
    geometry = navdy_model_geometry(model)
    payload.update(geometry)
    if geometry:
      payload.update(navdy_lead_geometry(model, radar_state, longitudinal_plan))
      probs = _items(getattr(model, "laneLineProbs", None), 3)
      left_code, right_code = stock_lane_codes(car_state) or (
        getattr(car_state, "leftLaneLine", None), getattr(car_state, "rightLaneLine", None),
      )
      for index, side, code in ((1, "Left", left_code), (2, "Right", right_code)):
        payload[f"navLane{side}Type"] = _lane_type(code, probs[index] if index < len(probs) else None)
  return payload
