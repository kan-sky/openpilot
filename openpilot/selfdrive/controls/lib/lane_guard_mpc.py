import time
from typing import NamedTuple

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import (
  LANE_GUARD_CHECK_T_END,
  LANE_GUARD_CHECK_T_START,
  LANE_GUARD_CURVE_THRESHOLD,
  LANE_GUARD_INSIDE_MARGIN,
  LANE_GUARD_MAX_LANE_WIDTH,
  LANE_GUARD_MIN_LANE_PROB,
  LANE_GUARD_MIN_LANE_WIDTH,
  get_laneless_margin_geometry,
)
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N


LANE_GUARD_PATH_COST = 2.0
LANE_GUARD_HEADING_COST = 0.11
LANE_GUARD_LAT_ACCEL_COST = 0.0
LANE_GUARD_LAT_JERK_COST = 0.04
LANE_GUARD_STEERING_RATE_COST = 700.0
LANE_GUARD_MAX_PATH_SHIFT = 0.45
LANE_GUARD_MAX_CURVATURE_DELTA = 0.0015
LANE_GUARD_MAX_COST = 1e6
LANE_GUARD_MIN_SPEED = 1.0


class LaneGuardMpcResult(NamedTuple):
  curvature: float
  active: bool
  min_inside_clearance: float | None
  solution_valid: bool
  solve_time: float


class LaneGuardMpc:
  def __init__(self, CP):
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.mpc = LateralMpc()
    self.x0 = np.zeros(4)
    self.last_solution_valid = False
    self.last_solve_time = 0.0
    self.reset()

  def reset(self, measured_curvature: float = 0.0, v_ego: float = 0.0):
    self.x0 = np.zeros(4)
    self.x0[3] = float(measured_curvature) * max(float(v_ego), LANE_GUARD_MIN_SPEED)
    self.mpc.reset(x0=self.x0)
    self.last_solution_valid = False
    self.last_solve_time = 0.0

  @staticmethod
  def _fallback(desired_curvature: float, min_inside_clearance: float | None = None,
                solve_time: float = 0.0) -> LaneGuardMpcResult:
    return LaneGuardMpcResult(float(desired_curvature), False, min_inside_clearance, False, solve_time)

  @staticmethod
  def _smoothstep(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)

  def _build_guarded_path(self, path_t: np.ndarray, path_y: np.ndarray,
                          inside_boundary_y: np.ndarray, desired_curvature: float) -> tuple[np.ndarray, bool]:
    guarded_y = path_y.copy()
    active_window = ((path_t >= LANE_GUARD_CHECK_T_START) &
                     (path_t <= LANE_GUARD_CHECK_T_END))
    if np.count_nonzero(active_window) < 2:
      return guarded_y, False

    ramp_in = self._smoothstep((path_t - LANE_GUARD_CHECK_T_START) / 0.35)
    ramp_out = self._smoothstep((LANE_GUARD_CHECK_T_END - path_t) / 0.50)
    window_weight = ramp_in * ramp_out * active_window.astype(float)

    if desired_curvature > LANE_GUARD_CURVE_THRESHOLD:
      violation = np.maximum(path_y - inside_boundary_y, 0.0)
      correction = -np.minimum(violation, LANE_GUARD_MAX_PATH_SHIFT) * window_weight
    elif desired_curvature < -LANE_GUARD_CURVE_THRESHOLD:
      violation = np.maximum(inside_boundary_y - path_y, 0.0)
      correction = np.minimum(violation, LANE_GUARD_MAX_PATH_SHIFT) * window_weight
    else:
      return guarded_y, False

    guarded_y += correction
    return guarded_y, bool(np.any(np.abs(correction) > 1e-4))

  def update(self, model_v2, v_ego: float, desired_curvature: float,
             lane_change_active: bool, action_t: float) -> LaneGuardMpcResult:
    if lane_change_active or not np.isfinite(desired_curvature):
      return self._fallback(desired_curvature)

    geometry = get_laneless_margin_geometry(model_v2, desired_curvature)
    if not geometry.valid:
      return self._fallback(desired_curvature, geometry.min_inside_clearance)

    if geometry.min_inside_clearance is None or geometry.min_inside_clearance >= LANE_GUARD_INSIDE_MARGIN:
      return LaneGuardMpcResult(float(desired_curvature), False, geometry.min_inside_clearance, True, 0.0)

    path_t = np.asarray(model_v2.position.t, dtype=float)
    path_x = np.asarray(model_v2.position.x, dtype=float)
    path_y = np.asarray(model_v2.position.y, dtype=float)
    heading_pts = np.asarray(model_v2.orientation.z, dtype=float)
    yaw_rate_pts = np.asarray(model_v2.orientationRate.z, dtype=float)
    velocity_x = np.asarray(model_v2.velocity.x, dtype=float)

    expected_len = LAT_MPC_N + 1
    arrays = (path_t, path_x, path_y, heading_pts, yaw_rate_pts, velocity_x,
              geometry.inside_boundary_y)
    if any(len(a) < expected_len for a in arrays) or not all(np.all(np.isfinite(a[:expected_len])) for a in arrays):
      return self._fallback(desired_curvature, geometry.min_inside_clearance)

    path_t = path_t[:expected_len]
    path_y = path_y[:expected_len]
    heading_pts = heading_pts[:expected_len]
    yaw_rate_pts = yaw_rate_pts[:expected_len]
    velocity_x = velocity_x[:expected_len]
    inside_boundary_y = geometry.inside_boundary_y[:expected_len]

    guarded_y, guard_active = self._build_guarded_path(path_t, path_y, inside_boundary_y, desired_curvature)
    if not guard_active:
      return LaneGuardMpcResult(float(desired_curvature), False, geometry.min_inside_clearance, True, 0.0)

    v_plan = np.clip(velocity_x, LANE_GUARD_MIN_SPEED, np.inf)
    lateral_factor = np.clip(self.factor1 - self.factor2 * v_plan ** 2, 0.0, np.inf)
    params = np.column_stack([v_plan, lateral_factor])

    self.mpc.set_weights(LANE_GUARD_PATH_COST, LANE_GUARD_HEADING_COST,
                         LANE_GUARD_LAT_ACCEL_COST, LANE_GUARD_LAT_JERK_COST,
                         LANE_GUARD_STEERING_RATE_COST)

    self.x0[0] = 0.0
    self.x0[1] = 0.0
    self.x0[2] = 0.0
    self.x0[3] = float(desired_curvature) * max(float(v_ego), LANE_GUARD_MIN_SPEED)

    started = time.monotonic()
    self.mpc.run(self.x0, params, guarded_y, heading_pts, yaw_rate_pts)
    solve_time = time.monotonic() - started

    solution_valid = (self.mpc.solution_status == 0 and
                      np.all(np.isfinite(self.mpc.x_sol)) and
                      np.all(np.isfinite(self.mpc.u_sol)) and
                      np.isfinite(self.mpc.cost) and
                      self.mpc.cost < LANE_GUARD_MAX_COST)
    self.last_solution_valid = bool(solution_valid)
    self.last_solve_time = float(solve_time)

    if not solution_valid:
      self.reset(desired_curvature, v_ego)
      return self._fallback(desired_curvature, geometry.min_inside_clearance, solve_time)

    output_t = float(np.clip(action_t, path_t[0], path_t[-1]))
    desired_yaw_rate = float(np.interp(output_t, path_t, self.mpc.x_sol[:, 3]))
    guarded_curvature = desired_yaw_rate / max(float(v_ego), LANE_GUARD_MIN_SPEED)
    guarded_curvature = float(np.clip(
      guarded_curvature,
      desired_curvature - LANE_GUARD_MAX_CURVATURE_DELTA,
      desired_curvature + LANE_GUARD_MAX_CURVATURE_DELTA,
    ))
    if desired_curvature > LANE_GUARD_CURVE_THRESHOLD:
      guarded_curvature = min(guarded_curvature, float(desired_curvature))
    elif desired_curvature < -LANE_GUARD_CURVE_THRESHOLD:
      guarded_curvature = max(guarded_curvature, float(desired_curvature))

    if not np.isfinite(guarded_curvature):
      self.reset(desired_curvature, v_ego)
      return self._fallback(desired_curvature, geometry.min_inside_clearance, solve_time)

    self.x0[3] = float(np.interp(DT_MDL, path_t, self.mpc.x_sol[:, 3]))
    guard_applied = abs(guarded_curvature - desired_curvature) > 1e-7
    return LaneGuardMpcResult(guarded_curvature, guard_applied, geometry.min_inside_clearance, True, solve_time)
