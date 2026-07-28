import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL

from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car.volkswagen.values import VolkswagenFlags
from typing import NamedTuple

def is_volkswagen_meb(CP) -> bool:
  # VW MEB(ID.4/ID.5) 판별 단일 소스. 플랫폼 플래그 기반이라 향후 다른 VW angle 제어
  # 차종이 추가되어도 MEB 전용 경로(곡률 폐루프, 리드선택, FCW 게이트)가 오활성되지 않음.
  return CP.brand == "volkswagen" and bool(CP.flags & VolkswagenFlags.MEB)

# Kans
from openpilot.common.params import Params
params = Params()

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s
MIN_STABLE_DELAY = 0.3

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
# Kans
MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED = 4.5  # m/s^2
# 커브 안쪽 마진
LANE_GUARD_CURVE_THRESHOLD = 0.0008
LANE_GUARD_INSIDE_MARGIN = 0.5
LANE_GUARD_MIN_LANE_PROB = 0.5
LANE_GUARD_MIN_LANE_WIDTH = 2.4
LANE_GUARD_MAX_LANE_WIDTH = 5.0
LANE_GUARD_CHECK_T_START = 0.5
LANE_GUARD_CHECK_T_END = 2.0

# Kans:
def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

class LanelessMarginGeometry(NamedTuple):
  valid: bool
  min_inside_clearance: float | None
  inside_boundary_y: np.ndarray


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, steer_actuator_delay, distances):
  if len(psis) != CONTROL_N:
    psis = [0.0] * CONTROL_N
    curvatures = [0.0] * CONTROL_N
    distances = [0.0] * CONTROL_N

  v_ego = max(MIN_SPEED, v_ego)

  delay = max(0.01, steer_actuator_delay)

  current_curvature_desired = curvatures[0]
  delayed_curvature_desired = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], curvatures)
  future_curvature_desired = np.interp(1.2, ModelConstants.T_IDXS[:CONTROL_N], curvatures)

  psi = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  distance = max(np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], distances), 0.001)

  psi_damping_straight = params.get_int("PsiDampingStraight") * 0.01
  psi_damping_s_curve = params.get_int("PsiDampingSCurve") * 0.01

  # 기본값 보정
  if psi_damping_straight <= 0.0:
    psi_damping_straight = 0.7  # 곡선 탈출시 heading변화량(psi)의 30% 정도만 풀어주고 70% 유지.
  if psi_damping_s_curve <= 0.0:
    psi_damping_s_curve = 0.5  # 반대방향 곡선 전환시 heading변화량(psi)의 50%만 반영해서 좀더 빨리 풀어줌.

  # 전환구간 출렁임 방지용
  psis_damping = 1.0  # 기본은 미래 heading 변화량을 그대로 반영 
  if v_ego > 5.0 and abs(current_curvature_desired) > 0.0001:
    # 커브 -> 직선
    if abs(future_curvature_desired) < 0.0004:
      psis_damping = psi_damping_straight
    # S자 곡선
    elif np.sign(current_curvature_desired) != np.sign(future_curvature_desired):
      psis_damping = psi_damping_s_curve

  psi *= psis_damping

  average_curvature_desired = psi / distance
  desired_curvature = 2.0 * average_curvature_desired - current_curvature_desired

  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)

  safe_desired_curvature = np.clip(desired_curvature,
    current_curvature_desired - max_curvature_rate * DT_MDL,
    current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature

def get_laneless_margin_geometry(model_v2, desired_curvature: float) -> LanelessMarginGeometry:
  empty = np.empty(0, dtype=float)
  fallback = LanelessMarginGeometry(False, None, empty)

  if (len(model_v2.laneLines) < 3 or len(model_v2.laneLineProbs) < 3 or
      not np.isfinite(desired_curvature)):
    return fallback

  path_t = np.asarray(model_v2.position.t, dtype=float)
  path_x = np.asarray(model_v2.position.x, dtype=float)
  path_y = np.asarray(model_v2.position.y, dtype=float)
  left_x = np.asarray(model_v2.laneLines[1].x, dtype=float)
  left_y = np.asarray(model_v2.laneLines[1].y, dtype=float)
  right_x = np.asarray(model_v2.laneLines[2].x, dtype=float)
  right_y = np.asarray(model_v2.laneLines[2].y, dtype=float)
  left_prob = float(model_v2.laneLineProbs[1])
  right_prob = float(model_v2.laneLineProbs[2])

  arrays = (path_t, path_x, path_y, left_x, left_y, right_x, right_y)
  if (not np.isfinite(left_prob) or not np.isfinite(right_prob) or
      len(path_t) != len(path_x) or len(path_x) != len(path_y) or len(path_x) < 3 or
      len(left_x) != len(left_y) or len(right_x) != len(right_y) or
      len(left_x) < 2 or len(right_x) < 2 or
      not all(np.all(np.isfinite(a)) for a in arrays) or
      not np.all(np.diff(path_x) > 0.0) or
      not np.all(np.diff(left_x) > 0.0) or not np.all(np.diff(right_x) > 0.0)):
    return fallback

  if left_prob < LANE_GUARD_MIN_LANE_PROB or right_prob < LANE_GUARD_MIN_LANE_PROB:
    return fallback

  if path_x[0] < max(left_x[0], right_x[0]) or path_x[-1] > min(left_x[-1], right_x[-1]):
    return fallback

  left_lane_y = np.interp(path_x, left_x, left_y)
  right_lane_y = np.interp(path_x, right_x, right_y)
  lane_width = left_lane_y - right_lane_y
  if not np.all((lane_width >= LANE_GUARD_MIN_LANE_WIDTH) &
                (lane_width <= LANE_GUARD_MAX_LANE_WIDTH)):
    return fallback

  check_mask = ((path_t >= LANE_GUARD_CHECK_T_START) &
                (path_t <= LANE_GUARD_CHECK_T_END))
  if np.count_nonzero(check_mask) < 2:
    return fallback

  if desired_curvature > LANE_GUARD_CURVE_THRESHOLD:
    inside_clearance = float(np.min(left_lane_y[check_mask] - path_y[check_mask]))
    inside_boundary_y = left_lane_y - LANE_GUARD_INSIDE_MARGIN
  elif desired_curvature < -LANE_GUARD_CURVE_THRESHOLD:
    inside_clearance = float(np.min(path_y[check_mask] - right_lane_y[check_mask]))
    inside_boundary_y = right_lane_y + LANE_GUARD_INSIDE_MARGIN
  else:
    return fallback

  return LanelessMarginGeometry(True, inside_clearance, inside_boundary_y)


def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  # Kans: 저속에서는 튜닝값 기반으로 횡가속 제한 완화, 고속일수록 기본값 3.0까지 부드럽게 전환.
  # 값이 높으면 조향 권한 증가, 낮으면 조향 권한 감소.
  max_lat_accel_low_speed = params.get_int("MaxLatAccelNoRollLowSpeed") * 0.1
  if max_lat_accel_low_speed <= 0.0:
    max_lat_accel_low_speed = MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED
  max_lat_accel_low_speed = float(np.clip(max_lat_accel_low_speed, 3.5, 4.5))

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lateral_accel_no_roll = float(np.interp(v_ego,
    [80 / 3.6, 120 / 3.6],
    [max_lat_accel_low_speed, MAX_LATERAL_ACCEL_NO_ROLL]))

  max_lat_accel = max_lateral_accel_no_roll + roll_compensation
  min_lat_accel = -max_lateral_accel_no_roll + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.3):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
    v_max = np.max(speeds)
  else:
    v_now = 0.0
    a_now = 0.0
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
    v_max = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop, v_now, v_max #v_target #v_now

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
