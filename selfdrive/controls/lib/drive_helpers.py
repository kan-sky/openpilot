import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL
# carrot
from cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants
import numpy as np

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
# Carrot
MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED = 4.5  # m/s^2

# Kans:
def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

# Carrot
def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, steer_actuator_delay, distances):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    distances = [0.0] * CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = max(0.01, steer_actuator_delay)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  delayed_curvature_desired = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], curvatures)
  future_curvature_desired = np.interp(1.2, ModelConstants.T_IDXS[:CONTROL_N], curvatures)

  psi = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)

  distance = max(np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], distances), 0.001)
  #average_curvature_desired = psi / (v_ego * delay)

  # curve -> straight or reverse curve
  if v_ego > 5 and abs(current_curvature_desired) > 0.002 and \
     (abs(future_curvature_desired) < 0.001 or np.sign(current_curvature_desired) != np.sign(future_curvature_desired)):
    psis_damping = 0.2
  else:
    psis_damping = 1.0
  #psi *= psis_damping


  average_curvature_desired = psi / distance
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  ### Kans: 커브가 길어질수록 조향 유지력(stability) 복원, 즉 안쪽 쏠림방지용 ###
  curve_delta = abs(desired_curvature - current_curvature_desired)
  # curve_delta: 목표 곡률(desired)과 현재 곡률(이전 스텝 setpoint)의 차이(절대값)
  # 값이 작다 = 곡률변화가 거의 없음(커브 중·후반부)
  # 값이 크다 = 커브 진입/탈출/급변 구간(주로 커브초반, 탈출)
  stability_factor = np.interp(curve_delta, [0.0, 0.0005, 0.0020, 0.0030], [0.96, 0.97, 0.99, 1.12],)
  # curve_delta가 0에 가까우면 0.96~0.94 수준으로 살짝 줄여서 안쪽 쏠림을 완화.
  # curve_delta가 커질수록  0.9 근처까지 줄여서 핸들이 확 꺾이지 않도록 완만하게 진입.
  # 숫자의미: 0.0005~0.002는 ‘완만↔급’ 변화 구간 정도. 차량/타이어에 따라 미세조정 여지 있음.
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  # curve_scale=전체 곡률을 약간만 줄여주는 기본 스케일.
  # 저속에서는 0.9 부근으로 더 완만하게, 고속에서도 1.0까지는 올리지 않고 0.97 정도로 살짝만 줄여 안쪽 차선으로 과하게 말리지 않도록 한다.
  curve_scale = np.interp(v_ego, [0.0, 15.0, 25.0], [0.95, 0.97, 0.99])
  # 최종안전곡률: 모델곡률(desired_curvature)에 curve_scale * stability_factor를 곱해
  # 항상 원래보다 더 완만하거나 같은 곡률을 사용하게 한다.
  safe_desired_curvature = np.clip(desired_curvature * curve_scale * stability_factor,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll):
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lateral_accel_no_roll = MAX_LATERAL_ACCEL_NO_ROLL
  if v_ego < 100 / 3.6:  # 100 km/h
    max_lateral_accel_no_roll = MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED
  max_lat_accel = max_lateral_accel_no_roll + roll_compensation
  min_lat_accel = -max_lateral_accel_no_roll + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv

# Carrot: model에 v_now, v_max 추가
def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
    v_max = np.max(speeds) # carrot
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
    v_now = 0.0 # carrot
    a_now = 0.0 # carrot
    v_max = 0.0 # carrot
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop, v_now, v_max # carrot
def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan_org(yaws, yaw_rates, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)

def curv_from_psis_dist(psi_target, psi_rate, distance, vego):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  distance = np.clip(distance, 1e-3, np.inf)   
  avg_curv = psi_target / distance             
  return 2.0 * avg_curv - (psi_rate / vego)    

def get_curvature_from_plan(yaws, yaw_rates, distances, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  dist = max(MIN_SPEED * action_t, np.interp(action_t, t_idxs, distances))
  return curv_from_psis_dist(psi_target, psi_rate, dist, vego)
