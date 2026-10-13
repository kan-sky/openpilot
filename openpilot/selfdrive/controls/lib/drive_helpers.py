import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL


# Kans
from openpilot.common.params import Params
params = Params()
MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MIN_STABLE_DELAY = 0.3

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2

# Kans:
def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def should_stop(v_ego: float, a_target: float) -> bool:
  return bool(v_ego < 0.3 and a_target < 0.1)

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


# Kans: MPC가 현재 구속(binding) 중인 장애물(lead0/lead1/cruise/trafficstop -
# long_mpc.py가 argmin으로 고른 것)이 얼마나 가까워야(m) 아래 속도/가속도
# 조건에 더해 should_stop이 latch될 수 있는지. 장애물 거리는 이미 원하는
# follow gap/정지선 오프셋을 뺀 값이라, ~0이면 "이미 의도한 정지 지점"이란
# 뜻이다 - MPC 계산 노이즈는 흡수할 만큼 넉넉하면서도 5~10m 일찍 latch되는
# 건 배제할 정도로 잡았다.
REMAINING_DISTANCE_GATE = 4.0

def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.3, remaining_distance=1000.0, standstill=False):
  if len(speeds) == len(t_idxs):
    v_target_now = speeds[0]
    a_target_now = accels[0]

    if action_t < MIN_STABLE_DELAY:
      v_target = v_target_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_target_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)

    a_target = 2 * (v_target - v_target_now) / action_t - a_target_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)

    # Kans: 콤마스톡의 should_stop()(아래 참고, modeld의 e2e 경로에서만 쓰임)은
    # 실제 v_ego + a_target<0.1을 게이트로 삼는다 - 실제 상태에 반응한다.
    # 이 'acc'-모드 경로는 대신 MPC 자신의 *예측* v_target/v_target_1sec만
    # 체크하고 가속도나 거리 조건이 없었다 - 컴포트 보정이 들어간 MPC 해는
    # 차가 실제로 목표 거리에 가까워지기 한참 전에 거의 0에 가까운 속도를
    # 예측할 수 있어서, LongCtrlState.stopping(거리 추적을 포기하는 상태)에
    # 일찍 락되면서 의도한 follow/정지선 거리보다 못 미쳐 멈추게 된다.
    # 콤마스톡의 a_target<0.1에 명시적인 남은거리 게이트를 더해서
    # 앞차추종/신호정지선 접근 양쪽에서 이 간극을 좁힌다.
    #
    # Kans: 차가 이미 물리적으로 정지해 있으면(CS.standstill), remaining_distance
    # 때문에 should_stop=False가 계속 유지되게 두면 안 된다 - 정지한 lead
    # 자체의 추적 위치가 MPC가 원하는 follow gap 대비 살짝 drift될 수 있어서,
    # 차가 안 움직이고 있는데도 remaining_distance가 게이트 값 위에
    # 무한정 머물 수 있다. 이 OR가 없으면 실차에서 should_stop이 standstill이
    # 이미 True인 동안 잠깐 False로 튈 때마다 "멈춤, 살짝 전진, 다시 멈춤"
    # 하는 다단계 패턴으로 나타났고, 처음 멈춘 지점이 원하는 gap에서 멀수록
    # (더 기어나갈 거리가 남을수록) 더 심했다.
    should_stop = (v_target < vEgoStopping and v_target_1sec < vEgoStopping and a_target < 0.1
                   and (remaining_distance < REMAINING_DISTANCE_GATE or standstill))

  else:
    v_target_now = 0.0
    v_target = 0.0
    a_target = 0.0
    should_stop = False

  return a_target, should_stop, v_target_now, v_target

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
