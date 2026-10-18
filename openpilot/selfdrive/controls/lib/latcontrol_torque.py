import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    # Kans: NNFF 등 추가 입력이 필요한 차종은 CI가 컨텍스트 함수를 제공
    self.set_torque_from_lateral_accel_context = getattr(CI, "set_torque_from_lateral_accel_context", None)
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    # Kans: 토크 실시간 튜닝
    # - Kp/Ki/Kf는 항상 설정값 사용 (LateralTorqueKpV/KiV/Kf)
    # - 가속계수/마찰/오프셋: LateralTorqueCustom=1이면 모두 설정값, 0이면 모두 torqued 학습값 (학습값 없으면 차량 기본값)
    # 피드포워드 배율 kf는 공용 PIDController를 바꾸지 않고 여기서만 곱한다
    self.frame = 0
    self.params = Params()
    self.lateralTorqueCustom = self.params.get_bool("LateralTorqueCustom")
    self.live_torque_params = None  # (latAccelFactor, latAccelOffset, friction)

    self.latAccelFactor_default = self.torque_params.latAccelFactor
    self.latAccelOffset_default = self.torque_params.latAccelOffset
    self.friction_default = self.torque_params.friction
    self.kf = 1.0
    self.update_tuning()
    # Kans: D항(Kd)은 사용하지 않음 - 콤마/캐럿과 동일하게 pid.update에 error_rate를 넘기지 않아 D=0
    # (예측 성분은 피드포워드/NNFF와 저크 기반 마찰보상이 담당)

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    # Kans: torqued 학습값 보관, 커스텀이 아니면 바로 반영
    self.live_torque_params = (latAccelFactor, latAccelOffset, friction)
    if not self.lateralTorqueCustom:
      self.apply_torque_params()

  def apply_torque_params(self):
    if self.lateralTorqueCustom:
      self.torque_params.latAccelFactor = self.params.get_float("LateralTorqueAccelFactor") * 0.001
      self.torque_params.latAccelOffset = self.params.get_float("LateralTorqueOffset") * 0.001
      self.torque_params.friction = self.params.get_float("LateralTorqueFriction") * 0.001
    elif self.live_torque_params is not None:
      self.torque_params.latAccelFactor, self.torque_params.latAccelOffset, self.torque_params.friction = self.live_torque_params
    else:
      self.torque_params.latAccelFactor = self.latAccelFactor_default
      self.torque_params.latAccelOffset = self.latAccelOffset_default
      self.torque_params.friction = self.friction_default
    self.update_limits()

  def update_tuning(self):
    self.lateralTorqueCustom = self.params.get_bool("LateralTorqueCustom")
    self.pid._k_p = ([0.0], [self.params.get_float("LateralTorqueKpV") * 0.01])
    self.pid._k_i = ([0.0], [self.params.get_float("LateralTorqueKiV") * 0.01])
    self.kf = self.params.get_float("LateralTorqueKf") * 0.01
    self.apply_torque_params()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    self.frame += 1
    if self.frame % 10 == 0:
      self.update_tuning()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      if self.set_torque_from_lateral_accel_context is not None:
        self.set_torque_from_lateral_accel_context(roll_compensation, CS.vEgo, CS.aEgo)
        # Kans: NNFF 차종은 속도/롤에 따라 토크↔횡가속 관계가 바뀌므로 PID 상한을 20Hz로 갱신 (토크 ±steer_max까지 사용)
        if self.frame % 5 == 0:
          self.update_limits()
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff * self.kf, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
