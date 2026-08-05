import math
from collections import deque

import numpy as np

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.realtime import DT_CTRL

# Current comma lateral-acceleration controller gains.
KP = 0.8
KI = 0.15
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

# Full NNFF uses torque-domain feedback, matching the original NNFF controller.
NNFF_KP = 1.0
NNFF_KI = 0.1
NNFF_KD = 0.0

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 2

LOW_SPEED_X = [0.0, 10.0, 20.0, 30.0]
LOW_SPEED_Y = [15.0, 13.0, 10.0, 5.0]
LAT_PLAN_MIN_IDX = 5


def sign(value: float) -> float:
  return 1.0 if value > 0.0 else (-1.0 if value < 0.0 else 0.0)


def get_predicted_lateral_jerk(lat_accels, t_diffs):
  lat_accels = np.asarray(lat_accels, dtype=float)
  count = min(len(t_diffs), max(0, len(lat_accels) - 1))
  if count == 0:
    return []
  return (np.diff(lat_accels[:count + 1]) / t_diffs[:count]).tolist()


def get_lookahead_value(future_values, current_value):
  if len(future_values) == 0:
    return current_value
  same_sign_values = [value for value in future_values if sign(value) == sign(current_value)]
  if len(same_sign_values) < len(future_values):
    return 0.0
  return min(same_sign_values + [current_value], key=abs)

def get_friction_torque(lateral_accel_error: float,
                        lateral_accel_deadzone: float,
                        torque_params) -> float:
  friction_lataccel = get_friction(
    lateral_accel_error,
    lateral_accel_deadzone,
    FRICTION_THRESHOLD,
    torque_params,
  )

  lat_accel_factor = float(torque_params.latAccelFactor)
  if abs(lat_accel_factor) < 1e-6:
    return 0.0

  return friction_lataccel / lat_accel_factor


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)

    self.params = Params()
    self.frame = 0

    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.torque_from_lateral_accel_context = CI.torque_from_lateral_accel_context
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()

    # Current comma path: PID output is lateral acceleration, converted to torque at the end.
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1 / DT_CTRL)
    self.update_limits()

    # Full NNFF torque
    self.nn_pid = PIDController(NNFF_KP, NNFF_KI, k_d=NNFF_KD, k_f=1.0, rate=1 / DT_CTRL)
    self.nn_pid.set_limits(self.steer_max, -self.steer_max)

    self.lateralTorqueCustom = self.params.get_bool("LateralTorqueCustom")

    self.latAccelFactor_default = float(self.torque_params.latAccelFactor)
    self.latAccelOffset_default = float(self.torque_params.latAccelOffset)
    self.friction_default = float(self.torque_params.friction)

    # default values while not yet arrived Live_torque_values
    self.live_lat_accel_factor = self.latAccelFactor_default
    self.live_lat_accel_offset = self.latAccelOffset_default
    self.live_friction = self.friction_default

    self.nnff_kp_default = NNFF_KP
    self.nnff_ki_default = NNFF_KI
    self.nnff_kd_default = NNFF_KD
    self.nnff_kf_default = 1.0

    self.extended_nnff_available = bool(getattr(CI, "extended_nnff_available", False))
    self.extended_nnff_model = getattr(CI, "extended_torque_nn_model", None)
    self.torque_from_nn = getattr(CI, "get_extended_torque_nn_ff", None)
    self.nn_friction_override = bool(getattr(self.extended_nnff_model, "friction_override", False))

    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = max(3, int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / DT_CTRL))
    self.lat_accel_request_buffer = deque([0.0] * self.lat_accel_request_buffer_len,
                                          maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / DT_CTRL)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), DT_CTRL)

    # Original full NNFF history/future layout: 3 past and 4 future samples.
    self.t_diffs = np.diff(ModelConstants.T_IDXS)
    self.friction_look_ahead_v = [1.4, 2.0]
    self.friction_look_ahead_bp = [9.0, 30.0]
    self.lat_jerk_friction_factor = 0.4
    self.lat_accel_friction_factor = 0.7
    self.nn_time_offset = CP.steerActuatorDelay + 0.2
    self.nn_future_times = np.asarray([0.3, 0.6, 1.0, 1.5], dtype=float) + self.nn_time_offset
    self.past_times = [-0.3, -0.2, -0.1]
    history_frames = [max(1, int(round(abs(value) / DT_CTRL))) for value in self.past_times]
    self.history_frame_offsets = [history_frames[0] - value for value in history_frames]
    self.lateral_accel_desired_deque = deque([0.0] * history_frames[0], maxlen=history_frames[0])
    self.roll_deque = deque([0.0] * history_frames[0], maxlen=history_frames[0])
    self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # always restore newest Live Torque values
    self.live_lat_accel_factor = latAccelFactor
    self.live_lat_accel_offset = latAccelOffset
    self.live_friction = friction
  
    if self.lateralTorqueCustom:
      return

    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  @staticmethod
  def _model_good(model_data) -> bool:
    try:
      return model_data is not None and len(model_data.acceleration.y) >= 2 and len(model_data.orientation.x) >= 2
    except (AttributeError, TypeError):
      return False

  def _model_lookahead_jerk(self, model_data, CS, desired_lateral_accel):
    lookahead = np.interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
    upper_idx = next((index for index, value in enumerate(ModelConstants.T_IDXS) if value > lookahead), len(ModelConstants.T_IDXS) - 1)
    predicted_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
    desired_time = max(DT_CTRL, self.nn_time_offset + 0.1)
    planned_accel = np.interp(desired_time, ModelConstants.T_IDXS, model_data.acceleration.y)
    desired_jerk = (planned_accel - desired_lateral_accel) / desired_time
    return get_lookahead_value(predicted_jerk[LAT_PLAN_MIN_IDX:upper_idx], desired_jerk)

  def _full_nnff(self, CS, params, model_data, setpoint, measurement, desired_lateral_accel,
                 actual_lateral_accel, roll_compensation, lateral_accel_deadzone):
    lookahead_lateral_jerk = self._model_lookahead_jerk(model_data, CS, desired_lateral_accel)
    lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
    lateral_jerk_measurement = 0.0

    roll = float(params.roll)
    self.roll_deque.append(roll)
    self.lateral_accel_desired_deque.append(desired_lateral_accel)

    adjusted_future_times = [
      float(time_value + 0.5 * CS.aEgo * (time_value / max(CS.vEgo, 1.0)))
      for time_value in self.nn_future_times
    ]
    past_rolls = [self.roll_deque[min(len(self.roll_deque) - 1, index)] for index in self.history_frame_offsets]
    future_rolls = [float(np.interp(time_value, ModelConstants.T_IDXS, model_data.orientation.x) + roll)
                    for time_value in adjusted_future_times]
    past_desired_accels = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque) - 1, index)]
                           for index in self.history_frame_offsets]
    future_planned_accels = [float(np.interp(time_value, ModelConstants.T_IDXS, model_data.acceleration.y))
                             for time_value in adjusted_future_times]

    nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                          + past_desired_accels + future_planned_accels + past_rolls + future_rolls
    nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                             + past_desired_accels + future_planned_accels + past_rolls + future_rolls

    torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
    torque_from_measurement = self.torque_from_nn(nnff_measurement_input)
    torque_error = float(torque_from_setpoint - torque_from_measurement)

    lateral_accel_error = desired_lateral_accel - actual_lateral_accel
    error_blend_factor = float(np.interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0]))
    if error_blend_factor > 0.0:
      torque_from_error = self.torque_from_nn([CS.vEgo, setpoint - measurement,
                                               lateral_jerk_setpoint - lateral_jerk_measurement, 0.0])
      if sign(torque_error) == sign(torque_from_error) and abs(torque_error) < abs(torque_from_error):
        torque_error = float(torque_error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor)

    friction_input = self.lat_accel_friction_factor * lateral_accel_error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
    nn_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
               + past_desired_accels + future_planned_accels + past_rolls + future_rolls
    feedforward_torque = float(self.torque_from_nn(nn_input))

    if self.nn_friction_override:
      torque_error += get_friction_torque(friction_input, lateral_accel_deadzone, self.torque_params)

    return torque_error, feedforward_torque, lookahead_lateral_jerk

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, CC, curvature_limited, model_data=None, lat_delay=0.0):
    self.frame += 1
    if self.frame % 10 == 0:
      custom_enabled = self.params.get_bool("LateralTorqueCustom")
      custom_was_enabled = self.lateralTorqueCustom

      if custom_enabled:
        # Reset only when Custom Torque changes from False to True.
        if not custom_was_enabled:
          self.pid.reset()
          self.nn_pid.reset()

        lat_accel_factor = self.params.get_float("LateralTorqueAccelFactor") * 0.001
        friction = self.params.get_float("LateralTorqueFriction") * 0.001

        nnff_kp = self.params.get_float("LateralTorqueKpV") * 0.01
        nnff_ki = self.params.get_float("LateralTorqueKiV") * 0.01
        nnff_kd = self.params.get_float("LateralTorqueKd") * 0.01
        nnff_kf = self.params.get_float("LateralTorqueKf") * 0.01

        if lat_accel_factor > 1e-6:
          self.torque_params.latAccelFactor = lat_accel_factor

        self.torque_params.latAccelOffset = self.latAccelOffset_default
        self.torque_params.friction = friction

        self.nn_pid._k_p = ([0.0], [nnff_kp])
        self.nn_pid._k_i = ([0.0], [nnff_ki])
        self.nn_pid._k_d = ([0.0], [nnff_kd])
        self.nn_pid.k_f = nnff_kf

        self.update_limits()

      elif custom_was_enabled:
        # Custom True -> False: restore the latest Live Torque values immediately.
        self.torque_params.latAccelFactor = self.live_lat_accel_factor
        self.torque_params.latAccelOffset = self.live_lat_accel_offset
        self.torque_params.friction = self.live_friction

        # Restore the default Full NNFF PID gains.
        self.nn_pid._k_p = ([0.0], [self.nnff_kp_default])
        self.nn_pid._k_i = ([0.0], [self.nnff_ki_default])
        self.nn_pid._k_d = ([0.0], [self.nnff_kd_default])
        self.nn_pid.k_f = self.nnff_kf_default

        self.pid.reset()
        self.nn_pid.reset()
        self.update_limits()

      self.lateralTorqueCustom = custom_enabled

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                            CS.vEgo, params.roll)
    actual_lateral_accel = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / DT_CTRL + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    error = expected_lateral_accel - actual_lateral_accel

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames,
                                -self.lat_accel_request_buffer_len + 1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx + 1] -
                        self.lat_accel_request_buffer[lookahead_idx - 1]) / (2 * DT_CTRL)
    buffered_desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)

    full_nnff = self.params.get_bool("NNFF") and self.extended_nnff_available and self._model_good(model_data)
    nnff_lite = not full_nnff and self.params.get_bool("NNFFLite") and self._model_good(model_data)

    if full_nnff:
      low_speed_factor = np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) ** 2
      setpoint = expected_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * measured_curvature
      torque_error, feedforward_torque, desired_lateral_jerk = self._full_nnff(
        CS, params, model_data, setpoint, measurement, future_desired_lateral_accel,
        actual_lateral_accel, roll_compensation, lateral_accel_deadzone,
      )

      if not active:
        output_torque = 0.0
        self.nn_pid.reset()
        pid_log.active = False
      else:
        pid_log.error = float(torque_error)
        freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
        output_torque = self.nn_pid.update(torque_error, speed=CS.vEgo,
                                           feedforward=feedforward_torque,
                                           freeze_integrator=freeze_integrator)
        pid_log.active = True
        pid_log.p = float(self.nn_pid.p)
        pid_log.i = float(self.nn_pid.i)
        pid_log.d = float(self.nn_pid.d)
        pid_log.f = float(self.nn_pid.f)
    else:
      setpoint = expected_lateral_accel
      measurement = actual_lateral_accel
      desired_lateral_jerk = buffered_desired_lateral_jerk
      if nnff_lite:
        desired_lateral_jerk = self._model_lookahead_jerk(model_data, CS, future_desired_lateral_accel)

      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      feedforward_lataccel = gravity_adjusted_future_lateral_accel - self.torque_params.latAccelOffset
      friction_input = error + JERK_GAIN * desired_lateral_jerk
      feedforward_lataccel += get_friction(friction_input, lateral_accel_deadzone,
                                           FRICTION_THRESHOLD, self.torque_params)

      if not active:
        output_torque = 0.0
        self.pid.reset()
        pid_log.active = False
      else:
        pid_log.error = float(error)
        freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
        output_lataccel = self.pid.update(error, speed=CS.vEgo,
                                          feedforward=feedforward_lataccel,
                                          freeze_integrator=freeze_integrator)
        output_torque = self.torque_from_lateral_accel_context(
          output_lataccel, self.torque_params, roll_compensation, CS.vEgo, CS.aEgo, gravity_adjusted=True,
        )
        pid_log.active = True
        pid_log.p = float(self.pid.p)
        pid_log.i = float(self.pid.i)
        pid_log.d = float(self.pid.d)
        pid_log.f = float(self.pid.f)

    if active:
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(
        self.steer_max - abs(output_torque) < 1e-3,
        CS, steer_limited_by_safety, curvature_limited,
      ))

    return -output_torque, 0.0, pid_log
