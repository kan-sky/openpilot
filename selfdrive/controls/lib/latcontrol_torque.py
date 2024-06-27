from collections import deque
import math
import numpy as np

from cereal import log
from common.numpy_fast import interp
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS
from common.params import Params

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NN = [8, 6, 1, 0]

# Takes past errors (v) and associated relative times (t) and returns a function
# that can be used to predict future errors. The function takes a time (t) and
# returns the predicted error at that time, assuming the error will converge to 0.
def get_predict_error_func(v, t, a=1.5):
  A = np.vstack([t, np.ones(len(t))]).T
  m, c = np.linalg.lstsq(A, v, rcond=1e-10)[0]

  def error(t):
    return np.exp(-a * t) * (m * t + c)

  return error

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki, k_d=self.torque_params.kd,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.paramsCount = 0

    self.lateralTorqueCustom = int(Params().get("LateralTorqueCustom", encoding="utf8"))
    self.lateralTorqueAccelFactor = float(int(Params().get("LateralTorqueAccelFactor", encoding="utf8")))*0.001
    self.lateralTorqueFriction = float(int(Params().get("LateralTorqueFriction", encoding="utf8")))*0.001
    
    # neural network feedforward
    self.use_nnff = CI.has_lateral_torque_nnff
    if self.use_nnff:
      # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      
      # setup future time offsets
      self.nnff_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nnff_future_times = [i + self.nnff_time_offset for i in future_times]
      self.nnff_future_times_np = np.array(self.nnff_future_times)
      
      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.error_deque = deque(maxlen=history_check_frames[0])

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    if self.lateralTorqueCustom > 0: 
      return
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update_params(self):
    self.paramsCount += 1
    if self.paramsCount > 30:
      self.paramsCount = 0
      lateralTorqueKp = float(int(Params().get("LateralTorqueKp", encoding="utf8")))*0.01
      lateralTorqueKi = float(int(Params().get("LateralTorqueKi", encoding="utf8")))*0.01
      lateralTorqueKd = float(int(Params().get("LateralTorqueKd", encoding="utf8")))*0.01
      lateralTorqueKf = float(int(Params().get("LateralTorqueKf", encoding="utf8")))*0.01
      self.pid._k_p = [[0], [lateralTorqueKp]]
      self.pid._k_i = [[0], [lateralTorqueKi]]
      self.pid._k_d = [[0], [lateralTorqueKd]]
      self.pid.k_f = lateralTorqueKf
    elif self.paramsCount == 10:
      self.lateralTorqueCustom = int(Params().get("LateralTorqueCustom", encoding="utf8"))
      self.lateralTorqueAccelFactor = float(int(Params().get("LateralTorqueAccelFactor", encoding="utf8")))*0.001
      self.lateralTorqueFriction = float(int(Params().get("LateralTorqueFriction", encoding="utf8")))*0.001
      if self.lateralTorqueCustom > 0:
        self.torque_params.latAccelFactor = self.lateralTorqueAccelFactor
        self.torque_params.friction = self.lateralTorqueFriction

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, lat_plan=None, model_data=None):
    self.update_params()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
      angle_steers_des = 0.0

    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        if self.use_nnff:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
          desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
        actual_lateral_jerk = 0.0
        desired_lateral_jerk = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2
      
      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y if not self.use_nnff else LOW_SPEED_Y_NN)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      
      model_planner_good = None not in [lat_plan, model_data] and all([len(i) >= CONTROL_N for i in [model_data.orientation.x, lat_plan.curvatures]])
      if self.use_nnff and model_planner_good:
        # update past data
        error = setpoint - measurement
        roll = params.roll
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(desired_lateral_accel)
        self.error_deque.append(error)
        
        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nnff_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [interp(t, T_IDXS, model_data.orientation.x) + roll for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [interp(t, T_IDXS[:CONTROL_N], lat_plan.curvatures) * CS.vEgo ** 2 for t in adjusted_future_times]
        past_errors = [self.error_deque[min(len(self.error_deque)-1, i)] for i in self.history_frame_offsets]
        future_error_func = get_predict_error_func(past_errors + [error], self.past_times + [0.0])
        future_errors = future_error_func(self.nnff_future_times_np).tolist()
        
        # compute NN error response
        lateral_jerk_error = 0.1 * (desired_lateral_jerk - actual_lateral_jerk)
        nnff_error_input = [CS.vEgo, error, lateral_jerk_error, 0.0] \
                              + past_errors + future_errors
        pid_log.error = self.torque_from_nn(nnff_error_input)
        
        # compute feedforward (same as nn setpoint output)
        nnff_input = [CS.vEgo, desired_lateral_accel, error, roll] \
                              + past_lateral_accels_desired + future_planned_lateral_accels \
                              + past_rolls + future_rolls
        ff = self.torque_from_nn(nnff_input)
        nnff_log = nnff_input + nnff_error_input
      else:
        gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
        torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                      lateral_accel_deadzone, friction_compensation=False)
        torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                      lateral_accel_deadzone, friction_compensation=False)
        pid_log.error = torque_from_setpoint - torque_from_measurement
        ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                            desired_lateral_accel - actual_lateral_accel,
                                            lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      #C2#pid_log.active = True
      #C2#pid_log.p = self.pid.p
      #C2#pid_log.i = self.pid.i
      #C2#pid_log.d = self.pid.d
      #C2#pid_log.f = self.pid.f
      #C2#pid_log.output = -output_torque
      #C2#pid_log.actualLateralAccel = actual_lateral_accel
      #C2#pid_log.desiredLateralAccel = desired_lateral_accel
      #C2#pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)
      if self.use_nnff:
        pid_log.nnffLog = nnff_log
      self.latDebugText = 'latAccel={:1.3f},Friction={:1.3f}'.format(self.torque_params.latAccelFactor, self.torque_params.friction)
      self.torqDebugText = 'Kp={:1.2f},Ki={:1.2f},Kd={:1.2f},Kf={:1.2f}'.format(self.torque_params.kp, self.torque_params.ki, self.torque_params.kd, self.torque_params.kf)
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll)) + params.angleOffsetDeg

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
