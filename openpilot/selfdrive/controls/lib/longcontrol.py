import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      elif CP.startingState:
        long_control_state = LongCtrlState.starting
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in (LongCtrlState.starting, LongCtrlState.pid):
      if should_stop:
        long_control_state = LongCtrlState.stopping
      elif v_ego > CP.vEgoStarting:
        long_control_state = LongCtrlState.pid

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

    self.params = Params()
    self.readParamCount = 0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
    elif self.readParamCount == 10:
      kp = self.params.get_float("LongTuningKpV") * 0.01
      ki = self.params.get_float("LongTuningKiV") * 0.001
      kf = self.params.get_float("LongTuningKf") * 0.01

      self.pid._k_p = ([0.0], [kp])
      self.pid._k_i = ([0.0], [ki])
      self.pid._k_f = ([0.0], [kf])

    self.pid.set_limits(accel_limits[1], accel_limits[0])

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo, should_stop, CS.brakePressed, CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL

      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

