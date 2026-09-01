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
      if starting_condition:
        if CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if should_stop:
        long_control_state = LongCtrlState.stopping
      elif v_ego > CP.vEgoStarting:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    # Kans: debug
    self.debug_stop = False

    self.params = Params()
    self.readParamCount = 0

    # Kans: should_stop() is v_ego<0.3 and a_target<0.1 - right after
    # leaving stopping, v_ego is still under that for a moment and a_target
    # can briefly dip under 0.1 before it settles, which was enough to snap
    # straight back into stopping (car resumes then immediately re-stops).
    # Ignore should_stop for a brief window right after leaving stopping.
    self._leave_stopping_grace_frames = 0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
    elif self.readParamCount == 10:
      longitudinalTuningKpV = self.params.get_float("LongTuningKpV") * 0.01
      longitudinalTuningKiV = self.params.get_float("LongTuningKiV") * 0.001
      self.pid._k_p = (self.CP.longitudinalTuning.kpBP, [longitudinalTuningKpV])
      self.pid._k_i = (self.CP.longitudinalTuning.kiBP, [longitudinalTuningKiV])
      self.pid._k_f = ([0], [self.params.get_float("LongTuningKf") * 0.01])

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    # Kans: debug
    if not self.debug_stop and CS.vEgo < 5.0 and a_target < -0.1:
      self.debug_stop = True
      print(f"\n========== LONG STOP DEBUG START ==========", flush=True)

    prev_long_control_state = self.long_control_state
    effective_should_stop = should_stop and self._leave_stopping_grace_frames == 0
    if self._leave_stopping_grace_frames > 0:
      self._leave_stopping_grace_frames -= 1

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo, effective_should_stop,
                                                       CS.brakePressed, CS.cruiseState.standstill)

    if prev_long_control_state == LongCtrlState.stopping and self.long_control_state != LongCtrlState.stopping:
      self._leave_stopping_grace_frames = int(0.5 / DT_CTRL)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel

      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    if self.debug_stop:
      print(f"vEgo={CS.vEgo:.3f} aTarget={a_target:.3f} shouldStop={should_stop} longCtrlState={self.long_control_state} outputAccel={self.last_output_accel:.3f}", flush=True)
      if CS.vEgo < 0.05:
        print(f"========== LONG STOP DEBUG END ==========\n", flush=True)
        self.debug_stop = False
 
    return self.last_output_accel



