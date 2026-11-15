import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, should_stop, brake_pressed, cruise_standstill):
  # Kans: starting 상태 추가 (CP.startingState인 차량만, GM 오토리쥼이 이 상태를 출발 신호로 사용)
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

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

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if should_stop:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

    # Kans: 정지 유지 감속 (StoppingAccel, 0.01 m/s^2 단위)
    self.params = Params()
    self.readParamCount = 0
    self.stopping_accel = 0.0

    # Kans: 롱컨 실시간 튜닝 (가속도 오차 PID: error = aTarget - aEgo)
    # LongTuningKpV(x0.01): 비례게인, 0이면 차량 기본값(0)
    # LongTuningKiV(x0.01): 적분게인, 0이면 차량 기본 속도별 값(CP.longitudinalTuning.kiBP/kiV)
    # LongTuningKf(x0.01): 피드포워드(aTarget) 배율, 0이면 1.0
    self.kp_default = self.pid._k_p
    self.ki_default = self.pid._k_i
    self.kf = 1.0

  def reset(self):
    self.pid.reset()

  def _update_tuning(self):
    kp = self.params.get_float("LongTuningKpV") * 0.01
    ki = self.params.get_float("LongTuningKiV") * 0.01
    kf = self.params.get_float("LongTuningKf") * 0.01
    self.pid._k_p = ([0.0], [kp]) if kp > 0.0 else self.kp_default
    self.pid._k_i = ([0.0], [ki]) if ki > 0.0 else self.ki_default
    self.kf = kf if kf > 0.0 else 1.0

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
      self.stopping_accel = self.params.get_float("StoppingAccel") * 0.01
      self._update_tuning()

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo, should_stop,
                                                       CS.brakePressed, CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      # Kans: StoppingAccel이 음수면 정지 유지 감속으로 사용 (기본 -1.0, -2.0은 정지 중 차가 흔들림)
      stopAccel = self.stopping_accel if self.stopping_accel < 0.0 else self.CP.stopAccel
      if output_accel > stopAccel:
        output_accel = min(output_accel, 0.0)
        # Kans: 차종별 stoppingDecelRate (미설정 차종은 기존 1.0)
        stopping_decel_rate = self.CP.stoppingDecelRate if self.CP.stoppingDecelRate > 0. else 1.0
        output_accel -= stopping_decel_rate * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      # Kans: 출발 가속
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target * self.kf)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
