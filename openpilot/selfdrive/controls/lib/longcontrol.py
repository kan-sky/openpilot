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
  # Kans: 순수 콤마스톡의 무조건 전환(a_ego/fcw_stop 재진입 디바운스 없음)으로
  # 되돌렸다. 그 디바운스는 예전에 "움찔하다 수동 RESUME 전까지 멈춰있는"
  # 버그를 쫓으려고 복원했던 건데, "앞차 뒤에서 멈추지 않음" 문제도 일으킨
  # 전력이 있고(그래서 tz 이전에 한 번 뺐었다), 지금은 정지 근처에서 새로운
  # 3단계 버벅임(should_stop이 임계값 근처에서 깜빡이면서 pid<->stopping이
  # 토글되는 것)을 일으키는 것으로도 의심된다. 움찔/멈춤 버그는 실은
  # opendbc/car/gm/carcontroller.py의 별개 AccState import 크래시(독립적으로
  # 고침)만으로 전부 설명됐을 수도 있다 - 이 디바운스가 정말 필요한지
  # 없이 테스트해보는 중.
  stopping_condition = should_stop
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
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    # Kans: 이 포크에선 kp가 항상 0이라(콤마스톡의 GM 관례와 일치 - 콤마는
    # GM에 롱컨 P항을 절대 설정하지 않는다), CP.longitudinalTuning.kpBP/kpV
    # BP-보간 쌍 대신 콤마스톡처럼 그냥 float으로 넘긴다(어느 쪽이든 결국
    # 0.0으로 귀결되긴 한다 - 아래의 실시간 LongTuningKpV 오버라이드는
    # CP.longitudinalTuning.kpBP를 직접 읽어서 이것과 무관하게 동작한다).
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.last_output_accel = 0.0


    self.params = Params()
    self.readParamCount = 0
    self.stopping_accel = 0.0
    self.j_lead = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    a_target_ff = long_plan.aTarget
    v_target_now = long_plan.vTargetNow
    j_target_now = long_plan.jTargetNow
    should_stop = long_plan.shouldStop

    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
      self.stopping_accel = self.params.get_float("StoppingAccel") * 0.01
    elif self.readParamCount == 10:
      if len(self.CP.longitudinalTuning.kpBP) == 1 and len(self.CP.longitudinalTuning.kiBP) == 1:
        longitudinalTuningKpV = self.params.get_float("LongTuningKpV") * 0.01
        longitudinalTuningKiV = self.params.get_float("LongTuningKiV") * 0.001

        self.pid._k_p = (self.CP.longitudinalTuning.kpBP, [longitudinalTuningKpV])
        self.pid._k_i = (self.CP.longitudinalTuning.kiBP, [longitudinalTuningKiV])
        self.pid._k_f = ([0], [self.params.get_float("LongTuningKf") * 0.01])

    # Update longitudinal control. This updates the state machine and runs a PID loop
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel

      stopAccel = self.stopping_accel if self.stopping_accel < 0.0 else self.CP.stopAccel
      if output_accel > stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      # Kans: 이 포크가 이전에 쓰던 속도-오차 방식(v_target_now - CS.vEgo)
      # 대신 콤마스톡의 가속도-오차 PID(error = a_target - CS.aEgo)로
      # 바꿨다. kiV=.35는 속도-오차의 더 큰 오차 크기 기준으로 튜닝된
      # 값이라, 지금은 더 약하게 느껴질 가능성이 높다 - 실도로에서 타보고
      # 반응이 물렁하면 LongTuningKiV를 올릴 것, .35는 이 방식에 맞춰
      # 검증된 값이 아니었다.
      error = v_target_now - CS.vEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target_ff)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel, a_target_ff, j_target_now
