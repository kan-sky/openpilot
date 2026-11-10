import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from opendbc.car import DT_CTRL, structs
from opendbc.car.gm.values import CAR, CanBus, CruiseButtons, CAMERA_ACC_CAR, EV_CAR, SDGM_CAR

LongCtrlState = structs.CarControl.Actuators.LongControlState

# 정지 직후에는 레이더 앞차 속도가 흔들려 앞차 출발로 오검출된다 - 이 시간 동안 정지를 유지해야 Creep Release 허용
CREEP_RELEASE_MIN_STANDSTILL = 1.0  # s

# 카메라 ACC(롱컨 openpilot) RES 연속 송신
# 실제 버튼은 스티어링 모듈 0x1E1 흐름에서 RES 4프레임(카운터 연속, 약 0.12초)이 나가고 ECM이 STANDSTILL(4)을 푼다.
# 0.12초마다 1프레임씩 자체 카운터로 끼워 넣으면 ECM이 반응하지 않았다 (rlog-tbl02).
RES_BURST_FRAMES = 4
RES_BURST_MIN_GAP = 0.3  # s, 연속 송신 사이 최소 간격 (길게 누름으로 보이지 않게)
RES_BURST_TIMEOUT = 0.25  # s, 모듈 프레임 30ms x 4 = 0.12초에 여유

# Trailblazer 풀스톱 미사용 정지 유지 (TblNoFullStop)
# ECM은 2CB GasRegenFullStopActive=1을 받으면 STANDSTILL(4)로 들어가고, 4에서는 가스와 합성 RES를 무시한다 (rlog-tbl02/03/-4).
# FullStopActive를 보내지 않으면 0xD 풀스톱 브레이크도 못 쓰므로 일반 브레이크(0xA)를 최대값까지 올려 정지를 유지한다.
HOLD_BRAKE_STEP = 40  # 25Hz 송신 1회당 증가량, 약 0.4초에 최대값 도달

# Kans: GM 오토리쥼/오토크루즈 모듈
# - carcontroller 25Hz 블록에서 가스/브레이크 계산 직후, 2CB/브레이크 송신 직전에 호출
# - 앞차 정보는 이 모듈에서 radarState를 직접 구독


class GMAutoResume:
  def __init__(self, CP, params):
    self.CP = CP
    self.params = params  # CarControllerParams
    self.params_ = Params()
    self.sm = messaging.SubMaster(['radarState'])

    self.auto_resume_enabled = False
    self.auto_cruise_enabled = False
    self.cruiseDelay_time = 0.0
    self.resumeDelay_time = 0.0
    self.accel_force = 0
    self.no_full_stop_capable = CP.carFingerprint == CAR.CHEVROLET_TRAILBLAZER and CP.openpilotLongitudinalControl
    self.no_full_stop = False
    self.hold_brake = 0
    self.hold_logged = False
    self._update_params()

    self.resume_activate = False
    self.resume_frame = 0
    self.resume_fault_guard = 0
    self.lead_start_count = 0  # 앞차 출발시도 횟수
    self.standstill_frame = 0  # 정지 시작 프레임 (0 = 주행 중)
    self.activateCruise_after_brake = False
    self.last_button_frame = 0

    # GM AutoCruise
    self.autoCruise_activate = False
    self.autoCruise_frame = 0
    self.autoCruise_try_count = 0  # 오토크루즈 버튼 재시도 횟수
    self._pending_activateCruise = False

    # RES 연속 송신 상태
    self.res_burst_enabled = CP.carFingerprint in CAMERA_ACC_CAR and CP.openpilotLongitudinalControl
    self.res_burst_left = 0
    self.res_burst_last_cnt = -1
    self.res_burst_end_frame = 0
    self.res_burst_start_frame = 0

    # 앞차 정보
    self.lead_present = False
    self.lead_distance = float('inf')
    self.lead_vrel = 0.0

  def _update_params(self):
    self.cruiseDelay_time = self.params_.get_float("CruiseDelay") * 0.01
    self.resumeDelay_time = self.params_.get_float("ResumeDelay") * 0.01
    self.auto_cruise_enabled = self.params_.get_int("AutoCruiseControl") > 0
    self.auto_resume_enabled = self.params_.get_int("AutoEngage") == 2
    self.accel_force = self.params_.get_int("AccelForce")
    self.no_full_stop = self.no_full_stop_capable and self.params_.get_int("TblNoFullStop") > 0

  def _update_lead(self):
    try:
      self.sm.update(0)
      lead = self.sm['radarState'].leadOne
    except Exception:
      lead = None
    if lead is not None and lead.status:
      self.lead_present = True
      self.lead_distance = float(lead.dRel)
      self.lead_vrel = float(lead.vRel)
    else:
      self.lead_present = False
      self.lead_distance = float('inf')
      self.lead_vrel = 0.0

  def update(self, CC, CS, frame, apply_gas, apply_brake, at_full_stop, send_btn):
    """반환: (send_gas, apply_brake, at_full_stop, acc_engaged)"""
    actuators = CC.actuators
    if frame % 100 == 0:
      self._update_params()
    self._update_lead()

    auto_cruise_enabled = self.auto_cruise_enabled
    auto_resume_enabled = self.auto_resume_enabled
    auto_longcontrol = auto_cruise_enabled or auto_resume_enabled
    pcm_standstill = CS.out.cruiseState.standstill  # == AccState.STANDSTILL

    if self.CP.autoResumeSng:
      resume = actuators.longControlState == LongCtrlState.starting or CC.cruiseControl.resume
      at_full_stop = at_full_stop and not resume

    if CC.cruiseControl.resume and pcm_standstill:
      if self.CP.carFingerprint in EV_CAR:
        acc_engaged = False
      else:
        acc_engaged = CC.enabled
    else:
      acc_engaged = CC.enabled

    # Kans: AutoResume 윈도우 중 2CB ACC state 보정
    starting = (actuators.longControlState == LongCtrlState.starting)
    auto_resume_window = (auto_resume_enabled and self.resume_frame != 0 and
      CS.out.vEgo < 3.5 and
      not CS.out.brakePressed and
      not CS.out.gasPressed)
    if auto_resume_window and (starting or CS.out.cruiseState.enabled):
      acc_engaged = True
      at_full_stop = False

    if CS.out.activateCruise > 0 and not self._pending_activateCruise:
      cloudlog.warning(f"[gm autocruise] activateCruise request: vEgo={CS.out.vEgo:.2f} "
                       f"brakePressed={CS.out.brakePressed} cruiseState.enabled={CS.out.cruiseState.enabled}")

    # 카메라 ACC ECM은 minEnableSpeed 미만에서 브레이크 없이 체결하면 폴트 - 그 속도 이상에서만 오토크루즈
    # (정지 중 브레이크 밟고 체결은 허용되지만, 오토크루즈는 브레이크 해제 상태에서만 버튼을 보내므로 해당 없음)
    if CS.out.activateCruise > 0 and not CS.out.brakePressed and CS.out.vEgo >= self.CP.minEnableSpeed:
      self._pending_activateCruise = True

    # 끄기 요청(-1)이 오거나 시도 구간 밖에서 크루즈가 이미 켜져 있으면 대기 요청 폐기
    # (남겨두면 나중에 운전자가 크루즈를 끈 직후 SET이 나가 재체결됨)
    cruise_off_request = CS.out.activateCruise < 0
    if cruise_off_request or (CS.out.cruiseState.enabled and not self.autoCruise_activate):
      if self._pending_activateCruise or (cruise_off_request and self.autoCruise_activate):
        cloudlog.warning(f"[gm autocruise] pending request dropped: activateCruise={CS.out.activateCruise} "
                         f"cruiseState.enabled={CS.out.cruiseState.enabled}")
      self._pending_activateCruise = False
      if cruise_off_request:
        self.autoCruise_activate = False
        self.autoCruise_frame = 0
        self.autoCruise_try_count = 0

    # Kans: 리쥼 + 가속펄스
    lead_drel = self.lead_distance
    lead_vrel = self.lead_vrel
    has_lead = self.lead_present and np.isfinite(self.lead_distance)
    near_stop_ego = (CS.out.vEgo < 0.3)
    resume_standstill = CS.out.standstill or CS.out.cruiseState.standstill
    if resume_standstill:
      if self.standstill_frame == 0:
        self.standstill_frame = frame
    else:
      self.standstill_frame = 0
    standstill_dt = (frame - self.standstill_frame) * DT_CTRL if self.standstill_frame != 0 else 0.0
    reopen_delay = max(self.resumeDelay_time * 1.5, 0.28)

    # follow 조건(정지/재출발 구간에서 vRel 흔들림 감안)
    lead_follow_ok = has_lead and (2.0 < lead_drel < 15.0) and (lead_vrel > 0.3)

    # 앞차 출발: Creep Release는 정지 완료 후에만 허용 (정지 직전 오검출로 stopping brake가 풀리는 것 방지)
    raw_lead_start = (has_lead and (4.0 < lead_drel < 10.0) and (lead_vrel > 0.4) and (near_stop_ego or resume_standstill))

    if raw_lead_start:
      self.lead_start_count = min(self.lead_start_count + 1, 5)
    else:
      self.lead_start_count = 0

    lead_start = self.lead_start_count >= 3

    # Kans: Creep Release Window
    # 완전 정지 후 앞차 출발이 확인되면 starting 진입 전 브레이크를 잠시 풀어준다.
    # resume_frame은 실제 AutoResume까지 이어지는 리쥼윈도우로 유지한다.
    # ECM이 STANDSTILL(4)이면 가스를 받지 않으므로 브레이크를 풀지 않는다 (운전자 RES 대기)
    if (auto_resume_enabled and lead_start and resume_standstill and self.resume_frame == 0 and
        standstill_dt >= CREEP_RELEASE_MIN_STANDSTILL and not pcm_standstill and not self.no_full_stop and
        not CS.out.brakePressed and not CS.out.gasPressed):
      self.resume_frame = frame
      self.resume_fault_guard = -1
      self.last_button_frame = frame - int(0.12 / DT_CTRL)
      cloudlog.warning(f"[gm autoresume] creep-release window opened: leadDRel={lead_drel:.2f} "
                       f"leadVRel={lead_vrel:.2f} vEgo={CS.out.vEgo:.2f} cstill={int(pcm_standstill)} standstill={standstill_dt:.2f}s")

    # resume_frame 갱신 후 계산
    creep_dt = (frame - self.resume_frame) * DT_CTRL if self.resume_frame != 0 else 999.0

    if self.resume_fault_guard == -1:
      # starting 진입 시 Creep Release 종료. resume_frame은 RES 동작을 위해 유지한다.
      if starting:
        self.resume_fault_guard = 0
      # 실제 차량이 움직이기 시작하면 브레이크 release 단계 종료.
      elif CS.out.vEgo > 0.4:
        self.resume_fault_guard = 0
      # 완전 정지 상태에서 확인된 앞차 출발에 한해서만 브레이크 해제.
      elif creep_dt < 0.22 and lead_start and resume_standstill:
        apply_brake = 0
      # Creep Release 시간만 종료. resume_frame은 아래 AutoResume cleanup에서 정리한다.
      else:
        self.resume_fault_guard = 0

    # AutoCruise: 크루즈 OFF 상태에서, 메인 활성(activateCruise) 신호가 있을 때
    if auto_cruise_enabled and self._pending_activateCruise and not CS.out.cruiseState.enabled:
      # Kans: AutoCruise (0.25초 윈도 안에 최대 2회 버튼 시도)
      if not self.autoCruise_activate:
        self.autoCruise_activate = True
        self.autoCruise_frame = frame
        self.autoCruise_try_count = 0

      if self.autoCruise_activate:
        within_window = (frame - self.autoCruise_frame) * DT_CTRL <= self.cruiseDelay_time  # 예: 0.25초

        if within_window and self.autoCruise_try_count < 2:
          # Kans: GM의 RESUME은 완전 정지 후 크루즈를 복구하지 못하고 SET만 된다 (실차 확인) - 자동체결은 DECEL_SET
          # ON 요청(1)이 유지될 때만 송신
          if (frame - self.last_button_frame) * DT_CTRL >= 0.12 and CS.out.activateCruise == 1:
            btn = CruiseButtons.DECEL_SET
            cloudlog.warning(f"[gm autocruise] send btn={btn} try={self.autoCruise_try_count} activateCruise={CS.out.activateCruise}")
            send_btn(btn)
            self.last_button_frame = frame
            self.autoCruise_try_count += 1

        # 종료 조건: 시간 초과 / 2회 시도 완료 / 크루즈 실제 ON
        if (not within_window) or (self.autoCruise_try_count >= 2) or CS.out.cruiseState.enabled:
          cloudlog.warning(f"[gm autocruise] window closed: tries={self.autoCruise_try_count} "
                           f"cruiseState.enabled={CS.out.cruiseState.enabled} timed_out={not within_window}")
          self.autoCruise_activate = False
          self.autoCruise_frame = 0
          self.autoCruise_try_count = 0
          self._pending_activateCruise = False

    # Kans: Auto Resume (RES only)
    # 풀스톱 미사용 모드는 ECM이 ACTIVE(1)에 머무르므로 RES 없이 longcontrol starting 가스로 출발한다
    elif auto_resume_enabled and starting and not self.no_full_stop:
      if self.resume_frame == 0 or self.resume_activate:
        self.resume_frame = frame
        self.resume_activate = False
        self.resume_fault_guard = 0
        # starting 진입 즉시 1회 RES 가능하도록 버튼 타이머 당김
        self.last_button_frame = frame - int(0.12 / DT_CTRL)
        self.activateCruise_after_brake = False

      # 브레이크 apply는 1틱만 (정규 브레이크 송신에 실리도록)
      if not self.activateCruise_after_brake:
        apply_brake = max(apply_brake, int(self.params.NEAR_STOP_BRAKE_PHASE))
        self.activateCruise_after_brake = True

      # starting이어도 standstill 확정 전에는 RES 버튼을 보내지 않음. SoftDisableAlert(Alert) 방지용.
      resume_ready_standstill = (CS.out.standstill or CS.out.cruiseState.standstill)

      if not resume_ready_standstill:
        self.resume_fault_guard = 0
        self.resume_activate = False

      # RES spam
      elif self.CP.carFingerprint in SDGM_CAR:
        # SDGM: starting이 짧을 수 있으니, 창이 열리면 1회는 반드시 쏨
        if self.resume_fault_guard == 0:
          cloudlog.warning(f"[gm autoresume] first RES_ACCEL sent (SDGM): leadDRel={lead_drel:.2f} vEgo={CS.out.vEgo:.2f}")
          send_btn(CruiseButtons.RES_ACCEL)
          self.last_button_frame = frame
          self.resume_fault_guard = 1
        # 2회차부터 0.12간격 유지
        elif self.resume_fault_guard < 2:
          if (frame - self.last_button_frame) * DT_CTRL >= 0.12:
            send_btn(CruiseButtons.RES_ACCEL)
            self.last_button_frame = frame
            self.resume_fault_guard += 1
        # SDGM: 창 닫기 시간을 너무 길게 끌지 않음(메인 꺼짐/flicker 방지)
        if (frame - self.resume_frame) * DT_CTRL >= reopen_delay:
          self.resume_activate = True
      else:
        # other cars: resumeDelay 이후부터 기존 방식으로
        ready = (self.resume_fault_guard == 0) or CS.out.cruiseState.enabled
        if ready and (self.resume_fault_guard < 2):
          # 버튼 주기 0.12초, 리쥼실패율 가장 낮은 값으로 보임.
          if (frame - self.last_button_frame) * DT_CTRL >= 0.12:
            if self.resume_fault_guard == 0:
              cloudlog.warning(f"[gm autoresume] first RES_ACCEL sent: leadDRel={lead_drel:.2f} vEgo={CS.out.vEgo:.2f} "
                               f"cstill={int(pcm_standstill)} resumeDelay={self.resumeDelay_time:.2f}")
            if self.res_burst_enabled:
              # 실제 송신은 100Hz 루프의 update_res_burst에서 모듈 프레임마다
              self._request_res_burst(frame, CS)
            else:
              send_btn(CruiseButtons.RES_ACCEL)
            self.last_button_frame = frame
            self.resume_fault_guard += 1  # 송신횟수 기록
        # 리쥼버튼 중단까지 지연시간(0.16~0.20)
        if (frame - self.resume_frame) * DT_CTRL >= self.resumeDelay_time:
          self.resume_activate = True

    else:
      self.activateCruise_after_brake = False
      if auto_resume_enabled:  # 오토리쥼이 진행중이면
        if self.resume_frame > 0 and (frame - self.resume_frame) * DT_CTRL > reopen_delay:
          self.resume_frame = 0
          self.resume_activate = False
          self.resume_fault_guard = 0
      # 오토크루즈 초기화도 여기서(오토크루즈 분기 진입 안 했을 때)
      self.autoCruise_try_count = 0
      self.autoCruise_frame = 0
      self.autoCruise_activate = False

    # RES 로직 이후에 dt/lead_ok/resume_active 계산 (윈도우 갱신 반영)
    resume_dt = (frame - self.resume_frame) * DT_CTRL if (self.resume_frame != 0) else 999.0
    no_lead_hill_ok = (not has_lead) and (self.resume_frame != 0) and (CS.out.vEgo < 0.5) and (resume_dt < 0.35)
    lead_ok = lead_follow_ok or no_lead_hill_ok

    # resume_active는 "가스 펄스" 조건 -> starting/크루즈ON 게이트
    resume_active = ((self.resume_frame != 0) and auto_longcontrol and not self.no_full_stop and (CS.out.vEgo < 3.5) and lead_ok and
                     (starting or CS.out.cruiseState.enabled) and (CS.out.standstill or CS.out.cruiseState.standstill))

    # 2CB: 실제 가스 송신
    if resume_active:
      send_gas = max(0, int(max(apply_gas, self.accel_force)))
      at_full_stop = False
    else:
      # GasRegenCmdActive needs to be 1 to avoid cruise faults. It describes the ACC state, not actuation
      send_gas = apply_gas

    if self.no_full_stop:
      apply_brake = self._update_no_full_stop_hold(CC, CS, apply_brake, pcm_standstill)
      at_full_stop = False

    return send_gas, apply_brake, at_full_stop, acc_engaged

  def _update_no_full_stop_hold(self, CC, CS, apply_brake, pcm_standstill):
    # 정지 상태에서 정지 브레이크 요청이 있을 때만 브레이크를 최대값까지 올린다 (출발 요청이면 apply_brake=0으로 즉시 해제)
    if CC.longActive and CS.out.standstill and apply_brake > 0:
      self.hold_brake = min(self.hold_brake + HOLD_BRAKE_STEP, self.params.MAX_BRAKE)
      if not self.hold_logged:
        cloudlog.warning(f"[gm nofullstop] hold start: cstill={int(pcm_standstill)} brake={apply_brake}")
        self.hold_logged = True
      return max(apply_brake, self.hold_brake)

    if self.hold_logged:
      cloudlog.warning(f"[gm nofullstop] hold end: cstill={int(pcm_standstill)} vEgo={CS.out.vEgo:.2f} "
                       f"longActive={CC.longActive} brake={apply_brake}")
      self.hold_logged = False
    self.hold_brake = 0
    return apply_brake

  def _request_res_burst(self, frame, CS):
    if self.res_burst_left > 0 or (frame - self.res_burst_end_frame) * DT_CTRL < RES_BURST_MIN_GAP:
      return
    self.res_burst_left = RES_BURST_FRAMES
    self.res_burst_last_cnt = -1
    self.res_burst_start_frame = frame
    cloudlog.warning(f"[gm autoresume] RES burst start: btnCnt={int(CS.buttons_counter) & 0x3} "
                     f"pcm={int(getattr(CS, 'pcm_acc_status', -1))} vEgo={CS.out.vEgo:.2f}")

  def update_res_burst(self, CS, frame, send_res):
    # 100Hz 호출: 스티어링 모듈의 새 0x1E1 프레임이 들어올 때마다 그 카운터+1로 RES 1프레임
    # (모듈 N → RES N+1 → 모듈 N+1 → RES N+2 ... 로 실제 버튼처럼 카운터가 이어지게)
    if self.res_burst_left <= 0:
      return
    # 운전자 페달 개입 또는 모듈 프레임이 끊겨 늦어지면 남은 RES는 버린다 (뒤늦은 RES 방지)
    if CS.out.brakePressed or CS.out.gasPressed or (frame - self.res_burst_start_frame) * DT_CTRL > RES_BURST_TIMEOUT:
      cloudlog.warning(f"[gm autoresume] RES burst aborted: left={self.res_burst_left} brake={int(CS.out.brakePressed)} "
                       f"gas={int(CS.out.gasPressed)}")
      self.res_burst_left = 0
      self.res_burst_end_frame = frame
      return
    cnt = int(CS.buttons_counter) & 0x3
    if cnt == self.res_burst_last_cnt:
      return
    self.res_burst_last_cnt = cnt
    send_res((cnt + 1) & 0x3)
    self.res_burst_left -= 1
    if self.res_burst_left == 0:
      self.res_burst_end_frame = frame
      cloudlog.warning(f"[gm autoresume] RES burst end: pcm={int(getattr(CS, 'pcm_acc_status', -1))} vEgo={CS.out.vEgo:.2f}")

  def button_bus(self):
    # 0x1E1 송신 버스: gm.h 허용목록에 맞춤 (카메라 스톡롱=2, 그 외=0)
    if self.CP.carFingerprint in CAMERA_ACC_CAR and not self.CP.openpilotLongitudinalControl:
      return CanBus.CAMERA
    return CanBus.POWERTRAIN
