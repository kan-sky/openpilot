from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs, create_gas_interceptor_command, ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.gm import gmcan
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.values import DBC, CanBus, CarControllerParams, CruiseButtons, AccState, EV_CAR, SDGM_CAR, ALT_ACCS, CAMERA_ACC_CAR
from opendbc.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import apply_deadzone

VisualAlert = structs.CarControl.HUDControl.VisualAlert
NetworkLocation = structs.CarParams.NetworkLocation
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15

# Constants for pitch compensation
PITCH_DEADZONE = 0.01 # [radians] 0.01 ≈ 1% grade
BRAKE_PITCH_FACTOR_BP = [5., 10.] # [m/s] smoothly revert to planned accel at low speeds
BRAKE_PITCH_FACTOR_V = [0., 1.] # [unitless in [0,1]]; don't touch

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.start_time = 0.
    self.apply_torque_last = 0
    self.apply_gas = 0
    self.apply_brake = 0
    # kans: button spam
    self.apply_speed = 0
    self.frame = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.cancel_counter = 0
    self.pedal_steady = 0.

    self.lka_steering_cmd_counter = 0
    self.lka_icon_status_last = (False, False)

    self.params = CarControllerParams(self.CP)
    self.params_ = Params() # kans: button spam

    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.packer_obj = CANPacker(DBC[self.CP.carFingerprint][Bus.radar])
    self.packer_ch = CANPacker(DBC[self.CP.carFingerprint][Bus.chassis])

    self.long_pitch = False
    self.use_ev_tables = False

    self.pitch = FirstOrderFilter(0., 0.09 * 4, DT_CTRL * 4)  # runs at 25 Hz
    self.accel_g = 0.0

    self.steerDeltaUpOrg = self.steerDeltaUp = self.steerDeltaUpLC = self.params.STEER_DELTA_UP
    self.steerDeltaDownOrg = self.steerDeltaDown = self.steerDeltaDownLC = self.params.STEER_DELTA_DOWN

    # GM AutoCruise / AutoResume
    self.activateCruise_after_brake = False
    self.autoCruise_activate = False
    self.autoCruise_frame = 0
    self.autoCruise_try_count = 0  # 오토크루즈 버튼 재시도 횟수
    self._pending_activateCruise = False

    self.resume_activate = False
    self.resume_frame = 0
    self.resume_fault_guard = 0
    self.lead_start_count = 0  # 앞차 출발시도 횟수

    self.btn_rc_pt = -1
    self.btn_rc_cam = -1
    self._brk_rc = -1

    self.cruiseDelay_time = 0.0
    self.resumeDelay_time = 0.0
    self._hill_detected = False
    self.accel_force = 0

    # AutoHold post-cruise guard timestamp
    self.autoHold_cruise_cancel_frame = -1

  def update(self, CC, CS, now_nanos):
    if self.frame % 50 == 0:
      steerMax = self.params_.get_int("CustomSteerMax")
      steerDeltaUp = self.params_.get_int("CustomSteerDeltaUp")
      steerDeltaDown = self.params_.get_int("CustomSteerDeltaDown")
      steerDeltaUpLC = self.params_.get_int("CustomSteerDeltaUpLC")
      steerDeltaDownLC = self.params_.get_int("CustomSteerDeltaDownLC")

      if steerMax > 0:
        self.params.STEER_MAX = steerMax
      if steerDeltaUp > 0:
        self.steerDeltaUp = steerDeltaUp
      else:
        self.steerDeltaUp = self.steerDeltaUpOrg
      if steerDeltaDown > 0:
        self.steerDeltaDown = steerDeltaDown
      else:
        self.steerDeltaDown = self.steerDeltaDownOrg
      if steerDeltaUpLC > 0:
        self.steerDeltaUpLC = steerDeltaUpLC
      else:
        self.steerDeltaUpLC = self.steerDeltaUp
      if steerDeltaDownLC > 0:
        self.steerDeltaDownLC = steerDeltaDownLC
      else:
        self.steerDeltaDownLC = self.steerDeltaDown

    self.long_pitch = self.params_.get_bool("LongPitch")
    self.use_ev_tables = self.params_.get_bool("EVTable")

    actuators = CC.actuators
    accel = brake_accel = actuators.accel
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    if hud_control.modelDesire in [3, 4]:
      self.params.STEER_DELTA_UP = self.steerDeltaUpLC
      self.params.STEER_DELTA_DOWN = self.steerDeltaDownLC
    else:
      self.params.STEER_DELTA_UP = self.steerDeltaUp
      self.params.STEER_DELTA_DOWN = self.steerDeltaDown

    can_sends = []

    # Steering (latest comma base)
    steer_step = self.params.STEER_STEP if CC.latActive else self.params.INACTIVE_STEER_STEP

    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      out_of_sync = self.lka_steering_cmd_counter % 4 != (CS.cam_lka_steering_cmd_counter + 1) % 4
      if CS.loopback_lka_steering_cmd_ts_nanos == 0 or out_of_sync:
        steer_step = self.params.STEER_STEP

    self.lka_steering_cmd_counter += 1 if CS.loopback_lka_steering_cmd_updated else 0

    last_lka_steer_msg_ms = (now_nanos - CS.loopback_lka_steering_cmd_ts_nanos) * 1e-6
    if (self.frame - self.last_steer_frame) >= steer_step and last_lka_steer_msg_ms > MIN_STEER_MSG_INTERVAL_MS:
      if CS.loopback_lka_steering_cmd_ts_nanos == 0:
        self.lka_steering_cmd_counter = CS.pt_lka_steering_cmd_counter + 1

      if CC.latActive:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)
      else:
        apply_torque = 0

      self.last_steer_frame = self.frame
      self.apply_torque_last = apply_torque
      idx = self.lka_steering_cmd_counter % 4
      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_torque, idx, CC.latActive))

    if self.CP.openpilotLongitudinalControl:
      # Gas/regen, brake, AutoHold, AutoCruise, AutoResume, UI - 25Hz
      if self.frame % 4 == 0:
        friction_sent_this_tick = False
        self.cruiseDelay_time = self.params_.get_float("CruiseDelay") * 0.01
        self.resumeDelay_time = self.params_.get_float("ResumeDelay") * 0.01
        auto_cruise_enabled = self.params_.get_int("AutoCruiseControl") > 0
        auto_resume_enabled = self.params_.get_int("AutoEngage") == 2
        self.accel_force = self.params_.get_int("AccelForce")
        auto_longcontrol = auto_cruise_enabled or auto_resume_enabled

        stopping = actuators.longControlState == LongCtrlState.stopping

        if self.long_pitch and len(CC.orientationNED) > 1:
          self.pitch.update(CC.orientationNED[1])
          self.accel_g = ACCELERATION_DUE_TO_GRAVITY * apply_deadzone(self.pitch.x, PITCH_DEADZONE)
          accel += self.accel_g
          brake_accel = actuators.accel + self.accel_g * np.interp(CS.out.vEgo, BRAKE_PITCH_FACTOR_BP, BRAKE_PITCH_FACTOR_V)

        at_full_stop = CC.longActive and CS.out.standstill
        near_stop = CC.longActive and (abs(CS.out.vEgo) < self.params.NEAR_STOP_BRAKE_PHASE)
        interceptor_gas_cmd = 0

        # 언덕감지(accel_g가 클수록 높은 경사)
        if self.accel_g > 0.25:
          self._hill_detected = True
        else:
          self._hill_detected = False

        # Regular gas/brake calculation
        if not CC.longActive:
          self.apply_gas = self.params.INACTIVE_REGEN
          self.apply_brake = 0
        elif near_stop and stopping and not CC.cruiseControl.resume:
          self.apply_gas = self.params.INACTIVE_REGEN
          self.apply_brake = int(min(-100 * self.CP.stopAccel, self.params.MAX_BRAKE))
        else:
          if self.CP.carFingerprint in EV_CAR and self.use_ev_tables:
            self.params.update_ev_gas_brake_threshold(CS.out.vEgo)
            self.apply_gas = int(round(np.interp(accel if self.long_pitch else actuators.accel, self.params.EV_GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
            self.apply_brake = int(round(np.interp(brake_accel if self.long_pitch else actuators.accel, self.params.EV_BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))
          else:
            self.apply_gas = int(round(np.interp(accel if self.long_pitch else actuators.accel, self.params.GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
            self.apply_brake = int(round(np.interp(brake_accel if self.long_pitch else actuators.accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))

          if stopping:
            self.apply_gas = self.params.INACTIVE_REGEN

        if self.CP.enableGasInterceptor and self.apply_gas > self.params.INACTIVE_REGEN and CS.out.cruiseState.standstill:
          # "Tap" the accelerator pedal to re-engage ACC
          interceptor_gas_cmd = self.params.SNG_INTERCEPTOR_GAS
          self.apply_brake = 0
          self.apply_gas = self.params.INACTIVE_REGEN

        idx = (self.frame // 4) % 4

        if self.CP.enableGasInterceptor:
          can_sends.append(create_gas_interceptor_command(self.packer_pt, interceptor_gas_cmd, idx))
        if self.CP.carFingerprint not in ALT_ACCS:
          friction_brake_bus = CanBus.CHASSIS
          # GM Camera exceptions
          # TODO: can we always check the longControlState?
          if self.CP.networkLocation == NetworkLocation.fwdCamera:
            at_full_stop = at_full_stop and stopping
            friction_brake_bus = CanBus.POWERTRAIN
            if self.CP.carFingerprint in SDGM_CAR:
              friction_brake_bus = CanBus.CAMERA

          # Kans: 크루즈 해제 직후 AutoHold 브레이크 명령 지연(1.5s)
          if CC.enabled or CS.out.cruiseState.enabled:
            self.autoHold_cruise_cancel_frame = self.frame

          auto_hold_cruise_cancel_delay = (
            self.autoHold_cruise_cancel_frame >= 0 and
            (self.frame - self.autoHold_cruise_cancel_frame) * DT_CTRL < 1.5
          )
          # Kans: AutoHold 조건
          auto_hold_cmd = (
            not CC.longActive and
            not CC.enabled and
            not CS.out.cruiseState.enabled and
            CS.autoHold and
            CS.autoHoldActive and
            not CS.out.gasPressed and
            CS.out.gearShifter in ['drive', 'low'] and
            CS.out.vEgo < 0.05 and
            not CS.out.regenBraking
          )

          if self.CP.autoResumeSng:
            resume = actuators.longControlState == LongCtrlState.starting or CC.cruiseControl.resume
            at_full_stop = at_full_stop and not resume

          if CC.cruiseControl.resume and CS.pcm_acc_status == AccState.STANDSTILL:
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

          # AutoHold가 실제 운전자 브레이크 개입으로 만들어진 상태인지
          manual_auto_hold = (
            CS.autoHold and
            (CS.autoHoldActive or CS.autoHoldActivated or CS.out.autoHoldActivated) and
            not CS.out.cruiseState.enabled and
            not CC.enabled
          )
          # 오토크루즈/오토리쥼 차단은 수동 AutoHold일 때만
          auto_hold_block_cruise = manual_auto_hold
          if auto_hold_block_cruise:
            self._pending_activateCruise = False
            self.autoCruise_activate = False
            self.autoCruise_frame = 0
            self.autoCruise_try_count = 0

            self.resume_frame = 0
            self.resume_activate = False
            self.resume_fault_guard = 0
            self.activateCruise_after_brake = False

          if CS.out.activateCruise > 0 and not auto_hold_block_cruise and not CS.out.brakePressed:
            self._pending_activateCruise = True

          # Kans: 리쥼 + 가속펄스
          # lead from CarState (user fields)
          lead_drel = CS.lead_distance
          lead_vrel = CS.lead_vrel
          has_lead = CS.lead_present and (CS.lead_distance is not None) and np.isfinite(CS.lead_distance)
          near_stop_ego = (CS.out.vEgo < 0.3)
          resume_standstill = CS.out.standstill or CS.out.cruiseState.standstill
          reopen_delay = max(self.resumeDelay_time * 1.5, 0.28)

          # follow 조건(정지/재출발 구간에서 vRel 흔들림 감안)
          lead_follow_ok = has_lead and (2.0 < lead_drel < 15.0) and (lead_vrel > 0.3)

          # 앞차 출발:
          # Creep Release는 정지 완료 후에만 허용한다.
          # 정지 직전 vEgo<0.3 상태에서 오검출로 stopping brake가 풀리는 것을 방지.
          raw_lead_start = (has_lead and (4.0 < lead_drel < 10.0) and (lead_vrel > 0.4) and resume_standstill)

          if raw_lead_start:
            self.lead_start_count = min(self.lead_start_count + 1, 5)
          else:
            self.lead_start_count = 0

          lead_start = self.lead_start_count >= 3

          # Kans: Creep Release Window
          # 완전 정지 후 앞차 출발이 확인되면 starting 진입 전 브레이크를 잠시 풀어준다.
          # resume_frame은 실제 AutoResume까지 이어지는 리쥼윈도우로 유지한다.
          if (auto_resume_enabled and lead_start and resume_standstill and self.resume_frame == 0 and not CS.out.brakePressed and not CS.out.gasPressed and not auto_hold_block_cruise):
            self.resume_frame = self.frame
            self.resume_fault_guard = -1
            self.last_button_frame = self.frame - int(0.12 / DT_CTRL)

          # resume_frame 갱신 후 계산
          creep_dt = (self.frame - self.resume_frame) * DT_CTRL if self.resume_frame != 0 else 999.0

          if self.resume_fault_guard == -1:
            # starting 진입 시 Creep Release 종료.
            # resume_frame은 RES 동작을 위해 유지한다.
            if starting:
              self.resume_fault_guard = 0

            # 실제 차량이 움직이기 시작하면 브레이크 release 단계 종료.
            elif CS.out.vEgo > 0.4:
              self.resume_fault_guard = 0

            # 완전 정지 상태에서 확인된 앞차 출발에 한해서만 브레이크 해제.
            elif creep_dt < 0.22 and lead_start and resume_standstill:
              self.apply_brake = 0

            # Creep Release 시간만 종료.
            # resume_frame은 여기서 지우지 않고 아래 AutoResume cleanup에서 정리한다.
            else:
              self.resume_fault_guard = 0


          # AutoCruise: 크루즈 OFF 상태에서, 메인 활성(activateCruise) 신호가 있을 때
          if auto_cruise_enabled and self._pending_activateCruise and not CS.out.cruiseState.enabled:
            # Kans: AutoCruise (0.25초 윈도 안에 최대 2회 버튼 시도)
            if not self.autoCruise_activate:
              self.autoCruise_activate = True
              self.autoCruise_frame = self.frame
              self.autoCruise_try_count = 0

            if self.autoCruise_activate:
              within_window = (self.frame - self.autoCruise_frame) * DT_CTRL <= self.cruiseDelay_time  # 예: 0.25초

              if within_window and self.autoCruise_try_count < 2:
                if (self.frame - self.last_button_frame) * DT_CTRL >= 0.12:
                  btn = CruiseButtons.RES_ACCEL if CS.out.activateCruise == 1 else CruiseButtons.DECEL_SET
                  self.send_btn(CS, can_sends, btn)
                  self.last_button_frame = self.frame
                  self.autoCruise_try_count += 1

              # 종료 조건: 시간 초과 / 2회 시도 완료 / 크루즈 실제 ON
              if (not within_window) or (self.autoCruise_try_count >= 2) or CS.out.cruiseState.enabled:
                self.autoCruise_activate = False
                self.autoCruise_frame = 0
                self.autoCruise_try_count = 0
                self._pending_activateCruise = False

          # Kans: Auto Resume (RES only)
          elif auto_resume_enabled and actuators.longControlState == LongCtrlState.starting and not manual_auto_hold:
            if self.resume_frame == 0 or self.resume_activate:
              self.resume_frame = self.frame
              self.resume_activate = False
              self.resume_fault_guard = 0
              # starting 진입 즉시 1회 RES 가능하도록 버튼 타이머 당김
              self.last_button_frame = self.frame - int(0.12 / DT_CTRL)
              self.activateCruise_after_brake = False

            # 브레이크 apply는 1틱만 (정규 브레이크 송신에 실리도록)
            if not self.activateCruise_after_brake:
              self.apply_brake = max(self.apply_brake, int(self.params.NEAR_STOP_BRAKE_PHASE))
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
                self.send_btn(CS, can_sends, CruiseButtons.RES_ACCEL)
                self.last_button_frame = self.frame
                self.resume_fault_guard = 1
              # 2회차부터 0.12간격 유지
              elif self.resume_fault_guard < 2:
                if (self.frame - self.last_button_frame) * DT_CTRL >= 0.12:
                  self.send_btn(CS, can_sends, CruiseButtons.RES_ACCEL)
                  self.last_button_frame = self.frame
                  self.resume_fault_guard += 1
              # SDGM: 창 닫기 시간을 너무 길게 끌지 않음(메인 꺼짐/flicker 방지)
              if (self.frame - self.resume_frame) * DT_CTRL >= reopen_delay:
                self.resume_activate = True
            else:
              # other cars: resumeDelay 이후부터 기존 방식으로
              ready = (self.resume_fault_guard == 0) or CS.out.cruiseState.enabled
              if ready and (self.resume_fault_guard < 2):
                # 버튼 주기 0.12초, 리쥼실패율 가장 낮은 값으로 보임.
                if (self.frame - self.last_button_frame) * DT_CTRL >= 0.12:
                  self.send_btn(CS, can_sends, CruiseButtons.RES_ACCEL)
                  self.last_button_frame = self.frame
                  self.resume_fault_guard += 1  # 송신횟수 기록
              # 리쥼버튼 중단까지 지연시간(0.16~0.20)
              if (self.frame - self.resume_frame) * DT_CTRL >= self.resumeDelay_time:
                self.resume_activate = True

          else:
            self.activateCruise_after_brake = False
            if auto_resume_enabled:  # 오토리쥼이 진행중이면
              if self.resume_frame > 0 and (self.frame - self.resume_frame) * DT_CTRL > reopen_delay:
                self.resume_frame = 0
                self.resume_activate = False
                self.resume_fault_guard = 0
            # 오토크루즈 초기화도 여기서(오토크루즈 분기 진입 안 했을 때)
            self.autoCruise_try_count = 0
            self.autoCruise_frame = 0
            self.autoCruise_activate = False

          # RES 로직 이후에 dt/lead_ok/resume_active 계산 (윈도우 갱신 반영)
          resume_dt = (self.frame - self.resume_frame) * DT_CTRL if (self.resume_frame != 0) else 999.0
          no_lead_hill_ok = (not has_lead) and (self.resume_frame != 0) and (CS.out.vEgo < 0.5) and (resume_dt < 0.35)
          lead_ok = lead_follow_ok or no_lead_hill_ok

          # resume_active는 "가스 펄스" 조건 -> starting/크루즈ON 게이트
          resume_active = (self.resume_frame != 0) and auto_longcontrol and (CS.out.vEgo < 3.5) and lead_ok and (starting or CS.out.cruiseState.enabled) and (CS.out.standstill or CS.out.cruiseState.standstill)

          # 2CB: 실제 가스 송신
          if resume_active:
            send_gas = max(0, int(max(self.apply_gas, self.accel_force)))
            at_full_stop = False
            can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, send_gas, idx, acc_engaged, at_full_stop))
          else:
            # GasRegenCmdActive needs to be 1 to avoid cruise faults. It describes the ACC state, not actuation
            can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_gas, idx, acc_engaged, at_full_stop))

          # Kans: 정규 브레이크 로직
          self._brk_rc = (self._brk_rc + 1) & 0x3
          brk_idx_base = self._brk_rc

          if auto_hold_cmd:
            hold_brake = max(self.apply_brake, self.params.NEAR_STOP_BRAKE_PHASE)
            hold_near_stop = CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE
            can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, friction_brake_bus, hold_brake, brk_idx_base, False, hold_near_stop, False, self.CP))
            CS.autoHoldActivated = True
          else:
            can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, friction_brake_bus, self.apply_brake, brk_idx_base, CC.enabled, near_stop, at_full_stop, self.CP))
            CS.autoHoldActivated = False

          # Send dashboard UI commands (ACC status)
          send_fcw = hud_alert == VisualAlert.fcw
          can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, CC.enabled,
                                                              hud_v_cruise * CV.MS_TO_KPH, hud_control, send_fcw))
      else:
        # to keep accel steady for logs when not sending gas
        accel += self.accel_g

      # Radar needs to know current speed and yaw rate (50hz),
      # and that ADAS is alive (10hz)
      if not self.CP.radarUnavailable:
        tt = self.frame * DT_CTRL
        time_and_headlights_step = 10
        if self.frame % time_and_headlights_step == 0:
          idx = (self.frame // time_and_headlights_step) % 4
          can_sends.append(gmcan.create_adas_time_status(CanBus.OBSTACLE, int((tt - self.start_time) * 60), idx))
          can_sends.append(gmcan.create_adas_headlights_status(self.packer_obj, CanBus.OBSTACLE))

        speed_and_accelerometer_step = 2
        if self.frame % speed_and_accelerometer_step == 0:
          idx = (self.frame // speed_and_accelerometer_step) % 4
          can_sends.append(gmcan.create_adas_steering_status(CanBus.OBSTACLE, idx))
          can_sends.append(gmcan.create_adas_accelerometer_speed_status(CanBus.OBSTACLE, abs(CS.out.vEgo), idx))

      if self.CP.networkLocation == NetworkLocation.gateway and self.frame % self.params.ADAS_KEEPALIVE_STEP == 0:
        can_sends += gmcan.create_adas_keepalive(CanBus.POWERTRAIN)

    else:
      # While car is braking, cancel button causes ECM to enter a soft disable state with a fault status.
      # A delayed cancellation allows camera to cancel and avoids a fault when user depresses brake quickly
      self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

      # 오토크루즈 '진입시도'중엔 CANCEL 송신금지
      auto_cruise_trying = (CS.out.activateCruise and not CS.out.cruiseState.enabled)

      # Stock longitudinal, integrated at camera
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.04:
        if self.cancel_counter > CAMERA_CANCEL_DELAY_FRAMES and (not auto_cruise_trying):
          self.last_button_frame = self.frame
          self.send_btn(CS, can_sends, CruiseButtons.CANCEL)

    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      # Silence "Take Steering" alert sent by camera, forward PSCMStatus with HandsOffSWlDetectionStatus=1
      if self.frame % 10 == 0:
        can_sends.append(gmcan.create_pscm_status(self.packer_pt, CanBus.CAMERA, CS.pscm_status))

    new_actuators = actuators.as_builder()
    new_actuators.accel = accel
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake
    new_actuators.speed = self.apply_speed # kans: button spam
    new_actuators.steeringAngleDeg = CS.out.steeringAngleDeg  # Kans: torqueLimits

    self.frame += 1
    return new_actuators, can_sends

  # GM: AutoResume
  def brake_input(self, brake_force):
    MAX_BRAKE = 400

    if brake_force > 0.0:
      raise ValueError("brake_force는 0.0이하라야 됨.")

    scaled = int(-brake_force * 100)
    return max(0, min(MAX_BRAKE, scaled))

  def send_btn(self, CS, can_sends, cruise_btn, bus=None):
    if bus is None:
      if self.CP.carFingerprint in CAMERA_ACC_CAR:
        bus = CanBus.CAMERA
      else:
        bus = CanBus.POWERTRAIN

    if bus == CanBus.CAMERA:
      if self.btn_rc_cam < 0:
        self.btn_rc_cam = int(CS.buttons_counter) & 0x3
      self.btn_rc_cam = (self.btn_rc_cam + 1) & 0x3
      rc = self.btn_rc_cam
    elif bus == CanBus.POWERTRAIN:
      if self.btn_rc_pt < 0:
        self.btn_rc_pt = int(CS.buttons_counter) & 0x3
      self.btn_rc_pt = (self.btn_rc_pt + 1) & 0x3
      rc = self.btn_rc_pt
    else:
      raise ValueError(f"Unsupported bus: {bus}")
    can_sends.append(gmcan.create_buttons(self.packer_pt, bus, rc, cruise_btn))

  def brake_strength(self) -> float:
    if self.CP.carFingerprint in EV_CAR or self.CP.carFingerprint in SDGM_CAR:
      return 0.4
    else:
      return 0.7
