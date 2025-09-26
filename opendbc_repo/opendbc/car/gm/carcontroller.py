from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, structs, create_gas_interceptor_command
from opendbc.car.gm import gmcan
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.values import DBC, CanBus, CarControllerParams, CruiseButtons, GMFlags, CC_ONLY_CAR, EV_CAR, AccState, CC_REGEN_PADDLE_CAR, CAR, CAMERA_ACC_CAR, SDGM_CAR, SASCM_CAR
from opendbc.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import apply_deadzone
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.car.cruise import VCruiseCarrot

VisualAlert = structs.CarControl.HUDControl.VisualAlert
NetworkLocation = structs.CarParams.NetworkLocation
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15

# constants for pitch compensation
PITCH_DEADZONE = 0.01 # [radians] 0.01 ? 1% grade
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

    # Kans: AutoResume
    self.activateCruise_after_brake = False
    self.v_cruise_carrot = VCruiseCarrot(self.CP)
    self.autoCruise_activate = False
    self.autoCruise_frame = 0
    self.resume_activate = False
    self.resume_frame = 0
    self.acc_engaged_latch = 0
    self.btn_rc_pt = -1
    self.btn_rc_cam = -1
    self._last_brake_idx = None  # 직전 전송 idx 저장용
    self._brk_rc = -1
    self.cruiseDelay_time = 0.0
    self.resumeDelay_time = 0.0
    self._hill_detect_count = 0  # 언덕 감지 카운터
    self._gas_sent = False 

  @staticmethod
  def calc_pedal_command(accel: float, long_active: bool, car_velocity) -> tuple[float, bool]:
    if not long_active: return 0., False
    press_regen_paddle = False

    if accel < -0.3: #-0.15:
      press_regen_paddle = True
      pedal_gas = 0
    else:
      # pedaloffset = 0.24
      pedaloffset = np.interp(car_velocity, [0., 3, 6, 30], [0.08, 0.175, 0.240, 0.240])
      pedal_gas = np.clip((pedaloffset + accel * 0.6), 0.0, 1.0)

      ####for safety.
      pedal_gas_max = np.interp(car_velocity, [0.0, 5, 30], [0.21, 0.3175, 0.3525])
      pedal_gas = np.clip(pedal_gas, 0.0, pedal_gas_max)
      ####for safety. end.

    return pedal_gas, press_regen_paddle

  def update(self, CC, CS, now_nanos):

    if self.frame % 50 == 0:
      params = Params()
      steerMax = params.get_int("CustomSteerMax")
      steerDeltaUp = params.get_int("CustomSteerDeltaUp")
      steerDeltaDown = params.get_int("CustomSteerDeltaDown")
      if steerMax > 0:
        self.params.STEER_MAX = steerMax
      if steerDeltaUp > 0:
        self.params.STEER_DELTA_UP = steerDeltaUp
      if steerDeltaDown > 0:
        self.params.STEER_DELTA_DOWN = steerDeltaDown

    self.long_pitch = Params().get_bool("LongPitch")
    self.use_ev_tables = Params().get_bool("EVTable")

    actuators = CC.actuators
    accel = brake_accel = actuators.accel
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0


    # Send CAN commands.
    can_sends = []

    # Steering (Active: 50Hz, inactive: 10Hz)
    steer_step = self.params.STEER_STEP if CC.latActive else self.params.INACTIVE_STEER_STEP

    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      # Also send at 50Hz:
      # - on startup, first few msgs are blocked
      # - until we're in sync with camera so counters align when relay closes, preventing a fault.
      #   openpilot can subtly drift, so this is activated throughout a drive to stay synced
      out_of_sync = self.lka_steering_cmd_counter % 4 != (CS.cam_lka_steering_cmd_counter + 1) % 4
      if CS.loopback_lka_steering_cmd_ts_nanos == 0 or out_of_sync:
        steer_step = self.params.STEER_STEP

    self.lka_steering_cmd_counter += 1 if CS.loopback_lka_steering_cmd_updated else 0

    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we
    # received the ASCMLKASteeringCmd loopback confirmation too recently
    last_lka_steer_msg_ms = (now_nanos - CS.loopback_lka_steering_cmd_ts_nanos) * 1e-6
    if (self.frame - self.last_steer_frame) >= steer_step and last_lka_steer_msg_ms > MIN_STEER_MSG_INTERVAL_MS:
      # Initialize ASCMLKASteeringCmd counter using the camera until we get a msg on the bus
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
      # Gas/regen, brakes, and UI commands - all at 25Hz
      if self.frame % 4 == 0:
        friction_sent_this_tick = False
        self.cruiseDelay_time = Params().get_float("CruiseDelay") * 0.01
        self.resumeDelay_time = Params().get_float("ResumeDelay") * 0.01
        auto_cruise_enabled = Params().get_int("AutoCruiseControl") > 0
        auto_engage_enabled = Params().get_int("AutoEngage") == 2
        # GM: softHold
        stopping = actuators.longControlState == LongCtrlState.stopping or CS.out.softHoldActive > 0

        # Pitch compensated acceleration;
        # TODO: include future pitch (sm['modelDataV2'].orientation.y) to account for long actuator delay
        if self.long_pitch and len(CC.orientationNED) > 1:
          self.pitch.update(CC.orientationNED[1])
          self.accel_g = ACCELERATION_DUE_TO_GRAVITY * apply_deadzone(self.pitch.x, PITCH_DEADZONE) # driving uphill is positive pitch
          accel += self.accel_g
          brake_accel = actuators.accel + self.accel_g * np.interp(CS.out.vEgo, BRAKE_PITCH_FACTOR_BP, BRAKE_PITCH_FACTOR_V)

        at_full_stop = CC.longActive and CS.out.standstill
        near_stop = CC.longActive and (abs(CS.out.vEgo) < self.params.NEAR_STOP_BRAKE_PHASE)
        interceptor_gas_cmd = 0
        press_regen_paddle = False
        if not CC.longActive:
          # ASCM sends max regen when not enabled
          self.apply_gas = self.params.INACTIVE_REGEN
          self.apply_brake = 0
        elif near_stop and stopping and not CC.cruiseControl.resume:
          self.apply_gas = self.params.INACTIVE_REGEN
          self.apply_brake = int(min(-100 * self.CP.stopAccel, self.params.MAX_BRAKE))
          press_regen_paddle = False
        else:
          # Normal operation
          if self.CP.carFingerprint in EV_CAR:
            self.params.update_ev_gas_brake_threshold(CS.out.vEgo)
            self.apply_gas = int(round(np.interp(accel if self.long_pitch else actuators.accel, self.params.EV_GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
            self.apply_brake = int(round(np.interp(brake_accel if self.long_pitch else actuators.accel, self.params.EV_BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))
          else:
            self.apply_gas = int(round(np.interp(accel if self.long_pitch else actuators.accel, self.params.GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
            self.apply_brake = int(round(np.interp(brake_accel if self.long_pitch else actuators.accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))
          # Don't allow any gas above inactive regen while stopping
          # FIXME: brakes aren't applied immediately when enabling at a stop
          if stopping:
            self.apply_gas = self.params.INACTIVE_REGEN
          if self.CP.carFingerprint in CC_ONLY_CAR:
            # gas interceptor only used for full long control on cars without ACC
            interceptor_gas_cmd, press_regen_paddle = self.calc_pedal_command(actuators.accel, CC.longActive, CS.out.vEgo)

        if self.CP.enableGasInterceptorDEPRECATED and self.apply_gas > self.params.INACTIVE_REGEN and CS.out.cruiseState.standstill:
          # "Tap" the accelerator pedal to re-engage ACC
          interceptor_gas_cmd = self.params.SNG_INTERCEPTOR_GAS
          self.apply_brake = 0
          press_regen_paddle = False
          self.apply_gas = self.params.INACTIVE_REGEN

        idx = (self.frame // 4) % 4

        if self.CP.flags & GMFlags.CC_LONG.value:
          if CC.longActive and CS.out.vEgo > self.CP.minEnableSpeed:
            # Using extend instead of append since the message is only sent intermittently
            can_sends.extend(gmcan.create_gm_cc_spam_command(self.packer_pt, self, CS, actuators))
        if self.CP.enableGasInterceptorDEPRECATED:
          can_sends.append(create_gas_interceptor_command(self.packer_pt, interceptor_gas_cmd, idx))
          if self.CP.carFingerprint in CC_REGEN_PADDLE_CAR and press_regen_paddle:
            can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))

        at_full_stop = CC.longActive and CS.out.standstill
        near_stop = CC.longActive and (abs(CS.out.vEgo) < self.params.NEAR_STOP_BRAKE_PHASE)
        friction_brake_bus = CanBus.CHASSIS
        # GM Camera exceptions
        # TODO: can we always check the longControlState?
        if self.CP.networkLocation == NetworkLocation.fwdCamera and self.CP.carFingerprint not in CC_ONLY_CAR:
          at_full_stop = at_full_stop and stopping
          friction_brake_bus = CanBus.POWERTRAIN
          if self.CP.carFingerprint in SDGM_CAR:
            friction_brake_bus = CanBus.CAMERA

        if CC.cruiseControl.resume and CS.pcm_acc_status == AccState.STANDSTILL:
          self.acc_engaged_latch = self.frame + int(0.2 / DT_CTRL)
        if self.CP.carFingerprint == CAR.CHEVROLET_VOLT:
          if CC.cruiseControl.resume and CS.pcm_acc_status == AccState.STANDSTILL:
            acc_engaged = False
          else:
            acc_engaged = CC.enabled
        else:
          acc_engaged = CC.enabled or (self.frame < self.acc_engaged_latch)

        if auto_cruise_enabled:
          # Kans: autoCruise
          if CS.out.activateCruise and not CS.out.cruiseState.enabled:
            if self.autoCruise_frame == 0:
              self.autoCruise_frame = self.frame
              self.autoCruise_activate = False
            self.activateCruise_after_brake = False # 오토크루즈가 되기 위해 브레이크 신호는 OFF여야 함.
            if actuators.longControlState != LongCtrlState.starting:
              if not self.autoCruise_activate:
                if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
                  self.last_button_frame = self.frame
                  self.send_btn(CS, can_sends, CruiseButtons.DECEL_SET)
                  self.autoCruise_activate = True  # 전송 직후 잠금
                  self.autoCruise_frame = self.frame  # 쿨다운 기준점
              if self.autoCruise_activate:
                if (self.frame - self.autoCruise_frame) * DT_CTRL >= self.cruiseDelay_time:
                  self.autoCruise_activate = False
          else:
            self.autoCruise_frame = 0
            self.autoCruise_activate = False

        # Kans: AutoResume 1st step
        if actuators.longControlState == LongCtrlState.starting:
          if CS.out.cruiseState.enabled and not self.activateCruise_after_brake: #브레이크신호 한번만 보내기 위한 조건.
            self._brk_rc = (self._brk_rc + 1) & 0x3
            brk_idx = self._brk_rc
            apply_brake = self.brake_input(-self.brake_strength())
            # 브레이크신호 전송(롱컨 꺼짐)
            if self.CP.carFingerprint == CAR.CHEVROLET_VOLT:  
              can_sends.append(gmcan.create_brake_command(self.packer_ch, CanBus.CHASSIS, apply_brake, brk_idx))
            elif self.CP.carFingerprint in SDGM_CAR:
              can_sends.append(gmcan.create_brake_command(self.packer_ch, CanBus.CHASSIS, apply_brake, brk_idx))
            elif self.CP.carFingerprint in CAMERA_ACC_CAR:
              can_sends.append(gmcan.create_brake_command(self.packer_pt, CanBus.POWERTRAIN, apply_brake, brk_idx))
            Params().put_bool_nonblocking("ActivateCruiseAfterBrake", True) # cruise.py에 브레이크 ON신호 전달
            self.activateCruise_after_brake = True # 브레이크신호는 한번만 보내고 초기화
            friction_sent_this_tick = True

        # Kans: AutoResume 2nd step
        if auto_engage_enabled:
          if actuators.longControlState == LongCtrlState.starting:
            if self.resume_frame == 0:
              self.resume_frame = self.frame
              self.resume_activate = False
            if not self.resume_activate:
              if (self.frame - self.last_button_frame) * DT_CTRL >= 0.08:
                self.last_button_frame = self.frame
                self.send_btn(CS, can_sends, CruiseButtons.RES_ACCEL)
              if self.frame % 2 == 1 and (self.frame - self.resume_frame) * DT_CTRL >= self.resumeDelay_time:
                self.resume_activate = True
          else:
            self.resume_frame = 0
            self.resume_activate = False

        if actuators.longControlState == LongCtrlState.starting:
          if CS.out.aEgo < -0.010:  # 밀림감지 임계값
            self._hill_detect_count += 1
          else:
            self._hill_detect_count = 0
          gas_force = 1000.0 if self._hill_detect_count >= 3 else 500.0
          if not self._gas_sent:  # 1회 전송 제한
            apply_gas = self.gas_input(gas_force)
            at_full_stop = CS.out.standstill
            can_sends.append(gmcan.create_gas_command(self.packer_pt, CanBus.POWERTRAIN, apply_gas, idx, at_full_stop))
            self._gas_sent = True
        else:
          self._gas_sent = False

        # GasRegenCmdActive needs to be 1 to avoid cruise faults. It describes the ACC state, not actuation
        can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_gas, idx, acc_engaged, at_full_stop))
        if not friction_sent_this_tick:
          self._brk_rc = (self._brk_rc + 1) & 0x3
          brk_idx_base = self._brk_rc
          if self.CP.carFingerprint == CAR.CHEVROLET_VOLT:
            can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, self.apply_brake,
                             brk_idx_base, CC.enabled, near_stop, at_full_stop, self.CP))
          elif self.CP.carFingerprint in SDGM_CAR:
            can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, self.apply_brake,
                             brk_idx_base, CC.enabled, near_stop, at_full_stop, self.CP))
          elif self.CP.carFingerprint in CAMERA_ACC_CAR:
            can_sends.append(gmcan.create_friction_brake_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_brake,
                             brk_idx_base, CC.enabled, near_stop, at_full_stop, self.CP))

        # Send dashboard UI commands (ACC status)
        send_fcw = hud_alert == VisualAlert.fcw
        can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, CC.enabled,
                                                            hud_v_cruise * CV.MS_TO_KPH, hud_control, send_fcw))
      else:
        # to keep accel steady for logs when not sending gas
        accel += self.accel_g

      # Radar needs to know current speed and yaw rate (50hz),
      # and that ADAS is alive (10hz)
      if not self.CP.radarUnavailable and self.CP.networkLocation != NetworkLocation.fwdCamera and self.CP.carFingerprint not in SDGM_CAR:
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

      # TODO: integrate this with the code block below?
      if (
          (self.CP.flags & GMFlags.PEDAL_LONG.value)  # Always cancel stock CC when using pedal interceptor
          or (self.CP.flags & GMFlags.CC_LONG.value and not CC.enabled)  # Cancel stock CC if OP is not active
      ) and CS.out.cruiseState.enabled:
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.04:
          self.last_button_frame = self.frame
          can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.POWERTRAIN, (CS.buttons_counter + 1) % 4, CruiseButtons.CANCEL))

    else:
      # While car is braking, cancel button causes ECM to enter a soft disable state with a fault status.
      # A delayed cancellation allows camera to cancel and avoids a fault when user depresses brake quickly
      self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

      # Stock longitudinal, integrated at camera
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.04:
        if self.cancel_counter > CAMERA_CANCEL_DELAY_FRAMES:
          self.last_button_frame = self.frame
          can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.CAMERA, (CS.buttons_counter + 1) % 4, CruiseButtons.CANCEL))

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

    self.frame += 1
    return new_actuators, can_sends

  # GM: AutoResume
  def brake_input(self, brake_force):
    MAX_BRAKE = 400
    ZERO_GAS = 0.0

    if brake_force > 0.0:
      raise ValueError("brake_force는 0.0이하라야 됨.")

    scaled_brake = max(0, min(MAX_BRAKE, int(brake_force * -100)))  # -를 +로 변환
    return -scaled_brake

  def gas_input(self, gas_force: float) -> int:
    SCALE = 0.125
    OFFSET = -22534
    ZERO_GAS = 0.0
    neutral_raw = int((ZERO_GAS - OFFSET) / SCALE)  # 중립값=약180272

    gas_force = max(ZERO_GAS, gas_force)
    raw_value = neutral_raw + int(gas_force)
    return raw_value

  def send_btn(self, CS, can_sends, cruise_btn, bus=None):
    if bus is None:
      bus = CanBus.CAMERA if self.CP.carFingerprint in CAMERA_ACC_CAR else CanBus.POWERTRAIN
    # RollinfCounter초기화
    if bus == CanBus.CAMERA:
      if self.btn_rc_cam < 0:
        self.btn_rc_cam = int(CS.buttons_counter) & 0x3
    elif bus == CanBus.POWERTRAIN:
      if self.btn_rc_pt < 0:
        self.btn_rc_pt = int(CS.buttons_counter) & 0x3
    # RollinfCounter +1증가
    if bus == CanBus.CAMERA:
      self.btn_rc_cam = (self.btn_rc_cam + 1) & 0x3
      rc = self.btn_rc_cam
    elif bus == CanBus.POWERTRAIN:
      self.btn_rc_pt = (self.btn_rc_pt + 1) & 0x3
      rc = self.btn_rc_pt
    else:
      raise ValueError(f"Unsupported bus: {bus}")

    can_sends.append(gmcan.create_buttons(self.packer_pt, bus, rc, cruise_btn))

  def brake_strength(self) -> float:
    if self.CP.carFingerprint in EV_CAR:
      return 0.4
    else:
      return 0.5
