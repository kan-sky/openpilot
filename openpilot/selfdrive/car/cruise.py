import math
import numpy as np

from opendbc.car.structs import car
from opendbc.car import structs, DT_CTRL
from openpilot.common.params import Params

GearShifter = structs.CarState.GearShifter
from openpilot.common.constants import CV


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self._activate_cruise_raw = -1
    self._activate_cruise_on_latch = 0
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph


  def _prepare_buttons(self, CS, v_cruise_kph):
    button_kph = v_cruise_kph
    button_type = 0
    buttonEvents = CS.buttonEvents

    SPEED_UP_UNIT = self._cruise_speed_unit_basic
    SPEED_DOWN_UNIT = self._cruise_speed_unit if self._cruise_button_mode in [1, 2, 3] else self._cruise_speed_unit_basic
    V_CRUISE_DELTA = 5
    is_metric = self.is_metric

    # long press tracking
    if self.button_cnt > 0:
      self.button_cnt += 1

    for b in buttonEvents:
      bt = b.type

      if b.pressed and self.button_cnt == 0 and bt in [
        ButtonType.accelCruise, ButtonType.decelCruise,
        ButtonType.gapAdjustCruise, ButtonType.cancel,
        ButtonType.lfaButton
      ]:
        self.button_cnt = 1
        self.button_prev = bt
        self.button_long_time = 40 if bt in [ButtonType.accelCruise, ButtonType.decelCruise] else 70

      elif not b.pressed and self.button_cnt > 0 and bt == self.button_prev:
        if bt == ButtonType.cancel:
          button_type = bt
        elif not self.long_pressed:
          if bt == ButtonType.accelCruise:
            unit = SPEED_UP_UNIT if is_metric else SPEED_UP_UNIT * CV.MPH_TO_KPH
            button_kph = math.ceil((button_kph + 0.01) / unit) * unit
          elif bt == ButtonType.decelCruise:
            unit = SPEED_DOWN_UNIT if is_metric else SPEED_DOWN_UNIT * CV.MPH_TO_KPH
            button_kph = math.floor((button_kph - 0.01) / unit) * unit
          button_type = bt
        self.long_pressed = False
        self.button_cnt = 0

    # Long press 처리
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      bt = self.button_prev

      #if bt == ButtonType.cancel:
      #  button_type = bt
      #  self.button_cnt = 0
      if bt in [ButtonType.accelCruise, ButtonType.decelCruise]:
        mod = button_kph % V_CRUISE_DELTA
        if bt == ButtonType.accelCruise:
          button_kph += V_CRUISE_DELTA - mod
        else:
          button_kph -= V_CRUISE_DELTA - (-mod % V_CRUISE_DELTA)
        button_type = bt
        self.button_cnt %= self.button_long_time
      else: #if bt in [ButtonType.gapAdjustCruise, ButtonType.lfaButton]:
        if self.button_cnt < self.button_long_time + 2:
          button_type = bt
        #self.button_cnt %= self.button_long_time

    return button_kph, button_type, self.long_pressed

  def _carrot_command(self, v_cruise_kph, button_type, long_pressed):
    if self.carrot_cmd_index_last != self.carrot_cmd_index:
      self.carrot_cmd_index_last = self.carrot_cmd_index
      print(f"Carrot command(cruise.py): {self.carrot_cmd} {self.carrot_arg}")
      if self.carrot_cmd == "CRUISE":
        if self.carrot_arg == "OFF":
          self._cruise_control(-2, -1, "Cruise off (carrot command)")
        elif self.carrot_arg == "ON":
          self._cruise_control(1, -1, "Cruise on (carrot command)")
        elif self.carrot_arg == "GO":
          if button_type == 0:
            button_type = ButtonType.accelCruise
            long_pressed = False
            self._add_log("Cruise accelCruise (carrot command)")
        elif self.carrot_arg == "STOP":
          v_cruise_kph = 5
          self._add_log("Cruise stop (carrot command)")

      elif self.carrot_cmd == "SPEED":
        if self.carrot_arg == "UP":
          v_cruise_kph = self._v_cruise_desired(None, v_cruise_kph)
          self._add_log("Cruise speed up (carrot command)")
        elif self.carrot_arg == "DOWN":
          if v_cruise_kph > 20:
            v_cruise_kph -= 10
            self._add_log("Cruise speed downup (carrot command)")
        else:
          speed_kph = int(self.carrot_arg)
          if 0 < speed_kph < 200:
            v_cruise_kph = speed_kph
            self._add_log(f"Cruise speed set to {v_cruise_kph} (carrot command)")       
    
    return v_cruise_kph, button_type, long_pressed

  def _update_cruise_buttons(self, CS, CC, v_cruise_kph):
    button_kph, button_type, long_pressed = self._prepare_buttons(CS, v_cruise_kph)

    v_cruise_kph, button_type, long_pressed = self._carrot_command(v_cruise_kph, button_type, long_pressed)

    if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
      self._paddle_decel_active = False
      if self.autoCruiseControl_cancel_timer > 0:
        self._add_log(f"AutoCruiseControl cancel timer RESET {button_type}")
        self.autoCruiseControl_cancel_timer = 0
      if self._cruise_cancel_state:
        self._add_log(f"Cruise Cancel state RESET {button_type}")
        self._cruise_cancel_state = False

    if not long_pressed:
      if button_type == ButtonType.accelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = False
        if self._soft_hold_active > 0:
          self._soft_hold_active = 0
        elif self._cruise_ready or not CC.enabled or CS.cruiseState.standstill or self.carrot_cruise_active:
          pass
        elif self._v_cruise_kph_at_brake > 0 and v_cruise_kph < self._v_cruise_kph_at_brake:
          v_cruise_kph = self._v_cruise_kph_at_brake
          self._v_cruise_kph_at_brake = 0
        elif self._cruise_button_mode == 0:
          v_cruise_kph = button_kph
        else:
          v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)
        self.carrot_cruise_active = False

      elif button_type == ButtonType.decelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = True
        #self.carrot_cruise_active = False

        if self._soft_hold_active > 0:
          self._cruise_control(-1, -1, "Cruise off,softhold mode (decelCruise)")
        elif self._cruise_ready:
          self._paddle_decel_active = True
          pass
        elif not CC.enabled:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self.v_ego_kph_set > v_cruise_kph + 2 and self._cruise_button_mode in [2, 3]:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self._cruise_button_mode in [0, 1]:
          v_cruise_kph = button_kph
        elif self.v_ego_kph_set < 1.0:
          self.carrot_cruise_active = True
        elif self.v_ego_kph_set > self._cruise_speed_min and v_cruise_kph > self.v_ego_kph_set:
          v_cruise_kph = self.v_ego_kph_set
        else:
          #self._cruise_control(-2, -1, "Cruise off (decelCruise)")
          self.carrot_cruise_active = True
          #self.events.append(EventName.audioPrompt)
        self._v_cruise_kph_at_brake = 0

      elif button_type == ButtonType.gapAdjustCruise:
        longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
        if CS.pcmCruiseGap == 0:
          personality = (self.params.get_int('LongitudinalPersonality') - 1) % longitudinalPersonalityMax
        else:
          personality = np.clip(CS.pcmCruiseGap - 1, 0, longitudinalPersonalityMax)
        self.params.put_int_nonblocking('LongitudinalPersonality', personality)
        #self.events.append(EventName.personalityChanged)
      elif button_type == ButtonType.lfaButton:
        self._lat_enabled = not self._lat_enabled
        self._add_log("Lateral " + ("enabled" if self._lat_enabled else "disabled"))
      elif button_type == ButtonType.cancel:
        self._paddle_decel_active = False
        self._cruise_cancel_state = True
    else:
      if button_type == ButtonType.accelCruise:
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.decelCruise:
        self._pause_auto_speed_up = True
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.gapAdjustCruise:
        self.params.put_int_nonblocking("MyDrivingMode", self.params.get_int("MyDrivingMode") % 4 + 1) # 1,2,3,4 (1:eco, 2:safe, 3:normal, 4:high speed)
      elif button_type == ButtonType.lfaButton:
        useLaneLineSpeed = max(1, self.useLaneLineSpeed)
        self.useLaneLineSpeedApply = useLaneLineSpeed if self.useLaneLineSpeedApply == 0 else 0

      elif button_type == ButtonType.cancel:
        self._cruise_cancel_state = True
        self._lat_enabled = False
        self._paddle_decel_active = False
        #self.params.put_bool_nonblocking("ExperimentalMode", not self.params.get_bool("ExperimentalMode"))
        self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")

    if self._paddle_decel_active and not CC.enabled:
      self._cruise_control(1, -1, "Cruise on (paddle decel)")

    v_cruise_kph = self._update_cruise_state(CS, CC, v_cruise_kph)
    return v_cruise_kph

  ## desiredSpeed :
  #   leadCar_distance, leadCar_speed, leadCar_accel,
  #   v_ego, tbt_distance, tbt_speed,
  #   nRoadLimitSpeed, vTurnSpeed
  #   gasPressed, brakePressed, standstill
  def _v_cruise_desired(self, CS, v_cruise_kph):
    if v_cruise_kph < 15:
      return 15

    unit = max(1, self._cruise_speed_unit)
    return min(self._cruise_speed_max, ((int(v_cruise_kph) // unit) + 1) * unit)

  def _cruise_control(self, enable, cancel_timer, reason):
    if self._cruise_cancel_state: # and self._soft_hold_active != 2:
      self._add_log(reason + " > Cancel state")
    elif enable > 0 and self._cancel_timer > 0 and cancel_timer >= 0:
      enable = 0
      self._add_log(reason + " > Canceled")
    else:
      if self.autoCruiseControl == 0 and enable != 0:
        enable = 0
        self._soft_hold_active = 0
        return
      if self.autoCruiseControl_cancel_timer > 0 and enable != 0:
        self._add_log(reason + " > timer Canceled")
        enable = 0
        self._soft_hold_active = 0
        return
      self._activate_cruise = enable
      # Kans: 크르즈온/오프 래치
      self._activate_cruise_raw = enable  # 복사본
      if enable > 0:  # ON(1)이면, hold_time(0.5s)유지
        self._activate_cruise_on_timer = int(self.activate_cruise_on_hold_time / DT_CTRL)
        self._activate_cruise_on_latch = 1  # 래치 온
      elif enable < 0:  # OFF(-1)면 래치해제
        self._activate_cruise_on_timer = 0
        self._activate_cruise_on_latch = 0

      self._cancel_timer = int(cancel_timer / 0.01)   # DT_CTRL: 0.01
      self._add_log(reason)

  def _check_safe_stop(self, CS, safe_distance = 3):
    v_ego = CS.vEgo
    decel_rate = 1.5
    d_stop_ego = (v_ego ** 2) / (2 * decel_rate)
    d_stop_rel = (self.v_rel ** 2) / (2 * decel_rate)

    d_final = self.d_rel - d_stop_ego - d_stop_rel

    if d_final >= safe_distance:
      return True, d_final
    else:
      return False, d_final

  def _update_cruise_state(self, CS, CC, v_cruise_kph):
    # Kans: activateCruise ON 래치 타이머 관리
    if self._activate_cruise_on_timer > 0:  # 아직 래치 유지중이면
      self._activate_cruise_on_timer -= 1  # -1 씩 내림.
      self._activate_cruise_on_latch = 1  # 안전하게 1로 유지 (혹시 외부에서 건드렸어도)
    else:  # 타이머가 0이하면 래치 해제
      self._activate_cruise_on_latch = 0

    if not CC.enabled:
      #self._pause_auto_speed_up = False
      if self._brake_pressed_count == -1 and self._soft_hold_active > 0:
        self._soft_hold_active = 2
        #self.autoCruiseControl_cancel_timer = 0
        self._cruise_control(1, -1, "Cruise on (soft hold)")
      # GM: autoResume
      #elif self.params.get_bool("ActivateCruiseAfterBrake"):
      #  self.params.put_bool_nonblocking("ActivateCruiseAfterBrake", False)
      #  self._cruise_control(1, -1, "Cruise on (brake)")
      elif self.v_cruise_kph < self.v_ego_kph_set:
        self.v_cruise_kph = self.v_ego_kph_set

    if self._soft_hold_active > 0:
      #self.events.append(EventName.softHold)
      #self._cruise_cancel_state = False
      pass

    if not self.disengage_on_accelerator and self._gas_tok and self.v_ego_kph_set >= self.autoGasTokSpeed:
      if not CC.enabled:
        #self._cruise_cancel_state = False
        self._cruise_control(1, -1, "Cruise on (gas tok)")
        if self.v_ego_kph_set > v_cruise_kph:
          v_cruise_kph = self.v_ego_kph_set
      else:
        v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)
    elif self._gas_pressed_count == -1 and self._brake_pressed_count < -15:
      has_lead = (self.d_rel is not None) and np.isfinite(self.d_rel)
      safe_lead = has_lead and (0.0 < self.d_rel < max(8.0, CS.vEgo * 1.2))
      if safe_lead:
        if CS.vEgo < 1.0:
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (safe speed)")
        else:
          self._cruise_control(-1, 0, "Cruise off (lead car too close)")
      elif not has_lead and self.v_ego_kph_set >= self.autoGasTokSpeed and not CC.enabled:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas pressed, no lead)")
      elif self.v_ego_kph_set < 30:
        self._cruise_control(-1, 0, "Cruise off (gas speed)")
      elif self.xState == 3:
        v_cruise_kph = min(self.v_ego_kph_set, v_cruise_kph)
        self._cruise_control(-1, 3, "Cruise off (traffic sign)")
      elif self.xState == 5:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1, "Cruise on (traffic light green)")
      elif CS.leftBlinker or CS.rightBlinker:
        pass
      elif not self.disengage_on_accelerator and self.v_ego_kph_set >= self.autoGasTokSpeed and not CC.enabled:
        v_cruise_kph = min(self.v_ego_kph_set, v_cruise_kph)
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas pressed)")
    elif (-15 <= self._brake_pressed_count <= -1) and self._soft_hold_active == 0:
      tr_gap = 0.8 # 0.8초앞 정도를 가까운 거리로 설정.
      MAX_CRUISE_DIST = min(30.0, max(15.0, CS.vEgo * tr_gap)) # 크루즈온 최소 거리.
      has_lead = (self.d_rel is not None) and np.isfinite(self.d_rel)
      safe_lead = has_lead and (3.0 <= self.d_rel <= MAX_CRUISE_DIST)
      if CS.leftBlinker or CS.rightBlinker:
        pass
      elif not has_lead and not CC.enabled:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1, "Cruise on (no lead)")
      elif safe_lead:
        if self.v_ego_kph_set > self.autoGasTokSpeed:
          v_cruise_kph = self.v_ego_kph_set
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (speed)")
        elif abs(CS.steeringAngleDeg) < 20:
          if self.xState in [3, 5]:
            if self.xState == 3:  # 감속중
              v_cruise_kph = self.v_ego_kph_set
            self._cruise_control(1, 0, "Cruise on (traffic sign)")
          elif 0 < self.d_rel < 20:
            self._cruise_control(1, -1 if self.v_ego_kph_set < 1 else 0, "Cruise on (lead car)")
      else:
        self._add_log(f"Skip auto cruise: weird lead d={self.d_rel:.1f}m")

    elif self._brake_pressed_count < 0 and self._gas_pressed_count < 0:
      if not CC.enabled:
        if self.d_rel > 0 and CS.vEgo > 0.02:
          safe_state, safe_dist = self._check_safe_stop(CS, 4)
          if abs(CS.steeringAngleDeg) > 70:
            pass
          elif not safe_state:
            self._cruise_control(1, -1, "Cruise on (fcw)")
          else:
            self._add_log(f"leadCar d={self.d_rel:.1f},v={self.v_rel:.1f},{CS.vEgo:.1f}, {safe_dist:.1f}")
            #self.events.append(EventName.stopStop)
        if CS.leftBlinker or CS.rightBlinker:
          pass
        else:
          if self.desiredSpeed < self.v_ego_kph_set:
            self._cruise_control(1, -1, "Cruise on (desired speed)")
          if self._cruise_ready:
            if self.xState in [3]:
              self._cruise_control(1, 0, "Cruise on (traffic sign)")
            elif self.d_rel > 0:
              self._cruise_control(1, 0, "Cruise on (lead car)")
      elif self._paddle_decel_active:
        if self.xState in [3]:
          self._paddle_decel_active = False
          v_cruise_kph = self.v_ego_kph_set
        elif self.d_rel > 0:
          self._paddle_decel_active = False
          v_cruise_kph = self.v_ego_kph_set
          

    if self._gas_pressed_count > self._gas_tok_timer:
      if CS.aEgo < -0.5:
        self._cruise_control(-1, 5.0, "Cruise off (gas pressed while braking)")
    if self._gas_pressed_count == 1 or CS.vEgo < 0.1:
      self._pause_auto_speed_up = False
      if self._gas_pressed_count == 1 and CS.vEgo < 0.1:
        self._cruise_control(-1, -1, "Cruise off (gasPressed)")
    elif self._brake_pressed_count == 1:
      self._pause_auto_speed_up = True

    return v_cruise_kph

  def _prepare_brake_gas(self, CS, CC):
    if CS.gasPressed:
      gas_pressed_start = self._gas_pressed_count <= 0
      self._paddle_decel_active = False
      self._gas_pressed_count = max(1, self._gas_pressed_count + 1)
      self._gas_pressed_count_last = self._gas_pressed_count
      self._gas_pressed_value = max(CS.gas, self._gas_pressed_value) if self._gas_pressed_count > 1 else CS.gas
      self._gas_tok = False
      #if self._cruise_cancel_state and self._soft_hold_active == 2:
      #  self._cruise_control(-1, -1, "Cruise off,softhold mode (gasPressed)")
      self._soft_hold_active = 0
      if gas_pressed_start and self.disengage_on_accelerator:
        self._cruise_ready = False
        self.carrot_cruise_active = False
        self._cruise_control(-1, 0, "Cruise off (gas pressed)")
    else:
      self._gas_tok = True if 0 < self._gas_pressed_count < self._gas_tok_timer else False
      self._gas_pressed_count = min(-1, self._gas_pressed_count - 1)
      if self._gas_pressed_count < -1:
        self._gas_pressed_count_last = 0
        self._gas_tok = False

    if CS.brakePressed:
      self._cruise_ready = False
      self._paddle_decel_active = False
      self._brake_pressed_count = max(1, self._brake_pressed_count + 1)
      if self._brake_pressed_count == 1 and self.enabled_last:
        self._v_cruise_kph_at_brake = self.v_cruise_kph
        self._add_log(f"{self.v_cruise_kph} Cruise speed at brake")
      # 정지 상태에서 일정 시간 이상 브레이크 → soft hold 진입
      self._soft_hold_count = self._soft_hold_count + 1 if CS.vEgo < 0.1 and CS.gearShifter == GearShifter.drive else 0
      if self.autoCruiseControl == 0 or self.CP.pcmCruise:
        self._soft_hold_active = 0
      else:
        self._soft_hold_active = 1 if self._soft_hold_count > 60 else 0
    else:
      self._soft_hold_count = 0
      self._brake_pressed_count = min(-1, self._brake_pressed_count - 1)

  # Kans:
  def get_activate_cruise(self):
    """
    carcontroller.py에서 사용할 activateCruise 값.
    -1: 크루즈 OFF, 1: ON 0: 없었음
    """
    if self._activate_cruise_raw < 0:  # 1) OFF(-1) 최우선. 한번 보내고 초기화(0)
      self._activate_cruise_raw = 0
      return -1
    if self._activate_cruise_on_latch > 0:  # ON 래치가 살아 있으면 1
      return 1
    return 0  # 그외 0
