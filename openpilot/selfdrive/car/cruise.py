import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

from opendbc.car import structs, DT_CTRL
GearShifter = structs.CarState.GearShifter

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 20
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


from openpilot.common.params import Params
#from openpilot.selfdrive.selfdrived.events import Events
#EventName = log.OnroadEvent.EventName

class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.frame = 0
    self.params_memory = Params("/dev/shm/params")
    self.params = Params()

    self.v_cruise_kph = 20
    self.v_cruise_cluster_kph = 20
    self.v_cruise_kph_last = 20

    # latest comma button timer state
    self.button_timers = {
      ButtonType.decelCruise: 0,
      ButtonType.accelCruise: 0,
    }
    self.button_change_states = {
      btn: {"standstill": False, "enabled": False}
      for btn in self.button_timers
    }

    self.enabled_last = False
    self.is_metric = True

    self.long_pressed = False
    self.button_cnt = 0
    self.button_prev = ButtonType.unknown
    self.button_long_time = 40

    self.v_ego_kph_set = 0
    self._cruise_speed_min, self._cruise_speed_max = 5, 161
    self._cruise_speed_unit = 5
    self._cruise_speed_unit_basic = 5
    self._cruise_button_mode = 3
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")

    self._gas_pressed_count = 0
    self._gas_pressed_count_last = 0
    self._gas_pressed_value = 0
    self._gas_tok_timer = int(0.4 / DT_CTRL)
    self._gas_tok = False

    self._brake_pressed_count = 0
    self._cruise_ready = False
    self._cruise_cancel_state = False
    self._pause_auto_speed_up = False
    self._activate_cruise = 1
    self._lat_enabled = self.params.get_int("AutoEngage") > 0
    self._v_cruise_kph_at_brake = 0
    self.cruise_state_available_last = False
    self._cruise_speed_initialized = False

    self.d_rel = 0
    self.v_rel = 0
    self.cruiseOnDist = 7.0

    self._cancel_timer = 0
    self._log_timer = 0
    self._log_timeout = int(3 / DT_CTRL)
    self.log = ""

    self.autoCruiseControl_cancel_timer = 0
    self._gear_ok = True
    self.autoCruiseControl = 0
    self.autoGasTokSpeed = 0
    self.autoGasSyncSpeed = 0
    # Carrot traffic-light state
    self.xState = 0
    self.xState_last = 0
    self.trafficState = 0
    self.trafficState_last = 0
    self.aTarget = 0.0

    # activateCruise ON latch
    self._activate_cruise_raw = 0
    self._activate_cruise_on_latch = 0
    self._activate_cruise_on_timer = 0
    self.activate_cruise_on_hold_time = 0.5


  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def _current_speed_for_initial_resume(self):
    return max(self.v_ego_kph_set, self._cruise_speed_min)

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
        #self.event = -1
    else:
      self.log = log
      #self.event = event
      self._log_timer = self._log_timeout
      # Kans: also persist to /data/log/swaglog so this is readable after a
      # drive with `grep "\[cruise\]" swaglog*` over SSH, instead of having to
      # screen-record the live logCarrot overlay to capture it.
      cloudlog.warning(f"[cruise] {log}")

  def update_params(self, is_metric):
    unit_factor = 1.0 if is_metric else CV.MPH_TO_KPH
    if self.frame % 10 == 0:
      self.autoCruiseControl = self.params.get_int("AutoCruiseControl")
      self.autoGasTokSpeed = self.params.get_int("AutoGasTokSpeed") * unit_factor
      self.autoGasSyncSpeed = self.params.get_int("AutoGasSyncSpeed")

      cruise_speed_unit = self.params.get_int("CruiseSpeedUnit")
      cruise_speed_unit_basic = self.params.get_int("CruiseSpeedUnitBasic")
      self._cruise_speed_unit = cruise_speed_unit if cruise_speed_unit > 0 else 5
      self._cruise_speed_unit_basic = cruise_speed_unit_basic if cruise_speed_unit_basic > 0 else 5

      self._cruise_button_mode = self.params.get_int("CruiseButtonMode")
      self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
      self.cruiseOnDist = self.params.get_float("CruiseOnDist") * 0.01
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

    v_cruise_delta = v_cruise_delta * (5 if long_press else 5) # Kans
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

  def update_v_cruise(self, CS, sm, is_metric):
    self._add_log("")
    self.update_params(is_metric)
    self.frame += 1
    CC = sm['carControl']
    # Kans: receive traffic-light state from longitudinal planner.
    self.xState_last = self.xState
    self.trafficState_last = self.trafficState
    if sm.alive['longitudinalPlan']:
      lp = sm['longitudinalPlan']
      self.xState = lp.xState
      self.trafficState = lp.trafficState
      self.aTarget = lp.aTarget
    if sm.alive['radarState']:
      lead = sm['radarState'].leadOne
      self.d_rel = lead.dRel if lead.present else 0
      self.v_rel = lead.vRel if lead.present else 0

    # Kans: GM's ECMPRDNL2.ManualMode bit is set whenever the Volt's regen
    # paddle is pulled (routine while driving, e.g. slowing for a lead car -
    # exactly when auto-engage is needed most), which opendbc maps to
    # GearShifter.manumatic, not drive - the physical shifter never actually
    # leaves D. Confirmed via swaglog capture showing every engage attempt
    # blocked by the 20s autoCruiseControl_cancel_timer below during normal
    # driving. Treat manumatic the same as drive here.
    self._gear_ok = CS.gearShifter in (GearShifter.drive, GearShifter.manumatic)
    if not self._gear_ok:
      self.autoCruiseControl_cancel_timer = int(20 / DT_CTRL)
    else:
      self.autoCruiseControl_cancel_timer = max(0, self.autoCruiseControl_cancel_timer - 1)

    self.v_cruise_kph_last = self.v_cruise_kph
    self.is_metric = is_metric
    self._cancel_timer = max(0, self._cancel_timer - 1)

    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    self._activate_cruise = 0

    self._prepare_brake_gas(CS, CC)

    if CC.enabled:
      self._cruise_ready = False

    v_cruise_kph = self._update_cruise_buttons(CS, CC, self.v_cruise_kph)

    if self._activate_cruise > 0:
      self._cruise_ready = False
    elif self._activate_cruise < 0:
      self._cruise_ready = self._activate_cruise == -2

    if CS.cruiseState.available:
      if not self.cruise_state_available_last:
        self._lat_enabled = True
        v_cruise_kph = self.v_ego_kph_set

      if not self.CP.pcmCruise:
        self.v_cruise_kph = np.clip(v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        # latest comma PCM ownership
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH

        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = np.clip(v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
      self.v_cruise_cluster_kph = self.v_cruise_kph

    self.cruise_state_available_last = CS.cruiseState.available
    self.enabled_last = CC.enabled

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # Initial set/resume speed is handled in update_v_cruise/_update_cruise_buttons.

    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else CS.vEgoCluster * CV.MS_TO_KPH

    v_ego_kph = int(round(np.clip(CS.vEgoCluster * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))
    print(CS.buttonEvents)
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents): # and self.v_cruise_initialized:
      self.v_cruise_kph = max(self._v_cruise_kph_at_brake, v_ego_kph) if self._v_cruise_kph_at_brake > 0 else self.v_cruise_kph_last
      self._add_log(f"{self.v_cruise_kph},{self._v_cruise_kph_at_brake} Cruise resume")
    else:
      self.v_cruise_kph = v_ego_kph
      self._add_log(f"{self.v_cruise_kph} Cruise Set")

    self.v_cruise_kph = np.clip(self.v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
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


  def _update_cruise_buttons(self, CS, CC, v_cruise_kph):
    button_kph, button_type, long_pressed = self._prepare_buttons(CS, v_cruise_kph)

    if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
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

        # Kans: without this gate, every accelCruise press (including the
        # spoofed RES_ACCEL/DECEL_SET carcontroller.py sends to auto-engage
        # cruise from off) fell through to _v_cruise_desired(), which assumes
        # cruise is already active and just steps the set speed up by one
        # unit from whatever v_cruise_kph last held - not necessarily the
        # current driving speed. Restored from carrot-wip: while not yet
        # engaged (or at a standstill), the first resume of the drive snaps
        # to the actual current speed instead.
        if self._cruise_ready or not CC.enabled or CS.cruiseState.standstill:
          if self._v_cruise_kph_at_brake > 0:
            v_cruise_kph = max(v_cruise_kph, self._v_cruise_kph_at_brake)
            self._v_cruise_kph_at_brake = 0
            self._cruise_speed_initialized = True
          elif not self._cruise_speed_initialized:
            v_cruise_kph = self._current_speed_for_initial_resume()
            self._cruise_speed_initialized = True
            self._add_log(f"{v_cruise_kph} Cruise resume from current speed")
        else:
          self._v_cruise_kph_at_brake = 0
          if self._cruise_button_mode == 0:
            v_cruise_kph = button_kph
          else:
            v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)
        self._cruise_speed_initialized = True

      elif button_type == ButtonType.decelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = True

        if not CC.enabled:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self.v_ego_kph_set > v_cruise_kph + 2 and self._cruise_button_mode in [2, 3]:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self.v_ego_kph_set > self._cruise_speed_min and v_cruise_kph > self.v_ego_kph_set:
          # Currently going slower than the set speed (e.g. a curve-speed
          # slowdown) - capture the current speed instead of just stepping
          # the set speed down by one unit, per devel-0815.
          v_cruise_kph = self.v_ego_kph_set
        else:
          # Kans: modes 2/3 had no fallback here, so a short decelCruise press
          # while cruising steadily at/near the set speed (the common case -
          # neither the softhold, disengaged, nor "going faster than set
          # speed" branches above apply) did nothing at all. button_kph is
          # already computed with the correct per-mode unit (SPEED_DOWN_UNIT
          # in _prepare_buttons), same as accelCruise's button_kph for mode 0.
          v_cruise_kph = button_kph

        self._v_cruise_kph_at_brake = 0

      elif button_type == ButtonType.gapAdjustCruise:
        longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
        if longitudinalPersonalityMax > 0:
          if CS.pcmCruiseGap == 0:
            personality = (self.params.get_int("LongitudinalPersonality") - 1) % longitudinalPersonalityMax
          else:
            personality = int(np.clip(CS.pcmCruiseGap - 1, 0, longitudinalPersonalityMax - 1))
          self.params.put_int_nonblocking("LongitudinalPersonality", personality)

      elif button_type == ButtonType.cancel:
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
      elif button_type == ButtonType.cancel:
        self._cruise_cancel_state = True
        self._lat_enabled = False
        #self._add_log("Lateral disabled")
        self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")

    return self._update_cruise_state(CS, CC, v_cruise_kph)

  ## desiredSpeed :
  #   leadCar_distance, leadCar_speed, leadCar_accel,
  #   v_ego, tbt_distance, tbt_speed,
  #   nRoadLimitSpeed, vTurnSpeed
  #   gasPressed, brakePressed, standstill
  def _v_cruise_desired(self, CS, v_cruise_kph):
    if v_cruise_kph < 15:
      return 15

    unit = self._cruise_speed_unit
    if not self.is_metric:
      unit *= CV.MPH_TO_KPH

    return min(self._cruise_speed_max, math.ceil((v_cruise_kph + 0.01) / unit) * unit)


  def _cruise_control(self, enable, cancel_timer, reason):
    if self._cruise_cancel_state:
      self._add_log(reason + " > Cancel state")
      return

    if enable > 0 and self._cancel_timer > 0 and cancel_timer >= 0:
      self._add_log(reason + " > Canceled")
      return

    if self.autoCruiseControl == 0 and enable != 0:
      return

    if self.autoCruiseControl_cancel_timer > 0 and enable != 0:
      self._add_log(reason + " > timer Canceled")
      return

    self._activate_cruise = enable
    self._activate_cruise_raw = enable

    if enable > 0:
      self._activate_cruise_on_timer = int(self.activate_cruise_on_hold_time / DT_CTRL)
      self._activate_cruise_on_latch = 1
    elif enable < 0:
      self._activate_cruise_on_timer = 0
      self._activate_cruise_on_latch = 0

    self._cancel_timer = int(cancel_timer / DT_CTRL) if cancel_timer > 0 else 0
    self._add_log(reason)


  def _check_safe_stop(self, CS, safe_distance=3):
    v_ego = CS.vEgo
    decel_rate = 1.5
    d_stop_ego = (v_ego ** 2) / (2 * decel_rate)
    d_stop_rel = (self.v_rel ** 2) / (2 * decel_rate)

    d_final = self.d_rel - d_stop_ego - d_stop_rel

    if d_final >= safe_distance:
      return True, d_final
    return False, d_final

  def _update_cruise_state(self, CS, CC, v_cruise_kph):
    # activateCruise ON latch timer
    if self._activate_cruise_on_timer > 0:
      self._activate_cruise_on_timer -= 1
      self._activate_cruise_on_latch = 1
    else:
      self._activate_cruise_on_latch = 0

    # Kans: traffic-light stop released.
    # e2eStop(3) / e2eStopped(5) -> e2eCruise(2) means Carrot released the stop target.
    traffic_start = self.xState_last in [3, 5] and self.xState == 2
    if traffic_start and not CC.enabled and not CS.brakePressed and self._gear_ok:
      self._cruise_control(1, -1, "Cruise on (traffic green)")

    # Kans: this whole if/elif chain (gas-tok -> exact-release-edge triggers ->
    # persistent-released/CruiseOnDist) was missing in tz - restored per the
    # user's own road-tested `devel-0815` branch (not carrot-wip's original,
    # which is rougher - devel-0815 tightens the release-edge windows with
    # has_lead/safe_lead distance checks and adds an explicit "no lead" case
    # and a traffic-light-green branch that carrot-wip doesn't have).
    #
    # Short gas-tok:
    # - cruise OFF: request AutoCruise and set current speed
    # - cruise ON : raise set speed to next configured unit
    if (not self.disengage_on_accelerator and self._gas_tok and
        self.v_ego_kph_set >= self.autoGasTokSpeed):
      if not CC.enabled:
        self._cruise_control(1, -1, "Cruise on (gas tok)")
        v_cruise_kph = max(v_cruise_kph, self.v_ego_kph_set)
      else:
        v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)

    # Gas pedal just released, and brake has been off for >0.15s already
    # (rules out "tapping brake then gas" sequences from matching this).
    elif self._gas_pressed_count == -1 and self._brake_pressed_count < -15:
      has_lead = self.d_rel > 0
      safe_lead = has_lead and (0.0 < self.d_rel < max(8.0, CS.vEgo * 1.2))
      if safe_lead:
        if CS.vEgo < 1.0:
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (safe speed)")
        else:
          self._cruise_control(-1, 0, "Cruise off (lead car too close)")
      elif not has_lead and self.v_ego_kph_set >= self.autoGasTokSpeed and not CC.enabled:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas pressed, no lead)")
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

    # Brake pedal released within the last 0.15s (a window, not just the exact
    # release frame). devel-0815 also required self._soft_hold_active == 0 -
    # soft hold was removed entirely from this fork, so that clause is dropped.
    elif -15 <= self._brake_pressed_count <= -1:
      tr_gap = 0.8  # ~0.8s ahead counts as "close"
      max_cruise_dist = min(30.0, max(15.0, CS.vEgo * tr_gap))
      has_lead = self.d_rel > 0
      safe_lead = has_lead and (3.0 <= self.d_rel <= max_cruise_dist)
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

    # Pedals released for a while now (not just this frame): FCW / CruiseOnDist.
    elif self._brake_pressed_count < 0 and self._gas_pressed_count < 0:
      if not CC.enabled:
        if self.d_rel > 0 and CS.vEgo > 0.02:
          safe_state, safe_dist = self._check_safe_stop(CS, 4)
          if abs(CS.steeringAngleDeg) > 70:
            pass
          elif not safe_state:
            self._cruise_control(1, -1, "Cruise on (fcw)")
          elif self.d_rel < self.cruiseOnDist:
            self._cruise_control(1, 0, "Cruise on (fcw dist)")
          else:
            self._add_log(f"leadCar d={self.d_rel:.1f},v={self.v_rel:.1f},{CS.vEgo:.1f}, {safe_dist:.1f}")
        if not (CS.leftBlinker or CS.rightBlinker) and self._cruise_ready:
          if self.xState == 3:
            self._cruise_control(1, 0, "Cruise on (traffic sign)")
          elif self.d_rel > 0:
            self._cruise_control(1, 0, "Cruise on (lead car)")

    # Gas held past the tok threshold (a real hold, not a quick tap): if the
    # driver has accelerated past the set cruise speed, sync v_cruise up to
    # it so releasing the pedal doesn't suddenly brake back down.
    if (self._gas_pressed_count > self._gas_tok_timer and self.autoGasSyncSpeed and
        self.v_ego_kph_set > v_cruise_kph):
      v_cruise_kph = self.v_ego_kph_set

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
      self._gas_pressed_count = max(1, self._gas_pressed_count + 1)
      self._gas_pressed_count_last = self._gas_pressed_count
      self._gas_pressed_value = max(CS.gas, self._gas_pressed_value) if self._gas_pressed_count > 1 else CS.gas
      self._gas_tok = False

      if gas_pressed_start and self.disengage_on_accelerator and CC.enabled:
        self._cruise_ready = False
        self._cruise_control(-1, 0, "Cruise off (gas pressed)")
    else:
      self._gas_tok = True if 0 < self._gas_pressed_count < self._gas_tok_timer else False
      self._gas_pressed_count = min(-1, self._gas_pressed_count - 1)
      if self._gas_pressed_count < -1:
        self._gas_pressed_count_last = 0
        self._gas_pressed_value = 0
        self._gas_tok = False

    if CS.brakePressed:
      self._cruise_ready = False
      self._brake_pressed_count = max(1, self._brake_pressed_count + 1)

      if self._brake_pressed_count == 1 and self.enabled_last:
        self._v_cruise_kph_at_brake = self.v_cruise_kph
        self._add_log(f"{self.v_cruise_kph} Cruise speed at brake")
    else:
      self._brake_pressed_count = min(-1, self._brake_pressed_count - 1)

  # Kans:
  def get_activate_cruise(self):
    if self._activate_cruise_raw < 0:  # 1) OFF(-1) 최우선. 한번 보내고 초기화(0)
      self._activate_cruise_raw = 0
      return -1
    if self._activate_cruise_on_latch > 0:  # ON 래치가 살아 있으면 1
      return 1
    return 0  # 그외 0
