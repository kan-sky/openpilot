import math

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, ET
import cereal.messaging as messaging
from openpilot.common.params import Params
import collections

EventName = car.OnroadEvent.EventName

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 30
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
    self.v_cruise_kph = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # carrot
    self.brake_pressed_count = 0
    self.brake_pressed_frame = 0
    self.gas_pressed_count = 0
    self.gas_pressed_count_prev = 0
    self.gas_pressed_max_aego = 0.0
    self.gas_pressed_value = 0
    self.softHoldActive = 0
    self.button_cnt = 0
    self.long_pressed = False
    self.button_prev = ButtonType.unknown
    self.cruiseActivate = 0
    self.params = Params()
    self.v_cruise_kph_set = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.cruiseSpeedTarget = 0
    self.xState = 0
    self.trafficState = 0
    self.sendEvent_frame = 0
    self.softHold_count = 0
    self.cruiseActiveReady = 0
    self.autoCruiseCancelState = 0  # 0: normal, 1:cancel, 2: timer cancel
    self.frame = 0
    self._log_timer = 0
    self.cruiseSpeedMax = V_CRUISE_MAX
    self.autoCruiseCancelTimer = 0
    self.sendEvent = None
    self.gas_tok_frame = 0
    self.button_long_time = 40
    self.accel_output = 0.0
    self.traffic_light_q = collections.deque(maxlen=int(2.0/DT_CTRL))
    self.traffic_light_count = -1
    self.traffic_state = 0
    self.v_ego_kph_set = 0

    # kans
    self.events = Events()

    # carrot params
    self.params_count = 0
    self.autoResumeFromGasSpeed = 0 # 자동인게이지 속도를 임의로 정함(0km/h)
    self.autoCancelFromGasMode = 0 # 가스페달에도 크루즈유지 모드로 임의설정
    self.autoResumeFromBrakeReleaseTrafficSign = 0 # 언브레이크시 오토크루즈 OFF
    self.autoCruiseControl = 0 # 벌트는 사용하지 않음
    self.cruiseButtonMode = 0
    self.autoSpeedUptoRoadSpeedLimit = 1 # 기본값 100%, 즉, 1곱하기.
    self.cruiseOnDist = 5 # 크루주온 거리 5미터로 임의설정
    self.softHoldMode = True # 소프트홀드 사용함으로 임의설정
    self.cruiseSpeedMin = 0 # 크루즈 최소속도 임의설정

    self.speedFromPCM = 0 # 벌트는 사용하지 않음.
    self.cruiseEcoControl = 0.2 # 크루즈연비 .2km/h로 임의설정
    self.cruiseSpeedUnit = 5
  def _params_update(self, car_controls):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      pass
    elif self.params_count == 20:
      self.autoResumeFromGasSpeed = 0 # 자동인게이지 속도를 임의로 정함(0km/h)
      self.autoCancelFromGasMode = 0 # 가스페달에도 크루즈유지 모드로 임의설정
      self.autoResumeFromBrakeReleaseTrafficSign = 0 # 언브레이크시 오토크루즈 OFF
      self.autoCruiseControl = 0 # 벌트는 사용하지 않음
      self.cruiseButtonMode = 0
      self.cruiseOnDist = 5 # 크루주온 거리 5미터로 임의설정
      self.softHoldMode = True # 소프트홀드 사용함으로 임의설정
      self.cruiseSpeedMin = 0
    elif self.params_count == 30:
      self.autoSpeedUptoRoadSpeedLimit = 1 # 기본값 100%, 즉, 1곱하기.
    elif self.params_count == 40:
      self.cruiseSpeedUnit = 5
    elif self.params_count == 50:
      self.cruiseEcoControl = 0.2 # 크루즈연비 .2km/h로 임의설정
    elif self.params_count >= 100:
      self.speedFromPCM = 0 # 벌트는 사용하지 않음.
      self.params_count = 0
  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, car_controls):
    self.v_cruise_kph_last = self.v_cruise_kph

    self._params_update(car_controls)
    self._add_log("")
    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self._update_v_cruise_apilot(CS, car_controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self._update_event_apilot(CS, car_controls)
        self.update_button_timers(CS, enabled)
      else:
        #self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        if self.params.get_int("SpeedFromPCM") == 1:
          self.v_cruise_kph_set = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        self._update_v_cruise_apilot(CS, car_controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        if self.params.get_int("SpeedFromPCM") == 0:
          self._update_event_apilot(CS, car_controls)
    else:
      self.v_cruise_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_kph_set = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.cruiseActivate = 0
      v_cruise_kph = self.update_apilot_cmd(car_controls, v_cruise_kph)

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

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), self.cruiseSpeedMin, self.cruiseSpeedMax)

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
    #if self.CP.pcmCruise:
    #  return

    #carrot
    if len(CS.buttonEvents) == 0:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_set = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = self.v_cruise_kph_set = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, self.cruiseSpeedMax)))

    self.v_cruise_cluster_kph = self.v_cruise_kph

  #def _make_event(self, car_controls, event_name):
  #  if (self.frame - self.sendEvent_frame) > (5.0 / DT_CTRL) or event_name != self.sendEvent:
  #    self.sendEvent = event_name
  #    car_controls.events.add(event_name)
  #    self.sendEvent_frame = self.frame

  def _update_event_apilot(self, CS, car_controls):
    self.events.clear()
    lp = car_controls.sm['longitudinalPlan']
    xState = lp.xState
    trafficState = lp.trafficState

    if xState != self.xState and car_controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0: #0:lead, 1:cruise, 2:e2eCruise, 3:e2eStop, 4:e2ePrepare, 5:e2eStopped
      if xState == 3 and CS.vEgo > 5.0:
        self.events.add(EventName.trafficStopping)  # stopping
      elif (xState == 4 or (xState == 2 and self.xState in [3,5])) and self.softHoldActive == 0:
        self.events.add(EventName.trafficSignGreen) # starting
    self.xState = xState

    if trafficState != self.trafficState: #0: off, 1:red, 2:green
      if self.softHoldActive == 2 and trafficState == 2:
        self.events.add(EventName.trafficSignChanged)
    self.trafficState = trafficState

  def _update_lead(self, car_controls):
    leadOne = car_controls.sm['radarState'].leadOne
    if leadOne.status: # and leadOne.radar:
      self.lead_dRel = leadOne.dRel
      self.lead_vRel = leadOne.vRel
      self.lead_vLead = leadOne.vLeadK
    else:
      self.lead_dRel = 0
      self.lead_vRel = 0
      self.lead_vLead = 0

  def _update_v_cruise_apilot(self, CS, car_controls):
    self._update_lead(car_controls)
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    if self.v_cruise_kph_set > 200:
      self.v_cruise_kph_set = self.cruiseSpeedMin
    v_cruise_kph = self.v_cruise_kph_set    
    v_cruise_kph = self._update_cruise_carrot(CS, v_cruise_kph, car_controls)
    v_cruise_kph_apply = self.cruise_control_speed(v_cruise_kph)

    self.v_cruise_kph_set = v_cruise_kph
    self.v_cruise_kph = v_cruise_kph_apply

  def update_apilot_cmd(self, car_controls, v_cruise_kph):

    self.traffic_light_q.append((-1, -1, "none", 0.0))
    self.traffic_light_count -= 1
    if self.traffic_light_count < 0:
      self.traffic_light_count = -1
      self.traffic_state = 0

    return v_cruise_kph

  def traffic_light(self, x, y, color, cnf):    
    traffic_red = 0
    traffic_green = 0
    traffic_red_trig = 0
    traffic_green_trig = 0
    for pdata in self.traffic_light_q:
      px, py, pcolor, pcnf = pdata
      if abs(x - px) < 0.2 and abs(y - py) < 0.2:
        if pcolor in ["Green Light", "Left turn"]:
          if color in ["Red Light", "Yellow Light"]:
            traffic_red_trig += cnf
            traffic_red += cnf
          elif color in ["Green Light", "Left turn"]:
            traffic_green += cnf
        elif pcolor in ["Red Light", "Yellow Light"]:
          if color in ["Green Light", "Left turn"]:
            traffic_green_trig += cnf
            traffic_green += cnf
          elif color in ["Red Light", "Yellow Light"]:
            traffic_red += cnf

    #print(self.traffic_light_q)
    if traffic_red_trig > 0:
      self.traffic_state = 11
      self._add_log("Red light triggered")
      #print("Red light triggered")
    elif traffic_green_trig > 0 and traffic_green > traffic_red:  #주변에 red light의 cnf보다 더 크면 출발... 감지오류로 출발하는경우가 생김.
      self.traffic_state = 22
      self._add_log("Green light triggered")
      #print("Green light triggered")
    elif traffic_red > 0:
      self.traffic_state = 1
      self._add_log("Red light continued")
      #print("Red light continued")
    elif traffic_green > 0:
      self.traffic_state = 2
      self._add_log("Green light continued")
      #print("Green light continued")
    else:
      pass

    self.traffic_light_q.append((x,y,color,cnf))

  def _add_log_auto_cruise(self, log):
    if self.autoCruiseControl != 0:
      self._add_log(log)

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.debugText = ""
    else:
      self.debugText = log
      self._log_timer = 300

  def _gas_released_cond(self, CS, v_cruise_kph, car_controls):
    if 0 < self.lead_dRel < CS.vEgo * 0.8 and self.autoCancelFromGasMode > 0:
      self.cruiseActivate = -1
      self._add_log_auto_cruise("Cruise Deactivate from gas.. too close leadCar!")
    elif self.autoCancelFromGasMode > 0 and self.v_ego_kph_set < self.autoResumeFromGasSpeed:
      self._add_log_auto_cruise("Cruise Deactivate from gas speed:{:.0f}".format(self.autoResumeFromGasSpeed));
      self.cruiseActivate = -1
    elif self.xState == 3 and self.autoCancelFromGasMode == 2:
      v_cruise_kph = self.v_ego_kph_set
      self._add_log_auto_cruise("Cruise Deactivate from gas pressed: traffic stopping");
      self.cruiseActivate = -1
      self.autoCruiseCancelTimer = 3.0 / DT_CTRL
    elif self.v_ego_kph_set > self.autoResumeFromGasSpeed > 0:
      if self.cruiseActivate <= 0:
        if self.gas_pressed_value > 0.6 or self.gas_pressed_count_prev > 3.0 / DT_CTRL:
          #if self.gas_pressed_max_aego < 1.5 or self.gas_pressed_value < 0.6:
          #  v_cruise_kph = self.v_ego_kph_set
          v_cruise_kph = self.v_ego_kph_set  
          self.autoCruiseCancelTimer = 0
          self._add_log_auto_cruise("Cruise Activate from gas(deep/long pressed)")          
        else:
          v_cruise_kph = self.v_ego_kph_set
          self._add_log_auto_cruise("Cruise Activate from gas(speed)")
      self.cruiseActivate = 1
    return v_cruise_kph

  def _brake_released_cond(self, CS, v_cruise_kph, car_controls):
    if self.autoResumeFromBrakeReleaseTrafficSign:
      if self.autoResumeFromGasSpeed < self.v_ego_kph_set:
        v_cruise_kph = self.v_ego_kph_set
        self._add_log_auto_cruise("Cruise Activate Brake Release")
        self.cruiseActivate = 1
      elif self.xState in [3, 5]:
        #v_cruise_kph = self.v_ego_kph_set
        self._add_log_auto_cruise("Cruise Activate from Traffic sign stop")
        self.cruiseActivate = 1
      elif 0 < self.lead_dRel < 20:
        v_cruise_kph = self.v_ego_kph_set  ## 천천히 주행하다가..지나가는 차를 잘못읽고 자동으로 크루즈가 켜지는 경우 툭튀언
        self._add_log_auto_cruise("Cruise Activate from Lead Car")
        self.cruiseActivate = 1
    return v_cruise_kph

  def _update_cruise_button(self, CS, v_cruise_kph, car_controls):
    ## ButtonEvent process
    button_kph = v_cruise_kph
    buttonEvents = CS.buttonEvents
    button_speed_up_diff = 1
    button_speed_dn_diff = 1

    button_type = 0
    if self.button_cnt > 0:
      self.button_cnt += 1
    for b in buttonEvents:
      if b.pressed and self.button_cnt==0 and b.type in [ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.gapAdjustCruise, ButtonType.cancel]:
        self.button_cnt = 1
        self.button_prev = b.type
        if b.type in [ButtonType.accelCruise, ButtonType.decelCruise]:
          self.button_long_time = 40
        else:
          self.button_long_time = 70
      elif not b.pressed and self.button_cnt > 0:
        if b.type == ButtonType.cancel:
          button_type = ButtonType.cancel
        elif not self.long_pressed and b.type == ButtonType.accelCruise:
          button_kph += button_speed_up_diff if car_controls.is_metric else button_speed_up_diff * CV.MPH_TO_KPH
          button_type = ButtonType.accelCruise
        elif not self.long_pressed and b.type == ButtonType.decelCruise:
          button_kph -= button_speed_dn_diff if car_controls.is_metric else button_speed_dn_diff * CV.MPH_TO_KPH
          button_type = ButtonType.decelCruise
        elif not self.long_pressed and b.type == ButtonType.gapAdjustCruise:
          button_type = ButtonType.gapAdjustCruise

        self.long_pressed = False
        self.button_cnt = 0
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      V_CRUISE_DELTA = 5
      if self.button_prev == ButtonType.cancel:
        button_type = ButtonType.cancel
        self.button_cnt = 0          
      elif self.button_prev == ButtonType.accelCruise:
        button_kph += V_CRUISE_DELTA - (button_kph % V_CRUISE_DELTA)
        button_type = ButtonType.accelCruise
        self.button_cnt %= self.button_long_time
      elif self.button_prev == ButtonType.decelCruise:
        button_kph -= V_CRUISE_DELTA - (-button_kph % V_CRUISE_DELTA)
        button_type = ButtonType.decelCruise
        self.button_cnt %= self.button_long_time
      elif self.button_prev == ButtonType.gapAdjustCruise:
        button_type = ButtonType.gapAdjustCruise
        self.button_cnt %= self.button_long_time

    button_kph = clip(button_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

    if button_type != 0 and car_controls.enabled:
      if self.long_pressed:
        if button_type in [ButtonType.accelCruise]:
          v_cruise_kph = button_kph
          self._add_log("Button long pressed..{:.0f}".format(v_cruise_kph))
        elif button_type in [ButtonType.decelCruise]:
          if self.cruiseButtonMode in [3]:
            self.traffic_light_count = 0.5 / DT_CTRL
            self.traffic_state = 33
            self.events.add(EventName.audioPrompt)
            self._add_log("Button force decel")
          else:
            v_cruise_kph = button_kph
            self._add_log("Button long pressed..{:.0f}".format(v_cruise_kph))
        elif button_type == ButtonType.gapAdjustCruise:          
          if False: #self.CP.pcmCruise:
            self._add_log("Button long gap pressed ..pcmCruise can't adjust")
          else:
            self._add_log("Button long gap pressed ..")
      else:
        if button_type == ButtonType.accelCruise:
          if self.softHoldActive > 0 and self.autoCruiseControl > 0:
            self.softHoldActive = 0
            self._add_log("Button softhold released ..")
          elif self.xState in [3, 5] and self.cruiseButtonMode in [3]: ## 5:e2eStopped
            self.traffic_light_count = 0.5 / DT_CTRL
            self.traffic_state = 22
            self._add_log("Button start (traffic ignore)")
          else:
            if self.cruiseButtonMode == 0:
              v_cruise_kph = button_kph
            elif self.cruiseButtonMode in [1,2,3]:
              v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
            self._add_log("Button speed up...{:.0f}".format(v_cruise_kph))
        elif button_type == ButtonType.decelCruise:
          if self.autoCruiseControl == 0 or self.cruiseButtonMode in [0,1]:
            v_cruise_kph = button_kph
            self._add_log("Button speed down...{:.0f}".format(v_cruise_kph))
          elif v_cruise_kph > self.v_ego_kph_set + 0:
            v_cruise_kph = self.v_ego_kph_set
            self._add_log("Button speed set...{:.0f}".format(v_cruise_kph))
          else:
            self.cruiseActiveReady = 1
            self.cruiseActivate = -1
            self.events.add(EventName.audioPrompt)
        elif button_type == ButtonType.cancel:
          print("************* cancel button pressed..")
        elif button_type == ButtonType.gapAdjustCruise:
          if False: #self.CP.pcmCruise:
            self._add_log("Button long gap pressed ..pcmCruise can't adjust")
          else:
            self._add_log("Button gap pressed ..")
            longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
            car_controls.personality = (car_controls.personality - 1) % longitudinalPersonalityMax
            self.params.put_nonblocking('LongitudinalPersonality', str(car_controls.personality))
         
    elif button_type != 0 and not car_controls.enabled:
      self.cruiseActivate = 0

    if CS.vEgo > 1.0:
      self.softHoldActive = 0

    if button_type in [ButtonType.cancel, ButtonType.accelCruise, ButtonType.decelCruise]:
      self.autoCruiseCancelTimer = 0
      if button_type == ButtonType.cancel:
        if self.autoCruiseCancelState > 0:
          pass
        else:
          self._add_log("Button cancel : Cruise OFF")
        self.autoCruiseCancelState = 1
        self.events.add(EventName.audioPrompt)
        print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))
      else:
        self.autoCruiseCancelState = 0
        print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))

    if self.brake_pressed_count > 0 or self.gas_pressed_count > 0:
      if self.cruiseActivate > 0:
        self.cruiseActivate = 0

    return v_cruise_kph

  def _update_cruise_carrot(self, CS, v_cruise_kph, car_controls):
    if v_cruise_kph > 200:
      self._add_log("VCruise: speed initialize....")
      v_cruise_kph = self.cruiseSpeedMin

    if CS.brakePressed:
      self.brake_pressed_count = max(1, self.brake_pressed_count + 1)
      self.softHold_count = self.softHold_count + 1 if self.softHoldMode > 0 and CS.vEgo < 0.1 else 0
      self.softHoldActive = 1 if self.softHold_count > 60 and car_controls.CP.openpilotLongitudinalControl else 0      
    else:
      self.softHold_count = 0
      self.brake_pressed_count = min(-1, self.brake_pressed_count - 1)

    if self.softHoldActive > 0 or CS.brakePressed:
      self.brake_pressed_frame = self.frame

    gas_tok = False
    if CS.gasPressed:
      self.gas_pressed_count = max(1, self.gas_pressed_count + 1)
      self.softHoldActive = 0
      self.gas_pressed_value = max(CS.gas, self.gas_pressed_value)
      self.gas_pressed_count_prev = self.gas_pressed_count
      self.gas_pressed_max_aego = max(self.gas_pressed_max_aego, CS.aEgo) if self.gas_pressed_count > 1 else 0
    else:
      gas_tok = True if 0 < self.gas_pressed_count < 0.4 / DT_CTRL else False  ## gas_tok: 0.4 seconds
      self.gas_pressed_count = min(-1, self.gas_pressed_count - 1)
      if self.gas_pressed_count < -1:
        self.gas_pressed_max = 0
        self.gas_pressed_count_prev = 0

    if car_controls.enabled or CS.brakePressed or CS.gasPressed:
      self.cruiseActiveReady = 0
      if CS.gasPressed and self.accel_output < -0.5:
        self.autoCruiseCancelTimer = 5.0 / DT_CTRL #잠시 오토크루멈춤
        self.cruiseActivate = -1
        self._add_log("Cruise off (GasPressed while braking)")

    v_cruise_kph = self._update_cruise_button(CS, v_cruise_kph, car_controls)

    ## Auto Engage/Disengage via Gas/Brake
    if gas_tok:
      if (self.autoCruiseCancelTimer == 0 or (self.frame - self.gas_tok_frame) < 1.0 / DT_CTRL):  ## 1초이내 더블 엑셀톡인경우..
        self.autoCruiseCancelTimer = 0
        if car_controls.enabled:
          if (self.frame - self.brake_pressed_frame) < 3.0 / DT_CTRL:
            v_cruise_kph = self.v_ego_kph_set
            self._add_log("Gas tok speed set to current (prev. brake pressed)")
          else:
            v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
            self._add_log("Gas tok speed up...{:.0f}".format(v_cruise_kph))
        elif self.autoResumeFromGasSpeed > 0:
          self._add_log_auto_cruise("Cruise Activate from GasTok")
          #v_cruise_kph = self.v_ego_kph_set
          self.cruiseActivate = 1
      self.gas_tok_frame = self.frame
    elif self.gas_pressed_count == -1:
      v_cruise_kph = self._gas_released_cond(CS, v_cruise_kph, car_controls)
      if self.autoCruiseCancelTimer > 0 and self.cruiseActivate > 0:
        self.cruiseActivate = 0
        self.cruiseActiveReady = 1

    elif self.brake_pressed_count == -1:
      if self.softHoldActive == 1 and self.softHoldMode > 0:
        self._add_log_auto_cruise("Cruise Activete from SoftHold")
        self.softHoldActive = 2
        self.cruiseActivate = 1
        self.autoCruiseCancelTimer = 0
      else:
        v_cruise_kph =  self._brake_released_cond(CS, v_cruise_kph, car_controls)
        if self.autoCruiseCancelTimer > 0 and self.cruiseActivate > 0:
          self.cruiseActivate = 0

    elif self.gas_pressed_count > 0 and self.v_ego_kph_set > v_cruise_kph:
      v_cruise_kph = self.v_ego_kph_set
      if V_CRUISE_MAX > v_cruise_kph > self.cruiseSpeedMax:
        self.cruiseSpeedMax = v_cruise_kph
    elif self.cruiseActiveReady > 0 and self.autoCruiseCancelTimer == 0:
      if 0 < self.lead_dRel or self.xState == 3:
        self._add_log_auto_cruise("Cruise Activate from Lead or Traffic sign stop")
        self.cruiseActivate = 1
    elif not car_controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0: # and self.autoCruiseCancelTimer == 0:
      cruiseOnDist = abs(self.cruiseOnDist)
      if self.autoCruiseControl >= 2 and self.lead_vRel < 0 and 0 < self.lead_dRel < CS.vEgo ** 2 / (2.0 * 2):
        self._add_log_auto_cruise("Auto Cruise Activate")
        self.cruiseActivate = 1
      elif cruiseOnDist > 0 and CS.vEgo > 0.02 and  0 < self.lead_dRel < cruiseOnDist:
        self.events.add(EventName.stopStop)
        self._add_log_auto_cruise("CruiseOnDist Activate")
        self.cruiseActivate = 1


    v_cruise_kph = self.update_apilot_cmd(car_controls, v_cruise_kph)

    if self.CP.pcmCruise:
      if self.v_ego_kph_set >= 10 or 0 < self.lead_dRel < 140:
        pass
      else:
        self.cruiseActivate = 0

    if self.autoCruiseControl < 1 or self.autoCruiseCancelState > 0 or not car_controls.enable_avail: # or CS.brakeHoldActive 벌트는 오토홀드 구현하면 추가해야 됨
      if self.cruiseActivate != 0:
        self._add_log_auto_cruise(f"Cancel auto Cruise = {self.cruiseActivate}")
      self.cruiseActivate = 0
      self.softHoldActive = 0
    v_cruise_kph = clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)
    return v_cruise_kph

  def v_cruise_speed_up(self, v_cruise_kph):
    for speed in range (10, int(self.cruiseSpeedMax), self.cruiseSpeedUnit):
      if v_cruise_kph < speed:
        v_cruise_kph = speed
        break
    return clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

  def cruise_control_speed(self, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph        
    if self.cruiseEcoControl > 0:
      if self.cruiseSpeedTarget > 0:
        if self.cruiseSpeedTarget < v_cruise_kph:
          self.cruiseSpeedTarget = v_cruise_kph
        elif self.cruiseSpeedTarget > v_cruise_kph:
          self.cruiseSpeedTarget = 0
      elif self.cruiseSpeedTarget == 0 and self.v_ego_kph_set + 3 < v_cruise_kph and v_cruise_kph > 20.0:  # 주행중 속도가 떨어지면 다시 크루즈연비제어 시작.
        self.cruiseSpeedTarget = v_cruise_kph

      if self.cruiseSpeedTarget != 0:  ## 크루즈 연비 제어모드 작동중일때: 연비제어 종료지점
        if self.v_ego_kph_set > self.cruiseSpeedTarget: # 설정속도를 초과하면..
          self.cruiseSpeedTarget = 0
        else:
          v_cruise_kph_apply = self.cruiseSpeedTarget + self.cruiseEcoControl  # + 설정 속도로 설정함.
    else:
      self.cruiseSpeedTarget = 0

    return v_cruise_kph_apply

