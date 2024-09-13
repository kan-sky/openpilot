from cereal import car
import cereal.messaging as messaging
from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import MAX_CTRL_SPEED, CarStateBase, CarControllerBase
from opendbc.car.volkswagen.values import CarControllerParams as VWCarControllerParams
from opendbc.car.hyundai.interface import ENABLE_BUTTONS as HYUNDAI_ENABLE_BUTTONS

from openpilot.selfdrive.selfdrived.events import Events

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
EventName = car.OnroadEvent.EventName
NetworkLocation = structs.CarParams.NetworkLocation


# TODO: the goal is to abstract this file into the CarState struct and make events generic
class MockCarState:
  def __init__(self):
    self.sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  def update(self, CS: car.CarState):
    self.sm.update(0)
    gps_sock = 'gpsLocationExternal' if self.sm.recv_frame['gpsLocationExternal'] > 1 else 'gpsLocation'

    CS.vEgo = self.sm[gps_sock].speed
    CS.vEgoRaw = self.sm[gps_sock].speed

    return CS


class CarSpecificEvents:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP

    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.no_steer_warning = False
    self.silent_steer_warning = True

    # kans: steer이벤트(일시불가) 줄이기 위해
    self.belowSteerSpeed_shown = False
    self.disable_belowSteerSpeed = False
    self.resumeRequired_shown = False
    self.disable_resumeRequired = False

    # tw: steer warning
    self.steer_warning = 0

  def update(self, CS: CarStateBase, CS_prev: car.CarState, CC: CarControllerBase, CC_prev: car.CarControl):
    if self.CP.carName in ('body', 'mock'):
      events = Events()

    elif self.CP.carName == 'subaru':
      events = self.create_common_events(CS.out, CS_prev)

    elif self.CP.carName == 'ford':
      events = self.create_common_events(CS.out, CS_prev, extra_gears=[GearShifter.manumatic])

    elif self.CP.carName == 'nissan':
      events = self.create_common_events(CS.out, CS_prev, extra_gears=[GearShifter.brake])

      if CS.lkas_enabled:  # type: ignore[attr-defined]
        events.add(EventName.invalidLkasSetting)

    elif self.CP.carName == 'mazda':
      events = self.create_common_events(CS.out, CS_prev)

      if CS.lkas_disabled:  # type: ignore[attr-defined]
        events.add(EventName.lkasDisabled)
      elif CS.low_speed_alert:  # type: ignore[attr-defined]
        events.add(EventName.belowSteerSpeed)

    elif self.CP.carName == 'chrysler':
      events = self.create_common_events(CS.out, CS_prev, extra_gears=[GearShifter.low])

      # Low speed steer alert hysteresis logic
      if self.CP.minSteerSpeed > 0. and CS.out.vEgo < (self.CP.minSteerSpeed + 0.5):
        self.low_speed_alert = True
      elif CS.out.vEgo > (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = False
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.carName == 'honda':
      events = self.create_common_events(CS.out, CS_prev, pcm_enable=False)

      if self.CP.pcmCruise and CS.out.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.belowEngageSpeed)

      if self.CP.pcmCruise:
        # we engage when pcm is active (rising edge)
        if CS.out.cruiseState.enabled and not CS_prev.cruiseState.enabled:
          events.add(EventName.pcmEnable)
        elif not CS.out.cruiseState.enabled and (CC_prev.actuators.accel >= 0. or not self.CP.openpilotLongitudinalControl):
          # it can happen that car cruise disables while comma system is enabled: need to
          # keep braking if needed or if the speed is very low
          if CS.out.vEgo < self.CP.minEnableSpeed + 2.:
            # non loud alert if cruise disables below 25mph as expected (+ a little margin)
            events.add(EventName.speedTooLow)
          else:
            events.add(EventName.cruiseDisabled)
      if self.CP.minEnableSpeed > 0 and CS.out.vEgo < 0.001:
        events.add(EventName.manualRestart)

    elif self.CP.carName == 'toyota':
      events = self.create_common_events(CS.out, CS_prev)

      if self.CP.openpilotLongitudinalControl:
        if CS.out.cruiseState.standstill and not CS.out.brakePressed:
          events.add(EventName.resumeRequired)
        if CS.low_speed_lockout:  # type: ignore[attr-defined]
          events.add(EventName.lowSpeedLockout)
        if CS.out.vEgo < self.CP.minEnableSpeed:
          events.add(EventName.belowEngageSpeed)
          if CC_prev.actuators.accel > 0.3:
            # some margin on the actuator to not false trigger cancellation while stopping
            events.add(EventName.speedTooLow)
          if CS.out.vEgo < 0.001:
            # while in standstill, send a user alert
            events.add(EventName.manualRestart)

    elif self.CP.carName == 'gm':
      # The ECM allows enabling on falling edge of set, but only rising edge of resume
      events = self.create_common_events(CS.out, CS_prev, extra_gears=[GearShifter.sport, GearShifter.low,
                                                          GearShifter.eco, GearShifter.manumatic],
                                         pcm_enable=self.CP.pcmCruise, enable_buttons=(ButtonType.decelCruise,))
      if not self.CP.pcmCruise:
        if any(b.type == ButtonType.accelCruise and b.pressed for b in CS.out.buttonEvents):
          events.add(EventName.buttonEnable)

      # Enabling at a standstill with brake is allowed
      # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
      below_min_enable_speed = CS.out.vEgo < self.CP.minEnableSpeed or CS.moving_backward  # type: ignore[attr-defined]
      if below_min_enable_speed and not (CS.out.standstill and CS.out.brake >= 20 and
                                         self.CP.networkLocation == NetworkLocation.fwdCamera):
        events.add(EventName.belowEngageSpeed)

      # kans:
      # 정지 상태이면서, 자동재개 신호(self.CP.autoResumeSng)가 비활성화되어 있고, 
      # resumeRequired 이벤트가 비활성화되어 있지 않으면, resumeRequired 이벤트를 활성화하고, 
      # resumeRequired 이벤트를 한번 보여주게 한다.
      if CS.out.cruiseState.standstill and not (self.CP.autoResumeSng or self.disable_resumeRequired):
        events.add(EventName.resumeRequired)
        self.resumeRequired_shown = True

      # kans: resumeRequired 이벤트가 표시된 후에는, 자동으로 재개될 때까지 resumeRequired 이벤트를 비활성화한다.
      if self.resumeRequired_shown and not CS.out.cruiseState.standstill:
        self.disable_resumeRequired = True

      # kans: 속도가 최소조향속도 미만이고, belowSteerSpeed 이벤트가 비활성화되어 있지 않으면, 
      # belowSteerSpeed 이벤트를 활성화하고,
      # belowSteerSpeed이벤트를 한번 보여주게 한다.
      if CS.out.vEgo < self.CP.minSteerSpeed and not self.disable_belowSteerSpeed:
        events.add(EventName.belowSteerSpeed)
        self.belowSteerSpeed_shown = True

      # kans: belowSteerSpeed 이벤트가 한번 표시된 후에는, 속도가 최소조향속도보다 높아질 때까지 belowSteerSpeed 이벤트를 비활성화한다.
      if self.belowSteerSpeed_shown and CS.out.vEgo >= self.CP.minSteerSpeed:
        self.disable_belowSteerSpeed = True # kans


    elif self.CP.carName == 'volkswagen':
      events = self.create_common_events(CS.out, CS_prev, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic],
                                         pcm_enable=not self.CP.openpilotLongitudinalControl,
                                         enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

      # Low speed steer alert hysteresis logic
      if (self.CP.minSteerSpeed - 1e-3) > VWCarControllerParams.DEFAULT_MIN_STEER_SPEED and CS.out.vEgo < (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = True
      elif CS.out.vEgo > (self.CP.minSteerSpeed + 2.):
        self.low_speed_alert = False
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

      if self.CP.openpilotLongitudinalControl:
        if CS.out.vEgo < self.CP.minEnableSpeed + 0.5:
          events.add(EventName.belowEngageSpeed)
        if CC_prev.enabled and CS.out.vEgo < self.CP.minEnableSpeed:
          events.add(EventName.speedTooLow)

      if CC.eps_timer_soft_disable_alert:  # type: ignore[attr-defined]
        events.add(EventName.steerTimeLimit)

    elif self.CP.carName == 'hyundai':
      # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
      # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
      # Main button also can trigger an engagement on these cars
      allow_enable = any(btn in HYUNDAI_ENABLE_BUTTONS for btn in CS.cruise_buttons) or any(CS.main_buttons)  # type: ignore[attr-defined]
      events = self.create_common_events(CS.out, CS_prev, pcm_enable=self.CP.pcmCruise, allow_enable=allow_enable)

      # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
      if CS.out.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
        self.low_speed_alert = True
      if CS.out.vEgo > (self.CP.minSteerSpeed + 4.):
        self.low_speed_alert = False
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    else:
      raise ValueError(f"Unsupported car: {self.CP.carName}")

    return events

  def create_common_events(self, CS: structs.CarState, CS_prev: car.CarState, extra_gears=None, pcm_enable=True,
                           allow_enable=True, enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise)):
    events = Events()

    if CS.doorOpen:
      events.add(EventName.doorOpen)
    if CS.seatbeltUnlatched:
      events.add(EventName.seatbeltNotLatched)
    if CS.gearShifter != GearShifter.drive and (extra_gears is None or
       CS.gearShifter not in extra_gears):
      events.add(EventName.wrongGear)
    if CS.gearShifter == GearShifter.reverse:
      events.add(EventName.reverseGear)
    if not CS.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if CS.espDisabled:
      events.add(EventName.espDisabled)
    if CS.espActive:
      events.add(EventName.espActive)
    if CS.stockFcw:
      events.add(EventName.stockFcw)
    if CS.stockAeb:
      events.add(EventName.stockAeb)
    if CS.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if CS.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)
    #if CS.brakeHoldActive and self.CP.openpilotLongitudinalControl:
    #  events.add(EventName.brakeHold)
    if CS.parkingBrake:
      events.add(EventName.parkBrake)
    if CS.accFaulted:
      events.add(EventName.accFaulted)
    if CS.steeringPressed:
      events.add(EventName.steerOverride)
    if CS.brakePressed and CS.standstill:
      events.add(EventName.preEnableStandstill)
    if CS.gasPressed:
      events.add(EventName.gasPressedOverride)
    if CS.vehicleSensorsInvalid:
      events.add(EventName.vehicleSensorsInvalid)

    # Handle button presses
    for b in CS.buttonEvents:
      # Enable OP long on falling edge of enable buttons (defaults to accelCruise and decelCruise, overridable per-port)
      if not self.CP.pcmCruise and (b.type in enable_buttons and not b.pressed):
        events.add(EventName.buttonEnable)
      # Disable on rising and falling edge of cancel for both stock and OP long
      #if b.type == ButtonType.cancel:
      #  events.add(EventName.buttonCancel)

    # Handle permanent and temporary steering faults
    # tw: steer warning
    self.steer_warning = self.steer_warning + 1 if CS_prev.steerFaultTemporary else 0
    self.steering_unpressed = 0 if CS_prev.steeringPressed else self.steering_unpressed + 1
    if CS_prev.steerFaultPermanent: # 스티어폴트 선행.
      events.add(EventName.steerUnavailable)

    elif CS_prev.steerFaultTemporary: # 일시오류 체크.
      if CS_prev.steeringPressed and (not CS_prev.steerFaultTemporary or self.no_steer_warning):
        self.no_steer_warning = True
      else:
        self.no_steer_warning = False

        # 핸들손올림이나 일시 스티어오류가 0.5초이상일때 경고하며, 운전자 개입은 안하는 것으로 한다.
        if self.steering_unpressed > int(0.5/DT_CTRL) and self.steer_warning > int(0.5/DT_CTRL):
          pass # events.add(EventName.steerTempUnavailable)
        else:
          events.add(EventName.steerTempUnavailableSilent)

    else:
      self.no_steer_warning = False
      self.silent_steer_warning = False


    # we engage when pcm is active (rising edge)
    # enabling can optionally be blocked by the car interface
    if pcm_enable:
      if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled and allow_enable:
        events.add(EventName.pcmEnable)
      elif not CS.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    return events
