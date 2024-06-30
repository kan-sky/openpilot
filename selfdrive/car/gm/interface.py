#!/usr/bin/env python3
from cereal import car
from math import fabs, exp
from panda import Panda

from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, create_button_event, scale_tire_stiffness, is_ecu_disconnected, get_safety_config
from selfdrive.car.gm.radar_interface import RADAR_HEADER_MSG
from selfdrive.car.gm.values import CAR, Ecu, ECU_FINGERPRINT, FINGERPRINTS, CruiseButtons, \
                                    CarControllerParams, EV_CAR, CAMERA_ACC_CAR, CanBus, GMFlags, CC_ONLY_CAR
from selfdrive.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD
from selfdrive.controls.lib.drive_helpers import get_friction

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel,
                CruiseButtons.GAP_DIST: ButtonType.gapAdjustCruise}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    desired_angle *= 0.09760208
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.04689655 * sigmoid * (v_ego + 10.028217)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint in (CAR.VOLT, CAR.VOLT2018, CAR.VOLT_CC):
      return self.get_steer_feedforward_volt
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  @staticmethod
  def torque_from_lateral_accel_bolt(lateral_accel_value: float, torque_params: car.CarParams.LateralTorqueTuning,
                                     lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      return 1 / (1 + exp(-val)) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear
    a, b, c, _ = [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178]  # weights computed offline

    steer_torque = (sig(lateral_accel_value * a) * b) + (lateral_accel_value * c)
    return float(steer_torque) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint == CAR.BOLT_EUV:
      return self.torque_from_lateral_accel_bolt
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False
    ret.enableBsm = True #0x142 in fingerprint[CanBus.POWERTRAIN]
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    #ret.enableGasInterceptor = 512 in fingerprint[0]
    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera)

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.15]

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kiBP = [0.]

    if candidate in CAMERA_ACC_CAR:
      ret.experimentalLongitudinalAvailable = candidate not in CC_ONLY_CAR
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarUnavailable = True  # no radar
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM
      ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      ret.minSteerSpeed = 10 * CV.KPH_TO_MS

      # Tuning for experimental long
      ret.longitudinalTuning.kpV = [2.0]
      ret.longitudinalTuning.kiV = [0.72]
      ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25

      if experimental_long:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarUnavailable = False # RADAR_HEADER_MSG not in fingerprint[CanBus.OBSTACLE] and not use_off_car_defaults
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, initial engage could (conservatively) be above -1mph
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.minSteerSpeed = 6.7 * CV.MPH_TO_MS

      # Tuning
      ret.longitudinalTuning.kpV = [1.75]
      ret.longitudinalTuning.kiV = [0.36]
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG
      ret.naviCluster = 0 #현기차용, carstate.py에 False 값을 주기 위한 것뿐임
    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.MALIBU, CAR.BUICK_REGAL, CAR.EQUINOX}



    # for autohold on ui icon
    # ret.enableAutoHold = 241 in fingerprint[0]
    # ret.openpilotLongitudinalControl = True
    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.steerActuatorDelay = 0.2  # Default delay, not measured yet

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz
    ret.longitudinalActuatorDelayLowerBound = 0.42
    ret.longitudinalActuatorDelayUpperBound = 0.5  # large delay to initially start braking
    if candidate == CAR.VOLT2018:
      ret.lateralTuning.init('torque')
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      tire_stiffness_factor = 0.469  # Stock Michelin Energy Saver A/S, LiveParameters
      ret.centerToFront = ret.wheelbase * 0.45  # Volt Gen 1, TODO corner weigh
      ret.steerActuatorDelay = 0.38
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.longitudinalTuning.kpBP = [0.]
      ret.longitudinalTuning.kpV = [1.75]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.36]
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.stopAccel = -0.5
      ret.startAccel = 0.8
      ret.vEgoStarting = 0.25
      ret.vEgoStopping = 0.25

    if ret.enableGasInterceptor:
      ret.flags |= GMFlags.PEDAL_LONG.value
      ret.minEnableSpeed = -1
      ret.pcmCruise = False
      ret.openpilotLongitudinalControl = True
      # Note: Low speed, stop and go not tested. Should be fairly smooth on highway
      ret.longitudinalTuning.kpV = [0.35, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.0]
      ret.longitudinalTuning.kiV = [0.1, 0.1]
      ret.longitudinalTuning.kf = 0.15
      ret.stoppingDecelRate = 0.8  # reach stopping target smoothly, brake_travel/s while trying to stop
      ret.vEgoStopping = 0.5  # Speed at which the car goes into stopping state, when car starts requesting stopping accel
      ret.vEgoStarting = 0.5  # Speed at which the car goes into starting state, when car starts requesting starting accel,
      # vEgoStarting needs to be > or == vEgoStopping to avoid state transition oscillation
      ret.stoppingControl = True

    elif candidate in CC_ONLY_CAR:
      ret.flags |= GMFlags.CC_LONG.value
      ret.radarUnavailable = True
      ret.experimentalLongitudinalAvailable = False
      ret.minEnableSpeed = 24 * CV.MPH_TO_MS
      ret.openpilotLongitudinalControl = True
      ret.pcmCruise = False

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.stoppingControl = True
    ret.startingState = True

    ret.longitudinalTuning.kf = 1.0
    ret.stoppingDecelRate = 3.0  # reach stopping target smoothly, brake_travel/s while trying to stop
    ret.stopAccel = -2.0  # Required acceleration to keep vehicle stationary
    ret.vEgoStopping = 0.5  # Speed at which the car goes into stopping state, when car starts requesting stopping accel
    ret.vEgoStarting = 0.5  # Speed at which the car goes into starting state, when car starts requesting starting accel,

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback, self.cp_chassis)

    ret.engineRpm = self.CS.engineRPM
    buttonEvents = [] # kans
    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.CS.cruise_buttons != CruiseButtons.UNPRESS or self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents.extend(create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT,
                                              unpressed_btn=CruiseButtons.UNPRESS))
    # kans : long_button GAP for cruise Mode(safety, ecco, high-speed..)
    if self.CS.distance_button_pressed:
      buttonEvents.append(car.CarState.ButtonEvent(pressed=True, type=ButtonType.gapAdjustCruise))
    ret.buttonEvents = buttonEvents # kans

    # The ECM allows enabling on falling edge of set, but only rising edge of resume
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=self.CP.pcmCruise, enable_buttons=(ButtonType.decelCruise,))
    if not self.CP.pcmCruise:
      if any(b.type == ButtonType.accelCruise and b.pressed for b in ret.buttonEvents):
        events.add(EventName.buttonEnable)

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed or self.CS.moving_backward
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20 and
                                       self.CP.networkLocation == NetworkLocation.fwdCamera):
      events.add(EventName.belowEngageSpeed)

    # kans: 정지 상태이면서, 자동재개 신호(self.CP.autoResumeSng)가 비활성화되어 있고, 
    # resumeRequired 이벤트가 비활성화되어 있지 않으면, resumeRequired 이벤트를 활성화하고, 
    # resumeRequired 이벤트를 한번 보여주게 한다.
    if ret.cruiseState.standstill and not self.CP.autoResumeSng and not self.CS.disable_resumeRequired:
      events.add(EventName.resumeRequired)
      self.CS.resumeRequired_shown = True

    # kans: resumeRequired 이벤트가 표시된 후에는, 자동으로 재개될 때까지 resumeRequired 이벤트를 비활성화한다.
    if self.CS.resumeRequired_shown and not ret.cruiseState.standstill:
      self.CS.disable_resumeRequired = True

    # kans: 속도가 최소조향속도 미만이고, belowSteerSpeed 이벤트가 비활성화되어 있지 않으면, 
    # belowSteerSpeed 이벤트를 활성화하고,
    # belowSteerSpeed이벤트를 한번 보여주게 한다.
    if ret.vEgo < self.CP.minSteerSpeed and not self.CS.disable_belowSteerSpeed:
      events.add(EventName.belowSteerSpeed)
      self.CS.belowSteerSpeed_shown = True

    # kans: belowSteerSpeed 이벤트가 한번 표시된 후에는, 속도가 최소조향속도보다 높아질 때까지 belowSteerSpeed 이벤트를 비활성화한다.
    if self.CS.belowSteerSpeed_shown and ret.vEgo > self.CP.minSteerSpeed:
      self.CS.disable_belowSteerSpeed = True # kans
    if (self.CP.flags & GMFlags.CC_LONG.value) and ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.speedTooLow)
    # 페달롱컨차(BOLT)를 위한 코드이므로 주석처리
    #if (self.CP.flags & GMFlags.PEDAL_LONG.value) and self.CP.transmissionType == TransmissionType.direct and not self.CS.single_pedal_mode:
    #  events.add(EventName.pedalInterceptorNoBrake)

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
