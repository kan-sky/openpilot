#!/usr/bin/env python3
import json
import os
from opendbc.car.structs import car
from math import fabs, exp
import numpy as np
from openpilot.common.params import Params
from opendbc.car import get_safety_config, structs
from opendbc.car.lateral import get_friction
from opendbc.car.common.basedir import BASEDIR
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.carstate import CarState
from opendbc.car.gm.radar_interface import RadarInterface, RADAR_HEADER_MSG, CAMERA_DATA_HEADER_MSG
from opendbc.car.gm.values import CAR, CarControllerParams, EV_CAR, CAMERA_ACC_CAR, CanBus, GMFlags, SDGM_CAR, GMSafetyFlags, ALT_ACCS, ASCM_INT, SASCM_CAR
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType

TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation

CAM_MSG = 0x320  # AEBCmd
                 # TODO: Is this always linked to camera presence?
ACCELERATOR_POS_MSG = 0xbe
TPMS_POS_MSG = 0x52B ## TPMS
PEDAL_MSG = 0x201

NON_LINEAR_TORQUE_PARAMS = {
  CAR.CHEVROLET_BOLT_EUV: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  CAR.GMC_ACADIA: [4.78003305, 1.0, 0.3122, 0.05591772],
  CAR.CHEVROLET_SILVERADO: [3.29974374, 1.0, 0.25571356, 0.0465122]
}

NEURAL_PARAMS_PATH = os.path.join(BASEDIR, 'torque_data/neural_ff_weights.json')

class NanoFFModel:
  def __init__(self, platform):
    with open(NEURAL_PARAMS_PATH) as f:
      self.weights = {
        k: np.asarray(v)
        for k, v in json.load(f)[platform].items()
      }

  def predict(self, inputs):
    w = self.weights
    x = np.asarray(inputs, dtype=float)

    x = (x - w["input_norm_mat"][:, 0]) / (
      w["input_norm_mat"][:, 1] - w["input_norm_mat"][:, 0]
    )

    x = np.maximum(0.0, x @ w["w_1"] + w["b_1"])
    x = np.maximum(0.0, x @ w["w_2"] + w["b_2"])
    x = np.maximum(0.0, x @ w["w_3"] + w["b_3"])
    x = x @ w["w_4"] + w["b_4"]

    return float(
      x[0] *
      (w["output_norm_mat"][1] - w["output_norm_mat"][0]) +
      w["output_norm_mat"][0]
    )

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport, structs.CarState.GearShifter.low,
                    structs.CarState.GearShifter.eco, structs.CarState.GearShifter.manumatic)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.CHEVROLET_VOLT:
      return self.get_steer_feedforward_volt
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  def get_lataccel_torque_siglin(self) -> tuple[list[float], np.ndarray]:

    def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
      # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
      # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
      # This has big effect on the stability about 0 (noise when going straight)
      non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
      assert non_linear_torque_params, "The params are not defined"
      a, b, c, d = non_linear_torque_params
      sig_input = a * lateral_acceleration
      sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
      steer_torque = (sig * b) + (lateral_acceleration * c) + d
      return float(steer_torque)

    lataccel_values = np.arange(-5.0, 5.0, 0.01)
    torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
    assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
    return torque_values, lataccel_values

  def set_torque_from_lateral_accel_context(self, roll_compensation: float, v_ego: float, a_ego: float) -> None:
    self._nano_roll_compensation = float(roll_compensation)
    self._nano_v_ego = float(v_ego)
    self._nano_a_ego = float(a_ego)

  def torque_from_lateral_accel_neural(self, lateral_acceleration: float,
                                       torque_params: structs.CarParams.LateralTorqueTuning) -> float:
    # Current latcontrol_torque already applies friction compensation in lateral-acceleration space.
    # Preserve the latest 2-argument callback API and only supply NanoFF's extra dynamic inputs here.
    roll_compensation = getattr(self, "_nano_roll_compensation", 0.0)
    inputs = [
      float(lateral_acceleration) + roll_compensation,
      roll_compensation,
      getattr(self, "_nano_v_ego", 0.0),
      getattr(self, "_nano_a_ego", 0.0),
    ]
    return float(self.neural_ff_model.predict(inputs))

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    with open(NEURAL_PARAMS_PATH) as f:
      neural_ff_cars = json.load(f).keys()

    if self.CP.carFingerprint in neural_ff_cars:
      self.neural_ff_model = NanoFFModel("CHEVROLET_VOLT")
      return self.torque_from_lateral_accel_neural
    elif self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(lateral_acceleration, lataccel_values, torque_values)
      return torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(torque, torque_values, lataccel_values)
      return lateral_accel_from_torque_siglin
    else:
      return self.lateral_accel_from_torque_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "gm"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False
    ret.enableBsm = 0x142 in fingerprint[CanBus.POWERTRAIN] or 0x142 in fingerprint[CanBus.CAMERA]
    ret.startAccel = 1.0
    ret.radarTimeStep = 0.067
    ret.alternativeExperience = 0
    params = Params()

    useEVTables = params.get_bool("EVTable")

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.EV.value
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.kiBP = [5., 35.]

    if candidate in (CAMERA_ACC_CAR | SDGM_CAR | ASCM_INT):
      ret.alphaLongitudinalAvailable = candidate not in (ASCM_INT | SDGM_CAR)
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarUnavailable = 0x460 not in fingerprint[CanBus.OBSTACLE]
      ret.pcmCruise = True
      ret.minEnableSpeed = -1 if candidate in SDGM_CAR else 5 * CV.KPH_TO_MS
      ret.minSteerSpeed = 10 * CV.KPH_TO_MS
      if candidate in SDGM_CAR:
        # Kans: SDGM은 0x2FF로 알파롱컨 사용
        ret.alphaLongitudinalAvailable = 0x2FF in fingerprint[CanBus.CAMERA]
        # SDGM은 항상 오파롱 사용
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_SDGM.value
        # BE(0xBE)가 없는 SDGM에서만 C9 브레이크 강제
        if ACCELERATOR_POS_MSG not in fingerprint[CanBus.POWERTRAIN]:
          ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.FORCE_BRAKE_C9.value
          ret.flags |= GMFlags.FORCE_BRAKE_C9.value
        ret.minEnableSpeed = -1.  # engage speed is decided by pcm
        ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      elif candidate in ASCM_INT:
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM.value
        ret.minSteerSpeed = 7 * CV.MPH_TO_MS
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.ASCM_INT.value
      else:
        # CAMERA_ACC_CAR
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM.value

      # Tuning for experimental long
      ret.longitudinalTuning.kiV = [2.0, 1.5]

      if alpha_long:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM_LONG.value

      if candidate in ALT_ACCS:
        ret.alphaLongitudinalAvailable = False
        ret.openpilotLongitudinalControl = False
        ret.minEnableSpeed = -1.  # engage speed is decided by PCM

    else:  # ASCM, OBD-II harness
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_ASCM_LONG.value
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      # LRR messages can take up to a few seconds to start sending after ignition, check camera data as well which starts earlier
      ret.radarUnavailable = RADAR_HEADER_MSG not in fingerprint[CanBus.OBSTACLE] and CAMERA_DATA_HEADER_MSG not in fingerprint[CanBus.OBSTACLE] and not docs
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, but initial engage must (conservatively) be above 18mph
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.minSteerSpeed = (6.7 if useEVTables else 7) * CV.MPH_TO_MS

      # Tuning
      ret.longitudinalTuning.kiV = [2.4, 1.5]

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to opendbc/car/tests/routes.py, we can remove it from this list.
    #ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.CHEVROLET_MALIBU, CAR.BUICK_REGAL} or \
    #                  (ret.networkLocation == NetworkLocation.gateway and ret.radarUnavailable)

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.3  # Default delay, not measured yet

    ret.steerLimitTimer = 0.4
    ret.longitudinalActuatorDelay = 0.3  # large delay to initially start braking

    if candidate == CAR.CHEVROLET_VOLT:
      ret.steerActuatorDelay = 0.3
      ret.longitudinalTuning.kpBP = [0.]
      ret.longitudinalTuning.kpV = [1.0]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [.35]
      ret.longitudinalTuning.kf = 1.0
      ret.stoppingDecelRate = 1.0 # brake_travel/s while trying to stop
      ret.vEgoStopping = 0.5 # 정지상태로 판단하는 속도(값이 작을수록 정지시작은 늦어질 수 있지만 출발조건을 빠르게 해줄 수 있음)
      ret.vEgoStarting = 0.3 # 출발상태로 판단하는 속도(값이 클수록 더 높은 속도까지 내주어서 출발가속이 강해질 수 있음)
      ret.stopAccel = -2.0
      ret.startingState = True
      ret.startAccel = 1.0
      ret.autoResumeSng = True
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_ACADIA_ASCM:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif candidate == CAR.CHEVROLET_MALIBU:
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_ASCM_LONG.value
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarUnavailable = False
      ret.pcmCruise = False
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.steerActuatorDelay = 0.2
      ret.autoResumeSng = True

    elif candidate == CAR.CHEVROLET_MALIBU_SASCM:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.autoResumeSng = True

    elif candidate == CAR.BUICK_LACROSSE:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_ESCALADE:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CADILLAC_ESCALADE_ESV, CAR.CADILLAC_ESCALADE_ESV_2019):
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm

      if candidate == CAR.CADILLAC_ESCALADE_ESV:
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
        ret.lateralTuning.pid.kf = 0.000045
      else:
        ret.steerActuatorDelay = 0.2
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_BOLT_EUV:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_SILVERADO:
      # On the Bolt, the ECM and camera independently check that you are either above 5 kph or at a stop
      # with foot on brake to allow engagement, but this platform only has that check in the camera.
      # TODO: check if this is split by EV/ICE with more platforms in the future
      if ret.openpilotLongitudinalControl:
        ret.minEnableSpeed = -1.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_TRAILBLAZER:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_XT6:
      ret.steerActuatorDelay = 0.2
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_XT4:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.minSteerSpeed = 30 * CV.MPH_TO_MS
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif candidate == CAR.CADILLAC_CT6_2019:
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.minEnableSpeed = -1
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_VOLT_2019:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_TRAX:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.startingState = True
    elif candidate == CAR.CHEVROLET_TRAVERSE:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_YUKON:
      ret.steerActuatorDelay = 0.5
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.dashcamOnly = True  # Needs steerRatio, tireStiffness, and lat accel factor tuning

    elif candidate == CAR.BUICK_BABYENCLAVE:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)


    if ACCELERATOR_POS_MSG not in fingerprint[CanBus.POWERTRAIN]:
      ret.flags |= GMFlags.NO_ACCELERATOR_POS_MSG.value
    return ret

  ## GM autoHold
  def update_auto_hold(self):
    self.CS.autoHoldActivated = False
    self.CS.out.autoHoldActivated = False

    if not self.CS.autoHold:
      self.CS.autoHoldActive = False
      return

    # 가스페달, 리제패들 해제
    if self.CS.out.gasPressed or self.CS.out.regenBraking:
      self.CS.autoHoldActive = False
      return

    # 이미 AutoHold 중이면 유지
    if self.CS.autoHoldActive:
      self.CS.autoHoldActivated = True
      self.CS.out.autoHoldActivated = True
      return

    # 브레이크 압력 기준
    if self.CP.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value:
      brake_hold_pressed = self.CS.out.brakePressed and self.CS.out.brake >= 0.04
    else:
      brake_hold_pressed = self.CS.out.brakePressed and self.CS.out.brake >= 8

    # 운전자 브레이크로 충분히 정지했을 때만 AutoHold 진입
    if self.CS.out.vEgo < 0.05 and brake_hold_pressed:
      self.CS.autoHoldActive = True
      self.CS.autoHoldActivated = True
      self.CS.out.autoHoldActivated = True
      return

  def apply(self, c, now_nanos, MD=None):
    self.CS.MD = MD
    self.update_auto_hold()
    can_sends = self.CC.update(c, self.CS, now_nanos)
    return can_sends
