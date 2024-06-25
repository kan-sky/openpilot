import copy
from cereal import car

from common.conversions import Conversions as CV
from common.numpy_fast import mean
from common.params import Params #kans
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import CAR, DBC, AccState, CanBus, STEER_THRESHOLD, CC_ONLY_CAR, CAMERA_ACC_CAR, CruiseButtons
from common.realtime import DT_CTRL

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
GearShifter = car.CarState.GearShifter
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS
LongCtrlState = car.CarControl.Actuators.LongControlState # kans


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.is_metric = False

    # GAP_DIST
    self.prev_distance_button = False
    self.distance_button_pressed = False

    # kans: steer이벤트(일시불가) 줄이기 위해
    self.belowSteerSpeed_shown = False
    self.disable_belowSteerSpeed = False
    self.resumeRequired_shown = False
    self.disable_resumeRequired = False

    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0

    # brakeLights
    self.regenPaddlePressed = False
    self.cruiseMain = False

    #Engine Rpm
    self.engineRPM = 0

    self.use_cluster_speed = True # Params().get_bool('UseClusterSpeed')

    self.buttons_counter = 0
    self.single_pedal_mode = False
    self.distance_button_pressed = False

    self.totalDistance = 0.0
    self.accFaultedCount = 0
    #GM <<<
    self.pitch = 0. # radians
    self.pitch_raw = 0. # radians
    self.pitch_ema = 1/100
    self.pitch_future_time = 0.5 # seconds #GM>>>

  def update(self, pt_cp, cam_cp, loopback_cp, chassis_cp):
    ret = car.CarState.new_message()

    self.distance_button_pressed = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"] != 0
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    ret.cruiseButtons = self.cruise_buttons
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]
    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    moving_forward = pt_cp.vl["EBCMWheelSpdRear"]["MovingForward"] != 0
    self.moving_backward = (pt_cp.vl["EBCMWheelSpdRear"]["MovingBackward"] != 0) and not moving_forward
    # GAP_DIST
    if self.cruise_buttons in [CruiseButtons.UNPRESS, CruiseButtons.INIT] and self.distance_button_pressed:
      self.cruise_buttons = CruiseButtons.GAP_DIST

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.rl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    if pt_cp.vl["ECMPRDNL2"]["ManualMode"] == 1:
      ret.gearShifter = self.parse_gear_shifter("T")
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None))

    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      if self.CP.carFingerprint in CC_ONLY_CAR:
        ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    else:
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["BrakePressed"] != 0
    else:
      # Some Volt 2016-17 have loose brake pedal push rod retainers which causes the ECM to believe
      # that the brake is being intermittently pressed without user interaction.
      # To avoid a cruise fault we need to use a conservative brake position threshold
      # https://static.nhtsa.gov/odi/tsbs/2017/MC-10137629-9999.pdf
      ret.brakePressed = ret.brake >= 8

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      self.single_pedal_mode = ret.gearShifter == GearShifter.low or pt_cp.vl["EVDriveMode"]["SinglePedalModeActive"] == 1

    cv_unit = 0.7256
    ret.tpms.fl = cv_unit * pt_cp.vl["TPMS"]["PRESSURE_FL"]
    ret.tpms.fr = cv_unit * pt_cp.vl["TPMS"]["PRESSURE_FR"]
    ret.tpms.rl = cv_unit * pt_cp.vl["TPMS"]["PRESSURE_RL"]
    ret.tpms.rr = cv_unit * pt_cp.vl["TPMS"]["PRESSURE_RR"]

    if self.CP.enableGasInterceptor:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      if self.CP.carFingerprint in (CAR.BOLT_EUV, CAR.BOLT_CC):
        ret.gasPressed = ret.gas > 20
      else:
        threshold = 20 if self.CP.carFingerprint in CAMERA_ACC_CAR else 4
        ret.gasPressed = ret.gas > threshold
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.parkingBrake = pt_cp.vl["VehicleIgnitionAlt"]["ParkBrake"] == 1
    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    # for delay Accfault event
    accFaulted = (pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.FAULTED or \
                      pt_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakeUnavailable"] == 1)
    startingState = LongCtrlState.starting
    self.accFaultedCount = self.accFaultedCount + 1 if accFaulted else 0
    ret.accFaulted = True if self.accFaultedCount > 50 else False
    if self.CP.carFingerprint in CC_ONLY_CAR:
      ret.accFaulted = False
    if self.CP.enableGasInterceptor:  # Flip CC main logic when pedal is being used for long TODO: switch to cancel cc
      ret.cruiseState.available = (not ret.cruiseState.available)
      ret.accFaulted = False
    # for using Accstate
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]

    ret.cruiseState.enabled = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] != AccState.OFF
    ret.cruiseState.standstill = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.STANDSTILL
    if startingState:
      ret.cruiseState.standstill = False
    self.cruiseMain = ret.cruiseState.available
    ret.cruiseMain = self.cruiseMain

    if self.CP.networkLocation == NetworkLocation.fwdCamera and self.CP.carFingerprint not in CC_ONLY_CAR:
      ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
      ret.stockAeb = cam_cp.vl["AEBCmd"]["AEBCmdActive"] != 0
      # openpilot controls nonAdaptive when not pcmCruise
      if self.CP.pcmCruise:
        ret.cruiseState.nonAdaptive = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCruiseState"] not in (2, 3)

    # kans: use cluster speed & vCluRatio(longitudialPlanner)
    self.is_metric = Params().get_bool("IsMetric")
    speed_conv = CV.KPH_TO_MS * 1.609344 if self.is_metric else CV.MPH_TO_MS
    cluSpeed = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"]
    ret.vEgoCluster = cluSpeed * speed_conv
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0
    #GM <<<
    self.pitch = self.pitch_ema * self.pitch_raw + (1 - self.pitch_ema) * self.pitch 
    ret.pitch = self.pitch #GM >>>>>
    # TODO: APILOT
    #Engine Rpm
    self.engineRPM = pt_cp.vl["ECMEngineStatus"]["EngineRPM"]

    # brakeLight
    ret.brakeLights = chassis_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakePressure"] != 0 or ret.brakePressed

    ret.cruiseGap = 1

    self.totalDistance += ret.vEgo * DT_CTRL
    ret.totalDistance = self.totalDistance
    ret.speedLimit = 0
    ret.speedLimitDistance = 0

    if ret.cruiseState.available:
      ret.cruiseState.pcmMode = True
    else:
      ret.cruiseState.pcmMode = False

    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("AEBCmdActive", "AEBCmd"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
      ]
      checks += [
        ("AEBCmd", 10),
        ("ASCMLKASteeringCmd", 10),
      ]
      if CP.carFingerprint not in CC_ONLY_CAR:
        signals += [
          ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
          ("ACCCruiseState", "ASCMActiveCruiseControlStatus"),
        ]
        checks += [
          ("ASCMActiveCruiseControlStatus", 25),
        ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA, enforce_checks=False)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("BrakePedalPos", "ECMAcceleratorPos"),
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("TurnSignals", "BCMTurnSignals"),
      ("AcceleratorPedal2", "AcceleratorPedal2"),
      ("CruiseState", "AcceleratorPedal2"),
      ("ACCButtons", "ASCMSteeringButton"),
      ("RollingCounter", "ASCMSteeringButton"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("MovingForward", "EBCMWheelSpdRear"),
      ("MovingBackward", "EBCMWheelSpdRear"),
      ("FrictionBrakeUnavailable", "EBCMFrictionBrakeStatus"),
      ("PRNDL2", "ECMPRDNL2"),
      ("ManualMode", "ECMPRDNL2"),
      ("LKADriverAppldTrq", "PSCMStatus"),
      ("LKATorqueDelivered", "PSCMStatus"),
      ("LKATorqueDeliveredStatus", "PSCMStatus"),
      ("HandsOffSWlDetectionStatus", "PSCMStatus"),
      ("HandsOffSWDetectionMode", "PSCMStatus"),
      ("LKATotalTorqueDelivered", "PSCMStatus"),
      ("PSCMStatusChecksum", "PSCMStatus"),
      ("RollingCounter", "PSCMStatus"),
      ("TractionControlOn", "ESPStatus"),
      ("ParkBrake", "VehicleIgnitionAlt"),
      ("CruiseMainOn", "ECMEngineStatus"),
      ("BrakePressed", "ECMEngineStatus"),
      ("DistanceButton", "ASCMSteeringButton"),
      ("RollingCounter", "ASCMLKASteeringCmd"),
      ("VehicleSpeed", "ECMVehicleSpeed"),
      ("EngineRPM", "ECMEngineStatus"),
      ("PRESSURE_FL", "TPMS"),
      ("PRESSURE_FR", "TPMS"),
      ("PRESSURE_RL", "TPMS"),
      ("PRESSURE_RR", "TPMS"),
      
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMPRDNL2", 10),
      ("PSCMStatus", 10),
      ("ESPStatus", 10),
      ("BCMDoorBeltStatus", 10),
      ("VehicleIgnitionAlt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("EBCMFrictionBrakeStatus", 20),
      ("AcceleratorPedal2", 33),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("ECMAcceleratorPos", 80),
      ("ECMVehicleSpeed", 20),
      ("TPMS", 0),
    ]

    # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("RollingCounter", "ASCMLKASteeringCmd"),
       ]
      checks += [
        ("ASCMLKASteeringCmd", 0),
      ]

    if CP.transmissionType == TransmissionType.direct:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle"),
        ("SinglePedalModeActive", "EVDriveMode"),
      ]
      checks += [
        ("EBCMRegenPaddle", 50),
        ("EVDriveMode", 0),
      ]


    if CP.carFingerprint in CC_ONLY_CAR:
      signals.remove(("BrakePedalPos", "ECMAcceleratorPos"))
      signals.append(("BrakePedalPosition", "EBCMBrakePedalPosition"))
      checks.remove(("ECMAcceleratorPos", 80))
      checks.append(("EBCMBrakePedalPosition", 100))

    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR"))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR"))
      checks.append(("GAS_SENSOR", 50))
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN, enforce_checks=False)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd"),
    ]

    checks = [
      ("ASCMLKASteeringCmd", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, enforce_checks=False)

  # for brakeLight
  @staticmethod
  def get_chassis_can_parser(CP):
    signals = [
      ("FrictionBrakePressure", "EBCMFrictionBrakeStatus"),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]["chassis"], signals, checks, CanBus.CHASSIS, enforce_checks=False)
