import copy
from opendbc.can import CANDefine, CANParser
from openpilot.cereal import car
from openpilot.common.params import Params #kans
import numpy as np
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gm.values import DBC, AccState, CruiseButtons, STEER_THRESHOLD, CAR, GMFlags, CAMERA_ACC_CAR, EV_CAR, SDGM_CAR, ALT_ACCS
import openpilot.cereal.messaging as messaging
import time

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation
GearShifter = structs.CarState.GearShifter
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.mainCruise, CruiseButtons.CANCEL: ButtonType.cancel,
                CruiseButtons.GAP_DIST: ButtonType.gapAdjustCruise}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.buttons_counter = 0
    self.single_pedal_mode = False
    self.pedal_steady = 0.
    self.cruise_buttons = 0
    # GAP_DIST
    self.distance_button = 0
    # Kans: ambient temperature (°C)
    self.ambient_c = 0.0
    # Kans: lead_car condition
    self.lead_status = False
    self.lead_vrel =  0.0
    self.lead_distance = float('inf')
    self.lead_speed = 0.0
    self.lead_accel = 0.0
    self.sm = messaging.SubMaster(['radarState', 'deviceState'])

    self.cruiseMain_on = True if Params().get_int("AutoEngage") == 2 else False
    # Kans:
    self.use_alpha_long = False
    self._dbg_op_enable_time = None
    self._dbg_op_printed = False
    self._cruise_off_cnt = 0

    # Kans: autoHold
    self.autoHold = True
    self.autoHoldActive = False
    self.autoHoldActivated = False

    # Kans: accFault delay
    self.startup_time = time.monotonic()

    # Kans: TPMS
    self.KPA_TO_PSI = 0.1450377377
    self.TPMS_GAIN = 1.125
    self.TPMS_OFFSET = 0.5

  def kpa_to_psi(self, kpa_g: float) -> float:
    return float(kpa_g) * self.KPA_TO_PSI

  def psi_display(self, psi_raw: float) -> float:
    return self.TPMS_GAIN * psi_raw - self.TPMS_OFFSET

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # The ECM allows enabling on falling edge of set, but only rising edge of resume
        if (b.type == ButtonType.accelCruise and b.pressed) or \
          (b.type == ButtonType.decelCruise and not b.pressed):
          return True
    return False

  def update(self, can_parsers) -> structs.CarState:
    ret = super().update(can_parsers)
    # Kans: lead_car condition (vision/fusion lead)
    try:
      self.sm.update(0)
    except Exception:
      pass

    lead = None
    try:
      rs = self.sm['radarState']
      lead = getattr(rs, 'leadOne', None)
    except Exception:
      lead = None

    if lead is not None:
      st = bool(getattr(lead, 'status', 0))
      self.lead_status = st

      if st:
        drel = getattr(lead, 'dRel', None)
        vrel = getattr(lead, 'vRel', None)
        arel = getattr(lead, 'aRel', None)

        vlead = getattr(lead, 'vLead', None)
        alead = getattr(lead, 'aLead', None)

        self.lead_distance = float(drel) if drel is not None else float('inf')
        self.lead_vrel = float(vrel) if vrel is not None else 0.0
        self.lead_speed = float(vlead) if vlead is not None else 0.0
        if arel is not None:
          self.lead_accel = float(arel)
        elif alead is not None:
          self.lead_accel = float(alead)
        else:
          self.lead_accel = 0.0
      else:
        self.lead_distance = float('inf')
        self.lead_vrel = 0.0
        self.lead_speed = 0.0
        self.lead_accel = 0.0
    else:
      self.lead_status = False
      self.lead_distance = float('inf')
      self.lead_vrel = 0.0
      self.lead_speed = 0.0
      self.lead_accel = 0.0

    # ambient temperature 업데이트
    if self.sm.updated.get('deviceState', False):
      ds = self.sm['deviceState']
      amb = getattr(ds, 'ambientTempCDEPRECATED', None)
      if amb is not None:
        self.ambient_c = float(amb)

    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    loopback_cp = can_parsers[Bus.loopback]

    ret = structs.CarState()

    prev_cruise_buttons = self.cruise_buttons
    prev_distance_button = self.distance_button
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]

    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    # GAP_DIST
    if self.cruise_buttons in [CruiseButtons.UNPRESS, CruiseButtons.INIT] and self.distance_button:
      self.cruise_buttons = CruiseButtons.GAP_DIST

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    # This is to avoid a fault where you engage while still moving backwards after shifting to D.
    # An Equinox has been seen with an unsupported status (3), so only check if either wheel is in reverse (2)
    left_whl_sign = -1 if pt_cp.vl["EBCMWheelSpdRear"]["RLWheelDir"] == 2 else 1
    right_whl_sign = -1 if pt_cp.vl["EBCMWheelSpdRear"]["RRWheelDir"] == 2 else 1
    self.parse_wheel_speeds(ret,
      left_whl_sign * pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      right_whl_sign * pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      left_whl_sign * pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      right_whl_sign * pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    # sample rear wheel speeds to match the safety which only uses the rear CAN message
    # standstill=True if ECM allows engagement with brake
    ret.standstill = abs(pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"]) <= STANDSTILL_THRESHOLD and \
                     abs(pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"]) <= STANDSTILL_THRESHOLD

    if pt_cp.vl["ECMPRDNL2"]["ManualMode"] == 1:
      ret.gearShifter = self.parse_gear_shifter("T")
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None))

    if self.CP.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value:
      ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    else:
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["BrakePressed"] != 0
    else:
      ret.brakePressed = ret.brake >= 8

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      self.single_pedal_mode = ret.gearShifter == GearShifter.low or pt_cp.vl["EVDriveMode"]["SinglePedalModeActive"] == 1 or (ret.regenBraking and GearShifter.manumatic) or (self.CP.carFingerprint in [CAR.CHEVROLET_BOLT_EUV, ] and self.CP.enableGasInterceptorDEPRECATED)

    if self.CP.enableGasInterceptorDEPRECATED:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      threshold = 20 if self.CP.carFingerprint in CAMERA_ACC_CAR else 4
      ret.gasPressed = ret.gas > threshold
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 0  # 1e-5

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

    ret.parkingBrake = pt_cp.vl["BCMGeneralPlatformStatus"]["ParkBrakeSwActive"] == 1
    # Kans:
    ecu_cruise_main = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    ret.cruiseState.available = ecu_cruise_main
    self.cruiseMain_on = ret.cruiseState.available
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1

    # Kans: 부팅초기 레이더/ACC 웜업 중 Cruise FAULT 무시
    cruise_faulted = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.FAULTED
    friction_brake_unavailable = pt_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakeUnavailable"] == 1
    startup_fault_ignore = (time.monotonic() - self.startup_time) < 60.0
    ret.accFaulted = ((cruise_faulted and not startup_fault_ignore) or friction_brake_unavailable)
    if self.CP.carFingerprint in CAR.CHEVROLET_TRAILBLAZER:
      ret.accFaulted = False

    ret.cruiseState.enabled = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] != AccState.OFF
    ret.cruiseState.standstill = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.STANDSTILL
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      if self.CP.carFingerprint not in ALT_ACCS:
        ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
        # This FCW signal only works for SDGM cars. CAM cars send FCW on GMLAN but this bit is always 0 for them
        ret.stockFcw = cam_cp.vl["ASCMActiveCruiseControlStatus"]["FCWAlert"] != 0
        if self.CP.pcmCruise:
          # openpilot controls nonAdaptive when not pcmCruise
          ret.cruiseState.nonAdaptive = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCruiseState"] not in (2, 3)
      else:
        ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
        ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0

    if self.CP.enableBsm:
      ret.leftBlindspot = pt_cp.vl["BCMBlindSpotMonitor"]["LeftBSM"] == 1
      ret.rightBlindspot = pt_cp.vl["BCMBlindSpotMonitor"]["RightBSM"] == 1

    prev_lkas_enabled = self.lkas_enabled
    self.lkas_enabled = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    # Kans: accStatus
    acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]
    self.pcm_acc_status = acc_status
    ret.accStatus = int(acc_status)
    # Kans: CruiseState OFF flicker filter
    #if acc_status == AccState.OFF:
    #  self._cruise_off_cnt = min(self._cruise_off_cnt + 1, 255)
    #else:
    #  self._cruise_off_cnt = 0

    # OFF가 3프레임(=30ms) 연속일 때만 진짜 OFF로 인정
    #ret.cruiseState.enabled = (self._cruise_off_cnt < 3)

    ret.vCluRatio = 1.0 if self.CP.carFingerprint in EV_CAR else 0.96

    # Kans: alpha long(SDGM)
    if self.CP.carFingerprint in SDGM_CAR:
      alpha_long_avail = cam_cp.vl["SDGM_ALPHA_LONG"]["AlphaLongAvailable"] == 1
      self.use_alpha_long = bool(alpha_long_avail)

    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.cruise_buttons != CruiseButtons.UNPRESS or prev_cruise_buttons != CruiseButtons.INIT:
      ret.buttonEvents = [
        *create_button_events(self.cruise_buttons, prev_cruise_buttons, BUTTONS_DICT,
                              unpressed_btn=CruiseButtons.UNPRESS),
        *create_button_events(self.distance_button, prev_distance_button,
                              {1: ButtonType.gapAdjustCruise}),
        *create_button_events(self.lkas_enabled, prev_lkas_enabled,
                              {1: ButtonType.lkas})
      ]

    if ret.vEgo < self.CP.minSteerSpeed:
      ret.lowSpeedAlert = True

    # Kans: autoHold
    self.autoHold = True
    ret.autoHoldActivated = self.autoHoldActivated

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      pt_messages += [
        ("ASCMLKASteeringCmd", float('nan')),
      ]
    if CP.transmissionType == TransmissionType.direct:
      pt_messages += [
        ("EBCMRegenPaddle", 50),
        ("EVDriveMode", float('nan')),
      ]
    cam_messages = []
    if CP.carFingerprint in SDGM_CAR:
      cam_messages += [
        ("SDGM_ALPHA_LONG", float('nan')),
      ]
    loopback_messages = [
      ("ASCMLKASteeringCmd", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], loopback_messages, 128),
    }

