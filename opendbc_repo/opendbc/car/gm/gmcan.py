from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.gm.values import CAR, CruiseButtons, CanBus
from opendbc.car.common.conversions import Conversions as CV

NetworkLocation = structs.CarParams.NetworkLocation


def is_trailblazer_camera_longitudinal(CP):
  return (CP.openpilotLongitudinalControl and
          CP.carFingerprint == CAR.CHEVROLET_TRAILBLAZER and
          CP.networkLocation == NetworkLocation.fwdCamera)


def get_longitudinal_sync_messages(CP):
  if is_trailblazer_camera_longitudinal(CP):
    # These are synchronization references, not platform-wide CAN validity
    # requirements. A slow camera startup must not invalidate the whole car.
    return [("ASCMGasRegenCmd", float('nan')), ("ASCMActiveCruiseControlStatus", float('nan'))]
  return []


def get_longitudinal_command_timing(CP, CS, frame):
  if is_trailblazer_camera_longitudinal(CP):
    # The stock command counter can lag ASCM_2CD by one cycle during a cold
    # start, then align with it after ACC becomes active. Follow the actual
    # stock 0x2CB command so both phases are handled without guessing.
    stock_references_ready = (
      CS.cam_ascm_2cb_counter_ts_nanos != 0 and
      CS.cam_stock_long_active is not None and
      CS.cam_acc_status is not None
    )
    if not stock_references_ready:
      return False, 0
    return CS.cam_ascm_2cb_counter_updated, CS.cam_ascm_2cb_counter
  return frame % 4 == 0, (frame // 4) % 4


def apply_driver_gas_override(car_fingerprint, gas_pressed, inactive_regen, apply_gas, apply_brake,
                              at_full_stop, near_stop):
  # The 2021-22 Trailblazer can sample the accelerator before controls has
  # cleared longActive. Emit a complete inactive command set during that
  # transition so Panda does not drop a counter-matched 0x2CB/0x315 pair.
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER and gas_pressed:
    return inactive_regen, 0, False, False
  return apply_gas, apply_brake, at_full_stop, near_stop


def apply_stock_longitudinal_gate(car_fingerprint, stock_long_active, inactive_regen, apply_gas, apply_brake,
                                  at_full_stop, near_stop, acc_engaged):
  # The Trailblazer's camera can revoke longitudinal authority before the ECM
  # cruise state changes. Stop actuation on that same stock command cycle so
  # the EBCM never sees an active replacement after the camera has gone idle.
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER and stock_long_active is not True:
    return inactive_regen, 0, False, False, False
  return apply_gas, apply_brake, at_full_stop, near_stop, acc_engaged


def get_acc_dashboard_enabled(car_fingerprint, enabled, in_drive, stock_long_active, stock_acc_status):
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER:
    return (enabled and in_drive and stock_long_active is True and stock_acc_status is not None and
            bool(stock_acc_status["ACCCmdActive"]))
  return enabled


# GM: AutoResume: brake signal to CAN
def create_brake_command(packer, bus, apply_brake, idx):
  mode = 0xA if apply_brake > 0 else 0x1
  brake = (0x1000 - apply_brake) & 0xFFF
  checksum = (0x10000 - (mode << 12) - brake - idx) & 0xFFFF

  values = {
    "RollingCounter": idx,
    "FrictionBrakeMode": mode,
    "FrictionBrakeChecksum": checksum,
    "FrictionBrakeCmd": -apply_brake
  }

  return packer.make_can_msg("EBCMFrictionBrakeCmd", bus, values)

def create_buttons(packer, bus, idx, button):
  values = {
    "ACCButtons": button,
    "RollingCounter": idx,
    "ACCAlwaysOne": 1,
    "DistanceButton": 0,
  }

  checksum = 240 + int(values["ACCAlwaysOne"] * 0xf)
  checksum += values["RollingCounter"] * (0x4ef if values["ACCAlwaysOne"] != 0 else 0x3f0)
  checksum -= int(values["ACCButtons"] - 1) << 4  # not correct if value is 0
  checksum -= 2 * values["DistanceButton"]

  values["SteeringButtonChecksum"] = checksum
  return packer.make_can_msg("ASCMSteeringButton", bus, values)


def create_pscm_status(packer, bus, pscm_status):
  values = {s: pscm_status[s] for s in [
    "HandsOffSWDetectionMode",
    "HandsOffSWlDetectionStatus",
    "LKATorqueDeliveredStatus",
    "LKADriverAppldTrq",
    "LKATorqueDelivered",
    "LKATotalTorqueDelivered",
    "RollingCounter",
    "PSCMStatusChecksum",
  ]}
  checksum_mod = int(1 - values["HandsOffSWlDetectionStatus"]) << 5
  values["HandsOffSWlDetectionStatus"] = 1
  values["PSCMStatusChecksum"] += checksum_mod
  return packer.make_can_msg("PSCMStatus", bus, values)


def create_steering_control(packer, bus, apply_torque, idx, lkas_active):
  values = {
    "LKASteeringCmdActive": lkas_active,
    "LKASteeringCmd": apply_torque,
    "RollingCounter": idx,
    "LKASteeringCmdChecksum": 0x1000 - (lkas_active << 11) - (apply_torque & 0x7ff) - idx
  }

  return packer.make_can_msg("ASCMLKASteeringCmd", bus, values)


def create_adas_keepalive(bus):
  dat = b"\x00\x00\x00\x00\x00\x00\x00"
  return [CanData(0x409, dat, bus), CanData(0x40a, dat, bus)]


def create_gas_regen_command(packer, bus, throttle, idx, enabled, at_full_stop, car_fingerprint=None):
  """Create 0x2CB, selecting the stock-verified checksum only for Trailblazer."""
  values = {
    "GasRegenCmdActive": enabled,
    "RollingCounter": idx,
    "GasRegenCmd": throttle,
    "GasRegenFullStopActive": at_full_stop,
    "GasRegenAccType": 1,
  }

  dat = packer.make_can_msg("ASCMGasRegenCmd", bus, values)[1]
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER:
    # Captured 2021-22 Trailblazer frames use one 24-bit subtraction, with
    # carry/borrow across bytes. Other GM platforms retain the established
    # byte-wise checksum until their stock frames prove the same requirement.
    checksum = (0x1000000 - int.from_bytes(dat[1:4], "big") - idx) & 0xFFFFFF
  else:
    checksum = (((0xff - dat[1]) & 0xff) << 16) | \
               (((0xff - dat[2]) & 0xff) << 8) | \
               ((0x100 - dat[3] - idx) & 0xff)
  values["GasRegenChecksum"] = ((1 - enabled) << 24) | checksum

  return packer.make_can_msg("ASCMGasRegenCmd", bus, values)


def create_friction_brake_command(packer, bus, apply_brake, idx, enabled, near_stop, at_full_stop, CP):
  mode = 0x1

  # TODO: Understand this better. Volts and ICE Camera ACC cars are 0x1 when enabled with no brake
  if enabled and CP.carFingerprint in (CAR.CHEVROLET_BOLT_EUV,):
    mode = 0x9

  if apply_brake > 0:
    mode = 0xa
    if at_full_stop:
      mode = 0xd

    # TODO: this is to have GM bringing the car to complete stop,
    # but currently it conflicts with OP controls, so turned off. Not set by all cars
    #elif near_stop:
    #  mode = 0xb

  apply_brake = max(0, min(0xFFF, apply_brake))
  brake = (0x1000 - apply_brake) & 0xfff
  checksum = (0x10000 - (mode << 12) - brake - idx) & 0xffff

  values = {
    "RollingCounter": idx,
    "FrictionBrakeMode": mode,
    "FrictionBrakeChecksum": checksum,
    "FrictionBrakeCmd": (0x1000 - apply_brake) & 0xfff,
  }

  return packer.make_can_msg("EBCMFrictionBrakeCmd", bus, values)


def create_acc_dashboard_command(packer, bus, enabled, target_speed_kph, hud_control, fcw, stock_acc_status=None):
  target_speed = min(target_speed_kph, 255)

  if stock_acc_status is not None:
    values = dict(stock_acc_status)

    # When longitudinal control is inactive, forward the exact stock state.
    # In particular, do not replace the Trailblazer's valid (2, 0, 0)
    # ACCCruiseState/constant-bit tuple with openpilot's generic (0, 1, 1).
    if not enabled:
      return packer.make_can_msg("ASCMActiveCruiseControlStatus", bus, values)
  else:
    values = {
      "ACCAlwaysOne": 1,
      "ACCResumeButton": 0,
      "ACCAlwaysOne2": 1,
    }

  # Preserve the stock protocol state while replacing only the fields needed
  # for openpilot's active longitudinal-control display.
  values.update({
    "ACCSpeedSetpoint": target_speed,
    "ACCGapLevel": hud_control.leadDistanceBars * enabled,  # 3 "far", 0 "inactive"
    "ACCCmdActive": enabled,
    "ACCLeadCar": hud_control.leadVisible,
    "FCWAlert": 0x3 if fcw else 0,
  })

  return packer.make_can_msg("ASCMActiveCruiseControlStatus", bus, values)


def create_adas_time_status(bus, tt, idx):
  dat = [(tt >> 20) & 0xff, (tt >> 12) & 0xff, (tt >> 4) & 0xff,
         ((tt & 0xf) << 4) + (idx << 2)]
  chksum = 0x1000 - dat[0] - dat[1] - dat[2] - dat[3]
  chksum = chksum & 0xfff
  dat += [0x40 + (chksum >> 8), chksum & 0xff, 0x12]
  return CanData(0xa1, bytes(dat), bus)


def create_adas_steering_status(bus, idx):
  dat = [idx << 6, 0xf0, 0x20, 0, 0, 0]
  chksum = 0x60 + sum(dat)
  dat += [chksum >> 8, chksum & 0xff]
  return CanData(0x306, bytes(dat), bus)


def create_adas_accelerometer_speed_status(bus, speed_ms, idx):
  spd = int(speed_ms * 16) & 0xfff
  accel = 0 & 0xfff
  # 0 if in park/neutral, 0x10 if in reverse, 0x08 for D/L
  #stick = 0x08
  near_range_cutoff = 0x27
  near_range_mode = 1 if spd <= near_range_cutoff else 0
  far_range_mode = 1 - near_range_mode
  dat = [0x08, spd >> 4, ((spd & 0xf) << 4) | (accel >> 8), accel & 0xff, 0]
  chksum = 0x62 + far_range_mode + (idx << 2) + dat[0] + dat[1] + dat[2] + dat[3] + dat[4]
  dat += [(idx << 5) + (far_range_mode << 4) + (near_range_mode << 3) + (chksum >> 8), chksum & 0xff]
  return CanData(0x308, bytes(dat), bus)


def create_adas_headlights_status(packer, bus):
  values = {
    "Always42": 0x42,
    "Always4": 0x4,
  }
  return packer.make_can_msg("ASCMHeadlight", bus, values)


def create_lka_icon_command(bus, active, critical, steer):
  if active and steer == 1:
    if critical:
      dat = b"\x50\xc0\x14"
    else:
      dat = b"\x50\x40\x18"
  elif active:
    if critical:
      dat = b"\x40\xc0\x14"
    else:
      dat = b"\x40\x40\x18"
  else:
    dat = b"\x00\x00\x00"
  return CanData(0x104c006c, dat, bus)

def create_regen_paddle_command(packer, bus):
  values = {
    "RegenPaddle": 0x20, #이 값은 패들의 강도일 가능성이 있음.
  }
  return packer.make_can_msg("EBCMRegenPaddle", bus, values)

def create_gm_cc_spam_command(packer, controller, CS, actuators):
  if controller.params_.get_bool("IsMetric"):
    _CV = CV.MS_TO_KPH
    RATE_UP_MAX = 0.04
    RATE_DOWN_MAX = 0.04
  else:
    _CV = CV.MS_TO_MPH
    RATE_UP_MAX = 0.2
    RATE_DOWN_MAX = 0.2

  accel = actuators.accel * _CV  # m/s/s to mph/s
  speedSetPoint = int(round(CS.out.cruiseState.speed * _CV))

  cruiseBtn = CruiseButtons.INIT
  if speedSetPoint == CS.CP.minEnableSpeed and accel < -1:
    cruiseBtn = CruiseButtons.CANCEL
    controller.apply_speed = 0
    rate = 0.04
  elif accel < 0:
    cruiseBtn = CruiseButtons.DECEL_SET
    if speedSetPoint > (CS.out.vEgo * _CV) + 3.0:  # If accel is changing directions, bring set speed to current speed as fast as possible
      rate = RATE_DOWN_MAX
    else:
      rate = max(-1 / accel, RATE_DOWN_MAX)
    controller.apply_speed = speedSetPoint - 1
  elif accel > 0:
    cruiseBtn = CruiseButtons.RES_ACCEL
    if speedSetPoint < (CS.out.vEgo * _CV) - 3.0:
      rate = RATE_UP_MAX
    else:
      rate = max(1 / accel, RATE_UP_MAX)
    controller.apply_speed = speedSetPoint + 1
  else:
    controller.apply_speed = speedSetPoint
    rate = float('inf')

  # Check rlogs closely - our message shouldn't show up on the pt bus for us
  # Or bus 2, since we're forwarding... but I think it does
  if (cruiseBtn != CruiseButtons.INIT) and ((controller.frame - controller.last_button_frame) * DT_CTRL > rate):
    controller.last_button_frame = controller.frame
    idx = (CS.buttons_counter + 1) % 4  # Need to predict the next idx for '22-23 EUV
    return [create_buttons(packer, CanBus.POWERTRAIN, idx, cruiseBtn)]
  else:
    return []
