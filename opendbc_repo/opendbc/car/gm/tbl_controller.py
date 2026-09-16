"""Chevrolet Trailblazer (2021-22) camera-longitudinal helpers.

The Trailblazer's stock camera keeps sending its own ACC state (0x2CB/0x370)
alongside openpilot's replacement commands, uses a checksum on 0x2CB that
differs from every other GM platform, and faults if engaged below 5 km/h.
This module isolates all of that car-specific plumbing so gmcan.py stays a
generic CAN-message builder shared by every GM car, and carstate.py/
carcontroller.py only call into a small, testable interface.
"""

import copy

from opendbc.car import structs
from opendbc.car.gm.values import CAR

NetworkLocation = structs.CarParams.NetworkLocation
ButtonType = structs.CarState.ButtonEvent.Type


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


def update_camera_longitudinal_state(CS, CP, cam_cp):
  """Track the Trailblazer's own stock 0x2CB/0x370 camera-long state. Mutates CS in place."""
  CS.cam_ascm_2cb_counter_updated = False
  CS.cam_stock_long_cancel = False
  if not is_trailblazer_camera_longitudinal(CP):
    return

  # The stock 0x2CB counter lags ASCM_2CD by one cycle during some cold starts
  # and aligns with it after ACC activates. Follow 0x2CB itself instead.
  counters = cam_cp.vl_all["ASCMGasRegenCmd"]["RollingCounter"]
  active_states = cam_cp.vl_all["ASCMGasRegenCmd"]["GasRegenCmdActive"]
  CS.cam_ascm_2cb_counter_updated = len(counters) > 0 and len(active_states) > 0
  if CS.cam_ascm_2cb_counter_updated:
    previous_stock_long_active = CS.cam_stock_long_active
    CS.cam_ascm_2cb_counter = int(counters[-1])
    CS.cam_ascm_2cb_counter_ts_nanos = cam_cp.ts_nanos["ASCMGasRegenCmd"]["RollingCounter"]
    CS.cam_stock_long_active = bool(active_states[-1])
    CS.cam_stock_long_cancel = previous_stock_long_active is True and not CS.cam_stock_long_active

  # Keep the camera's complete ACC state as the template for the replacement
  # 0x370. This Trailblazer generation expects ACCCruiseState and the two
  # constant bits to retain the stock values (2, 0, 0).
  if len(cam_cp.vl_all["ASCMActiveCruiseControlStatus"]["ACCCruiseState"]) > 0:
    CS.cam_acc_status = copy.copy(cam_cp.vl["ASCMActiveCruiseControlStatus"])


def create_stock_long_cancel_button_events() -> list:
  """Create an atomic synthetic cancel click for a stock ACC authority loss."""
  # VCruiseCarrot tracks button duration from matching press/release edges. A
  # press-only synthetic cancel latches that tracker forever, is eventually
  # interpreted as a long press, and prevents later SET/RES button handling.
  # Keep both edges in one carState message so they cannot be lost separately.
  return [
    structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel),
    structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel),
  ]


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


def compute_gas_regen_checksum(car_fingerprint, dat, idx):
  """0x2CB (ASCMGasRegenCmd) checksum, minus the GasRegenCmdActive high bit."""
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER:
    # Captured 2021-22 Trailblazer frames use one 24-bit subtraction, with
    # carry/borrow across bytes. Other GM platforms retain the established
    # byte-wise checksum until their stock frames prove the same requirement.
    return (0x1000000 - int.from_bytes(dat[1:4], "big") - idx) & 0xFFFFFF
  return (((0xff - dat[1]) & 0xff) << 16) | \
         (((0xff - dat[2]) & 0xff) << 8) | \
         ((0x100 - dat[3] - idx) & 0xff)
