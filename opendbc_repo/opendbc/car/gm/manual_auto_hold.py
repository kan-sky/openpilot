"""Disengaged Traverse pedal hold, independent of ACC stopping and resume."""
import math

from opendbc.car import structs
from opendbc.car.gm.values import CAR, GMSafetyFlags

ARM_BRAKE = 80  # ECMAcceleratorPos.BrakePedalPos raw units, not percent.
ARM_SPEED = 1.5  # m/s; arming is not a brake command.
HOLD_BRAKE = 400
HOLD_SPEED = 0.05


def manual_hold_supported(CP):
  required = GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.HW_SDGM | GMSafetyFlags.MANUAL_AUTO_HOLD
  incompatible = (GMSafetyFlags.CC_LONG | GMSafetyFlags.NO_ACC | GMSafetyFlags.PEDAL_LONG |
                  GMSafetyFlags.GAS_INTERCEPTOR | GMSafetyFlags.HW_ASCM_LONG | GMSafetyFlags.EV)
  return (CP.carFingerprint == CAR.CHEVROLET_TRAVERSE and CP.openpilotLongitudinalControl and
          not CP.enableGasInterceptorDEPRECATED and bool(CP.safetyConfigs) and
          (CP.safetyConfigs[0].safetyParam & required) == required and
          not CP.safetyConfigs[0].safetyParam & incompatible)


def eligible(cs, controls_active):
  return (not controls_active and not cs.cruiseState.enabled and cs.canValid and not cs.canTimeout and
          not cs.accFaulted and not cs.gasPressed and not cs.regenBraking and not cs.parkingBrake and
          cs.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low) and
          math.isfinite(cs.vEgo) and math.isfinite(cs.vEgoRaw))


def stopped(cs):
  return cs.standstill and abs(cs.vEgo) < HOLD_SPEED and abs(cs.vEgoRaw) < HOLD_SPEED


class ManualAutoHold:
  def __init__(self):
    self.reset()

  def reset(self):
    self.armed = False
    self.active = False
    self.pedal_released = False
    self.press_peak = 0.

  def update(self, cs, pedal, controls_active):
    if (not eligible(cs, controls_active) or not math.isfinite(pedal) or not 0 <= pedal <= 255 or
        max(abs(cs.vEgo), abs(cs.vEgoRaw)) > ARM_SPEED or (self.active and not stopped(cs))):
      self.reset()
      return

    if cs.brakePressed and pedal >= ARM_BRAKE:
      self.armed = True
    elif not self.active and not cs.brakePressed:
      self.armed = False
    if not self.active and self.armed and cs.brakePressed and stopped(cs):
      self.active = True
      self.pedal_released = False
      self.press_peak = 0.

    if self.active:
      if cs.brakePressed:
        if self.pedal_released:
          self.press_peak = max(self.press_peak, pedal)
      else:
        if self.pedal_released and 0 < self.press_peak < ARM_BRAKE:
          self.reset()
          return
        self.pedal_released = True
        self.press_peak = 0.

  def command(self, cs, controls_active):
    return HOLD_BRAKE if self.active and eligible(cs, controls_active) and stopped(cs) else 0
