"""Receive-only Traverse outside air temperature for display.

ECM 0x3FC byte4 is the filtered value the cluster shows (raw * 0.5 - 40 C): it can
fall at any time but only rises while moving. byte7 mirrors byte4 on every observed
frame, and the radar-bus 0x303 outside-air value matched it on 8576/8596 frames.
"""

TRAVERSE_OAT_ADDRESS = 0x3FC
TRAVERSE_OAT_BUS = 0
OAT_MAX_AGE_NS = 2_000_000_000  # 4 Hz message; tolerate a few missed frames
OAT_MIN_C = -40.0
OAT_MAX_C = 85.0


def decode_outside_air_temp(data: bytes) -> float | None:
  if len(data) != 8 or data[4] != data[7]:
    return None
  celsius = data[4] * 0.5 - 40.0
  return celsius if OAT_MIN_C <= celsius <= OAT_MAX_C else None


class TraverseOutsideAirTemp:
  def __init__(self):
    self.sample_ns = -1
    self.celsius: float | None = None

  def update(self, can_packets, now_ns: int) -> float | None:
    for sample_ns, frames in can_packets:
      if not self.sample_ns <= sample_ns <= now_ns or now_ns - sample_ns > OAT_MAX_AGE_NS:
        continue
      for address, data, bus in frames:
        if address == TRAVERSE_OAT_ADDRESS and bus == TRAVERSE_OAT_BUS:
          celsius = decode_outside_air_temp(data)
          if celsius is not None:
            self.celsius = celsius
            self.sample_ns = sample_ns
    if self.celsius is not None and 0 <= now_ns - self.sample_ns <= OAT_MAX_AGE_NS:
      return self.celsius
    return None


def outside_air_temp_c(car_state) -> float | None:
  """Valid display temperature from carState, or None (unsupported, stale, or invalid)."""
  oat = getattr(car_state, "outsideAirTemp", None)
  if not getattr(oat, "valid", False):
    return None
  try:
    celsius = float(getattr(oat, "celsius", None))
  except (TypeError, ValueError):
    return None
  return celsius if OAT_MIN_C <= celsius <= OAT_MAX_C else None
