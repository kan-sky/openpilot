"""Receive-only Traverse stock camera lane classification."""

import time

TRAVERSE_LANE_ADDRESS = 0x352  # F_Vision_Environment_3
TRAVERSE_LANE_BUS = 1
STOCK_LANE_MAX_AGE_NS = 250_000_000
UNKNOWN_LANES = (-1, -1)


def _lane_code(raw: int) -> int:
  return 1 if raw == 1 else 0 if raw == 2 else -1


def lane_boottime_ns() -> int:
  # Match pandad's nanos_since_boot(), including time spent suspended.
  clock = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)
  return time.clock_gettime_ns(clock)


class TraverseStockLaneMarkings:
  def __init__(self):
    self.sample_ns = -1
    self.codes = UNKNOWN_LANES

  def update(self, can_packets, now_ns: int) -> tuple[int, int]:
    for sample_ns, frames in can_packets:
      if not self.sample_ns <= sample_ns <= now_ns or now_ns - sample_ns > STOCK_LANE_MAX_AGE_NS:
        continue
      for address, data, bus in frames:
        if address == TRAVERSE_LANE_ADDRESS and bus == TRAVERSE_LANE_BUS and len(data) == 8:
          marking = data[6]
          self.codes = (_lane_code((marking >> 3) & 7), _lane_code(marking & 7))
          self.sample_ns = sample_ns
    return self.codes if 0 <= now_ns - self.sample_ns <= STOCK_LANE_MAX_AGE_NS else UNKNOWN_LANES

  def publisher_sample_ns(self, input_now_ns: int, publisher_now_ns: int) -> int:
    age = input_now_ns - self.sample_ns
    if self.sample_ns > 0 and 0 <= age <= STOCK_LANE_MAX_AGE_NS:
      return max(0, publisher_now_ns - age)
    return 0


def stock_lane_codes(car_state) -> tuple[int, int] | None:
  """None is legacy/unsupported; a supported car never falls back to ONNX."""
  markings = getattr(car_state, "stockLaneMarkings", None)
  if getattr(markings, "supported", False):
    return markings.left, markings.right
  return None


def stock_lane_sample_fresh(car_state, now_ns: int | None = None) -> bool:
  markings = getattr(car_state, "stockLaneMarkings", None)
  sample_ns = getattr(markings, "sampleMonoTime", 0)
  if sample_ns <= 0:
    return False
  if now_ns is None:
    now_ns = lane_boottime_ns()
  return 0 <= now_ns - sample_ns <= STOCK_LANE_MAX_AGE_NS
