"""Cloud-only scheduling. Never sleeps or changes vehicle/control state."""
import math
import random


def bounded_interval(value, default, minimum, maximum):
  try:
    seconds = float(value)
  except (ValueError, TypeError, OverflowError):
    return default
  return max(minimum, min(maximum, seconds)) if math.isfinite(seconds) else default


class UploadBackoff:
  """Per-channel retry delay, so a failed route upload cannot silence telemetry."""
  def __init__(self, base=30.0, maximum=300.0, jitter=random.random):
    self.base = base
    self.maximum = maximum
    self.jitter = jitter
    self.failures = 0

  def success(self):
    self.failures = 0

  def failure_delay(self):
    delay = min(self.maximum, self.base * (2 ** min(self.failures, 16)))
    self.failures += 1
    # Positive jitter spreads reconnects without retrying earlier than base.
    return min(self.maximum, delay * (1.0 + 0.2 * self.jitter()))
