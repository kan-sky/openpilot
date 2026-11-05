from __future__ import annotations

import math

try:
  from cluster_config import AMBIENT_ACCEL_DEADBAND_MPS2, AMBIENT_ACCEL_SMOOTH_TAU_SECONDS, AMBIENT_MAX_ACCEL_MPS2
except ModuleNotFoundError:
  from openpilot.selfdrive.carrot.cluster.cluster_config import AMBIENT_ACCEL_DEADBAND_MPS2, AMBIENT_ACCEL_SMOOTH_TAU_SECONDS, AMBIENT_MAX_ACCEL_MPS2


def _clamp(value: float, low: float, high: float) -> float:
  return max(low, min(high, value))


def ambient_power_gauge(accel_mps2: float | int | None) -> tuple[str, float]:
  try:
    value = 0.0 if accel_mps2 is None else float(accel_mps2)
  except (TypeError, ValueError):
    value = 0.0
  if not math.isfinite(value) or abs(value) < 0.005:
    return "idle", 0.0
  return ("power" if value > 0.0 else "brake"), _clamp(abs(value) / AMBIENT_MAX_ACCEL_MPS2, 0.0, 1.0)


def smooth_ambient_accel(previous: float, accel_mps2: float | int | None, elapsed_seconds: float) -> float:
  try:
    target = 0.0 if accel_mps2 is None else float(accel_mps2)
  except (TypeError, ValueError):
    target = 0.0
  if not math.isfinite(target) or abs(target) < AMBIENT_ACCEL_DEADBAND_MPS2:
    target = 0.0
  if not math.isfinite(previous):
    previous = 0.0
  elapsed = _clamp(float(elapsed_seconds), 0.0, 0.25)
  alpha = 1.0 - math.exp(-elapsed / AMBIENT_ACCEL_SMOOTH_TAU_SECONDS)
  value = previous + (target - previous) * alpha
  return 0.0 if target == 0.0 and abs(value) < 0.01 else value


def extrapolated_media_position_ms(
  position_ms: int | None,
  duration_ms: int | None,
  is_playing: bool,
  received_at_ms: int | None,
  now_ms: int,
) -> int | None:
  if position_ms is None or position_ms < 0:
    return None
  position = int(position_ms)
  if is_playing and received_at_ms is not None and now_ms > received_at_ms:
    position += now_ms - received_at_ms
  if duration_ms is not None and duration_ms >= 0:
    position = min(position, int(duration_ms))
  return max(0, position)


def fitted_text_size(
  measured_width: float,
  available_width: float,
  max_size: float,
  min_size: float,
) -> float:
  if measured_width <= 0.0 or available_width <= 0.0:
    return max_size
  return _clamp(max_size * available_width / measured_width, min_size, max_size)


def leftward_loop_marquee_offset(
  elapsed_seconds: float,
  loop_distance: float,
  speed_px_per_second: float,
  hold_seconds: float,
) -> float:
  if loop_distance <= 0.0 or speed_px_per_second <= 0.0:
    return 0.0
  hold = max(0.0, hold_seconds)
  move = loop_distance / speed_px_per_second
  cycle = hold + move
  if cycle <= 0.0:
    return 0.0

  phase = max(0.0, elapsed_seconds) % cycle
  if phase <= hold:
    return 0.0
  return -min(loop_distance, (phase - hold) * speed_px_per_second)


def ambient_bsm_edges(left_blindspot: bool, right_blindspot: bool) -> tuple[bool, bool]:
  return bool(left_blindspot), bool(right_blindspot)


def smooth_bsm_opacity(
  previous: float,
  active: bool,
  elapsed_seconds: float,
  fade_seconds: float,
) -> float:
  value = previous if math.isfinite(previous) else 0.0
  value = _clamp(value, 0.0, 1.0)
  target = 1.0 if active else 0.0
  if fade_seconds <= 0.0:
    return target
  step = _clamp(float(elapsed_seconds) / fade_seconds, 0.0, 1.0)
  if target > value:
    return min(target, value + step)
  return max(target, value - step)
