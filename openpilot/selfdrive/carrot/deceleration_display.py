"""Shared display-only deceleration verdict; callers own message freshness."""
import math
from typing import NamedTuple

from openpilot.selfdrive.carrot.deceleration_source import DECELERATION_SOURCE_LABELS


class DecelerationState(NamedTuple):
  label: str
  phase: str
  source: str


def _source(value):
  source = str(value or '').strip().lower()
  for suffix in (':n', ':v', ':c'):
    source = source.removesuffix(suffix)
  return source


def _number(value):
  try:
    return float(value)
  except (TypeError, ValueError, OverflowError):
    return math.nan


def deceleration_state(car, selfdrive, control, plan, carrot) -> DecelerationState | None:
  if any(message is None for message in (car, selfdrive, control, plan, carrot)):
    return None
  if not getattr(selfdrive, 'enabled', False) or not getattr(control, 'longActive', False):
    return None
  selected = _number(getattr(car, 'vCruise', None))
  applied_target = _number(getattr(plan, 'cruiseTarget', None))
  desired = _number(getattr(carrot, 'desiredSpeed', None))
  if not (math.isfinite(selected) and 0 < selected < 255 and math.isfinite(applied_target) and 0 < desired <= 250):
    return None

  candidates = []
  # Missing metadata in an older publisher means unknown, not completed.
  for candidate in getattr(carrot, 'decelerationCandidates', ()):
    source = _source(getattr(candidate, 'source', ''))
    cap = _number(getattr(candidate, 'speedKph', None))
    final = _number(getattr(candidate, 'targetKph', None))
    if source not in DECELERATION_SOURCE_LABELS or source == 'gas':
      continue
    if not (math.isfinite(cap) and 0 < cap <= 250 and math.isfinite(final) and 0 <= final < 200):
      continue
    if final == 0 and cap >= 200:
      continue
    distance = _number(getattr(candidate, 'distanceM', -1))
    distance = distance if math.isfinite(distance) and distance >= 0 else math.inf
    candidates.append((source, cap, final, distance))
  if not candidates:
    return None

  selected_source = _source(getattr(carrot, 'desiredSource', ''))
  overridden = getattr(car, 'gasPressed', False) or getattr(car, 'brakePressed', False) or selected_source == 'gas'

  def is_applied(candidate):
    source, cap, _, _ = candidate
    return (not overridden and source == selected_source and desired < selected and
            abs(cap - desired) < 1.01 and abs(applied_target - desired) <= 1.0)

  # Curvature is calculated continuously; show curves only while their cap is applied.
  # Filter before selection so an unapplied curve cannot mask a pending camera.
  candidates = [candidate for candidate in candidates if candidate[0] not in ('vturn', 'route') or is_applied(candidate)]
  if not candidates:
    return None

  def priority(candidate):
    _, cap, final, distance = candidate
    # A satisfied curve/road cap must not hide a pending camera/turn slowdown.
    needs_reduction = (final > 0 and final < selected) or cap < selected
    return (0 if is_applied(candidate) else 1 if needs_reduction else 2, cap, distance, final)

  source, cap, final, _ = candidate = min(candidates, key=priority)
  applied = is_applied(candidate)
  phase = 'waiting'
  # Match the published integer cap used by the planner, never actual speed.
  if not overridden and final > 0 and (selected <= final or (applied and desired <= final + 0.1)):
    phase = 'complete'
  elif applied:
    phase = 'decelerating'
  return DecelerationState(DECELERATION_SOURCE_LABELS[source].upper(), phase, source)
