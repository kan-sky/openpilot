"""Pure helpers for Wayon drive-health reporting and route regression tests."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any


OPERATING_STATES = (
  "disconnected", "offroad", "offroad_door", "exit_courtesy",
  "onroad", "onroad_door", "reverse", "overspeed",
)


def finite(value: Any, default: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return default
  return parsed if math.isfinite(parsed) else default


def cutin_risk_stage(score: Any, distance_m: Any = None, relative_speed_mps: Any = None) -> dict[str, Any]:
  """Convert the existing radar cut-in score into a stable, explainable stage."""
  score_value = max(0.0, min(1.0, finite(score)))
  distance = max(0.0, finite(distance_m, 999.0))
  closing_speed = max(0.0, -finite(relative_speed_mps))
  ttc = distance / closing_speed if closing_speed > 0.2 else None

  if score_value >= 0.72 or (score_value >= 0.55 and ttc is not None and ttc < 2.2):
    level, name = 3, "brake"
  elif score_value >= 0.35:
    level, name = 2, "prepare"
  elif score_value >= 0.15:
    level, name = 1, "watch"
  else:
    level, name = 0, "clear"

  return {
    "level": level,
    "name": name,
    "score": round(score_value, 3),
    "ttcS": round(ttc, 2) if ttc is not None and ttc < 60.0 else None,
  }


def resolve_operating_state(*, connected: bool, onroad: bool, door_open: bool = False,
                            gear: str = "unknown", overspeed: bool = False,
                            exit_courtesy: bool = False) -> str:
  """Resolve one canonical state shared by cloud, app and ambient previews."""
  if not connected:
    return "disconnected"
  if str(gear).lower() == "reverse":
    return "reverse"
  if onroad and overspeed:
    return "overspeed"
  if onroad and door_open:
    return "onroad_door"
  if onroad:
    return "onroad"
  if door_open:
    return "offroad_door"
  if exit_courtesy:
    return "exit_courtesy"
  return "offroad"


def evaluate_route_report(report: dict[str, Any], minimum_stop_score: int = 70) -> dict[str, Any]:
  """Turn a route report into deterministic regression checks."""
  stop = report.get("stopQuality") or {}
  cutin = report.get("cutInRisk") or {}
  openpilot = report.get("openpilot") or {}
  stop_count = int(stop.get("count") or 0)
  stop_score = stop.get("score")
  checks = {
    "stopQuality": {
      "status": "skip" if stop_count == 0 else "pass" if finite(stop_score) >= minimum_stop_score else "fail",
      "value": stop_score,
      "minimum": minimum_stop_score,
    },
    "openpilotCoverage": {"status": "info", "value": finite(openpilot.get("activePercent"))},
    "cutInRisk": {
      "status": "info",
      "events": int(cutin.get("eventCount") or 0),
      "maximumLevel": int(cutin.get("maxLevel") or 0),
    },
  }
  failed = [name for name, check in checks.items() if check["status"] == "fail"]
  return {"passed": not failed, "failedChecks": failed, "checks": checks}


@dataclass
class StopQualityEvent:
  started_at_s: float
  ended_at_s: float
  duration_s: float
  approach_speed_mps: float
  peak_decel_mps2: float
  peak_jerk_mps3: float
  terminal_decel_mps2: float
  terminal_jerk_mps3: float
  score: int
  grade: str

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)


class StopQualityTracker:
  """Observational stop evaluator. It never changes longitudinal commands."""

  def __init__(self) -> None:
    self.events: list[StopQualityEvent] = []
    self._samples: list[tuple[float, float, float, float]] = []
    self._active = False
    self._stationary_since: float | None = None
    self._last_t: float | None = None
    self._last_a: float | None = None

  def update(self, timestamp_s: Any, speed_mps: Any, accel_mps2: Any, standstill: bool = False) -> StopQualityEvent | None:
    timestamp = finite(timestamp_s)
    speed = max(0.0, finite(speed_mps))
    accel = finite(accel_mps2)
    dt = timestamp - self._last_t if self._last_t is not None else 0.0
    jerk = (accel - self._last_a) / dt if self._last_a is not None and 0.015 <= dt <= 1.0 else 0.0
    self._last_t, self._last_a = timestamp, accel

    if not self._active and 0.35 < speed <= 12.0 and accel < -0.12:
      self._active = True
      self._samples = []
      self._stationary_since = None

    if not self._active:
      return None

    self._samples.append((timestamp, speed, accel, jerk))
    if len(self._samples) > 1200:
      self.reset_active()
      return None

    if standstill or speed <= 0.08:
      if self._stationary_since is None:
        self._stationary_since = timestamp
      if timestamp - self._stationary_since >= 0.25:
        return self._finish()
    else:
      self._stationary_since = None

    if speed > 13.0 or (len(self._samples) > 8 and accel > 0.35 and speed > 1.0):
      self.reset_active()
    return None

  def reset_active(self) -> None:
    self._active = False
    self._samples = []
    self._stationary_since = None

  def _finish(self) -> StopQualityEvent | None:
    samples = self._samples
    self.reset_active()
    if len(samples) < 4:
      return None
    start_t, approach_speed, _, _ = samples[0]
    end_t = samples[-1][0]
    peak_decel = abs(min(sample[2] for sample in samples))
    peak_jerk = max(abs(sample[3]) for sample in samples)
    terminal = [sample for sample in samples if sample[1] <= 1.0] or samples[-4:]
    terminal_decel = abs(min(sample[2] for sample in terminal))
    terminal_jerk = max(abs(sample[3]) for sample in terminal)

    penalty = (
      max(0.0, peak_decel - 2.2) * 9.0
      + max(0.0, peak_jerk - 3.0) * 4.0
      + max(0.0, terminal_decel - 1.2) * 18.0
      + max(0.0, terminal_jerk - 2.0) * 7.0
    )
    score = max(0, min(100, round(100.0 - penalty)))
    grade = "excellent" if score >= 90 else "good" if score >= 78 else "rough" if score >= 60 else "harsh"
    event = StopQualityEvent(
      started_at_s=round(start_t, 3),
      ended_at_s=round(end_t, 3),
      duration_s=round(max(0.0, end_t - start_t), 2),
      approach_speed_mps=round(approach_speed, 2),
      peak_decel_mps2=round(peak_decel, 2),
      peak_jerk_mps3=round(peak_jerk, 2),
      terminal_decel_mps2=round(terminal_decel, 2),
      terminal_jerk_mps3=round(terminal_jerk, 2),
      score=score,
      grade=grade,
    )
    self.events.append(event)
    return event

  def summary(self) -> dict[str, Any]:
    if not self.events:
      return {"count": 0, "score": None, "grade": "unavailable", "harshCount": 0, "events": []}
    score = round(sum(event.score for event in self.events) / len(self.events))
    grade = "excellent" if score >= 90 else "good" if score >= 78 else "rough" if score >= 60 else "harsh"
    return {
      "count": len(self.events),
      "score": score,
      "grade": grade,
      "harshCount": sum(event.grade in ("rough", "harsh") for event in self.events),
      "events": [event.to_dict() for event in self.events[-20:]],
    }


def telemetry_signature(payload: dict[str, Any]) -> tuple[Any, ...]:
  """Fields that deserve an immediate cloud update when they change."""
  vehicle = payload.get("vehicle") or {}
  can = vehicle.get("can") or {}
  openpilot = payload.get("openpilot") or {}
  panda = payload.get("panda") or {}
  cutin = payload.get("cutInRisk") or {}
  connections = payload.get("connections") or {}
  return (
    bool(payload.get("onroad")), bool(payload.get("ignition")), bool(payload.get("enabled")),
    bool(openpilot.get("active")), str(openpilot.get("state") or ""),
    bool(can.get("valid")), bool(can.get("timeout")),
    bool(vehicle.get("doorOpen")), bool(vehicle.get("parkingBrake")),
    str(vehicle.get("gear") or ""), str(panda.get("faultStatus") or ""),
    str(payload.get("thermalStatus") or ""), int(cutin.get("level") or 0),
    str(payload.get("systemState") or ""),
    tuple(sorted((name, item.get("state")) for name, item in connections.items() if isinstance(item, dict))),
  )
