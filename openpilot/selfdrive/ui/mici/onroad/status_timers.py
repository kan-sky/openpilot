"""Display-only stop timers. Never create events or affect brake control."""
from dataclasses import dataclass


def format_mmss(elapsed: float) -> str:
  seconds = max(0, int(elapsed))
  return f"{seconds // 60:02d}:{seconds % 60:02d}"


@dataclass(frozen=True)
class TimerDisplay:
  kind: str
  elapsed: float


class StatusTimers:
  def __init__(self):
    self.reset()

  def reset(self):
    self._session = None
    self._parking_since = None
    self._hold_since = None
    self._last_fresh_at = None

  def update(self, *, now: float, started: bool, session: int, fresh: bool, standstill: bool,
             parking_brake: bool, hold_active: bool, alert_type: str | None, critical: bool = False,
             hold_without_alert: bool = True) -> TimerDisplay | None:
    if not started:
      self.reset()
      return None
    if session != self._session:
      self.reset()
      self._session = session

    if not fresh:
      # Hide invalid state immediately; tolerate at most one second of gaps
      # without inventing a new stop when the same signal returns.
      if self._last_fresh_at is None or not 0 <= now - self._last_fresh_at <= 1.:
        self.reset()
      return None
    if not standstill:
      self.reset()
      return None
    self._last_fresh_at = now

    self._parking_since = (now if self._parking_since is None else self._parking_since) if parking_brake else None
    self._hold_since = (now if self._hold_since is None else self._hold_since) if hold_active else None
    # Track the stop behind other alerts. Only a confirmed hold may replace the
    # low-speed steering notice; all other warnings and communication faults win.
    if critical:
      return None
    if self._parking_since is not None and alert_type in (None, "parkBrake/noEntry", "silentParkBrake/noEntry"):
      return TimerDisplay("parking", max(0., now - self._parking_since))
    if self._hold_since is not None and (alert_type in ("resumeRequired/warning", "brakeHold/warning", "softHold/warning") or
                                        (hold_without_alert and alert_type in (None, "belowSteerSpeed/warning"))):
      return TimerDisplay("hold", max(0., now - self._hold_since))
    return None
