"""Display-only trip and fade state. Never publishes vehicle/control messages."""
import math
from pathlib import Path

FADE_DURATION = 0.55
SCREEN_WAKE_FADE_DURATION = 0.85
SCREEN_SLEEP_FADE_DURATION = 2.0
TRIP_SUMMARY_DISPLAY_SECONDS = 10 * 60
DOOR_PATH = Path('/dev/shm/params/d/OffroadDoorOpen')


def smoothstep(progress: float) -> float:
  progress = max(0.0, min(1.0, progress))
  return progress * progress * (3.0 - 2.0 * progress)


class TripSummary:
  def __init__(self):
    self.distance_m = 0.0
    self.duration_s = 0.0
    self.expires_at = 0.0
    self.started_at = None
    self.last_sample = None

  def start(self, now: float):
    self.distance_m = self.duration_s = self.expires_at = 0.0
    self.started_at = self.last_sample = now

  def sample(self, now: float, speed: float | None):
    if self.started_at is None:
      return
    dt = now - self.last_sample
    self.last_sample = now
    self.duration_s = max(0.0, now - self.started_at)
    # Do not fill a missing-data/render stall with the last known speed.
    if 0.0 < dt <= 1.0 and speed is not None and math.isfinite(speed):
      self.distance_m += max(0.0, speed) * dt

  def finish(self, now: float):
    if self.started_at is not None:
      self.sample(now, None)
      self.expires_at = now + TRIP_SUMMARY_DISPLAY_SECONDS
    self.started_at = self.last_sample = None

  def visible(self, now: float) -> bool:
    return now < self.expires_at

  def distance_text(self) -> str:
    return f"{self.distance_m / 1000.0:.1f}km"

  def duration_text(self) -> str:
    minutes = max(1, int(round(self.duration_s / 60.0)))
    if minutes >= 60:
      return f"{minutes // 60}시간 {minutes % 60}분"
    return f"{minutes}분"


class SurfaceFade:
  def __init__(self, surface: str = 'offroad'):
    self.source = self.target = surface
    self.start_time = None

  def change(self, surface: str, now: float, instant: bool = False):
    current, _ = self.state(now)
    if instant:
      self.source = self.target = surface
      self.start_time = None
    elif surface != self.target:
      self.source, self.target, self.start_time = current, surface, now

  def state(self, now: float) -> tuple[str, float]:
    if self.start_time is None:
      return self.target, 0.0
    progress = max(0.0, (now - self.start_time) / FADE_DURATION)
    if progress >= 1.0:
      self.source = self.target
      self.start_time = None
      return self.target, 0.0
    surface = self.source if progress < 0.5 else self.target
    return surface, smoothstep(2 * progress if progress < 0.5 else 2 * (1 - progress))


class ScreenFade:
  def __init__(self):
    self.was_awake = True
    self.wake_time = None

  def alpha(self, now: float, *, awake: bool, offroad: bool, sleep_at: float, enabled: bool = True) -> float:
    if awake and not self.was_awake and offroad:
      self.wake_time = now
    self.was_awake = awake
    if not enabled or not offroad:
      self.wake_time = None
      return 0.0
    wake = 0.0
    if self.wake_time is not None:
      wake = 1.0 - smoothstep((now - self.wake_time) / SCREEN_WAKE_FADE_DURATION)
      if wake == 0.0:
        self.wake_time = None
    sleep = smoothstep(1.0 - (sleep_at - now) / SCREEN_SLEEP_FADE_DURATION) if sleep_at > 0 else 0.0
    return max(wake, sleep)


class DoorWake:
  """Consume only new closed-to-open edges from the existing offroad CAN reader."""
  def __init__(self, path: Path = DOOR_PATH):
    self.path = path
    # Existing files are a baseline. A first file created after UI startup is
    # a new observation, including the first door opening after a reboot.
    self.last_open = self._read() or False

  def _read(self) -> bool | None:
    try:
      with self.path.open('rb') as stream:
        raw = stream.read(8).strip()
    except OSError:
      return None
    if raw not in (b'0', b'1'):
      return None
    return raw == b'1'

  def poll(self, *, offroad: bool) -> bool:
    opened = self._read()
    if opened is None:
      return False
    wake = offroad and self.last_open is False and opened
    self.last_open = opened
    return wake
