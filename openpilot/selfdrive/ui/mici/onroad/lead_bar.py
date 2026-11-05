"""Short-lived presentation of a lead, independent of detection and control."""
from dataclasses import dataclass
import math

import pyray as rl


@dataclass(frozen=True)
class LeadBarTarget:
  x: float
  y: float
  width: float
  radar: bool
  track_id: int = -1


class LeadBar:
  RADAR_COLOR = (32, 216, 106)
  VISION_COLOR = (80, 190, 255)
  HEIGHT = 3.0
  FADE_IN = 0.18
  FADE_OUT = 0.20
  POSITION_TAU = 0.10

  def __init__(self):
    self.reset()

  def reset(self):
    self.alpha = 0.0
    self._target = None
    self._geometry = None
    self._previous = None

  def draw(self, target: LeadBarTarget | None, now: float):
    if not math.isfinite(now):
      self.reset()
      return
    dt = 0.0 if self._previous is None else now - self._previous
    if dt < 0 or dt > 0.5:
      self.reset()
      dt = 0.0
    self._previous = now

    if target is not None and (target.width <= 0 or not all(math.isfinite(value) for value in (target.x, target.y, target.width))):
      target = None
    if target is not None:
      # Do not slide the old marker across the screen onto a different radar car.
      replaced = (self._target is not None and self._target.radar and target.radar and
                  self._target.track_id >= 0 and target.track_id >= 0 and self._target.track_id != target.track_id)
      geometry = (target.x, target.y, max(24.0, min(140.0, target.width)))
      if self._geometry is None or replaced:
        self._geometry = geometry
        self.alpha = 0.0
        dt = 0.0
      else:
        blend = 1.0 - math.exp(-dt / self.POSITION_TAU)
        self._geometry = tuple(old + blend * (new - old) for old, new in zip(self._geometry, geometry, strict=True))
      self._target = target
      self.alpha = min(1.0, self.alpha + dt / self.FADE_IN)
    else:
      self.alpha = max(0.0, self.alpha - dt / self.FADE_OUT)
      if self.alpha <= 1e-6:
        self.alpha = 0.0
        self._target = self._geometry = None

    if self._geometry is None or self.alpha <= 0.0:
      return
    x, y, width = self._geometry
    color = self.RADAR_COLOR if self._target.radar else self.VISION_COLOR
    # A thin dark underlay keeps the small bar visible over bright camera pixels.
    rl.draw_rectangle_rounded(rl.Rectangle(x - width / 2 - 1, y - 1, width + 2, self.HEIGHT + 2),
                              1.0, 6, rl.Color(0, 0, 0, round(160 * self.alpha)))
    rl.draw_rectangle_rounded(rl.Rectangle(x - width / 2, y, width, self.HEIGHT),
                              1.0, 6, rl.Color(*color, round(255 * self.alpha)))
