"""Display speed-target progress, never physical braking or actual speed compliance."""
import pyray as rl

from openpilot.selfdrive.carrot.carrot_man_input import get_carrot_man
from openpilot.selfdrive.carrot.deceleration_display import DecelerationState, deceleration_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


def selected_deceleration(sm, started_frame: int) -> DecelerationState | None:
  def fresh(service):
    return sm.valid[service] and sm.alive[service] and sm.recv_frame[service] > 0 and sm.recv_frame[service] >= started_frame

  if not all(fresh(service) for service in ('carState', 'selfdriveState', 'carrotMan', 'carControl', 'longitudinalPlan')):
    return None
  return deceleration_state(sm['carState'], sm['selfdriveState'], sm['carControl'], sm['longitudinalPlan'], get_carrot_man(sm))


class DecelerationBadge:
  WIDTH = 76
  HEIGHT = 26
  PERIOD = 1.0
  GREEN = (32, 216, 106, 255)
  ORANGE = (255, 149, 0, 255)
  WHITE = (255, 255, 255, 255)

  def __init__(self):
    self.reset()

  def reset(self):
    self._active_state = None
    self._active_since = 0.0

  def draw(self, rect: rl.Rectangle, font, state: DecelerationState | None, now: float):
    if state is None:
      self.reset()
      return
    label, phase, source = state
    active_state = (source, phase)
    if active_state != self._active_state or now < self._active_since:
      self._active_state = active_state
      self._active_since = now
    # Live curve geometry can alternate at entry/exit. Confirm green briefly;
    # invalid/overridden/disengaged inputs still clear it immediately.
    if phase == 'complete' and source in ('route', 'vturn') and now - self._active_since < 0.3:
      phase = 'decelerating'
    color = self.GREEN if phase == 'complete' else self.WHITE
    if phase == 'decelerating' and (now - self._active_since) % self.PERIOD < 0.6:
      color = self.ORANGE
    box = rl.Rectangle(rect.x + 136, rect.y + rect.height - 78, self.WIDTH, self.HEIGHT)
    rl.draw_rectangle_rounded(box, 0.3, 6, rl.Color(*color))
    size = 18
    measured = measure_text_cached(font, label, size)
    size *= min(1, (self.WIDTH - 16) / max(1, measured.x))
    measured = measure_text_cached(font, label, size)
    color = rl.Color(8, 24, 14, 255)
    rl.draw_text_ex(font, label, rl.Vector2(box.x + (box.width - measured.x) / 2,
                                          box.y + (box.height - measured.y) / 2), size, 0, color)
