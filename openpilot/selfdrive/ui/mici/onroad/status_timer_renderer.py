"""Wayon-style centered icon/time, isolated from the alert typography cache."""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.status_timers import TimerDisplay, format_mmss
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignmentVertical
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.label import UnifiedLabel

ICON_SIZE = 74
FONT_SIZE = 62
TIMER_GAP = 18


class StatusTimerRenderer:
  def __init__(self):
    self._icons = {kind: gui_app.texture(f"icons/{name}.png", ICON_SIZE, ICON_SIZE, keep_aspect_ratio=True)
                   for kind, name in (("parking", "parking"), ("hold", "autohold"))}
    self._label = UnifiedLabel("", font_size=FONT_SIZE, font_weight=FontWeight.DISPLAY,
                               alignment_vertical=TextAlignmentVertical.MIDDLE, elide=False)

  def draw(self, rect: rl.Rectangle, timer: TimerDisplay, alpha: float, y_offset: float = 0.) -> None:
    text = format_mmss(timer.elapsed)
    icon = self._icons[timer.kind]
    font = gui_app.font(FontWeight.DISPLAY)
    size = measure_text_cached(font, text, FONT_SIZE)
    # Preserve the normal 62px size; exceptionally long stops must still fit.
    # Keep the permanent left/right BSD strips clear even after hours of holding.
    available = max(1., rect.width - icon.width - TIMER_GAP - 128)
    font_size = min(FONT_SIZE, max(1, int(FONT_SIZE * available / max(1., size.x))))
    size = measure_text_cached(font, text, font_size)
    group_x = rect.x + (rect.width - icon.width - TIMER_GAP - size.x) / 2
    center_y = rect.y + rect.height / 2 + y_offset
    alpha = max(0., min(1., alpha))

    if timer.kind == "hold":
      center_color = rl.Color(0, 0, 0, round(170 * alpha))
      transparent = rl.Color(0, 0, 0, 0)
      half_height = min(75, int(rect.height / 2))
      rl.draw_rectangle_gradient_v(int(rect.x), int(center_y - half_height), int(rect.width), half_height, transparent, center_color)
      rl.draw_rectangle_gradient_v(int(rect.x), int(center_y), int(rect.width), half_height, center_color, transparent)

    rl.draw_texture_ex(icon, rl.Vector2(group_x, center_y - icon.height / 2), 0., 1., rl.Color(255, 255, 255, round(255 * alpha)))
    self._label.set_text(text)
    self._label.set_font_size(font_size)
    self._label.set_text_color(rl.Color(255, 255, 255, round(255 * .9 * alpha)))
    self._label.render(rl.Rectangle(group_x + icon.width + TIMER_GAP, center_y - size.y / 2, size.x + 2, size.y))
