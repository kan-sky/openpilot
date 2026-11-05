import time
from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.offroad_presentation import TripSummary
from openpilot.system.ui.lib.application import FontWeight, MousePos, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel


class MiciWelcomeLayout(Widget):
  def __init__(self, trip: TripSummary):
    super().__init__()
    self.trip = trip
    self._on_settings_click: Callable | None = None

    def label(text, size, color=rl.WHITE):
      return UnifiedLabel(text, font_size=size, font_weight=FontWeight.DISPLAY, text_color=color,
                          alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
                          alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE, wrap_text=False)

    self._greeting = label("안녕하세요!", 72)
    self._message = label("안전한 주행 되세요", 40, rl.GRAY)
    self._title = label("수고하셨습니다", 42, rl.Color(255, 255, 255, 215))
    self._distance = label(trip.distance_text, 52)
    self._duration = label(trip.duration_text, 52)

  def set_callbacks(self, on_settings: Callable):
    self._on_settings_click = on_settings

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._on_settings_click:
      self._on_settings_click()

  @staticmethod
  def _render_value(label: UnifiedLabel, text: str, rect: rl.Rectangle):
    width = measure_text_cached(gui_app.font(FontWeight.DISPLAY), text, 52).x
    label.set_font_size(min(52, int(52 * (rect.width - 16) / max(width, 1))))
    label.render(rect)

  def _render(self, _):
    x, y, width = self.rect.x + 18, self.rect.y, self.rect.width - 36
    if not self.trip.visible(time.monotonic()):
      self._greeting.render(rl.Rectangle(x, y + 28, width, 96))
      self._message.render(rl.Rectangle(x, y + 132, width, 58))
      return

    self._title.render(rl.Rectangle(x, y + 20, width, 56))
    middle = x + width / 2
    rl.draw_rectangle(int(middle), int(y + 98), 2, 82, rl.Color(255, 255, 255, 32))
    self._render_value(self._distance, self.trip.distance_text(), rl.Rectangle(x, y + 104, width / 2, 70))
    self._render_value(self._duration, self.trip.duration_text(), rl.Rectangle(middle, y + 104, width / 2, 70))
