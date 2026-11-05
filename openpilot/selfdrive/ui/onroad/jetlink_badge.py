from __future__ import annotations

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets.label import gui_label

COLORS = {
  'active': rl.Color(0, 255, 0, 230),
  'ready': rl.Color(255, 255, 255, 230),
  'loading': rl.Color(255, 220, 0, 230),
  'error': rl.Color(255, 70, 70, 255),
}


def draw_jetlink_badge(status: tuple[str, str] | None, bounds: rl.Rectangle, font_size: int,
                       *, stacked: bool = False) -> rl.Rectangle | None:
  if status is None:
    return None
  text, state = status
  color = COLORS.get(state)
  if not text or color is None:
    return None
  if stacked and text.isascii():
    text = text.replace(' ', '\n', 1)
  weight = FontWeight.SEMI_BOLD if text.isascii() else FontWeight.DISPLAY
  font_size = min(font_size, 30) if not text.isascii() else font_size
  font = gui_app.font(weight)
  pad_x = 4 if stacked else 10 if font_size <= 22 else 18
  size = measure_text_cached(font, text, font_size)
  if bounds.width < measure_text_cached(font, '...', font_size).x + 2 * pad_x or bounds.height < size.y:
    return None
  width = bounds.width if stacked else min(bounds.width, size.x + 2 * pad_x)
  rect = rl.Rectangle(bounds.x + bounds.width - width, bounds.y, width, bounds.height)
  roundness = min(1., 16. / min(rect.width, rect.height))
  rl.draw_rectangle_rounded(rect, roundness, 8, rl.Color(0, 0, 0, 170))
  rl.draw_rectangle_rounded_lines_ex(rect, roundness, 8, 2, color)
  alignment = TextAlignment.CENTER if stacked else TextAlignment.LEFT
  gui_label(rl.Rectangle(rect.x + pad_x, rect.y, rect.width - 2 * pad_x, rect.height),
            text, font_size, color, weight, alignment=alignment, elide_right=True)
  return rect


def draw_jetlink_detail(text: str, bounds: rl.Rectangle, font_size: int, elapsed: float) -> None:
  if not text:
    return
  weight = FontWeight.MEDIUM if text.isascii() else FontWeight.DISPLAY
  font = gui_app.font(weight)
  padding = 4 if font_size <= 14 else 12
  line_height = measure_text_cached(font, 'Ag', font_size).y + (1 if font_size <= 14 else 2)
  rows = int((bounds.height - 2 * padding) // line_height)
  if rows < 1 or bounds.width <= 2 * padding:
    return
  lines = wrap_text(font, text, font_size, int(bounds.width - 2 * padding))
  if not lines:
    return
  pages = (len(lines) + rows - 1) // rows
  start = (int(max(0., elapsed) // 4) % pages) * rows
  rl.draw_rectangle_rounded(bounds, min(1., 16. / min(bounds.width, bounds.height)), 8, rl.Color(0, 0, 0, 220))
  for row, line in enumerate(lines[start:start + rows]):
    gui_label(rl.Rectangle(bounds.x + padding, bounds.y + padding + row * line_height,
                           bounds.width - 2 * padding, line_height), line, font_size, rl.WHITE, weight, elide_right=False)
