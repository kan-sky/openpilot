"""Render the actual timer widget at C4 resolution; no vehicle connections."""
import argparse
from pathlib import Path

import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.status_timer_renderer import StatusTimerRenderer
from openpilot.selfdrive.ui.mici.onroad.status_timers import TimerDisplay
from openpilot.system.ui.lib.application import FontWeight, gui_app


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  gui_app.init_window('C4 stop timers native preview', font_weights=[FontWeight.DISPLAY])
  try:
    renderer = StatusTimerRenderer()
    for width in (536, 476):
      rect = rl.Rectangle(536 - width, 0, width, 240)
      for kind in ('parking', 'hold'):
        for elapsed in (83, 6103, 259200):
          for _ in range(3):
            rl.begin_drawing()
            rl.clear_background(rl.BLACK)
            rl.draw_rectangle_rec(rect, rl.Color(38, 40, 42, 255))
            renderer.draw(rect, TimerDisplay(kind, elapsed), 1.)
            rl.end_drawing()
          label = renderer._label
          assert label.text_width <= label.rect.width
          assert len(label._cached_wrapped_lines) == 1
          assert label.rect.x >= rect.x
          assert label.rect.x + label.rect.width <= rect.x + rect.width
          assert label.rect.y >= rect.y and label.rect.y + label.rect.height <= rect.y + rect.height
          image = rl.load_image_from_screen()
          try:
            assert rl.export_image(image, str(args.output / f'{kind}-{width}-{elapsed}.png'))
          finally:
            rl.unload_image(image)
    print('PASS: 12 native frames, single-line timers contained in 536/476 x 240 viewports')
  finally:
    gui_app.close()


if __name__ == '__main__':
  main()
