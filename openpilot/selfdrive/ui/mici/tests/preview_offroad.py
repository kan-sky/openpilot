"""Render the real welcome widget on a desktop, without vehicle publishers."""
import argparse
from pathlib import Path
import time

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.welcome import MiciWelcomeLayout
from openpilot.selfdrive.ui.offroad_presentation import TripSummary
from openpilot.system.ui.lib.application import FontWeight, gui_app


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  gui_app.init_window('C4 offroad native preview', font_weights=[FontWeight.DISPLAY, FontWeight.KOREAN, FontWeight.UNIFONT])
  trip = TripSummary()
  welcome = MiciWelcomeLayout(trip)
  try:
    for name, alpha in [('greeting', 0), ('summary', 0), ('summary-long', 0), ('fade-half', .5), ('fade-black', 1)]:
      if name != 'greeting':
        trip.distance_m, trip.duration_s = 23400, 2280
        trip.expires_at = time.monotonic() + 600
      if name == 'summary-long':
        trip.distance_m, trip.duration_s = 1234567, 44940
      for _ in range(3):
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        welcome.render(rl.Rectangle(0, 0, 536, 240))
        rl.draw_rectangle(0, 0, 536, 240, rl.Color(0, 0, 0, round(255 * alpha)))
        rl.end_drawing()
      image = rl.load_image_from_screen()
      try:
        assert rl.export_image(image, str(args.output / f'{name}.png'))
      finally:
        rl.unload_image(image)
  finally:
    gui_app.close()


if __name__ == '__main__':
  main()
