"""Native 536x240 lead-bar animation check with synthetic geometry, no device I/O."""
import argparse
from pathlib import Path

import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.lead_bar import LeadBar, LeadBarTarget


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', required=True, type=Path)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
  rl.init_window(536, 240, 'C4 lead bars: synthetic native render')
  rl.set_target_fps(20)
  font = rl.get_font_default()
  target = rl.load_render_texture(536, 240)
  bar = LeadBar()
  samples = []
  try:
    for frame in range(100):
      now = frame / 20
      radar = frame < 50
      visible = 5 <= frame < 25 or 55 <= frame < 75
      lead = LeadBarTarget(268, 140, 80, radar, 2 if radar else -1) if visible else None
      rl.begin_texture_mode(target)
      rl.clear_background(rl.Color(30, 32, 34, 255))
      rl.draw_rectangle(268, 0, 268, 240, rl.Color(220, 222, 224, 255))
      # This is a contrast/animation fixture, not a road-camera or vehicle capture.
      rl.draw_text_ex(font, 'RADAR' if radar else 'VISION', rl.Vector2(24, 24), 20, 0, rl.Color(220, 222, 224, 255))
      rl.draw_text_ex(font, '536x240 / 20 FPS', rl.Vector2(305, 25), 16, 0, rl.Color(30, 32, 34, 255))
      bar.draw(lead, now)
      rl.end_texture_mode()
      picture = rl.load_image_from_texture(target.texture)
      try:
        rl.image_flip_vertical(picture)
        assert (picture.width, picture.height) == (536, 240)
        pixels = bytes(rl.ffi.buffer(picture.data, 536 * 240 * 4))
        samples.append(tuple(pixels[(141 * 536 + 250) * 4:(141 * 536 + 250) * 4 + 3]))
        assert rl.export_image(picture, str(args.output / f'frame-{frame:03d}.png'))
      finally:
        rl.unload_image(picture)
      rl.begin_drawing()
      rl.clear_background(rl.BLACK)
      rl.draw_texture_rec(target.texture, rl.Rectangle(0, 0, 536, -240), rl.Vector2(0, 0), rl.WHITE)
      rl.end_drawing()
    assert samples[4] == samples[5] == samples[29] == samples[54] == samples[55] == samples[79] == (30, 32, 34)
    assert samples[10] == samples[24] == LeadBar.RADAR_COLOR
    assert samples[60] == samples[74] == LeadBar.VISION_COLOR
    assert samples[5][1] < samples[6][1] < samples[7][1] < samples[8][1] < samples[9][1]
    assert samples[24][1] > samples[25][1] > samples[26][1] > samples[27][1] > samples[28][1]
    assert samples[55][2] < samples[56][2] < samples[57][2] < samples[58][2] < samples[59][2]
    assert samples[74][2] > samples[75][2] > samples[76][2] > samples[77][2] > samples[78][2]
    print('PASS: 100 native 536x240 frames; exact colors, timed fades and complete disappearance. Synthetic inputs, not a device capture.')
  finally:
    rl.unload_render_texture(target)
    rl.close_window()


if __name__ == '__main__':
  main()
