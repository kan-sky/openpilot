"""Verify local and cereal-HUD phone media with the native raylib renderer."""
import argparse
import base64
from dataclasses import replace
from io import BytesIO
import os
from pathlib import Path
import sys
from unittest.mock import patch  # noqa: TID251 -- freeze clocks in this standalone visual verifier

from PIL import Image, ImageChops, ImageStat
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from hud_protocol import PhoneMediaEncoder, PhoneMediaReceiver


def configure_preview_graphics():
  if Path('/TICI').is_file() and os.environ.get('RAYLIB_BACKEND', 'comma') != 'comma':
    raise RuntimeError('C4 preview requires the verified comma backend; headless is unsupported')
  from cluster_graphics_cleanup import install_graphics_cleanup
  install_graphics_cleanup()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  configure_preview_graphics()
  from cluster_live import standby_state
  from cluster_models import PhoneMediaInfo
  from cluster_renderer import ClusterUiRenderer
  args.output.mkdir(parents=True, exist_ok=True)
  # A deterministic, clearly synthetic album cover exercises actual PNG decode.
  cover = Image.new('RGB', (180, 180), '#267b64')
  buffer = BytesIO()
  cover.save(buffer, format='PNG')
  art = base64.b64encode(buffer.getvalue()).decode()
  state = replace(standby_state(), speed_kph=87., accel_mps2=.7, gear_text='D', cruise_kph=100,
                  cruise_gap=2, cruise_display_state='engaged', lfa_active=True, speed_limit_kph=80,
                  left_blindspot=True, phone_media=PhoneMediaInfo(title='이등병의 편지', artist='김광석',
                  is_playing=True, duration_ms=240000, position_ms=105000, received_at_ms=100000000,
                  art_base64=art, art_mime='image/png'))
  with patch('time.time', return_value=100000.), patch('time.monotonic', return_value=10.), patch('time.perf_counter', return_value=10.):
    encoder, receiver = PhoneMediaEncoder(), PhoneMediaReceiver()
    record = {'version': 1, 'phone_media': encoder.encode(state.phone_media)}
    transported = replace(state, phone_media=receiver.decode((10., record)))
    for theme in ('dark', 'light'):
      renderer = ClusterUiRenderer(width=1920, height=480, theme_mode=theme)
      try:
        renderer.open(hidden=True)
        assert renderer._owned_ambient_fonts == {'light', 'regular', 'medium', 'semibold', 'bold'}
        renderer._ambient_bsm_left_opacity = 1.
        renderer.render_to_file(state, args.output / (theme + '-local.png'))
        renderer.render_to_file(transported, args.output / (theme + '-jetson.png'))
        assert not renderer._ambient_step_errors_logged, renderer._ambient_step_errors_logged
        assert renderer._phone_media_unicode_font is not None, 'Unicode font fallback'
        assert renderer._phone_media_art_texture is not None, 'Cover art decode failed'
      finally:
        renderer.close()
      local = Image.open(args.output / (theme + '-local.png')).convert('RGB')
      remote = Image.open(args.output / (theme + '-jetson.png')).convert('RGB')
      assert ImageChops.difference(local, remote).getbbox() is None, 'Transport changed rendered pixels'
      assert max(ImageStat.Stat(remote).stddev) > 10, 'Blank display'
      print(theme, '1920x480 pixel-identical; Korean text, art, speed, cruise, BSM rendered')


if __name__ == '__main__':
  main()
