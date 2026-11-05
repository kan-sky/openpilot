"""Real layout/wrapping code with Pillow font metrics, without a GL/device context."""
import ast
import math
import random
import string
import time
import unittest
from dataclasses import dataclass
from enum import IntEnum, StrEnum
from functools import lru_cache
from itertools import zip_longest
from types import SimpleNamespace
from typing import NamedTuple

from PIL import Image, ImageDraw, ImageFont
from openpilot.selfdrive.selfdrived import alert_presentation as presentation
from openpilot.selfdrive.selfdrived.tests.test_alert_presentation import ROOT, load_events, callback_args


@dataclass
class Rect:
  x: float = 0
  y: float = 0
  width: float = 0
  height: float = 0


class Widget:
  def __init__(self):
    self._rect = Rect()

  @property
  def rect(self):
    return self._rect

  def render(self, rect):
    self._rect = rect


@lru_cache(maxsize=256)
def pil_font(size):
  return ImageFont.truetype(str(ROOT / "selfdrive/assets/fonts/Pretendard-SemiBold.ttf"), max(1, round(size * 1.16)))


def measured(_font, text, size, spacing=0):
  font = pil_font(size)
  # Raylib advances individual glyphs rather than applying HarfBuzz kerning.
  return SimpleNamespace(x=sum(font.getlength(c) for c in text) + max(0, len(text) - 1) * spacing, y=size * 1.16)


def load_layout():
  rl = SimpleNamespace(Rectangle=Rect, Vector2=lambda x, y: SimpleNamespace(x=x, y=y),
                       Color=lambda *args: args, WHITE=(255, 255, 255, 255))
  font = SimpleNamespace(texture=SimpleNamespace(id=1))
  ns = {
    "math": math, "time": time, "random": random, "string": string, "IntEnum": IntEnum, "StrEnum": StrEnum,
    "NamedTuple": NamedTuple, "dataclass": dataclass, "zip_longest": zip_longest, "Widget": Widget, "rl": rl,
    "FontWeight": SimpleNamespace(NORMAL=0, DISPLAY=0, ROMAN=0),
    "TextAlignment": SimpleNamespace(LEFT=0, CENTER=1, RIGHT=2),
    "TextAlignmentVertical": SimpleNamespace(TOP=0, MIDDLE=1, BOTTOM=2),
    "FONT_SCALE": 1.16, "DEFAULT_TEXT_SIZE": 60, "DEFAULT_TEXT_COLOR": rl.WHITE,
    "gui_app": SimpleNamespace(font=lambda _: font), "font_fallback": lambda f, text: f,
    "measure_text_cached": measured, "find_emoji": lambda _: [], "font_sizes": presentation.font_sizes,
    "AlertSize": SimpleNamespace(none=0, small=1, mid=2, full=3),
    "AlertStatus": SimpleNamespace(normal=0, userPrompt=1, critical=2),
    "car": SimpleNamespace(CarControl=SimpleNamespace(HUDControl=SimpleNamespace(VisualAlert=SimpleNamespace(none=0)))),
    "ui_state": SimpleNamespace(sm={"carState": SimpleNamespace(leftBlinker=True, rightBlinker=False)}),
  }
  for path in ("system/ui/lib/wrap_text.py", "system/ui/widgets/label.py", "selfdrive/ui/mici/onroad/alert_renderer.py"):
    tree = ast.parse((ROOT / path).read_text())
    body = [ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
    for node in tree.body:
      if isinstance(node, (ast.Import, ast.ImportFrom)):
        continue
      if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in ("AlertSize", "AlertStatus") for t in node.targets):
        continue
      body.append(node)
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), path, "exec"), ns)
  renderer = ns['AlertRenderer'].__new__(ns['AlertRenderer'])
  renderer._rect = Rect(60, 20, 476, 240)
  renderer._alpha_filter = SimpleNamespace(x=1.)
  renderer._alert_y_filter = SimpleNamespace(x=20.)
  renderer._alert_text1_label = ns['UnifiedLabel']('', elide=False)
  renderer._alert_text2_label = ns['UnifiedLabel']('', elide=False)
  renderer._text_layout_key = None
  renderer._text_layout_heights = (0., 0.)
  renderer._last_icon_side = None
  renderer._txt_turn_signal_left = SimpleNamespace(width=104, height=96)
  renderer._txt_turn_signal_right = SimpleNamespace(width=104, height=96)
  renderer._txt_blind_spot_left = SimpleNamespace(width=134, height=150)
  renderer._txt_blind_spot_right = SimpleNamespace(width=134, height=150)
  return renderer, ns


def rendered_alerts():
  m = load_events()
  renderer, ns = load_layout()
  for event, kinds in m.EVENTS.items():
    for kind in kinds:
      ev = m.Events()
      ev.add(event)
      ev.event_counters[event] = 10000
      ev._steer_warning_fault_frames = m.TEMP_STEER_WARNING_DELAY_FRAMES
      for alert in ev.create_alerts([kind], callback_args()):
        if alert.alert_size == m.AlertSize.none:
          continue
        ui_alert = ns['Alert'](alert.alert_text_1, alert.alert_text_2, alert.alert_size,
                               alert.alert_status, alert.visual_alert, alert.alert_type)
        layout = renderer._icon_helper(ui_alert)
        renderer._draw_text(ui_alert, layout)
        yield alert, renderer, layout


class TestMiciAlertLayout(unittest.TestCase):
  def test_all_visible_events_fit_actual_viewport_without_elision(self):
    count = 0
    for alert, renderer, layout in rendered_alerts():
      with self.subTest(alert=alert.alert_type):
        labels = (renderer._alert_text1_label, renderer._alert_text2_label)
        for text, label in zip((alert.alert_text_1, alert.alert_text_2), labels, strict=True):
          if not text:
            continue
          self.assertEqual(label.text, text)
          self.assertFalse(label._elide)
          self.assertLessEqual(label.text_width, layout.text_rect.width)
          self.assertGreaterEqual(label.rect.x, renderer.rect.x)
          self.assertGreaterEqual(label.rect.y, renderer.rect.y)
          self.assertLessEqual(label.rect.x + label.rect.width, renderer.rect.x + renderer.rect.width)
          self.assertLessEqual(label.rect.y + label.get_content_height(int(label.rect.width)), renderer.rect.y + renderer.rect.height)
          self.assertGreaterEqual(label.font_size, 12)
        if alert.alert_text_1 and alert.alert_text_2:
          self.assertGreaterEqual(labels[1].rect.y, labels[0].rect.y + labels[0].rect.height)
      count += 1
    self.assertGreater(count, 150)

  def test_icons_do_not_overlap_text(self):
    for alert, renderer, layout in rendered_alerts():
      if not layout.icon:
        continue
      if layout.icon.side == 'left':
        icon_right = renderer.rect.x + layout.icon.margin_x + layout.icon.texture.width
        self.assertGreater(layout.text_rect.x, icon_right)
      else:
        icon_left = renderer.rect.x + renderer.rect.width - layout.icon.margin_x - layout.icon.texture.width
        self.assertLess(layout.text_rect.x + layout.text_rect.width, icon_left)

  def test_fitting_is_cached_and_preserves_case_and_secondary_text(self):
    renderer, ns = load_layout()
    alert = ns['Alert']("AEB 작동", "차량 AEB 작동됨", size=1, alert_type="stockAeb/permanent")
    layout = renderer._icon_helper(alert)
    renderer._draw_text(alert, layout)
    self.assertEqual(renderer._alert_text1_label.text, "AEB 작동")
    self.assertEqual(renderer._alert_text2_label.text, "차량 AEB 작동됨")
    key = renderer._text_layout_key
    renderer._draw_text(alert, layout)
    self.assertEqual(renderer._text_layout_key, key)

  def test_short_notices_keep_each_line_unwrapped_in_both_c4_layouts(self):
    m = load_events()
    for event, title, subtitle, title_size in (
      (m.EventName.trafficStopping, "정지합니다", "전방 신호 감지됨.", 60),
      (m.EventName.radarCutin, "끼어드는 차량 감지", "전방 차량을 확인하세요", 48),
    ):
      events = m.Events()
      events.add(event)
      alert, = events.create_alerts([m.ET.WARNING], callback_args())
      self.check_short_notice_layout(alert, title, subtitle, title_size)

  def check_short_notice_layout(self, alert, title_text, subtitle_text, title_size):
    for width in (476, 536):
      with self.subTest(width=width):
        renderer, ns = load_layout()
        renderer._rect = Rect(536 - width, 0, width, 240)
        renderer._alert_y_filter.x = 0.
        ui_alert = ns['Alert'](alert.alert_text_1, alert.alert_text_2, alert.alert_size,
                               alert.alert_status, alert.visual_alert, alert.alert_type)
        layout = renderer._icon_helper(ui_alert)
        renderer._draw_text(ui_alert, layout)
        title, subtitle = renderer._alert_text1_label, renderer._alert_text2_label
        self.assertEqual((title.font_size, subtitle.font_size), (title_size, 30))
        for label, text in ((title, title_text), (subtitle, subtitle_text)):
          self.assertEqual(label.text, text)
          self.assertEqual(label._cached_wrapped_lines, [text])
          self.assertLessEqual(label.text_width, layout.text_rect.width)
          self.assertLessEqual(label.rect.y + label.rect.height, 240)
        self.assertGreaterEqual(subtitle.rect.y, title.rect.y + title.rect.height)

  def test_unresponsive_fallback_alerts_fit(self):
    renderer, ns = load_layout()
    for name in ("ALERT_STARTUP_PENDING", "ALERT_CRITICAL_TIMEOUT", "ALERT_CRITICAL_REBOOT"):
      alert = ns[name]
      renderer._draw_text(alert, renderer._icon_helper(alert))
      title, subtitle = renderer._text_layout_heights
      self.assertLessEqual(title + subtitle + 4, 224)
      self.assertEqual(renderer._alert_text1_label.text, alert.text1)
      self.assertEqual(renderer._alert_text2_label.text, alert.text2)


def preview():
  """In-memory typography preview; no UI, file writes, or device access."""
  wanted = {"preLaneChangeLeft/warning", "laneChangeBlocked/warning", "driverDistracted2/permanent",
            "lowMemory/permanent", "calibrationInvalid/permanent", "steerSaturated/warning",
            "controlsMismatch/immediateDisable", "audioFeedback/permanent"}
  canvas = Image.new("RGB", (536 * 2, 270 * 4), "#202020")
  draw = ImageDraw.Draw(canvas)
  index = 0
  for alert, renderer, layout in rendered_alerts():
    if alert.alert_type not in wanted:
      continue
    x, y = (index % 2) * 536, (index // 2) * 270
    draw.rectangle((x + 60, y + 30, x + 535, y + 269), fill=("#171717", "#9f4900", "#b80015")[alert.alert_status])
    draw.text((x + 4, y + 4), alert.alert_type, fill="white", font=ImageFont.load_default())
    for label in (renderer._alert_text1_label, renderer._alert_text2_label):
      if not label.text:
        continue
      line_y = y + 30 + label.rect.y - renderer.rect.y
      for line in label._cached_wrapped_lines:
        line_x = x + label.rect.x
        if label._alignment == 2:
          line_x += label.rect.width - measured(None, line, label.font_size).x
        draw.text((line_x, line_y), line, font=pil_font(label.font_size), fill="white", anchor="lt")
        line_y += label.font_size * 1.16 * .9
    index += 1
  return canvas


if __name__ == "__main__":
  unittest.main()
