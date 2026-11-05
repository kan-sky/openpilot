"""CPU-only native atlas verification; never opens a window or vehicle IPC."""
import importlib.util
import ast
import re
import struct
from types import SimpleNamespace

import pytest

from openpilot.system.ui.lib.tests.test_c4_fonts import application_scope
from openpilot.system.ui.lib.tests.test_raylib_text import UI_DIR


FONT_DIR = UI_DIR.parents[1] / "selfdrive/assets/fonts"


def metrics(path):
  entries = {}
  for line in path.read_text().splitlines():
    if line.startswith("char id="):
      fields = {k: int(v) for k, v in re.findall(r"(\w+)=\s*(-?\d+)", line)}
      entries[fields["id"]] = fields
  return entries


def supported(entries):
  return {cp for cp, e in entries.items() if e["xadvance"] > 0 or (e["width"] > 0 and e["height"] > 0)}


def emoji_finder():
  scope = {"re": re}
  tree = ast.parse((UI_DIR / "lib/emoji.py").read_text())
  body = [n for n in tree.body if (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "EMOJI_REGEX" for t in n.targets))
          or (isinstance(n, ast.FunctionDef) and n.name == "find_emoji")]
  exec(compile(ast.Module(body=body, type_ignores=[]), "emoji.py", "exec"), scope)
  return scope["find_emoji"]


@pytest.fixture(scope="module")
def atlases(tmp_path_factory):
  rl = pytest.importorskip("pyray")
  spec = importlib.util.spec_from_file_location("c4_font_generator", FONT_DIR / "process.py")
  generator = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(generator)
  spec = importlib.util.spec_from_file_location("c4_korean_generator", FONT_DIR / "process_korean.py")
  korean_generator = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(korean_generator)
  korean_generator.process = generator
  output = tmp_path_factory.mktemp("c4-font-atlas")
  base, international = generator._char_sets()
  generator.FONT_DIR = output
  # The desktop raylib ABI has six arguments; the target's newer ABI has seven.
  load = rl.load_font_data
  try:
    if len(rl.ffi.typeof(rl.rl.LoadFontData).args) == 6:
      rl.load_font_data = lambda data, size, height, cps, count, kind, out_count: load(data, size, height, cps, count, kind)
    generator._process_font(FONT_DIR / "Pretendard-SemiBold.ttf", base)
    korean_generator.generate(FONT_DIR / "Pretendard-SemiBold.ttf", output)
    generator._process_font(FONT_DIR / "unifont.otf", international)
  finally:
    rl.load_font_data = load
  return output


def test_all_modern_hangul_syllables_and_common_compatibility_jamo_have_real_glyphs(atlases):
  coverage = supported(metrics(atlases / "Pretendard-KR-SemiBold.fnt"))
  assert set(range(0xAC00, 0xD7A4)) <= coverage
  assert set(range(0x3131, 0x3164)) <= coverage


def test_unsupported_jamo_and_ui_symbols_have_real_unifont_glyphs(atlases):
  korean = supported(metrics(atlases / "Pretendard-KR-SemiBold.fnt"))
  backup = supported(metrics(atlases / "unifont.fnt"))
  assert not {0x1100, 0x1161, 0x11A8} & korean
  assert set(range(0x1100, 0x1200)) | set(range(0x3131, 0x318F)) <= korean | backup
  assert set(map(ord, "설정 ⚙ ✓ × ° ◀ ▶ ✔ ⌫ ⇧ ○ ● ↳ € £ ¥")) <= korean | backup


def test_generated_atlas_face_and_size_and_texture_budget(atlases):
  from PIL import Image
  for stem, size in (("Pretendard-SemiBold", 200), ("Pretendard-KR-SemiBold", 48)):
    first_line = (atlases / f"{stem}.fnt").read_text().splitlines()[0]
    assert f'face="{stem}"' in first_line
    assert f"size=-{size} " in first_line
  with Image.open(atlases / "Pretendard-KR-SemiBold.png") as image:
    assert max(image.size) <= 8192
    assert image.width * image.height <= 8192 * 8192  # Previous KaiGen atlas.


def test_font_source_really_is_semibold_600():
  data = (FONT_DIR / "Pretendard-SemiBold.ttf").read_bytes()
  count = struct.unpack_from(">H", data, 4)[0]
  tables = {tag: offset for tag, _, offset, _ in (struct.unpack_from(">4sIII", data, 12 + 16 * i) for i in range(count))}
  assert struct.unpack_from(">H", data, tables[b"OS/2"] + 4)[0] == 600


def test_real_atlas_coverage_selects_consistent_measure_and_draw_fonts(atlases):
  scope = application_scope()
  weights = scope["FontWeight"]
  fonts = {}
  for i, weight in enumerate(weights, start=1):
    entries = metrics(atlases / weight).values()
    fonts[weight] = SimpleNamespace(texture=SimpleNamespace(id=i), glyphCount=len(entries),
                                   glyphs=[SimpleNamespace(value=e["id"], advanceX=e["xadvance"]) for e in entries],
                                   recs=[SimpleNamespace(width=e["width"], height=e["height"]) for e in entries])
  app = SimpleNamespace(_font_coverage={}, font=fonts.__getitem__, has_font=lambda w: w in fonts)
  app.font_supports_text = lambda f, t: scope["font_supports_text"](app, f, t)
  scope["gui_app"] = app
  import sys
  with pytest.MonkeyPatch.context() as patch:
    patch.setitem(sys.modules, "openpilot.system.ui.lib.emoji", SimpleNamespace(find_emoji=emoji_finder()))
    for text, expected in (("100 MAX 00:00", weights.NORMAL), ("안녕하세요", weights.KOREAN),
                           ("정지합니다\n전방 신호 감지됨.", weights.KOREAN), ("가", weights.UNIFONT), ("설정 ⚙", weights.UNIFONT)):
      selected = scope["font_fallback"](fonts[weights.NORMAL], text)
      assert selected is fonts[expected]
      assert app.font_supports_text(selected, text)
      for fragment in text.split():
        assert scope["font_fallback"](selected, fragment) is selected


def test_korean_translation_and_event_characters_are_renderable(atlases):
  from openpilot.selfdrive.ui.tests.test_mici_alert_presentation import rendered_alerts
  coverage = set().union(*(supported(metrics(p)) for p in atlases.glob("*.fnt")))
  texts = [(FONT_DIR.parents[1] / "ui/translations/app_ko.po").read_text()]
  for alert, _, _ in rendered_alerts():
    texts.extend((alert.alert_text_1, alert.alert_text_2))
  find_emoji = emoji_finder()
  missing = set()
  for text in texts:
    for start, end, _ in reversed(find_emoji(text)):
      text = text[:start] + text[end:]
    missing.update(c for c in text if not c.isspace() and ord(c) not in coverage)
  assert not missing, repr(sorted(missing))


def test_timer_icon_and_long_numbers_stay_inside_bsm_margins(atlases):
  from openpilot.selfdrive.ui.mici.onroad.status_timers import TimerDisplay, format_mmss
  entries = metrics(atlases / "Pretendard-SemiBold.fnt")
  drawn = []
  class Label:
    def __init__(self, *args, **kwargs):
      pass
    def set_text(self, text):
      self.text = text
    def set_font_size(self, size):
      self.size = size
    def set_text_color(self, color):
      pass
    def render(self, rect):
      drawn.append(rect)

  def measure(_, text, size):
    return SimpleNamespace(x=sum(entries[ord(c)]["xadvance"] for c in text) * size * 1.16 / 200, y=size * 1.16)

  rl = SimpleNamespace(Rectangle=lambda x, y, width, height: SimpleNamespace(x=x, y=y, width=width, height=height),
                       Vector2=lambda x, y: SimpleNamespace(x=x, y=y), Color=lambda *args: args,
                       draw_rectangle_gradient_v=lambda *args: None,
                       draw_texture_ex=lambda icon, pos, *args: drawn.append(SimpleNamespace(x=pos.x, y=pos.y, width=icon.width, height=icon.height)))
  scope = {"rl": rl, "TimerDisplay": TimerDisplay, "format_mmss": format_mmss,
           "UnifiedLabel": Label, "TextAlignmentVertical": SimpleNamespace(MIDDLE=1),
           "FontWeight": SimpleNamespace(DISPLAY=0), "measure_text_cached": measure,
           "gui_app": SimpleNamespace(texture=lambda *args, **kwargs: SimpleNamespace(width=74, height=74), font=lambda _: None)}
  path = UI_DIR.parents[1] / "selfdrive/ui/mici/onroad/status_timer_renderer.py"
  tree = ast.parse(path.read_text())
  tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
  exec(compile(tree, str(path), "exec"), scope)
  renderer = scope["StatusTimerRenderer"]()
  for kind in ("parking", "hold"):
    for elapsed in (0, 59, 60, 5999, 360000, 3600000):
      drawn.clear()
      renderer.draw(rl.Rectangle(0, 0, 536, 240), TimerDisplay(kind, elapsed), 1.)
      icon, text = drawn
      assert icon.x >= 64
      assert icon.x + icon.width + 18 == pytest.approx(text.x)
      assert text.x + text.width <= 536 - 64 + 2
      assert 0 <= text.y <= text.y + text.height <= 240
