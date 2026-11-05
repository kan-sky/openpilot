"""C4 font selection/coverage without loading a window or device services."""
import ast
import re
import sys
from enum import StrEnum
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.system.ui.lib.tests.test_raylib_text import UI_DIR, load_definitions


def application_scope(big=False):
  scope = {"BIG_UI": big, "StrEnum": StrEnum, "rl": SimpleNamespace(Font=object), "Path": Path,
           "FONT_SOURCE_EXTS": (".ttf", ".otf")}
  path = UI_DIR / "lib/application.py"
  load_definitions(path, scope, {"FontWeight", "font_fallback"})
  tree = ast.parse(path.read_text())
  app = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "GuiApplication")
  names = {"font_supports_text", "_resolve_font_path", "_font_codepoints", "_load_font_path"}
  methods = [n for n in app.body if isinstance(n, ast.FunctionDef) and n.name in names]
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), scope)
  return scope


def font(texture_id, chars, empty=""):
  chars = sorted(set(chars) | set(empty))
  return SimpleNamespace(texture=SimpleNamespace(id=texture_id), glyphCount=len(chars),
                         glyphs=[SimpleNamespace(value=ord(c), advanceX=0 if c in empty else 10) for c in chars],
                         recs=[SimpleNamespace(width=0 if c in empty else 10, height=0 if c in empty else 10) for c in chars])


@pytest.fixture
def fonts(monkeypatch):
  scope = application_scope()
  weights = scope["FontWeight"]
  latin = "".join(map(chr, range(32, 127))) + "é°–"
  normal = font(1, latin, empty="한글힣가⚙中")
  korean = font(2, latin + "한글힣ㄱㅏ", empty="가⚙中")
  unifont = font(3, latin + "한글힣ㄱㅏ가⚙中\ufffd")
  loaded = {weights.NORMAL: normal, weights.KOREAN: korean, weights.UNIFONT: unifont}
  app = SimpleNamespace(_font_coverage={}, has_font=lambda w: w in loaded, font=loaded.__getitem__)
  app.font_supports_text = lambda f, t: scope["font_supports_text"](app, f, t)
  scope["gui_app"] = app
  monkeypatch.setitem(sys.modules, "openpilot.system.ui.lib.emoji", SimpleNamespace(
    find_emoji=lambda t: [(m.start(), m.end(), m.group()) for m in re.finditer("🙂", t)]))
  return SimpleNamespace(**scope, normal=normal, korean=korean, unifont=unifont, loaded=loaded)


def test_c4_all_text_roles_share_semibold_and_load_only_three_atlases():
  weights = application_scope()["FontWeight"]
  for role in ("NORMAL", "MEDIUM", "BOLD", "SEMI_BOLD", "PRETENDARD", "DISPLAY", "DISPLAY_REGULAR", "ROMAN"):
    assert getattr(weights, role) == "Pretendard-SemiBold.fnt"
  assert {w.value for w in weights} == {"Pretendard-SemiBold.fnt", "Pretendard-KR-SemiBold.fnt", "unifont.fnt"}


def test_big_ui_keeps_existing_fonts():
  w = application_scope(big=True)["FontWeight"]
  assert w.NORMAL == w.ROMAN == w.DISPLAY_REGULAR == "Inter-Regular.fnt"
  assert w.MEDIUM == "Inter-Medium.fnt"
  assert w.SEMI_BOLD == "Inter-SemiBold.fnt"
  assert w.BOLD == "Inter-Bold.fnt"
  assert w.DISPLAY == w.KOREAN == "KaiGenGothicKR-Bold.fnt"


@pytest.mark.parametrize("text,target", [
  ("100 MAX 00:00", "normal"), ("Café 35°", "normal"), ("한글 100", "korean"),
  ("힣 ㄱㅏ", "korean"), ("가", "unifont"), ("한글 ⚙", "unifont"), ("中", "unifont"),
  ("한글\n100\t", "korean"), ("AB🙂한글", "korean"), ("🙂", "normal"),
  ("한글".encode(), "korean"), (b"\xff", "unifont"), (None, "normal"), ("", "normal"),
])
def test_actual_coverage_not_codepoint_presence_selects_font(fonts, text, target):
  assert fonts.font_fallback(fonts.normal, text) is getattr(fonts, target)


def test_wrapped_elided_and_emoji_fragments_keep_the_selected_font(fonts):
  for whole, chosen in [("ABCDEFG한글", fonts.korean), ("AB🙂한글", fonts.korean), ("AB⚙", fonts.unifont)]:
    selected = fonts.font_fallback(fonts.normal, whole)
    assert selected is chosen
    for fragment in ("AB", "CD", "...", ""):
      assert fonts.font_fallback(selected, fragment) is selected


def test_coverage_cached_and_bootstrap_missing_fallbacks_safe(fonts):
  assert not fonts.gui_app.font_supports_text(fonts.normal, "한글")
  coverage = fonts.gui_app._font_coverage[1]
  assert fonts.gui_app.font_supports_text(fonts.normal, "A\n\t")
  assert fonts.gui_app._font_coverage[1] is coverage
  fonts.loaded.pop(fonts.FontWeight.KOREAN)
  fonts.loaded.pop(fonts.FontWeight.UNIFONT)
  assert fonts.font_fallback(fonts.normal, "한글") is fonts.normal


def test_korean_atlas_uses_existing_source_without_an_extra_font_binary(tmp_path):
  scope = application_scope()
  source = tmp_path / "Pretendard-SemiBold.ttf"
  source.touch()
  atlas = tmp_path / "Pretendard-KR-SemiBold.fnt"
  assert scope["_resolve_font_path"](None, atlas) == source
  atlas.touch()
  assert scope["_resolve_font_path"](None, atlas) == atlas


def test_source_fallback_sizes_and_hangul_are_bounded():
  scope = application_scope()
  w = scope["FontWeight"]
  app = SimpleNamespace(_font_codepoints=lambda weight: scope["_font_codepoints"](None, weight))
  calls = []
  scope["rl"].ffi = SimpleNamespace(new=lambda _, v: v, cast=lambda _, v: v)
  scope["rl"].load_font_ex = lambda path, size, cps, count: calls.append((size, set(cps), count))
  for weight in (w.NORMAL, w.KOREAN, w.UNIFONT):
    scope["_load_font_path"](app, Path("Pretendard-SemiBold.ttf"), weight)
  assert [c[0] for c in calls] == [200, 48, 16]
  hangul = set(range(0xAC00, 0xD7A4))
  assert not hangul & calls[0][1]
  assert hangul <= calls[1][1]
  assert len(calls[1][1]) < 12000
  assert {0x1100, 0x1161, 0x3131, 0x4E2D} <= calls[2][1]


def test_generator_and_scons_include_the_extra_atlas():
  generator = (UI_DIR.parents[1] / "selfdrive/assets/fonts/process_korean.py").read_text()
  build = (UI_DIR.parents[1] / "selfdrive/ui/SConscript").read_text()
  assert 'stem = "Pretendard-KR-SemiBold"' in generator
  assert "alias.symlink_to(source.resolve())" in generator
  assert 'action=f"python3 {korean_generator}"' in build
  assert "Pretendard-KR-SemiBold.fnt" in build
  assert "Pretendard-KR-SemiBold.png" in build


@pytest.mark.parametrize("name", ["preview_offroad.py", "preview_new_ui.py"])
def test_korean_ui_previews_load_fallback_atlases(name):
  path = UI_DIR.parents[1] / "selfdrive/ui/mici/tests" / name
  tree = ast.parse(path.read_text())
  call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == "init_window")
  weights = next(k.value.elts for k in call.keywords if k.arg == "font_weights")
  assert {w.attr for w in weights} >= {"DISPLAY", "KOREAN", "UNIFONT"}
