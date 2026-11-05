import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.deceleration_source import external_navigation_connected, navigation_status_presentation


MICI_HUD_PATH = Path(__file__).resolve().parents[1] / "mici" / "onroad" / "hud_renderer.py"


def _draw_set_speed_source() -> str:
  tree = ast.parse(MICI_HUD_PATH.read_text(encoding="utf-8"))
  hud = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "HudRenderer")
  method = next(node for node in hud.body if isinstance(node, ast.FunctionDef) and node.name == "_draw_set_speed")
  return ast.unparse(method)


def test_mici_navigation_status_is_shifted_one_character_left() -> None:
  source = _draw_set_speed_source()
  assert "panel_x + panel_w * 0.6 - 26" in source


def test_mici_navigation_status_uses_source_specific_colors() -> None:
  source = _draw_set_speed_source()
  assert "Color(199, 125, 255, 230)" in source
  assert "Color(244, 172, 54, 230)" in source


@pytest.fixture
def navigation_badge():
  # Execute the real draw method; isolate only graphics and unrelated controls.
  tree = ast.parse(MICI_HUD_PATH.read_text(encoding="utf-8"))
  hud = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "HudRenderer")
  method = next(node for node in hud.body if isinstance(node, ast.FunctionDef) and node.name == "_draw_set_speed")
  draws = []
  ui = SimpleNamespace(is_metric=True)
  raylib = SimpleNamespace(
    Rectangle=lambda *args: args, Color=lambda *args: args, Vector2=lambda *args: args,
    WHITE=(255, 255, 255, 255), draw_texture=lambda *args: None,
    draw_rectangle_rounded=lambda *args: None, draw_rectangle_rounded_lines_ex=lambda *args: None,
    draw_text_ex=lambda *args: None,
  )
  scope = {
    "rl": raylib, "ui_state": ui,
    "external_navigation_connected": external_navigation_connected,
    "navigation_status_presentation": navigation_status_presentation,
    "draw_text_ui_style": lambda text, x, y, size, color, **kwargs: draws.append((text, color)),
    "measure_text_cached": lambda *args: SimpleNamespace(x=10, y=10),
  }
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(MICI_HUD_PATH), "exec"), scope)
  renderer = SimpleNamespace(
    _set_speed_override=SimpleNamespace(compute=lambda *args: SimpleNamespace(active=False)),
    _txt_speed_bg=SimpleNamespace(width=420, height=220), _debug_speed_panel=False,
    _font_display=None, _font_semi_bold=None, _engaged=False, is_cruise_set=False, set_speed=0, speed=0,
    _get_driving_mode_text_and_color=lambda: ("", None), _get_cruise_gap=lambda: 1, _get_gear_text=lambda: "P",
  )

  def draw(sm):
    ui.sm = sm
    draws.clear()
    scope["_draw_set_speed"](renderer, SimpleNamespace(x=0, y=0, height=600))
    return [(text, color) for text, color in draws if text in ("NAVI", "vNAVI")]

  return draw


class NavigationSubMaster(dict):
  def __init__(self, **carrot_man):
    super().__init__(carrotMan=SimpleNamespace(**carrot_man), carrotNavi=SimpleNamespace(connected=False))
    self.alive = {"carrotMan": True, "carrotNavi": True}
    self.valid = dict(self.alive)


def test_navdy_badge_follows_fresh_control_without_acc_or_network(navigation_badge):
  sm = NavigationSubMaster(externalNavigationSource="navdy", externalNavigationActive=True, remote="", activeCarrot=3)
  assert navigation_badge(sm) == [("NAVI", (244, 172, 54, 230))]
  sm["carrotMan"].externalNavigationActive = False
  assert navigation_badge(sm) == []
  sm["carrotMan"].externalNavigationActive = True
  assert navigation_badge(sm) == [("NAVI", (244, 172, 54, 230))]


@pytest.mark.parametrize("vehicle_available", [False, True])
@pytest.mark.parametrize("condition", ["inactive", "dead", "invalid", "missing_health"])
def test_navdy_unavailable_never_uses_old_network_inputs(navigation_badge, condition, vehicle_available):
  sm = NavigationSubMaster(
    externalNavigationSource="navdy", externalNavigationActive=True, remote="192.0.2.1", activeCarrot=3,
    vehicleNaviAvailable=vehicle_available,
  )
  sm["carrotNavi"].connected = True
  if condition == "inactive":
    sm["carrotMan"].externalNavigationActive = False
  elif condition == "dead":
    sm.alive["carrotMan"] = False
  elif condition == "invalid":
    sm.valid["carrotMan"] = False
  else:
    sm.alive.pop("carrotMan")
  assert navigation_badge(sm) == ([("vNAVI", (199, 125, 255, 230))] if vehicle_available else [])


@pytest.mark.parametrize("source", [None, "network"])
@pytest.mark.parametrize("input_kind", ["remote", "carrotNavi"])
def test_legacy_network_badge_requires_healthy_input(navigation_badge, source, input_kind):
  sm = NavigationSubMaster(remote="192.0.2.1" if input_kind == "remote" else "")
  if source is not None:
    sm["carrotMan"].externalNavigationSource = source
    sm["carrotMan"].externalNavigationActive = False
  sm["carrotNavi"].connected = input_kind == "carrotNavi"
  assert navigation_badge(sm) == [("NAVI", (244, 172, 54, 230))]
  service = "carrotMan" if input_kind == "remote" else "carrotNavi"
  sm.alive[service] = False
  assert navigation_badge(sm) == []
  sm.alive[service] = True
  sm.valid[service] = False
  assert navigation_badge(sm) == []


def test_network_active_flag_does_not_need_legacy_peers(navigation_badge):
  sm = NavigationSubMaster(externalNavigationSource="network", externalNavigationActive=True)
  assert navigation_badge(sm) == [("NAVI", (244, 172, 54, 230))]
  sm.valid["carrotMan"] = False
  assert navigation_badge(sm) == []


@pytest.mark.parametrize("source", [None, "network", "navdy"])
def test_vehicle_navigation_only_shows_vehicle_badge(navigation_badge, source):
  sm = NavigationSubMaster(activeCarrot=3, desiredSource="hda", vehicleNaviAvailable=True)
  if source is not None:
    sm["carrotMan"].externalNavigationSource = source
    sm["carrotMan"].externalNavigationActive = False
  assert navigation_badge(sm) == [("vNAVI", (199, 125, 255, 230))]
