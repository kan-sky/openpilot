from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.mici.tests.test_vision_renderer import mici_modules as mici_modules
from openpilot.selfdrive.ui.tests.test_carrot_model_renderer_lane_visibility import model_renderer_module as model_renderer_module


@pytest.fixture
def lane_renderer(mici_modules, monkeypatch):
  module, _ = mici_modules
  renderer = object.__new__(module.ModelRenderer)
  renderer._rect = module.rl.Rectangle(0, 0, 536, 240)
  renderer._lane_lines = [module.ModelPoints(projected_points=np.full((4, 2), i, dtype=np.float32)) for i in range(4)]
  renderer._road_edges = [module.ModelPoints(projected_points=np.full((4, 2), i + 4, dtype=np.float32)) for i in range(2)]
  renderer._lane_line_probs = np.array([0.8, 0.3, 0.6, 0.1], dtype=np.float32)
  renderer._road_edge_stds = np.array([0.2, 0.8], dtype=np.float32)
  renderer._lane_marking_codes = [-1, 24, 0, -1]
  renderer._lane_marking_segments = [[], [np.ones((4, 2)) * 20], [np.ones((4, 2)) * 30], []]
  renderer._torque_filter = SimpleNamespace(x=0)
  state = SimpleNamespace(new_ui=True, status=module.UIStatus.ENGAGED, lat_active=True)
  monkeypatch.setattr(module, 'ui_state', state)
  calls = []
  monkeypatch.setattr(module, 'draw_polygon', lambda _rect, points, color: calls.append((points, (color.r, color.g, color.b, color.a))))
  return module, renderer, state, calls


@pytest.mark.parametrize('status', ['DISENGAGED', 'ENGAGED', 'OVERRIDE'])
@pytest.mark.parametrize('prob', [0.0, 0.249, 0.299, 0.3, 0.301, 0.7, 1.0])
@pytest.mark.parametrize('code', [-1, 0, 1, 24])
def test_new_ui_is_white_and_continuous_across_types_confidence_and_status(lane_renderer, status, prob, code):
  module, renderer, state, calls = lane_renderer
  state.status = getattr(module.UIStatus, status)
  state.lat_active = status != 'DISENGAGED'
  renderer._lane_line_probs[:] = prob
  renderer._lane_marking_codes = [code] * 4
  renderer._torque_filter.x = 0.9
  renderer._draw_lane_lines()
  assert len(calls) == 6
  expected_alpha = int(np.clip(renderer._lane_line_probs[0], 0.0, 0.7) * 255)
  for i, (points, color) in enumerate(calls):
    assert points is (renderer._lane_lines[i] if i < 4 else renderer._road_edges[i - 4]).projected_points
    assert color[:3] == (255, 255, 255)
    if i < 4:
      assert color[3] == expected_alpha
  assert calls[4][1][3] == 178
  assert calls[5][1][3] == 50


def test_confidence_changes_only_opacity_and_does_not_modify_input(lane_renderer):
  _, renderer, _, calls = lane_renderer
  original_codes = renderer._lane_marking_codes.copy()
  original_segments = renderer._lane_marking_segments.copy()
  alpha = []
  for prob in (0, 0.1, 0.299, 0.301, 0.5, 0.7, 0.9):
    renderer._lane_line_probs[:] = prob
    before = renderer._lane_line_probs.copy()
    renderer._draw_lane_lines()
    alpha.append(calls[-6][1][3])
    np.testing.assert_array_equal(before, renderer._lane_line_probs)
  assert alpha == sorted(alpha)
  assert alpha[0] == 0 and alpha[-1] == 178
  assert abs(alpha[2] - alpha[3]) <= 1
  assert renderer._lane_marking_codes == original_codes
  assert all(a is b for a, b in zip(renderer._lane_marking_segments, original_segments, strict=True))


def test_empty_geometry_stays_absent(lane_renderer):
  _, renderer, _, calls = lane_renderer
  for line in renderer._lane_lines + renderer._road_edges:
    line.projected_points = np.empty((0, 2), dtype=np.float32)
  renderer._draw_lane_lines()
  assert not calls


def test_switching_off_restores_legacy_markings_and_colors(lane_renderer):
  _, renderer, state, calls = lane_renderer
  renderer._lane_line_probs[:] = 0.9
  renderer._draw_lane_lines()
  white = calls.copy()
  calls.clear()
  state.new_ui = False
  renderer._draw_lane_lines()
  assert calls[1][0] is renderer._lane_marking_segments[1][0]
  assert calls[1][1] == (218, 202, 37, 220)
  assert calls[2][0] is renderer._lane_marking_segments[2][0]
  assert calls[2][1] == (255, 255, 255, 220)
  renderer._lane_line_probs[1:3] = 0.2
  calls.clear()
  renderer._draw_lane_lines()
  assert calls[1][1][:3] == (0, 255, 64)
  assert calls[2][1][:3] == (0, 255, 64)
  assert calls[4][1][:3] == (0, 255, 64)
  renderer._lane_line_probs[:] = 0.9
  state.new_ui = True
  calls.clear()
  renderer._draw_lane_lines()
  assert all(a[0] is b[0] and a[1] == b[1] for a, b in zip(calls, white, strict=True))
