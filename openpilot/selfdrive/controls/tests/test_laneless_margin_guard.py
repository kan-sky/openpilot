import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import get_laneless_margin_guard


PATH_T = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 3.0])
PATH_X = np.array([0.0, 4.0, 9.0, 15.0, 22.0, 35.0])
LANE_X = PATH_X.copy()
LEFT_Y = np.full_like(LANE_X, 1.5)
RIGHT_Y = np.full_like(LANE_X, -1.5)


def test_left_guard_reduces_positive_curvature_for_inside_intrusion():
  path_y = np.array([0.0, 0.8, 1.2, 1.2, 1.2, 1.2])
  result = get_laneless_margin_guard(PATH_T, PATH_X, path_y, LANE_X, LEFT_Y, LANE_X, RIGHT_Y, 0.9, 0.9, 0.002)

  assert result.active
  assert np.isclose(result.min_inside_clearance, 0.3)
  assert result.curvature < 0.002


def test_right_guard_reduces_negative_curvature_for_inside_intrusion():
  path_y = np.array([0.0, -0.8, -1.2, -1.2, -1.2, -1.2])
  result = get_laneless_margin_guard(PATH_T, PATH_X, path_y, LANE_X, LEFT_Y, LANE_X, RIGHT_Y, 0.9, 0.9, -0.002)

  assert result.active
  assert np.isclose(result.min_inside_clearance, 0.3)
  assert result.curvature > -0.002


def test_guard_does_not_activate_when_clearance_is_sufficient_or_lane_probability_is_low():
  centered_path = np.zeros_like(PATH_X)
  result = get_laneless_margin_guard(PATH_T, PATH_X, centered_path, LANE_X, LEFT_Y, LANE_X, RIGHT_Y, 0.9, 0.9, 0.002)
  low_prob_result = get_laneless_margin_guard(PATH_T, PATH_X, np.full_like(PATH_X, 1.2), LANE_X, LEFT_Y,
                                               LANE_X, RIGHT_Y, 0.4, 0.9, 0.002)

  assert not result.active
  assert np.isclose(result.min_inside_clearance, 1.5)
  assert not low_prob_result.active


def test_guard_rejects_invalid_lane_geometry():
  path_y = np.full_like(PATH_X, 1.2)
  result = get_laneless_margin_guard(PATH_T, PATH_X, path_y, LANE_X, np.full_like(LANE_X, 1.0),
                                     LANE_X, np.full_like(LANE_X, 0.0), 0.9, 0.9, 0.002)

  assert not result.active
  assert result.min_inside_clearance is None
