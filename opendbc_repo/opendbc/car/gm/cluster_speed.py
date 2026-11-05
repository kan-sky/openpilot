from bisect import bisect_right

from opendbc.car.common.conversions import Conversions as CV


# Wayon/Sunnypilot b7660d1d40: measured on the user's Traverse.
# Each tuple is (cluster_display_kph, openpilot_raw_display_kph).
# Keep this display calibration out of wheel speeds, vEgo and acceleration.
_CLUSTER_TO_RAW_DISPLAY_TABLE = (
  (25.0, 24.0),
  (26.0, 25.0),
  (27.0, 25.0),
  (28.0, 27.0),
  (29.0, 28.0),
  (30.0, 28.0),
  (31.0, 29.0),
  (32.0, 30.0),
  (33.0, 31.0),
  (34.0, 32.0),
  (35.0, 33.0),
  (36.0, 34.0),
  (37.0, 35.0),
  (38.0, 36.0),
  (39.0, 36.0),
  (40.0, 37.0),
  (41.0, 38.0),
  (42.0, 39.0),
  (43.0, 40.0),
  (44.0, 41.0),
  (45.0, 42.0),
  (46.0, 43.0),
  (47.0, 44.0),
  (48.0, 45.0),
  (49.0, 46.0),
  (50.0, 47.0),
  (55.0, 52.0),
  (60.0, 56.0),
  (65.0, 61.0),
  (70.0, 66.0),
  (75.0, 71.0),
  (80.0, 76.0),
  (85.0, 81.0),
  (90.0, 86.0),
  (95.0, 90.0),
  (100.0, 95.0),
  (105.0, 100.0),
  (110.0, 105.0),
  (115.0, 110.0),
  (120.0, 115.0),
  (125.0, 120.0),
  (130.0, 124.0),
  (135.0, 129.0),
  (140.0, 134.0),
  (145.0, 139.0),
  (150.0, 144.0),
  (155.0, 149.0),
  (160.0, 154.0),
  (165.0, 158.0),
  (170.0, 163.0),
  (175.0, 168.0),
  (180.0, 173.0),
)


def _build_raw_to_cluster_table():
  raw_to_cluster = {}
  for cluster_kph, raw_kph in _CLUSTER_TO_RAW_DISPLAY_TABLE:
    raw_to_cluster[raw_kph] = min(cluster_kph, raw_to_cluster.get(raw_kph, cluster_kph))
  return tuple(sorted(raw_to_cluster.items()))


_RAW_TO_CLUSTER_DISPLAY_TABLE = _build_raw_to_cluster_table()
_CLUSTER_TO_RAW_X = tuple(point[0] for point in _CLUSTER_TO_RAW_DISPLAY_TABLE)
_CLUSTER_TO_RAW_Y = tuple(point[1] for point in _CLUSTER_TO_RAW_DISPLAY_TABLE)
_RAW_TO_CLUSTER_X = tuple(point[0] for point in _RAW_TO_CLUSTER_DISPLAY_TABLE)
_RAW_TO_CLUSTER_Y = tuple(point[1] for point in _RAW_TO_CLUSTER_DISPLAY_TABLE)


def _interp_with_extrapolation(x_points: tuple[float, ...], y_points: tuple[float, ...], x: float) -> float:
  if not x_points:
    return x
  if len(x_points) == 1:
    return y_points[0]

  if x <= x_points[0]:
    left_idx = 0
  elif x >= x_points[-1]:
    left_idx = len(x_points) - 2
  else:
    left_idx = bisect_right(x_points, x) - 1

  right_idx = left_idx + 1
  x0 = x_points[left_idx]
  x1 = x_points[right_idx]
  y0 = y_points[left_idx]
  y1 = y_points[right_idx]
  if x1 == x0:
    return max(y0, y1)
  t = (x - x0) / (x1 - x0)
  return y0 + (y1 - y0) * t


def gm_cluster_display_kph_from_raw_display_kph(raw_display_kph: float) -> float:
  if raw_display_kph <= 0.0:
    return raw_display_kph
  return _interp_with_extrapolation(_RAW_TO_CLUSTER_X, _RAW_TO_CLUSTER_Y, raw_display_kph)


def gm_raw_display_kph_from_cluster_display_kph(cluster_display_kph: float) -> float:
  if cluster_display_kph <= 0.0:
    return cluster_display_kph
  return _interp_with_extrapolation(_CLUSTER_TO_RAW_X, _CLUSTER_TO_RAW_Y, cluster_display_kph)


def gm_cluster_cruise_speed_from_raw_ms(raw_speed_ms: float) -> float:
  if raw_speed_ms <= 0.0:
    return raw_speed_ms
  raw_display_kph = raw_speed_ms * CV.MS_TO_KPH
  return gm_cluster_display_kph_from_raw_display_kph(raw_display_kph) * CV.KPH_TO_MS


def gm_raw_cruise_speed_from_cluster_ms(cluster_speed_ms: float) -> float:
  if cluster_speed_ms <= 0.0:
    return cluster_speed_ms
  cluster_display_kph = cluster_speed_ms * CV.MS_TO_KPH
  return gm_raw_display_kph_from_cluster_display_kph(cluster_display_kph) * CV.KPH_TO_MS
