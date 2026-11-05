import math

from openpilot.selfdrive.carrot.cluster.cluster_config import cluster_theme_brightness_percent, kst_clock_text_12h
from openpilot.selfdrive.carrot.cluster.cluster_layout import (
  ambient_bsm_edges,
  ambient_power_gauge,
  extrapolated_media_position_ms,
  fitted_text_size,
  leftward_loop_marquee_offset,
  smooth_bsm_opacity,
  smooth_ambient_accel,
)


def test_cluster_theme_brightness_uses_fixed_dark_and_light_levels():
  assert cluster_theme_brightness_percent("dark") == 20
  assert cluster_theme_brightness_percent("light") == 60


def test_cluster_theme_brightness_auto_switches_on_kst_schedule():
  assert cluster_theme_brightness_percent("auto", 1_767_301_140) == 20  # 2026-01-02 05:59 KST
  assert cluster_theme_brightness_percent("auto", 1_767_301_200) == 60  # 2026-01-02 06:00 KST
  assert cluster_theme_brightness_percent("auto", 1_767_344_340) == 60  # 2026-01-02 17:59 KST
  assert cluster_theme_brightness_percent("auto", 1_767_344_400) == 20  # 2026-01-02 18:00 KST


def test_kst_clock_text_12h_omits_period_and_leading_hour_zero():
  assert kst_clock_text_12h(now=0) == "9:00"
  assert kst_clock_text_12h(now=3 * 60 * 60) == "12:00"
  assert kst_clock_text_12h(now=5 * 60 * 60 + 24 * 60) == "2:24"
  assert kst_clock_text_12h(now=15 * 60 * 60 + 5 * 60) == "12:05"


def test_ambient_power_gauge_maps_accel_to_signed_fill():
  assert ambient_power_gauge(0.0) == ("idle", 0.0)
  assert ambient_power_gauge(2.5) == ("power", 0.5)
  assert ambient_power_gauge(5.0) == ("power", 1.0)
  assert ambient_power_gauge(-10.0) == ("brake", 1.0)


def test_smooth_ambient_accel_filters_noise_and_uses_deadband():
  assert smooth_ambient_accel(0.0, 0.05, 0.1) == 0.0
  first = smooth_ambient_accel(0.0, 2.0, 0.1)
  second = smooth_ambient_accel(first, 2.0, 0.1)
  assert 0.0 < first < second < 2.0
  assert math.isfinite(smooth_ambient_accel(float("nan"), None, 0.1))


def test_ambient_bsm_edges_keeps_left_and_right_independent():
  assert ambient_bsm_edges(False, False) == (False, False)
  assert ambient_bsm_edges(True, False) == (True, False)
  assert ambient_bsm_edges(False, True) == (False, True)


def test_smooth_bsm_opacity_fades_in_and_out_over_configured_duration():
  assert smooth_bsm_opacity(0.0, True, 0.175, 0.35) == 0.5
  assert smooth_bsm_opacity(0.5, True, 0.175, 0.35) == 1.0
  assert smooth_bsm_opacity(1.0, False, 0.0875, 0.35) == 0.75
  assert smooth_bsm_opacity(0.25, False, 0.175, 0.35) == 0.0


def test_extrapolated_media_position_advances_only_while_playing():
  assert extrapolated_media_position_ms(10_000, 60_000, True, 100_000, 102_500) == 12_500
  assert extrapolated_media_position_ms(10_000, 60_000, False, 100_000, 102_500) == 10_000
  assert extrapolated_media_position_ms(59_000, 60_000, True, 100_000, 102_500) == 60_000
  assert extrapolated_media_position_ms(None, 60_000, True, 100_000, 102_500) is None


def test_fitted_text_size_shrinks_only_to_configured_minimum():
  assert fitted_text_size(200.0, 274.0, 49.0, 34.0) == 49.0
  assert fitted_text_size(392.0, 274.0, 49.0, 34.0) == 34.25
  assert fitted_text_size(800.0, 274.0, 49.0, 34.0) == 34.0


def test_leftward_loop_marquee_only_moves_right_to_left():
  assert leftward_loop_marquee_offset(1.0, 120.0, 30.0, 1.5) == 0.0
  assert leftward_loop_marquee_offset(2.5, 120.0, 30.0, 1.5) == -30.0
  assert leftward_loop_marquee_offset(5.0, 120.0, 30.0, 1.5) == -105.0
  assert leftward_loop_marquee_offset(5.5, 120.0, 30.0, 1.5) == 0.0
  assert leftward_loop_marquee_offset(7.5, 120.0, 30.0, 1.5) == -15.0
