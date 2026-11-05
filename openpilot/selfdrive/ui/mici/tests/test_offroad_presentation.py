import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.offroad_presentation import DoorWake, ScreenFade, SurfaceFade, TripSummary, smoothstep


def test_greeting_trip_and_expiry():
  trip = TripSummary()
  assert not trip.visible(0)
  trip.start(100)
  for i in range(1, 101):
    trip.sample(100 + i / 10, 10)
  trip.finish(110)
  assert trip.distance_m == pytest.approx(100)
  assert trip.duration_s == 10
  assert trip.distance_text() == '0.1km'
  assert trip.duration_text() == '1분'
  assert trip.visible(709.99)
  assert not trip.visible(710)
  trip.start(720)
  assert trip.distance_m == 0
  assert not trip.visible(720)


@pytest.mark.parametrize('speed', [None, math.nan, math.inf, -20, 0])
def test_invalid_speed_does_not_add_distance(speed):
  trip = TripSummary()
  trip.start(0)
  trip.sample(0.1, speed)
  assert trip.distance_m == 0
  assert trip.duration_s == 0.1


def test_no_stale_final_speed_or_gap_distance():
  trip = TripSummary()
  trip.finish(50)
  assert not trip.visible(50)
  trip.start(100)
  trip.sample(101, 10)
  trip.sample(111, 40)
  trip.finish(3700)
  assert trip.distance_m == 10
  assert trip.duration_s == 3600
  assert trip.duration_text() == '1시간 0분'


def test_surface_fade_midpoint_completion_reversal_and_instant():
  fade = SurfaceFade()
  fade.change('onroad', 0)
  assert fade.state(0) == ('offroad', 0)
  assert fade.state(.275) == ('onroad', 1)
  assert fade.state(.55) == ('onroad', 0)
  assert fade.start_time is None
  fade.change('offroad', 1)
  fade.change('onroad', 1.1)
  assert fade.state(1.1)[0] == 'onroad'
  fade.change('offroad', 1.2, instant=True)
  assert fade.state(1.2) == ('offroad', 0)


def test_wake_and_sleep_fade():
  fade = ScreenFade()
  args = {'awake': True, 'offroad': True, 'sleep_at': 30}
  assert fade.alpha(28, **args) == 0
  assert fade.alpha(29, **args) == .5
  assert fade.alpha(30, **args) == 1
  fade.was_awake = False  # The timeout callback runs even when rendering stops.
  args['sleep_at'] = 70
  assert fade.alpha(40, **args) == 1
  assert fade.alpha(40.425, **args) == pytest.approx(.5)
  assert fade.alpha(40.85, **args) == pytest.approx(0, abs=1e-12)
  assert fade.alpha(69.5, awake=True, offroad=False, sleep_at=70) == 0
  assert fade.alpha(69.5, enabled=False, **args) == 0


@pytest.mark.parametrize('t, expected', [(-1, 0), (0, 0), (.5, .5), (1, 1), (2, 1)])
def test_easing_is_bounded(t, expected):
  assert smoothstep(t) == expected


def test_door_wakes_once_per_new_open_without_replaying_cached_value(tmp_path):
  path = tmp_path / 'door'
  path.write_bytes(b'1')
  wake = DoorWake(path)
  assert not wake.poll(offroad=True)
  path.write_bytes(b'0')
  assert not wake.poll(offroad=True)
  path.write_bytes(b'1')
  assert wake.poll(offroad=True)
  assert not wake.poll(offroad=True)
  path.unlink()
  assert not wake.poll(offroad=True)
  path.write_bytes(b'garbage')
  assert not wake.poll(offroad=True)
  path.write_bytes(b'1')
  assert not wake.poll(offroad=True)
  path.write_bytes(b'0')
  wake.poll(offroad=False)
  path.write_bytes(b'1')
  assert not wake.poll(offroad=False)
  assert not wake.poll(offroad=True)


def test_first_door_event_after_reboot_is_not_lost(tmp_path):
  path = tmp_path / 'door'
  wake = DoorWake(path)
  assert not wake.poll(offroad=True)
  path.write_bytes(b'1')
  assert wake.poll(offroad=True)
  assert not wake.poll(offroad=True)


def main_class(ns):
  # Exercise the actual transition handlers without creating cameras or a window.
  path = Path(__file__).parents[1] / 'layouts/main.py'
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MiciMainLayout')
  ns.update(Scroller=object, Widget=object, ONROAD_DELAY=2.5)
  exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), ns)
  return ns['MiciMainLayout']


@pytest.fixture
def layout():
  clock = NS(now=100.0)
  cs = NS(standstill=True, canValid=True, canTimeout=False, vEgoCluster=12, vEgo=10)
  sm = {'carState': cs}

  class Messages(dict):
    valid = {'carState': True}
    alive = {'carState': True}
    recv_frame = {'carState': 100}
    recv_time = {'carState': 100}
    logMonoTime = {'carState': 100_000_000_000}

  ui = NS(started=False, ignition=False, engaged=False, sm=Messages(sm), started_frame=100,
          params=NS(get_int=lambda _: 0, get_bool=lambda _: False), show_camera_with_cluster=False)
  device = NS(awake=True, _interaction_time=130, add_interactive_timeout_callback=lambda _: None)
  gui = NS(widget_in_stack=lambda _: False, pop_widgets_to=lambda obj, callback=None, **kw: callback() if callback else None)
  cls = main_class({'time': NS(monotonic=lambda: clock.now), 'rl': NS(get_time=lambda: clock.now),
                    'gui_app': gui, 'ui_state': ui, 'device': device, 'PC': False})
  obj = object.__new__(cls)
  obj._screen_fade = ScreenFade()
  obj._trip = TripSummary()
  obj._surface = SurfaceFade()
  obj._setup = True
  obj._prev_onroad = False
  obj._prev_standstill = True
  obj._prev_active_alerts = 0
  obj._onroad_time_delay = None
  obj._in_plot_mode = False
  obj._show_plot_mode = 0
  obj._onboarding_window = object()
  obj._home_layout = NS(rect=NS(x=0), set_callbacks=lambda **kw: None)
  obj._info_layout = NS(set_callbacks=lambda **kw: None)
  obj._onroad_layout = NS(set_cluster_hud_connected=lambda *args: None)
  obj._debug_layout = object()
  obj._alerts_layout = NS(active_alerts=lambda: 0)
  obj._scroller = NS(scroll_to=lambda *args, **kw: None)
  obj._sync_offroad_scroll = lambda **kw: None
  return obj, ui, clock


def test_short_ignition_cycle_cancels_pending_onroad_and_plot(layout):
  obj, ui, clock = layout
  ui.started = ui.ignition = True
  obj._handle_transitions()
  assert obj._onroad_time_delay == 100
  obj._in_plot_mode = True
  obj._surface.change('plot', clock.now, instant=True)
  clock.now += 1
  ui.started = ui.ignition = False
  obj._handle_transitions()
  assert obj._onroad_time_delay is None
  assert not obj._in_plot_mode
  assert obj._surface.target == 'offroad'
  clock.now += 3
  obj._handle_transitions()
  assert obj._surface.target == 'offroad'
  assert obj._trip.visible(clock.now)


def test_stale_standstill_edge_cannot_open_onroad_while_offroad(layout):
  obj, ui, _ = layout
  ui.sm['carState'].standstill = False
  obj._handle_transitions()
  assert obj._surface.target == 'offroad'


def test_movement_cancels_onroad_blackout(layout):
  obj, ui, clock = layout
  ui.started = ui.ignition = True
  obj._handle_transitions()
  clock.now += 3
  obj._handle_transitions()
  assert obj._surface.start_time is not None
  ui.sm['carState'].standstill = False
  obj._handle_transitions()
  assert obj._surface.state(clock.now) == ('onroad', 0)


@pytest.mark.parametrize('standstill', [False, True])
def test_no_onroad_tap_callback_that_hides_alerts(layout, standstill):
  obj, ui, _ = layout
  ui.started = ui.engaged = True
  ui.sm['carState'].standstill = standstill
  taps = []
  obj._onroad_layout.set_click_callback = taps.append
  obj._setup_callbacks()
  assert taps == []


def test_moving_plot_switch_never_blacks_out(layout):
  obj, ui, clock = layout
  ui.started = True
  ui.sm['carState'].standstill = False
  obj._scroll_to(obj._debug_layout)
  assert obj._surface.state(clock.now) == ('plot', 0)
  obj._in_plot_mode = False
  obj._scroll_to(obj._onroad_layout)
  assert obj._surface.state(clock.now) == ('onroad', 0)


def test_timeout_arms_wake_and_does_not_block_summary(layout):
  obj, _, clock = layout
  obj._trip.start(clock.now - 20)
  obj._trip.finish(clock.now)
  obj._on_interactive_timeout()
  assert not obj._screen_fade.was_awake
  assert obj._trip.visible(clock.now)
  assert obj._surface.state(clock.now) == ('offroad', 0)


@pytest.mark.parametrize('field,value', [('valid', False), ('alive', False), ('recv_frame', 99),
                                         ('recv_time', 98), ('logMonoTime', 98_000_000_000)])
def test_summary_rejects_stale_messages(layout, field, value):
  obj, ui, clock = layout
  obj._trip.start(99.9)
  getattr(ui.sm, field)['carState'] = value
  obj._sample_trip(clock.now)
  assert obj._trip.distance_m == 0


def test_summary_uses_actual_cluster_speed(layout):
  obj, _, clock = layout
  obj._trip.start(99.9)
  obj._sample_trip(clock.now)
  assert obj._trip.distance_m == pytest.approx(1.2)


def test_device_door_wake_runs_without_widget_rendering(tmp_path):
  path = Path(__file__).parents[2] / 'ui_state.py'
  cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'Device')
  now = NS(value=100.0)
  ui = NS(started=False, ignition=False)
  ns = {'time': NS(monotonic=lambda: now.value), 'ui_state': ui,
        'gui_app': NS(mouse_events=[], big_ui=lambda: False), 'PC': False, 'Callable': object}
  exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), ns)
  device = object.__new__(ns['Device'])
  door = tmp_path / 'door'
  device._door_wake = DoorWake(door)
  device._ignition = False
  device._interaction_time = 90
  device._override_interactive_timeout = None
  device._prev_timed_out = False
  device._interactive_timeout_callbacks = []
  powers = []
  device._set_awake = powers.append
  door.write_bytes(b'0')
  device._update_wakefulness()
  assert powers[-1] is False
  door.write_bytes(b'1')
  device._update_wakefulness()
  assert powers[-1] is True
  assert device._interaction_time == 130
  now.value = 129
  device._update_wakefulness()
  assert device._interaction_time == 130
  now.value = 131
  device._update_wakefulness()
  assert powers[-1] is False  # An open door does not keep the screen on forever.


def test_sleep_fade_cancels_on_new_timeout_or_ignition():
  fade = ScreenFade()
  assert fade.alpha(29.5, awake=True, offroad=True, sleep_at=30) > .8
  assert fade.alpha(29.5, awake=True, offroad=True, sleep_at=59.5) == 0
  assert fade.alpha(29.5, awake=True, offroad=False, sleep_at=30) == 0
