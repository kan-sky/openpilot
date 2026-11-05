"""Display-only transitions and the real alert renderer with isolated UI inputs."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.mici.onroad.status_timers import StatusTimers, format_mmss
from openpilot.selfdrive.ui.tests.test_mici_alert_presentation import load_layout


def update(timers, now=10., **changes):
  inputs = {'now': now, 'started': True, 'session': 100, 'fresh': True, 'standstill': True,
            'parking_brake': False, 'hold_active': False, 'alert_type': None}
  inputs.update(changes)
  return timers.update(**inputs)


@pytest.mark.parametrize(('elapsed', 'text'), [(-5, '00:00'), (0, '00:00'), (59.9, '00:59'), (60, '01:00'),
                                             (3600, '60:00'), (6000, '100:00'), (259200, '4320:00')])
def test_time_format(elapsed, text):
  assert format_mmss(elapsed) == text


def test_parking_tracks_state_behind_alerts_and_resets_on_release():
  timers = StatusTimers()
  assert update(timers, parking_brake=True).elapsed == 0
  assert update(timers, 20, parking_brake=True, alert_type='driverDistracted/warning') is None
  assert update(timers, 30, parking_brake=True).elapsed == 20
  assert update(timers, 31) is None
  assert update(timers, 32, parking_brake=True).elapsed == 0


@pytest.mark.parametrize('alert', ['', 'parkBrake/userDisable', 'brakeHold/noEntry', 'steerSaturated/warning',
                                  'controlsMismatch/immediateDisable', 'cameraMalfunction/permanent'])
def test_other_alerts_always_win(alert):
  assert update(StatusTimers(), parking_brake=True, hold_active=True, alert_type=alert) is None


@pytest.mark.parametrize('alert', [None, 'parkBrake/noEntry', 'silentParkBrake/noEntry'])
def test_parking_eligible_alerts(alert):
  assert update(StatusTimers(), parking_brake=True, alert_type=alert).kind == 'parking'


@pytest.mark.parametrize('alert', [None, 'resumeRequired/warning', 'brakeHold/warning', 'softHold/warning'])
def test_hold_requires_actual_hold_and_survives_alert_interruption(alert):
  timers = StatusTimers()
  assert update(timers, alert_type=alert) is None
  assert update(timers, hold_active=True, alert_type=alert).elapsed == 0
  assert update(timers, 20, hold_active=True, alert_type='steerSaturated/warning') is None
  assert update(timers, 25, hold_active=True, alert_type=alert).elapsed == 15
  assert update(timers, 26, alert_type=alert) is None


@pytest.mark.parametrize('condition', [{'started': False}, {'fresh': False}, {'standstill': False}])
def test_offroad_stale_or_moving_clears_both_clocks(condition):
  timers = StatusTimers()
  update(timers, parking_brake=True, hold_active=True)
  assert update(timers, 20, parking_brake=True, hold_active=True, **condition) is None
  assert update(timers, 30, parking_brake=True, hold_active=True).elapsed == 0
  assert update(timers, 31, parking_brake=True, hold_active=True, session=101).elapsed == 0


def test_critical_and_parking_hold_precedence():
  timers = StatusTimers()
  assert update(timers, parking_brake=True, hold_active=True, critical=True) is None
  assert update(timers, 11, parking_brake=True, hold_active=True).kind == 'parking'


def test_short_invalid_gap_is_hidden_and_does_not_reset_uninterrupted_stop():
  timers = StatusTimers()
  update(timers, parking_brake=True)
  assert update(timers, 10.1, parking_brake=True, fresh=False) is None
  assert update(timers, 10.5, parking_brake=True).elapsed == .5
  assert update(timers, 10.6, parking_brake=True, fresh=False) is None
  assert update(timers, 10.7, parking_brake=False) is None
  assert update(timers, 10.8, parking_brake=True).elapsed == 0


def make_renderer(monkeypatch):
  renderer, ns = load_layout()
  cs = SimpleNamespace(parkingBrake=False, standstill=True, brakeHoldActive=False, softHoldActive=0,
                       cruiseState=SimpleNamespace(standstill=False), vEgo=0., gasPressed=False,
                       brakePressed=False, gearShifter='drive')
  cc = SimpleNamespace(enabled=False, longActive=False, actuators=SimpleNamespace(longControlState='off', accel=0.))
  ss = SimpleNamespace(enabled=False, active=False)
  ns['car'].CarControl.Actuators = SimpleNamespace(LongControlState=SimpleNamespace(stopping='stopping'))
  ns['car'].CarState = SimpleNamespace(GearShifter=SimpleNamespace(
    **{gear: gear for gear in ('drive', 'sport', 'low', 'eco', 'manumatic')}))

  class SM(dict):
    valid = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), True)
    alive = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), True)
    recv_frame = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), 101)
    recv_time = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), 10.0)

  sm = SM(carState=cs, carControl=cc, selfdriveState=ss)
  ns['ui_state'] = SimpleNamespace(sm=sm, started=True, started_frame=100)
  monkeypatch.setattr(ns['time'], 'monotonic', lambda: 10.)
  renderer._status_timers = StatusTimers()
  renderer._last_started_frame = 100
  renderer._prev_alert = None
  renderer._pending_presentation = None
  draws = []
  renderer._status_timer_renderer = SimpleNamespace(draw=lambda *args: draws.append(args))
  renderer._alpha_filter.update = lambda v: setattr(renderer._alpha_filter, 'x', v)
  renderer._alert_y_filter.update = lambda v: setattr(renderer._alert_y_filter, 'x', v)
  renderer._draw_background = lambda _: None
  renderer._draw_icons = lambda _: None
  return renderer, ns, cs, sm, draws


def set_commanded_hold(sm):
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = True
  sm['carControl'].enabled = sm['carControl'].longActive = True
  sm['carControl'].actuators.longControlState = 'stopping'
  sm['carControl'].actuators.accel = -.507


def test_stopped_active_longitudinal_hold_without_stock_hold_signal(monkeypatch):
  renderer, ns, cs, sm, _ = make_renderer(monkeypatch)
  set_commanded_hold(sm)
  assert not (cs.brakeHoldActive or cs.softHoldActive or cs.cruiseState.standstill)
  below_speed = ns['Alert'](alert_type='belowSteerSpeed/warning')
  assert renderer._timer_display(below_speed).kind == 'hold'
  assert renderer._timer_display(None).kind == 'hold'


def test_native_message_enums_and_sound_only_alert_transition(monkeypatch):
  from openpilot.cereal import car, log

  renderer, ns, _, sm, draws = make_renderer(monkeypatch)
  ns['car'] = car
  cs = car.CarState.new_message(standstill=True, vEgo=0., gearShifter='drive')
  cc = car.CarControl.new_message(enabled=True, longActive=True)
  cc.actuators.longControlState = 'stopping'
  cc.actuators.accel = -.507
  ss = log.SelfdriveState.new_message(enabled=True, active=True, alertSize='small', alertStatus='normal',
                                    alertType='belowSteerSpeed/warning', alertText1='Steer assist unavailable')
  sm.update(carState=cs.as_reader(), carControl=cc.as_reader(), selfdriveState=ss.as_reader())
  sm.updated = dict.fromkeys(sm, True)
  for seconds, kind, size in ((0., 'belowSteerSpeed/warning', 'small'), (1., 'audioRefuse/warning', 'none'),
                              (1.2, 'belowSteerSpeed/warning', 'small')):
    now = 10. + seconds
    monkeypatch.setattr(ns['time'], 'monotonic', lambda now=now: now)
    sm.recv_time = dict.fromkeys(sm, now)
    ss.alertType, ss.alertSize = kind, size
    before = {name: message.to_dict() for name, message in sm.items()}
    renderer.will_render()
    renderer._render(renderer.rect)
    assert renderer.timer_visible
    assert draws[-1][1].kind == 'hold'
    assert draws[-1][1].elapsed == pytest.approx(seconds)
    assert before == {name: message.to_dict() for name, message in sm.items()}
  assert len(draws) == 3


@pytest.mark.parametrize(('target', 'field', 'value'), [
  ('selfdriveState', 'enabled', False), ('selfdriveState', 'active', False),
  ('carControl', 'enabled', False), ('carControl', 'longActive', False),
  ('actuators', 'longControlState', 'off'), ('actuators', 'longControlState', 'pid'),
  ('actuators', 'longControlState', 'starting'), ('actuators', 'accel', 0.),
  ('actuators', 'accel', 1.), ('actuators', 'accel', float('nan')),
  ('actuators', 'accel', -float('inf')),
  ('carState', 'standstill', False), ('carState', 'vEgo', .11), ('carState', 'vEgo', -.11),
  ('carState', 'vEgo', float('nan')), ('carState', 'vEgo', float('inf')),
  ('carState', 'gasPressed', True), ('carState', 'brakePressed', True),
  ('carState', 'gearShifter', 'park'), ('carState', 'gearShifter', 'neutral'),
  ('carState', 'gearShifter', 'reverse'), ('carState', 'gearShifter', 'unknown'),
])
def test_commanded_hold_requires_current_active_stop(monkeypatch, target, field, value):
  renderer, ns, _, sm, _ = make_renderer(monkeypatch)
  set_commanded_hold(sm)
  obj = sm['carControl'].actuators if target == 'actuators' else sm[target]
  setattr(obj, field, value)
  assert renderer._timer_display(ns['Alert'](alert_type='belowSteerSpeed/warning')) is None
  assert renderer._timer_display(None) is None


@pytest.mark.parametrize('service', ['carState', 'selfdriveState', 'carControl'])
@pytest.mark.parametrize('field,value', [('valid', False), ('alive', False), ('recv_frame', 99),
                                       ('recv_time', 9.74), ('recv_time', 10.01)])
def test_commanded_hold_rejects_invalid_old_or_future_input(monkeypatch, service, field, value):
  renderer, ns, _, sm, _ = make_renderer(monkeypatch)
  set_commanded_hold(sm)
  getattr(sm, field)[service] = value
  assert renderer._timer_display(ns['Alert'](alert_type='belowSteerSpeed/warning')) is None


def test_low_speed_warning_yields_only_to_confirmed_stopped_hold():
  for alert_type in (None, 'belowSteerSpeed/warning'):
    assert update(StatusTimers(), hold_active=True, alert_type=alert_type).kind == 'hold'
    assert update(StatusTimers(), hold_active=False, alert_type=alert_type) is None
    assert update(StatusTimers(), hold_active=True, standstill=False, alert_type=alert_type) is None
    assert update(StatusTimers(), hold_active=True, critical=True, alert_type=alert_type) is None
    assert update(StatusTimers(), hold_active=True, hold_without_alert=False, alert_type=alert_type) is None


def test_commanded_hold_survives_sound_only_transition_but_never_hides_other_warnings(monkeypatch):
  renderer, ns, _, sm, draws = make_renderer(monkeypatch)
  set_commanded_hold(sm)
  below = ns['Alert']('Steer assist unavailable', '10 km/h', size=1, status=1,
                      alert_type='belowSteerSpeed/warning')
  for seconds, alert in ((0., below), (1., None), (1.2, below)):
    now = 10. + seconds
    monkeypatch.setattr(ns['time'], 'monotonic', lambda now=now: now)
    sm.recv_time = dict.fromkeys(sm.recv_time, now)
    renderer.get_alert = lambda _, alert=alert: alert
    renderer.will_render()
    assert renderer.timer_visible
    renderer._render(renderer.rect)
    assert draws[-1][1].kind == 'hold'
    assert draws[-1][1].elapsed == pytest.approx(seconds)
  assert len(draws) == 3
  for alert_type in ('steerSaturated/warning', 'driverDistracted2/permanent', 'fcw/permanent',
                     'steerUnavailable/immediateDisable', 'accFaulted/immediateDisable', 'trafficStopping/warning'):
    renderer.get_alert = lambda _, alert_type=alert_type: ns['Alert']('Warning', size=1, alert_type=alert_type)
    renderer.will_render()
    assert not renderer.timer_visible
    renderer._render(renderer.rect)
  assert len(draws) == 3


def test_commanded_hold_drops_immediately_and_restarts_clock_after_override(monkeypatch):
  renderer, ns, _, sm, _ = make_renderer(monkeypatch)
  set_commanded_hold(sm)
  below = ns['Alert'](alert_type='belowSteerSpeed/warning')
  assert renderer._timer_display(below).elapsed == 0.
  sm['carControl'].longActive = False
  assert renderer._timer_display(below) is None
  monkeypatch.setattr(ns['time'], 'monotonic', lambda: 11.)
  sm.recv_time = dict.fromkeys(sm.recv_time, 11.0)
  sm['carControl'].longActive = True
  assert renderer._timer_display(below).elapsed == 0.


@pytest.mark.parametrize('signal', ['parkingBrake', 'brakeHoldActive', 'softHoldActive', 'cruiseState'])
def test_actual_car_state_mapping(monkeypatch, signal):
  renderer, ns, cs, _, _ = make_renderer(monkeypatch)
  alert = None
  if signal == 'cruiseState':
    cs.cruiseState.standstill = True
    assert renderer._timer_display(None) is None
    alert = ns['Alert'](alert_type='resumeRequired/warning')
  else:
    setattr(cs, signal, 2 if signal == 'softHoldActive' else True)
  assert renderer._timer_display(alert).kind == ('parking' if signal == 'parkingBrake' else 'hold')
  if signal == 'softHoldActive':
    cs.softHoldActive = 1
    assert renderer._timer_display(None) is None


@pytest.mark.parametrize('service', ['carState', 'selfdriveState'])
@pytest.mark.parametrize('field,value', [('valid', False), ('alive', False), ('recv_frame', 99),
                                       ('recv_time', 8.99), ('recv_time', 10.1)])
def test_no_timer_from_old_invalid_or_future_sample(monkeypatch, service, field, value):
  renderer, _, cs, sm, _ = make_renderer(monkeypatch)
  cs.parkingBrake = True
  getattr(sm, field)[service] = value
  assert renderer._timer_display(None) is None


def test_render_timer_then_same_alert_retains_font_cache(monkeypatch):
  renderer, ns, cs, _, draws = make_renderer(monkeypatch)
  alert = ns['Alert']('조향 한계', '핸들을 잡아주세요', size=1, alert_type='steerSaturated/warning')
  renderer.get_alert = lambda _: alert
  renderer._render(renderer.rect)
  before = (renderer._text_layout_key, renderer._alert_text1_label.font_size, renderer._alert_text1_label.text)
  renderer.get_alert = lambda _: None
  cs.parkingBrake = True
  renderer.will_render()
  assert renderer.timer_visible
  renderer._render(renderer.rect)
  assert draws[-1][1].kind == 'parking'
  assert renderer._prev_alert is None
  renderer.get_alert = lambda _: alert
  renderer.will_render()
  assert not renderer.timer_visible
  renderer._render(renderer.rect)
  assert len(draws) == 1
  assert before == (renderer._text_layout_key, renderer._alert_text1_label.font_size, renderer._alert_text1_label.text)


def test_render_offroad_resets_and_timeout_is_not_replaced(monkeypatch):
  renderer, ns, cs, _, draws = make_renderer(monkeypatch)
  cs.parkingBrake = True
  renderer.get_alert = lambda _: None
  renderer._render(renderer.rect)
  for key in ('ALERT_STARTUP_PENDING', 'ALERT_CRITICAL_TIMEOUT', 'ALERT_CRITICAL_REBOOT'):
    renderer.get_alert = lambda _, key=key: ns[key]
    renderer._render(renderer.rect)
    assert renderer._alert_text1_label.text == ns[key].text1
  assert len(draws) == 1
  ns['ui_state'].started = False
  assert renderer._render(renderer.rect) is False
  assert renderer._prev_alert is None
  assert renderer._alpha_filter.x == 0


def test_stock_standstill_clock_survives_other_alerts_but_needs_resume_to_show(monkeypatch):
  renderer, ns, cs, sm, _ = make_renderer(monkeypatch)
  cs.cruiseState.standstill = True
  resume = ns['Alert'](alert_type='resumeRequired/warning')
  assert renderer._timer_display(resume).elapsed == 0
  monkeypatch.setattr(ns['time'], 'monotonic', lambda: 10.5)
  assert renderer._timer_display(ns['Alert'](alert_type='steerSaturated/warning')) is None
  assert renderer._timer_display(None) is None
  assert renderer._timer_display(resume).elapsed == .5
  cs.cruiseState.standstill = False
  assert renderer._timer_display(resume) is None


def test_hud_and_timer_use_the_same_frame_snapshot(monkeypatch):
  renderer, _, cs, sm, draws = make_renderer(monkeypatch)
  cs.parkingBrake = True
  renderer.get_alert = lambda _: None
  renderer.will_render()
  assert renderer.timer_visible
  sm.alive['carState'] = False
  renderer._render(renderer.rect)
  assert len(draws) == 1
  renderer.will_render()
  assert not renderer.timer_visible
  renderer._render(renderer.rect)
  assert len(draws) == 1


def test_explicit_wayon_event_font_sizes_stay_applied():
  from openpilot.selfdrive.selfdrived.alert_presentation import font_sizes, load_overrides
  count = 0
  for config in load_overrides().values():
    if config.get('source') != 'EVENTS':
      continue
    # Use the actual camelCase type used by the event bus.
    parts = config['eventType'].lower().split('_')
    kind = parts[0] + ''.join(p.title() for p in parts[1:])
    sizes = font_sizes(config['eventName'], kind, 'short', 'short', True)
    for index, field in enumerate(('titleFontPx', 'subtitleFontPx')):
      if field in config:
        assert sizes[index] == config[field]
        count += 1
  assert count >= 10


def ui_method(filename, class_name, name, ns):
  path = Path(__file__).parents[1] / 'onroad' / filename
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[future, method], type_ignores=[])), str(path), 'exec'), ns)
  return ns[name]


def test_hud_timer_hides_only_overlapping_panel_and_repaints_cache_immediately():
  calls, keys = [], []
  obj = SimpleNamespace(_status_timer_visible=False, _overlay_state=None, _traffic_light_info=None,
                        _can_draw_top_icons=True, _show_wheel_critical=False, _new_ui_active=False)
  for name in ('_draw_set_speed', '_draw_steering_wheel', '_draw_egpu_badge', '_draw_cruise_speed_animation'):
    setattr(obj, name, lambda _, name=name: calls.append(name))
  paint = ui_method('hud_renderer.py', 'HudRenderer', '_paint', {})
  obj._paint = lambda rect: paint(obj, rect)
  obj._overlay = SimpleNamespace(draw=lambda rect, callback, key, **_: (keys.append(key), callback(rect)))
  render = ui_method('hud_renderer.py', 'HudRenderer', '_render', {'ui_state': SimpleNamespace(new_ui=False)})
  for visible in (False, True, False):
    obj._status_timer_visible = visible
    calls.clear()
    render(obj, None)
    assert ('_draw_set_speed' in calls) is not visible
    assert ('_draw_cruise_speed_animation' in calls) is not visible
    assert '_draw_steering_wheel' in calls and '_draw_egpu_badge' in calls
  assert keys[0] == keys[2] != keys[1]


def test_traffic_light_countdown_remains_above_timer():
  calls = []
  wheel = SimpleNamespace(width=50, height=50)
  obj = SimpleNamespace(_status_timer_visible=True, _show_wheel_critical=False, _txt_wheel=wheel,
                        _wheel_y_filter=SimpleNamespace(x=0),
                        _draw_steering_wheel_icon=lambda *args: calls.append('wheel'),
                        _draw_traffic_light_info=lambda *args: calls.append(('signal', args)),
                        _draw_wheel_side_info=lambda *args: calls.append('clock-debug'))
  draw = ui_method('hud_renderer.py', 'HudRenderer', '_draw_steering_wheel', {})
  rect = SimpleNamespace(x=60, y=0)
  draw(obj, rect)
  assert calls == ['wheel', ('signal', (155, 43))]
  obj._status_timer_visible = False
  calls.clear()
  draw(obj, rect)
  assert calls == ['wheel', 'clock-debug']


def test_bsd_strips_survive_timer_while_only_optional_vision_card_is_hidden():
  draws = []
  cs = SimpleNamespace(leftBlindspot=True, rightBlindspot=True)

  class SM(dict):
    valid = {'carState': True}
    alive = {'carState': True}
    recv_frame = {'carState': 1}

  ns = {
    'ui_state': SimpleNamespace(started=True, started_frame=1, share_data=True, sm=SM(carState=cs)),
    'time': SimpleNamespace(monotonic_ns=lambda: 1),
    'blindspot_source_packet': lambda *_: None, 'blindspot_sources': lambda *_: (True, False),
    'AMBER': 'amber', 'LIGHT_RED': 'red', 'BLUE': 'blue',
    'rl': SimpleNamespace(Rectangle=lambda *args: args, draw_rectangle_rounded=lambda *args: draws.append(args)),
  }
  obj = SimpleNamespace(_status_timer_visible=True, _text=lambda *args: None)
  render = ui_method('vision_renderer.py', 'VisionRenderer', '_render', ns)
  render(obj, SimpleNamespace(x=60, y=0, width=476, height=240))
  assert [args[0] for args in draws] == [(66, 78, 6, 56), (524, 78, 6, 56)]
