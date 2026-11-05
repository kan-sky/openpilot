import ast
import importlib.util
import sys
import time
from collections import namedtuple
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ONROAD = Path(__file__).parents[1] / 'onroad'


@pytest.fixture
def scene(monkeypatch):
  calls = []
  now = [10.]
  class Filter:
    def __init__(self, x, *_args):
      self.x = x
    def update(self, x):
      self.x = x
      return x
  class Torque:
    def __init__(self):
      self._torque_filter = Filter(0)
      self._torque_line_alpha_filter = Filter(0)
    def render(self, rect):
      calls.append(('torque', rect))
  rl = NS(Rectangle=namedtuple('Rectangle', 'x y width height'), Vector2=namedtuple('Vector2', 'x y'),
          Color=namedtuple('Color', 'r g b a'), WHITE='white', BLANK='blank', get_time=lambda: now[0])
  for name in ('draw_texture_pro', 'draw_texture_ex', 'draw_text_ex', 'draw_circle_gradient', 'draw_rectangle_rounded'):
    setattr(rl, name, lambda *args, name=name: calls.append((name, *args)))
  class SM(dict):
    valid = dict.fromkeys(('carState', 'selfdriveState', 'carControl', 'carrotMan', 'longitudinalPlan'), True)
    alive = valid.copy()
    recv_frame = dict.fromkeys(valid, 101)
    seen = valid.copy()
    recv_time = dict.fromkeys(valid, time.monotonic())
  cs = NS(leftBlindspot=False, rightBlindspot=False, steeringAngleDeg=5, vCruise=100, gasPressed=False, brakePressed=False)
  sm = SM(carState=cs, carControl=NS(latActive=False, longActive=True), selfdriveState=NS(enabled=True),
          carrotMan=NS(desiredSpeed=250, desiredSource='none'), longitudinalPlan=NS(cruiseTarget=100))
  ui = NS(started=True, started_frame=100, status='engaged', is_metric=True, sm=sm, new_ui=True)
  gui = NS(target_fps=20, texture=lambda name, w, h, **kw: NS(name=name, width=w, height=h, flip_x=kw.get('flip_x', False)))
  for name, value in {
    'pyray': rl,
    'openpilot.common.filter_simple': NS(FirstOrderFilter=Filter),
    'openpilot.selfdrive.ui.ui_state': NS(ui_state=ui, UIStatus=NS(DISENGAGED='disengaged')),
    'openpilot.system.ui.lib.application': NS(gui_app=gui),
    'openpilot.system.ui.lib.multilang': NS(tr=lambda text: text),
    'openpilot.system.ui.lib.text_measure': NS(measure_text_cached=lambda _, text, size: NS(x=len(text) * size * .6, y=size)),
    'openpilot.selfdrive.ui.mici.onroad.torque_bar': NS(TorqueBar=Torque),
  }.items():
    monkeypatch.setitem(sys.modules, name, value)
  badge_name = 'openpilot.selfdrive.ui.mici.onroad.deceleration_badge'
  badge_spec = importlib.util.spec_from_file_location(badge_name, ONROAD / 'deceleration_badge.py')
  badge_module = importlib.util.module_from_spec(badge_spec)
  monkeypatch.setitem(sys.modules, badge_name, badge_module)
  badge_spec.loader.exec_module(badge_module)
  bsm_name = 'openpilot.selfdrive.ui.mici.onroad.blind_spot_indicators'
  bsm_spec = importlib.util.spec_from_file_location(bsm_name, ONROAD / 'blind_spot_indicators.py')
  bsm_module = importlib.util.module_from_spec(bsm_spec)
  monkeypatch.setitem(sys.modules, bsm_name, bsm_module)
  bsm_spec.loader.exec_module(bsm_module)
  spec = importlib.util.spec_from_file_location('_new_ui_test', ONROAD / 'wayon_hud.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  hud = NS(_can_draw_top_icons=True, _status_timer_visible=False, _show_wheel_critical=False,
           is_cruise_set=True, _engaged=True, set_speed=100, _set_speed_changed_time=10,
           _font_display='display', _font_semi_bold='semi', _turn_intent=NS(render=lambda rect: calls.append(('turn', rect))))
  return NS(renderer=module.WayonHud(), module=module, ui=ui, sm=sm, cs=cs, hud=hud, calls=calls, now=now,
            rect=rl.Rectangle(0, 0, 476, 240), badge=badge_module)


def texts(scene):
  return [call[2] for call in scene.calls if call[0] == 'draw_text_ex']


def test_max_is_temporary_and_uses_real_selected_speed(scene):
  s = scene
  s.renderer.draw(s.rect, s.hud)
  assert texts(s) == ['100', 'MAX']
  assert s.renderer.drawing_top_icons()
  s.now[0] += 2.6
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert not texts(s) and not s.renderer.drawing_top_icons()
  assert any(call[0] == 'torque' for call in s.calls)
  s.hud.set_speed = 120
  s.hud._set_speed_changed_time = s.now[0]
  s.ui.is_metric = False
  s.renderer.draw(s.rect, s.hud)
  assert texts(s) == ['75', 'MAX']


@pytest.mark.parametrize('condition', ['warning', 'timer', 'stale', 'old_session', 'offroad'])
def test_warning_timer_and_bad_inputs_clear_max_immediately(scene, condition):
  s = scene
  s.renderer.draw(s.rect, s.hud)
  s.calls.clear()
  if condition == 'warning':
    s.hud._can_draw_top_icons = False
  elif condition == 'timer':
    s.hud._status_timer_visible = True
  elif condition == 'stale':
    s.sm.alive = dict(s.sm.alive, carState=False)
  elif condition == 'old_session':
    s.ui.started_frame = 102
  else:
    s.ui.started = False
  s.renderer.draw(s.rect, s.hud)
  assert not texts(s) and not s.renderer.drawing_top_icons()
  assert any(call[0] == 'torque' for call in s.calls) == (condition == 'warning')


def test_bsd_hides_ordinary_wheel_but_not_critical_wheel(scene):
  s = scene
  s.cs.leftBlindspot = True
  s.renderer.draw(s.rect, s.hud)
  assert next(call for call in s.calls if call[0] == 'draw_texture_pro')[-1].a == 0
  s.calls.clear()
  s.hud._show_wheel_critical = True
  s.renderer.draw(s.rect, s.hud)
  wheel = next(call for call in s.calls if call[0] == 'draw_texture_pro')
  assert wheel[1].name.endswith('wheel_critical.png') and wheel[-1].a == 255
  assert any(call[0] == 'draw_texture_ex' for call in s.calls)


def test_disengaged_and_reset_remove_old_presentation(scene):
  s = scene
  s.hud._engaged = False
  s.ui.status = 'disengaged'
  s.renderer.draw(s.rect, s.hud)
  assert not texts(s)
  s.renderer._speed_alpha.x = 1
  s.renderer._torque_bar._torque_filter.x = .8
  s.renderer.reset()
  assert not s.renderer.drawing_top_icons()
  assert s.renderer._torque_bar._torque_filter.x == 0


@pytest.mark.parametrize('blindspot', [False, True])
def test_lateral_only_keeps_torque_and_yields_wheel_to_wayon_bsm(scene, blindspot):
  s = scene
  s.ui.status = 'disengaged'
  s.hud._engaged = False
  s.sm['carControl'].latActive = True
  s.cs.leftBlindspot = blindspot
  s.renderer.draw(s.rect, s.hud)
  color = next(call for call in s.calls if call[0] == 'draw_texture_pro')[-1]
  assert color.r == 0 and color.g == 255
  assert (color.a > 0) == (not blindspot)
  assert s.renderer._torque_bar.lateral_active
  assert bool(bsm_icons(s)) == blindspot
  assert not texts(s)
  s.sm.alive = dict(s.sm.alive, carControl=False)
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert not s.renderer._torque_bar.lateral_active
  assert next(call for call in s.calls if call[0] == 'draw_texture_pro')[-1].a == 0


def test_lane_change_alert_keeps_turn_intent_below_alert(scene):
  s = scene
  s.hud._can_draw_top_icons = False
  s.renderer.draw(s.rect, s.hud)
  assert any(call[0] == 'turn' for call in s.calls)
  assert not texts(s)


def test_real_hud_dispatch_reuses_one_wayon_instance_and_restores_legacy(scene, monkeypatch):
  s = scene
  tree = ast.parse((ONROAD / 'hud_renderer.py').read_text())
  klass = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'HudRenderer')
  method = next(node for node in klass.body if isinstance(node, ast.FunctionDef) and node.name == '_render')
  namespace = {'rl': s.module.rl, 'ui_state': s.ui}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(ONROAD / 'hud_renderer.py'), 'exec'), namespace)
  allocations, draws = [], []
  def create():
    allocations.append(True)
    return s.renderer
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.mici.onroad.wayon_hud', NS(WayonHud=create))
  h = s.hud
  h._new_ui_active, h._wayon_hud, h._overlay_state = False, None, None
  h._traffic_light_info = None
  h._overlay = NS(draw=lambda *a, **kw: draws.append('legacy'), invalidate=lambda: draws.append('invalidate'))
  h._paint = lambda rect: None
  h._draw_egpu_badge = lambda rect: draws.append('egpu')
  h._draw_traffic_light_info = lambda *args: None
  h._draw_cruise_speed_animation = lambda rect: draws.append('animation')
  for _ in range(20):
    s.ui.new_ui = True
    namespace['_render'](h, s.rect)
    assert draws[-1] == 'egpu'
    s.ui.new_ui = False
    namespace['_render'](h, s.rect)
    assert draws[-2:] == ['legacy', 'animation']
  assert len(allocations) == 1
  assert draws.count('invalidate') == 40


def test_new_ui_is_display_only_and_existing_overlay_owners_remain():
  text = (ONROAD / 'augmented_road_view.py').read_text()
  for fragment in ('self._model_renderer.render', 'self._alert_renderer.render', 'self._traffic_light.render',
                   'self._vision_renderer.render', 'self._hud_renderer.draw_jetlink_status'):
    assert fragment in text
  source = (ONROAD / 'wayon_hud.py').read_text()
  assert all(forbidden not in source for forbidden in ('PubMaster', 'Params(', 'put_bool', 'controls_allowed'))


@pytest.mark.parametrize('source,label', [('atc', 'TURN'), ('atc2', 'TURN'), ('cam', 'CAM'), ('hda', 'CAM'),
                                        ('cam:v', 'CAM'), ('bump:n', 'BUMP'), ('hda_section', 'SECTION'),
                                        ('vturn', 'VTURN'), ('route', 'ROUTE'), ('school', 'SCHOOL'),
                                        ('section', 'SECTION'), ('road', 'ROAD'), ('police', 'POLICE'),
                                        ('waze', 'WAZE'), ('hda_bump', 'BUMP')])
@pytest.mark.parametrize('cap,final,selected,phase', [(160, 80, 120, 'waiting'), (100, 80, 120, 'decelerating'),
                                                   (80, 80, 120, 'complete'), (160, 80, 70, 'complete')])
def test_selected_deceleration_source_labels(scene, source, label, cap, final, selected, phase):
  set_deceleration(scene, source, cap, final, selected)
  if source in ('vturn', 'route') and cap >= selected:
    assert scene.badge.selected_deceleration(scene.sm, 100) is None
    return
  assert scene.badge.selected_deceleration(scene.sm, 100).label == label
  assert scene.badge.selected_deceleration(scene.sm, 100).phase == phase


def set_deceleration(s, source='cam', cap=100., final=80., selected=120.):
  s.cs.vCruise = selected
  s.sm['carrotMan'] = NS(desiredSpeed=int(cap), desiredSource=source,
                         decelerationCandidates=[NS(source=source, speedKph=cap, targetKph=final)])
  s.sm['longitudinalPlan'].cruiseTarget = min(selected, int(cap))
  s.hud._set_speed_changed_time = -10


@pytest.mark.parametrize('actual', [0, 70, 95, 120])
def test_progress_uses_same_reason_cap_not_actual_vehicle_speed(scene, actual):
  s = scene
  s.cs.vEgo = s.cs.vEgoCluster = actual / 3.6
  for cap, phase in [(250, 'waiting'), (160, 'waiting'), (120, 'waiting'), (110, 'decelerating'),
                     (100, 'decelerating'), (81.99, 'decelerating'), (81.01, 'decelerating'),
                     (80.99, 'complete'), (80, 'complete')]:
    set_deceleration(s, cap=cap)
    # Repeated rounded targets and a stopped car must never imply completion.
    for _ in range(5):
      assert s.badge.selected_deceleration(s.sm, 100).phase == phase


@pytest.mark.parametrize('condition', ['disengaged', 'lateral_only', 'stale_plan', 'invalid_control',
                                      'old_plan', 'nan_plan', 'invalid_set'])
def test_deceleration_is_hidden_without_current_engaged_control(scene, condition):
  s = scene
  set_deceleration(s)
  if condition == 'disengaged':
    s.sm['selfdriveState'].enabled = False
  elif condition == 'lateral_only':
    s.sm['carControl'].longActive = False
  elif condition == 'stale_plan':
    s.sm.alive['longitudinalPlan'] = False
  elif condition == 'invalid_control':
    s.sm.valid['carControl'] = False
  elif condition == 'old_plan':
    s.sm.recv_frame['longitudinalPlan'] = 99
  elif condition == 'nan_plan':
    s.sm['longitudinalPlan'].cruiseTarget = float('nan')
  else:
    s.cs.vCruise = 255
  assert s.badge.selected_deceleration(s.sm, 100) is None
  s.renderer.draw(s.rect, s.hud)
  assert not texts(s)
  assert not any(call[0] == 'draw_rectangle_rounded' for call in s.calls)


@pytest.mark.parametrize('condition', ['gas', 'brake', 'gas_floor', 'other_target'])
@pytest.mark.parametrize('cap', [80, 100])
def test_valid_information_stays_white_when_cap_is_overridden(scene, condition, cap):
  s = scene
  set_deceleration(s, cap=cap)
  if condition in ('gas', 'brake'):
    setattr(s.cs, condition + 'Pressed', True)
  elif condition == 'gas_floor':
    s.sm['carrotMan'].desiredSource = 'gas'
  else:
    s.sm['longitudinalPlan'].cruiseTarget = 40
  assert s.badge.selected_deceleration(s.sm, 100).phase == 'waiting'


def test_pending_camera_survives_unrelated_neutral_winner_and_clears_when_invalid(scene):
  s = scene
  set_deceleration(s, cap=250)
  s.sm['carrotMan'].desiredSource = 'road'
  s.sm['carrotMan'].desiredSpeed = 200
  assert s.badge.selected_deceleration(s.sm, 100) == ('CAM', 'waiting', 'cam')
  s.sm['carrotMan'].decelerationCandidates = []
  assert s.badge.selected_deceleration(s.sm, 100) is None


def test_actual_winning_reason_precedes_pending_camera_and_never_borrows_completion(scene):
  s = scene
  set_deceleration(s, cap=100)
  s.sm['carrotMan'].decelerationCandidates.append(NS(source='road', speedKph=60., targetKph=60.))
  s.sm['carrotMan'].desiredSource, s.sm['carrotMan'].desiredSpeed = 'road', 60
  s.sm['longitudinalPlan'].cruiseTarget = 60
  assert s.badge.selected_deceleration(s.sm, 100) == ('ROAD', 'complete', 'road')
  s.sm['carrotMan'].decelerationCandidates.pop()
  assert s.badge.selected_deceleration(s.sm, 100) == ('CAM', 'waiting', 'cam')


@pytest.mark.parametrize('source', ['vturn', 'route', 'road'])
def test_unnecessary_green_candidate_cannot_mask_pending_camera(scene, source):
  s = scene
  set_deceleration(s, cap=250)
  s.sm['carrotMan'].decelerationCandidates.append(NS(source=source, speedKph=150., targetKph=150.))
  s.sm['carrotMan'].desiredSource, s.sm['carrotMan'].desiredSpeed = source, 150
  assert s.badge.selected_deceleration(s.sm, 100) == ('CAM', 'waiting', 'cam')
  s.cs.vCruise = 70
  assert s.badge.selected_deceleration(s.sm, 100).phase == 'complete'


@pytest.mark.parametrize('camera_first', [False, True])
def test_nearer_pending_event_wins_saturated_envelope_tie(scene, camera_first):
  s = scene
  set_deceleration(s, cap=250)
  camera = s.sm['carrotMan'].decelerationCandidates[0]
  camera.distanceM = 300.
  turn = NS(source='atc', speedKph=250., targetKph=30., distanceM=3000.)
  s.sm['carrotMan'].decelerationCandidates = [camera, turn] if camera_first else [turn, camera]
  s.sm['carrotMan'].desiredSource, s.sm['carrotMan'].desiredSpeed = 'road', 200
  assert s.badge.selected_deceleration(s.sm, 100) == ('CAM', 'waiting', 'cam')
  camera.distanceM, turn.distanceM = 3000., 300.
  assert s.badge.selected_deceleration(s.sm, 100) == ('TURN', 'waiting', 'atc')


def test_new_camera_or_target_does_not_inherit_previous_green(scene):
  s = scene
  for cap, final, phase in [(80, 80, 'complete'), (160, 60, 'waiting'), (100, 60, 'decelerating'), (60, 60, 'complete')]:
    set_deceleration(s, cap=cap, final=final)
    assert s.badge.selected_deceleration(s.sm, 100).phase == phase


def test_unknown_curve_endpoint_can_blink_but_cannot_invent_completion(scene):
  s = scene
  set_deceleration(s, source='vturn', cap=70, final=0)
  for _ in range(20):
    assert s.badge.selected_deceleration(s.sm, 100).phase == 'decelerating'
  set_deceleration(s, source='vturn', cap=70, final=0, selected=60)
  assert s.badge.selected_deceleration(s.sm, 100) is None


@pytest.mark.parametrize('source', ['vturn', 'route'])
def test_unapplied_curve_noise_never_draws_a_ready_badge(scene, source):
  s = scene
  for tick in range(200):
    # Current endpoint alternates with unknown while a prior lower cap is held.
    set_deceleration(s, source=source, cap=175.442, final=175.442 if tick % 5 == 0 else 0, selected=100)
    s.now[0] = 10 + tick * .05
    s.calls.clear()
    s.renderer.draw(s.rect, s.hud)
    assert s.badge.selected_deceleration(s.sm, 100) is None
    assert not texts(s)
    assert not any(call[0] == 'draw_rectangle_rounded' for call in s.calls)


@pytest.mark.parametrize('condition', ['unseen', 'expired', 'future', 'invalid', 'dead', 'old_session',
                                      'zero', 'legacy', 'neutral_200', 'neutral_250', 'unknown', 'gas', 'empty', 'nan_cap', 'nan_final'])
def test_badge_hides_missing_expired_and_neutral_navigation(scene, condition):
  s = scene
  set_deceleration(s)
  if condition == 'unseen':
    s.sm.seen['carrotMan'] = False
  elif condition == 'expired':
    s.sm.recv_time['carrotMan'] -= 1.01
  elif condition == 'future':
    s.sm.recv_time['carrotMan'] += 5
  elif condition == 'invalid':
    s.sm.valid['carrotMan'] = False
  elif condition == 'dead':
    s.sm.alive['carrotMan'] = False
  elif condition == 'old_session':
    s.sm.recv_frame['carrotMan'] = 99
  elif condition == 'zero':
    s.sm['carrotMan'].desiredSpeed = 0
  elif condition == 'legacy':
    del s.sm['carrotMan'].decelerationCandidates
  elif condition in ('neutral_200', 'neutral_250'):
    s.sm['carrotMan'].decelerationCandidates[0].speedKph = int(condition[-3:])
    s.sm['carrotMan'].decelerationCandidates[0].targetKph = 0
  elif condition in ('nan_cap', 'nan_final'):
    setattr(s.sm['carrotMan'].decelerationCandidates[0], 'speedKph' if condition == 'nan_cap' else 'targetKph', float('nan'))
  else:
    s.sm['carrotMan'].decelerationCandidates[0].source = '' if condition == 'empty' else condition
  assert s.badge.selected_deceleration(s.sm, 100) is None


@pytest.mark.parametrize('source,label', [('cam', 'CAM'), ('atc', 'TURN'), ('vturn', 'VTURN'), ('route', 'ROUTE')])
def test_badge_blinks_orange_white_at_one_hz_keeps_dark_text_and_does_not_move(scene, source, label):
  s = scene
  set_deceleration(s, source=source)
  boxes, colors = [], []
  for now in (10, 10.59, 10.7, 11.01):
    s.now[0] = now
    s.calls.clear()
    s.renderer.draw(s.rect, s.hud)
    assert texts(s) == [label]
    assert tuple(next(call for call in s.calls if call[0] == 'draw_text_ex')[-1]) == (8, 24, 14, 255)
    _, box, roundness, segments, color = next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')
    assert roundness == .3 and segments == 6
    boxes.append(box)
    colors.append(tuple(color))
  assert boxes == [s.module.rl.Rectangle(136, 162, 76, 26)] * 4
  assert colors == [(255, 149, 0, 255)] * 2 + [(255, 255, 255, 255), (255, 149, 0, 255)]
  s.sm['carControl'].longActive = False
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert not texts(s)
  assert s.renderer._deceleration_badge._active_state is None
  s.sm['carControl'].longActive = True
  s.now[0] = 11.7
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert tuple(next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')[-1]) == s.badge.DecelerationBadge.ORANGE


@pytest.mark.parametrize('phase,color', [('waiting', (255, 255, 255, 255)), ('complete', (32, 216, 106, 255))])
def test_waiting_and_complete_are_steady_and_do_not_blink(scene, phase, color):
  s = scene
  for now in (10., 10.7, 11.4, 12.9):
    s.calls.clear()
    s.renderer._deceleration_badge.draw(s.rect, s.hud._font_semi_bold, ('CAM', phase, 'cam'), now)
    assert tuple(next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')[-1]) == color


@pytest.mark.parametrize('source', ['route', 'vturn'])
def test_curve_green_requires_stable_completion_and_lost_endpoint_clears_immediately(scene, source):
  s = scene
  badge = s.renderer._deceleration_badge
  for now, phase, green in [(10., 'complete', False), (10.15, 'decelerating', False),
                             (10.2, 'complete', False), (10.49, 'complete', False),
                             (10.51, 'complete', True), (10.6, 'decelerating', False)]:
    s.calls.clear()
    badge.draw(s.rect, s.hud._font_semi_bold, (source.upper(), phase, source), now)
    color = tuple(next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')[-1])
    assert (color == badge.GREEN) == green
    if phase == 'complete' and not green:
      assert color == badge.ORANGE
  s.calls.clear()
  badge.draw(s.rect, s.hud._font_semi_bold, None, 10.7)
  assert not s.calls and badge._active_state is None


def test_badge_clears_and_resets_blink_when_phase_or_source_changes(scene):
  s = scene
  badge = s.renderer._deceleration_badge
  for state in [None, ('CAM', 'waiting', 'cam'), ('CAM', 'complete', 'cam'), ('CAM', 'decelerating', 'hda')]:
    badge.draw(s.rect, s.hud._font_semi_bold, ('CAM', 'decelerating', 'cam'), 10.)
    badge.draw(s.rect, s.hud._font_semi_bold, state, 10.7)
    s.calls.clear()
    badge.draw(s.rect, s.hud._font_semi_bold, ('CAM', 'decelerating', 'cam'), 10.8)
    assert tuple(next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')[-1]) == badge.ORANGE


def test_compact_badge_clears_wheel_and_fits_long_label_at_offset(scene):
  s = scene
  rect = s.module.rl.Rectangle(60, 10, 476, 240)
  s.renderer._deceleration_badge.draw(rect, s.hud._font_semi_bold, ('SECTION', 'complete', 'section'), 10.)
  _, box, *_ = next(call for call in s.calls if call[0] == 'draw_rectangle_rounded')
  _, _, label, position, size, spacing, _ = next(call for call in s.calls if call[0] == 'draw_text_ex')
  assert box.x == rect.x + 136 and box.y == rect.y + 162
  assert box.x > rect.x + 46 + 25 * 2 ** .5 + 15
  assert box.x >= rect.x + 20 + 108 + 8
  assert box.y + box.height < rect.y + 200
  assert label == 'SECTION' and spacing == 0
  assert box.x + 8 <= position.x and position.x + len(label) * size * .6 <= box.x + box.width - 8
  assert box.y <= position.y and position.y + size <= box.y + box.height


@pytest.mark.parametrize('condition', ['warning', 'timer', 'critical', 'offroad'])
def test_deceleration_yields_to_warnings_and_timers(scene, condition):
  s = scene
  set_deceleration(s)
  s.renderer.draw(s.rect, s.hud)
  assert 'CAM' in texts(s)
  if condition == 'warning':
    s.hud._can_draw_top_icons = False
  elif condition == 'timer':
    s.hud._status_timer_visible = True
  elif condition == 'critical':
    s.hud._show_wheel_critical = True
  else:
    s.ui.started = False
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert 'CAM' not in texts(s)
  assert s.renderer._deceleration_badge._active_state is None


def bsm_icons(scene):
  return [call for call in scene.calls if call[0] == 'draw_texture_ex' and call[1].name.endswith('blind_spot_left.png')]


@pytest.mark.parametrize('left,right', [(True, False), (False, True), (True, True), (False, False)])
def test_wayon_bsm_original_asset_position_and_side(scene, left, right):
  s = scene
  s.cs.leftBlindspot, s.cs.rightBlindspot = left, right
  s.renderer.draw(s.rect, s.hud)
  icons = bsm_icons(s)
  assert [call[2].x for call in icons] == ([20] if left else []) + ([348] if right else [])
  assert [call[1].flip_x for call in icons] == ([False] if left else []) + ([True] if right else [])
  assert all((call[1].width, call[1].height, call[2].y) == (108, 128, 100) for call in icons)
  assert all(tuple(call[-1]) == (255, 255, 255, 255) for call in icons)
  assert ('MAX' in texts(s)) == (not left)




@pytest.mark.parametrize('condition', ['offroad', 'invalid', 'dead', 'unseen', 'old_session'])
def test_wayon_bsm_clears_immediately_without_current_carstate(scene, condition):
  s = scene
  s.cs.leftBlindspot = s.cs.rightBlindspot = True
  s.renderer.draw(s.rect, s.hud)
  assert len(bsm_icons(s)) == 2
  if condition == 'offroad':
    s.ui.started = False
  elif condition == 'invalid':
    s.sm.valid['carState'] = False
  elif condition == 'dead':
    s.sm.alive['carState'] = False
  elif condition == 'unseen':
    s.sm.recv_frame['carState'] = 0
  else:
    s.ui.started_frame = 102
  s.calls.clear()
  s.renderer.draw(s.rect, s.hud)
  assert not bsm_icons(s) and not s.renderer._blind_spots.detected


@pytest.mark.parametrize('service', ['selfdriveState', 'carControl', 'longitudinalPlan'])
def test_bsm_does_not_depend_on_controls_or_model(scene, service):
  s = scene
  s.cs.rightBlindspot = True
  s.sm.alive[service] = False
  s.renderer.draw(s.rect, s.hud)
  assert len(bsm_icons(s)) == 1


@pytest.mark.parametrize('condition', ['critical', 'timer'])
def test_bsm_yields_to_primary_warning_or_centered_timer(scene, condition):
  s = scene
  s.cs.leftBlindspot = s.cs.rightBlindspot = True
  setattr(s.hud, '_show_wheel_critical' if condition == 'critical' else '_status_timer_visible', True)
  s.renderer.draw(s.rect, s.hud)
  assert not bsm_icons(s)
  assert s.renderer._blind_spots.detected
  if condition == 'critical':
    wheel = next(call for call in s.calls if call[0] == 'draw_texture_pro')
    assert wheel[1].name.endswith('wheel_critical.png') and wheel[-1].a == 255


def test_bsm_session_and_ui_reset_discard_fade_memory(scene):
  s = scene
  s.cs.leftBlindspot = True
  s.renderer.draw(s.rect, s.hud)
  assert s.renderer._blind_spots.detected
  s.cs.leftBlindspot = False
  s.renderer.reset()
  assert not s.renderer._blind_spots.detected


def test_bsm_uses_original_fade_in_and_out(scene, monkeypatch):
  # Exercise the real filter, not the instantaneous HUD fixture above.
  spec = importlib.util.spec_from_file_location('_real_bsm_filter', ONROAD.parents[3] / 'common/filter_simple.py')
  filters = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(filters)
  module = sys.modules['openpilot.selfdrive.ui.mici.onroad.blind_spot_indicators']
  monkeypatch.setattr(module, 'FirstOrderFilter', filters.FirstOrderFilter)
  bsm = module.BlindSpotIndicators()
  scene.cs.leftBlindspot = True
  bsm.update()
  assert bsm._left_alpha.x == pytest.approx(.25)
  for _ in range(19):
    bsm.update()
  assert bsm._left_alpha.x > .99
  scene.cs.leftBlindspot = False
  bsm.update()
  assert .7 < bsm._left_alpha.x < .75 and bsm.detected
  for _ in range(19):
    bsm.update()
  assert not bsm.detected
