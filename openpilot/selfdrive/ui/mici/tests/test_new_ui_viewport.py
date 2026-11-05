"""Exercise real view composition/projection without camera IPC or publishers."""
import ast
from collections import namedtuple
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.fixture
def view():
  calls, transforms = [], []
  rl = NS(Rectangle=namedtuple('Rectangle', 'x y width height'), Vector2=namedtuple('Vector2', 'x y'),
          Color=namedtuple('Color', 'r g b a'), WHITE='white', BLACK='black')
  for name in ('begin_scissor_mode', 'end_scissor_mode', 'draw_rectangle_rec', 'draw_texture_ex',
               'draw_texture_pro', 'draw_rectangle_rounded_lines_ex', 'draw_rectangle', 'draw_circle'):
    setattr(rl, name, lambda *args, name=name: calls.append((name, *args)))
  class Camera:
    def _render(self, rect):
      calls.append(('camera', rect))
      self._calc_frame_matrix(rect)
  class SM(dict):
    recv_frame = {'liveCalibration': 100}
  ui = NS(new_ui=False, started=True, show_model_view=0, sm=SM(carState=NS(vEgo=20)))
  namespace = dict(rl=rl, ui_state=ui, np=np, time=time, CameraView=Camera, SIDE_PANEL_WIDTH=60,
                   WIDE_CAM='wide', CAM_Y_OFFSET=20, DEFAULT_DEVICE_CAMERA=None,
                   car=NS(CarControl=NS(HUDControl=NS(VisualAlert=NS(steerRequired='critical')))),
                   gui_app=NS(is_recording=lambda: False),
                   messaging=NS(new_message=lambda *a, **kw: NS(uiDebug=NS())),
                   **{n: NS(active=lambda: False) for n in ('native_draw', 'native_geometry', 'native_text')})
  source = Path(__file__).parents[1] / 'onroad/augmented_road_view.py'
  original = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'AugmentedRoadView')
  methods = [n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in ('_render', '_calc_frame_matrix', '_road_view_mode')]
  klass = ast.ClassDef(name='UnderTest', bases=[ast.Name(id='CameraView', ctx=ast.Load())], keywords=[],
                       body=methods, decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[klass], type_ignores=[])), str(source), 'exec'), namespace)
  obj = namespace['UnderTest']()
  obj.rect = rl.Rectangle(0, 0, 536, 240)
  obj._last_rect_dims = (0, 0)
  obj._suppress_camera_for_cluster = False
  obj._refresh_plot_mode = lambda *_: None
  obj._plot_mode = 0
  obj._switch_stream_if_needed = lambda *_: None
  obj._update_calibration = lambda: None
  obj.client = object()
  obj.stream_type = 'road'
  intrinsic = np.array([[1000., 0., 964.], [0., 1000., 604.], [0., 0., 1.]])
  obj.device_camera = NS(fcam=NS(intrinsics=intrinsic), ecam=NS(intrinsics=intrinsic))
  obj.view_from_calib = obj.view_from_wide_calib = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
  obj._fade_texture = NS(width=476, height=240)
  obj._render_diagnostics = NS(start=lambda: None, finish=lambda: None, values={}, call=lambda name, fn, *a: fn(*a))
  obj._overlay_state = NS(update=lambda *args: calls.append(('overlay', args[-1])))
  obj._pm = NS(send=lambda *args: calls.append(('uiDebug',)))
  def widget(name):
    return NS(render=lambda rect: calls.append((name, rect)))
  obj._model_renderer = widget('model')
  obj._model_renderer.set_transform = lambda transform: transforms.append(transform.copy())
  obj._hud_renderer = widget('hud')
  obj._hud_renderer.set_status_timer_visible = lambda *_: None
  obj._hud_renderer.set_can_draw_top_icons = lambda *_: None
  obj._hud_renderer.set_wheel_critical_icon = lambda *_: None
  obj._hud_renderer.drawing_top_icons = lambda: False
  obj._hud_renderer.draw_jetlink_status = lambda rect, alert: calls.append(('jetlink', rect))
  obj._vision_renderer = widget('vision')
  obj._vision_renderer.set_status_timer_visible = lambda *_: None
  obj._alert_renderer = widget('alert')
  obj._alert_renderer.timer_visible = False
  obj._alert_renderer.will_render = lambda: (None, True)
  obj._driver_state_renderer = NS(draw_onroad=lambda rect, alert, **kw: calls.append(('dm', rect, alert, kw)))
  obj._wayon_driver_renderer = NS(draw_onroad=lambda rect, alert: calls.append(('wayon_dm', rect, alert)))
  obj._traffic_light = widget('signal')
  obj._traffic_light.is_visible = lambda: False
  obj._confidence_ball = widget('confidence')
  obj._bookmark_icon = widget('bookmark')
  obj._offroad_label = widget('offroad')
  return NS(obj=obj, ui=ui, rl=rl, calls=calls, transforms=transforms)


@pytest.mark.parametrize('new_ui', [False, True])
@pytest.mark.parametrize('mode', range(4))
@pytest.mark.parametrize('suppress', [False, True])
def test_full_width_composition_and_legacy_restore(view, new_ui, mode, suppress):
  v = view
  v.ui.new_ui, v.ui.show_model_view = new_ui, mode
  v.obj._suppress_camera_for_cluster = suppress
  v.obj.rect = v.rl.Rectangle(17, 9, 536, 240)
  v.obj._render(None)
  expected = v.rl.Rectangle(17, 9, 536 if new_ui else 476, 240)
  assert v.obj._content_rect == expected
  assert ('begin_scissor_mode', *expected) in v.calls
  names = [c[0] for c in v.calls]
  assert ('camera' in names) == (not suppress and mode in (0, 1))
  assert ('model' in names) == (not suppress and mode in (0, 2))
  assert ('signal' in names) == ('confidence' in names) == (not new_ui)
  for name in ('hud', 'vision', 'alert', 'jetlink', 'draw_rectangle_rounded_lines_ex'):
    assert next(c[1] for c in v.calls if c[0] == name) == expected
  assert ('overlay', (suppress, mode, new_ui)) in v.calls
  assert {'dm', 'bookmark', 'uiDebug'} <= set(names)
  assert next(c[2] for c in v.calls if c[0] == 'dm') == new_ui
  assert next(c[2] for c in v.calls if c[0] == 'wayon_dm') == (not new_ui)
  if not suppress:
    fade = next(c for c in v.calls if c[0] == ('draw_texture_pro' if new_ui else 'draw_texture_ex'))
    if new_ui:
      assert fade[2] == v.rl.Rectangle(0, 0, 476, 240) and fade[3] == expected
    else:
      assert fade[2] == v.rl.Vector2(17, 9)


def test_legacy_signal_still_hides_confidence_ball(view):
  view.obj._traffic_light.is_visible = lambda: True
  view.obj._render(None)
  names = [c[0] for c in view.calls]
  assert 'signal' in names and 'confidence' not in names


@pytest.mark.parametrize('blocked_by', ['alert', 'timer', 'max'])
def test_wayon_dm_yields_to_alert_timer_and_max(view, blocked_by):
  view.ui.new_ui = True
  if blocked_by == 'alert':
    view.obj._alert_renderer.will_render = lambda: (NS(), True)
  elif blocked_by == 'timer':
    view.obj._alert_renderer.timer_visible = True
  else:
    view.obj._hud_renderer.drawing_top_icons = lambda: True
  view.obj._render(None)
  assert next(c[2] for c in view.calls if c[0] == 'wayon_dm') is True


@pytest.mark.parametrize('mode', [0, 2])
@pytest.mark.parametrize('stream', ['road', 'wide'])
def test_toggle_reprojects_camera_and_vehicle_overlays_without_stale_center(view, mode, stream):
  v = view
  v.obj.stream_type, v.ui.show_model_view = stream, mode
  matrices, widths, projected = [], [], []
  for new_ui in (False, True, False):
    v.ui.new_ui = new_ui
    v.obj._render(None)
    matrices.append(v.obj._cached_matrix.copy())
    widths.append(v.obj._content_rect.width)
    point = v.transforms[-1] @ np.array([30., 0., 0.])
    projected.append(point[:2] / point[2])
  assert widths == [476, 536, 476]
  np.testing.assert_allclose(projected, [[238., 100.], [268., 100.], [238., 100.]])
  np.testing.assert_allclose(matrices[0], matrices[2])
  # Same camera zoom and vertical position, with 60px more horizontal viewport.
  assert matrices[0][0, 0] * 476 == pytest.approx(matrices[1][0, 0] * 536)
  np.testing.assert_allclose(matrices[0][1:], matrices[1][1:])


def test_new_ui_without_onroad_keeps_offroad_cover_and_no_status_rail(view):
  view.ui.new_ui, view.ui.started = True, False
  view.obj._render(None)
  assert view.obj._content_rect.width == 536
  names = [c[0] for c in view.calls]
  assert 'offroad' in names and 'draw_rectangle' in names
  assert 'signal' not in names and 'confidence' not in names
