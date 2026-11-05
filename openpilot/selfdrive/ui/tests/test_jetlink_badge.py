import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock  # noqa: TID251

import pytest
import pyray as rl
from openpilot.cereal import log

from openpilot.common import jetlink_status

ROOT = Path(__file__).resolve().parents[3]


class FakeParams:
  def __init__(self, *args):
    pass

  def get(self, *args, **kwargs):
    return None

  def get_bool(self, key):
    return key == 'IsMetric'

  def get_int(self, key):
    return {'ShowDateTime': 1, 'LongitudinalPersonality': 1}.get(key, 0)


class FakeSubMaster(dict):
  def __init__(self, services):
    super().__init__()
    for service in services:
      event = log.Event.new_message()
      if log.Event.schema.fields[service].proto.slot.type.which() in ('list', 'data', 'text'):
        self[service] = event.init(service, 0)
      else:
        self[service] = event.init(service)
    self.recv_frame = dict.fromkeys(services, 1)
    self.updated = dict.fromkeys(services, False)
    self.alive = dict.fromkeys(services, True)
    self.valid = dict.fromkeys(services, True)
    self.frame = 1

  def update(self, timeout):
    pass


def load_ui_modules(monkeypatch):
  # Replace device I/O only. Load the real UI state, HUDs and drawing helper.
  messaging = ModuleType('openpilot.cereal.messaging')
  messaging.SubMaster = FakeSubMaster
  params = ModuleType('openpilot.common.params')
  params.Params = FakeParams
  prime = ModuleType('openpilot.selfdrive.ui.lib.prime_state')
  prime.PrimeState = lambda: SimpleNamespace(start=Mock())
  monkeypatch.setitem(sys.modules, messaging.__name__, messaging)
  monkeypatch.setattr(sys.modules['openpilot.cereal'], 'messaging', messaging, raising=False)
  monkeypatch.setitem(sys.modules, params.__name__, params)
  monkeypatch.setitem(sys.modules, prime.__name__, prime)

  modules = []
  for name in ('selfdrive.ui.ui_state', 'selfdrive.ui.onroad.jetlink_badge', 'selfdrive.ui.onroad.exp_button',
               'selfdrive.ui.onroad.hud_renderer', 'selfdrive.ui.mici.onroad.hud_renderer'):
    fullname = 'openpilot.' + name
    spec = importlib.util.spec_from_file_location(fullname, ROOT / (name.replace('.', '/') + '.py'))
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, fullname, module)
    spec.loader.exec_module(module)
    modules.append(module)
  return SimpleNamespace(state=modules[0], drawing=modules[1], wide=modules[3], compact=modules[4])


@pytest.fixture
def ui(monkeypatch):
  existing = set(sys.modules)
  yield load_ui_modules(monkeypatch)
  for name in set(sys.modules) - existing:
    if name.startswith(('openpilot.selfdrive.ui.', 'openpilot.system.ui.widgets')):
      sys.modules.pop(name, None)


@pytest.fixture
def status_files(tmp_path, monkeypatch):
  monkeypatch.setattr(jetlink_status, 'LINK_STATUS', tmp_path / 'link.json')
  monkeypatch.setattr(jetlink_status, 'MODEL_STATUS', tmp_path / 'model.json')
  monkeypatch.setattr(jetlink_status.time, 'monotonic', lambda: 100.)

  def write(state='ready', active=False, severity='ok', link_age=0., health_age=0., error='', link_fields=None, model_fields=None):
    link = {'state': state, 'updated': 100. - link_age, 'peer': {'carrot_host': 'jetson'},
            'telemetry_updated': 100., 'telemetry': {'carrot_health': {'severity': severity, 'age_s': health_age}}}
    model = {'active': active, 'updated': 100., 'error': error}
    link.update(link_fields or {})
    model.update(model_fields or {})
    jetlink_status.LINK_STATUS.write_text(json.dumps(link))
    jetlink_status.MODEL_STATUS.write_text(json.dumps(model))
  return write


@pytest.mark.parametrize(('values', 'expected'), [
  ({'state': 'connecting'}, ('jetSON 연결 대기', 'loading')),
  ({'state': 'loading'}, ('jetSON 모델 준비 중', 'loading')),
  ({}, ('jetSON READY', 'ready')),
  ({'active': True}, ('jetSON', 'active')),
  ({'state': 'waiting'}, None),
  ({'state': 'retrying'}, ('jetSON ERROR', 'error')),
  ({'state': 'blocked'}, ('jetSON ERROR', 'error')),
  ({'error': 'inference failed'}, ('jetSON ERROR', 'error')),
  ({'severity': 'error'}, ('jetSON ERROR', 'error')),
  ({'severity': 'warning', 'active': True}, ('jetSON HOT', 'loading')),
  ({'severity': 'unknown'}, ('jetSON DATA?', 'loading')),
  ({'health_age': 3., 'active': True}, ('jetSON DATA?', 'loading')),
  ({'link_age': 3.}, ('jetSON ERROR', 'error')),
])
def test_status_contract(status_files, values, expected):
  status_files(**values)
  assert jetlink_status.badge() == expected


def test_missing_or_broken_status_is_quiet(status_files):
  assert jetlink_status.badge() is None
  jetlink_status.LINK_STATUS.write_text('{partial')
  assert jetlink_status.badge() is None
  jetlink_status.LINK_STATUS.write_text(json.dumps({'state': 'waiting', 'updated': 100.}))
  assert jetlink_status.badge() is None


def test_real_files_flow_through_ui_update(ui, status_files, monkeypatch):
  state = ui.state.ui_state
  for method in ('_update_state', '_update_status'):
    monkeypatch.setattr(state, method, Mock())
  monkeypatch.setattr(ui.state.device, 'update', Mock())
  for values, expected in [({'state': 'loading'}, ('jetSON 모델 준비 중', 'loading')),
                           ({}, ('jetSON READY', 'ready')),
                           ({'active': True}, ('jetSON', 'active')),
                           ({'error': 'inference failed'}, ('jetSON ERROR', 'error')),
                           ({'state': 'waiting'}, None)]:
    status_files(**values)
    state._jetlink_update_time = float('-inf')
    state.update()
    assert state.jetlink_badge == expected


def test_status_polling_is_limited_to_four_reads_per_second(ui, monkeypatch):
  read = Mock(return_value=(('jetSON READY', 'ready'), ''))
  monkeypatch.setattr(ui.state, 'read_jetlink_status', read)
  for now in (100., 100.05, 100.20, 100.25):
    monkeypatch.setattr(ui.state.time, 'monotonic', lambda now=now: now)
    ui.state.ui_state._update_jetlink_status()
  assert read.call_count == 2


@pytest.mark.parametrize('error', [OSError(), ValueError(), TypeError(), KeyError(), OverflowError()])
def test_status_errors_clear_old_badge_without_crashing(ui, monkeypatch, error):
  ui.state.ui_state.jetlink_badge = ('jetSON', 'active')
  ui.state.ui_state.jetlink_detail = 'Old error'
  monkeypatch.setattr(ui.state, 'read_jetlink_status', Mock(side_effect=error))
  ui.state.ui_state._update_jetlink_status()
  assert ui.state.ui_state.jetlink_badge is None
  assert ui.state.ui_state.jetlink_detail == ''


@pytest.mark.parametrize('kind', ['wide', 'compact'])
@pytest.mark.parametrize('detail', ['', 'Link: IPC disconnected'])
def test_both_huds_draw_current_badge(ui, monkeypatch, kind, detail):
  module = getattr(ui, kind)
  renderer = object.__new__(module.HudRenderer)
  renderer._can_draw_top_icons = True
  renderer._new_ui_active = False
  renderer._status_timer_visible = False
  renderer.is_cruise_available = False
  renderer._font_display = None
  renderer._exp_button = SimpleNamespace(render=Mock())
  renderer._plot_renderer = SimpleNamespace(update=Mock(), paint=Mock())
  renderer._overlay_state = None
  renderer._overlay = SimpleNamespace(draw=lambda rect, paint, key, **kwargs: paint(rect))
  renderer._blink_timer = renderer._disp_timer = renderer._show_plot_mode = 0
  renderer._traffic_light_info = None
  renderer._show_wheel_critical = False
  for name in ('_draw_set_speed', '_draw_steering_wheel', '_draw_date_time', '_draw_tpms',
               '_refresh_hud_params', '_draw_cruise_speed_animation'):
    setattr(renderer, name, Mock())
  monkeypatch.setattr(rl, 'draw_rectangle_gradient_v', Mock())
  draw = Mock()
  monkeypatch.setattr(module, 'draw_jetlink_badge', draw)
  draw_detail = Mock()
  monkeypatch.setattr(module, 'draw_jetlink_detail', draw_detail)
  ui.state.ui_state.jetlink_badge = ('jetSON ERROR', 'error') if detail else ('jetSON READY', 'ready')
  ui.state.ui_state.jetlink_detail = detail
  width, height = (2160, 1080) if kind == 'wide' else (476, 240)
  renderer._render(rl.Rectangle(30, 20, width, height))
  if kind == 'compact':
    renderer.draw_jetlink_status(rl.Rectangle(30, 20, width, height))
  assert draw.call_args.args[0] == ui.state.ui_state.jetlink_badge
  bounds = draw.call_args.args[1]
  assert 30 <= bounds.x < bounds.x + bounds.width <= 30 + width
  assert 20 <= bounds.y < bounds.y + bounds.height <= 20 + height
  if detail:
    assert draw_detail.call_args.args[0] == detail
    detail_bounds = draw_detail.call_args.args[1]
    assert 30 <= detail_bounds.x < detail_bounds.x + detail_bounds.width <= 30 + width
    assert bounds.y + bounds.height <= detail_bounds.y < detail_bounds.y + detail_bounds.height <= 20 + height
  if kind == 'compact':
    assert draw.call_args.kwargs == {'stacked': True}
    # The existing gear box ends at x + 373; the header/traffic row is above y + 100.
    assert bounds.x >= 30 + 373 and bounds.y >= 20 + 100
    draw.reset_mock()
    draw_detail.reset_mock()
    renderer.set_can_draw_top_icons(False)
    renderer._render(rl.Rectangle(0, 0, width, height))
    draw.assert_not_called()
    draw_detail.assert_not_called()
    renderer.draw_jetlink_status(rl.Rectangle(0, 0, width, height))
    draw.assert_called_once()


@pytest.mark.parametrize('status', [log.SelfdriveState.AlertStatus.normal,
                                  log.SelfdriveState.AlertStatus.userPrompt,
                                  log.SelfdriveState.AlertStatus.critical])
@pytest.mark.parametrize('size', [log.SelfdriveState.AlertSize.small,
                                log.SelfdriveState.AlertSize.mid,
                                log.SelfdriveState.AlertSize.full])
def test_compact_jetlink_survives_ordinary_alerts_but_yields_to_safety_alerts(ui, monkeypatch, status, size):
  renderer = object.__new__(ui.compact.HudRenderer)
  draw, detail = Mock(), Mock()
  monkeypatch.setattr(ui.compact, 'draw_jetlink_badge', draw)
  monkeypatch.setattr(ui.compact, 'draw_jetlink_detail', detail)
  ui.state.ui_state.jetlink_badge = ('jetSON READY', 'ready')
  ui.state.ui_state.jetlink_detail = 'Connection details'
  renderer.draw_jetlink_status(rl.Rectangle(30, 20, 476, 240), SimpleNamespace(status=status, size=size))
  if status == log.SelfdriveState.AlertStatus.normal and size != log.SelfdriveState.AlertSize.full:
    draw.assert_called_once()
    bounds = draw.call_args.args[1]
    assert 30 + 373 <= bounds.x < bounds.x + bounds.width <= 506
    assert 20 + 120 <= bounds.y < bounds.y + bounds.height <= 260
    assert draw.call_args.kwargs == {'stacked': True}
  else:
    draw.assert_not_called()
  detail.assert_not_called()


def test_compact_status_is_drawn_after_alert_in_real_view():
  source = (ROOT / 'selfdrive/ui/mici/onroad/augmented_road_view.py').read_text()
  assert source.index('self._alert_renderer.render(self._content_rect)') < source.index(
    'self._hud_renderer.draw_jetlink_status(self._content_rect, alert_to_render)')


@pytest.mark.parametrize(('values', 'fragments'), [
  ({'state': 'connecting'}, ['handshake not complete']),
  ({'state': 'loading'}, ['Model preparation']),
  ({'state': 'loading', 'link_fields': {'preparation': {'stage': 'download', 'fraction': .42, 'message': 'Fetching model'}}},
   ['download 42%', 'Fetching model']),
  ({'state': 'loading', 'link_fields': {'preparation': {'stage': 'waiting', 'message': 'Park before engine creation'}}},
   ['waiting', 'Park before engine creation']),
  ({'state': 'retrying', 'link_fields': {'error': 'gadget write had no reader for 15.000s'}}, ['Link: gadget write had no reader']),
  ({'state': 'blocked', 'link_fields': {'error': 'USB cleanup failed'}}, ['Link: USB cleanup failed']),
  ({'error': 'IPC disconnected'}, ['Model: IPC disconnected']),
  ({'state': 'retrying', 'error': 'IPC disconnected', 'link_fields': {'error': 'No such device'}},
   ['Model: IPC disconnected', 'Link: No such device']),
  ({'link_age': 3., 'link_fields': {'error': 'Old link error'}}, ['status missing or stale']),
  ({'link_age': 3., 'error': 'IPC disconnected'}, ['Model: IPC disconnected', 'status missing or stale']),
  ({'health_age': 3.}, ['health data missing, stale or unknown']),
  ({'severity': 'error', 'link_fields': {'telemetry': {'carrot_health': {'severity': 'error', 'age_s': 0., 'reason': 'GPU 101C'}}}},
   ['Host: GPU 101C']),
  ({'severity': 'error'}, ['Host health error; no details reported']),
  ({'severity': 'warning'}, ['Host health warning']),
  ({'state': 'retrying'}, ['no detailed error was reported']),
])
def test_wait_and_error_details_use_reported_reasons(status_files, values, fragments):
  status_files(**values)
  status, detail = jetlink_status.display_status()
  assert status[1] in ('error', 'loading')
  assert all(fragment in detail for fragment in fragments)
  assert 'Old link error' not in detail


@pytest.mark.parametrize('values', [{}, {'active': True}, {'state': 'waiting'}])
def test_healthy_or_absent_host_has_no_detail(status_files, values):
  status_files(**values)
  assert jetlink_status.display_status()[1] == ''


def test_stale_model_error_is_not_reported_as_current(status_files):
  status_files(state='retrying', error='Old model error', model_fields={'updated': 90.}, link_fields={'error': 'Current link error'})
  detail = jetlink_status.display_status()[1]
  assert detail == 'Link: Current link error'


def test_badge_and_detail_share_one_snapshot(status_files, monkeypatch):
  status_files(state='loading')
  read = Mock(wraps=jetlink_status._read)
  monkeypatch.setattr(jetlink_status, '_read', read)
  jetlink_status.display_status()
  assert [call.args[0] for call in read.call_args_list] == [jetlink_status.LINK_STATUS, jetlink_status.MODEL_STATUS]


def test_details_are_bounded_and_control_characters_removed(status_files):
  status_files(state='retrying', link_fields={'error': 'bad\n\t\x00\x1b' + 'x' * 1000})
  detail = jetlink_status.display_status()[1]
  assert len(detail) <= 306 and detail.isprintable()


def test_detail_page_timer_resets_only_on_changed_status(ui, monkeypatch):
  state = ui.state.ui_state
  read = Mock(return_value=(('jetSON ERROR', 'error'), 'Link: timed out'))
  monkeypatch.setattr(ui.state, 'read_jetlink_status', read)
  for now in (100., 101., 105.):
    monkeypatch.setattr(ui.state.time, 'monotonic', lambda now=now: now)
    state._update_jetlink_status()
    assert state.jetlink_detail_since == 100.
  read.return_value = (('jetSON READY', 'ready'), '')
  monkeypatch.setattr(ui.state.time, 'monotonic', lambda: 106.)
  state._update_jetlink_status()
  assert state.jetlink_detail == '' and state.jetlink_detail_since == 106.


def test_detail_pages_preserve_all_lines(ui, monkeypatch):
  drawing = ui.drawing
  monkeypatch.setattr(drawing.gui_app, 'font', lambda weight: None)
  monkeypatch.setattr(drawing, 'measure_text_cached', lambda *args: rl.Vector2(12, 12))
  lines = [f'line {i}' for i in range(8)]
  monkeypatch.setattr(drawing, 'wrap_text', lambda *args: lines)
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', Mock())
  label = Mock()
  monkeypatch.setattr(drawing, 'gui_label', label)
  seen = []
  for elapsed in (0., 4., 8.):
    label.reset_mock()
    drawing.draw_jetlink_detail('error', rl.Rectangle(386, 170, 80, 52), 11, elapsed)
    seen.extend(call.args[1] for call in label.call_args_list)
    for call in label.call_args_list:
      bounds = call.args[0]
      assert 386 <= bounds.x and bounds.x + bounds.width <= 466
      assert 170 <= bounds.y and bounds.y + bounds.height <= 222
      assert call.kwargs['elide_right'] is False
  assert seen == lines
  label.reset_mock()
  drawing.draw_jetlink_detail('error', rl.Rectangle(386, 170, 80, 52), 11, 12.)
  assert label.call_args_list[0].args[1] == lines[0]


@pytest.mark.parametrize(('status', 'font_size'), [(('jetSON READY', 'ready'), 22), (('jetSON', 'active'), 38),
                                                 (('jetSON ERROR', 'error'), 22), (('Jetson update pending', 'loading'), 22)])
def test_badge_stays_inside_bounds_and_uses_status_color(ui, monkeypatch, status, font_size):
  drawing = ui.drawing
  monkeypatch.setattr(drawing.gui_app, 'font', lambda weight: None)
  monkeypatch.setattr(drawing, 'measure_text_cached', lambda font, text, size: rl.Vector2(len(text) * size * .6, size))
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', Mock())
  border = Mock()
  monkeypatch.setattr(rl, 'draw_rectangle_rounded_lines_ex', border)
  label = Mock()
  monkeypatch.setattr(drawing, 'gui_label', label)
  bounds = rl.Rectangle(10, 20, 180, 50)
  rect = drawing.draw_jetlink_badge(status, bounds, font_size)
  assert 10 <= rect.x < rect.x + rect.width <= 190
  assert rect.y == 20 and rect.height == 50
  assert border.call_args.args[-1] == drawing.COLORS[status[1]]
  assert label.call_args.kwargs['elide_right']


def test_compact_badge_keeps_host_and_status_on_separate_lines(ui, monkeypatch):
  drawing = ui.drawing
  monkeypatch.setattr(drawing.gui_app, 'font', lambda weight: None)
  monkeypatch.setattr(drawing, 'measure_text_cached',
                      lambda font, text, size: rl.Vector2(max(map(len, text.split('\n'))) * 8, len(text.split('\n')) * size))
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', Mock())
  monkeypatch.setattr(rl, 'draw_rectangle_rounded_lines_ex', Mock())
  label = Mock()
  monkeypatch.setattr(drawing, 'gui_label', label)
  rect = drawing.draw_jetlink_badge(('jetSON READY', 'ready'), rl.Rectangle(386, 164, 80, 58), 18, stacked=True)
  assert rect is not None
  assert label.call_args.args[1] == 'jetSON\nREADY'
  assert label.call_args.kwargs['alignment'] == drawing.TextAlignment.CENTER


@pytest.mark.parametrize(('status', 'bounds'), [(None, (0, 0, 180, 38)), (('', 'ready'), (0, 0, 180, 38)),
                                             (('Jetson', 'unknown'), (0, 0, 180, 38)),
                                             (('jetSON READY', 'ready'), (0, 0, 1, 38)),
                                             (('jetSON READY', 'ready'), (0, 0, 180, 1))])
def test_hidden_or_invalid_badge_draws_nothing(ui, monkeypatch, status, bounds):
  drawing = ui.drawing
  monkeypatch.setattr(drawing.gui_app, 'font', lambda weight: None)
  monkeypatch.setattr(drawing, 'measure_text_cached', lambda font, text, size: rl.Vector2(len(text) * size, size))
  draw = Mock()
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', draw)
  assert drawing.draw_jetlink_badge(status, rl.Rectangle(*bounds), 22) is None
  draw.assert_not_called()
