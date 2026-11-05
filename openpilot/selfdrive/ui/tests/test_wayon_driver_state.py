"""Exercise the production renderer without starting the UI or accessing Params."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter


ROOT = Path(__file__).resolve().parents[3]


class Rect:
  def __init__(self, x=0, y=0, width=0, height=0):
    self.x, self.y, self.width, self.height = x, y, width, height


class Widget:
  def set_rect(self, rect):
    self._rect = rect


class State(dict):
  def __init__(self):
    super().__init__(driverMonitoringState=log.DriverMonitoringState.new_message(),
                     selfdriveState=log.SelfdriveState.new_message(),
                     driverStateV2=log.DriverStateV2.new_message())
    self.recv_frame = {'driverStateV2': 2}
    self.healthy = True

  def all_checks(self, _):
    return self.healthy


def load_renderer():
  calls = []
  rl = SimpleNamespace(Rectangle=Rect, Vector2=lambda x, y: SimpleNamespace(x=x, y=y),
                       Color=lambda *rgba: rgba, get_time=lambda: 0.225,
                       draw_texture_ex=lambda *args: calls.append(('ex', args)),
                       draw_texture_pro=lambda *args: calls.append(('pro', args)))
  gui = SimpleNamespace(target_fps=20, texture=lambda name, w, h: SimpleNamespace(name=name, width=w, height=h))
  state = State()
  ui = SimpleNamespace(sm=state, started_frame=1, is_onroad=lambda: True)
  ns = {'rl': rl, 'np': np, 'math': math, 'log': log, 'FirstOrderFilter': FirstOrderFilter,
        'Widget': Widget, 'gui_app': gui, 'ui_state': ui}
  path = ROOT / 'selfdrive/ui/mici/onroad/driver_state.py'
  tree = ast.parse(path.read_text())
  tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
  exec(compile(tree, str(path), 'exec'), ns)
  renderer = ns['DriverStateRenderer']()
  renderer.set_should_draw(True)
  state['driverMonitoringState'].activePolicy = log.DriverMonitoringState.MonitoringPolicy.vision
  state['driverMonitoringState'].visionPolicyState.awarenessPercent = 100
  return renderer, ns, calls


def settle(renderer, frames=80):
  for _ in range(frames):
    renderer._update_state()


def test_phone_icon_fades_in_and_returns_to_person():
  renderer, ns, calls = load_renderer()
  dm = ns['ui_state'].sm['driverMonitoringState']
  settle(renderer)
  assert renderer._phone_icon_filter.x == 0
  dm.visionPolicyState.distractedTypes.phone = True
  renderer._update_state()
  assert 0 < renderer._phone_icon_filter.x < 1
  settle(renderer)
  renderer._render(None)
  phone_calls = [args for kind, args in calls if kind == 'pro' and args[0] is renderer._phone_icon]
  assert len(phone_calls) == 1
  assert 0 < phone_calls[0][4] <= ns['PHONE_RING_DEGREES']
  assert phone_calls[0][2].width < renderer._rect.width
  assert renderer._phone_icon_filter.x > .999
  dm.visionPolicyState.distractedTypes.phone = False
  settle(renderer)
  assert renderer._phone_icon_filter.x < .001


@pytest.mark.parametrize('condition', ['stale', 'disabled', 'alert', 'before_start'])
def test_phone_icon_not_left_active_when_monitoring_hidden_or_stale(condition):
  renderer, ns, _ = load_renderer()
  sm = ns['ui_state'].sm
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = True
  settle(renderer)
  if condition == 'stale':
    sm.healthy = False
  elif condition == 'disabled':
    sm['driverMonitoringState'].dm2Disabled = True
  elif condition == 'alert':
    sm['selfdriveState'].alertSize = log.SelfdriveState.AlertSize.mid
  else:
    sm.recv_frame['driverStateV2'] = 0
  settle(renderer)
  assert renderer._phone_icon_filter.x < .001
  if condition != 'stale':
    assert not renderer.should_draw


def test_attention_cone_green_and_orange_without_changing_dm_state():
  renderer, ns, calls = load_renderer()
  sm = ns['ui_state'].sm
  dm = sm['driverMonitoringState']
  for awareness, color in [(100, ns['CONE_COLOR_GREEN']), (94, ns['CONE_COLOR_ORANGE'])]:
    dm.visionPolicyState.awarenessPercent = awareness
    sm['driverMonitoringState'] = dm.as_reader()
    before = dm.to_dict()
    for _ in range(80):
      renderer._update_state()
      renderer._render(None)
    assert calls[-1][1][-1][:3] == color
    assert dm.to_dict() == before


def test_visual_pose_comes_from_monitoring_packet_even_without_raw_orientation():
  renderer, ns, _ = load_renderer()
  dm = ns['ui_state'].sm['driverMonitoringState']
  dm.visionPolicyState.pose.pitch = .1
  dm.visionPolicyState.pose.yaw = .2
  settle(renderer)
  assert renderer._pitch_filter.x == pytest.approx(.1 + math.radians(6))
  assert renderer._yaw_filter.x == pytest.approx(-.2)
  assert renderer.should_draw


def test_phone_asset_is_real_png_not_lfs_pointer():
  data = (ROOT / 'selfdrive/assets/icons_mici/onroad/driver_monitoring/phone.png').read_bytes()
  assert data.startswith(b'\x89PNG\r\n\x1a\n')
  assert len(data) > 1000
