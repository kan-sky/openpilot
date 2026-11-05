"""Real Wayon drawing/filter code with isolated read-only UI inputs."""
import hashlib
import importlib.util
from collections import namedtuple
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log

ONROAD = Path(__file__).parents[1] / 'onroad'


@pytest.fixture
def scene(monkeypatch):
  now, calls = [10.], []
  rl = NS(Rectangle=namedtuple('Rectangle', 'x y width height'), Vector2=namedtuple('Vector2', 'x y'),
          Color=namedtuple('Color', 'r g b a'), get_time=lambda: now[0])
  for name in ('draw_texture_ex', 'draw_texture_pro'):
    setattr(rl, name, lambda *args, name=name: calls.append((name, *args)))
  gui = NS(target_fps=20, texture=lambda name, w, h: NS(name=name, width=w, height=h))
  class Widget:
    def set_rect(self, rect):
      self._rect = rect
    def set_position(self, x, y):
      self._rect = self._rect._replace(x=x, y=y)
    def set_enabled(self, enabled):
      self.enabled = enabled
    def render(self):
      self._update_state()
      self._render(self._rect)
  class SM(dict):
    valid = dict.fromkeys(('driverMonitoringState', 'driverStateV2'), True)
    alive = valid.copy()
    recv_frame = dict.fromkeys(valid, 101)
    logMonoTime = dict.fromkeys(valid, 10_000_000_000)
    def all_checks(self, services):
      return all(self.valid[s] and self.alive[s] for s in services)
  dm = log.DriverMonitoringState.new_message(activePolicy='vision')
  dm.visionPolicyState.faceDetected = True
  dm.visionPolicyState.awarenessPercent = 100
  dm.visionPolicyState.pose.pitch = 0
  sm = SM(driverMonitoringState=dm, driverStateV2=log.DriverStateV2.new_message(frameId=100),
          selfdriveState=log.SelfdriveState.new_message(enabled=True))
  ui = NS(sm=sm, started=True, started_frame=100, always_on_dm=False, is_onroad=lambda: True)
  for name, value in {
    'pyray': rl, 'openpilot.selfdrive.ui.ui_state': NS(ui_state=ui),
    'openpilot.system.ui.lib.application': NS(gui_app=gui),
    'openpilot.system.ui.widgets': NS(Widget=Widget),
  }.items():
    monkeypatch.setitem(sys.modules, name, value)
  modules = []
  for filename in ('driver_state', 'wayon_driver_preview'):
    name = f'openpilot.selfdrive.ui.mici.onroad.{filename}'
    spec = importlib.util.spec_from_file_location(name, ONROAD / f'{filename}.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    modules.append(module)
  monkeypatch.setattr(modules[-1].time, 'monotonic', lambda: now[0])
  return NS(renderer=modules[-1].WayonDriverPreview(), original=modules[0].DriverStateRenderer, ui=ui, sm=sm,
            dm=dm, model=sm['driverStateV2'], now=now, calls=calls, rect=rl.Rectangle(17, 9, 536, 240))


def draw(s, frames=40):
  for _ in range(frames):
    s.calls.clear()
    s.renderer.draw_onroad(s.rect, False)
  return s.calls


def test_original_artwork_colors_geometry_and_no_eyes_or_labels(scene):
  s = scene
  calls = draw(s)
  assert not s.renderer.enabled
  assert s.renderer._render.__func__ is s.original._render
  assert s.renderer._rect == (33, 19, 60, 60)
  assert [c[1].name.rsplit('/', 1)[-1] for c in calls] == ['dm_background.png', 'dm_person.png', 'dm_cone.png']
  assert all(c[1].width == 52 for c in calls[1:])
  assert calls[-1][-1] == (0, 255, 64, 254)
  assert calls[-1][-2] == pytest.approx(0)
  assert not hasattr(s.renderer, 'client') and not hasattr(s.renderer, '_font')


def test_phone_crossfade_animation_and_return_to_person(scene):
  s = scene
  s.dm.visionPolicyState.distractedTypes.phone = True
  calls = draw(s)
  phone = next(c for c in calls if c[1].name.endswith('phone.png'))
  assert phone[0] == 'draw_texture_pro' and phone[-1][-1] > 230
  assert phone[3].width != 52 and phone[-2] != 0
  assert not any(c[1].name.endswith('dm_person.png') for c in calls)
  s.dm.visionPolicyState.distractedTypes.phone = False
  assert not any(c[1].name.endswith('phone.png') for c in draw(s))
  assert any(c[1].name.endswith('dm_person.png') for c in s.calls)


@pytest.mark.parametrize('reason', ['awareness', 'distracted', 'lockout', 'always_on_lockout'])
def test_warning_uses_wayon_orange_without_changing_monitoring(scene, reason):
  s = scene
  if reason == 'awareness':
    s.dm.visionPolicyState.awarenessPercent = 90
  elif reason == 'distracted':
    s.dm.visionPolicyState.isDistracted = True
  else:
    setattr(s.dm, 'lockout' if reason == 'lockout' else 'alwaysOnLockout', True)
  s.sm.update({name: packet.as_reader() for name, packet in s.sm.items()})
  before = {name: packet.to_dict() for name, packet in s.sm.items()}
  assert draw(s)[-1][-1][:3] == (255, 115, 0)
  assert {name: packet.to_dict() for name, packet in s.sm.items()} == before


@pytest.mark.parametrize('service', ['driverMonitoringState', 'driverStateV2'])
@pytest.mark.parametrize(('field', 'value'), [('valid', False), ('alive', False), ('recv_frame', 100),
                                           ('logMonoTime', 0), ('logMonoTime', 9_500_000_000), ('logMonoTime', 11_000_000_000)])
def test_stale_data_never_retains_green_pose_or_phone(scene, service, field, value):
  s = scene
  s.dm.visionPolicyState.distractedTypes.phone = True
  draw(s)
  getattr(s.sm, field)[service] = value
  calls = draw(s, 1)
  assert [c[1].name.rsplit('/', 1)[-1] for c in calls] == ['dm_background.png', 'dm_person.png']
  assert not s.renderer.effective_active and s.renderer._phone_icon_filter.x == 0


@pytest.mark.parametrize('condition', ['camera', 'face', 'pose', 'wheel'])
def test_unavailable_face_or_vision_policy_never_looks_active(scene, condition):
  s = scene
  draw(s)
  if condition == 'camera':
    s.dm.cameraUnavailable = True
  elif condition == 'face':
    s.dm.visionPolicyState.faceDetected = False
  elif condition == 'pose':
    s.dm.visionPolicyState.pose.yaw = float('nan')
  else:
    s.dm.activePolicy = 'wheeltouch'
  assert not any(c[1].name.endswith('dm_cone.png') for c in draw(s, 1))


def test_fresh_publications_with_frozen_model_frame_remain_neutral_until_new_frame(scene):
  s = scene
  draw(s)
  for delta in (0.5, 0.1, 0.1):
    s.now[0] += delta
    s.sm.logMonoTime = dict.fromkeys(s.sm.logMonoTime, int(s.now[0] * 1e9))
    assert not any(c[1].name.endswith('dm_cone.png') for c in draw(s, 1))
  s.model.frameId += 1
  assert any(c[1].name.endswith('dm_cone.png') for c in draw(s, 1))


def test_hide_close_and_always_on_dm_gating(scene):
  s = scene
  draw(s)
  assert not s.renderer.draw_onroad(s.rect, True)
  assert s.renderer._fade_filter.x == 0
  s.sm['selfdriveState'].enabled = False
  assert not s.renderer.draw_onroad(s.rect, False)
  s.ui.always_on_dm = True
  assert s.renderer.draw_onroad(s.rect, False)
  s.dm.dm2Disabled = True
  assert not s.renderer.draw_onroad(s.rect, False)
  s.dm.dm2Disabled = False
  s.sm['selfdriveState'].alertSize = 'small'
  assert not s.renderer.draw_onroad(s.rect, False)
  s.sm['selfdriveState'].alertSize = 'none'
  s.ui.started = False
  assert not s.renderer.draw_onroad(s.rect, False)
  s.ui.started = True
  s.renderer.close()
  s.renderer.close()
  assert not s.renderer.draw_onroad(s.rect, False)


def test_new_drive_and_hidden_mode_reset_old_pose(scene):
  s = scene
  s.dm.visionPolicyState.pose.yaw = .2
  draw(s)
  assert s.renderer._yaw_filter.x < 0
  s.ui.started_frame = 99
  s.dm.visionPolicyState.pose.yaw = -.2
  draw(s, 1)
  assert s.renderer._yaw_filter.x > 0
  s.renderer.draw_onroad(s.rect, True)
  assert s.renderer._yaw_filter.x == 0 and s.renderer._fade_filter.x == 0


def test_assets_match_wayon_reference_bytes():
  assets = ONROAD.parents[2] / 'assets/icons_mici/onroad/driver_monitoring'
  expected = {
    'dm_person': 'e2772c6a9fe9c57099d347ad49f0cb7c906593f1fdf0e6dde96d104baf0200b0',
    'dm_cone': '5f5d4b7bd406114945e53f4d56089102ee83ba7d0a4590a371f7acab90c7ad53',
    'dm_background': 'cb89d9f11cf44992f92142aa5ad84e1ac700a2601aff2abab373e2a822af149e',
    'phone': 'bf73e864baa71f8ddb3dc4ee3ebc4f5aa475de82db9b189500e3ef65f961d266',
  }
  for name, digest in expected.items():
    assert hashlib.sha256((assets / f'{name}.png').read_bytes()).hexdigest() == digest
