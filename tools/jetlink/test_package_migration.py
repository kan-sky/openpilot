"""Package migration contracts without device I/O or native graphics imports."""
import ast
import asyncio
import hashlib
import io
import json
from pathlib import Path
import tarfile
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

import build_cluster_bundle as builder
import install_cluster as installer
from test_cluster_install import bundle_at, runtime_at
from test_remote_support import api
from openpilot.common import jetlink_status as status

ROOT = Path(__file__).resolve().parents[2]


def test_remote_helper_and_maintenance_import_use_repository_root():
  from openpilot.selfdrive.carrot import jetson_maintenance
  from tools.jetlink.manage_jetson import daemon
  assert jetson_maintenance.daemon is daemon
  assert api.ROOT == ROOT
  assert api.HELPER == ROOT / 'tools/jetlink/remote_support.py'
  source = ROOT / 'openpilot/selfdrive/carrot/comma_remote.py'
  tree = ast.parse(source.read_text())
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  scope = {'Path': Path, '__file__': str(source)}
  exec(compile(ast.Module(body=main.body[:1], type_ignores=[]), str(source), 'exec'), scope)
  assert scope['helper'] == ROOT / 'tools/jetlink/setup_comma_remote.py'


@pytest.mark.parametrize('age,fresh', [(0., True), (2.999, True), (3., False), (-.1, False)])
def test_fresh_keeps_path_api_for_hud_protocol(tmp_path, age, fresh):
  path = tmp_path / 'status.json'
  data = {'updated': 100. - age, 'active': True}
  path.write_text(json.dumps(data))
  assert status._fresh(path, 100.) == (data if fresh else {})


def test_display_and_web_take_one_snapshot_each(tmp_path, monkeypatch):
  link, model = tmp_path / 'link.json', tmp_path / 'model.json'
  link.write_text(json.dumps({'updated': 100., 'state': 'loading', 'peer': {'carrot_host': 'jetson'},
                              'preparation': {'stage': 'download', 'fraction': .5, 'message': 'Test model'}}))
  model.write_text(json.dumps({'updated': 100., 'active': False}))
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 100.)
  read = Mock(wraps=status._read)
  monkeypatch.setattr(status, '_read', read)
  badge, detail = status.display_status()
  assert badge == ('jetSON 모델 준비 중', 'loading') and detail == 'download 50%: Test model'
  assert [call.args[0] for call in read.call_args_list] == [link, model]
  read.reset_mock()
  value = status.web_status()
  assert value['detail'] == detail and value['fresh']
  assert [call.args[0] for call in read.call_args_list] == [link, model]


def test_overlay_lists_new_package_and_standalone_navdy_relay():
  files = builder.source_files()
  assert installer.NAVDY_RELAY in files
  assert any(name.startswith('openpilot/selfdrive/carrot/cluster/') for name in files)
  assert not any(name.startswith(('selfdrive/', 'common/', 'system/')) for name in files)
  assert not any('/tests/' in name for name in files)


@pytest.mark.parametrize('field,value', [('format', 1), ('package_layout', 'legacy'), ('hud_capability', 'hl_cluster_v1')])
def test_legacy_layout_and_wire_bundles_are_rejected_before_extract(tmp_path, field, value):
  bundle, _ = bundle_at(tmp_path)
  with tarfile.open(bundle) as archive:
    files = {member.name: archive.extractfile(member).read() for member in archive.getmembers()}
  manifest = json.loads(files['HL_CLUSTER_MANIFEST.json'])
  manifest[field] = value
  files['HL_CLUSTER_MANIFEST.json'] = json.dumps(manifest).encode()
  with tarfile.open(bundle, 'w:gz') as archive:
    for name, data in files.items():
      member = tarfile.TarInfo(name)
      member.size = len(data)
      archive.addfile(member, io.BytesIO(data))
  out = tmp_path / 'out'
  out.mkdir()
  with pytest.raises(ValueError, match='Wrong runtime format'):
    installer.extract_verified(bundle, out, hashlib.sha256(bundle.read_bytes()).hexdigest())
  assert not list(out.iterdir())


def test_legacy_installed_base_is_not_silently_migrated(tmp_path):
  runtime = runtime_at(tmp_path)
  (runtime / 'current/openpilot/__init__.py').unlink()
  with pytest.raises(ValueError, match='Incomplete upstream runtime'):
    installer.check(runtime)


def test_navdy_prerequisite_never_opens_usb_or_starts_adb(tmp_path, monkeypatch):
  monkeypatch.setattr(installer.subprocess, 'run', lambda *a, **kw: pytest.fail('No device or service commands'))
  monkeypatch.setattr(installer.shutil, 'which', lambda _: None)
  installer.check_navdy_prerequisites(tmp_path)
  relay = tmp_path / installer.NAVDY_RELAY
  relay.parent.mkdir(parents=True)
  relay.touch()
  with pytest.raises(ValueError, match='requires adb'):
    installer.check_navdy_prerequisites(tmp_path)
  monkeypatch.setattr(installer.shutil, 'which', lambda _: '/usr/bin/adb')
  installer.check_navdy_prerequisites(tmp_path)


def hud_method(relative, name, scope):
  tree = ast.parse((ROOT / relative).read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HudRenderer')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  exec(compile(ast.Module(body=[method], type_ignores=[]), relative, 'exec'), scope)
  return scope[name]


@pytest.mark.parametrize('active,recovery', [(False, None), (True, None), (False, ('eGPU RECOVER', 'loading'))])
@pytest.mark.parametrize('alert_status,size', [(None, None), ('normal', 'small'), ('normal', 'full'), ('critical', 'small')])
def test_compact_status_yields_to_egpu_recovery_and_safety_alerts(active, recovery, alert_status, size):
  draw, detail = Mock(), Mock()
  def rectangle(x, y, width, height):
    return NS(x=x, y=y, width=width, height=height)
  state = NS(usbgpu_active=active, usbgpu_recovery_badge=recovery, jetlink_badge=('jetSON ERROR', 'error'),
             jetlink_detail='Link unavailable', jetlink_detail_since=100.)
  scope = {'rl': NS(Rectangle=rectangle), 'ui_state': state, 'time': NS(monotonic=lambda: 101.),
           'draw_jetlink_badge': draw, 'draw_jetlink_detail': detail,
           'log': NS(SelfdriveState=NS(AlertStatus=NS(normal='normal'), AlertSize=NS(full='full')))}
  method = hud_method('openpilot/selfdrive/ui/mici/onroad/hud_renderer.py', 'draw_jetlink_status', scope)
  alert = NS(status=alert_status, size=size) if alert_status else None
  method(None, rectangle(30, 20, 476, 240), alert)
  visible = not (active or recovery or alert_status == 'critical' or size == 'full')
  assert draw.called == visible
  assert detail.called == (visible and alert is None)
  if draw.called:
    bounds = draw.call_args.args[1]
    assert 30 + 373 <= bounds.x < bounds.x + bounds.width <= 506
    assert 20 + 120 <= bounds.y < bounds.y + bounds.height <= 260


def test_compact_detail_is_uncached_and_after_alerts():
  source = (ROOT / 'openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py').read_text()
  assert source.index('self._alert_renderer.render(self._content_rect)') < source.index(
    'self._hud_renderer.draw_jetlink_status(self._content_rect, alert_to_render)')


def test_legacy_brightness_is_hidden_without_mutating_catalog_or_value(monkeypatch):
  from copy import deepcopy
  from openpilot.selfdrive.carrot.server.services import params, settings
  item = {'name': 'ClusterHudBrightness', 'default': 80, 'min': 0, 'max': 100}
  groups = {'HUD': [item, {'name': 'ClusterHudTheme'}]}
  categories = [{'groups': [{'id': 'HUD', 'sections': [{'items': ['ClusterHudBrightness', 'ClusterHudTheme']}]}]}]
  original = deepcopy((groups, categories))
  view, counts, menu, hidden = settings.filter_settings_catalog_for_brand(
    groups, [{'group': 'HUD', 'count': 2}], categories, '', hidden_settings=params.READ_ONLY_PARAMS)
  assert hidden == {'ClusterHudBrightness'}
  assert [entry['name'] for entry in view['HUD']] == ['ClusterHudTheme']
  assert counts[0]['count'] == 1
  assert menu[0]['groups'][0]['sections'][0]['items'] == ['ClusterHudTheme']
  assert (groups, categories) == original
  monkeypatch.setattr(params, 'HAS_PARAMS', False)
  monkeypatch.setattr(params, '_mem_store', {'ClusterHudBrightness': '80'})
  with pytest.raises(ValueError, match='cluster theme'):
    params.set_param_value('ClusterHudBrightness', 100, item)
  assert params._mem_store['ClusterHudBrightness'] == '80'


def test_brightness_restore_preview_never_offers_a_write(monkeypatch):
  from openpilot.selfdrive.carrot.server.services import params
  monkeypatch.setattr(params, 'HAS_PARAMS', True)
  monkeypatch.setattr(params, 'Params', lambda: NS())
  monkeypatch.setattr(params, 'ParamKeyType', NS())
  monkeypatch.setattr(params, '_catalog_definitions', lambda: {})
  preview = params.preview_param_restore_values({'ClusterHudBrightness': 100})
  assert preview['summary']['skipped'] == 1
  assert preview['entries'][0]['apply'] is False
  write = Mock(side_effect=AssertionError('Legacy value must be preserved'))
  monkeypatch.setattr(params, 'set_param_value', write)
  params.restore_param_values_from_backup({'ClusterHudBrightness': 100})
  write.assert_not_called()


def test_brightness_http_write_is_forbidden_before_any_parameter_io():
  from aiohttp import web
  from openpilot.selfdrive.carrot.server.services.params import INTERNAL_SESSION_PARAMS, READ_ONLY_PARAMS
  source = ROOT / 'openpilot/selfdrive/carrot/server/features/params.py'
  tree = ast.parse(source.read_text())
  method = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == 'api_param_set')
  scope = {'web': web, 'INTERNAL_SESSION_PARAMS': INTERNAL_SESSION_PARAMS, 'READ_ONLY_PARAMS': READ_ONLY_PARAMS}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
  async def body():
    return {'name': 'ClusterHudBrightness', 'value': 100}
  response = asyncio.run(scope['api_param_set'](NS(json=body)))
  assert response.status == 403
