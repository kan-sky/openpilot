import asyncio
import base64
import hashlib
from contextlib import asynccontextmanager
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import pytest
from aiohttp import ClientError, web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.navigation.tests.test_map_client import setup as setup


@pytest.fixture
def feature(monkeypatch, tmp_path):
  name = 'openpilot.selfdrive.carrot.server.features.navigation_test'
  spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / 'features/navigation.py')
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, name, module)
  spec.loader.exec_module(module)
  values = {}
  monkeypatch.setattr(module, 'get_param_value', lambda key, default: values.get(key, default))
  monkeypatch.setattr(module, 'set_param_value', lambda key, value: values.__setitem__(key, value))
  module.values = values
  for key, filename in [('PREVIEW_PATH', 'map.png'), ('STATUS_PATH', 'status.json'), ('WEB_DEMAND_PATH', 'web-demand')]:
    monkeypatch.setattr(module.nav_preview, key, tmp_path / filename)
  monkeypatch.setattr(module, 'MODEL_STATUS_PATH', tmp_path / 'model.json')
  return module


@asynccontextmanager
async def client_for(feature, http=None):
  app = web.Application()
  if http is not None:
    app['http'] = http
  feature.register(app)
  async with TestClient(TestServer(app)) as client:
    yield client


def test_destination_roundtrip_clears_waypoints_without_changing_source(feature):
  feature.values.update(CarrotNaviSource=0, NavDestinationWaypoints='[[1,2]]')
  place = {'latitude': 37., 'longitude': 127., 'place_name': 'Test <script>name</script>'}
  async def run():
    async with client_for(feature) as client:
      assert (await client.post('/api/navigation/destination', json=place)).status == 200
      assert json.loads(feature.values['NavDestination']) == place
      assert feature.values['NavDestinationWaypoints'] == ''
      response = await client.get('/api/navigation/destination')
      body = await response.json()
      assert body['destination'] == place
      assert body['navigation_source'] == 'network' and body['destination_editable']
      assert not body['map_ready'] and not body['model_input_ready']
      assert body['pipeline_status'] == 'model_status_missing'
      assert response.headers['Cache-Control'] == 'no-store'
      assert (await client.delete('/api/navigation/destination')).status == 200
      assert feature.values['NavDestination'] == ''
      assert feature.values['CarrotNaviSource'] == 0
  asyncio.run(run())


@pytest.mark.parametrize('payload', [
  [], {}, None, {'latitude': 91, 'longitude': 127}, {'latitude': 0, 'longitude': 181},
  {'latitude': None, 'longitude': 0}, {'latitude': True, 'longitude': 0},
  {'latitude': '37', 'longitude': 127}, {'latitude': float('nan'), 'longitude': 0},
  {'latitude': 0, 'longitude': float('inf')}, {'latitude': 0, 'longitude': 0, 'place_name': {}},
  {'latitude': 0, 'longitude': 0, 'place_name': 'x' * 257},
  {'latitude': 0, 'longitude': 0, 'place_name': '\nname'},
  {'latitude': 0, 'longitude': 0, 'place_name': 'External Navi'},
])
def test_rejects_invalid_destination_without_writing(feature, payload):
  async def run():
    async with client_for(feature) as client:
      assert (await client.post('/api/navigation/destination', data=json.dumps(payload),
                                headers={'Content-Type': 'application/json'})).status == 400
      assert feature.values == {}
      assert not feature.nav_preview.WEB_DEMAND_PATH.exists()
  asyncio.run(run())


@pytest.mark.parametrize('values', [
  {'CarrotNaviSource': 1}, {'CarrotNaviSource': 'invalid'},
  {'NavDestination': json.dumps({'latitude': 37, 'longitude': 127, 'place_name': 'External Navi'})},
])
def test_preserves_navdy_and_external_destination(feature, values):
  feature.values.update(values)
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert not status['destination_editable']
      assert (await client.post('/api/navigation/destination', json={'latitude': 0, 'longitude': 0})).status == 409
      assert (await client.delete('/api/navigation/destination')).status == 409
      assert feature.values == values
  asyncio.run(run())


@pytest.mark.parametrize('headers', [{'Origin': 'http://example.invalid'}, {'Origin': 'null'},
                                    {'Sec-Fetch-Site': 'cross-site'}, {'Sec-Fetch-Site': 'same-site'}])
def test_rejects_cross_origin_operations(feature, headers):
  async def run():
    async with client_for(feature) as client:
      for method, path in [('GET', 'destination'), ('POST', 'destination'), ('DELETE', 'destination'),
                           ('GET', 'search?q=Seoul'), ('GET', 'map.png')]:
        assert (await client.request(method, '/api/navigation/' + path, headers=headers)).status == 403
      assert feature.values == {}
  asyncio.run(run())


def test_request_size_type_json_and_storage_errors(feature, monkeypatch):
  async def run():
    async with client_for(feature) as client:
      url = '/api/navigation/destination'
      assert (await client.post(url, data='{}')).status == 415
      assert (await client.post(url, data='{', headers={'Content-Type': 'application/json'})).status == 400
      assert (await client.post(url, json={'latitude': 0, 'longitude': 0, 'extra': 'x' * 4096})).status == 413
      def fail(*args):
        raise OSError('private path')
      monkeypatch.setattr(feature, 'set_param_value', fail)
      for response in [await client.post(url, json={'latitude': 0, 'longitude': 0}), await client.delete(url)]:
        assert response.status == 503
        assert 'private path' not in await response.text()
  asyncio.run(run())


def test_malformed_saved_destination_and_missing_key(feature):
  feature.values['NavDestination'] = '{'
  async def run():
    async with client_for(feature) as client:
      assert (await (await client.get('/api/navigation/destination')).json())['destination'] is None
      assert (await client.get('/api/navigation/search?q=Seoul')).status == 409
      for query in ('', 'a', 'x' * 201):
        assert (await client.get('/api/navigation/search', params={'q': query})).status == 400
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204
      assert response.headers['X-Nav-Map-Status'] == 'waiting'
      assert feature.nav_preview.WEB_DEMAND_PATH.exists()
  asyncio.run(run())


PNG = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aMZkAAAAASUVORK5CYII=')


def provenance(source='mapbox', route_id='123', *, now=None):
  now = time.monotonic() if now is None else now
  return {'source': source, 'route_id': route_id, 'updated': now, 'route_expires': now + 3., 'context_expires': now + 2.}


def write_preview(feature, reason='ready', **changes):
  feature.nav_preview.PREVIEW_PATH.write_bytes(PNG)
  metadata = {'reason': reason, 'provenance': provenance(), 'updated': time.monotonic(),
              'rendered_at': time.monotonic(), 'png_sha256': hashlib.sha256(PNG).hexdigest()}
  metadata.update(changes)
  feature.nav_preview.STATUS_PATH.write_text(json.dumps(metadata))


def write_model(feature, **changes):
  value = {'updated': time.monotonic(), 'active': True, 'nav_model': True, 'nav_input_supported': True,
           'nav_route_available': True, 'nav_tbt_available': True, 'nav_map_available': True, 'nav_input_reason': 'ready',
           'nav_provenance': provenance()}
  value.update(changes)
  feature.MODEL_STATUS_PATH.write_text(json.dumps(value))


def test_live_n16_status_and_real_png_without_parameter_writes(feature):
  write_preview(feature)
  write_model(feature)
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert status['map_ready'] and status['map_status_fresh']
      assert status['model_input_ready'] and status['model_status_fresh'] and status['model_active']
      assert status['pipeline_status'] == status['reason'] == 'ready'
      assert status['destination'] is None
      response = await client.get('/api/navigation/map.png')
      assert response.status == 200 and await response.read() == PNG
      assert response.headers['Content-Type'] == 'image/png'
      assert response.headers['X-Nav-Map-Status'] == 'ready'
      assert response.headers['Cache-Control'] == 'no-store'
      assert response.headers['X-Content-Type-Options'] == 'nosniff'
      assert feature.nav_preview.WEB_DEMAND_PATH.exists()
      assert feature.values == {}
  asyncio.run(run())


@pytest.mark.parametrize('changes,reason', [
  ({'active': False}, 'model_inactive'), ({'active': 'true'}, 'model_inactive'),
  ({'nav_model': False}, 'unsupported_model'), ({'nav_model': 1}, 'unsupported_model'),
  ({'nav_input_supported': False}, 'unsupported_model'), ({'nav_input_supported': None}, 'unsupported_model'),
  ({'nav_route_available': False}, 'route_unavailable'), ({'nav_route_available': 1}, 'route_unavailable'),
  ({'nav_tbt_available': False}, 'tbt_unavailable'), ({'nav_tbt_available': 'true'}, 'tbt_unavailable'),
  ({'nav_map_available': False}, 'map_not_yet_available'), ({'nav_map_available': None}, 'map_not_yet_available'),
  ({'nav_input_reason': 'context_expired'}, 'context_expired'),
  ({'nav_input_reason': 'ready?access_token=sk.secret'}, 'navigation_unavailable'),
  ({'nav_input_reason': []}, 'navigation_unavailable'),
])
def test_model_requires_all_live_input_flags_not_a_preview_or_selection(feature, changes, reason):
  write_preview(feature)
  write_model(feature, **changes)
  feature.values.update(NavDestination=json.dumps({'latitude': 37, 'longitude': 127}), JetlinkModel='N16')
  status = feature.navigation_status('network')
  assert status['map_ready']
  assert not status['model_input_ready']
  assert status['pipeline_status'] == reason
  assert 'sk.secret' not in json.dumps(status)


@pytest.mark.parametrize('updated', [None, True, '100', 0, -1, float('nan'), float('inf'), 10**400, 'stale', 'future'])
def test_model_status_requires_recent_monotonic_timestamp(feature, updated):
  if updated == 'stale':
    updated = time.monotonic() - 2.5
  elif updated == 'future':
    updated = time.monotonic() + 10
  write_model(feature, updated=updated)
  status = feature.navigation_status('network')
  assert not status['model_status_fresh'] and not status['model_input_ready']
  assert not status['model_active'] and not status['nav_route_available']


def test_model_freshness_boundary_and_independence_from_preview(feature, monkeypatch):
  monkeypatch.setattr(feature.time, 'monotonic', lambda: 102.499)
  write_preview(feature)
  feature.nav_preview.PREVIEW_PATH.unlink()
  write_model(feature, updated=100)
  assert feature.navigation_status('network')['model_input_ready']
  assert not feature.navigation_status('network')['map_ready']
  monkeypatch.setattr(feature.time, 'monotonic', lambda: 102.5)
  status = feature.navigation_status('network')
  assert not status['model_input_ready'] and status['reason'] == 'model_status_stale'


@pytest.mark.parametrize('payload', ['{', '[]', 'null', 'true', '"secret"', 'x' * 65537])
def test_malformed_or_oversized_model_status_is_redacted(feature, payload):
  feature.MODEL_STATUS_PATH.write_text(payload)
  status = feature.navigation_status('network')
  assert status['reason'] == 'model_status_unavailable'
  assert not status['model_input_ready']
  assert 'secret' not in json.dumps(status)


@pytest.mark.parametrize('reason', ['idle', 'route', 'position', 'instruction', 'tiles', 'route_position',
                                   'context_changed', 'slow', 'secret\r\naccess_token=sk.secret', {'secret': 'value'}])
def test_nonready_preview_returns_reason_without_placeholder_png(feature, reason):
  write_preview(feature, reason)
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert not status['map_ready'] and not status['model_input_ready']
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204 and not await response.read()
      expected = reason if isinstance(reason, str) and reason in feature.MAP_REASONS else 'unavailable'
      assert status['map_reason'] == response.headers['X-Nav-Map-Status'] == expected
      assert 'secret' not in str(response.headers) + json.dumps(status)
  asyncio.run(run())


@pytest.mark.parametrize('filename', ['PREVIEW_PATH', 'STATUS_PATH'])
@pytest.mark.parametrize('condition', ['missing', 'stale', 'future'])
def test_preview_needs_both_fresh_image_and_fresh_reason(feature, filename, condition):
  write_preview(feature)
  path = getattr(feature.nav_preview, filename)
  if condition == 'missing':
    path.unlink()
  else:
    stamp = time.time() + (60 if condition == 'future' else -3)  # noqa: TID251 - filesystem mtime
    os.utime(path, (stamp, stamp))
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert not status['map_ready'] and not status['map_status_fresh']
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204
      assert response.headers['X-Nav-Map-Status'] == ('waiting' if condition == 'missing' else 'stale')
  asyncio.run(run())


@pytest.mark.parametrize('payload', [b'', b'<html>secret</html>', b'\x89PNG\r\n\x1a\n' + b'x' * 1048576])
def test_corrupt_or_oversized_image_is_not_served(feature, payload):
  write_preview(feature)
  feature.nav_preview.PREVIEW_PATH.write_bytes(payload)
  async def run():
    async with client_for(feature) as client:
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204 and not await response.read()
      assert response.headers['X-Nav-Map-Status'] == 'unavailable'
  asyncio.run(run())


@pytest.mark.parametrize('source,reason', [(1, 'provenance_unverified'), ('invalid', 'source_unavailable')])
def test_readonly_preview_allowed_but_previous_network_route_cannot_authorize_navdy(feature, source, reason):
  write_preview(feature)
  write_model(feature)
  feature.values['CarrotNaviSource'] = source
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert not status['destination_editable'] and not status['model_input_ready'] and not status['map_ready']
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204 and response.headers['X-Nav-Map-Status'] == reason
      assert feature.nav_preview.WEB_DEMAND_PATH.exists()
      assert (await client.post('/api/navigation/destination', json={'latitude': 0, 'longitude': 0})).status == 409
      assert (await client.delete('/api/navigation/destination')).status == 409
      assert feature.values == {'CarrotNaviSource': source}
  asyncio.run(run())


def test_external_destination_allows_preview_without_taking_ownership(feature, monkeypatch):
  write_preview(feature)
  feature.values['NavDestination'] = json.dumps({'latitude': 37, 'longitude': 127, 'place_name': 'External Navi'})
  original = feature.values.copy()
  monkeypatch.setattr(feature.nav_preview, 'WEB_DEMAND_PATH', feature.nav_preview.PREVIEW_PATH / 'not-a-directory')
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert status['map_ready'] and not status['destination_editable']
      assert not status['model_input_ready']
      assert (await client.get('/api/navigation/map.png')).status == 200
      assert feature.values == original
  asyncio.run(run())


@pytest.mark.parametrize('source,configured', [('navdy', 1), ('mapbox', 0), ('network-v1', 0), ('network-v2', 0)])
def test_matching_selected_route_can_show_map_and_model_without_destination_ownership(feature, source, configured):
  route = provenance(source, '18446744073709551615')
  write_preview(feature, provenance=route)
  write_model(feature, nav_provenance=route)
  feature.values['CarrotNaviSource'] = configured
  async def run():
    async with client_for(feature) as client:
      status = await (await client.get('/api/navigation/destination')).json()
      assert status['model_input_ready'] and status['map_ready']
      assert status['model_provenance_verified'] and status['map_provenance_verified']
      assert status['model_navigation_source'] == status['map_navigation_source'] == source
      assert status['model_route_id'] == status['map_route_id'] == route['route_id']
      assert status['destination_editable'] == (configured == 0)
      response = await client.get('/api/navigation/map.png')
      assert response.status == 200 and await response.read() == PNG
      assert response.headers['X-Nav-Map-Source'] == source
      assert response.headers['X-Nav-Map-Route'] == route['route_id']
  asyncio.run(run())


@pytest.mark.parametrize('model_state,reason', [('missing', 'model_status_missing'), ('inactive', 'model_inactive'),
                                              ('stale', 'model_status_stale'), ('unsupported', 'unsupported_model')])
def test_navdy_preview_does_not_promote_a_missing_or_off_jetson(feature, model_state, reason):
  feature.values['CarrotNaviSource'] = 1
  write_preview(feature, provenance=provenance('navdy'))
  if model_state != 'missing':
    changes = {'inactive': {'active': False}, 'stale': {'updated': time.monotonic() - 3.},
               'unsupported': {'nav_model': False}}[model_state]
    write_model(feature, nav_provenance=provenance('network-v2'), **changes)
  status = feature.navigation_status('navdy')
  assert status['map_ready']
  assert status['pipeline_status'] == reason and not status['model_input_ready']
  assert not status['nav_route_available'] and not status['nav_tbt_available'] and not status['nav_map_available']


@pytest.mark.parametrize('preview_state', ['missing', 'stale', 'legacy', 'no_gps'])
@pytest.mark.parametrize('model_state', ['missing', 'live'])
def test_offroad_missing_model_and_unproven_preview_do_not_claim_live_navdy_input(feature, preview_state, model_state):
  feature.values['CarrotNaviSource'] = 1
  if preview_state != 'missing':
    write_preview(feature, reason='position' if preview_state == 'no_gps' else 'ready',
                  provenance={} if preview_state == 'no_gps' else provenance('navdy'))
    if preview_state == 'stale':
      stamp = time.time() - 3.  # noqa: TID251 - filesystem mtime
      os.utime(feature.nav_preview.STATUS_PATH, (stamp, stamp))
    elif preview_state == 'legacy':
      feature.nav_preview.STATUS_PATH.write_text('{"reason":"ready"}')
  if model_state == 'live':
    write_model(feature, nav_provenance=provenance('navdy'))
  before = feature.MODEL_STATUS_PATH.read_bytes() if feature.MODEL_STATUS_PATH.exists() else None
  status = feature.navigation_status('navdy')
  assert not status['model_input_ready'] and not status['map_ready']
  assert not status['nav_map_available'] and not status['model_input_valid_for_ms']
  assert status['pipeline_status'] == ('model_status_missing' if model_state == 'missing' else 'provenance_unverified')
  assert status['model_status_fresh'] == (model_state == 'live')
  assert (feature.MODEL_STATUS_PATH.read_bytes() if feature.MODEL_STATUS_PATH.exists() else None) == before


def test_model_with_no_png_still_requires_matching_current_context(feature):
  write_preview(feature, provenance=provenance('navdy'))
  write_model(feature, nav_provenance=provenance('navdy'))
  feature.nav_preview.PREVIEW_PATH.unlink()
  status = feature.navigation_status('navdy')
  assert status['model_input_ready'] and not status['map_ready']
  assert 0 < status['model_input_valid_for_ms'] <= 2000
  feature.nav_preview.STATUS_PATH.unlink()
  assert not feature.navigation_status('navdy')['model_input_ready']


def test_real_map_input_and_preview_writer_metadata_roundtrip_for_full_navdy_route(feature, setup):
  _, context, _, model, reader = setup
  context['navRoute'].navigationSource = context['navInstructionCarrot'].navigationSource = 'navdy'
  context['navRoute'].coordinates *= 2823
  feature.values['CarrotNaviSource'] = 1
  inputs = reader.update(model)
  feature.nav_preview.write_preview(inputs['nav_map'][0, 0].astype('uint8'), feature.nav_preview.PREVIEW_PATH,
                                    provenance=feature.nav_preview.diagnostic_provenance(context, 10.), rendered_at=9.8)
  write_model(feature, **reader.status)
  status = feature.navigation_status('navdy')
  assert status['map_ready'] and status['model_input_ready']
  assert status['model_route_id'] == '123' and status['model_navigation_source'] == 'navdy'
  assert status['model_input_valid_for_ms'] == 1699 or status['model_input_valid_for_ms'] == 1700
  model.active = False
  reader.update(model)
  write_model(feature, **reader.status, active=False)
  status = feature.navigation_status('navdy')
  assert status['map_ready'] and not status['model_input_ready'] and status['pipeline_status'] == 'model_inactive'


@pytest.mark.parametrize('field,value', [
  ('source', 'network-v2'), ('source', ''), ('source', ['navdy']), ('source', 'access_token=secret'),
  ('route_id', '124'), ('route_id', 123), ('route_id', True), ('route_id', '0'), ('route_id', '0123'),
  ('route_id', '18446744073709551616'), ('route_id', '1' * 100), ('route_id', '\u0661'),
  ('updated', 'stale'), ('updated', 'future'), ('updated', float('nan')), ('updated', 10**400),
  ('route_expires', 0), ('route_expires', float('inf')), ('context_expires', 0), ('context_expires', float('inf')),
])
def test_navdy_model_must_match_current_renderer_context_without_renewing_deadlines(feature, field, value):
  write_preview(feature, provenance=provenance('navdy'))
  model_context = provenance('navdy')
  if value == 'stale':
    value = time.monotonic() - 3.
  elif value == 'future':
    value = time.monotonic() + 1.
  model_context[field] = value
  write_model(feature, nav_provenance=model_context)
  status = feature.navigation_status('navdy')
  assert status['map_ready'] and not status['model_input_ready']
  assert not status['model_provenance_verified'] and not status['nav_map_available']
  assert 'secret' not in json.dumps(status)


@pytest.mark.parametrize('source', ['network', 'navdy'])
@pytest.mark.parametrize('legacy', ['preview', 'model', 'both'])
def test_unproven_legacy_metadata_never_authorizes_current_inputs(feature, source, legacy):
  actual = 'navdy' if source == 'navdy' else 'mapbox'
  write_preview(feature, provenance=provenance(actual))
  write_model(feature, nav_provenance=provenance(actual))
  if legacy in ('preview', 'both'):
    feature.nav_preview.STATUS_PATH.write_text(json.dumps({'reason': 'ready'}))
  if legacy in ('model', 'both'):
    write_model(feature, nav_provenance=None)
  status = feature.navigation_status(source)
  assert not status['model_input_ready'] and not status['nav_route_available']
  assert status['map_ready'] == (legacy == 'model')


@pytest.mark.parametrize('condition', ['source', 'generation', 'expired', 'future', 'render_old', 'hash'])
def test_current_navdy_preview_provenance_is_bounded_and_bound_to_png(feature, condition):
  feature.values['CarrotNaviSource'] = 1
  current = provenance('navdy')
  changes = {'provenance': current}
  if condition == 'source':
    current['source'] = 'mapbox'
  elif condition == 'generation':
    current['route_id'] = ''
  elif condition == 'expired':
    current['context_expires'] = time.monotonic()
  elif condition == 'future':
    current['updated'] += 10.
  elif condition == 'render_old':
    changes['rendered_at'] = time.monotonic() - 2.
  else:
    changes['png_sha256'] = '0' * 64
  write_preview(feature, **changes)
  async def run():
    async with client_for(feature) as client:
      assert not (await (await client.get('/api/navigation/destination')).json())['map_ready']
      assert (await client.get('/api/navigation/map.png')).status == 204
  asyncio.run(run())


@pytest.mark.parametrize('change', ['source', 'generation', 'png', 'legacy', 'cancel'])
def test_preview_rechecks_source_route_and_bytes_after_read(feature, monkeypatch, change):
  feature.values['CarrotNaviSource'] = 1
  write_preview(feature, provenance=provenance('navdy'))
  read = feature.read_bounded
  def replace_after_read(path, limit):
    result = read(path, limit)
    if path == feature.nav_preview.PREVIEW_PATH:
      if change == 'source':
        feature.values['CarrotNaviSource'] = 0
      elif change == 'legacy':
        feature.nav_preview.STATUS_PATH.write_text('{"reason":"ready"}')
      else:
        write_preview(feature, reason='route' if change == 'cancel' else 'ready',
                      provenance=provenance('navdy', '124' if change == 'generation' else '123'),
                      png_sha256='0' * 64 if change == 'png' else hashlib.sha256(PNG).hexdigest())
    return result
  monkeypatch.setattr(feature, 'read_bounded', replace_after_read)
  async def run():
    async with client_for(feature) as client:
      response = await client.get('/api/navigation/map.png')
      assert response.status == 204 and response.headers['X-Nav-Map-Status'] == 'context_changed'
  asyncio.run(run())


def test_status_does_not_read_secret_token_or_forward_private_model_fields(feature, monkeypatch):
  feature.values['MapboxPublicKey'] = 'pk.test-private'
  write_model(feature, error='sk.secret', access_token='sk.secret', model='sk.secret')
  def read(key, default):
    assert key in {'CarrotNaviSource', 'NavDestination', 'MapboxPublicKey'}
    return feature.values.get(key, default)
  monkeypatch.setattr(feature, 'get_param_value', read)
  async def run():
    async with client_for(feature) as client:
      response = await client.get('/api/navigation/destination')
      body = await response.text()
      assert 'pk.test-private' not in body and 'sk.secret' not in body
      assert json.loads(body)['key_configured']
  asyncio.run(run())


def mock_search(*, payload=None, status=200, error=None, block=None):
  async def chunks(_):
    for i in range(0, len(payload), 7):
      yield payload[i:i + 7]
  @asynccontextmanager
  async def get(url, **kwargs):
    assert url == 'https://api.mapbox.com/search/geocode/v6/forward'
    assert kwargs['params']['access_token'] == 'pk.test-private'
    assert kwargs['params']['permanent'] == 'true'
    assert kwargs['timeout'].total == 8
    assert not kwargs['allow_redirects']
    if block:
      await block.wait()
    if error:
      raise error
    yield NS(status=status, content=NS(iter_chunked=chunks))
  return NS(get=get)


def test_search_fragmented_response_and_rate_limit_without_token_disclosure(feature):
  feature.values['MapboxPublicKey'] = 'pk.test-private'
  payload = json.dumps({'features': [{'geometry': {'coordinates': [127., 37.]},
                                     'properties': {'full_address': 'Test destination'}}]}).encode()
  async def run():
    async with client_for(feature, mock_search(payload=payload)) as client:
      response = await client.get('/api/navigation/search?q=Seoul')
      body = await response.text()
      assert response.status == 200 and 'pk.test-private' not in body
      assert json.loads(body)['results'][0]['latitude'] == 37.
      assert (await client.get('/api/navigation/search?q=Seoul')).status == 429
  asyncio.run(run())


@pytest.mark.parametrize('kwargs', [
  {'payload': b'{bad json'}, {'payload': b'{"features": {}}'},
  {'payload': b'{"features":[{"geometry":{"coordinates":[true,37]}}]}'},
  {'payload': b'x' * (1024 * 1024 + 1)}, {'status': 401}, {'status': 302},
  {'error': ClientError('URL contains pk.test-private')}, {'error': TimeoutError()},
])
def test_bad_search_is_bounded_and_redacted(feature, kwargs):
  feature.values['MapboxPublicKey'] = 'pk.test-private'
  async def run():
    async with client_for(feature, mock_search(**kwargs)) as client:
      response = await client.get('/api/navigation/search?q=Seoul')
      assert response.status == 502
      assert 'pk.test-private' not in await response.text()
  asyncio.run(run())


def test_search_concurrency_limit(feature):
  feature.values['MapboxPublicKey'] = 'pk.test-private'
  async def run():
    event = asyncio.Event()
    async with client_for(feature, mock_search(payload=b'{"features":[]}', block=event)) as client:
      first = asyncio.create_task(client.get('/api/navigation/search?q=Seoul'))
      for _ in range(100):
        if client.app[feature.SEARCH_STATE].lock.locked():
          break
        await asyncio.sleep(.005)
      assert client.app[feature.SEARCH_STATE].lock.locked()
      assert (await client.get('/api/navigation/search?q=Busan')).status == 429
      event.set()
      assert (await first).status == 200
  asyncio.run(run())
