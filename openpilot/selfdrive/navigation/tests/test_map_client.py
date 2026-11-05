import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink.contracts import DEFAULT_SPEC, N16_SPEC
from openpilot.selfdrive.navigation import map_client
from openpilot.selfdrive.navigation.helpers import MAX_MANEUVERS
from openpilot.selfdrive.navigation.map_client import MAP_SHAPE, NAVIGATION_SHAPE, MapInput


class Context(dict):
  def __init__(self):
    super().__init__(
      navRoute=NS(carrotRouteId=123, navigationSource='mapbox', coordinates=[NS(latitude=37., longitude=127.), NS(latitude=37.01, longitude=127.01)]),
      navInstructionCarrot=NS(carrotRouteId=123, navigationSource='mapbox', instructionMonoTime=9700000000,
                             distanceMonoTime=9700000000, allManeuvers=[NS(type='turn', modifier='left', distance=120.)]))
    self.valid = {'navRoute': True, 'navInstructionCarrot': True}
    self.recv_time = {'navRoute': 9.6, 'navInstructionCarrot': 9.7}
    self.updates = []

  def update(self, timeout):
    self.updates.append(timeout)


class Client:
  def __init__(self):
    self.connected = True
    self.buffer = NS(width=256, height=256, data=bytearray(np.arange(65536, dtype=np.uint8).tobytes()))
    self.timestamp_sof = 123
    self.timestamp_eof = 9800000000
    self.connects, self.receives = [], []

  def is_connected(self):
    return self.connected

  def connect(self, blocking):
    self.connects.append(blocking)
    return self.connected

  def recv(self, timeout):
    self.receives.append(timeout)
    result, self.buffer = self.buffer, None
    return result


@pytest.fixture
def setup(monkeypatch):
  clock, context, client = [10.], Context(), Client()
  monkeypatch.setattr(map_client.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(map_client, '_make_context', lambda: context)
  monkeypatch.setattr(map_client, '_make_client', lambda: client)
  model = NS(active=True, usbgpu=False, spec=N16_SPEC, views={'nav_map': NS(shape=MAP_SHAPE), 'navigation': NS(shape=NAVIGATION_SHAPE)})
  return clock, context, client, model, MapInput()


def assert_zero(result):
  assert result['nav_map'].shape == MAP_SHAPE and result['nav_map'].dtype == np.float32
  assert result['navigation'].shape == NAVIGATION_SHAPE and result['navigation'].dtype == np.float32
  assert not result['nav_map'].any() and not result['navigation'].any()


def test_upstream_stream_context_timestamps_and_pixel_scale(setup):
  _, context, client, model, reader = setup
  result = reader.update(model, 10.)
  np.testing.assert_array_equal(result['nav_map'].ravel(), np.arange(65536, dtype=np.uint8))
  assert result['navigation'][0, 15] == 1
  assert result['navigation'][0, -1] == pytest.approx(1.17)
  assert context.updates == [0] and client.receives == [0]
  assert reader.status['nav_map_available'] and not reader.status['nav_input_not_yet_available']


@pytest.mark.parametrize('source,count,ready', [('navdy', 5627, True), ('navdy', 5646, True), ('navdy', 8192, True),
                                               ('navdy', 8193, False), ('mapbox', 4097, False),
                                               ('network-v2', 4097, False)])
def test_full_navdy_route_has_separate_bounded_capacity(setup, source, count, ready):
  _, context, _, model, reader = setup
  context['navRoute'].navigationSource = source
  context['navInstructionCarrot'].navigationSource = source
  context['navRoute'].coordinates = [NS(latitude=37. + i * .00001, longitude=127.) for i in range(count)]
  result = reader.update(model)
  assert bool(result['nav_map'].any()) == ready
  assert bool(result['navigation'].any()) == ready
  assert bool(reader.status['nav_provenance']) == ready
  if ready:
    assert reader.status['nav_provenance']['source'] == source
    assert reader.status['nav_provenance']['route_id'] == '123'


def test_diagnostic_provenance_tracks_route_replacement_expiry_and_model_exit(setup):
  clock, context, client, model, reader = setup
  reader.update(model)
  assert reader.status['nav_provenance'] == {'source': 'mapbox', 'route_id': '123', 'updated': 10.,
                                          'route_expires': 12.6, 'context_expires': 11.7}
  context['navRoute'].navigationSource = context['navInstructionCarrot'].navigationSource = 'navdy'
  context['navRoute'].carrotRouteId = context['navInstructionCarrot'].carrotRouteId = 2**64 - 1
  result = reader.update(model)
  assert not result['nav_map'].any() and result['navigation'].any()
  assert reader.status['nav_provenance']['source'] == 'navdy'
  assert reader.status['nav_provenance']['route_id'] == str(2**64 - 1)
  assert reader.status['nav_input_reason'] == 'map_not_yet_available'
  clock[0] = 11.71
  assert_zero(reader.update(model))
  assert reader.status['nav_provenance']['context_expires'] == 0.
  model.active = False
  assert reader.update(model) == {}
  assert reader.status['nav_provenance'] == {}
  assert client.receives == [0, 0]


@pytest.mark.parametrize('services', [('navRoute',), ('navInstructionCarrot',), ('navRoute', 'navInstructionCarrot')])
def test_clock_resampled_after_submaster_stamps_new_messages(setup, services):
  clock, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  update = context.update

  def receive_context(timeout):
    update(timeout)
    clock[0] += .001
    for service in services:
      context.recv_time[service] = clock[0]
    clock[0] += .002

  context.update = receive_context
  for _ in range(3):
    result = reader.update(model, 10.)
    assert result['nav_map'].any()
    assert result['navigation'][0, 15] == 1.
    assert result['navigation'][0, -1] == pytest.approx(
      (120. - 10. * (clock[0] - context['navInstructionCarrot'].distanceMonoTime / 1e9)) / 100.)
    assert reader.status['nav_input_reason'] == 'ready'


def test_context_update_latency_expires_cached_context(setup):
  clock, context, client, model, reader = setup
  assert reader.update(model)['nav_map'].any()

  def delayed_update(timeout):
    clock[0] += 3.

  context.update = delayed_update
  assert_zero(reader.update(model))
  assert reader.image is None
  assert reader.status['nav_input_reason'] == 'route_unavailable'
  assert client.receives == [0]


@pytest.mark.parametrize('index', [0, 32, 2047])
def test_long_upstream_route_keeps_usable_tbt(setup, index):
  _, context, _, model, reader = setup
  context['navInstructionCarrot'].allManeuvers = [
    NS(type='turn', modifier='right', distance=-1.) for _ in range(2048)]
  context['navInstructionCarrot'].allManeuvers[index] = NS(type='turn', modifier='left', distance=120.)
  result = reader.update(model, 10.)
  assert result['nav_map'].any()
  assert result['navigation'][0, 15] == 1.
  assert result['navigation'][0, -1] == pytest.approx(1.17)


def test_no_producer_retains_only_fresh_route_gated_tbt_and_nonblocking_retry(setup):
  clock, context, client, model, reader = setup
  client.connected = False
  for now in (10., 10.05, 10.5, 11.):
    clock[0] = now
    result = reader.update(model)
    assert not result['nav_map'].any() and result['navigation'].any()
  assert client.connects == [False, False]
  assert not client.receives
  assert reader.status['nav_input_reason'] == 'map_not_yet_available'


def test_current_schema_without_route_id_fails_closed_without_map_connection(setup):
  _, context, client, model, reader = setup
  del context['navRoute'].carrotRouteId
  assert_zero(reader.update(model))
  assert reader.status['nav_input_not_yet_available']
  assert reader.status['nav_input_reason'] == 'route_unavailable'
  assert not client.receives and not client.connects


@pytest.mark.parametrize('change', ['invalid', 'missing_id', 'empty', 'one_point', 'too_many', 'stale', 'future', 'nan'])
def test_invalid_route_zeros_both_and_clears_prior_image(setup, change):
  clock, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  if change == 'invalid':
    context.valid['navRoute'] = False
  elif change == 'missing_id':
    context['navRoute'].carrotRouteId = 0
  elif change in ('empty', 'one_point', 'too_many'):
    context['navRoute'].coordinates = [NS()] * {'empty': 0, 'one_point': 1, 'too_many': 4097}[change]
  else:
    context.recv_time['navRoute'] = {'stale': 7., 'future': 11., 'nan': float('nan')}[change]
  assert_zero(reader.update(model))
  assert reader.image is None


@pytest.mark.parametrize('change', ['invalid', 'stale', 'future', 'nan', 'empty', 'unknown', 'unknown_modifier',
                                    'unknown_sided_modifier', 'passed', 'malformed', 'too_many'])
def test_invalid_tbt_zeros_both(setup, change):
  _, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  if change == 'invalid':
    context.valid['navInstructionCarrot'] = False
  elif change in ('stale', 'future', 'nan'):
    context.recv_time['navInstructionCarrot'] = {'stale': 8., 'future': 11., 'nan': float('nan')}[change]
  elif change == 'empty':
    context['navInstructionCarrot'].allManeuvers = []
  elif change == 'unknown':
    context['navInstructionCarrot'].allManeuvers[0].type = 'invalid'
  elif change in ('unknown_modifier', 'unknown_sided_modifier'):
    context['navInstructionCarrot'].allManeuvers = [
      NS(type='turn' if change == 'unknown_modifier' else 'fork', modifier='not-right', distance=50.),
      NS(type='turn', modifier='left', distance=120.)]
  elif change == 'passed':
    context['navInstructionCarrot'].allManeuvers[0].distance = -1
  elif change == 'malformed':
    context['navInstructionCarrot'].allManeuvers = [NS()]
  else:
    context['navInstructionCarrot'].allManeuvers *= MAX_MANEUVERS + 1
  assert_zero(reader.update(model))
  assert reader.image is None


@pytest.mark.parametrize('change', ['wrong_route', 'zero_route', 'stale', 'future', 'wrong_size', 'short_buffer'])
def test_bad_map_keeps_only_tbt(setup, change):
  _, _, client, model, reader = setup
  if change == 'wrong_route':
    client.timestamp_sof = 124
  elif change == 'zero_route':
    client.timestamp_sof = 0
  elif change == 'stale':
    client.timestamp_eof = 9500000000
  elif change == 'future':
    client.timestamp_eof = 11000000000
  elif change == 'wrong_size':
    client.buffer.width = 128
  else:
    client.buffer.data = bytes(65535)
  result = reader.update(model)
  assert not result['nav_map'].any() and result['navigation'].any()
  assert reader.status['nav_tbt_available'] and not reader.status['nav_map_available']


def test_cached_image_expires_from_render_start_not_receive_time(setup):
  clock, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  clock[0] = 10.31
  result = reader.update(model)
  assert not result['nav_map'].any() and result['navigation'].any()
  clock[0] = 11.71
  assert_zero(reader.update(model))


def test_copy_visionipc_ring_data_before_buffer_reuse(setup):
  _, _, client, model, reader = setup
  original = client.buffer.data
  reader.update(model)
  original[:] = bytes(65536)
  assert reader.update(model)['nav_map'].max() == 255


def test_route_replacement_cannot_reuse_old_map(setup):
  _, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  context['navRoute'].carrotRouteId += 1
  context.recv_time['navRoute'] = 10.
  assert_zero(reader.update(model))
  context.recv_time['navInstructionCarrot'] = 10.
  context['navInstructionCarrot'].carrotRouteId = context['navRoute'].carrotRouteId
  result = reader.update(model)
  assert not result['nav_map'].any()
  assert result['navigation'].any()


def test_same_route_heartbeat_does_not_change_generation_gate(setup):
  _, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  context.recv_time['navRoute'] = 10.
  assert reader.update(model)['nav_map'].any()


def test_disconnect_clears_cached_map_immediately(setup):
  _, _, client, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  client.connected = False
  result = reader.update(model)
  assert not result['nav_map'].any() and result['navigation'].any()
  assert reader.image is None


@pytest.mark.parametrize('component', ['context_create', 'context_update', 'client_create', 'connect', 'recv'])
def test_optional_ipc_failures_do_not_escape_and_retry_is_bounded(setup, monkeypatch, component):
  clock, context, client, model, reader = setup
  calls = []
  def fail(*args):
    calls.append(clock[0])
    raise RuntimeError('optional IPC unavailable')
  if component == 'context_create':
    monkeypatch.setattr(map_client, '_make_context', fail)
  elif component == 'context_update':
    context.update = fail
  elif component == 'client_create':
    monkeypatch.setattr(map_client, '_make_client', fail)
  else:
    setattr(client, component, fail)
    client.connected = component != 'connect'
  for now in (10., 10.05, 10.5):
    clock[0] = now
    result = reader.update(model)
    assert not result['nav_map'].any()
    if component.startswith('context'):
      assert_zero(result)
  assert calls == [10.]
  clock[0] = 11.
  reader.update(model)
  assert calls == [10., 11.]


@pytest.mark.parametrize('delay', [.4, 2.])
def test_native_receive_latency_is_included_in_freshness(setup, delay):
  clock, _, client, model, reader = setup
  receive = client.recv
  def slow_receive(timeout):
    clock[0] += delay
    return receive(timeout)
  client.recv = slow_receive
  result = reader.update(model)
  assert not result['nav_map'].any()
  if delay == 2.:
    assert_zero(result)
  else:
    assert result['navigation'].any()


@pytest.mark.parametrize('kind', ['internal', 'cinque', 'egpu', 'inactive_n16', 'unknown_nav', 'modified_n16', 'wrong_shape', 'partial'])
def test_every_non_n16_path_has_no_ipc_or_model_input_changes(setup, monkeypatch, kind):
  _, _, _, model, reader = setup
  if kind == 'internal':
    model.views = {}
  elif kind == 'cinque':
    model.spec = DEFAULT_SPEC
  elif kind == 'egpu':
    model.usbgpu = True
  elif kind == 'inactive_n16':
    model.active = False
  elif kind == 'unknown_nav':
    model.spec = replace(N16_SPEC, sha256='0' * 64)
  elif kind == 'modified_n16':
    model.spec = replace(N16_SPEC, nbytes=N16_SPEC.nbytes + 1)
  elif kind == 'wrong_shape':
    model.views['navigation'].shape = (47,)
  else:
    del model.views['nav_map']
  def forbidden():
    raise AssertionError('Non-N16 model initialized navigation IPC')
  monkeypatch.setattr(map_client, '_make_context', forbidden)
  monkeypatch.setattr(map_client, '_make_client', forbidden)
  assert reader.update(model) == {}
  assert reader.sm is reader.client is reader.map is reader.navigation is None
  assert not reader.status['nav_input_supported']


def test_n16_exit_and_reentry_cannot_reuse_old_map(setup):
  _, _, client, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  assert reader.update(NS(views={})) == {}
  assert reader.image is None
  result = reader.update(model)
  assert not result['nav_map'].any() and result['navigation'].any()
  assert client.receives == [0, 0]


def test_n16_native_fallback_releases_navigation_context(setup):
  _, _, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  model.active = False
  assert reader.update(model) == {}
  assert reader.sm is reader.client is reader.image is reader.map is reader.navigation is None


@pytest.mark.parametrize('speed', [float('nan'), float('inf'), -1., True, 10 ** 400])
def test_invalid_speed_zeros_navigation(setup, speed):
  _, _, _, model, reader = setup
  assert_zero(reader.update(model, speed))


def test_real_n16_packer_gets_upstream_inputs_then_zeros(setup):
  from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
  clock, _, _, model, reader = setup
  packed = np.zeros(N16_SPEC.packed_nelem, np.float32)
  views = {name: array.reshape(shape) for (name, shape), array in zip(
    N16_SPEC.packed_shapes.items(), np.split(packed, np.cumsum(N16_SPEC.packed_sizes[:-1])), strict=True)}
  model.views = views
  previous = np.zeros(8, np.float32)
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1., 0.], np.float32),
            'action_t': np.array([.2, .3], np.float32)}
  fill_inputs(views, {**inputs, **reader.update(model, 10.)}, previous)
  np.testing.assert_array_equal(views['nav_map'].ravel(), np.arange(65536, dtype=np.uint8))
  assert views['navigation'][0, -1] == pytest.approx(1.17)
  clock[0] = 12.
  fill_inputs(views, {**inputs, **reader.update(model, 10.)}, previous)
  assert not packed[12:].any()
  np.testing.assert_array_equal(views['traffic_convention'], [[1., 0.]])


def test_actual_map_snapshot_enters_model_then_expires(setup, monkeypatch, tmp_path):
  from openpilot.selfdrive.navigation.map_frame import SnapshotClient, write_frame
  clock, _, _, model, reader = setup
  path = tmp_path / 'model.frame'
  monkeypatch.setattr(map_client, '_make_client', lambda: SnapshotClient(path))
  pixels = np.arange(65536, dtype=np.uint8).reshape(256, 256)
  write_frame(pixels, 123, 9_800_000_000, path)
  result = reader.update(model, 10.)
  np.testing.assert_array_equal(result['nav_map'][0, 0], pixels)
  assert result['navigation'][0, 15] == 1
  assert reader.status['nav_input_reason'] == 'ready'
  clock[0] = 10.31
  assert not reader.update(model)['nav_map'].any()
  assert not reader.status['nav_map_available']


def test_modeld_navigation_is_connected_and_preserves_fault_validity():
  path = Path(__file__).resolve().parents[2] / 'modeld/modeld.py'
  tree = ast.parse(path.read_text())
  calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
  assert any(isinstance(n, ast.ImportFrom) and n.module == 'openpilot.selfdrive.navigation.map_client' for n in ast.walk(tree))
  validity = next(node for node in ast.walk(tree) if isinstance(node, ast.Assign)
                  and any(isinstance(target, ast.Name) and target.id == 'model_output_valid' for target in node.targets))
  assert ast.unparse(validity.value) == "live_calib_seen and getattr(model, 'output_valid', True)"
  for name in ('fill_model_msg', 'fill_pose_msg'):
    call = next(node for node in calls if ast.unparse(node.func) == name)
    assert ast.unparse(call.args[-1]) == 'model_output_valid'
