import hashlib
import json

import pytest

from openpilot.selfdrive.navdy.model_navigation import ModelNavigationTracker, ModelRouteCache, age_model_feedback, decode_route
from openpilot.selfdrive.navdy.navigation import NavigationReader


def encode(points):
  encoded = ''
  previous = [0, 0]
  for point in points:
    for axis, coordinate in enumerate(point):
      value = round(coordinate * 1e6)
      delta = value - previous[axis]
      previous[axis] = value
      delta = ~(delta << 1) if delta < 0 else delta << 1
      while delta >= 32:
        encoded += chr((32 | delta & 31) + 63)
        delta >>= 5
      encoded += chr(delta + 63)
  return encoded


POINTS = [(37., 127.), (37.0001, 127.0001), (37.0002, 127.0001)]
TOKEN = 'a' * 32


def write_feedback(path, value, received, token=TOKEN):
  temporary = path.with_suffix('.tmp')
  temporary.write_text(json.dumps({'receivedAt': received, 'feedback': {'modelNavigation': value},
                                  'modelRouteCacheToken': token}))
  temporary.replace(path)


def write_cache(path, state=None, token=TOKEN):
  temporary = path.with_suffix('.tmp')
  temporary.write_text(json.dumps(dict((state or tracker()).route_cache(), bridgeCacheToken=token)))
  temporary.replace(path)


def packet(points=POINTS, *, sequence=1, route='route-a', session='session-a', index=0):
  encoded = encode(points)
  return {'version': 1, 'source': 'tmap', 'session': session, 'sequence': sequence,
          'active': True, 'offRoute': False, 'routeId': route, 'ageMs': 100, 'validForMs': 1900,
          'maneuver': {'type': 'turn', 'modifier': 'left', 'distanceMeters': 100.},
          'position': {'latitude': 37., 'longitude': 127., 'bearingDegrees': 0., 'speedMps': 0.,
                       'observedAtMs': 100000, 'ageMs': 200},
          'routeChunk': {'encoding': 'polyline6', 'index': index, 'count': (len(encoded) + 1023) // 1024,
                         'encodedLength': len(encoded), 'pointCount': len(points),
                         'sha256': hashlib.sha256(encoded.encode()).hexdigest(), 'data': encoded[index * 1024:(index + 1) * 1024]}}


def tracker(value=None, *, now=10.):
  result = ModelNavigationTracker()
  result.accept(value or packet(), now, now)
  return result


def test_complete_geometry_and_original_observations():
  state = tracker()
  model = state.snapshot(10.)
  assert model.points == tuple(POINTS)
  assert model.observed == 9.9
  assert model.position[-1] == 9.8
  assert model.maneuver_type == 'turn' and model.modifier == 'left'
  assert state.request(10.) == {'routeId': 'route-a', 'index': -1}


def test_duplicate_heartbeat_never_renews_instruction_or_position():
  state = tracker()
  first = state.snapshot(10.)
  state.accept(packet(), 11., 11.)
  again = state.snapshot(11.)
  assert again.observed == first.observed
  assert again.position[-1] == first.position[-1]
  assert state.snapshot(11.91) is None
  state.accept(packet(), 12., 12.)
  assert state.snapshot(12.) is None


def test_new_guidance_does_not_renew_old_position():
  state = tracker()
  assert state.snapshot(10.).position[-1] == 9.8
  state.accept(packet(sequence=2), 11.5, 11.5)
  assert state.snapshot(11.5).position[-1] == 9.8
  state.accept(packet(sequence=3), 12., 12.)
  assert state.snapshot(12.).position is None


@pytest.mark.parametrize('change', [None, {'active': False}, {'offRoute': True}, {'ageMs': 2001},
                                  {'validForMs': 0}, {'sequence': True}, {'source': 'unknown'}])
def test_cancel_or_invalid_input_clears_and_old_sequence_cannot_restore(change):
  state = tracker()
  value = {**packet(sequence=2), **change} if change is not None else None
  state.accept(value, 10.1, 10.1)
  assert state.snapshot(10.1) is None
  state.accept(packet(), 10.2, 10.2)
  assert state.snapshot(10.2) is None


def test_out_of_order_does_not_replace_current_generation():
  state = tracker(packet(sequence=4, route='new'))
  state.accept(packet(sequence=3), 10.1, 10.1)
  assert state.snapshot(10.1).identity[-1] == 'new'


def test_same_geometry_new_generation_is_distinct():
  state = tracker()
  before = state.snapshot(10.)
  state.accept(packet(sequence=2, route='route-b'), 10.1, 10.1)
  after = state.snapshot(10.1)
  assert before.points == after.points and before.session != after.session


def test_fragmented_route_reordering_and_missing_chunks():
  points = [(37. + i * .0001, 127. + i * .00001) for i in range(1000)]
  first = packet(points, index=2)
  state = tracker(first)
  assert state.snapshot(10.) is None
  assert state.request(10.)['index'] == 0
  for index in range(first['routeChunk']['count']):
    if index == 1:
      continue
    state.accept(packet(points, index=index), 10.1 + index / 100., 10.1 + index / 100.)
  assert state.request(10.3)['index'] == 1
  assert state.snapshot(10.3) is None
  state.accept(packet(points, index=1), 10.4, 10.4)
  assert len(state.snapshot(10.4).points) == 1000


@pytest.mark.parametrize('field,value', [('data', 'bad'), ('sha256', '0' * 64), ('count', 65),
                                      ('pointCount', 8193), ('encodedLength', 65537), ('index', -1)])
def test_bad_manifest_or_hash_never_publishes(field, value):
  item = packet()
  item['routeChunk'][field] = value
  state = tracker(item)
  assert not state.points and state.snapshot(10.) is None


@pytest.mark.parametrize('encoded,count', [('????', 2), ('~~~~~~~?', 2), ('_p~iF', 2), ('a', 2), ('\n????', 2)])
def test_malformed_polyline(encoded, count):
  digest = hashlib.sha256(encoded.encode()).hexdigest()
  with pytest.raises(ValueError):
    decode_route(encoded, count, digest)


def test_route_points_cap_and_geo_bounds():
  for points in ([(0., 0.), (90., 1.)], [(0., 0.), (1., 181.)], POINTS * 3000):
    data = encode(points)
    with pytest.raises(ValueError):
      decode_route(data, len(points), hashlib.sha256(data.encode()).hexdigest())


@pytest.mark.parametrize('count', [5627, 8192])
def test_long_whole_route_preserved_without_decimation(count):
  points = [(round(37. + i * .00001, 6), 127.) for i in range(count)]
  first = packet(points)
  state = tracker(first)
  assert state.snapshot(10.) is None
  for index in range(1, first['routeChunk']['count']):
    now = 10. + index / 100.
    state.accept(packet(points, index=index), now, now)
  assert state.snapshot(now).points == tuple(points)
  assert state.route_cache()['polyline6'] == encode(points)
  assert state.request(now)['index'] == -1


def test_round_trip_age_charged_to_independent_position_and_deadline():
  value = {'modelNavigation': packet()}
  age_model_feedback(value, 250.5)
  assert value['modelNavigation']['ageMs'] == 350.5
  assert value['modelNavigation']['position']['ageMs'] == 450.5
  assert value['modelNavigation']['validForMs'] == 1649
  age_model_feedback(value, 2000)
  state = tracker(value['modelNavigation'])
  assert state.snapshot(10.) is None


def test_sidecar_cache_requires_fresh_matching_feedback(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  complete = tracker()
  write_cache(reader.model_route_path, complete)
  value = packet()
  del value['routeChunk']
  write_feedback(path, value, 10.)
  assert reader.model_read(10.).points == tuple(POINTS)
  assert reader.model_read(12.) is None
  value.update(sequence=2, routeId='route-b')
  write_feedback(path, value, 12.1)
  assert reader.model_read(12.1) is None


def test_import_does_not_accept_other_source_or_session():
  complete = tracker()
  cache = complete.route_cache()
  for field in ('source', 'session', 'routeId'):
    incoming = packet()
    del incoming['routeChunk']
    state = tracker(incoming)
    state.import_route({**cache, field: 'different'})
    assert state.snapshot(10.) is None


def test_unknown_maneuver_never_invents_straight():
  item = packet()
  item['maneuver']['modifier'] = 'unknown'
  assert tracker(item).snapshot(10.) is None


def test_empty_legacy_feedback_cannot_enable_model():
  state = tracker({'navigationFeedback': {'guidance': {'primaryType': 12}}})
  assert state.snapshot(10.) is None


def test_retired_process_cannot_reclaim_route_with_higher_sequence():
  state = tracker(packet(sequence=10, session='old'))
  state.accept(packet(sequence=1, session='new'), 10.1, 10.1)
  state.accept(packet(sequence=20, session='old'), 10.2, 10.2)
  assert state.snapshot(10.2).identity[1] == 'new'


def test_cache_cannot_override_different_manifest_for_same_id():
  cache = tracker().route_cache()
  points = [(38. + i * .0001, 126.) for i in range(1000)]
  state = tracker(packet(points))
  state.import_route(cache)
  assert not state.points


def test_position_rollback_does_not_lower_high_water():
  state = tracker()
  assert state.snapshot(10.).position
  value = packet(sequence=2)
  value['position']['observedAtMs'] -= 1
  state.accept(value, 10.1, 10.1)
  assert state.snapshot(10.1).position is None
  state.accept(packet(sequence=3), 11.9, 11.9)
  assert state.snapshot(11.9).position is None


def test_chunk_transfer_does_not_renew_position_before_first_snapshot():
  points = [(37. + i * .0001, 127.) for i in range(1000)]
  first = packet(points)
  first['position']['ageMs'] = 1900
  state = tracker(first)
  assert state.snapshot(10.) is None
  for index in range(first['routeChunk']['count']):
    value = packet(points, index=index)
    state.accept(value, 10.5, 10.5)
  assert state.snapshot(10.5).position is None


def test_replayed_observation_cannot_reselect_previous_provider():
  state = tracker(packet(sequence=10))
  value = {**packet(), 'source': 'naver'}
  state.accept(value, 10.2, 10.2)
  state.accept(packet(sequence=10), 11., 11.)
  assert state.snapshot(11.).identity[0] == 'naver'
  assert state.snapshot(11.).observed == pytest.approx(10.1)


@pytest.mark.parametrize('duration', [2001, 60001, -1, float('nan'), float('inf'), True])
def test_invalid_exchange_duration_invalidates_model(duration):
  value = {'modelNavigation': packet()}
  age_model_feedback(value, duration)
  assert value['modelNavigation'] is None


def test_disconnect_and_reconnect_require_new_verified_geometry(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  complete = tracker()
  write_cache(reader.model_route_path, complete)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  path.unlink()
  reader.model_route_path.unlink()
  assert reader.model_read(10.2) is None
  value = packet(sequence=2)
  del value['routeChunk']
  write_feedback(path, value, 10.4)
  assert reader.model_read(10.4) is None


def test_stale_sidecar_cannot_restore_geometry_from_another_bridge_assembly(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  assert reader.model_read(12.1) is None
  value = packet(sequence=2)
  del value['routeChunk']
  write_feedback(path, value, 12.3, token='b' * 32)
  assert reader.model_read(12.3) is None


def test_new_complete_route_before_next_poll_is_not_blacklisted(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  points = [(37. + i * .0001, 127.) for i in range(1000)]
  complete = ModelNavigationTracker()
  count = packet(points)['routeChunk']['count']
  for index in range(count):
    value = packet(points, sequence=2, route='route-b', index=index)
    complete.accept(value, 12.4, 12.4)
  replacement = tmp_path / 'replacement.json'
  write_cache(replacement, complete, token='b' * 32)
  replacement.replace(reader.model_route_path)
  write_feedback(path, value, 12.4, token='b' * 32)
  state = reader.model_read(12.4)
  assert state is not None and state.identity[-1] == 'route-b' and len(state.points) == 1000


def test_waiting_fresh_frame_prevents_reader_only_expiry(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  value = packet(sequence=2)
  del value['routeChunk']
  write_feedback(path, value, 11.85)
  assert reader.model_read(11.95) is not None, vars(reader.model_tracker)


def test_pending_frame_is_read_at_expiry_inside_poll_interval(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.).points == tuple(POINTS)
  assert reader.model_read(11.85).points == tuple(POINTS)
  write_feedback(path, packet(sequence=2), 11.87)
  assert reader.model_read(11.92).points == tuple(POINTS)
  assert reader.model_tracker.sequence == 2


def test_expiry_poll_does_not_renew_unchanged_frame(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.).points == tuple(POINTS)
  assert reader.model_read(11.85).points == tuple(POINTS)
  assert reader.model_read(11.92) is None


def test_expiry_poll_survives_an_intervening_freshness_clear(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  assert reader.model_read(11.85) is not None
  write_feedback(path, packet(sequence=2), 11.87)
  assert not reader.model_tracker._fresh(11.91)
  assert reader.model_read(11.92).points == tuple(POINTS)


def test_reader_only_silence_recovers_same_bridge_cache_without_replay(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.) is not None
  assert reader.model_read(12.1) is None
  old = packet()
  del old['routeChunk']
  write_feedback(path, old, 12.2)
  assert reader.model_read(12.2) is None
  fresh = packet(sequence=2)
  del fresh['routeChunk']
  write_feedback(path, fresh, 12.4)
  assert reader.model_read(12.4) is not None


def test_sidecar_replacement_cannot_pair_old_feedback_with_new_geometry(tmp_path):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.)
  assert reader.model_read(10.).points == tuple(POINTS)
  points = [(38., 128.), (38.001, 128.001)]
  replacement = tracker(packet(points, sequence=2), now=10.2)
  write_cache(reader.model_route_path, replacement, token='b' * 32)
  assert reader.model_read(10.2) is None
  value = packet(points, sequence=2)
  del value['routeChunk']
  write_feedback(path, value, 10.4, token='b' * 32)
  assert reader.model_read(10.4).points == tuple(points)


@pytest.mark.parametrize('token', [None, '', 'a', 'x' * 32, True])
def test_model_reader_requires_bridge_assembly_attestation(tmp_path, token):
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10., token=token)
  assert reader.model_read(10.) is None
  assert reader.model_tracker.reason == 'model_route_unverified'


def test_bridge_cache_rotates_on_reassembly_not_heartbeat(tmp_path):
  path = tmp_path / 'route.json'
  writes = []
  def writer(target, value):
    writes.append(value)
    target.write_text(json.dumps(value))
  cache = ModelRouteCache(path, writer)
  state = tracker()
  first = cache.update(state)
  assert json.loads(path.read_text())['bridgeCacheToken'] == first
  assert cache.update(state) == first and len(writes) == 1
  state.clear()
  assert cache.update(state) is None and not path.exists()
  state.accept(packet(sequence=2), 10.2, 10.2)
  second = cache.update(state)
  assert second != first and len(writes) == 2
  # A same-route clear and complete reassembly can occur within one bridge tick.
  state.clear()
  state.accept(packet(sequence=3), 10.4, 10.4)
  third = cache.update(state)
  assert third not in (first, second) and len(writes) == 3


def test_reader_samples_clock_after_lock_and_file_read(tmp_path, monkeypatch):
  from openpilot.selfdrive.navdy import navigation
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  write_cache(reader.model_route_path)
  write_feedback(path, packet(), 10.15)
  clock = [10.]
  monkeypatch.setattr(navigation.time, 'monotonic', lambda: clock[0])
  class DelayedLock:
    def __enter__(self):
      clock[0] = max(clock[0], 10.1)
    def __exit__(self, *args):
      pass
  reader.lock = DelayedLock()
  loads = navigation.json.loads
  def delayed_loads(data):
    clock[0] = 10.2
    return loads(data)
  monkeypatch.setattr(navigation.json, 'loads', delayed_loads)
  assert reader.model_read() is not None
  assert reader.status()['modelNavigationReason'] == ''


def test_freshness_checks_preserve_specific_rejection_reason():
  state = tracker()
  state.clear('model_route_invalid')
  assert state.snapshot(10.1) is None
  assert state.request(10.1) == {'routeId': '', 'index': 0}
  assert state.reason == 'model_route_invalid'
