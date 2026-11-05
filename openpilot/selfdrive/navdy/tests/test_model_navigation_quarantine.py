import json

import pytest

from openpilot.selfdrive.navdy.model_navigation import ModelNavigationTracker, ModelRouteCache, age_model_feedback
from openpilot.selfdrive.navdy.navigation import NavigationReader
from openpilot.selfdrive.navdy.tests.test_model_navigation import packet, write_feedback


POINTS = [(round(37. + i * .0001, 6), round(127. + i * .00001, 6)) for i in range(1000)]


def complete():
  state = ModelNavigationTracker()
  count = packet(POINTS)['routeChunk']['count']
  assert count > 1
  for index in range(count):
    now = 10. + index / 100.
    state.accept(packet(POINTS, index=index), now, now)
  assert state.snapshot(now).points == tuple(POINTS)
  return state


def expire(state):
  assert state.snapshot(11.91) is None
  assert state.route_cache() is None
  assert not state.points and not state.chunks and state.current is None


def aged_tail(delay, *, sequence=1, age=2010):
  value = packet(POINTS, sequence=sequence)
  value['ageMs'] = age - delay
  value['validForMs'] = 2000 - value['ageMs']
  return age_model_feedback({'modelNavigation': value}, delay)['modelNavigation']


@pytest.mark.parametrize('delay', [20, 200, 600])
@pytest.mark.parametrize('chunk', [True, False])
@pytest.mark.parametrize('age', [2000, 2010])
@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
def test_expired_tail_round_trip_age_preserves_quarantine_without_restoring(delay, chunk, age, arrival):
  state = complete()
  original = state.points
  tail = aged_tail(delay, age=age)
  assert tail['ageMs'] == age and tail['validForMs'] == 0
  if not chunk:
    del tail['routeChunk']
  deadline = state.deadline
  high_water = dict(state.high_water)
  state.accept(tail, arrival, arrival)
  assert state.snapshot(arrival) is None and state.route_cache() is None
  assert state.deadline == deadline and state.high_water == high_water
  state.accept(packet(POINTS, sequence=2, index=1), 12., 12.)
  assert state.snapshot(12.).points is original


@pytest.mark.parametrize('delay', [20, 200, 600])
@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
@pytest.mark.parametrize('change', [
  {'active': False}, {'offRoute': True}, {'active': 1}, {'version': True},
  {'ageMs': '2010'}, {'ageMs': float('nan')}, {'validForMs': False}, {'validForMs': '0'},
  {'maneuver': []}, {'position': {'ageMs': 'bad'}}, {'routeChunk': {}}, {'routeChunk': 'bad'},
])
def test_expired_tail_malformed_or_inactive_discards_quarantine(delay, arrival, change):
  state = complete()
  state.accept({**aged_tail(delay), **change}, arrival, arrival)
  state.accept(packet(POINTS, sequence=2, index=1), 11.93, 11.93)
  assert state.snapshot(11.93) is None


@pytest.mark.parametrize('delay', [20, 200, 600])
@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
@pytest.mark.parametrize('field,value', [('sha256', '0' * 64), ('data', '?' * 1024), ('pointCount', 999)])
def test_expired_tail_conflicting_chunk_discards_quarantine(delay, arrival, field, value):
  state = complete()
  tail = aged_tail(delay)
  tail['routeChunk'][field] = value
  state.accept(tail, arrival, arrival)
  state.accept(packet(POINTS, sequence=2, index=1), 11.93, 11.93)
  assert state.snapshot(11.93) is None


@pytest.mark.parametrize('delay', [20, 200, 600])
@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
def test_expired_tail_newer_stale_sequence_still_discards_quarantine(delay, arrival):
  state = complete()
  state.accept(aged_tail(delay, sequence=2), arrival, arrival)
  state.accept(packet(POINTS, sequence=3, index=1), 11.93, 11.93)
  assert state.snapshot(11.93) is None


@pytest.mark.parametrize('field,value', [('source', 'naver'), ('session', 'other'), ('routeId', 'other')])
@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
def test_expired_tail_requires_exact_high_water_identity(field, value, arrival):
  state = complete()
  tail = aged_tail(200)
  tail[field] = value
  state.accept(tail, arrival, arrival)
  state.accept(packet(POINTS, sequence=2, index=1), 11.93, 11.93)
  assert state.snapshot(11.93) is None


@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
def test_expired_tail_shorter_valid_for_preserves_only_until_original_expiry(arrival):
  state = complete()
  tail = packet(POINTS)
  tail['ageMs'], tail['validForMs'] = 1790, 190
  age_model_feedback({'modelNavigation': tail}, 200)
  assert tail['ageMs'] == 1990 and tail['validForMs'] == 0
  state.accept(tail, arrival, arrival)
  assert state.snapshot(arrival) is None
  state.accept(packet(POINTS, sequence=2, index=1), 12., 12.)
  assert state.snapshot(12.) is not None


@pytest.mark.parametrize('arrival', [11.85, 11.9, 11.92])
@pytest.mark.parametrize('age', [2000, 2010])
@pytest.mark.parametrize('offset', [-.001, 0., .001])
def test_expired_tail_retention_uses_earliest_original_and_tail_deadline(arrival, age, offset):
  state = complete()
  tail = aged_tail(200, age=age)
  first_expiry = min(state.deadline, arrival, arrival - age / 1000. + 2.)
  state.accept(tail, arrival, arrival)
  for now in (12., 13., 14., 15., 16., 17., 18., 19., 20., 21.):
    state.accept(aged_tail(200), now, now)
    assert state.snapshot(now) is None and state.route_cache() is None
  restore_at = first_expiry + 10. + offset
  state.accept(packet(POINTS, sequence=2, index=1), restore_at, restore_at)
  assert (state.snapshot(restore_at) is not None) is (offset < 0)


@pytest.mark.parametrize('restore_at,expected', [(21.89, True), (21.9, False)])
def test_expired_tail_repetition_cannot_renew_original_quarantine_expiry(restore_at, expected):
  state = complete()
  for now in (11.92, 12., 13., 14., 15., 16., 17., 18., 19., 20., 21.):
    state.accept(aged_tail(200), now, now)
    assert state.snapshot(now) is None and state.route_cache() is None
  state.accept(packet(POINTS, sequence=2, index=1), restore_at, restore_at)
  assert (state.snapshot(restore_at) is not None) is expected


@pytest.mark.parametrize('gap', ['expiry', 'missing'])
def test_quarantine_recovers_only_with_new_sequence_and_matching_chunk(gap):
  state = complete()
  original = state.points
  generation = state.cache_generation
  if gap == 'expiry':
    expire(state)
  else:
    state.accept(None, 11., 11.)
    assert state.snapshot(11.) is None and state.route_cache() is None
  state.accept(packet(POINTS, sequence=2, index=1), 12., 12.)
  assert state.snapshot(12.).points is original
  assert state.request(12.)['index'] == -1
  assert state.cache_generation > generation
  assert state.observed == 11.9 and state.deadline == 13.9


def test_quarantine_no_chunk_then_same_sequence_proof_cannot_restore():
  state = complete()
  expire(state)
  fresh = packet(POINTS, sequence=2)
  del fresh['routeChunk']
  state.accept(fresh, 12., 12.)
  assert state.snapshot(12.) is None
  state.accept(packet(POINTS, sequence=2), 12.1, 12.1)
  assert state.snapshot(12.1) is None
  state.accept(packet(POINTS, sequence=3), 12.2, 12.2)
  assert state.snapshot(12.2).points == tuple(POINTS)


@pytest.mark.parametrize('sequence', [1, 0])
def test_quarantine_old_sequence_cannot_restore(sequence):
  state = complete()
  expire(state)
  state.accept(packet(POINTS, sequence=sequence), 12., 12.)
  assert state.snapshot(12.) is None and state.route_cache() is None


@pytest.mark.parametrize('restore_at,expected', [(21.89, True), (21.9, False), (22., False)])
def test_quarantine_first_expiry_is_not_renewed_by_missing_components(restore_at, expected):
  state = complete()
  expire(state)
  for now in (12., 13., 14., 15., 16., 17., 18., 19., 20., 21.):
    state.accept(None, now, now)
    assert state.snapshot(now) is None and state.route_cache() is None
  state.accept(packet(POINTS, sequence=2), restore_at, restore_at)
  assert (state.snapshot(restore_at) is not None) is expected


def test_quarantine_pending_new_sequences_do_not_renew_retention():
  state = complete()
  expire(state)
  for sequence in range(2, 12):
    incoming = packet(POINTS, sequence=sequence)
    del incoming['routeChunk']
    now = 10. + sequence
    state.accept(incoming, now, now)
    assert state.snapshot(now) is None
  state.accept(packet(POINTS, sequence=12), 21.9, 21.9)
  assert state.snapshot(21.9) is None


@pytest.mark.parametrize('change', [{'active': False}, {'offRoute': True}, {'routeChunk': {}}])
def test_quarantine_same_sequence_invalidation_cannot_be_ignored(change):
  state = complete()
  expire(state)
  state.accept({**packet(POINTS), **change}, 11.92, 11.92)
  state.accept(packet(POINTS, sequence=2), 11.93, 11.93)
  assert state.snapshot(11.93) is None


def test_quarantine_missing_hash_discards_candidate():
  state = complete()
  expire(state)
  incoming = packet(POINTS, sequence=2)
  del incoming['routeChunk']['sha256']
  state.accept(incoming, 12., 12.)
  state.accept(packet(POINTS, sequence=3), 12.1, 12.1)
  assert state.snapshot(12.1) is None


def test_quarantine_expiry_during_accept_requires_chunk_proof():
  state = complete()
  incoming = packet(POINTS, sequence=2)
  del incoming['routeChunk']
  state.accept(incoming, 11.91, 11.91)
  assert state.snapshot(11.91) is None
  state.accept(packet(POINTS, sequence=3), 12., 12.)
  assert state.snapshot(12.).points == tuple(POINTS)


@pytest.mark.parametrize('change', [
  {'active': False}, {'offRoute': True}, {'version': 2}, {'ageMs': 2001},
  {'validForMs': 0}, {'sequence': True}, {'source': 'unknown'},
  {'routeChunk': 'invalid'}, {'routeChunk': {}},
])
def test_quarantine_invalid_or_inactive_input_discards_geometry(change):
  state = complete()
  expire(state)
  state.accept({**packet(POINTS, sequence=2), **change}, 12., 12.)
  state.accept(packet(POINTS, sequence=3), 12.1, 12.1)
  assert state.snapshot(12.1) is None


@pytest.mark.parametrize('field,value', [
  ('sha256', '0' * 64), ('encodedLength', 3072), ('count', 4), ('pointCount', 999),
  ('encoding', 'polyline5'), ('index', -1), ('data', '?' * 1024),
])
def test_quarantine_full_manifest_and_exact_bytes_are_required(field, value):
  state = complete()
  expire(state)
  incoming = packet(POINTS, sequence=2)
  incoming['routeChunk'][field] = value
  state.accept(incoming, 12., 12.)
  assert state.snapshot(12.) is None
  # A later valid proof cannot undo the intervening incompatible route evidence.
  state.accept(packet(POINTS, sequence=3), 12.1, 12.1)
  assert state.snapshot(12.1) is None


@pytest.mark.parametrize('field,value', [('source', 'naver'), ('session', 'other'), ('routeId', 'other')])
def test_quarantine_source_session_or_route_change_discards_geometry(field, value):
  state = complete()
  expire(state)
  state.accept({**packet(POINTS, sequence=2), field: value}, 12., 12.)
  state.accept(packet(POINTS, sequence=3), 12.1, 12.1)
  assert state.snapshot(12.1) is None


@pytest.mark.parametrize('clear', ['clear', 'clear_geometry', 'model_disconnected', 'model_link_changed', 'model_link_stale'])
def test_quarantine_explicit_clear_discards_geometry(clear):
  state = complete()
  expire(state)
  if clear in ('clear', 'clear_geometry'):
    getattr(state, clear)()
  else:
    state.clear(clear)
  state.accept(packet(POINTS, sequence=2), 12., 12.)
  assert state.snapshot(12.) is None


@pytest.mark.parametrize('event', ['stale_packet', 'link_silence', 'rollback', 'expired_proof'])
def test_quarantine_link_stale_clock_rollback_or_stale_proof_cannot_restore(event):
  state = complete()
  expire(state)
  if event == 'stale_packet':
    state.accept(None, 9., 12.)
  elif event == 'link_silence':
    state.snapshot(14.)
  elif event == 'rollback':
    state.snapshot(11.)
  else:
    value = packet(POINTS, sequence=2)
    value['ageMs'], value['validForMs'] = 1900, 100
    state.accept(value, 11.8, 12.)
  state.accept(packet(POINTS, sequence=3), 14.1, 14.1)
  assert state.snapshot(14.1) is None


def test_quarantine_partial_geometry_is_never_recovered():
  state = ModelNavigationTracker()
  state.accept(packet(POINTS), 10., 10.)
  expire(state)
  state.accept(packet(POINTS, sequence=2, index=1), 12., 12.)
  assert state.snapshot(12.) is None and len(state.chunks) == 1


def test_quarantine_restore_rotates_sidecar_token_and_preserves_reader_barrier(tmp_path):
  state = complete()
  path = tmp_path / 'feedback.json'
  reader = NavigationReader(path)
  cache = ModelRouteCache(reader.model_route_path, lambda p, v: p.write_text(json.dumps(v)))
  old_token = cache.update(state)
  write_feedback(path, packet(POINTS), 10.1, old_token)
  assert reader.model_read(10.1) is not None
  expire(state)
  assert cache.update(state) is None and not cache.path.exists()
  write_feedback(path, None, 11.91, None)
  assert reader.model_read(11.91) is None
  incoming = packet(POINTS, sequence=2, index=1)
  state.accept(incoming, 12., 12.)
  new_token = cache.update(state)
  assert new_token is not None and new_token != old_token
  # New sidecar with old outer token must still fail closed.
  write_feedback(path, incoming, 12., old_token)
  assert reader.model_read(12.) is None
  write_feedback(path, incoming, 12.2, new_token)
  assert reader.model_read(12.2).points == tuple(POINTS)


@pytest.mark.parametrize('previously_complete', [False, True])
@pytest.mark.parametrize('tail_offset', [-.05, 0., .02])
def test_quarantine_300ms_jitter_recovers_only_previously_complete_geometry(previously_complete, tail_offset):
  points = [(round(37. + i * .0001, 6), round(127. + i * .0001, 6)) for i in range(5572)]
  count = packet(points)['routeChunk']['count']
  assert count > 20
  state = ModelNavigationTracker()
  if previously_complete:
    for index in range(count):
      now = 10. + index * .01
      state.accept(packet(points, index=index), now, now)
    assert len(state.snapshot(now).points) == 5572
  ready = []
  # Measured fixture ordering: poll crosses old deadline before the next 1 Hz
  # observation arrives in the ACK, after 300 ms each way plus 0..40 ms jitter.
  for second in range(10):
    now = 12. + second
    tail = packet(points, sequence=1 + second)
    tail['ageMs'], tail['validForMs'] = 1410, 590
    age_model_feedback({'modelNavigation': tail}, 600)
    tail_at = state.deadline + tail_offset if state.value is not None else now
    state.accept(tail, tail_at, tail_at)
    assert state.snapshot(tail_at) is None and state.route_cache() is None
    assert state.request(now) == {'routeId': '', 'index': 0}
    assert state.snapshot(now) is None and state.route_cache() is None
    for fragment in range(5):
      at = now + .08 + fragment * .2
      incoming = packet(points, sequence=2 + second, index=fragment)
      incoming['ageMs'] = 1120 + fragment * 200
      incoming['validForMs'] = 880 - fragment * 200
      state.accept(incoming, at, at)
      ready.append(state.snapshot(at) is not None)
  assert all(ready) if previously_complete else not any(ready)
