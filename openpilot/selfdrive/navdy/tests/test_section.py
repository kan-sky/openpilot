from copy import deepcopy

import pytest

from openpilot.selfdrive.navdy.navigation import FeedbackTracker
from openpilot.selfdrive.navdy.tests.test_navigation import packet


def section_packet(sequence=1, **fields):
  value = packet(sequence)
  value['navigationFeedback']['validForMs'] = 4000
  value['navigationFeedback']['guidance']['section'] = {
    'version': 1, 'session': 'phone-section', 'sequence': sequence, 'ageMs': 0,
    'active': True, 'limitKph': 110, 'distanceM': 7899, 'offRoute': False, 'suspended': False, **fields,
  }
  return value


def test_section_and_point_camera_survive_independently():
  tracker = FeedbackTracker()
  for i, kind in enumerate(('section', 'mobile', 'fixed', 'mobile')):
    value = section_packet(i + 1)
    value['cameraType'] = kind
    assert tracker.accept(value, 10 + i * .2, 10 + i * .2)
    state = tracker.snapshot(10 + i * .2)
    assert state.speed.section_active
    assert (state.speed.section_speed_limit_kph, state.speed.section_remaining_distance_m) == (110, 7899)
    assert state.speed.sdi_present == (kind != 'section')


def test_new_observations_renew_lifetime_but_not_distance_sequence():
  tracker = FeedbackTracker()
  for i in range(40):
    tracker.accept(section_packet(i + 1), 10 + i, 10 + i)
    state = tracker.snapshot(10 + i)
    assert state.speed.section_active
    assert state.speed.sequence == 1


def test_source_and_relay_age_are_added_and_duplicates_cannot_renew():
  tracker = FeedbackTracker()
  value = section_packet(ageMs=2000)
  value['navigationFeedback']['ageMs'] = 1000
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10.99).speed.section_active
  for now in (10.5, 11, 11.5):
    replay = section_packet(sequence=1)
    replay['navigationFeedback']['sequence'] = int(now * 100)
    tracker.accept(replay, now, now)
  assert not tracker.snapshot(11.5).speed.section_active


@pytest.mark.parametrize('age,active', [(3999, True), (4000, False), (4001, False)])
def test_source_age_boundary(age, active):
  tracker = FeedbackTracker()
  tracker.accept(section_packet(ageMs=age), 10, 10)
  assert tracker.snapshot(10).speed.section_active == active


@pytest.mark.parametrize('change', [
  {'active': False}, {'active': 1}, {'suspended': True}, {'offRoute': True}, {'distanceM': 0},
  {'distanceM': -1}, {'distanceM': True}, {'distanceM': 100001}, {'distanceM': float('nan')},
  {'limitKph': 0}, {'limitKph': True}, {'limitKph': 141}, {'limitKph': '110'}, {'limitKph': 110.0}, {'distanceM': 7899.0},
  {'version': True}, {'sequence': True}, {'ageMs': -1}, {'ageMs': float('inf')}, {'session': ''},
])
def test_bad_or_inactive_source_clears_without_legacy_revival(change):
  tracker = FeedbackTracker()
  value = section_packet()
  value['cameraType'] = 'section'
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.section_active
  value = section_packet(2)
  value['navigationFeedback']['guidance']['section'].update(change)
  value['cameraType'] = 'section'
  tracker.accept(value, 10.5, 10.5)
  state = tracker.snapshot(10.5)
  assert not state.speed.section_active and not state.speed.sdi_present


@pytest.mark.parametrize('clear', ['null', 'missing', 'guidance_stop', 'guidance_expiry', 'link_expiry', 'relay_restart'])
def test_clear_propagates_to_speed_sequence_and_cannot_replay(clear):
  tracker = FeedbackTracker()
  value = section_packet()
  value['cameraType'] = 'section'
  tracker.accept(value, 10, 10)
  sequence = tracker.snapshot(10).speed.sequence
  newer = deepcopy(value)
  newer['navigationFeedback']['sequence'] += 1
  if clear == 'null':
    newer['navigationFeedback']['guidance']['section'] = None
  elif clear == 'missing':
    newer['navigationFeedback']['guidance'].pop('section')
  elif clear == 'guidance_stop':
    newer['navigationFeedback']['guidance']['active'] = False
  elif clear == 'relay_restart':
    newer['navigationFeedback']['session'] = 'new-hud'
  elif clear == 'guidance_expiry':
    newer['navigationFeedback']['validForMs'] = 0
  else:
    assert tracker.snapshot(12) is None
  tracker.accept(newer, 12.1 if clear == 'link_expiry' else 10.5, 12.1 if clear == 'link_expiry' else 10.5)
  state = tracker.snapshot(12.1 if clear == 'link_expiry' else 10.5)
  assert not state.speed.section_active and not state.speed.sdi_present
  assert state.speed.sequence > sequence


def test_old_or_retired_generation_never_replaces_current_section():
  tracker = FeedbackTracker()
  tracker.accept(section_packet(5), 10, 10)
  old = section_packet(4, limitKph=40)
  old['navigationFeedback']['sequence'] = 6
  tracker.accept(old, 10.1, 10.1)
  assert tracker.snapshot(10.1).speed.section_speed_limit_kph == 110
  current = section_packet(1, session='new-phone', limitKph=100)
  current['navigationFeedback']['sequence'] = 7
  tracker.accept(current, 10.2, 10.2)
  assert tracker.snapshot(10.2).speed.section_speed_limit_kph == 100
  old['navigationFeedback']['sequence'] = 8
  tracker.accept(old, 10.3, 10.3)
  assert tracker.snapshot(10.3).speed.section_speed_limit_kph == 100


def test_long_section_is_not_restricted_to_point_camera_distance_bound():
  tracker = FeedbackTracker()
  tracker.accept(section_packet(distanceM=15000), 10, 10)
  assert tracker.snapshot(10).speed.section_remaining_distance_m == 15000


def test_old_phone_legacy_section_remains_compatible():
  tracker = FeedbackTracker()
  value = packet()
  value['cameraType'] = 'section'
  tracker.accept(value, 10, 10)
  state = tracker.snapshot(10)
  assert not state.speed.section_active
  assert state.speed.sdi_type == 4


def test_legacy_downgrade_requires_relay_session_change():
  tracker = FeedbackTracker()
  tracker.accept(section_packet(), 10, 10)
  value = packet(2)
  value['cameraType'] = 'section'
  tracker.accept(value, 10.1, 10.1)
  assert not tracker.snapshot(10.1).speed.sdi_present
  value['navigationFeedback']['session'] = 'restarted-hud-legacy-phone'
  tracker.accept(value, 10.2, 10.2)
  assert tracker.snapshot(10.2).speed.sdi_type == 4
