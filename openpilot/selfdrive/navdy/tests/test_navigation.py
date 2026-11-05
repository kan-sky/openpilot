import json
from pathlib import Path

import pytest

from openpilot.selfdrive.navdy.navigation import FeedbackTracker, NavigationReader, camera_distance


def packet(sequence=1, age=0, active=True):
  return {
    'roadPreview': {'version': 1, 'session': 'road-session', 'sequence': sequence, 'ageMs': age, 'roadLimitKph': 60},
    'navigationFeedback': {'version': 1, 'session': 'navdy-session', 'sequence': sequence, 'ageMs': age,
                           'validForMs': max(0, 2500 - age),
                           'guidance': {'active': active, 'primaryType': 12, 'primaryDistanceMeters': 120,
                                        'secondaryType': 13, 'secondaryDistanceMeters': 500,
                                        'roadName': '서울역 "앞"\n좌회전', 'remainingDistanceMeters': 1500,
                                        'remainingTimeSeconds': 240,
                                        'trafficSignal': {'visible': True, 'red': True, 'left': False,
                                                          'green': False, 'remainingSeconds': 20}}},
    'cameraSpeedKph': 50, 'cameraSource': 'trafficNotification', 'cameraType': 'fixed', 'cameraDistance': '350 m',
  }


def test_complete_packet_and_independent_expiry():
  tracker = FeedbackTracker()
  assert tracker.accept(packet(age=1000), 10, 10)
  state = tracker.snapshot(10.5)
  assert state.speed.road_limit_kph == 60 and state.speed.sdi_speed_limit_kph == 50
  assert state.current.turn_type == 12 and state.current.distance_m == 120
  assert state.next.turn_type == 13 and state.route.remaining_distance_m == 1500
  assert state.current.main_text == '서울역 "앞"\n좌회전'
  assert state.traffic.lamp == 'red' and state.traffic.remain_sec == 18
  assert tracker.snapshot(12) is None  # No fresh USB ACK.


def test_repeated_ack_does_not_rejuvenate_frozen_guidance():
  tracker = FeedbackTracker()
  for now in (10, 11, 12, 13, 14):
    tracker.accept(packet(age=0), now, now)  # Same source session/sequence, even with dishonest age=0.
  state = tracker.snapshot(14)
  assert state.speed.sdi_present  # Legacy camera is independently current.
  assert state.speed.road_limit_kph is None
  assert not state.current.present and not state.traffic.present


def test_fresh_new_sequence_and_restart_session():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10, 10)
  tracker.accept(packet(sequence=2), 13, 13)
  assert tracker.snapshot(14).current.present
  value = packet(sequence=0)
  value['navigationFeedback']['session'] = 'restarted'
  tracker.accept(value, 14, 14)
  assert tracker.snapshot(15).current.present


def test_inactive_clear_preserves_only_fresh_independent_road_and_camera():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10, 10)
  tracker.accept(packet(sequence=2, active=False), 11, 11)
  state = tracker.snapshot(11)
  assert not state.current.present and not state.route.present and not state.traffic.present
  assert state.speed.road_limit_kph == 60


def test_road_heartbeat_does_not_reset_camera_distance_sequence():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10, 10)
  sequence = tracker.snapshot(10).speed.sequence
  tracker.accept(packet(sequence=2), 11, 11)
  assert tracker.snapshot(11).speed.sequence == sequence
  changed = packet(sequence=3)
  changed['cameraDistance'] = '300 m'
  tracker.accept(changed, 12, 12)
  assert tracker.snapshot(12).speed.sequence > sequence


def test_fresh_phone_sdi_bump_is_independent_of_legacy_camera():
  tracker = FeedbackTracker()
  value = packet()
  value['navigationFeedback']['guidance']['sdi'] = {'type': 22, 'distanceMeters': 127}
  tracker.accept(value, 10, 10)
  state = tracker.snapshot(10)
  assert state.speed.sdi_present and state.speed.secondary_sdi_present
  assert state.speed.secondary_sdi_type == 22 and state.speed.secondary_sdi_distance_m == 127
  for now in (11, 12, 13, 14):
    tracker.accept(value, now, now)
  assert not tracker.snapshot(14).speed.secondary_sdi_present


@pytest.mark.parametrize('mutate', [
  lambda p: p['roadPreview'].update(roadLimitKph=True),
  lambda p: p['roadPreview'].update(roadLimitKph=61),
  lambda p: p['roadPreview'].update(ageMs=-1),
  lambda p: p['roadPreview'].update(ageMs=4001),
  lambda p: p['roadPreview'].update(version=True),
  lambda p: p['roadPreview'].update(sequence=True),
])
def test_bad_road_does_not_become_speed_limit(mutate):
  value = packet()
  mutate(value)
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.road_limit_kph is None


@pytest.mark.parametrize('value', [None, [], True, float('nan'), float('inf'), -1, 100000])
def test_bad_receive_timestamp(value):
  assert not FeedbackTracker().accept(packet(), value, 10)


def test_malformed_camera_clears_prior_camera():
  tracker = FeedbackTracker()
  tracker.accept(packet(), 10, 10)
  value = packet(2)
  value['cameraType'] = []
  tracker.accept(value, 11, 11)
  assert not tracker.snapshot(11).speed.sdi_present


def test_old_apk_still_supplies_camera_without_claiming_tbt():
  value = packet()
  value.pop('navigationFeedback')
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.sdi_present
  assert not tracker.snapshot(10).current.present and not tracker.extended


def test_signal_deadline_cannot_be_extended_by_ack_or_packet_loss():
  tracker = FeedbackTracker()
  value = packet()
  value['navigationFeedback']['validForMs'] = 100
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).traffic.present
  replay = packet()
  tracker.accept(replay, 10.05, 10.05)
  assert not tracker.snapshot(10.11).traffic.present
  assert not tracker.snapshot(10.11).current.present


def test_older_extended_apk_without_signal_deadline_cannot_publish_signal():
  tracker = FeedbackTracker()
  value = packet()
  value['navigationFeedback'].pop('validForMs')
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).current.present
  assert not tracker.snapshot(10).traffic.present


def test_reader_checks_record_age_not_file_mtime(tmp_path):
  path = tmp_path / 'feedback.json'
  path.write_text(json.dumps({'receivedAt': 10, 'feedback': packet()}))
  reader = NavigationReader(path)
  assert reader.read(10).current.present
  path.write_text(json.dumps({'receivedAt': 10, 'feedback': packet()}))
  assert reader.read(12) is None
  path.write_text('[]')
  assert reader.read(13) is None
  path.write_text('x' * 34000)
  assert reader.read(14) is None


@pytest.mark.parametrize(('value', 'expected'), [('1.3km', 1300), ('320 m', 320), ('--', None), ('-3 m', None),
                                               ('0 m', None), ('9999999 km', None), ([], None)])
def test_camera_distance(value, expected):
  assert camera_distance(value) == expected


def test_no_vehicle_command_or_bluetooth_reset_in_transport():
  source = (Path(__file__).parents[1] / 'transport.py').read_text()
  for forbidden in ('kill-server', 'svc bluetooth', 'force-stop', '/sys/kernel', 'reboot'):
    assert forbidden not in source


def test_preview_bump_survives_another_selected_sign():
  value = packet()
  value['roadPreview'].update(guidance=True, bumpDistanceM=96)
  value['navigationFeedback']['guidance']['sdi'] = {'type': 18, 'distanceMeters': 40}
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  state = tracker.snapshot(10)
  assert state.speed.secondary_sdi_type == 22
  assert state.speed.secondary_sdi_distance_m == 96
  assert state.speed.sdi_type == 1


def test_guidance_bump_takes_precedence_over_preview():
  value = packet()
  value['roadPreview'].update(guidance=True, bumpDistanceM=130)
  value['navigationFeedback']['guidance']['sdi'] = {'type': 22, 'distanceMeters': 96}
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.secondary_sdi_distance_m == 96


@pytest.mark.parametrize('distance', [0, -1, None, True, '0', float('nan'), float('inf')])
def test_new_bump_clear_cannot_revive_older_preview(distance):
  value = packet()
  value['roadPreview'].update(guidance=True, bumpDistanceM=96)
  value['navigationFeedback']['guidance']['sdi'] = {'type': 22, 'distanceMeters': 5}
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.secondary_sdi_distance_m == 5
  value['navigationFeedback']['sequence'] = 2
  value['navigationFeedback']['guidance']['sdi']['distanceMeters'] = distance
  tracker.accept(value, 11, 11)
  state = tracker.snapshot(11)
  assert not state.speed.secondary_sdi_present
  assert state.speed.secondary_sdi_distance_m == 0


@pytest.mark.parametrize('distance', [True, None, '96', -1, 0, 2001, float('nan'), float('inf')])
def test_preview_bump_rejects_invalid_distance(distance):
  value = packet()
  value['roadPreview'].update(guidance=True, bumpDistanceM=distance)
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert not tracker.snapshot(10).speed.secondary_sdi_present


@pytest.mark.parametrize('clear', ['guidance', 'road', 'expiry'])
def test_preview_bump_clear_and_frozen_sequence(clear):
  value = packet()
  value['roadPreview'].update(guidance=True, bumpDistanceM=96)
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  assert tracker.snapshot(10).speed.secondary_sdi_present
  if clear == 'guidance':
    value['navigationFeedback']['sequence'] = 2
    value['navigationFeedback']['guidance']['active'] = False
  elif clear == 'road':
    value['roadPreview']['sequence'] = 2
    value['roadPreview']['guidance'] = False
  for now in (11, 12, 13, 14):
    tracker.accept(value, now, now)
    if clear != 'expiry':
      assert not tracker.snapshot(now).speed.secondary_sdi_present
  assert not tracker.snapshot(14).speed.secondary_sdi_present


@pytest.mark.parametrize('segments,total', [
  ([{'endM': 1500, 'status': 'G'}], 1500),
  ([{'endM': 500, 'status': 'G'}, {'endM': 1500, 'status': 'R'}], 1500),
])
def test_route_traffic_is_preserved(segments, total):
  value = packet()
  value['navigationFeedback']['guidance'].update(trafficTotalMeters=total, trafficSegments=segments)
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  route = tracker.snapshot(10).route
  assert route.traffic_total_m == total
  assert route.traffic_segments == tuple((s['endM'], s['status']) for s in segments)


@pytest.mark.parametrize('segments', [
  [{'endM': True, 'status': 'G'}], [{'endM': float('nan'), 'status': 'G'}],
  [{'endM': 2000, 'status': 'G'}], [{'endM': 500, 'status': 'G'}, {'endM': 400, 'status': 'R'}],
  [{'endM': 500, 'status': 'bad'}], [{'endM': 500, 'status': []}],
  [{'endM': i+1, 'status': 'G'} for i in range(65)], 'not-an-array',
])
def test_bad_route_traffic_cannot_break_guidance(segments):
  value = packet()
  value['navigationFeedback']['guidance'].update(trafficTotalMeters=1500, trafficSegments=segments)
  tracker = FeedbackTracker()
  tracker.accept(value, 10, 10)
  state = tracker.snapshot(10)
  assert state.current.present
  assert state.route.traffic_segments == () and state.route.traffic_total_m == 0
