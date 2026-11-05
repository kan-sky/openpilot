import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ, NAVI_SOURCE_NAVDY
from openpilot.selfdrive.carrot.tests.test_deceleration_candidates import make_serv, publish
from openpilot.selfdrive.navdy.navigation import FeedbackTracker
from openpilot.selfdrive.navdy.tests.test_section import section_packet


def test_actual_phone_json_types_reach_published_section_and_clear(monkeypatch):
  # Synthetic SDK inputs serialized by the production Java SectionRelay helper.
  fixture = Path(__file__).parents[2] / 'navdy/tests/fixtures/section-phone-20261101.jsonl'
  values = [json.loads(line) for line in fixture.read_text().splitlines()]
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  for i, (guidance, kind, source, cap) in enumerate(zip(
      values, ('mobile', 'fixed', 'none'), ('section', 'cam', 'road'), (110, 80, 200), strict=True)):
    clock[0] = 10 + i * .1
    value = section_packet(i + 1)
    value['navigationFeedback']['guidance'] = guidance
    value.update(cameraType=kind, cameraSpeedKph=80, cameraDistance='1 m')
    tracker.accept(value, clock[0], clock[0])
    result = publish(serv, sm, pm, sent)
    assert (result.desiredSource, result.desiredSpeed) == (source, cap)
    assert any(c.source == 'section' for c in result.decelerationCandidates) == (i < 2)


def setup(monkeypatch, mode=2):
  serv, sm, pm, sent = make_serv(monkeypatch, AutoNaviSpeedCtrlMode=mode, NaviSource=1)
  serv.navi_source = NAVI_SOURCE_NAVDY
  serv.external_navigation_active = True
  tracker = FeedbackTracker()
  clock = [10.0]
  serv.navdy_navigation.read = lambda: tracker.snapshot(clock[0])
  serv._update_carrot_navi = CarrotServ._update_carrot_navi.__get__(serv)
  return serv, sm, pm, sent, tracker, clock


@pytest.mark.parametrize('limit', [100, 110])
@pytest.mark.parametrize('mode', [0, 1, 2, 3])
def test_section_survives_mobile_replacement_and_filters_only_mobile(monkeypatch, limit, mode):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch, mode)
  for i, kind in enumerate(('section', 'mobile', 'mobile', 'fixed', 'section')):
    clock[0] = 10 + i * .2
    value = section_packet(i + 1, limitKph=limit)
    value.update(cameraType=kind, cameraSpeedKph=120, cameraDistance='20 m')
    tracker.accept(value, clock[0], clock[0])
    result = publish(serv, sm, pm, sent)
    if mode == 0:
      assert result.desiredSource == 'road' and result.desiredSpeed == 200
    else:
      assert (result.desiredSource, result.desiredSpeed) == ('section', limit)
      assert (result.xSpdType, result.xSpdLimit) == (4, limit)
      assert any(c.source == 'section' for c in result.decelerationCandidates)


def test_lower_fixed_camera_then_return_to_continuous_section(monkeypatch):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  value = section_packet(limitKph=100)
  value.update(cameraType='fixed', cameraSpeedKph=80, cameraDistance='1 m')
  tracker.accept(value, 10, 10)
  result = publish(serv, sm, pm, sent)
  assert (result.desiredSource, result.desiredSpeed) == ('cam', 80)
  assert {c.source for c in result.decelerationCandidates} == {'section', 'cam'}
  sm['selfdriveState'].distanceTraveled = 10
  clock[0] = 10.1
  result = publish(serv, sm, pm, sent)
  assert (result.desiredSource, result.desiredSpeed) == ('section', 100)
  assert result.xSpdDist == 7889


def test_hazard_updates_never_rewind_the_other_distance(monkeypatch):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  for i in range(12):
    clock[0] = 10 + i * .1
    value = section_packet(i + 1, limitKph=100)
    value.update(cameraType='mobile' if i % 2 else 'fixed', cameraSpeedKph=120, cameraDistance=f'{300-i} m')
    tracker.accept(value, clock[0], clock[0])
    sm['selfdriveState'].distanceTraveled = i * 3
    result = publish(serv, sm, pm, sent)
    assert result.xSpdDist == 7899 - i * 3
  # Section changes cannot replay the point camera's pre-motion position.
  serv.xSpdDist = 200
  value = section_packet(13, distanceM=7000)
  value.update(cameraType='mobile', cameraSpeedKph=120, cameraDistance='289 m')
  serv.autoNaviSpeedCtrlMode = 3
  tracker.accept(value, 11.2, 11.2)
  clock[0] = 11.2
  publish(serv, sm, pm, sent)
  assert serv.navdy_section_distance == 7000


@pytest.mark.parametrize('distance', [8, 26])
def test_frozen_near_end_distance_cannot_be_rearmed_by_point_updates(monkeypatch, distance):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  for i in range(15):
    clock[0] = 10 + i * .1
    value = section_packet(i + 1, distanceM=distance)
    value.update(cameraType='mobile', cameraDistance=f'{300-i} m')
    tracker.accept(value, clock[0], clock[0])
    sm['selfdriveState'].distanceTraveled = i * 3
    result = publish(serv, sm, pm, sent)
    assert (result.desiredSource == 'section') == (i * 3 < distance)


def test_section_change_preserves_point_distance_and_equal_caps_prefer_section(monkeypatch):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch, 3)
  value = section_packet(limitKph=100)
  value.update(cameraType='fixed', cameraSpeedKph=100, cameraDistance='20 m')
  tracker.accept(value, 10, 10)
  assert publish(serv, sm, pm, sent).desiredSource == 'section'
  serv.xSpdDist = 8
  value = section_packet(2, distanceM=7000)
  value.update(cameraType='fixed', cameraSpeedKph=100, cameraDistance='20 m')
  tracker.accept(value, 10.1, 10.1)
  clock[0] = 10.1
  publish(serv, sm, pm, sent)
  assert serv.xSpdDist == 8


def test_heartbeat_only_eventually_revokes_actual_cap(monkeypatch):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  for i in range(51):
    clock[0] = 10 + i * .1
    value = section_packet(sequence=1)
    value['navigationFeedback']['sequence'] = i + 1
    value['cameraType'] = 'section'
    tracker.accept(value, clock[0], clock[0])
    result = publish(serv, sm, pm, sent)
    assert (result.desiredSource == 'section') == (i < 40)


def test_clear_then_new_observation_is_authoritative_not_old_distance_memory(monkeypatch):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  tracker.accept(section_packet(), 10, 10)
  publish(serv, sm, pm, sent)
  sm['selfdriveState'].distanceTraveled = 30
  clock[0] = 10.1
  tracker.accept(section_packet(2, active=False), 10.1, 10.1)
  publish(serv, sm, pm, sent)
  clock[0] = 10.2
  tracker.accept(section_packet(3), 10.2, 10.2)
  publish(serv, sm, pm, sent)
  assert serv.navdy_section_distance == 7899


@pytest.mark.parametrize('clear', ['inactive', 'expired', 'disconnect', 'distance', 'suspended', 'offroute'])
def test_section_revocation_reaches_actual_published_cap(monkeypatch, clear):
  serv, sm, pm, sent, tracker, clock = setup(monkeypatch)
  value = section_packet()
  value['cameraType'] = 'section'
  tracker.accept(value, 10, 10)
  assert publish(serv, sm, pm, sent).desiredSource == 'section'
  newer = section_packet(2)
  newer['cameraType'] = 'section'
  if clear == 'inactive':
    newer['navigationFeedback']['guidance']['section']['active'] = False
  elif clear == 'suspended':
    newer['navigationFeedback']['guidance']['section']['suspended'] = True
  elif clear == 'offroute':
    newer['navigationFeedback']['guidance']['section']['offRoute'] = True
  elif clear == 'distance':
    sm['selfdriveState'].distanceTraveled = 7900
  elif clear == 'expired':
    newer['navigationFeedback']['guidance']['section']['ageMs'] = 4000
  clock[0] = 12.1 if clear == 'disconnect' else 10.1
  if clear not in ('disconnect', 'distance'):
    tracker.accept(newer, clock[0], clock[0])
  result = publish(serv, sm, pm, sent)
  assert result.desiredSource == 'road' and result.desiredSpeed == 200
  assert not any(c.source == 'section' for c in result.decelerationCandidates)
