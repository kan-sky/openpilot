"""Replay actual CarrotServ methods without IPC, CAN or device services."""
from dataclasses import replace
from unittest.mock import Mock  # noqa: TID251 -- mock-only; tests use pytest

import pytest

from openpilot.selfdrive.carrot.tests.test_navigation_source import services as services
from openpilot.selfdrive.navdy.navigation import FeedbackTracker
from openpilot.selfdrive.navdy.tests.test_navigation import packet


def hazard_packet(hazard="camera", **kwargs):
  value = packet(**kwargs)
  if hazard in ("bump", "both"):
    value["navigationFeedback"]["guidance"]["sdi"] = {"type": 22, "distanceMeters": 120}
  if hazard == "bump":
    value["cameraSpeedKph"] = 0
  return value


@pytest.fixture
def replay(services):
  cls, values = services
  values.update(CarrotNaviSource=1, AutoNaviSpeedCtrlMode=3, AutoNaviSpeedSafetyFactor=100,
                AutoNaviSpeedBumpSpeed=20)
  svc = cls()
  svc.params_memory = Mock()
  svc.navdy_navigation = Mock()
  tracker = FeedbackTracker()

  def apply(value, now):
    assert tracker.accept(value, now, now)
    state = tracker.snapshot(now)
    svc.navdy_navigation.read.return_value = state
    svc._update_carrot_navi(None)
    return state

  return svc, tracker, apply


@pytest.mark.parametrize("hazard", ["camera", "bump"])
@pytest.mark.parametrize("change", ["heartbeat", "expiry", "clear", "limit"])
def test_road_component_does_not_rewind_selected_hazard(replay, hazard, change):
  svc, _, apply = replay
  apply(hazard_packet(hazard), 10)
  initial = 350 if hazard == "camera" else 120
  assert svc.xSpdDist == initial
  svc.xSpdDist -= 50
  value = hazard_packet(hazard)
  value["navigationFeedback"]["sequence"] = 2  # Keep a bump fresh independently of the road.
  now = 11
  if change == "expiry":
    now = 14
  elif change == "clear":
    value["roadPreview"] = None
  else:
    value["roadPreview"]["sequence"] = 2
    if change == "limit":
      value["roadPreview"]["roadLimitKph"] = 80
  apply(value, now)
  assert svc.xSpdDist == initial - 50
  assert svc.xSpdType == (1 if hazard == "camera" else 22)
  assert svc.carrot_navi_road_limit_valid == (change not in ("expiry", "clear"))
  if change == "limit":
    assert svc.nRoadLimitSpeed == 80


@pytest.mark.parametrize("change", ["expiry", "clear", "distance"])
def test_other_hazard_changes_do_not_rewind_selected_camera(replay, change):
  svc, _, apply = replay
  apply(hazard_packet("both"), 10)
  svc.xSpdDist = 300
  value = hazard_packet("both")
  now = 11
  if change == "expiry":
    now = 14
    value["roadPreview"]["sequence"] = 2
  else:
    value["navigationFeedback"]["sequence"] = 2
    if change == "clear":
      value["navigationFeedback"]["guidance"].pop("sdi")
    else:
      value["navigationFeedback"]["guidance"]["sdi"]["distanceMeters"] = 100
  apply(value, now)
  assert svc.xSpdDist == 300
  assert svc.xSpdType == 1
  assert svc.nSdiPlusType == (22 if change == "distance" else -1)


@pytest.mark.parametrize("hazard", ["camera", "bump"])
def test_actual_new_hazard_distance_is_applied(replay, hazard):
  svc, _, apply = replay
  apply(hazard_packet(hazard), 10)
  svc.xSpdDist -= 50
  value = hazard_packet(hazard)
  if hazard == "camera":
    value["cameraDistance"] = "280 m"
    expected = 280
  else:
    value["navigationFeedback"]["sequence"] = 2
    value["navigationFeedback"]["guidance"]["sdi"]["distanceMeters"] = 60
    expected = 60
  apply(value, 11)
  assert svc.xSpdDist == expected


@pytest.mark.parametrize(("field", "value"), [("cameraType", "section"), ("cameraSpeedKph", 60)])
def test_actual_camera_kind_or_limit_change_is_applied(replay, field, value):
  svc, _, apply = replay
  apply(hazard_packet(), 10)
  svc.xSpdDist = 300
  changed = hazard_packet()
  changed[field] = value
  apply(changed, 11)
  assert svc.xSpdDist == 350
  assert svc.xSpdType == (4 if field == "cameraType" else 1)
  assert svc.xSpdLimit == (60 if field == "cameraSpeedKph" else 50)


def test_selected_camera_expiry_switches_to_bump_not_camera_progress(replay):
  svc, _, apply = replay
  apply(hazard_packet("both"), 10)
  svc.xSpdDist = 300
  apply(hazard_packet("bump"), 11)
  assert svc.xSpdType == 22
  assert svc.xSpdDist == 120


@pytest.mark.parametrize("reset", ["new_session", "disconnect"])
def test_session_boundary_does_not_reuse_previous_progress(replay, reset):
  svc, _, apply = replay
  state = apply(hazard_packet(), 10)
  svc.xSpdDist = 300
  if reset == "disconnect":
    svc.navdy_navigation.read.return_value = None
    assert not svc._update_carrot_navi(None)
  else:
    state = replace(state, session_id="replacement-session")
  svc.navdy_navigation.read.return_value = state
  assert svc._update_carrot_navi(None)
  assert svc.xSpdDist == 350


def test_passed_hazard_is_not_moved_ahead_by_road_change(replay):
  svc, _, apply = replay
  apply(hazard_packet(), 10)
  svc.xSpdDist = 0
  svc.xSpdType = -1
  svc.xSpdLimit = 0
  value = hazard_packet()
  value["roadPreview"].update(sequence=2, roadLimitKph=80)
  apply(value, 11)
  assert svc.xSpdDist == 0


def test_actual_feedback_restart_resets_distance_history(replay):
  svc, _, apply = replay
  first = apply(hazard_packet(), 10)
  svc.xSpdDist = 300
  changed = hazard_packet()
  changed['navigationFeedback']['session'] = 'new-hud-process'
  restarted = apply(changed, 11)
  assert first.session_id != restarted.session_id
  assert svc.xSpdDist == 350


def test_network_updates_keep_existing_reapply_behavior(services):
  cls, values = services
  values.update(CarrotNaviSource=0, AutoNaviSpeedCtrlMode=3, AutoNaviSpeedSafetyFactor=100)
  svc = cls()
  svc.params_memory = Mock()
  message = {
    "schemaVersion": 1, "connected": True, "sessionId": "network",
    "speed": {"meta": {"present": True, "sequence": 1}, "roadLimitValid": True,
              "roadLimitKph": 60, "sdiPresent": True, "sdiType": 1,
              "sdiDistanceM": 350, "sdiSpeedLimitKph": 50},
  }
  sm = Mock(alive={"carrotNavi": True}, valid={"carrotNavi": True}, updated={"carrotNavi": True})
  sm.__getitem__ = Mock(return_value=message)
  assert svc._update_carrot_navi(sm)
  svc.xSpdDist = 300
  message["speed"]["meta"]["sequence"] = 2
  message["speed"]["roadLimitKph"] = 80
  assert svc._update_carrot_navi(sm)
  assert svc.xSpdDist == 350
  assert svc.nRoadLimitSpeed == 80
