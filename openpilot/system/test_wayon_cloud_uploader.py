#!/usr/bin/env python3
import json
import os
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import requests

# Direct execution starts inside system/, so add openpilot root explicitly.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openpilot.system.wayon_cloud_uploader import (device_details_payload, enhance_wide_snapshot, gps_payload, openpilot_details_payload,
                                                   panda_details_payload, resolve_onroad_state,
                                                   upload_pending_impacts, upload_pending_vehicle_events,
                                                   vehicle_details_payload)
from openpilot.system.wayon_impact import enqueue_impact_event, peek_impact_event, peek_impact_events
from openpilot.system.wayon_vehicle_events import door_lock_event, enqueue_vehicle_event, peek_vehicle_event
from openpilot.cereal import car, log
from openpilot.system import wayon_cloud_uploader as uploader, wayon_impact, wayon_vehicle_events


NOW = datetime(2026, 7, 19, tzinfo=timezone.utc)


@pytest.fixture(autouse=True)
def isolated_inputs(monkeypatch, tmp_path):
  class FrozenDatetime(datetime):
    @classmethod
    def now(cls, tz=None):
      return NOW.astimezone(tz) if tz is not None else NOW.replace(tzinfo=None)

  for module in (uploader, wayon_impact, wayon_vehicle_events):
    monkeypatch.setattr(module, "datetime", FrozenDatetime)
  monkeypatch.setattr(uploader.time, "time", lambda: NOW.timestamp())
  monkeypatch.setattr(uploader.time, "monotonic", lambda: 100.0)

  def memory_params(root=None):
    if root is None:
      return SimpleNamespace(get_bool=lambda key: key == "IsOffroad")
    assert root == "/dev/shm/params"
    return SimpleNamespace(get_param_path=lambda key: str(tmp_path / key))

  monkeypatch.setattr(uploader, "Params", memory_params)
  monkeypatch.setattr(uploader.psutil, "virtual_memory", lambda: SimpleNamespace(available=1024 * 1024 * 1024))
  monkeypatch.setattr(uploader, "NAVDY_HEALTH_PATH", tmp_path / "navdy-status.json")
  monkeypatch.setattr(uploader, "UPLOAD_HEALTH_PATH", tmp_path / "upload-health.json")
  monkeypatch.setattr(uploader, "ROUTE_STATE_PATH", tmp_path / "route-state.json")
  def no_network(*args, **kwargs):
    pytest.fail("unit tests must not access cloud or vehicle services")
  monkeypatch.setattr(requests, "get", no_network)
  monkeypatch.setattr(requests, "post", no_network)


class FakeSubMaster(dict):
  def __init__(self, gps=None, seen=True, receive_time=100.0, **services):
    super().__init__(gpsLocation=gps, gpsLocationExternal=gps, **services)
    self.services = list(self)
    self.seen = {key: seen for key in self}
    self.valid = {key: True for key in self}
    self.alive = {key: seen for key in self}
    self.recv_time = {key: receive_time for key in self}
    self.logMonoTime = {key: int(receive_time * 1e9) for key in self}


def test_wide_snapshot_enhancement_lifts_dark_frames_without_clipping():
  image = np.full((8, 12, 3), 32, dtype=np.uint8)
  image[0, 0] = 255

  enhanced = enhance_wide_snapshot(image)

  assert enhanced.dtype == np.uint8
  assert enhanced.shape == image.shape
  assert np.median(enhanced) >= 68
  assert np.array_equal(enhanced[0, 0], image[0, 0])


def test_wide_snapshot_enhancement_leaves_bright_frames_unchanged():
  image = np.full((8, 12, 3), 96, dtype=np.uint8)
  assert enhance_wide_snapshot(image) is image


def test_started_override_is_authoritative():
  assert resolve_onroad_state(True, False) is False
  assert resolve_onroad_state(False, True) is True
  assert resolve_onroad_state(True) is True


def test_missing_gps_clears_current_location():
  sm = FakeSubMaster(gps=None, seen=False)
  params = SimpleNamespace(get=lambda _key: None)
  assert gps_payload(sm, params) == {"fresh": False, "source": "unavailable"}


def test_stale_gps_clears_current_location():
  gps = SimpleNamespace(
    hasFix=True,
    latitude=37.5,
    longitude=127.0,
    unixTimestampMillis=1,
  )
  sm = FakeSubMaster(gps=gps, receive_time=0.0)
  params = SimpleNamespace(get=lambda _key: None)
  assert gps_payload(sm, params) == {"fresh": False, "source": "unavailable"}


def test_ai_telemetry_helpers_expose_numeric_thermal_power_and_drive_state():
  device = SimpleNamespace(
    deviceType="tici", networkType="wifi", networkStrength="great", networkMetered=False,
    freeSpacePercent=71.5, memoryUsagePercent=42, gpuUsagePercent=5, cpuUsagePercent=[10, 20],
    powerDrawW=8.5, somPowerDrawW=4.2, offroadPowerUsageUwh=2_000_000,
    carBatteryCapacityUwh=18_000_000, cpuTempC=[55.0, 56.0], gpuTempC=[52.0], dspTempC=54.0,
    memoryTempC=50.0, modemTempC=[44.0], pmicTempC=[46.0], intakeTempC=38.0,
    exhaustTempC=42.0, gnssTempC=43.0, bottomSocTempC=51.0, maxTempC=56.0,
    thermalZones=[SimpleNamespace(name="cpu", temp=55.0)], thermalStatus="ok",
    fanSpeedPercentDesired=25, screenBrightnessPercent=60,
  )
  device_payload = device_details_payload(device)
  assert device_payload["thermal"]["temperaturesC"]["max"] == 56.0
  assert device_payload["power"]["offroadUsageWh"] == 2.0

  panda = SimpleNamespace(
    pandaType="tres", ignitionLine=True, ignitionCan=True, voltage=12_400,
    current=800, faultStatus="none", faults=[], uptime=60, heartbeatLost=False,
    interruptLoad=0.2, rxBufferOverflow=0, txBufferOverflow=0, spiChecksumErrorCount=0,
    harnessStatus="normal", controlsAllowed=True, controlsAllowedLateral=True,
    controlsAllowedLongitudinal=True, safetyModel="hyundaiCanfd", safetyParam=0,
    safetyRxInvalid=0, safetyTxBlocked=0, safetyRxChecksInvalid=False,
  )
  assert panda_details_payload(panda)["estimatedPowerW"] == 9.92

  cruise = SimpleNamespace(enabled=True, available=True, standstill=False, nonAdaptive=False,
                           speed=27.0, speedCluster=27.0)
  car = SimpleNamespace(
    vEgoCluster=20.0, vEgo=19.8, vEgoRaw=19.9, aEgo=0.1, yawRate=0.01, standstill=False,
    gearShifter="drive", steeringAngleDeg=1.2, steeringRateDeg=0.3, steeringPressed=False,
    gasPressed=False, brakePressed=False, parkingBrake=False, brakeHoldActive=False,
    leftBlinker=False, rightBlinker=True, leftBlindspot=False, rightBlindspot=True,
    doorOpen=False, seatbeltUnlatched=False, fuelGauge=0.6, charging=False,
    canValid=True, canTimeout=False, canErrorCounter=0, steerFaultTemporary=False,
    steerFaultPermanent=False, cruiseState=cruise,
  )
  vehicle = vehicle_details_payload(FakeSubMaster(carState=car), True)
  assert vehicle["speedKph"] == 72.0
  assert vehicle["rightBlindspot"] is True

  selfdrive = SimpleNamespace(
    state="enabled", enabled=True, active=True, engageable=True, experimentalMode=False,
    personality="standard", alertText1="", alertText2="", alertType="",
    alertStatus="normal", alertSize="none", alertSound="none", alertHudVisual="none",
  )
  assert openpilot_details_payload(FakeSubMaster(selfdriveState=selfdrive))["active"] is True


def test_vehicle_event_upload_removes_only_after_success(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  event = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=7)).isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(event, queue)
  posted = []

  def fake_post(config, path, payload):
    posted.append((path, payload))
    return {"ok": True}

  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", fake_post)
  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now) == 1
  assert posted == [("/api/vehicle-event", {**event, "deviceId": "device"})]


def test_vehicle_event_upload_holds_unlock_for_six_seconds(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  event = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=5.9)).isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(event, queue)
  posted = []
  monkeypatch.setattr(
    "openpilot.system.wayon_cloud_uploader.post_json",
    lambda config, path, payload: posted.append((path, payload)),
  )

  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now) == 0
  assert posted == []
  assert peek_vehicle_event(queue) == event


def test_vehicle_event_upload_sends_unlock_after_six_seconds(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  event = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=6.1)).isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(event, queue)
  posted = []
  monkeypatch.setattr(
    "openpilot.system.wayon_cloud_uploader.post_json",
    lambda config, path, payload: posted.append((path, payload)),
  )

  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now) == 1
  assert [payload["locked"] for _, payload in posted] == [False]
  assert peek_vehicle_event(queue) is None


def test_vehicle_event_upload_sends_pair_relocked_before_four_seconds(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  unlocked = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=3.9)).isoformat().replace("+00:00", "Z"),
  }
  locked = {
    **door_lock_event(True),
    "id": "lock-event",
    "occurredAt": now.isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(unlocked, queue)
  enqueue_vehicle_event(locked, queue)
  posted = []
  monkeypatch.setattr(
    "openpilot.system.wayon_cloud_uploader.post_json",
    lambda config, path, payload: posted.append((path, payload)),
  )

  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now) == 2
  assert [payload["locked"] for _, payload in posted] == [False, True]
  assert peek_vehicle_event(queue) is None


def test_vehicle_event_upload_suppresses_pair_relocked_from_four_to_six_seconds(tmp_path, monkeypatch):
  for elapsed_s in (4.0, 4.98, 6.0):
    queue = tmp_path / f"vehicle_events_{elapsed_s}.jsonl"
    suppressed_state = tmp_path / f"suppressed_{elapsed_s}.json"
    now = NOW
    unlocked = {
      **door_lock_event(False),
      "id": f"unlock-{elapsed_s}",
      "occurredAt": (now - timedelta(seconds=elapsed_s)).isoformat().replace("+00:00", "Z"),
    }
    locked = {
      **door_lock_event(True),
      "id": f"lock-{elapsed_s}",
      "occurredAt": now.isoformat().replace("+00:00", "Z"),
    }
    enqueue_vehicle_event(unlocked, queue)
    enqueue_vehicle_event(locked, queue)
    posted = []
    monkeypatch.setattr(
      "openpilot.system.wayon_cloud_uploader.post_json",
      lambda config, path, payload: posted.append((path, payload)),
    )
    marker_ms = int((now - timedelta(seconds=elapsed_s + 1.0)).timestamp() * 1000)

    assert upload_pending_vehicle_events(
      {"endpoint": "test", "token": "test"}, "device", queue, now=now,
      suppressed_state_path=suppressed_state,
      fetch_status_fn=lambda config: {"refresh_action": f"CLICKED_AT_{marker_ms}"}) == 0
    assert posted == []
    assert peek_vehicle_event(queue) is None
    suppressed = json.loads(suppressed_state.read_text())
    assert suppressed["elapsedSeconds"] == elapsed_s
    assert suppressed["reason"] == "vehicle_status_refresh"
    assert suppressed["refreshMarker"]["source"] == "refresh_action"


def test_vehicle_event_upload_sends_five_second_pair_without_refresh_marker(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  unlocked = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=4.98)).isoformat().replace("+00:00", "Z"),
  }
  locked = {
    **door_lock_event(True),
    "id": "lock-event",
    "occurredAt": now.isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(unlocked, queue)
  enqueue_vehicle_event(locked, queue)
  posted = []
  monkeypatch.setattr(
    "openpilot.system.wayon_cloud_uploader.post_json",
    lambda config, path, payload: posted.append((path, payload)),
  )

  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now,
    fetch_status_fn=lambda config: {}) == 2
  assert [payload["locked"] for _, payload in posted] == [False, True]
  assert peek_vehicle_event(queue) is None


def test_vehicle_event_upload_sends_pair_relocked_after_six_seconds(tmp_path, monkeypatch):
  queue = tmp_path / "vehicle_events.jsonl"
  now = NOW
  unlocked = {
    **door_lock_event(False),
    "id": "unlock-event",
    "occurredAt": (now - timedelta(seconds=6.1)).isoformat().replace("+00:00", "Z"),
  }
  locked = {
    **door_lock_event(True),
    "id": "lock-event",
    "occurredAt": now.isoformat().replace("+00:00", "Z"),
  }
  enqueue_vehicle_event(unlocked, queue)
  enqueue_vehicle_event(locked, queue)
  posted = []
  monkeypatch.setattr(
    "openpilot.system.wayon_cloud_uploader.post_json",
    lambda config, path, payload: posted.append((path, payload)),
  )

  assert upload_pending_vehicle_events(
    {"endpoint": "test", "token": "test"}, "device", queue, now=now) == 2
  assert [payload["locked"] for _, payload in posted] == [False, True]
  assert peek_vehicle_event(queue) is None


def test_impact_upload_captures_both_cameras_and_cleans_local_media(tmp_path, monkeypatch):
  queue = tmp_path / "impact_queue.jsonl"
  media_root = tmp_path / "impact_media"
  event = {
    "id": "impact-with-cameras",
    "detectedAt": "2026-07-19T00:00:00Z",
    "severity": "light",
    "captureRequested": True,
  }
  enqueue_impact_event(event, queue)
  posted = []
  operations = []

  def fake_post(config, path, payload):
    operations.append(path)
    posted.append((path, payload))
    return {"ok": True}

  image = np.zeros((8, 12, 3), dtype=np.uint8)
  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", fake_post)

  def capture():
    operations.append("capture")
    return image, image

  assert upload_pending_impacts(
    {"endpoint": "test", "token": "test"},
    "device",
    queue,
    media_root=media_root,
    capture_fn=capture,
    now=datetime(2026, 7, 19, tzinfo=timezone.utc),
  ) == 1
  assert operations == ["/api/impact", "capture", "/api/impact-media"]
  assert [path for path, _ in posted] == ["/api/impact", "/api/impact-media"]
  assert posted[1][1]["captureStatus"] == "complete"
  assert posted[1][1]["wideJpegBase64"]
  assert posted[1][1]["driverJpegBase64"]
  assert peek_impact_event(queue) is None
  assert not (media_root / "impact-with-cameras").exists()


def test_impact_media_retry_does_not_repost_impact(tmp_path, monkeypatch):
  queue = tmp_path / "impact_queue.jsonl"
  media_root = tmp_path / "impact_media"
  event = {
    "id": "impact-media-retry",
    "detectedAt": "2026-07-19T00:00:00Z",
    "severity": "light",
    "captureRequested": True,
  }
  enqueue_impact_event(event, queue)
  calls = []
  image = np.zeros((8, 12, 3), dtype=np.uint8)

  def failing_media(config, path, payload):
    calls.append(path)
    if path == "/api/impact-media":
      raise RuntimeError("temporary media failure")
    return {"ok": True}

  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", failing_media)
  now = datetime(2026, 7, 19, tzinfo=timezone.utc)
  try:
    upload_pending_impacts(
      {"endpoint": "test", "token": "test"}, "device", queue,
      media_root=media_root, capture_fn=lambda: (image, image), now=now,
    )
    assert False, "media upload should fail"
  except RuntimeError:
    pass

  queued = peek_impact_event(queue)
  assert queued["impactUploaded"] is True
  assert queued["uploadAttempts"] == 1
  assert queued["nextUploadAt"] == "2026-07-19T00:01:00Z"
  assert calls == ["/api/impact", "/api/impact-media"]

  assert upload_pending_impacts(
    {"endpoint": "test", "token": "test"}, "device", queue,
    media_root=media_root, capture_fn=lambda: (image, image), now=now,
  ) == 0
  assert calls == ["/api/impact", "/api/impact-media"]


def test_suppressed_impact_skips_capture_and_media(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  enqueue_impact_event({"id": "suppressed", "detectedAt": NOW.isoformat(), "captureRequested": True}, queue)
  calls = []

  def post(config, path, payload):
    calls.append(path)
    return {"ok": True, "suppressed": True}

  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", post)
  assert upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media",
                               capture_fn=lambda: pytest.fail("suppressed impact must not capture")) == 1
  assert calls == ["/api/impact"]
  assert peek_impact_event(queue) is None


def test_deferred_media_head_cannot_block_new_impact(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  enqueue_impact_event({"id": "old", "detectedAt": NOW.isoformat(), "impactUploaded": True,
                        "nextUploadAt": "2026-07-19T06:00:00Z"}, queue)
  enqueue_impact_event({"id": "new", "detectedAt": "2026-07-19T00:00:00Z"}, queue)
  calls = []
  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json",
                      lambda config, path, payload: calls.append((path, payload["id"])) or {"ok": True})
  assert upload_pending_impacts({}, "device", queue, limit=1, media_root=tmp_path / "media",
                               now=datetime(2026, 7, 19, tzinfo=timezone.utc)) == 1
  assert calls == [("/api/impact", "new")]
  assert [event["id"] for event in peek_impact_events(queue)] == ["old"]


def test_all_notifications_precede_camera_work(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  now = datetime(2026, 7, 19, tzinfo=timezone.utc)
  for event_id in ("first", "second"):
    enqueue_impact_event({"id": event_id, "detectedAt": now.isoformat(), "captureRequested": True}, queue)
  calls = []
  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json",
                      lambda config, path, payload: calls.append(path) or {"ok": True})

  def capture():
    calls.append("capture")
    return np.zeros((4, 4, 3), dtype=np.uint8), np.zeros((4, 4, 3), dtype=np.uint8)

  assert upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media", capture_fn=capture, now=now) == 2
  assert calls == ["/api/impact", "/api/impact", "capture", "/api/impact-media", "capture", "/api/impact-media"]


def test_failed_event_does_not_block_other_notifications(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  for event_id in ("bad", "good"):
    enqueue_impact_event({"id": event_id, "detectedAt": NOW.isoformat()}, queue)
  calls = []

  def post(config, path, payload):
    calls.append(payload["id"])
    if payload["id"] == "bad":
      raise RuntimeError("temporary failure")
    return {"ok": True}

  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", post)
  with pytest.raises(RuntimeError, match="temporary failure"):
    upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media")
  assert calls == ["bad", "good"]
  assert [event["id"] for event in peek_impact_events(queue)] == ["bad"]


def test_capture_exceptions_are_bounded_and_do_not_resend_notification(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  now = datetime(2026, 7, 19, tzinfo=timezone.utc)
  enqueue_impact_event({"id": "camera-fails", "detectedAt": now.isoformat(), "captureRequested": True}, queue)
  calls, captures = [], []
  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json",
                      lambda config, path, payload: calls.append((path, payload)) or {"ok": True})

  def capture():
    captures.append(True)
    raise RuntimeError("camera unavailable")

  for attempt, elapsed in enumerate((0, 60), start=1):
    with pytest.raises(RuntimeError, match="capture incomplete"):
      upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media", capture_fn=capture,
                             now=now + timedelta(seconds=elapsed))
    assert peek_impact_event(queue)["captureAttempts"] == attempt
    assert peek_impact_event(queue)["impactUploaded"] is True

  assert upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media", capture_fn=capture,
                               now=now + timedelta(seconds=360)) == 1
  assert len(captures) == 3
  assert [path for path, _ in calls] == ["/api/impact", "/api/impact-media"]
  assert calls[-1][1]["captureStatus"] == "failed"
  assert calls[-1][1]["captureAttempts"] == 3
  assert peek_impact_event(queue) is None


def test_old_backlog_is_saved_without_taking_misleading_current_photo(tmp_path, monkeypatch):
  queue = tmp_path / "queue.jsonl"
  # Older than the capture window, but still inside the queue's 24-hour TTL.
  enqueue_impact_event({"id": "old", "detectedAt": (NOW - timedelta(hours=1)).isoformat(), "captureRequested": True}, queue)
  calls = []
  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json",
                      lambda config, path, payload: calls.append((path, payload)) or {"ok": True, "stale": True})
  assert upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media",
                               capture_fn=lambda: pytest.fail("old impact must not take a current photo"),
                               now=datetime(2026, 7, 19, tzinfo=timezone.utc)) == 1
  assert calls[-1][1]["captureStatus"] == "failed"
  assert calls[-1][1]["captureAttempts"] == 0


@pytest.mark.parametrize("status,error,reset", [(404, "impact_not_found", True), (403, "unauthorized", False),
                                               (404, "other_error", False)])
def test_missing_legacy_record_revalidates_same_event_without_bypassing_auth(tmp_path, monkeypatch, status, error, reset):
  queue = tmp_path / "queue.jsonl"
  now = datetime(2026, 7, 19, tzinfo=timezone.utc)
  enqueue_impact_event({"id": "legacy", "detectedAt": (now - timedelta(hours=1)).isoformat(), "impactUploaded": True,
                        "captureRequested": True, "uploadAttempts": 12}, queue)
  response = requests.Response()
  response.status_code = status
  response._content = json.dumps({"error": error}).encode()

  def post(config, path, payload):
    assert path == "/api/impact-media"
    raise requests.HTTPError(response=response)

  monkeypatch.setattr("openpilot.system.wayon_cloud_uploader.post_json", post)
  with pytest.raises(requests.HTTPError):
    upload_pending_impacts({}, "device", queue, media_root=tmp_path / "media", now=now)
  event = peek_impact_event(queue)
  assert event["impactUploaded"] is (not reset)
  assert event["uploadAttempts"] == (1 if reset else 13)
  assert event["id"] == "legacy"


@pytest.fixture
def telemetry_sm():
  return FakeSubMaster(
    gps=log.GpsLocationData.new_message(hasFix=True, latitude=37.5, longitude=127.0,
                                        unixTimestampMillis=int(NOW.timestamp() * 1000)),
    deviceState=log.DeviceState.new_message(started=True, powerDrawW=8.5),
    pandaStates=[log.PandaState.new_message(pandaType="tres", voltage=12400, current=800,
                                          ignitionLine=True, controlsAllowed=True, spiChecksumErrorCount=7, harnessStatus="normal")],
    carState=car.CarState.new_message(vEgo=19.8, vEgoCluster=20.0, canValid=True, gearShifter="drive"),
    selfdriveState=log.SelfdriveState.new_message(state="enabled", enabled=True, active=True),
    radarState=log.RadarState.new_message(),
  )


def telemetry(sm, started=True):
  return uploader.telemetry_payload(sm, SimpleNamespace(get=lambda key: "test-device"), "test-device", started)


def test_actual_carrot_schema_telemetry(telemetry_sm):
  payload = telemetry(telemetry_sm)
  panda = payload["panda"]
  assert panda["spiErrorCount"] == panda["spiChecksumErrorCount"] == 7
  assert panda["controlsAllowed"] is True
  assert panda["controlsAllowedLateral"] is None
  assert panda["controlsAllowedLongitudinal"] is None
  assert payload["vehicle"]["available"] is True
  assert payload["openpilot"]["active"] is True
  assert payload["gps"]["fresh"] is True
  assert payload["vehicleSpeedMps"] == 20.0
  assert payload["connections"]["vehicleCan"]["state"] == "ok"
  assert payload["connections"]["panda"]["state"] == "ok"
  assert payload["voltageV"] == 12.4
  assert payload["vehicleInput"]["kind"] == "vehicleInputEstimate"
  json.dumps(payload, allow_nan=False)


@pytest.mark.parametrize("raw,expected", [(5027, 5.027), (12400, 12.4), (50, .05), (0, 0.0),
                                        (None, None), (-1, None), (float("nan"), None),
                                        (float("inf"), None), (True, None), ("bad", None)])
def test_voltage_conversion_is_explicit_millivolts(raw, expected):
  assert uploader.voltage_v_from_raw(raw) == expected


def test_bench_input_is_diagnostic_not_vehicle_battery(telemetry_sm):
  panda = telemetry_sm["pandaStates"][0]
  panda.pandaType = "cuatro"
  panda.voltage = 5027
  panda.harnessStatus = "notConnected"
  panda.ignitionLine = False
  payload = telemetry(telemetry_sm, started=False)
  assert payload["voltageV"] is None
  assert payload["deviceInput"]["source"] == "pandaStates"
  assert payload["deviceInput"]["fresh"] is True
  assert payload["deviceInput"]["voltageMv"] == 5027
  assert payload["deviceInput"]["voltageV"] == 5.027
  assert payload["panda"]["voltageV"] == 5.027
  assert payload["vehicleInput"]["qualified"] is False
  assert payload["vehicleInput"]["reason"] == "harness_not_connected"
  assert payload["currentMa"] == payload["deviceInput"]["currentMa"] == 800
  assert payload["powerW"] == payload["deviceInput"]["powerW"] == pytest.approx(4.0216)
  payload["connections"] = {}
  event, _ = uploader.health_event_from_telemetry(payload)
  assert event["severity"] == "normal"
  assert "battery" not in event["detail"]
  assert "vehicle input" not in event["detail"]


@pytest.mark.parametrize("harness", ["normal", "flipped"])
@pytest.mark.parametrize("started", [False, True])
@pytest.mark.parametrize("voltage,expected", [(5027, "critical"), (11400, "critical"),
                                             (11700, "warning"), (12400, "normal")])
def test_connected_vehicle_input_qualification_and_health(telemetry_sm, harness, started, voltage, expected):
  panda = telemetry_sm["pandaStates"][0]
  panda.harnessStatus = harness
  panda.voltage = voltage
  panda.ignitionLine = started
  payload = telemetry(telemetry_sm, started=started)
  assert payload["voltageV"] == voltage / 1000.0
  assert payload["vehicleInput"]["qualified"] is True
  assert payload["vehicleInput"]["source"] == "pandaStates.voltage"
  assert payload["vehicleInput"]["kind"] == "vehicleInputEstimate"
  payload["connections"] = {}
  event, _ = uploader.health_event_from_telemetry(payload)
  assert event["severity"] == expected
  assert "battery" not in event["detail"]
  if expected != "normal":
    assert event["detail"] == f"vehicle input estimate {voltage / 1000:.1f}V"
  assert event["snapshot"]["vehicleInput"] == payload["vehicleInput"]


@pytest.mark.parametrize("condition", ["stale_receive", "stale_publish", "invalid", "unseen", "missing", "unknown", "zero"])
def test_unavailable_panda_cannot_qualify_voltage(telemetry_sm, condition):
  panda = telemetry_sm["pandaStates"][0]
  panda.voltage = 11400
  if condition == "stale_receive":
    telemetry_sm.recv_time["pandaStates"] = 96
  elif condition == "stale_publish":
    telemetry_sm.logMonoTime["pandaStates"] = 96_000_000_000
  elif condition == "invalid":
    telemetry_sm.valid["pandaStates"] = False
  elif condition == "unseen":
    telemetry_sm.seen["pandaStates"] = False
  elif condition == "missing":
    telemetry_sm["pandaStates"] = []
  elif condition == "unknown":
    panda.pandaType = "unknown"
  else:
    panda.voltage = 0
  payload = telemetry(telemetry_sm)
  assert payload["voltageV"] is None
  assert payload["vehicleInput"]["qualified"] is False
  if condition not in ("unknown", "zero"):
    assert payload["deviceInput"]["fresh"] is False
    assert payload["deviceInput"]["voltageV"] is None
  payload["connections"] = {}
  event, _ = uploader.health_event_from_telemetry(payload)
  assert event["severity"] == "normal"


@pytest.mark.parametrize("vehicle_input", [{}, {"qualified": False, "fresh": True, "voltageV": 5.0},
                                          {"qualified": True, "fresh": False, "kind": "vehicleInputEstimate", "voltageV": 5.0}])
def test_health_never_classifies_unqualified_legacy_voltage(vehicle_input):
  event, _ = uploader.health_event_from_telemetry({"voltageV": 5.0, "vehicleInput": vehicle_input})
  assert event["severity"] == "normal"
  assert event["snapshot"]["voltageV"] is None


@pytest.mark.parametrize("field,value", [("seen", False), ("valid", False), ("recv_time", 96.9),
                                        ("recv_time", 101.0), ("logMonoTime", 96_900_000_000),
                                        ("logMonoTime", 101_000_000_000)])
def test_stale_invalid_or_future_sources_never_report_live(telemetry_sm, field, value):
  for service in ("carState", "selfdriveState", "pandaStates", "gpsLocation", "gpsLocationExternal", "radarState"):
    getattr(telemetry_sm, field)[service] = value
  payload = telemetry(telemetry_sm)
  assert payload["panda"] is None
  assert payload["voltageV"] is None
  assert payload["ignition"] is None
  assert payload["vehicleSpeedMps"] is None
  assert payload["vehicle"]["available"] is False
  assert payload["enabled"] is False
  assert payload["openpilot"]["active"] is False
  assert payload["gps"]["fresh"] is False
  assert payload["cutInRisk"]["available"] is False
  for name in ("panda", "vehicleCan", "openpilot", "gps"):
    assert payload["connections"][name]["state"] == "fault"


def test_offroad_never_reports_cached_engagement(telemetry_sm):
  telemetry_sm.valid["selfdriveState"] = False
  telemetry_sm.recv_time["carState"] = 0
  payload = telemetry(telemetry_sm, started=False)
  assert payload["enabled"] is False
  assert payload["openpilot"]["active"] is False
  assert payload["openpilot"]["reason"] == "offroad"
  assert payload["systemState"] == "offroad"
  for name in ("vehicleCan", "openpilot"):
    assert payload["connections"][name]["state"] == "inactive"
  payload["connections"] = {name: payload["connections"][name] for name in ("vehicleCan", "openpilot")}
  event, _ = uploader.health_event_from_telemetry(payload)
  assert event["severity"] == "normal"


@pytest.mark.parametrize("can_valid,can_timeout", [(False, False), (True, True)])
def test_fresh_but_invalid_can_is_fault(telemetry_sm, can_valid, can_timeout):
  telemetry_sm["carState"].canValid = can_valid
  telemetry_sm["carState"].canTimeout = can_timeout
  assert telemetry(telemetry_sm)["connections"]["vehicleCan"]["state"] == "fault"


def test_stale_device_values_are_unknown(telemetry_sm):
  telemetry_sm.recv_time["deviceState"] = 96
  payload = telemetry(telemetry_sm)
  assert payload["device"] is None
  assert payload["devicePowerW"] is None
  assert payload["connections"]["comma"]["state"] == "fault"


@pytest.mark.parametrize("timestamp", [0, -1, 1, int(NOW.timestamp() * 1000) + 16000])
def test_gps_epoch_must_be_current(telemetry_sm, timestamp):
  # Negative values cannot be encoded by the unsigned schema, so exercise
  # the same adapter with an invalid decoded/test input too.
  gps = SimpleNamespace(hasFix=True, latitude=37.5, longitude=127.0, unixTimestampMillis=timestamp)
  telemetry_sm["gpsLocation"] = telemetry_sm["gpsLocationExternal"] = gps
  assert telemetry(telemetry_sm)["gps"] == {"fresh": False, "source": "unavailable"}


@pytest.mark.parametrize("age,recent", [(1, True), (121, False), (-1, False)])
def test_last_gps_uses_memory_params_and_is_never_a_live_fix(tmp_path, age, recent):
  path = tmp_path / "LastGPSPosition"
  path.write_text(json.dumps({"latitude": 37.5, "longitude": 127.0, "bearing": 12.0}))
  os.utime(path, (NOW.timestamp() - age, NOW.timestamp() - age))
  persistent = SimpleNamespace(get=lambda key: pytest.fail("must not read persistent LastGPSPosition"))
  payload = gps_payload(FakeSubMaster(seen=False), persistent)
  assert payload["source"] == "lastGpsPosition"
  assert payload["timestampSource"] == "paramsMtime"
  assert payload["fresh"] is False
  assert payload["recent"] is recent


@pytest.mark.parametrize("value", [[], {"latitude": "invalid", "longitude": 127},
                                 {"latitude": 91, "longitude": 127}, {"latitude": float("nan"), "longitude": 127},
                                 {"latitude": 37, "longitude": 127, "bearing": float("inf")}])
def test_malformed_memory_gps_is_ignored(tmp_path, value):
  (tmp_path / "LastGPSPosition").write_text(json.dumps(value))
  assert gps_payload(FakeSubMaster(seen=False), None) == {"fresh": False, "source": "unavailable"}


@pytest.mark.parametrize("updated,connected,expected", [(99.5, True, "ok"), (91.9, True, "fault"),
                                                       (101, True, "fault"), (99, False, "fault"),
                                                       (float("nan"), True, "fault"),
                                                       ("2026-07-19T00:00:00Z", True, "fault")])
def test_actual_navdy_status_monotonic_contract(telemetry_sm, updated, connected, expected):
  uploader.NAVDY_HEALTH_PATH.write_text(json.dumps({"updatedAt": updated, "connected": connected, "transport": "bluetooth"}))
  connection = telemetry(telemetry_sm)["connections"]["navdy"]
  assert connection["state"] == expected
  assert connection["updatedAt"] is None
  if expected == "ok":
    assert connection["ageSeconds"] == .5
    assert connection["updatedAtMonotonic"] == 99.5
  json.dumps(connection, allow_nan=False)


def ambient_command(mode="manual"):
  zone = {"enabled": True, "rgb": [12, 34, 56], "brightness": 40}
  payload = {"schema": "wayon.ambient.command.v1", "mode": mode, "requestedAt": NOW.isoformat(),
             "durationSeconds": 900 if mode == "manual" else 0}
  if mode == "manual":
    payload.update(zone1=zone.copy(), zone2=zone.copy())
  if mode == "profile":
    payload["profile"] = {
      "enabled": True, "driving": {"zone1": zone.copy(), "zone2": zone.copy()},
      "onroadDoor": {"enabled": True, "zone1": zone.copy(), "zone2": zone.copy()},
      "offroadDoor": {"enabled": True, "zone1": zone.copy(), "zone2": zone.copy()},
      "exitCourtesy": {"enabled": True, "zone2": zone.copy(), "durationSeconds": 120},
      "overspeed": {"enabled": True, "zone1": zone.copy(), "brightnessCap": 50},
      "reverseOff": {"enabled": True}, "dataWatchdog": {"enabled": True, "timeoutSeconds": 20},
      "timing": {"fadeMilliseconds": 1000, "doorCloseDelaySeconds": 20, "doorMaxOnMinutes": 20,
                 "transitionUpdatesPerSecond": 30},
    }
  return {"id": "test-command", "deviceId": "test-device", "createdAt": NOW.isoformat(),
          "expiresAt": (NOW + timedelta(seconds=900 if mode == "manual" else 60)).isoformat(), "payload": payload}


@pytest.mark.parametrize("mode", ["manual", "auto", "profile"])
def test_valid_ambient_matches_actual_navdy_reader(tmp_path, mode):
  from openpilot.selfdrive.navdy.ambient import read_override
  path = tmp_path / "override"
  params = SimpleNamespace(put=lambda *args: None, get=lambda key: None)
  command = ambient_command(mode)
  assert uploader.apply_ambient_command(params, command, path, device_id="test-device")
  value = read_override((path,), now=NOW.timestamp())
  assert value["id"] == command["id"]
  assert value["mode"] == mode
  expired = read_override((path,), now=NOW.timestamp() + 901)
  assert (expired is not None) == (mode == "profile")


@pytest.mark.parametrize("mutation", [
  lambda c: c.update(deviceId="another-device"),
  lambda c: c.update(id=""),
  lambda c: c.update(expiresAt=NOW.isoformat()),
  lambda c: c.update(expiresAt="2026-07-19T01:00:00"),
  lambda c: c.update(expiresAt="bad"),
  lambda c: c["payload"].update(mode="invalid"),
  lambda c: c["payload"].update(schema="unknown"),
  lambda c: c["payload"].update(durationSeconds=1201),
  lambda c: c["payload"].update(zone1=None),
  lambda c: c["payload"]["zone1"].update(rgb=[256, 0, 0]),
  lambda c: c["payload"]["zone1"].update(brightness=float("nan")),
  lambda c: c["payload"]["zone1"].update(enabled="false"),
  lambda c: c["payload"].update(extra="x" * 16384),
])
def test_invalid_ambient_never_writes(tmp_path, mutation):
  command = ambient_command()
  mutation(command)
  path = tmp_path / "override"
  params = SimpleNamespace(put=lambda *args: pytest.fail("invalid command must not write"))
  assert not uploader.apply_ambient_command(params, command, path, device_id="test-device")
  assert not path.exists()


@pytest.mark.parametrize("mode", ["manual", "auto", "profile"])
def test_ambient_ack_is_params_acceptance_not_physical_application(monkeypatch, mode):
  command = ambient_command(mode)
  stored = {}
  params = SimpleNamespace(put=lambda key, value: stored.update({key: value}), get=stored.get)
  posts = []
  monkeypatch.setattr(uploader, "get_json", lambda *args: {"command": command})
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: posts.append((path, payload)))
  assert uploader.poll_ambient_command({}, params, "test-device")
  assert json.loads(stored["WayonAmbientOverride"])["id"] == command["id"]
  assert posts[0][0] == "/api/ambient/ack"
  assert posts[0][1]["stage"] == "comma_params"
  assert posts[0][1]["accepted"] is True
  assert posts[0][1]["applied"] is None
  assert posts[0][1]["navdyApplied"] is None


@pytest.mark.parametrize("status", ["pending", "delivered"])
@pytest.mark.parametrize("mode,mutation", [
  (mode, lambda c: c["payload"].update(schema="unknown")) for mode in ("manual", "auto", "profile")
] + [
  ("manual", lambda c: c.update(payload=None)),
  ("manual", lambda c: c["payload"].update(durationSeconds=1201)),
  ("manual", lambda c: c["payload"]["zone1"].update(rgb=[256, 0, 0])),
  ("manual", lambda c: c["payload"]["zone2"].update(brightness=float("nan"))),
  ("auto", lambda c: c["payload"].update(durationSeconds=1)),
  ("profile", lambda c: c["payload"]["profile"]["timing"].update(fadeMilliseconds=5001)),
  ("profile", lambda c: c["payload"].update(profile=None)),
  ("manual", lambda c: c["payload"].update(extra="sensitive-payload" * 16384)),
])
def test_invalid_ambient_ack_terminates_worker_pending_without_payload(monkeypatch, status, mode, mutation):
  command = ambient_command(mode)
  command["status"] = status
  command["payload"]["private"] = "sensitive-payload"
  mutation(command)
  posts = []

  def poll(*args):
    return {"command": command if command["status"] in ("pending", "delivered") else None}

  def ack(config, path, payload):
    posts.append((path, payload))
    # b7660d1d handleAmbientCommandAck: only applied === false means failed.
    command["status"] = "failed" if payload["applied"] is False else "acknowledged"
    return {"ok": True, "status": command["status"], "changes": 1}

  monkeypatch.setattr(uploader, "get_json", poll)
  monkeypatch.setattr(uploader, "post_json", ack)
  monkeypatch.setattr(uploader, "write_ambient_override", lambda *args: pytest.fail("invalid command must not write"))
  assert not uploader.poll_ambient_command({}, None, "test-device")
  assert command["status"] == "failed"
  assert posts == [("/api/ambient/ack", {
    "id": command["id"], "deviceId": "test-device", "accepted": False, "applied": False,
    "navdyApplied": None, "stage": "comma_validation", "reason": "invalid_ambient_command",
    "acknowledgedAt": uploader.utc_now(),
  })]
  serialized = json.dumps(posts[0][1])
  assert "sensitive-payload" not in serialized
  assert len(serialized.encode("utf-8")) < 512
  assert not uploader.poll_ambient_command({}, None, "test-device")
  assert len(posts) == 1


@pytest.mark.parametrize("command", [
  None, {}, [], "invalid",
  {"id": "", "deviceId": "test-device"},
  {"id": " ", "deviceId": "test-device"},
  {"id": 123, "deviceId": "test-device"},
  {"id": "x" * 129, "deviceId": "test-device"},
  {"id": "test-command"},
  {"id": "test-command", "deviceId": "another-device"},
])
def test_absent_or_unaddressable_ambient_command_never_acks(monkeypatch, command):
  monkeypatch.setattr(uploader, "get_json", lambda *args: {"command": command})
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("no identifiable own-device command to ACK"))
  monkeypatch.setattr(uploader, "write_ambient_override", lambda *args: pytest.fail("invalid command must not write"))
  assert not uploader.poll_ambient_command({}, None, "test-device")


def test_invalid_ambient_ack_failure_uses_delivery_backoff(monkeypatch, tmp_path):
  from openpilot.system.wayon_ambient_delivery import AmbientCommandDelivery

  command = ambient_command()
  command["payload"]["schema"] = "unknown"
  calls = []

  def fail_ack(config, path, payload):
    calls.append(payload)
    raise requests.Timeout("ACK unavailable")

  monkeypatch.setattr(uploader, "get_json", lambda *args: {"command": command})
  monkeypatch.setattr(uploader, "post_json", fail_ack)
  delivery = AmbientCommandDelivery(tmp_path / "wake")
  poll = lambda: uploader.poll_ambient_command({}, None, "test-device")
  for now, delay in [(0, 5), (5, 10), (15, 20), (35, 40), (75, 60), (135, 60)]:
    with pytest.raises(requests.Timeout):
      delivery.run(now, poll)
    assert delivery.next_retry == now + delay
    count = len(calls)
    assert not delivery.run(now + delay - 1, poll)
    assert len(calls) == count
    assert calls[-1]["accepted"] is False
    assert calls[-1]["applied"] is False
  monkeypatch.setattr(uploader, "post_json", lambda *args: {"ok": True, "status": "failed", "changes": 1})
  assert delivery.run(195, poll)
  assert delivery.failures == 0
  assert delivery.next_poll == 255


def test_failed_ambient_write_is_not_acknowledged(monkeypatch):
  monkeypatch.setattr(uploader, "get_json", lambda *args: {"command": ambient_command()})
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("failed write must not ACK"))
  def fail(*args):
    raise OSError("readback mismatch")
  monkeypatch.setattr(uploader, "write_ambient_override", fail)
  with pytest.raises(OSError):
    uploader.poll_ambient_command({}, None, "test-device")


def route_message(service, mono, **fields):
  message = log.Event.new_message(logMonoTime=int(mono * 1e9), valid=True)
  message.init(service)
  for key, value in fields.items():
    setattr(getattr(message, service), key, value)
  return message


def route_fixture(monkeypatch, anchor=None):
  epoch = int(NOW.timestamp() * 1000)
  messages = [route_message("carState", 10, vEgo=5.0)]
  if anchor in ("initData", "clocks"):
    messages.insert(0, route_message(anchor, 10, wallTimeNanos=epoch * 1_000_000))
  messages.extend(route_message("gpsLocation", mono, hasFix=True, latitude=37.5 + index * .001, longitude=127.0,
                                unixTimestampMillis=epoch + int((mono - 10) * 1000) if anchor == "gps" else 0)
                  for index, mono in enumerate((12, 25)))
  monkeypatch.setattr(uploader, "route_log_messages", lambda path, **kwargs: iter(messages))
  return {"route_name": "000001a3--c20ba54385", "segments": [(0, Path("segment"), Path("qlog"))],
          "mtime": NOW.timestamp() - 60}


@pytest.mark.parametrize("anchor", ["initData", "clocks", "gps"])
def test_counter_route_uses_recorded_epoch_not_upload_time(monkeypatch, anchor):
  group = route_fixture(monkeypatch, anchor)
  monkeypatch.setattr(uploader, "utc_now", lambda: pytest.fail("upload time is not route time"))
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  assert payload["startedAt"] == "2026-07-19T00:00:00Z"
  assert payload["endedAt"] == "2026-07-19T00:00:15Z"
  assert payload["route"][0]["time"] == "2026-07-19T00:00:02Z"
  assert payload["durationS"] == 15
  assert "_mono" not in payload["route"][0]


def test_undated_route_is_not_uploaded_or_marked_complete(monkeypatch):
  group = route_fixture(monkeypatch)
  assert uploader.route_started_at(group["route_name"]) is None
  assert uploader.parse_segment_dir_name(group["route_name"] + "--12") == (group["route_name"], 12)
  monkeypatch.setattr(uploader, "read_route_state", lambda: {})
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("undated route must not upload"))
  monkeypatch.setattr(uploader, "mark_route_uploaded", lambda *args: pytest.fail("must remain eligible for retry"))
  assert uploader.upload_recent_route_summary({}, "test-device") is False


def test_legacy_date_route_is_supported(monkeypatch):
  group = route_fixture(monkeypatch)
  group["route_name"] = "2026-07-19--00-00-00"
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  assert payload["startedAt"] == "2026-07-19T00:00:00Z"
  assert payload["timestampSource"] == "routeName"


@pytest.mark.parametrize("suffix", ["", ".bz2", ".zst"])
def test_bounded_reader_uses_real_carrot_messages(tmp_path, suffix):
  import bz2
  import zstandard as zstd
  messages = [route_message("initData", 10, wallTimeNanos=int(NOW.timestamp() * 1e9)),
              route_message("gpsLocation", 12, hasFix=True, latitude=37.5, longitude=127.0),
              route_message("gpsLocation", 25, hasFix=True, latitude=37.501, longitude=127.0)]
  raw = b"".join(message.to_bytes() for message in messages)
  encoded = bz2.compress(raw) if suffix == ".bz2" else zstd.compress(raw) if suffix == ".zst" else raw
  path = tmp_path / ("qlog" + suffix)
  path.write_bytes(encoded)
  group = {"route_name": "000001a3--c20ba54385", "segments": [(0, tmp_path, path)]}
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  assert payload["startedAt"] == "2026-07-19T00:00:00Z"
  assert payload["route"][1]["time"] == "2026-07-19T00:00:15Z"


def test_route_reader_aborts_on_transition_without_partial_summary(tmp_path, monkeypatch):
  raw = route_message("carState", 10, vEgo=5.0).to_bytes()
  path = tmp_path / "qlog"
  path.write_bytes(raw * 256)
  reader = uploader.route_log_messages(path)
  assert next(reader).which() == "carState"
  monkeypatch.setattr(uploader, "Params", lambda: SimpleNamespace(get_bool=lambda key: key == "IsOnroad"))
  with pytest.raises(RuntimeError, match="interrupted"):
    list(reader)
  group = {"route_name": "000001a3--c20ba54385", "segments": [(0, tmp_path, path)]}
  assert uploader.summarize_route_from_logs(group, {}, "test-device") is None


@pytest.mark.parametrize("available,allowed", [(256 * 1024 * 1024, True), (256 * 1024 * 1024 - 1, False)])
def test_route_memory_guard(monkeypatch, available, allowed):
  monkeypatch.setattr(uploader.psutil, "virtual_memory", lambda: SimpleNamespace(available=available))
  assert uploader.route_summary_allowed() is allowed
  assert not uploader.route_summary_allowed(SimpleNamespace(get_bool=lambda key: False))


def test_route_reader_rejects_large_files_and_rlog_fallback(tmp_path, monkeypatch):
  path = tmp_path / "qlog"
  path.write_bytes(b"x" * 9)
  monkeypatch.setattr(uploader, "ROUTE_LOG_MAX_BYTES", 8)
  with pytest.raises(uploader.ResourceLimitExceeded, match="compressed limit"):
    list(uploader.route_log_messages(path))
  (tmp_path / "rlog").write_bytes(b"data")
  assert uploader.segment_log_file(tmp_path) is None


@pytest.mark.parametrize("suffix", [".bz2", ".zst"])
def test_route_reader_rejects_decompression_expansion(tmp_path, monkeypatch, suffix):
  import bz2
  import zstandard as zstd
  path = tmp_path / ("qlog" + suffix)
  path.write_bytes(bz2.compress(b"x" * 10000) if suffix == ".bz2" else zstd.compress(b"x" * 10000))
  monkeypatch.setattr(uploader, "ROUTE_LOG_MAX_DECOMPRESSED_BYTES", 1024)
  with pytest.raises((uploader.ResourceLimitExceeded, zstd.ZstdError), match="decompressed limit|too much memory"):
    list(uploader.route_log_messages(path))


def test_route_upload_rechecks_offroad_before_post(monkeypatch):
  group = route_fixture(monkeypatch, "gps")
  monkeypatch.setattr(uploader, "read_route_state", dict)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "route_upload_signature", lambda *args, **kwargs: "signature")
  def summarize(*args, **kwargs):
    monkeypatch.setattr(uploader, "route_summary_allowed", lambda: False)
    return {"startedAt": NOW.isoformat()}
  monkeypatch.setattr(uploader, "summarize_route_from_logs", summarize)
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("onroad transition must abort upload"))
  assert uploader.upload_recent_route_summary({}, "test-device") is False


def analysis_route(tmp_path, name="00000001--abcdef", *, anchor="gps", incident=True, origin=100):
  directory = tmp_path / (name + "--0")
  directory.mkdir()
  epoch = int((NOW.timestamp() + origin - 100) * 1e9)
  messages = []
  if anchor in ("initData", "clocks"):
    messages.append(route_message(anchor, origin, wallTimeNanos=epoch))
  for index in range(20):
    mono = origin + index * 0.1
    messages.extend([
      route_message("selfdriveState", mono, active=True, enabled=True),
      route_message("carControl", mono, longActive=True, actuators={"accel": 1.0 if incident else 0.0}),
      route_message("longitudinalPlan", mono, aTarget=-0.5 if incident else 0.0),
      route_message("carState", mono, vEgo=10.0, aEgo=0.5 if incident else 0.0),
    ])
  messages.extend([
    route_message("gpsLocation", origin + 2, hasFix=True, latitude=37.5, longitude=127.0,
                  unixTimestampMillis=epoch // 1_000_000 + 2000 if anchor == "gps" else 0),
    route_message("carState", origin + 15, vEgo=10, aEgo=0),
    route_message("gpsLocation", origin + 15, hasFix=True, latitude=37.501, longitude=127.0),
  ])
  path = directory / "qlog"
  path.write_bytes(b"".join(message.to_bytes() for message in messages))
  return {"route_name": name, "segments": [(0, directory, path)], "mtime": NOW.timestamp() - 60}


@pytest.mark.parametrize("anchor", ["initData", "clocks", "gps"])
def test_full_analysis_and_real_blackbox_share_origin_and_worker_contract(tmp_path, monkeypatch, anchor):
  import io
  import zipfile

  group = analysis_route(tmp_path, anchor=anchor)
  config = {"blackbox_root": str(tmp_path / "blackbox")}
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  captures, trips = [], []

  # Deliberately use the actual API signature: no metadata keyword is accepted.
  def upload(config, device_id, kind, captured_at, content_type, camera_layout, payload, duration_s=None):
    assert (device_id, kind, content_type, camera_layout) == ("test-device", "clip", "application/zip", "road_blackbox")
    assert 0 < duration_s <= 60
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
      assert set(archive.namelist()) == {"manifest.json", "signals.jsonl.gz"}
      captures.append(json.loads(archive.read("manifest.json")))
    assert captured_at == captures[-1]["event"]["time"]
    return {"ok": True, "id": "capture-1"}

  def post(config, path, payload):
    assert path == "/api/trips"
    trips.append(payload)
    return {"ok": True, "id": payload["id"]}

  monkeypatch.setattr(uploader, "upload_live_capture", upload)
  monkeypatch.setattr(uploader, "post_json", post)
  assert uploader.upload_recent_route_summary(config, "test-device")
  analysis = trips[0]["analysis"]
  assert trips[0]["report"]["schemaVersion"] == "wayon-trip-report-v1"
  assert analysis["schemaVersion"] == "wayon-drive-report-v2"
  assert analysis["dataQuality"]["routeFirstMonoS"] == captures[0]["routeFirstMonoS"] == 100
  assert analysis["dataQuality"]["source"] == "sampled_qlog"
  assert analysis["dataQuality"]["timestampAvailable"] is True
  assert analysis["dataQuality"]["unexplainedBrakingAssessment"] == "unknown_missing_assist_context"
  event = analysis["blackbox"]["events"][0]
  assert event["type"] == "unexplained_acceleration"
  assert event["time"] == "2026-07-19T00:00:00.500Z"
  assert event["package"]["status"] == "uploaded"
  assert event["package"]["captureId"] == "capture-1"
  assert event["package"]["videoAvailable"] is False
  assert captures[0]["tripId"] == trips[0]["id"]
  assert captures[0]["cameraLayout"] == "road_blackbox"
  assert uploader.read_route_state()["route_uploads"][group["route_name"]]["complete"] is True
  assert not uploader.upload_recent_route_summary(config, "test-device")
  assert len(trips) == len(captures) == 1


def test_analysis_upgrade_backfills_recent_routes_in_bounded_cycles(tmp_path, monkeypatch):
  groups = [analysis_route(tmp_path, f"0000000{index}--abcdef", incident=False) for index in range(3)]
  uploader.write_route_state({"uploaded_routes": [group["route_name"] for group in groups]})
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: groups)
  monkeypatch.setattr(uploader, "build_blackbox_archive", lambda *args, **kwargs: pytest.fail("no event means no video work"))
  trips = []
  def post(config, path, payload):
    trips.append(payload)
    return {"ok": True, "id": payload["id"]}
  monkeypatch.setattr(uploader, "post_json", post)
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert len(trips) == 2
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert [trip["routeName"] for trip in trips] == [group["route_name"] for group in groups]
  assert not uploader.upload_recent_route_summary({}, "test-device")
  # Same route identity, but changed report revision must upgrade existing rows.
  monkeypatch.setattr(uploader, "ROUTE_UPLOAD_SCHEMA", "next-analysis-revision")
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert len(trips) == 5


def test_recent_backfill_skips_grace_period_and_retries_without_starving_history(tmp_path, monkeypatch):
  groups = [analysis_route(tmp_path, f"0000000{index}--abcdef", incident=False) for index in range(3)]
  groups[0]["mtime"] = NOW.timestamp()
  groups[1]["segments"][0][2].write_bytes(b"malformed")
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: groups)
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert [trip["routeName"] for trip in trips] == [groups[2]["route_name"]]
  state = uploader.read_route_state()
  assert groups[1]["route_name"] not in state["uploaded_routes"]
  assert state["route_uploads"][groups[1]["route_name"]]["complete"] is False
  assert state["route_uploads"][groups[1]["route_name"]]["retryAfter"] == NOW.timestamp() + 300


def test_route_signature_tracks_files_and_bounded_policy(tmp_path):
  group = analysis_route(tmp_path, incident=False)
  signature = uploader.route_upload_signature(group, {})
  assert signature != uploader.route_upload_signature(group, {"blackbox_auto_upload": False})
  assert signature != uploader.route_upload_signature(group, {"blackbox_upload_limit_per_route": 1})
  assert uploader.route_upload_signature(group, {}, device_id="one") != uploader.route_upload_signature(group, {}, device_id="two")
  path = group["segments"][0][2]
  os.utime(path, ns=(path.stat().st_atime_ns, path.stat().st_mtime_ns + 1))
  assert signature != uploader.route_upload_signature(group, {})
  assert uploader.blackbox_upload_limits({"blackbox_upload_limit_per_route": 1000, "blackbox_max_upload_bytes": 10**20}) == (4, 48 * 1024 * 1024)


def test_locked_latest_route_does_not_block_completed_history(tmp_path, monkeypatch):
  groups = [analysis_route(tmp_path, f"0000000{index}--abcdef", incident=False) for index in range(2)]
  (groups[0]["segments"][0][1] / "qlog.lock").touch()
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: groups)
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert [trip["routeName"] for trip in trips] == [groups[1]["route_name"]]
  assert uploader.read_route_state()["uploaded_routes"] == [groups[1]["route_name"]]


def test_confirmed_capture_reused_after_failed_trip_upload(tmp_path, monkeypatch):
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  captures, trips = [], []
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: captures.append(args) or {"ok": True, "id": "capture-1"})
  def fail_post(*args):
    raise requests.Timeout("private-response-not-for-report")
  monkeypatch.setattr(uploader, "post_json", fail_post)
  config = {"blackbox_root": str(tmp_path / "blackbox")}
  assert not uploader.upload_recent_route_summary(config, "test-device")
  state = uploader.read_route_state()
  entry = state["route_uploads"][group["route_name"]]
  assert entry["complete"] is False
  assert list(entry["packages"].values())[0]["captureId"] == "capture-1"
  assert not state.get("uploaded_routes")
  monkeypatch.setattr(uploader.time, "time", lambda: NOW.timestamp() + 301)
  monkeypatch.setattr(uploader, "build_blackbox_archive", lambda *args, **kwargs: pytest.fail("confirmed capture must be reused"))
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary(config, "test-device")
  assert len(captures) == len(trips) == 1
  assert trips[0]["analysis"]["blackbox"]["events"][0]["package"]["captureId"] == "capture-1"


@pytest.mark.parametrize("result", [None, {}, {"ok": True}, {"ok": False, "id": "bad"}, {"ok": True, "id": "x" * 129}])
def test_blackbox_unconfirmed_response_cannot_be_marked_uploaded(tmp_path, monkeypatch, result):
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: result)
  assert not uploader.prepare_blackbox_archives(group, payload, {"blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  package = payload["analysis"]["blackbox"]["events"][0]["package"]
  assert package["status"] == "error"
  assert "captureId" not in package


@pytest.mark.parametrize("kind", ["offroad", "memory", "deadline"])
@pytest.mark.parametrize("error_type", [uploader.WorkAborted, uploader.ResourceLimitExceeded])
def test_blackbox_interruption_never_posts_or_marks_route(tmp_path, monkeypatch, kind, error_type):
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  def abort(*args, **kwargs):
    if kind == "offroad":
      monkeypatch.setattr(uploader, "route_summary_allowed", lambda: False)
    elif kind == "memory":
      monkeypatch.setattr(uploader.psutil, "virtual_memory", lambda: SimpleNamespace(available=0))
    elif kind == "deadline":
      monkeypatch.setattr(uploader.time, "monotonic", lambda: 1000)
    assert not kwargs["allowed"]()
    raise error_type("work interrupted")
  monkeypatch.setattr(uploader, "build_blackbox_archive", abort)
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: pytest.fail("interrupted package must not upload"))
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("interrupted route must not upload"))
  assert not uploader.upload_recent_route_summary({}, "test-device")
  state = uploader.read_route_state()
  assert not state.get("uploaded_routes")
  assert state["route_uploads"][group["route_name"]]["complete"] is False


def test_blackbox_resource_limit_posts_analysis_once_without_retry_loop(tmp_path, monkeypatch):
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  builds, trips = [], []
  def fail(*args, **kwargs):
    builds.append(args[1]["id"])
    raise uploader.ResourceLimitExceeded("private-resource-details")
  monkeypatch.setattr(uploader, "build_blackbox_archive", fail)
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: pytest.fail("no package to upload"))
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  config = {"blackbox_root": str(tmp_path / "blackbox")}
  assert uploader.upload_recent_route_summary(config, "test-device")
  blackbox = trips[0]["analysis"]["blackbox"]
  assert blackbox["events"][0]["package"] == {"status": "unavailable", "reason": "resource_limit"}
  assert blackbox["unavailableEventCount"] == 1
  assert "private-resource-details" not in json.dumps(trips)
  assert uploader.read_route_state()["route_uploads"][group["route_name"]]["complete"] is True
  monkeypatch.setattr(uploader.time, "time", lambda: NOW.timestamp() + 301)
  assert not uploader.upload_recent_route_summary(config, "test-device")
  assert len(builds) == len(trips) == 1


def test_one_resource_limited_event_does_not_block_other_events(tmp_path, monkeypatch):
  import copy
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  events = payload["analysis"]["blackbox"]["events"]
  events.append({**copy.deepcopy(events[0]), "id": "second-event"})
  real_build = uploader.build_blackbox_archive
  def build(*args, **kwargs):
    if args[1]["id"] != "second-event":
      raise uploader.ResourceLimitExceeded("signal bytes")
    return real_build(*args, **kwargs)
  monkeypatch.setattr(uploader, "build_blackbox_archive", build)
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: {"ok": True, "id": "second-capture"})
  assert uploader.prepare_blackbox_archives(group, payload, {"blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  assert events[0]["package"]["status"] == "unavailable"
  assert events[1]["package"]["captureId"] == "second-capture"


def two_segment_route(tmp_path):
  group = analysis_route(tmp_path, incident=False)
  other = analysis_route(tmp_path, "00000002--abcdef", incident=False, origin=160)
  _, directory, path = other["segments"][0]
  group["segments"].append((1, directory, path))
  return group


@pytest.mark.parametrize("segment_count", [2, 61])
def test_long_route_byte_and_message_limits_reset_per_segment(tmp_path, monkeypatch, segment_count):
  from dataclasses import replace
  group = two_segment_route(tmp_path)
  for index in range(2, segment_count):
    other = analysis_route(tmp_path, f"segment-{index}", incident=False, origin=100 + index * 60)
    _, directory, path = other["segments"][0]
    group["segments"].append((index, directory, path))
  per_file_bytes = max(path.stat().st_size for _, _, path in group["segments"])
  monkeypatch.setattr(uploader, "ROUTE_WORK_LIMITS", replace(uploader.ROUTE_WORK_LIMITS,
                      max_total_decoded_bytes=per_file_bytes, max_messages=83))
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  quality = payload["analysis"]["dataQuality"]
  assert quality["partial"] is False
  assert quality["allListedSegmentsRead"] is True
  assert quality["readSegments"] == list(range(segment_count))
  assert quality["decodedBytes"] == sum(path.stat().st_size for _, _, path in group["segments"])
  assert quality["decodedBytes"] > per_file_bytes
  assert quality["messagesRead"] == 83 * segment_count
  assert quality["unreadSegments"] == []


@pytest.mark.parametrize("limit", ["time", "bytes", "messages"])
def test_route_local_limit_posts_explicit_partial_analysis(tmp_path, monkeypatch, limit):
  group = two_segment_route(tmp_path)
  original_reader = uploader.route_log_messages
  def reader(path, *, budget):
    if path == group["segments"][1][2]:
      if limit == "time":
        monkeypatch.setattr(uploader.time, "monotonic", lambda: 131)
        budget.check()
      raise uploader.ResourceLimitExceeded(limit)
    yield from original_reader(path, budget=budget)
  monkeypatch.setattr(uploader, "route_log_messages", reader)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({}, "test-device")
  quality = trips[0]["analysis"]["dataQuality"]
  assert quality["partial"] is trips[0]["routePartial"] is True
  assert quality["allListedSegmentsRead"] is False
  assert quality["readSegments"] == [0]
  assert quality["unreadSegments"] == [1]
  assert quality["partialReasons"] == ["resource_limit"]
  assert quality["observedEndAt"] == trips[0]["endedAt"] == "2026-07-19T00:00:15Z"
  assert trips[0]["report"]["dataQuality"] == {k: quality[k] for k in trips[0]["report"]["dataQuality"]}
  assert uploader.read_route_state()["route_uploads"][group["route_name"]]["complete"] is True


@pytest.mark.parametrize("guard", ["offroad", "memory", "outer_deadline"])
def test_route_partial_observations_never_override_outer_abort(tmp_path, monkeypatch, guard):
  group = two_segment_route(tmp_path)
  original_reader = uploader.route_log_messages
  def reader(path, *, budget):
    if path == group["segments"][1][2]:
      if guard == "offroad":
        monkeypatch.setattr(uploader, "route_summary_allowed", lambda: False)
      elif guard == "memory":
        monkeypatch.setattr(uploader.psutil, "virtual_memory", lambda: SimpleNamespace(available=0))
      else:
        monkeypatch.setattr(uploader.time, "monotonic", lambda: 191)
      raise uploader.ResourceLimitExceeded("limit concurrent with abort")
    yield from original_reader(path, budget=budget)
  monkeypatch.setattr(uploader, "route_log_messages", reader)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("partial result must not bypass outer guard"))
  assert not uploader.upload_recent_route_summary({}, "test-device")
  assert not uploader.read_route_state().get("uploaded_routes")


@pytest.mark.parametrize("change", ["rlog_deleted", "video_deleted", "schema_upgrade"])
def test_confirmed_capture_survives_signature_change_for_same_event(tmp_path, monkeypatch, change):
  group = analysis_route(tmp_path)
  directory = group["segments"][0][1]
  (directory / "rlog").write_bytes(group["segments"][0][2].read_bytes())
  (directory / "qcamera.ts").write_bytes(b"not-rendered-in-unit-test")
  from openpilot.system import wayon_event_blackbox as box
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: None)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  captures, trips = [], []
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: captures.append(args) or {"ok": True, "id": "retained-capture"})
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  config = {"blackbox_root": str(tmp_path / "blackbox")}
  assert uploader.upload_recent_route_summary(config, "test-device")
  if change == "schema_upgrade":
    monkeypatch.setattr(uploader, "ROUTE_UPLOAD_SCHEMA", "new-schema")
  else:
    (directory / ("rlog" if change == "rlog_deleted" else "qcamera.ts")).unlink()
  monkeypatch.setattr(uploader, "build_blackbox_archive", lambda *args, **kwargs: pytest.fail("same confirmed event must not be rebuilt"))
  assert uploader.upload_recent_route_summary(config, "test-device")
  assert len(captures) == 1
  assert len(trips) == 2
  assert trips[1]["analysis"]["blackbox"]["events"][0]["package"]["captureId"] == "retained-capture"


@pytest.mark.parametrize("coverage", ["partial", "missing"])
def test_confirmed_full_report_and_capture_ids_survive_coverage_downgrade(tmp_path, monkeypatch, coverage):
  import copy
  group = analysis_route(tmp_path)
  directory = group["segments"][0][1]
  rlog = directory / "rlog"
  rlog.write_bytes(group["segments"][0][2].read_bytes())
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: {"ok": True, "id": "confirmed-capture"})
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(copy.deepcopy(payload)) or {"ok": True, "id": payload["id"]})
  config = {"blackbox_root": str(tmp_path / "blackbox")}
  assert uploader.upload_recent_route_summary(config, "test-device")
  before = uploader.read_route_state()["route_uploads"][group["route_name"]]
  receipt = before["confirmedReport"]
  assert receipt["schema"] == uploader.ROUTE_RECEIPT_SCHEMA
  assert receipt["tripId"] == trips[0]["id"]
  assert receipt["signature"] == before["signature"]
  assert receipt["coverage"]["partial"] is False
  assert receipt["coverage"]["readSegments"] == [0]
  assert receipt["confirmedAt"] == uploader.utc_now()

  candidate = copy.deepcopy(trips[0])
  if coverage == "missing":
    candidate["analysis"].pop("dataQuality")
  else:
    candidate["analysis"]["dataQuality"].update(partial=True, allListedSegmentsRead=False, readSegments=[], unreadSegments=[0])
    candidate["analysis"]["blackbox"]["events"] = []
  rlog.unlink()  # Deleter changes the signature but must not erase the cloud report.
  monkeypatch.setattr(uploader, "summarize_route_from_logs", lambda *args, **kwargs: candidate)
  monkeypatch.setattr(uploader, "prepare_blackbox_archives", lambda *args, **kwargs: pytest.fail("reject before changing capture checkpoints"))
  assert not uploader.upload_recent_route_summary(config, "test-device")
  after = uploader.read_route_state()["route_uploads"][group["route_name"]]
  assert len(trips) == 1
  assert after["confirmedReport"] == receipt
  assert after["packages"] == before["packages"]
  assert after["signature"] != receipt["signature"]
  assert after["skipReason"] == "coverage_regression"
  monkeypatch.setattr(uploader, "summarize_route_from_logs", lambda *args, **kwargs: pytest.fail("same rejected signature must not loop"))
  assert not uploader.upload_recent_route_summary(config, "test-device")


@pytest.mark.parametrize("previous_partial,change,rejected", [
  (False, "same", False),
  (False, "partial", True),
  (False, "fewer_segments", True),
  (False, "shorter_end", True),
  (False, "later_start", True),
  (False, "unknown", True),
  (True, "same", False),
  (True, "full_upgrade", False),
  (True, "shorter_end", True),
  (True, "different_segments", True),
  (True, "unknown", True),
])
def test_confirmed_coverage_can_only_be_preserved_or_extended(previous_partial, change, rejected):
  import copy
  quality = {"partial": previous_partial, "allListedSegmentsRead": not previous_partial,
             "readSegments": [0, 1], "unreadSegments": [2] if previous_partial else [], "missingSegments": [],
             "observedStartAt": "2026-07-19T00:00:00Z", "observedEndAt": "2026-07-19T00:02:00Z"}
  payload = {"id": "device-route", "analysis": {"schemaVersion": "wayon-drive-report-v2", "dataQuality": quality}}
  receipt = uploader.route_upload_receipt(payload, "confirmed-signature")
  candidate = copy.deepcopy(payload)
  current = candidate["analysis"]["dataQuality"]
  if change == "partial":
    current.update(partial=True, allListedSegmentsRead=False, readSegments=[0], unreadSegments=[1])
  elif change == "fewer_segments":
    current["readSegments"] = [0]
  elif change == "shorter_end":
    current["observedEndAt"] = "2026-07-19T00:01:00Z"
  elif change == "later_start":
    current["observedStartAt"] = "2026-07-19T00:00:01Z"
  elif change == "unknown":
    candidate["analysis"].pop("dataQuality")
  elif change == "full_upgrade":
    current.update(partial=False, allListedSegmentsRead=True, readSegments=[0, 1, 2], unreadSegments=[])
    current["observedEndAt"] = "2026-07-19T00:03:00Z"
  elif change == "different_segments":
    current.update(readSegments=[0, 2], unreadSegments=[1])
  assert uploader.route_report_would_lose_coverage(receipt, candidate) is rejected
  # Coverage receipts apply only to the confirmed trip, not another device.
  candidate["id"] = "another-device-route"
  assert not uploader.route_report_would_lose_coverage(receipt, candidate)


@pytest.mark.parametrize("old_receipt", [None, {}, {"coverage": {"partial": False}},
                                        {"schema": "wayon-route-upload-receipt-v1", "tripId": "test-device-00000001--abcdef", "coverage": None}])
def test_legacy_unknown_report_receipt_allows_partial_analysis_upgrade(tmp_path, monkeypatch, old_receipt):
  group = two_segment_route(tmp_path)
  original_reader = uploader.route_log_messages
  def reader(path, *, budget):
    if path == group["segments"][1][2]:
      raise uploader.ResourceLimitExceeded("partial source")
    yield from original_reader(path, budget=budget)
  monkeypatch.setattr(uploader, "route_log_messages", reader)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  uploader.write_route_state({"uploaded_routes": [group["route_name"]], "route_uploads": {
    group["route_name"]: {"signature": "legacy-signature", "complete": True, "confirmedReport": old_receipt}}})
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({}, "test-device")
  assert len(trips) == 1
  receipt = uploader.read_route_state()["route_uploads"][group["route_name"]]["confirmedReport"]
  assert receipt["coverage"]["partial"] is True
  assert receipt["coverage"]["readSegments"] == [0]
  assert receipt["coverage"]["unreadSegments"] == [1]


def test_failed_reupload_cannot_replace_confirmed_coverage_receipt(tmp_path, monkeypatch):
  group = analysis_route(tmp_path, incident=False)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({}, "test-device")
  receipt = uploader.read_route_state()["route_uploads"][group["route_name"]]["confirmedReport"]
  monkeypatch.setattr(uploader, "ROUTE_UPLOAD_SCHEMA", "next-schema")
  def fail(*args):
    raise requests.Timeout("unconfirmed")
  monkeypatch.setattr(uploader, "post_json", fail)
  assert not uploader.upload_recent_route_summary({}, "test-device")
  after = uploader.read_route_state()["route_uploads"][group["route_name"]]
  assert after["confirmedReport"] == receipt
  assert after["complete"] is False


def test_confirmed_trip_receipt_survives_immediate_offroad_guard_loss(tmp_path, monkeypatch):
  group = analysis_route(tmp_path, incident=False)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  def post(config, path, payload):
    monkeypatch.setattr(uploader, "route_summary_allowed", lambda: False)
    return {"ok": True, "id": payload["id"]}
  monkeypatch.setattr(uploader, "post_json", post)
  assert not uploader.upload_recent_route_summary({}, "test-device")
  entry = uploader.read_route_state()["route_uploads"][group["route_name"]]
  assert entry["complete"] is False
  assert entry["confirmedReport"]["coverage"]["partial"] is False
  assert entry["confirmedReport"]["signature"] == entry["signature"]


@pytest.mark.parametrize("change", ["event", "window", "device"])
def test_changed_event_identity_does_not_reuse_confirmed_capture(tmp_path, monkeypatch, change):
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  event = payload["analysis"]["blackbox"]["events"][0]
  packages = {event["id"]: {"status": "uploaded", "captureId": "old-capture",
                            "archiveName": uploader.blackbox_archive_name(payload["id"], event)}}
  if change == "event":
    event["id"] = "new-event"
  elif change == "window":
    event["captureWindow"]["endOffsetS"] -= 1
  else:
    payload["id"] = "other-device-trip"
  uploads = []
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: uploads.append(args) or {"ok": True, "id": "new-capture"})
  assert uploader.prepare_blackbox_archives(group, payload, {"blackbox_root": str(tmp_path / "blackbox")}, "test-device", packages=packages)
  assert len(uploads) == 1
  assert event["package"]["captureId"] == "new-capture"


def test_transition_during_capture_checkpoints_id_but_not_trip_completion(tmp_path, monkeypatch):
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  def upload(*args):
    monkeypatch.setattr(uploader, "route_summary_allowed", lambda: False)
    return {"ok": True, "id": "confirmed-before-transition"}
  monkeypatch.setattr(uploader, "upload_live_capture", upload)
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("no trip post after transition"))
  assert not uploader.upload_recent_route_summary({"blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  entry = uploader.read_route_state()["route_uploads"][group["route_name"]]
  assert not entry["complete"]
  assert list(entry["packages"].values())[0]["captureId"] == "confirmed-before-transition"


@pytest.mark.parametrize("policy,reason", [({"blackbox_auto_upload": False}, "auto_upload_disabled"),
                                          ({"blackbox_upload_limit_per_route": 0}, "upload_limit")])
def test_blackbox_disabled_or_zero_limit_does_not_build_raw_video(tmp_path, monkeypatch, policy, reason):
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  monkeypatch.setattr(uploader, "build_blackbox_archive", lambda *args, **kwargs: pytest.fail("policy must stop before video work"))
  assert uploader.prepare_blackbox_archives(group, payload, {**policy, "blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  assert payload["analysis"]["blackbox"]["events"][0]["package"] == {"status": "pending", "reason": reason}


def test_blackbox_build_and_upload_count_are_limited_before_work(tmp_path, monkeypatch):
  import copy
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  template = payload["analysis"]["blackbox"]["events"][0]
  events = [{**copy.deepcopy(template), "id": f"event-{index}"} for index in range(6)]
  payload["analysis"]["blackbox"]["events"] = events
  builds, uploads = [], []
  real_build = uploader.build_blackbox_archive
  def build(*args, **kwargs):
    builds.append(args[1]["id"])
    return real_build(*args, **kwargs)
  monkeypatch.setattr(uploader, "build_blackbox_archive", build)
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: uploads.append(args) or {"ok": True, "id": f"capture-{len(uploads)}"})
  assert uploader.prepare_blackbox_archives(group, payload, {"blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  assert len(builds) == len(uploads) == 4
  assert all(event["package"] == {"status": "pending", "reason": "upload_limit"} for event in events[4:])


def test_oversize_blackbox_is_not_read_or_uploaded(tmp_path, monkeypatch):
  group = analysis_route(tmp_path)
  payload = uploader.summarize_route_from_logs(group, {}, "test-device")
  monkeypatch.setattr(uploader, "upload_live_capture", lambda *args: pytest.fail("oversized archive must not upload"))
  assert uploader.prepare_blackbox_archives(group, payload, {"blackbox_root": str(tmp_path / "blackbox"), "blackbox_max_upload_bytes": 1}, "test-device")
  assert payload["analysis"]["blackbox"]["events"][0]["package"]["reason"] == "archive_too_large"


def test_capture_failure_is_retryable_and_report_contains_no_exception_payload(tmp_path, monkeypatch):
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  def fail(*args):
    raise requests.Timeout("secret-in-transport-error")
  monkeypatch.setattr(uploader, "upload_live_capture", fail)
  trips = []
  monkeypatch.setattr(uploader, "post_json", lambda config, path, payload: trips.append(payload) or {"ok": True, "id": payload["id"]})
  assert uploader.upload_recent_route_summary({"blackbox_root": str(tmp_path / "blackbox")}, "test-device")
  package = trips[0]["analysis"]["blackbox"]["events"][0]["package"]
  assert package["status"] == "error"
  assert package["reason"] == "Timeout"
  assert "captureId" not in package
  assert "secret-in-transport-error" not in json.dumps(trips)
  assert uploader.read_route_state()["route_uploads"][group["route_name"]]["complete"] is False


def test_changing_route_is_not_posted_as_completed_snapshot(tmp_path, monkeypatch):
  group = analysis_route(tmp_path, incident=False)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  def prepare(*args, **kwargs):
    path = group["segments"][0][2]
    os.utime(path, ns=(path.stat().st_atime_ns, path.stat().st_mtime_ns + 1))
    return True
  monkeypatch.setattr(uploader, "prepare_blackbox_archives", prepare)
  monkeypatch.setattr(uploader, "post_json", lambda *args: pytest.fail("changing route must not post"))
  assert not uploader.upload_recent_route_summary({}, "test-device")
  assert not uploader.read_route_state().get("uploaded_routes")


@pytest.mark.parametrize("result", [{}, {"ok": True, "id": "wrong-trip"}, {"ok": False}])
def test_trip_completion_needs_confirmed_matching_response(tmp_path, monkeypatch, result):
  group = analysis_route(tmp_path, incident=False)
  monkeypatch.setattr(uploader, "recent_route_groups", lambda age: [group])
  monkeypatch.setattr(uploader, "post_json", lambda *args: result)
  assert not uploader.upload_recent_route_summary({}, "test-device")
  assert not uploader.read_route_state().get("uploaded_routes")


@pytest.mark.parametrize("segments", [[1], [0, 0], [0, 10000000], [0, 2, 1]])
def test_unknown_origin_or_unbounded_segment_indices_fail_closed(tmp_path, monkeypatch, segments):
  group = analysis_route(tmp_path)
  _, directory, path = group["segments"][0]
  group["segments"] = [(index, directory, path) for index in segments]
  monkeypatch.setattr(uploader, "route_log_messages", lambda *args, **kwargs: pytest.fail("must reject before reading"))
  assert uploader.summarize_route_from_logs(group, {}, "test-device") is None


def test_route_work_budget_and_recording_lock_are_enforced(tmp_path, monkeypatch):
  from dataclasses import replace
  group = analysis_route(tmp_path)
  monkeypatch.setattr(uploader, "ROUTE_WORK_LIMITS", replace(uploader.ROUTE_WORK_LIMITS, max_messages=2))
  assert uploader.summarize_route_from_logs(group, {}, "test-device") is None
  (group["segments"][0][1] / "qlog.lock").touch()
  with pytest.raises(uploader.WorkAborted):
    uploader.route_upload_signature(group, {})


@pytest.mark.parametrize("identity", [None, [], {"endpoint": "https://test.invalid", "token": "wayon_test", "enabled": False},
                                      {"endpoint": "http://test.invalid", "token": "wayon_test"},
                                      {"endpoint": "https://test.invalid", "token": "legacy-token"},
                                      {"endpoint": "https://test.invalid", "token": "wayon_"},
                                      {"endpoint": "https://", "token": "wayon_test"}])
def test_config_rejection_cannot_fall_back_to_raw_file(tmp_path, monkeypatch, identity):
  path = tmp_path / "config.json"
  path.write_text(json.dumps({"endpoint": "https://test.invalid", "token": "wayon_test"}))
  monkeypatch.setattr(uploader, "CONFIG_PATH", path)
  monkeypatch.setattr(uploader, "ensure_wayon_identity", lambda path: identity)
  assert uploader.read_config() is None


def test_config_accepts_only_validated_https_identity(monkeypatch):
  identity = {"endpoint": "https://test.invalid/", "token": "wayon_test", "enabled": True, "device_id": "test-device"}
  monkeypatch.setattr(uploader, "ensure_wayon_identity", lambda path: identity)
  assert uploader.read_config() == {**identity, "endpoint": "https://test.invalid"}


def test_config_exception_does_not_fall_back_or_expose_secret(monkeypatch, capsys):
  def fail(path):
    raise ValueError("private-test-token")
  monkeypatch.setattr(uploader, "ensure_wayon_identity", fail)
  assert uploader.read_config() is None
  assert "private-test-token" not in capsys.readouterr().out
