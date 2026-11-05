import json
from datetime import datetime, timedelta, UTC
from pathlib import Path

import pytest

from openpilot.system import wayon_vehicle_events
from openpilot.system.wayon_impact import (
  IMPACT_EVENT_TTL_S,
  MAX_QUEUED_IMPACTS,
  DoorLockTracker,
  ImpactDetector,
  append_impact_diagnostic,
  enqueue_impact_event,
  peek_impact_event,
  peek_impact_events,
  remove_impact_event,
  update_impact_event,
)

GRAVITY = (0.0, 0.0, 9.80665)
STILL_GYRO = (0.0, 0.0, 0.0)
NOW = datetime(2026, 10, 28, 12, tzinfo=UTC)
DETECTED_AT = NOW.isoformat()


@pytest.fixture(autouse=True)
def frozen_queue_clock(monkeypatch):
  class Clock(datetime):
    @classmethod
    def now(cls, tz=None):
      return NOW

  monkeypatch.setattr(wayon_vehicle_events, "datetime", Clock)


def queued_impact(event_id, **fields):
  return {"id": event_id, "detectedAt": DETECTED_AT, **fields}


def prime(detector: ImpactDetector, start: float = 0.0) -> float:
  now = start
  for _ in range(50):
    detector.update(GRAVITY, STILL_GYRO, now)
    now += 0.01
  return now


def test_detector_ignores_motion_before_arm_delay():
  detector = ImpactDetector(arm_delay_s=10.0, warmup_s=0.0)
  detector.update(GRAVITY, STILL_GYRO, 0.0)

  assert detector.update((8.0, 0.0, 9.80665), STILL_GYRO, 1.0) is None
  assert not detector.armed


def test_detector_ignores_small_vibration():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  for index in range(100):
    vibration = 0.1 if index % 2 else -0.1
    assert detector.update((vibration, 0.0, 9.80665), STILL_GYRO, now) is None
    now += 0.01


def test_detector_ignores_measured_offroad_noise():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  for index in range(200):
    noise = 0.02 if index % 2 else -0.02
    assert detector.update((noise, 0.0, 9.80665), (0.0, 0.01, 0.0), now) is None
    now += 0.01

  assert detector.last_dynamic_g < 0.01


def test_detector_confirms_two_sample_impact():
  detector = ImpactDetector(
    arm_delay_s=0.0,
    warmup_s=0.0,
    impulse_dynamic_g=2.0,
    strong_dynamic_g=2.0,
  )
  now = prime(detector)

  assert detector.update((4.5, 0.0, 9.80665), (0.0, 0.2, 0.0), now) is None
  event = detector.update((5.0, 0.0, 9.80665), (0.0, 0.3, 0.0), now + 0.01)

  assert event is not None
  assert event["severity"] == "light"
  assert event["peakDynamicG"] >= 0.45
  assert event["sampleCount"] == 2


def test_detector_catches_light_parking_impact_at_new_threshold():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  event = detector.update((2.05, 0.0, 9.80665), STILL_GYRO, now)

  assert event is not None
  assert 0.20 <= event["peakDynamicG"] < 0.30


def test_detector_catches_measured_single_sample_parking_impact():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  # The parked comma measured 0.0663 g and 11.49 g/s during a deliberate body impact.
  assert detector.update((-0.48, 0.0, 9.80665), STILL_GYRO, now) is None
  event = detector.update((0.65, 0.0, 9.80665), STILL_GYRO, now + 0.01)

  assert event is not None
  assert event["severity"] == "light"
  assert 0.06 <= event["peakDynamicG"] < 0.07
  assert event["peakJerkGPerSec"] >= 8.0


def test_detector_catches_small_two_sample_body_shake():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  assert detector.update((0.20, 0.0, 9.80665), STILL_GYRO, now) is None
  event = detector.update((0.38, 0.0, 9.80665), STILL_GYRO, now + 0.01)

  assert event is not None
  assert event["severity"] == "light"
  assert 0.03 <= event["peakDynamicG"] < 0.04


def test_detector_rejects_slow_body_motion_at_similar_dynamic_g():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  assert detector.update((0.45, 0.0, 9.80665), STILL_GYRO, now) is None
  assert detector.update((0.55, 0.0, 9.80665), STILL_GYRO, now + 0.1) is None


def test_detector_suppresses_impact_while_detection_disabled():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0)
  now = prime(detector)

  assert detector.update((2.5, 0.0, 9.80665), STILL_GYRO, now, detection_enabled=False) is None
  assert detector.update((2.6, 0.0, 9.80665), STILL_GYRO, now + 0.01, detection_enabled=False) is None


def test_door_lock_tracker_requires_locked_state_and_persists(tmp_path: Path):
  state_path = tmp_path / "door_lock_status.json"
  tracker = DoorLockTracker(required=True, arm_delay_s=3.0, state_path=state_path)

  assert not tracker.detection_allowed(10.0)
  assert tracker.update(0, 0x130, bytes.fromhex("020000"), 10.0)
  assert tracker.locked is False
  assert not tracker.detection_allowed(20.0)

  assert tracker.update(0, 0x130, bytes.fromhex("040000"), 20.0)
  assert tracker.locked is True
  assert not tracker.update(0, 0x130, bytes.fromhex("010000"), 21.0)
  assert tracker.locked is True
  assert not tracker.detection_allowed(22.9)
  assert tracker.detection_allowed(23.0)

  restored = DoorLockTracker(required=True, arm_delay_s=3.0, state_path=state_path)
  assert restored.locked is None
  assert not restored.detection_allowed(0.0)
  assert restored.update(0, 0x130, bytes.fromhex("040000"), 1.0)
  assert not restored.detection_allowed(3.9)
  assert restored.detection_allowed(4.0)


def test_door_lock_tracker_ignores_other_can_frames(tmp_path: Path):
  tracker = DoorLockTracker(required=True, state_path=tmp_path / "state.json")

  assert not tracker.update(1, 0x130, bytes(8), 0.0)
  assert not tracker.update(0, 0x123, bytes(8), 0.0)
  assert not tracker.update(0, 0x130, bytes(0), 0.0)
  assert tracker.locked is None


def test_update_impact_event_preserves_queue_order(tmp_path: Path):
  queue = tmp_path / "impact_queue.jsonl"
  enqueue_impact_event(queued_impact("first", severity="light"), queue)
  enqueue_impact_event(queued_impact("second", severity="moderate"), queue)

  updated = update_impact_event("first", {"captureAttempts": 1}, queue)

  assert updated == queued_impact("first", severity="light", captureAttempts=1)
  assert peek_impact_event(queue) == updated
  assert remove_impact_event("first", queue)
  assert peek_impact_event(queue) == queued_impact("second", severity="moderate")


def test_detector_uses_cooldown_after_strong_impact():
  detector = ImpactDetector(arm_delay_s=0.0, warmup_s=0.0, cooldown_s=30.0)
  now = prime(detector)

  first = detector.update((10.0, 0.0, 9.80665), STILL_GYRO, now)
  second = detector.update((-10.0, 0.0, 9.80665), STILL_GYRO, now + 1.0)

  assert first is not None
  assert second is None


def test_impact_queue_round_trip(tmp_path: Path):
  queue = tmp_path / "impact_queue.jsonl"
  first = queued_impact("first", peakDynamicG=0.5)
  second = queued_impact("second", peakDynamicG=0.8)

  enqueue_impact_event(first, queue)
  enqueue_impact_event(second, queue)
  assert peek_impact_event(queue) == first
  assert remove_impact_event("first", queue)
  assert peek_impact_event(queue) == second
  assert remove_impact_event("second", queue)
  assert peek_impact_event(queue) is None


def test_impact_queue_snapshot_preserves_order_and_empty_queue(tmp_path: Path):
  queue = tmp_path / "impact_queue.jsonl"
  assert peek_impact_events(queue) == []
  first, second = queued_impact("first"), queued_impact("second")
  enqueue_impact_event(first, queue)
  enqueue_impact_event(second, queue)
  assert peek_impact_events(queue) == [first, second]
  assert peek_impact_event(queue) == first


def test_impact_diagnostics_append_and_rotate(tmp_path: Path):
  diagnostics = tmp_path / "impact_diagnostics.jsonl"
  for index in range(100):
    append_impact_diagnostic({"index": index, "peakDynamicG": index / 100.0}, diagnostics, max_bytes=1024)

  lines = diagnostics.read_text().splitlines()
  assert lines
  assert '"index":99' in lines[-1]
  assert diagnostics.stat().st_size <= 1024


@pytest.mark.parametrize("required", [False, True])
def test_persisted_lock_never_initializes_live_state(tmp_path, required):
  state = tmp_path / "state.json"
  state.write_text(json.dumps({"locked": True, "observedAt": DETECTED_AT}))
  tracker = DoorLockTracker(required=required, state_path=state)
  assert tracker.locked is None
  assert tracker.locked_since is None
  assert tracker.last_sample_at is None


def test_lock_state_expires_and_needs_new_settling_period(tmp_path):
  tracker = DoorLockTracker(required=True, state_path=tmp_path / "state", max_age_s=5)
  tracker.update(0, 0x130, b"\x04", 10)
  assert tracker.detection_allowed(14)
  assert not tracker.detection_allowed(16)
  assert tracker.locked is None
  assert json.loads(tracker.state_path.read_text())["locked"] is None
  tracker.update(0, 0x130, b"\x04", 17)
  assert not tracker.detection_allowed(19)
  assert tracker.detection_allowed(20)


def test_repeated_lock_frames_refresh_freshness_without_resetting_settling(tmp_path):
  tracker = DoorLockTracker(required=True, state_path=tmp_path / "state", max_age_s=2)
  assert tracker.update(0, 0x130, b"\x04", 10)
  for now in (11, 12, 13, 14):
    assert not tracker.update(0, 0x130, b"\x04", now)
  assert tracker.locked_since == 10
  assert tracker.last_sample_at == 14
  assert tracker.detection_allowed(15)
  assert not tracker.update(0, 0x130, b"\x02", 13)
  assert tracker.locked is True


@pytest.mark.parametrize("sample_time", [0.49, 0.2, 1.0, float("nan"), float("inf")])
def test_detector_rejects_duplicate_reordered_gapped_or_invalid_time(sample_time):
  detector = ImpactDetector(arm_delay_s=0, warmup_s=0)
  prime(detector)
  # Use the exact last timestamp for the duplicate case (the prime loop rounds).
  if sample_time == 0.49:
    sample_time = detector.last_sample_at
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, sample_time) is None
  assert not detector.armed


def test_detector_does_not_confirm_expired_candidate():
  detector = ImpactDetector(arm_delay_s=0, warmup_s=0, impulse_dynamic_g=2, strong_dynamic_g=2)
  now = prime(detector)
  assert detector.update((0.3, 0, 9.80665), STILL_GYRO, now) is None
  assert detector.candidate_hits == 1
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, now + 0.19) is None
  assert detector.candidate_hits == 1


def test_detector_reset_discards_candidate_but_preserves_cooldown():
  detector = ImpactDetector(arm_delay_s=0, warmup_s=0, cooldown_s=30)
  now = prime(detector)
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, now)
  detector.reset()
  assert detector.first_sample_at is None
  assert detector.gravity is None
  now = prime(detector, now + 1)
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, now) is None


def test_detector_disabled_interval_requires_new_gravity_and_warmup():
  detector = ImpactDetector(arm_delay_s=0.2, warmup_s=0)
  now = prime(detector)
  assert detector.armed
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, now, detection_enabled=False) is None
  assert detector.gravity is None
  assert not detector.armed
  assert detector.update((4.0, 0, 9.80665), STILL_GYRO, now + 0.01) is None
  assert detector.gravity == [4.0, 0, 9.80665]
  assert not detector.armed


def test_impact_queue_ttl_and_missing_or_invalid_timestamps(tmp_path):
  queue = tmp_path / "queue"
  for event in (
    queued_impact("expired", detectedAt=(NOW - timedelta(seconds=IMPACT_EVENT_TTL_S + 1)).isoformat()),
    {"id": "undated"}, queued_impact("invalid", detectedAt="invalid"),
    queued_impact("naive", detectedAt="2026-10-28T12:00:00"),
    queued_impact("future", detectedAt=(NOW + timedelta(seconds=1)).isoformat()),
    queued_impact("valid"),
  ):
    enqueue_impact_event(event, queue)
  assert peek_impact_events(queue) == [queued_impact("valid")]
  assert update_impact_event("expired", {"nextUploadAt": DETECTED_AT}, queue) is None


def test_impact_queue_is_bounded_and_deduplicated(tmp_path):
  queue = tmp_path / "queue"
  for index in range(MAX_QUEUED_IMPACTS + 5):
    enqueue_impact_event(queued_impact(str(index)), queue)
  events = peek_impact_events(queue)
  assert len(events) == MAX_QUEUED_IMPACTS
  assert events[0]["id"] == "5"
  enqueue_impact_event({**events[-1], "captureAttempts": 2}, queue)
  assert len(peek_impact_events(queue)) == MAX_QUEUED_IMPACTS
