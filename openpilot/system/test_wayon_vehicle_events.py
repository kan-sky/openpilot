import json
from datetime import datetime, timedelta, UTC

import pytest

from openpilot.system import wayon_vehicle_events
from openpilot.system.wayon_vehicle_events import (
  MAX_QUEUED_EVENT_BYTES,
  MAX_QUEUED_VEHICLE_EVENTS,
  VEHICLE_EVENT_TTL_S,
  ParkingUnlockReminder,
  door_lock_event,
  enqueue_vehicle_event,
  parking_unlocked_event,
  peek_vehicle_event,
  peek_vehicle_events,
  remove_vehicle_event,
)

NOW = datetime(2026, 10, 28, 12, tzinfo=UTC)


@pytest.fixture(autouse=True)
def frozen_clock(monkeypatch):
  class Clock(datetime):
    @classmethod
    def now(cls, tz=None):
      return NOW

  monkeypatch.setattr(wayon_vehicle_events, "datetime", Clock)


def test_door_lock_event_shape():
  event = door_lock_event(True)

  assert event["id"]
  assert event["eventType"] == "door_lock"
  assert event["locked"] is True
  assert event["test"] is False
  assert event["occurredAt"].endswith("Z")


def test_parking_unlocked_event_shape():
  event = parking_unlocked_event(179.6)

  assert event["id"]
  assert event["eventType"] == "parking_unlocked"
  assert event["delaySeconds"] == 180
  assert event["test"] is False
  assert event["occurredAt"].endswith("Z")


def test_parking_unlock_reminder_waits_for_known_unlocked_state():
  reminder = ParkingUnlockReminder(delay_s=180)

  assert not reminder.update(None, 0)
  assert not reminder.update(None, 300)
  assert not reminder.update(False, 300)
  assert not reminder.update(False, 479.9)
  assert reminder.update(False, 480)
  assert not reminder.update(False, 900)


def test_parking_unlock_reminder_is_reset_and_rearmed_by_lock():
  reminder = ParkingUnlockReminder(delay_s=180)

  assert not reminder.update(False, 10)
  assert not reminder.update(True, 100)
  assert not reminder.update(False, 400)
  assert not reminder.update(False, 579.9)
  assert reminder.update(False, 580)
  assert not reminder.update(False, 900)

  assert not reminder.update(True, 901)
  assert not reminder.update(False, 1000)
  assert not reminder.update(False, 1179.9)
  assert reminder.update(False, 1180)


def test_parking_unlock_reminder_can_be_disabled():
  reminder = ParkingUnlockReminder(delay_s=0, enabled=False)

  assert not reminder.update(False, 0)


def test_vehicle_event_queue_round_trip(tmp_path):
  queue = tmp_path / "vehicle_events.jsonl"
  unlocked = {**door_lock_event(False), "id": "unlocked"}
  locked = {**door_lock_event(True), "id": "locked"}

  enqueue_vehicle_event(unlocked, queue)
  enqueue_vehicle_event(locked, queue)
  assert peek_vehicle_event(queue) == unlocked
  assert remove_vehicle_event("unlocked", queue)
  assert peek_vehicle_event(queue) == locked
  assert remove_vehicle_event("locked", queue)
  assert peek_vehicle_event(queue) is None


@pytest.mark.parametrize("interruption", [None, True, "onroad", "clock_backwards"])
def test_reminder_requires_continuous_known_offroad_unlocked_state(interruption):
  reminder = ParkingUnlockReminder(delay_s=180)
  assert not reminder.update(False, 10)
  if interruption == "onroad":
    assert not reminder.update(False, 100, offroad=False)
  elif interruption == "clock_backwards":
    assert not reminder.update(False, 5)
  else:
    assert not reminder.update(interruption, 100)
  assert not reminder.update(False, 180)
  if interruption != "clock_backwards":
    assert not reminder.update(False, 359)
    assert reminder.update(False, 360)


def test_vehicle_queue_ttl_preserves_current_schema_and_valid_events(tmp_path):
  path = tmp_path / "queue"
  valid = door_lock_event(True)
  boundary = {**door_lock_event(False), "occurredAt": (NOW - timedelta(seconds=VEHICLE_EVENT_TTL_S)).isoformat()}
  expired = {**parking_unlocked_event(180), "occurredAt": (NOW - timedelta(seconds=VEHICLE_EVENT_TTL_S + 1)).isoformat()}
  for event in (expired, boundary, valid):
    enqueue_vehicle_event(event, path)
  assert peek_vehicle_events(path) == [boundary, valid]
  assert set(valid) == {"id", "eventType", "occurredAt", "locked", "test"}


def test_vehicle_queue_bounds_and_corrupt_lines(tmp_path):
  path = tmp_path / "queue"
  for index in range(MAX_QUEUED_VEHICLE_EVENTS + 2):
    enqueue_vehicle_event({**door_lock_event(True), "id": str(index)}, path)
  events = peek_vehicle_events(path)
  assert len(events) == MAX_QUEUED_VEHICLE_EVENTS
  assert events[0]["id"] == "2"
  valid = door_lock_event(True)
  path.write_text("broken\n[]\n" + "x" * (MAX_QUEUED_EVENT_BYTES + 1) + "\n" + json.dumps(valid) + "\n")
  assert peek_vehicle_events(path) == [valid]
