#!/usr/bin/env python3
import fcntl
import json
import math
import os
import uuid
from collections import deque
from datetime import datetime, UTC
from pathlib import Path

DEFAULT_VEHICLE_EVENT_QUEUE_PATH = Path(os.getenv(
  "WAYON_VEHICLE_EVENT_QUEUE", "/data/wayon_cloud/vehicle_event_queue.jsonl"))
MAX_QUEUED_VEHICLE_EVENTS = 128
VEHICLE_EVENT_TTL_S = 10 * 60
MAX_QUEUED_EVENT_BYTES = 64 * 1024
DEFAULT_PARKING_UNLOCKED_REMINDER_DELAY_S = 180.0
DEFAULT_DOOR_LOCK_WAKE_FILTER_MIN_S = 4.0
DEFAULT_DOOR_LOCK_WAKE_FILTER_MAX_S = 6.0


def utc_now() -> str:
  return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def door_lock_event(locked: bool, test: bool = False) -> dict:
  return {
    "id": str(uuid.uuid4()),
    "eventType": "door_lock",
    "occurredAt": utc_now(),
    "locked": bool(locked),
    "test": bool(test),
  }


def parking_unlocked_event(delay_s: float, test: bool = False) -> dict:
  return {
    "id": str(uuid.uuid4()),
    "eventType": "parking_unlocked",
    "occurredAt": utc_now(),
    "delaySeconds": int(round(max(0.0, delay_s))),
    "test": bool(test),
  }


class ParkingUnlockReminder:
  def __init__(self, delay_s: float = DEFAULT_PARKING_UNLOCKED_REMINDER_DELAY_S,
               enabled: bool = True) -> None:
    self.delay_s = max(0.0, float(delay_s))
    self.enabled = bool(enabled)
    self.unlocked_since: float | None = None
    self.alerted = False
    self.last_update_at: float | None = None

  def reset(self) -> None:
    self.unlocked_since = None
    self.alerted = False
    self.last_update_at = None

  def update(self, locked: bool | None, now: float, offroad: bool = True) -> bool:
    if not self.enabled or not offroad or locked is not False or not math.isfinite(now):
      self.reset()
      return False
    if self.last_update_at is not None and now < self.last_update_at:
      self.reset()
    self.last_update_at = now
    if self.alerted:
      return False
    if self.unlocked_since is None:
      self.unlocked_since = now
    if now - self.unlocked_since < self.delay_s:
      return False

    self.alerted = True
    return True


def read_queued_events(handle, timestamp_field: str, ttl_s: float, max_events: int) -> list[dict]:
  """Bound memory and reject expired, undated, or future-dated notifications."""
  handle.seek(0)
  events: deque[dict] = deque(maxlen=max_events)
  now = datetime.now(UTC)
  while line := handle.readline(MAX_QUEUED_EVENT_BYTES + 1):
    if len(line) > MAX_QUEUED_EVENT_BYTES:
      # Do not parse a suffix of an oversized line as a new event.
      while not line.endswith("\n"):
        line = handle.readline(MAX_QUEUED_EVENT_BYTES + 1)
        if not line:
          break
      continue
    try:
      event = json.loads(line)
      if not isinstance(event, dict) or not event.get("id"):
        continue
      created_at = datetime.fromisoformat(str(event.get(timestamp_field, "")).replace("Z", "+00:00"))
      age = (now - created_at).total_seconds()
      if 0 <= age <= ttl_s:
        events.append(event)
    except (TypeError, ValueError, OverflowError):
      continue
  return list(events)


def _read_events(handle) -> list[dict]:
  return read_queued_events(handle, "occurredAt", VEHICLE_EVENT_TTL_S, MAX_QUEUED_VEHICLE_EVENTS)


def _write_events(handle, events: list[dict]) -> None:
  handle.seek(0)
  handle.truncate()
  for event in events:
    handle.write(json.dumps(event, separators=(",", ":"), ensure_ascii=False) + "\n")
  handle.flush()
  os.fsync(handle.fileno())


def enqueue_vehicle_event(event: dict, path: Path = DEFAULT_VEHICLE_EVENT_QUEUE_PATH,
                          max_events: int = MAX_QUEUED_VEHICLE_EVENTS) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open("a+", encoding="utf-8") as handle:
    fcntl.flock(handle, fcntl.LOCK_EX)
    events = [queued for queued in _read_events(handle) if queued.get("id") != event.get("id")]
    events.append(dict(event))
    _write_events(handle, events[-max(1, min(MAX_QUEUED_VEHICLE_EVENTS, max_events)):])
    fcntl.flock(handle, fcntl.LOCK_UN)


def peek_vehicle_event(path: Path = DEFAULT_VEHICLE_EVENT_QUEUE_PATH) -> dict | None:
  events = peek_vehicle_events(path, limit=1)
  return events[0] if events else None


def peek_vehicle_events(path: Path = DEFAULT_VEHICLE_EVENT_QUEUE_PATH,
                        limit: int | None = None) -> list[dict]:
  try:
    with path.open("r", encoding="utf-8") as handle:
      fcntl.flock(handle, fcntl.LOCK_SH)
      events = _read_events(handle)
      fcntl.flock(handle, fcntl.LOCK_UN)
  except FileNotFoundError:
    return []
  return events if limit is None else events[:max(0, limit)]


def remove_vehicle_event(event_id: str, path: Path = DEFAULT_VEHICLE_EVENT_QUEUE_PATH) -> bool:
  return remove_vehicle_events({event_id}, path) > 0


def remove_vehicle_events(event_ids: set[str],
                          path: Path = DEFAULT_VEHICLE_EVENT_QUEUE_PATH) -> int:
  if not event_ids:
    return 0
  try:
    with path.open("r+", encoding="utf-8") as handle:
      fcntl.flock(handle, fcntl.LOCK_EX)
      events = _read_events(handle)
      remaining = [event for event in events if event.get("id") not in event_ids]
      removed = len(events) - len(remaining)
      if removed:
        _write_events(handle, remaining)
      fcntl.flock(handle, fcntl.LOCK_UN)
      return removed
  except FileNotFoundError:
    return 0
