#!/usr/bin/env python3
import json
import math
import os
import time
from datetime import datetime, timedelta, UTC
from pathlib import Path

from openpilot.cereal import log
from openpilot.system.wayon_impact import (
  DEFAULT_ARM_DELAY_S,
  DEFAULT_DOOR_LOCK_STATE_PATH,
  DEFAULT_IMPULSE_DYNAMIC_G,
  DEFAULT_IMPULSE_JERK_G_PER_S,
  DEFAULT_LOCK_STATE_MAX_AGE_S,
  DEFAULT_MIN_DYNAMIC_G,
  DEFAULT_MIN_JERK_G_PER_S,
  DEFAULT_STRONG_DYNAMIC_G,
  DoorLockTracker,
  ImpactDetector,
  append_impact_diagnostic,
  enqueue_impact_event,
  utc_now,
)
from openpilot.system.wayon_vehicle_events import (
  DEFAULT_PARKING_UNLOCKED_REMINDER_DELAY_S,
  ParkingUnlockReminder,
  door_lock_event,
  enqueue_vehicle_event,
  parking_unlocked_event,
)

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
STATUS_PATH = Path(os.getenv("WAYON_IMPACT_STATUS", "/data/wayon_cloud/impact_status.json"))
DEVICE_STATE_MAX_AGE_S = 2.5
PANDA_STATE_MAX_AGE_S = 0.5
SENSOR_MAX_AGE_S = 0.2
CAN_MAX_AGE_S = 0.5
DOOR_IMPACT_CONFIRM_S = 0.5
MAX_CAN_MESSAGES = 256
MAX_CAN_FRAMES = 8192
MAX_CAN_LOCK_SAMPLES = 128
MAX_CAN_MESSAGE_BYTES = 256 * 1024


def read_config() -> dict:
  try:
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
      config = json.load(handle)
      return config if isinstance(config, dict) else {}
  except (OSError, ValueError):
    return {}


def detector_from_config(config: dict) -> ImpactDetector:
  return ImpactDetector(
    arm_delay_s=float(config.get("impact_arm_delay_s", DEFAULT_ARM_DELAY_S)),
    warmup_s=float(config.get("impact_warmup_s", 3.0)),
    cooldown_s=float(config.get("impact_cooldown_s", 30.0)),
    min_dynamic_g=float(config.get("impact_min_dynamic_g", DEFAULT_MIN_DYNAMIC_G)),
    min_jerk_g_per_s=float(config.get("impact_min_jerk_g_per_s", DEFAULT_MIN_JERK_G_PER_S)),
    impulse_dynamic_g=float(config.get("impact_impulse_dynamic_g", DEFAULT_IMPULSE_DYNAMIC_G)),
    impulse_jerk_g_per_s=float(config.get("impact_impulse_jerk_g_per_s", DEFAULT_IMPULSE_JERK_G_PER_S)),
    strong_dynamic_g=float(config.get("impact_strong_dynamic_g", DEFAULT_STRONG_DYNAMIC_G)),
  )


def config_bool(config: dict, key: str, default: bool = False) -> bool:
  value = config.get(key, default)
  if isinstance(value, str):
    return value.strip().lower() in ("1", "true", "yes", "on")
  return bool(value)


def lock_tracker_from_config(config: dict, car_fingerprint: str = "") -> DoorLockTracker:
  # Only this vehicle's receive-side CAN layout has been verified here.
  traverse = car_fingerprint == "CHEVROLET_TRAVERSE"
  activity_guard = config_bool(config, "impact_door_activity_guard", traverse)
  return DoorLockTracker(
    required=not activity_guard and config_bool(config, "impact_require_door_locked"),
    bus=int(config.get("impact_lock_can_bus", 0)),
    address=int(config.get("impact_lock_can_address", 0x130)),
    byte_index=int(config.get("impact_lock_can_byte", 0)),
    mask=int(config.get("impact_lock_can_mask", 0x06)),
    unlocked_value=int(config.get("impact_lock_can_unlocked_value", 0x02)),
    arm_delay_s=float(config.get("impact_lock_arm_delay_s", 3.0)),
    state_path=Path(config.get("impact_lock_state_path", DEFAULT_DOOR_LOCK_STATE_PATH)),
    max_age_s=float(config.get("impact_lock_state_max_age_s", DEFAULT_LOCK_STATE_MAX_AGE_S)),
    require_doors_closed=not activity_guard and config_bool(config, "impact_require_doors_closed"),
    activity_guard=activity_guard,
  )


def detector_status(detector: ImpactDetector, lock_tracker: DoorLockTracker) -> dict:
  now = time.monotonic()
  return {
    "armDelayS": detector.arm_delay_s,
    "minDynamicG": detector.min_dynamic_g,
    "minJerkGPerSec": detector.min_jerk_g_per_s,
    "impulseDynamicG": detector.impulse_dynamic_g,
    "impulseJerkGPerSec": detector.impulse_jerk_g_per_s,
    "strongDynamicG": detector.strong_dynamic_g,
    "requireDoorLocked": lock_tracker.required,
    "doorsLocked": lock_tracker.locked,
    "requireDoorsClosed": lock_tracker.require_doors_closed,
    "doorOpen": lock_tracker.door_open,
    "doorActivityGuard": lock_tracker.activity_guard,
    "doorStateObserved": lock_tracker.locked is not None and lock_tracker.door_open is not None,
    "doorStateRefreshRequired": lock_tracker.lock_refresh_required or lock_tracker.door_refresh_required,
    "lockObservationAgeS": None if lock_tracker.last_sample_at is None else round(now - lock_tracker.last_sample_at, 1),
    "doorObservationAgeS": None if lock_tracker.last_door_sample_at is None else round(now - lock_tracker.last_door_sample_at, 1),
  }


def operating_state(detector: ImpactDetector, lock_tracker: DoorLockTracker,
                    received_first_sample: bool, now: float) -> str:
  lock_tracker.expire(now)
  if lock_tracker.watches_doors and lock_tracker.door_open is True:
    return "pausedDoorOpen"
  if lock_tracker.door_refresh_required or (lock_tracker.require_doors_closed and lock_tracker.door_open is None):
    return "waitingForDoorState"
  if lock_tracker.lock_refresh_required or (lock_tracker.required and lock_tracker.locked is None):
    return "waitingForLockState"
  if (lock_tracker.required or lock_tracker.activity_guard) and lock_tracker.locked is False:
    return "pausedUnlocked"
  if not lock_tracker.detection_allowed(now):
    return "lockSettling"
  if not received_first_sample or not detector.armed:
    return "warmingUp"
  return "armed"


def write_status(state: str, **details) -> None:
  STATUS_PATH.parent.mkdir(parents=True, exist_ok=True)
  payload = {"state": state, "updatedAt": utc_now(), **details}
  temporary = STATUS_PATH.with_suffix(".tmp")
  temporary.write_text(json.dumps(payload, separators=(",", ":"), ensure_ascii=False), encoding="utf-8")
  os.replace(temporary, STATUS_PATH)


def sensor_vector(message, measurement: str) -> tuple[float, float, float] | None:
  try:
    if message.which() != measurement:
      return None
    values = getattr(message, measurement).v
    if len(values) != 3:
      return None
    vector = tuple(float(value) for value in values)
    return vector if all(math.isfinite(value) for value in vector) else None
  except Exception:
    return None


def service_fresh(sm, service: str, now: float, max_age_s: float) -> bool:
  return bool(sm.seen[service] and sm.valid[service]
              and sm.logMonoTime[service] > 0
              and 0 <= now - sm.logMonoTime[service] / 1e9 <= max_age_s
              and 0 <= now - sm.recv_time[service] <= max_age_s)


def offroad_allowed(sm, now: float) -> bool:
  if not service_fresh(sm, "deviceState", now, DEVICE_STATE_MAX_AGE_S) \
      or not service_fresh(sm, "pandaStates", now, PANDA_STATE_MAX_AGE_S):
    return False
  pandas = sm["pandaStates"]
  return not sm["deviceState"].started and bool(len(pandas)) and all(
    panda.pandaType != "unknown" and not panda.ignitionLine and not panda.ignitionCan
    and not panda.controlsAllowed for panda in pandas
  )


def sensor_sample(sm, service: str, measurement: str, now: float):
  if not service_fresh(sm, service, now, SENSOR_MAX_AGE_S):
    return None
  sample_at = sm[service].timestamp / 1e9
  if sample_at <= 0 or not 0 <= now - sample_at <= SENSOR_MAX_AGE_S:
    return None
  vector = sensor_vector(sm[service], measurement)
  return (vector, sample_at) if vector is not None else None


class CanStreamHealth:
  """Fresh empty CAN envelopes prove the receiver is alive, not that doors are locked."""
  def __init__(self) -> None:
    self.sample_at: float | None = None
    self.received_at: float | None = None

  def observe(self, sample_at: float, received_at: float) -> None:
    if self.sample_at is None or sample_at > self.sample_at:
      self.sample_at = sample_at
      self.received_at = received_at

  def fresh(self, now: float) -> bool:
    return all(value is not None and 0 <= now - value <= CAN_MAX_AGE_S
               for value in (self.sample_at, self.received_at))


def drain_lock_can(sock, tracker: DoorLockTracker, now: float | None = None,
                   health: CanStreamHealth | None = None) -> tuple[list, bool]:
  """Keep fresh lock frames in wire order, with fixed work/storage bounds.

  An incomplete drain is not a latest-state observation. The caller discards
  the batch, invalidates lock history and reconnects to drop the old backlog.
  """
  samples = []
  frame_count = 0
  for _ in range(MAX_CAN_MESSAGES):
    raw = sock.receive(non_blocking=True)
    if raw is None:
      return samples, True
    if len(raw) > MAX_CAN_MESSAGE_BYTES:
      return [], False
    try:
      with log.Event.from_bytes(raw) as event:
        if event.which() != "can" or not event.valid:
          return [], False
        sample_at = event.logMonoTime / 1e9
        received_at = time.monotonic() if now is None else now
        if sample_at <= 0 or not 0 <= received_at - sample_at <= CAN_MAX_AGE_S:
          continue
        if health is not None:
          health.observe(sample_at, received_at)
        frame_count += len(event.can)
        if frame_count > MAX_CAN_FRAMES:
          return [], False
        for frame in event.can:
          if tracker.matches_frame(frame.src, frame.address):
            if len(samples) >= MAX_CAN_LOCK_SAMPLES:
              return [], False
            samples.append((sample_at, int(frame.src), int(frame.address), bytes(frame.dat)))
    except Exception:
      return [], False
  # One bounded probe distinguishes exactly-full from an unfinished drain.
  return (samples, True) if sock.receive(non_blocking=True) is None else ([], False)


class OffroadImpactMonitor:
  def __init__(self, detector: ImpactDetector, tracker: DoorLockTracker, reminder: ParkingUnlockReminder) -> None:
    self.detector = detector
    self.tracker = tracker
    self.reminder = reminder
    self.offroad_since: float | None = None
    self.last_can_at = 0.0
    self.pending_impact: tuple[float, dict] | None = None
    self.can_health = CanStreamHealth()
    self.panda_snapshot: tuple | None = None
    self.guard_established = False
    self.pause_reason = "waitingForOffroad"

  def reset(self) -> None:
    self.detector.reset()
    self.tracker.reset()
    self.reminder.reset()
    self.offroad_since = None
    self.last_can_at = 0.0
    self.pending_impact = None
    self.pause_reason = "waitingForOffroad"

  def interrupt(self, reason: str) -> None:
    self.reset()
    if self.tracker.activity_guard:
      self.tracker.invalidate()
    self.pause_reason = reason
    self.guard_established = True

  def transport_healthy(self, safety, now: float) -> bool:
    if not self.tracker.activity_guard:
      return True
    if not self.can_health.fresh(now) or not 0 <= self.tracker.bus <= 2:
      return False
    panda = safety["pandaStates"][0]
    bus = getattr(panda, f"canState{self.tracker.bus}")
    snapshot = (int(panda.uptime), str(panda.harnessStatus), str(panda.pandaType), int(panda.rxBufferOverflow),
                int(panda.spiChecksumErrorCount), int(bus.totalRxLostCnt), int(bus.busOffCnt), int(bus.canCoreResetCnt))
    previous = self.panda_snapshot
    self.panda_snapshot = snapshot
    if previous is not None and (snapshot[0] < previous[0] or snapshot[1:] != previous[1:]):
      return False
    return (panda.harnessStatus != "notConnected" and not panda.heartbeatLost and panda.faultStatus == "none"
            and not bus.busOff and not bus.errorWarning and not bus.errorPassive and bus.receiveErrorCnt == 0)

  def update(self, imu, safety, can_samples: list, can_complete: bool, now: float) -> tuple[list[dict], dict | None]:
    if not offroad_allowed(safety, now):
      active = (service_fresh(safety, "deviceState", now, DEVICE_STATE_MAX_AGE_S) and safety["deviceState"].started) or (
        service_fresh(safety, "pandaStates", now, PANDA_STATE_MAX_AGE_S)
        and any(p.ignitionLine or p.ignitionCan or p.controlsAllowed for p in safety["pandaStates"]))
      if self.guard_established and not active:
        self.interrupt("waitingForOffroad")
      else:
        self.reset()
        self.guard_established = False
        self.panda_snapshot = None
      return [], None
    if not can_complete or not self.transport_healthy(safety, now):
      if not can_complete or self.guard_established or self.panda_snapshot is not None:
        self.interrupt("waitingForCanReceiver")
      else:
        # Subscription startup is not a lost established connection. Wait for
        # the first heartbeat without requiring sleeping doors to speak.
        self.reset()
        self.pause_reason = "waitingForCanReceiver"
      return [], None
    self.guard_established = True
    if self.offroad_since is None:
      self.offroad_since = now

    vehicle_events = []
    wall_now = datetime.now(UTC)
    previous_can_at = self.last_can_at
    for sample_at, bus, address, data in can_samples:
      # A fresh door edge can precede the first offroad poll by milliseconds.
      # The activity policy must not drop that edge and call the state unknown.
      if (not self.tracker.activity_guard and sample_at < self.offroad_since) \
          or sample_at <= previous_can_at or sample_at < self.last_can_at \
          or not 0 <= now - sample_at <= CAN_MAX_AGE_S:
        continue
      self.tracker.expire(sample_at)
      was_known = self.tracker.locked is not None
      changed = self.tracker.update(bus, address, data, sample_at)
      self.last_can_at = sample_at
      if not self.tracker.detection_allowed(sample_at):
        # An open/unlock followed by close/lock in one drain must discard the
        # old gravity/candidate too, even if there is no new IMU sample yet.
        self.detector.reset()
        self.pending_impact = None
      if changed:
        self.reminder.reset()
        if was_known and self.tracker.locked is not None:
          event = door_lock_event(self.tracker.locked)
          # Preserve edge spacing for the uploader's refresh-pair filter.
          event["occurredAt"] = (wall_now - timedelta(seconds=now - sample_at)).isoformat(
            timespec="milliseconds").replace("+00:00", "Z")
          vehicle_events.append(event)

    self.tracker.expire(now)
    if not self.tracker.detection_allowed(now):
      self.detector.reset()
      self.pending_impact = None
    if self.reminder.update(self.tracker.locked, now):
      vehicle_events.append(parking_unlocked_event(self.reminder.delay_s))

    accel = sensor_sample(imu, "accelerometer", "acceleration", now)
    gyro = sensor_sample(imu, "gyroscope", "gyroUncalibrated", now)
    if accel is None or gyro is None or min(accel[1], gyro[1]) < self.offroad_since:
      self.detector.reset()
      self.pending_impact = None
      return vehicle_events, None
    if not imu.updated["accelerometer"]:
      return vehicle_events, None
    event = self.detector.update(accel[0], gyro[0], accel[1], self.tracker.detection_allowed(now))
    if not self.detector.armed:
      self.pending_impact = None
    if event is not None:
      event["captureRequested"] = True
      if self.tracker.guards_impacts:
        # Door CAN may arrive just after the physical door jolt. Keep the
        # original detection timestamp, but allow those edges to cancel it.
        self.pending_impact = (now, event)
        event = None
    if self.pending_impact is not None and now - self.pending_impact[0] >= DOOR_IMPACT_CONFIRM_S:
      _, event = self.pending_impact
      self.pending_impact = None
    return vehicle_events, event


def queue_if_offroad(safety, monitor: OffroadImpactMonitor, event: dict, enqueue, *, impact: bool = False) -> bool:
  # CAN processing and disk IO may span a state transition. Recheck each write.
  safety.update(0)
  now = time.monotonic()
  if monitor.offroad_since is None or not offroad_allowed(safety, now):
    monitor.interrupt("waitingForOffroad")
    return False
  if not monitor.transport_healthy(safety, now):
    monitor.interrupt("waitingForCanReceiver")
    return False
  if impact and not monitor.tracker.detection_allowed(now):
    monitor.detector.reset()
    monitor.pending_impact = None
    return False
  enqueue(event)
  return True


def main() -> None:
  from openpilot.cereal import car, messaging
  from openpilot.common.params import Params

  config = read_config()
  detector = detector_from_config(config)
  car_fingerprint = ""
  raw_params = Params().get("CarParamsPersistent")
  if raw_params:
    try:
      with car.CarParams.from_bytes(raw_params) as cp:
        car_fingerprint = cp.carFingerprint
    except Exception:
      print("Wayon impact: cannot read vehicle identity; using explicit configuration", flush=True)
  lock_tracker = lock_tracker_from_config(config, car_fingerprint)
  parking_unlock_reminder = ParkingUnlockReminder(
    delay_s=float(config.get(
      "parking_unlocked_reminder_delay_s", DEFAULT_PARKING_UNLOCKED_REMINDER_DELAY_S)),
    enabled=config_bool(config, "parking_unlocked_reminder_enabled", True),
  )
  sm = messaging.SubMaster(["accelerometer", "gyroscope"], poll="accelerometer")
  safety = messaging.SubMaster(["deviceState", "pandaStates"])
  can_sock = messaging.sub_sock("can", conflate=False)
  monitor = OffroadImpactMonitor(detector, lock_tracker, parking_unlock_reminder)
  lock_tracker._persist()
  reported_state: str | None = None
  diagnostic_interval_s = max(10.0, float(config.get("impact_diagnostic_interval_s", 60.0)))
  next_diagnostic_at: float | None = None
  diagnostic_samples = 0
  peak_dynamic_g = 0.0
  peak_total_g = 0.0
  peak_jerk_g_per_s = 0.0
  peak_gyro_rad_per_s = 0.0

  write_status("starting", **detector_status(detector, lock_tracker))
  print(f"Wayon impact: waiting for IMU, arm delay {detector.arm_delay_s:.0f}s", flush=True)

  while True:
    sm.update(100)
    safety.update(0)
    can_samples, can_complete = drain_lock_can(can_sock, lock_tracker, health=monitor.can_health)
    if not can_complete:
      del can_sock
      can_sock = messaging.sub_sock("can", conflate=False)
    safety.update(0)
    now = time.monotonic()
    vehicle_events, event = monitor.update(sm, safety, can_samples, can_complete, now)
    for vehicle_event in vehicle_events:
      if not queue_if_offroad(safety, monitor, vehicle_event, enqueue_vehicle_event):
        event = None
        break

    if event is not None:
      if queue_if_offroad(safety, monitor, event, enqueue_impact_event, impact=True):
        write_status("impactQueued", lastEvent=event, **detector_status(detector, lock_tracker))
        print(f"Wayon impact: queued {event['severity']} impact ({event['peakDynamicG']:.2f}g)", flush=True)

    state = (monitor.pause_reason if monitor.offroad_since is None else
             operating_state(detector, lock_tracker, detector.first_sample_at is not None, now))
    if state != reported_state:
      reported_state = state
      write_status(state, **detector_status(detector, lock_tracker))
      print(f"Wayon impact: {state}", flush=True)

    if not detector.armed:
      next_diagnostic_at = None
      diagnostic_samples = 0
      peak_dynamic_g = peak_total_g = peak_jerk_g_per_s = peak_gyro_rad_per_s = 0.0
      continue
    if not sm.updated["accelerometer"]:
      continue

    if detector.armed and next_diagnostic_at is None:
      next_diagnostic_at = now + diagnostic_interval_s

    if detector.armed:
      diagnostic_samples += 1
      peak_dynamic_g = max(peak_dynamic_g, detector.last_dynamic_g)
      peak_total_g = max(peak_total_g, detector.last_total_g)
      peak_jerk_g_per_s = max(peak_jerk_g_per_s, detector.last_jerk_g_per_s)
      peak_gyro_rad_per_s = max(peak_gyro_rad_per_s, detector.last_gyro_rad_per_s)

    if detector.armed and next_diagnostic_at is not None and now >= next_diagnostic_at:
      diagnostic = {
        "measuredAt": utc_now(),
        "windowS": diagnostic_interval_s,
        "sampleCount": diagnostic_samples,
        "peakDynamicG": round(peak_dynamic_g, 4),
        "peakTotalG": round(peak_total_g, 4),
        "peakJerkGPerSec": round(peak_jerk_g_per_s, 2),
        "peakGyroRadPerSec": round(peak_gyro_rad_per_s, 4),
        **detector_status(detector, lock_tracker),
      }
      append_impact_diagnostic(diagnostic)
      write_status(state, recentPeak=diagnostic, **detector_status(detector, lock_tracker))
      next_diagnostic_at = now + diagnostic_interval_s
      diagnostic_samples = 0
      peak_dynamic_g = 0.0
      peak_total_g = 0.0
      peak_jerk_g_per_s = 0.0
      peak_gyro_rad_per_s = 0.0


if __name__ == "__main__":
  main()
