#!/usr/bin/env python3
import base64
import hashlib
import io
import json
import math
import os
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit

import requests
import psutil

from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.system.camera_lease import CameraLease
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths
from openpilot.system.wayon_impact import peek_impact_events, remove_impact_event, update_impact_event
from openpilot.system.wayon_vehicle_events import (
  DEFAULT_DOOR_LOCK_WAKE_FILTER_MAX_S,
  DEFAULT_DOOR_LOCK_WAKE_FILTER_MIN_S,
  peek_vehicle_events,
  remove_vehicle_event,
  remove_vehicle_events,
)
from openpilot.system.wayon_identity import ensure_wayon_identity
from openpilot.system.wayon_cloud_policy import UploadBackoff, bounded_interval
from openpilot.system.wayon_ambient_delivery import AmbientCommandDelivery
from openpilot.system.wayon_camera import sample_fresh
from openpilot.system.wayon_drive_report import DriveReportAccumulator
from openpilot.system.wayon_event_blackbox import (
  BLACKBOX_SCHEMA, CAMERA_LAYOUT, DEFAULT_BLACKBOX_ROOT, WorkAborted, WorkBudget, WorkLimits,
  ResourceLimitExceeded, build_blackbox_archive, prune_blackbox_archives,
)
from openpilot.system.wayon_live_archive import upload_live_capture
from openpilot.system.wayon_drive_quality import (
  StopQualityTracker,
  cutin_risk_stage,
  resolve_operating_state,
  telemetry_signature,
)

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", str(Path.home() / ".wayon_cloud" / "config.json") if PC else "/data/wayon_cloud/config.json"))
ROUTE_STATE_PATH = Path(os.getenv("WAYON_CLOUD_ROUTE_STATE", str(CONFIG_PATH.with_name("route_state.json"))))
UPLOAD_HEALTH_PATH = Path(os.getenv("WAYON_CLOUD_UPLOAD_HEALTH", "/dev/shm/wayon_cloud_health.json"))
NAVDY_HEALTH_PATH = Path(os.getenv("WAYON_NAVDY_HEALTH", "/dev/shm/navdy-status.json"))
USER_AGENT = "wayon-cloud-uploader/1.0"
GPS_SERVICE_MAX_AGE_S = 3.0
GPS_TIMESTAMP_MAX_AGE_MS = 15_000
LAST_GPS_POSITION_MAX_AGE_MS = 120_000
IMPACT_MEDIA_ROOT = Path(os.getenv("WAYON_IMPACT_MEDIA_ROOT", "/data/wayon_cloud/impact_media"))
LAST_SUPPRESSED_VEHICLE_PAIR_PATH = Path(os.getenv(
  "WAYON_LAST_SUPPRESSED_VEHICLE_PAIR",
  "/data/wayon_cloud/last_suppressed_vehicle_pair.json",
))
MAX_IMPACT_CAPTURE_ATTEMPTS = 3
MAX_IMPACT_CAPTURE_AGE_S = 600.0
IMPACT_UPLOAD_RETRY_DELAYS_S = (60, 300, 900, 3600, 21600)
WAYON_WIDE_SNAPSHOT_WARMUP_S = 0.5
WAYON_DRIVER_SNAPSHOT_WARMUP_S = 0.8
WAYON_SNAPSHOT_EXPOSURE_READY_RATIO = 0.78
WAYON_SNAPSHOT_EXPOSURE_STABLE_FRAMES = 3
WAYON_SNAPSHOT_EXPOSURE_MAX_WAIT_S = 5.0
WAYON_WIDE_SNAPSHOT_TARGET_MEDIAN = 70.0
WAYON_WIDE_SNAPSHOT_MIN_GAMMA = 0.62
OFFROAD_SNAPSHOT_TIMEOUT_S = 30.0
DEFAULT_VEHICLE_STATUS_URL = (
  "https://mycarserver-fb85e-default-rtdb.firebaseio.com/car_status.json"
)
DEFAULT_DOOR_LOCK_REFRESH_MARKER_WINDOW_S = 30.0
DOOR_LOCK_REFRESH_MARKER_FUTURE_TOLERANCE_S = 3.0

DEFAULT_TELEMETRY_INTERVAL_ONROAD = 30.0
DEFAULT_TELEMETRY_INTERVAL_OFFROAD = 300.0
DEFAULT_ROUTE_SUMMARY_INTERVAL_OFFROAD = 60.0
DEFAULT_ROUTE_SUMMARY_GRACE_PERIOD = 45.0
DEFAULT_ROUTE_SUMMARY_MAX_AGE = 24.0 * 60.0 * 60.0
DEFAULT_LATEST_ROUTE_GPS_MAX_AGE = 2.0 * 60.0 * 60.0
DEFAULT_ROUTE_POINT_INTERVAL = 10.0
DEFAULT_ROUTE_POINT_MIN_DISTANCE_M = 15.0
DEFAULT_ROUTE_POINT_LIMIT = 720
DEFAULT_SNAPSHOT_INTERVAL_OFFROAD = 3600.0
AMBIENT_OVERRIDE_PATH = Path(os.getenv("WAYON_AMBIENT_OVERRIDE_PATH", "/data/params/d/WayonAmbientOverride"))
CONFIG_RELOAD_INTERVAL = 60.0
LOOP_SLEEP_ONROAD = 1.0
LOOP_SLEEP_OFFROAD = 1.0
LOG_FILE_CANDIDATES = ("qlog.zst", "qlog.bz2", "qlog")
ROUTE_LOG_MAX_BYTES = 8 * 1024 * 1024
ROUTE_LOG_MAX_DECOMPRESSED_BYTES = 32 * 1024 * 1024
ROUTE_MIN_AVAILABLE_BYTES = 256 * 1024 * 1024
ROUTE_UPLOAD_SCHEMA = "wayon-carrot-analysis-v4-coverage-receipt-v1"
ROUTE_RECEIPT_SCHEMA = "wayon-route-upload-receipt-v1"
ROUTE_RETRY_INTERVAL_S = 300.0
ROUTE_WORK_LIMITS = WorkLimits(max_log_bytes=ROUTE_LOG_MAX_BYTES,
                               max_decoded_log_bytes=ROUTE_LOG_MAX_DECOMPRESSED_BYTES,
                               max_seconds=30.0)
DEFAULT_BLACKBOX_UPLOAD_LIMIT_PER_ROUTE = 4
DEFAULT_BLACKBOX_MAX_UPLOAD_BYTES = 48 * 1024 * 1024
STATE_SERVICES = ["deviceState", "pandaStates", "gpsLocationExternal", "gpsLocation", "selfdriveState"]
TELEMETRY_SERVICES = STATE_SERVICES + ["carState", "radarState"]


def utc_now():
  return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def read_json_file(path):
  try:
    with Path(path).open("r", encoding="utf-8") as f:
      payload = json.load(f)
    return payload if isinstance(payload, dict) else {}
  except (OSError, TypeError, ValueError, json.JSONDecodeError):
    return {}


def write_json_atomic(path, payload):
  path = Path(path)
  try:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
    os.replace(temporary, path)
  except OSError:
    pass


def iso_age_seconds(value, now_epoch=None):
  try:
    timestamp = datetime.fromisoformat(str(value).replace("Z", "+00:00")).timestamp()
  except (TypeError, ValueError):
    return None
  return max(0.0, (time.time() if now_epoch is None else now_epoch) - timestamp)


def service_timestamps_payload(sm, services):
  now_monotonic = time.monotonic()
  result = {}
  for service in services:
    receive_time = float(sm.recv_time.get(service, 0.0))
    age = now_monotonic - receive_time if receive_time > 0.0 else None
    seen = bool(sm.seen.get(service, False))
    alive = bool(sm.alive.get(service, False)) if hasattr(sm, "alive") else seen
    freq_ok = bool(sm.freq_ok.get(service, False)) if hasattr(sm, "freq_ok") else alive
    result[service] = {
      "seen": seen,
      "alive": alive,
      "frequencyOk": freq_ok,
      "valid": bool(sm.valid.get(service, False)),
      "fresh": sample_fresh(sm, service, now=now_monotonic),
      "ageSeconds": round(age, 3) if age is not None and math.isfinite(age) else None,
    }
  return result


def connection_state(ok, detail, updated_at=None, age_seconds=None):
  return {
    "state": "ok" if ok else "fault",
    "detail": detail,
    "updatedAt": updated_at,
    "ageSeconds": round(age_seconds, 1) if age_seconds is not None else None,
  }


def connection_diagnostics(sm, services, gps, panda, vehicle, openpilot):
  timestamps = service_timestamps_payload(sm, services)
  navdy = read_json_file(NAVDY_HEALTH_PATH)
  navdy_updated = navdy.get("updatedAt")
  navdy_age = time.monotonic() - navdy_updated if type(navdy_updated) in (int, float) else None
  if navdy_age is not None and (not math.isfinite(navdy_age) or navdy_age < 0):
    navdy_age = None
  upload = read_json_file(UPLOAD_HEALTH_PATH)
  upload_age = iso_age_seconds(upload.get("lastSuccessAt"))
  can = vehicle.get("can") if isinstance(vehicle, dict) else {}
  offroad = vehicle.get("reason") == "offroad"
  car_ok = (vehicle.get("available", False) and sample_fresh(sm, "carState")
            and bool(can.get("valid", False)) and not bool(can.get("timeout", False)))
  panda_ok = (sample_fresh(sm, "pandaStates") and panda is not None and not panda.get("heartbeatLost", False)
              and panda.get("faultStatus") in ("none", "normal", "") and not panda.get("faults"))
  navdy_ok = navdy_age is not None and navdy_age <= 8.0 and navdy.get("connected") is True
  gps_timestamp_ms = gps.get("timestampMillis")
  gps_age = max(0.0, time.time() - float(gps_timestamp_ms) / 1000.0) if gps_timestamp_ms else None
  service_age = lambda name: timestamps.get(name, {}).get("ageSeconds")
  connections = {
    "comma": connection_state(sample_fresh(sm, "deviceState"), "cereal device state",
                              age_seconds=service_age("deviceState")),
    "vehicleCan": connection_state(car_ok, "CAN valid" if car_ok else "CAN unavailable or invalid",
                                   age_seconds=service_age("carState")),
    "panda": connection_state(panda_ok, "safety interface online" if panda_ok else "panda fault or heartbeat missing",
                              age_seconds=service_age("pandaStates")),
    "gps": connection_state(bool(gps.get("fresh", False)), str(gps.get("source") or "unavailable"),
                            age_seconds=gps_age),
    "openpilot": connection_state(sample_fresh(sm, "selfdriveState") and bool(openpilot.get("available", False)),
                                  str(openpilot.get("state") or "unavailable"),
                                  age_seconds=service_age("selfdriveState")),
    "navdy": {**connection_state(navdy_ok, str(navdy.get("transport") or "disconnected"), age_seconds=navdy_age),
              "updatedAtMonotonic": navdy_updated if navdy_age is not None else None},
    "cloud": connection_state(upload_age is not None and upload_age <= 600.0,
                              "last telemetry upload", upload.get("lastSuccessAt"), upload_age),
  }
  if offroad:
    for name in ("vehicleCan", "openpilot"):
      connections[name] = {**connection_state(False, "inactive offroad"), "state": "inactive"}
  return connections


def radar_cutin_payload(sm):
  if not sample_fresh(sm, "radarState"):
    return {"available": False, "reason": "radarState_unavailable"}
  try:
    lead = sm["radarState"].leadCutInRisk
    if not bool(lead.status):
      return cutin_risk_stage(0.0)
    result = cutin_risk_stage(lead.score, lead.dRel, lead.vRel)
    result.update({
      "trackId": int(lead.radarTrackId),
      "distanceM": round(float(lead.dRel), 2),
      "lateralM": round(float(lead.yRel), 2),
      "relativeSpeedMps": round(float(lead.vRel), 2),
    })
    return result
  except Exception:
    return cutin_risk_stage(0.0)


def read_config():
  try:
    config = ensure_wayon_identity(CONFIG_PATH)
  except Exception as exc:
    print(f"Wayon cloud: identity setup failed: {type(exc).__name__}")
    return None

  # None is a deliberate rejection (including disabled/unenrolled config),
  # never permission to reopen the raw file and bypass identity validation.
  if not isinstance(config, dict) or config.get("enabled", True) is False:
    return None
  endpoint = config.get("endpoint")
  token = config.get("token")
  if not isinstance(endpoint, str) or not isinstance(token, str) or not token.startswith("wayon_") or len(token) <= 6:
    return None
  try:
    parsed = urlsplit(endpoint)
    if parsed.scheme != "https" or not parsed.hostname or parsed.username is not None or parsed.password is not None:
      return None
  except ValueError:
    return None
  return {**config, "endpoint": endpoint.rstrip("/"), "token": token}


def get_param_str(params, key):
  value = params.get(key)
  if isinstance(value, bytes):
    return value.decode("utf-8", "replace")
  return str(value or "")


def post_json(config, path, payload):
  response = requests.post(
    f"{config['endpoint']}{path}",
    json=payload,
    headers={
      "Authorization": f"Bearer {config['token']}",
      "Content-Type": "application/json",
      "User-Agent": USER_AGENT,
    },
    timeout=15,
  )
  response.raise_for_status()
  return response.json() if response.content else {}


def get_json(config, path, params=None):
  response = requests.get(
    f"{config['endpoint']}{path}",
    params=params,
    headers={
      "Authorization": f"Bearer {config['token']}",
      "User-Agent": USER_AGENT,
    },
    timeout=15,
  )
  response.raise_for_status()
  return response.json() if response.content else {}


def write_ambient_override(params, serialized, path=AMBIENT_OVERRIDE_PATH):
  try:
    params.put("WayonAmbientOverride", serialized)
    # Carrot's Params.put wrapper does not return the underlying write result.
    if get_param_str(params, "WayonAmbientOverride") == serialized:
      return
  except Exception:
    pass

  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  temporary.write_text(serialized, encoding="utf-8")
  os.replace(temporary, path)
  if path.read_text(encoding="utf-8") != serialized:
    raise OSError("ambient override readback mismatch")


def ambient_timestamp(value):
  try:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    return parsed.timestamp() if parsed.tzinfo is not None else None
  except (AttributeError, ValueError, TypeError, OverflowError):
    return None


def ambient_integer(value, minimum, maximum):
  return type(value) is int and minimum <= value <= maximum


def ambient_zone_valid(zone):
  return (isinstance(zone, dict) and type(zone.get("enabled")) is bool
          and isinstance(zone.get("rgb"), list) and len(zone["rgb"]) == 3
          and all(ambient_integer(value, 0, 255) for value in zone["rgb"])
          and ambient_integer(zone.get("brightness"), 0, 100)
          and ("automaticBrightness" not in zone or type(zone["automaticBrightness"]) is bool))


def ambient_profile_valid(profile):
  if not isinstance(profile, dict) or type(profile.get("enabled")) is not bool:
    return False
  zones = {"driving": ("zone1", "zone2"), "onroadDoor": ("zone1", "zone2"),
           "offroadDoor": ("zone1", "zone2"), "exitCourtesy": ("zone2",), "overspeed": ("zone1",)}
  for section, names in zones.items():
    value = profile.get(section)
    if not isinstance(value, dict) or any(not ambient_zone_valid(value.get(name)) for name in names):
      return False
  for section in ("onroadDoor", "offroadDoor", "exitCourtesy", "overspeed", "reverseOff", "dataWatchdog"):
    if not isinstance(profile.get(section), dict) or type(profile[section].get("enabled")) is not bool:
      return False
  ranges = {"exitCourtesy": {"durationSeconds": (0, 600)}, "overspeed": {"brightnessCap": (1, 100)},
            "dataWatchdog": {"timeoutSeconds": (5, 120)},
            "timing": {"fadeMilliseconds": (200, 5000), "doorCloseDelaySeconds": (0, 120),
                       "doorMaxOnMinutes": (1, 60), "transitionUpdatesPerSecond": (5, 40)}}
  return all(isinstance(profile.get(section), dict)
             and all(ambient_integer(profile[section].get(key), *bounds) for key, bounds in fields.items())
             for section, fields in ranges.items())


def apply_ambient_command(params, command, path=AMBIENT_OVERRIDE_PATH, device_id=None):
  if not isinstance(command, dict):
    return False
  if any(not isinstance(command.get(key), str) or not 1 <= len(command[key].strip()) <= 128
         for key in ("id", "deviceId")):
    return False
  if device_id is not None and command["deviceId"] != device_id:
    return False
  payload = command.get("payload")
  if not isinstance(payload, dict) or payload.get("schema") != "wayon.ambient.command.v1":
    return False
  created = ambient_timestamp(command.get("createdAt"))
  expires = ambient_timestamp(command.get("expiresAt"))
  now = time.time()
  if created is None or expires is None or not created <= now + 15 or not max(created, now) < expires:
    return False
  mode = payload.get("mode")
  if mode == "manual":
    if (not ambient_integer(payload.get("durationSeconds"), 30, 1200)
        or expires - created > payload["durationSeconds"]
        or not all(ambient_zone_valid(payload.get(name)) for name in ("zone1", "zone2"))):
      return False
  elif mode in ("auto", "profile"):
    if payload.get("durationSeconds") != 0 or expires - created > 60:
      return False
    if mode == "profile" and not ambient_profile_valid(payload.get("profile")):
      return False
  else:
    return False

  stored = {
    **payload,
    "id": str(command["id"]),
    "deviceId": str(command.get("deviceId") or ""),
    "createdAt": command.get("createdAt"),
    "expiresAt": command.get("expiresAt"),
  }
  try:
    serialized = json.dumps(stored, separators=(",", ":"), allow_nan=False)
  except (TypeError, ValueError, OverflowError, RecursionError):
    return False
  # Navdy read_override rejects anything over this byte limit.
  if len(serialized.encode("utf-8")) > 16384:
    return False
  write_ambient_override(params, serialized, path)
  return True


def poll_ambient_command(config, params, device_id):
  response = get_json(config, "/api/ambient/command", {"deviceId": device_id})
  command = response.get("command") if isinstance(response, dict) else None
  if not apply_ambient_command(params, command, device_id=device_id):
    command_id = command.get("id") if isinstance(command, dict) else None
    if (isinstance(command_id, str) and 1 <= len(command_id) <= 128 and command_id.strip()
        and isinstance(device_id, str) and 1 <= len(device_id) <= 128
        and command.get("deviceId") == device_id):
      # A failed ACK removes this command from the Worker's pending/delivered
      # poll. Keep diagnostics fixed-size and never echo the rejected payload.
      post_json(config, "/api/ambient/ack", {
        "id": command_id,
        "deviceId": device_id,
        "accepted": False,
        "applied": False,
        "navdyApplied": None,
        "stage": "comma_validation",
        "reason": "invalid_ambient_command",
        "acknowledgedAt": utc_now(),
      })
    return False

  post_json(config, "/api/ambient/ack", {
    "id": command["id"],
    "deviceId": device_id,
    # The v1 API treats only applied=false as failure. Null acknowledges
    # delivery without claiming a Navdy/BLE application ACK we do not have.
    "accepted": True,
    "applied": None,
    "navdyApplied": None,
    "stage": "comma_params",
    "detail": "Params stored and read back; Navdy and physical lighting unverified",
    "acknowledgedAt": utc_now(),
  })
  return True


def fetch_vehicle_status(config):
  response = requests.get(
    str(config.get("vehicle_status_url", DEFAULT_VEHICLE_STATUS_URL)),
    headers={"User-Agent": USER_AGENT},
    timeout=5,
  )
  response.raise_for_status()
  payload = response.json()
  return payload if isinstance(payload, dict) else {}


def impact_media_directory(event_id, media_root=IMPACT_MEDIA_ROOT):
  safe_id = "".join(character for character in str(event_id) if character.isalnum() or character in "-_")[:128]
  return Path(media_root) / (safe_id or "unknown")


def jpeg_bytes(array, quality=78, subsampling=-1):
  from PIL import Image

  buffer = io.BytesIO()
  Image.fromarray(array).save(buffer, "JPEG", quality=quality, subsampling=subsampling, optimize=True)
  return buffer.getvalue()


def write_bytes_atomic(path, data):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + ".tmp")
  temporary.write_bytes(data)
  os.replace(temporary, path)


def read_impact_media(event_id, media_root=IMPACT_MEDIA_ROOT):
  directory = impact_media_directory(event_id, media_root)
  media = {}
  for camera in ("wide", "driver"):
    path = directory / f"{camera}.jpg"
    if path.is_file():
      media[camera] = path.read_bytes()

  captured_at = None
  try:
    captured_at = json.loads((directory / "capture.json").read_text(encoding="utf-8")).get("capturedAt")
  except (OSError, TypeError, ValueError):
    pass
  return media, captured_at


def capture_and_store_impact_media(event_id, media_root=IMPACT_MEDIA_ROOT, capture_fn=None):
  media, captured_at = read_impact_media(event_id, media_root)
  if len(media) == 2:
    return media, captured_at

  capture_fn = capture_fn or capture_offroad_images
  wide, driver = capture_fn()
  directory = impact_media_directory(event_id, media_root)
  captured_at = captured_at or utc_now()
  for camera, image in (("wide", wide), ("driver", driver)):
    if camera not in media and image is not None:
      write_bytes_atomic(directory / f"{camera}.jpg", jpeg_bytes(image))
  write_bytes_atomic(
    directory / "capture.json",
    json.dumps({"capturedAt": captured_at}, separators=(",", ":")).encode("utf-8"),
  )
  return read_impact_media(event_id, media_root)


def remove_impact_media(event_id, media_root=IMPACT_MEDIA_ROOT):
  shutil.rmtree(impact_media_directory(event_id, media_root), ignore_errors=True)


def _impact_retry_due(event, now=None):
  retry_at = str(event.get("nextUploadAt") or "")
  if not retry_at:
    return True
  try:
    retry_time = datetime.fromisoformat(retry_at.replace("Z", "+00:00"))
    if retry_time.tzinfo is None:
      retry_time = retry_time.replace(tzinfo=timezone.utc)
    return (now or datetime.now(timezone.utc)) >= retry_time
  except ValueError:
    return True


def _update_queued_impact(event_id, values, queue_path):
  if queue_path is None:
    return update_impact_event(event_id, values)
  return update_impact_event(event_id, values, queue_path)


def _defer_impact_upload(event, event_id, queue_path, now=None):
  attempts = int(event.get("uploadAttempts", 0)) + 1
  delay = IMPACT_UPLOAD_RETRY_DELAYS_S[min(attempts - 1, len(IMPACT_UPLOAD_RETRY_DELAYS_S) - 1)]
  retry_at = (now or datetime.now(timezone.utc)).timestamp() + delay
  retry_iso = datetime.fromtimestamp(retry_at, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
  _update_queued_impact(event_id, {"uploadAttempts": attempts, "nextUploadAt": retry_iso}, queue_path)


def upload_pending_impacts(config, device_id, queue_path=None, limit=3,
                           media_root=IMPACT_MEDIA_ROOT, capture_fn=None, now=None):
  uploaded = 0
  events = peek_impact_events() if queue_path is None else peek_impact_events(queue_path)
  # New notifications outrank media retries, including a deferred queue head.
  events.sort(key=lambda event: (not event.get("impactUploaded"), str(event.get("detectedAt", ""))), reverse=True)
  pending = [event for event in events if _impact_retry_due(event, now)][:max(1, limit)]
  media_pending = []
  first_error = None

  def finish(event_id):
    remove_impact_media(event_id, media_root)
    if queue_path is None:
      remove_impact_event(event_id)
    else:
      remove_impact_event(event_id, queue_path)

  # Send every selected notification before any blocking camera work.
  for event in pending:
    event_id = str(event.get("id", ""))
    try:
      if not event.get("impactUploaded"):
        response = post_json(config, "/api/impact", {**event, "deviceId": device_id})
        event = _update_queued_impact(event_id, {
          "impactUploaded": True,
          "impactSuppressed": response.get("suppressed") is True,
          "uploadAttempts": 0,
          "nextUploadAt": None,
        }, queue_path) or {**event, "impactUploaded": True, "impactSuppressed": response.get("suppressed") is True}
      if event.get("impactSuppressed"):
        finish(event_id)
        uploaded += 1
        continue
      media_pending.append(event)
    except Exception as exc:
      _defer_impact_upload(event, event_id, queue_path, now)
      first_error = first_error or exc

  for event in media_pending:
    event_id = str(event.get("id", ""))
    try:
      if event.get("captureRequested"):
        media, captured_at = read_impact_media(event_id, media_root)
        attempts = int(event.get("captureAttempts", 0))
        try:
          detected_at = datetime.fromisoformat(str(event.get("detectedAt", "")).replace("Z", "+00:00"))
          age = ((now or datetime.now(timezone.utc)) - detected_at).total_seconds()
          capture_current = 0 <= age <= MAX_IMPACT_CAPTURE_AGE_S
        except (TypeError, ValueError):
          capture_current = False

        if len(media) < 2 and capture_current and attempts < MAX_IMPACT_CAPTURE_ATTEMPTS:
          attempts += 1
          event = _update_queued_impact(event_id, {"captureAttempts": attempts}, queue_path) or event
          try:
            media, captured_at = capture_and_store_impact_media(event_id, media_root, capture_fn)
          except Exception as exc:
            # Camera failures must count toward the bound and cannot block alerts.
            print(f"Wayon impact: camera capture failed ({attempts}/{MAX_IMPACT_CAPTURE_ATTEMPTS}): {type(exc).__name__}")
            media, captured_at = read_impact_media(event_id, media_root)

        capture_complete = all(camera in media for camera in ("wide", "driver"))
        # Never take a current photo and label it as a much older queued impact.
        if not capture_complete and capture_current and attempts < MAX_IMPACT_CAPTURE_ATTEMPTS:
          raise RuntimeError(f"impact camera capture incomplete ({attempts}/{MAX_IMPACT_CAPTURE_ATTEMPTS})")

        capture_status = "complete" if capture_complete else "partial" if media else "failed"
        post_json(config, "/api/impact-media", {
          "id": event_id,
          "deviceId": device_id,
          "capturedAt": captured_at or utc_now(),
          "captureStatus": capture_status,
          "captureAttempts": attempts,
          "wideJpegBase64": base64.b64encode(media["wide"]).decode("ascii") if "wide" in media else None,
          "driverJpegBase64": base64.b64encode(media["driver"]).decode("ascii") if "driver" in media else None,
        })
    except Exception as exc:
      if isinstance(exc, requests.HTTPError) and exc.response is not None and exc.response.status_code == 404:
        try:
          missing = exc.response.json().get("error") == "impact_not_found"
        except (ValueError, AttributeError):
          missing = False
        if missing:
          # Recover legacy entries marked uploaded after a suppression response.
          event = _update_queued_impact(event_id, {"impactUploaded": False, "uploadAttempts": 0}, queue_path) or event
      _defer_impact_upload(event, event_id, queue_path, now)
      first_error = first_error or exc
      continue

    finish(event_id)
    uploaded += 1
  if first_error is not None:
    raise first_error
  return uploaded


def _vehicle_event_datetime(event):
  try:
    parsed = datetime.fromisoformat(str(event.get("occurredAt", "")).replace("Z", "+00:00"))
    return parsed if parsed.tzinfo is not None else parsed.replace(tzinfo=timezone.utc)
  except (TypeError, ValueError):
    return None


def _next_door_relock(events, unlock_event):
  unlock_at = _vehicle_event_datetime(unlock_event)
  if unlock_at is None:
    return None, None
  for event in events[1:]:
    if event.get("eventType") != "door_lock":
      continue
    if event.get("locked") is not True:
      return None, None
    locked_at = _vehicle_event_datetime(event)
    if locked_at is None:
      return event, None
    return event, (locked_at - unlock_at).total_seconds()
  return None, None


def _epoch_seconds(value):
  if isinstance(value, str):
    value = value.rsplit("_", 1)[-1].strip()
  try:
    timestamp = float(value)
  except (TypeError, ValueError):
    return None
  if timestamp > 100_000_000_000:
    timestamp /= 1000.0
  return timestamp if timestamp > 0.0 and math.isfinite(timestamp) else None


def _matching_vehicle_refresh_marker(config, unlock_event, fetch_status_fn):
  unlock_at = _vehicle_event_datetime(unlock_event)
  if unlock_at is None:
    return None
  try:
    status = fetch_status_fn(config)
  except Exception as exc:
    print(f"Wayon vehicle: refresh marker unavailable, sending alerts: {exc}", flush=True)
    return None

  window_s = max(0.0, float(config.get(
    "door_lock_refresh_marker_window_s",
    DEFAULT_DOOR_LOCK_REFRESH_MARKER_WINDOW_S,
  )))
  unlock_timestamp = unlock_at.timestamp()
  for key in ("refresh_action", "cmd_refresh"):
    marker_timestamp = _epoch_seconds(status.get(key))
    if marker_timestamp is None:
      continue
    elapsed_s = unlock_timestamp - marker_timestamp
    if -DOOR_LOCK_REFRESH_MARKER_FUTURE_TOLERANCE_S <= elapsed_s <= window_s:
      return {"source": key, "timestamp": marker_timestamp, "elapsedSeconds": elapsed_s}
  return None


def _record_suppressed_vehicle_pair(unlock_event, relock_event, elapsed_s, path,
                                    refresh_marker=None):
  record = {
    "recordedAt": utc_now(),
    "reason": "vehicle_status_refresh",
    "elapsedSeconds": round(elapsed_s, 3),
    "unlockEvent": unlock_event,
    "relockEvent": relock_event,
  }
  if refresh_marker is not None:
    record["refreshMarker"] = refresh_marker
  temp_path = path.with_suffix(path.suffix + ".tmp")
  try:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path.write_text(json.dumps(record, separators=(",", ":"), ensure_ascii=False), encoding="utf-8")
    os.replace(temp_path, path)
  except OSError as exc:
    print(f"Wayon vehicle: failed to record suppressed pair: {exc}", flush=True)


def upload_pending_vehicle_events(config, device_id, queue_path=None, limit=3, now=None,
                                  suppressed_state_path=None, fetch_status_fn=None):
  uploaded = 0
  fetch_status_fn = fetch_status_fn or fetch_vehicle_status
  for _ in range(max(1, limit)):
    events = peek_vehicle_events() if queue_path is None else peek_vehicle_events(queue_path)
    event = events[0] if events else None
    if event is None:
      break

    if event.get("eventType") == "door_lock" and event.get("locked") is False:
      filter_max_s = max(0.0, float(config.get(
        "door_lock_notification_pair_window_s",
        DEFAULT_DOOR_LOCK_WAKE_FILTER_MAX_S,
      )))
      filter_min_s = min(filter_max_s, max(0.0, float(config.get(
        "door_lock_notification_pair_min_s",
        DEFAULT_DOOR_LOCK_WAKE_FILTER_MIN_S,
      ))))
      relock_event, relock_elapsed_s = _next_door_relock(events, event)

      if relock_event is not None and relock_elapsed_s is not None \
          and filter_min_s <= relock_elapsed_s <= filter_max_s:
        refresh_marker = _matching_vehicle_refresh_marker(config, event, fetch_status_fn)
        if refresh_marker is not None:
          state_path = suppressed_state_path or LAST_SUPPRESSED_VEHICLE_PAIR_PATH
          _record_suppressed_vehicle_pair(
            event, relock_event, relock_elapsed_s, state_path, refresh_marker)
          event_ids = {str(event.get("id", "")), str(relock_event.get("id", ""))}
          if queue_path is None:
            remove_vehicle_events(event_ids)
          else:
            remove_vehicle_events(event_ids, queue_path)
          print(
            f"Wayon vehicle: suppressed status refresh pair ({relock_elapsed_s:.3f}s)",
            flush=True,
          )
          continue

      if relock_event is None:
        occurred_at = _vehicle_event_datetime(event)
        current_time = now or datetime.now(timezone.utc)
        if occurred_at is not None and (current_time - occurred_at).total_seconds() <= filter_max_s:
          break

    payload = {**event, "deviceId": device_id}
    post_json(config, "/api/vehicle-event", payload)
    event_id = str(event.get("id", ""))
    if queue_path is None:
      remove_vehicle_event(event_id)
    else:
      remove_vehicle_event(event_id, queue_path)
    uploaded += 1
  return uploaded


def enum_name(value):
  try:
    return str(value).split(".")[-1]
  except Exception:
    return ""


def numeric_list(values):
  try:
    return [float(value) for value in values]
  except Exception:
    return []


def device_details_payload(device_state):
  temperatures = {
    "cpu": numeric_list(device_state.cpuTempC),
    "gpu": numeric_list(device_state.gpuTempC),
    "dsp": float(device_state.dspTempC),
    "memory": float(device_state.memoryTempC),
    "modem": numeric_list(device_state.modemTempC),
    "pmic": numeric_list(device_state.pmicTempC),
    "intake": float(device_state.intakeTempC),
    "exhaust": float(device_state.exhaustTempC),
    "gnss": float(device_state.gnssTempC),
    "bottomSoc": float(device_state.bottomSocTempC),
    "max": float(device_state.maxTempC),
  }
  try:
    temperatures["zones"] = [
      {"name": str(zone.name), "tempC": float(zone.temp)}
      for zone in device_state.thermalZones
    ]
  except Exception:
    temperatures["zones"] = []

  return {
    "type": enum_name(device_state.deviceType),
    "network": {
      "type": enum_name(device_state.networkType),
      "strength": enum_name(device_state.networkStrength),
      "metered": bool(device_state.networkMetered),
    },
    "usage": {
      "freeSpacePercent": float(device_state.freeSpacePercent),
      "memoryPercent": int(device_state.memoryUsagePercent),
      "gpuPercent": int(device_state.gpuUsagePercent),
      "cpuPercent": [int(value) for value in device_state.cpuUsagePercent],
    },
    "power": {
      "drawW": float(device_state.powerDrawW),
      "somDrawW": float(device_state.somPowerDrawW),
      "offroadUsageUwh": int(device_state.offroadPowerUsageUwh),
      "offroadUsageWh": float(device_state.offroadPowerUsageUwh) / 1_000_000.0,
      "carBatteryCapacityUwh": int(device_state.carBatteryCapacityUwh),
      "carBatteryCapacityWh": float(device_state.carBatteryCapacityUwh) / 1_000_000.0,
    },
    "thermal": {
      "status": enum_name(device_state.thermalStatus),
      "fanPercent": int(device_state.fanSpeedPercentDesired),
      "temperaturesC": temperatures,
    },
    "screenBrightnessPercent": int(device_state.screenBrightnessPercent),
  }


def panda_details_payload(panda_state):
  if panda_state is None:
    return None

  voltage_v = voltage_v_from_raw(panda_state.voltage)
  current_ma = current_ma_from_raw(panda_state.current)
  return {
    "type": enum_name(panda_state.pandaType),
    "ignitionLine": bool(panda_state.ignitionLine),
    "ignitionCan": bool(panda_state.ignitionCan),
    "voltageV": voltage_v,
    "currentMa": current_ma,
    "estimatedPowerW": voltage_v * current_ma / 1000.0 if voltage_v is not None and current_ma is not None else None,
    "faultStatus": enum_name(panda_state.faultStatus),
    "faults": [enum_name(fault) for fault in panda_state.faults],
    "uptimeS": int(panda_state.uptime),
    "heartbeatLost": bool(panda_state.heartbeatLost),
    "interruptLoad": float(panda_state.interruptLoad),
    "rxBufferOverflow": int(panda_state.rxBufferOverflow),
    "txBufferOverflow": int(panda_state.txBufferOverflow),
    "spiErrorCount": int(panda_state.spiChecksumErrorCount),
    "spiChecksumErrorCount": int(panda_state.spiChecksumErrorCount),
    "harnessStatus": enum_name(panda_state.harnessStatus),
    "controlsAllowed": bool(panda_state.controlsAllowed),
    "controlsAllowedLateral": getattr(panda_state, "controlsAllowedLateral", None),
    "controlsAllowedLongitudinal": getattr(panda_state, "controlsAllowedLongitudinal", None),
    "safetyModel": enum_name(panda_state.safetyModel),
    "safetyParam": int(panda_state.safetyParam),
    "safetyRxInvalid": int(panda_state.safetyRxInvalid),
    "safetyTxBlocked": int(panda_state.safetyTxBlocked),
    "safetyRxChecksInvalid": bool(panda_state.safetyRxChecksInvalid),
  }


def vehicle_details_payload(sm, started):
  if not started:
    return {"available": False, "reason": "offroad"}
  if not sample_fresh(sm, "carState"):
    return {"available": False, "reason": "carState_unavailable"}

  try:
    car_state = sm["carState"]
    cruise = car_state.cruiseState
    speed = car_state_speed_payload(car_state)
    return {
      "available": True,
      "speedMps": speed["speedMps"],
      "speedKph": speed["speedMps"] * 3.6,
      "speedSource": speed["source"],
      "rawSpeedMps": float(car_state.vEgoRaw),
      "accelerationMps2": float(car_state.aEgo),
      "yawRateRadPerSec": float(car_state.yawRate),
      "standstill": bool(car_state.standstill),
      "gear": enum_name(car_state.gearShifter),
      "steeringAngleDeg": float(car_state.steeringAngleDeg),
      "steeringRateDegPerSec": float(car_state.steeringRateDeg),
      "steeringPressed": bool(car_state.steeringPressed),
      "gasPressed": bool(car_state.gasPressed),
      "brakePressed": bool(car_state.brakePressed),
      "parkingBrake": bool(car_state.parkingBrake),
      "brakeHoldActive": bool(car_state.brakeHoldActive),
      "leftBlinker": bool(car_state.leftBlinker),
      "rightBlinker": bool(car_state.rightBlinker),
      "leftBlindspot": bool(car_state.leftBlindspot),
      "rightBlindspot": bool(car_state.rightBlindspot),
      "doorOpen": bool(car_state.doorOpen),
      "seatbeltUnlatched": bool(car_state.seatbeltUnlatched),
      "fuelGauge": float(car_state.fuelGauge),
      "charging": bool(car_state.charging),
      "can": {
        "valid": bool(car_state.canValid),
        "timeout": bool(car_state.canTimeout),
        "errorCounter": int(car_state.canErrorCounter),
      },
      "steeringFault": {
        "temporary": bool(car_state.steerFaultTemporary),
        "permanent": bool(car_state.steerFaultPermanent),
      },
      "cruise": {
        "enabled": bool(cruise.enabled),
        "available": bool(cruise.available),
        "standstill": bool(cruise.standstill),
        "nonAdaptive": bool(cruise.nonAdaptive),
        "speedMps": float(cruise.speed),
        "speedKph": float(cruise.speed) * 3.6,
        "clusterSpeedMps": float(cruise.speedCluster),
      },
    }
  except Exception:
    return {"available": False, "reason": "carState_unavailable"}


def openpilot_details_payload(sm, started=True):
  if not started:
    return {"available": False, "reason": "offroad", "state": "disabled",
            "enabled": False, "active": False, "engageable": False}
  if not sample_fresh(sm, "selfdriveState"):
    return {"available": False, "reason": "selfdriveState_unavailable",
            "enabled": False, "active": False, "engageable": False}
  try:
    state = sm["selfdriveState"]
    return {
      "available": True,
      "state": enum_name(state.state),
      "enabled": bool(state.enabled),
      "active": bool(state.active),
      "engageable": bool(state.engageable),
      "experimentalMode": bool(state.experimentalMode),
      "personality": enum_name(state.personality),
      "alert": {
        "text1": str(state.alertText1),
        "text2": str(state.alertText2),
        "type": str(state.alertType),
        "status": enum_name(state.alertStatus),
        "size": enum_name(state.alertSize),
        "sound": enum_name(state.alertSound),
        "hudVisual": enum_name(state.alertHudVisual),
      },
    }
  except Exception:
    return {"available": False}


def first_panda_state(panda_states):
  for panda_state in panda_states:
    if panda_state.pandaType != log.PandaState.PandaType.unknown:
      return panda_state
  return panda_states[0] if len(panda_states) else None


def current_ma_from_raw(raw_current):
  if raw_current is None:
    return None

  current = float(raw_current)
  return current / 1000.0 if current > 10000.0 else current


def voltage_v_from_raw(raw_voltage):
  if raw_voltage is None or isinstance(raw_voltage, bool):
    return None
  try:
    voltage_mv = float(raw_voltage)
  except (TypeError, ValueError, OverflowError):
    return None
  # Panda health reports millivolts, including readings below 100 mV.
  return voltage_mv / 1000.0 if math.isfinite(voltage_mv) and voltage_mv >= 0 else None


def power_input_payload(panda_state, panda):
  """Only fresh, valid Panda samples may reach this adapter."""
  device_input = {
    "source": "pandaStates",
    "fresh": panda is not None,
    "voltageMv": int(panda_state.voltage) if panda_state is not None else None,
    "voltageV": panda.get("voltageV") if panda is not None else None,
    "currentMa": panda.get("currentMa") if panda is not None else None,
    "powerW": panda.get("estimatedPowerW") if panda is not None else None,
    "harnessStatus": panda.get("harnessStatus") if panda is not None else None,
  }
  voltage = device_input["voltageV"]
  if panda is None:
    reason = "panda_unavailable"
  elif panda.get("type") in (None, "", "unknown"):
    reason = "panda_type_unknown"
  elif device_input["harnessStatus"] not in ("normal", "flipped"):
    reason = "harness_not_connected"
  elif voltage is None or voltage <= 0:
    reason = "voltage_unavailable"
  else:
    reason = None
  vehicle_input = {
    "source": "pandaStates.voltage",
    "kind": "vehicleInputEstimate",
    "fresh": device_input["fresh"],
    "qualified": reason is None,
    "reason": reason,
    "voltageV": voltage if reason is None else None,
  }
  return device_input, vehicle_input


def car_state_speed_payload(car_state):
  v_ego_cluster = float(car_state.vEgoCluster)
  if v_ego_cluster != 0.0:
    return {"speedMps": max(0.0, v_ego_cluster), "source": "vEgoCluster"}
  return {"speedMps": max(0.0, float(car_state.vEgo)), "source": "vEgo"}


def vehicle_speed_payload(sm, started):
  if not started:
    return {"speedMps": 0.0, "source": "offroad"}
  if not sample_fresh(sm, "carState"):
    return {}

  try:
    return car_state_speed_payload(sm["carState"])
  except Exception:
    return {}


def last_gps_payload(params_memory=None):
  try:
    params_memory = Params("/dev/shm/params") if params_memory is None else params_memory
    # Open the same inode for the value and mtime, including during atomic puts.
    with Path(params_memory.get_param_path("LastGPSPosition")).open("rb") as stream:
      location = stream.read(4097)
      timestamp_millis = int(os.fstat(stream.fileno()).st_mtime * 1000)
    if len(location) > 4096:
      return {}
    if isinstance(location, bytes):
      location = location.decode("utf-8", "replace")
    data = json.loads(location or "{}")
  except Exception:
    return {}

  if not isinstance(data, dict) or not valid_gps_coordinates(data.get("latitude"), data.get("longitude")):
    return {}
  try:
    bearing = float(data.get("bearing", 0.0))
    if not math.isfinite(bearing):
      return {}
  except (TypeError, ValueError):
    return {}
  return {
    "latitude": float(data["latitude"]),
    "longitude": float(data["longitude"]),
    "bearingDeg": bearing,
    "source": "lastGpsPosition",
    "timestampMillis": timestamp_millis,
    "timestampSource": "paramsMtime",
  }


def valid_gps_coordinates(latitude, longitude):
  return (type(latitude) in (int, float) and type(longitude) in (int, float)
          and math.isfinite(latitude) and math.isfinite(longitude)
          and -90 <= latitude <= 90 and -180 <= longitude <= 180
          and not (abs(latitude) <= 0.001 and abs(longitude) <= 0.001))


def gps_payload(sm, params):
  candidates = []
  now_monotonic = time.monotonic()
  now_millis = int(time.time() * 1000)
  for socket in ("gpsLocationExternal", "gpsLocation"):
    try:
      gps = sm[socket]
      timestamp_millis = int(gps.unixTimestampMillis)
      timestamp_fresh = timestamp_millis > 0 and abs(now_millis - timestamp_millis) <= GPS_TIMESTAMP_MAX_AGE_MS
      if (sample_fresh(sm, socket, now=now_monotonic, max_age=GPS_SERVICE_MAX_AGE_S) and timestamp_fresh
          and gps.hasFix and valid_gps_coordinates(gps.latitude, gps.longitude)):
        candidates.append((sm.recv_time.get(socket, 0), gps))
    except Exception:
      continue

  if not candidates:
    fallback = last_gps_payload()
    if fallback:
      timestamp_millis = fallback.get("timestampMillis", 0)
      # A recently written last-known position is not a current GPS service fix.
      fallback["fresh"] = False
      fallback["recent"] = bool(timestamp_millis and 0 <= now_millis - timestamp_millis <= LAST_GPS_POSITION_MAX_AGE_MS)
      return fallback
    return {"fresh": False, "source": "unavailable"}

  _, gps = max(candidates, key=lambda item: item[0])
  return {
    "latitude": float(gps.latitude),
    "longitude": float(gps.longitude),
    "altitude": float(gps.altitude),
    "bearingDeg": float(gps.bearingDeg),
    "accuracyM": float(gps.horizontalAccuracy),
    "timestampMillis": int(gps.unixTimestampMillis),
    "source": enum_name(gps.source),
    "satellites": int(gps.satelliteCount),
    "fresh": True,
  }


def resolve_onroad_state(device_state_started: bool, started_override: bool | None = None) -> bool:
  return bool(device_state_started) if started_override is None else bool(started_override)


def telemetry_payload(sm, params, device_id, started_override: bool | None = None):
  device_state = sm["deviceState"]
  device_fresh = sample_fresh(sm, "deviceState")
  panda_state = first_panda_state(sm["pandaStates"]) if sample_fresh(sm, "pandaStates") else None
  started = resolve_onroad_state(device_fresh and device_state.started, started_override)
  vehicle_speed = vehicle_speed_payload(sm, started)

  ignition = None
  if panda_state is not None:
    ignition = bool(panda_state.ignitionLine or panda_state.ignitionCan)

  device = device_details_payload(device_state) if device_fresh else None
  panda = panda_details_payload(panda_state)
  device_input, vehicle_input = power_input_payload(panda_state, panda)
  vehicle = vehicle_details_payload(sm, started)
  openpilot = openpilot_details_payload(sm, started)
  gps = gps_payload(sm, params)
  services = list(getattr(sm, "services", TELEMETRY_SERVICES if started else STATE_SERVICES))
  service_timestamps = service_timestamps_payload(sm, services)
  connections = connection_diagnostics(sm, services, gps, panda, vehicle, openpilot)
  connected = (connections.get("comma", {}).get("state") == "ok"
               and connections.get("vehicleCan", {}).get("state") in ("ok", "inactive"))
  system_state = resolve_operating_state(
    connected=connected,
    onroad=started,
    door_open=bool(vehicle.get("doorOpen", False)),
    gear=str(vehicle.get("gear") or "unknown"),
  )

  return {
    "schemaVersion": "wayon-telemetry-v3",
    "deviceId": device_id,
    "updatedAt": utc_now(),
    "timestamps": {
      "generatedAt": utc_now(),
      "services": service_timestamps,
    },
    "connections": connections,
    "systemState": system_state,
    "onroad": started,
    "ignition": ignition,
    "enabled": bool(openpilot.get("enabled", False)),
    # Legacy clients interpret voltageV as a vehicle measurement.
    "voltageV": vehicle_input["voltageV"],
    "deviceInput": device_input,
    "vehicleInput": vehicle_input,
    "currentMa": device_input["currentMa"],
    "powerW": device_input["powerW"],
    "devicePowerW": float(device_state.powerDrawW) if device_fresh else None,
    "thermalStatus": enum_name(device_state.thermalStatus) if device_fresh else None,
    "fanPercent": int(device_state.fanSpeedPercentDesired) if device_fresh else None,
    "screenBrightnessPercent": int(device_state.screenBrightnessPercent) if device_fresh else None,
    "gps": gps,
    "vehicleSpeedMps": vehicle_speed.get("speedMps"),
    "vehicleSpeedSource": vehicle_speed.get("source"),
    "dongleId": get_param_str(params, "DongleId"),
    "device": device,
    "panda": panda,
    "vehicle": vehicle,
    "openpilot": openpilot,
    "cutInRisk": radar_cutin_payload(sm) if started else cutin_risk_stage(0.0),
  }


def fresh_telemetry_payload(params, device_id, started):
  services = TELEMETRY_SERVICES if started else STATE_SERVICES
  sm = messaging.SubMaster(services)
  deadline = time.monotonic() + (2.0 if started else 1.0)
  while time.monotonic() < deadline:
    sm.update(100)
    required = ("deviceState", "pandaStates", "carState", "selfdriveState") if started else ("deviceState", "pandaStates")
    if all(sample_fresh(sm, name) for name in required):
      break
  return telemetry_payload(sm, params, device_id, started_override=started)


def connection_status_signature(payload):
  return tuple(sorted(
    (name, str(item.get("state") or "unknown"))
    for name, item in (payload.get("connections") or {}).items()
    if isinstance(item, dict)
  ))


def health_event_from_telemetry(payload, previous_signature=None):
  signature = connection_status_signature(payload)
  vehicle_input = payload.get("vehicleInput") or {}
  voltage = (vehicle_input.get("voltageV") if vehicle_input.get("qualified") is True
             and vehicle_input.get("fresh") is True and vehicle_input.get("kind") == "vehicleInputEstimate" else None)
  thermal = str(payload.get("thermalStatus") or "").lower()
  panda = payload.get("panda") or {}
  cutin = payload.get("cutInRisk") or {}
  issues = []
  severity = "normal"
  if voltage is not None and float(voltage) <= 11.5:
    issues.append(f"vehicle input estimate {float(voltage):.1f}V")
    severity = "critical"
  elif voltage is not None and float(voltage) < 11.9:
    issues.append(f"vehicle input estimate {float(voltage):.1f}V")
    severity = "warning"
  if thermal in ("yellow", "red", "danger", "overheated"):
    issues.append(f"device thermal {thermal}")
    severity = "critical" if thermal in ("red", "danger", "overheated") else "warning"
  if str(panda.get("faultStatus") or "none").lower() not in ("", "none", "normal"):
    issues.append(f"safety interface {panda.get('faultStatus')}")
    severity = "warning" if severity == "normal" else severity
  cutin_level = int(cutin.get("level") or 0)
  if cutin_level >= 2:
    issues.append(f"cut-in risk {cutin.get('name') or cutin_level}")
    severity = "critical" if cutin_level >= 3 else "warning"
  health_signature = (signature, tuple(issues), severity)
  if health_signature == previous_signature:
    return None, health_signature
  faults = [name for name, state in signature if state not in ("ok", "inactive")]
  details = faults + issues
  return {
    "id": f"health-{payload.get('deviceId', 'unknown')}-{int(time.time() * 1000)}",
    "deviceId": payload.get("deviceId"),
    "occurredAt": payload.get("updatedAt") or utc_now(),
    "category": "vehicle_health" if issues else "connection",
    "severity": "warning" if faults and severity == "normal" else severity,
    "title": "Vehicle attention required" if issues else "Connection degraded" if faults else "Connections restored",
    "detail": ", ".join(details) if details else "All monitored connections are healthy",
    "snapshot": {
      "connections": payload.get("connections") or {},
      "voltageV": voltage,
      "vehicleInput": vehicle_input,
      "thermalStatus": payload.get("thermalStatus"),
      "systemState": payload.get("systemState"),
    },
  }, health_signature


def haversine_m(a, b):
  lat1 = math.radians(a["latitude"])
  lon1 = math.radians(a["longitude"])
  lat2 = math.radians(b["latitude"])
  lon2 = math.radians(b["longitude"])
  d_lat = lat2 - lat1
  d_lon = lon2 - lon1
  h = math.sin(d_lat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
  return 6371000.0 * 2.0 * math.atan2(math.sqrt(h), math.sqrt(1.0 - h))


def route_distance_m(route):
  return sum(haversine_m(route[i - 1], route[i]) for i in range(1, len(route)))


def read_route_state():
  try:
    with ROUTE_STATE_PATH.open("r", encoding="utf-8") as f:
      state = json.load(f)
  except FileNotFoundError:
    return {"uploaded_routes": []}
  except Exception as exc:
    print(f"Wayon cloud: failed to read route state: {exc}")
    return {"uploaded_routes": []}

  if not isinstance(state, dict):
    return {"uploaded_routes": []}
  uploaded_routes = state.get("uploaded_routes", [])
  if not isinstance(uploaded_routes, list):
    uploaded_routes = []
  progress = state.get("route_uploads", {})
  return {"uploaded_routes": [str(route) for route in uploaded_routes][:100],
          "route_uploads": {key: value for key, value in list(progress.items())[:100] if isinstance(value, dict)}
          if isinstance(progress, dict) else {}}


def write_route_state(state):
  try:
    ROUTE_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = ROUTE_STATE_PATH.with_suffix(f"{ROUTE_STATE_PATH.suffix}.tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
      json.dump(state, f, ensure_ascii=False, separators=(",", ":"))
    tmp_path.replace(ROUTE_STATE_PATH)
  except Exception as exc:
    print(f"Wayon cloud: failed to write route state: {exc}")
    raise


def save_route_progress(route_name, signature, packages, *, complete=False, uploaded=False, receipt=None, skip_reason=None):
  state = read_route_state()
  progress = state.get("route_uploads", {})
  entry = {"signature": signature, "packages": packages, "complete": complete,
           "retryAfter": 0 if complete else time.time() + ROUTE_RETRY_INTERVAL_S}
  confirmed = receipt if receipt is not None else progress.get(route_name, {}).get("confirmedReport")
  if isinstance(confirmed, dict):
    entry["confirmedReport"] = confirmed
  if skip_reason is not None:
    entry["skipReason"] = skip_reason
  state["route_uploads"] = dict(list({route_name: entry, **{k: v for k, v in progress.items() if k != route_name}}.items())[:100])
  if uploaded:
    state["uploaded_routes"] = ([route_name] + [r for r in state.get("uploaded_routes", []) if r != route_name])[:100]
  write_route_state(state)


def mark_route_uploaded(route_name, signature=None, packages=None, complete=True, receipt=None):
  if signature is not None:
    save_route_progress(route_name, signature, packages or {}, complete=complete, uploaded=True, receipt=receipt)
    return
  state = read_route_state()
  uploaded_routes = [route for route in state.get("uploaded_routes", []) if route != route_name]
  uploaded_routes.insert(0, route_name)
  state["uploaded_routes"] = uploaded_routes[:100]
  write_route_state(state)


def parse_segment_dir_name(name):
  try:
    route_name, segment = name.rsplit("--", 1)
  except ValueError:
    return None

  if not segment.isdigit() or not route_name:
    return None
  return route_name, int(segment)


def route_started_at(route_name):
  # Legacy date names carry UTC; Carrot's counter--random-ID names do not.
  try:
    return datetime.strptime(route_name, "%Y-%m-%d--%H-%M-%S").replace(tzinfo=timezone.utc)
  except ValueError:
    return None


def iso_from_timestamp_ms(timestamp_ms):
  try:
    timestamp_ms = int(timestamp_ms)
  except Exception:
    return None

  if timestamp_ms <= 0:
    return None
  try:
    return datetime.fromtimestamp(timestamp_ms / 1000.0, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
  except (ValueError, OverflowError, OSError):
    return None


def segment_log_file(segment_dir):
  for filename in LOG_FILE_CANDIDATES:
    path = segment_dir / filename
    if path.is_file() and 0 < path.stat().st_size <= ROUTE_LOG_MAX_BYTES:
      return path
  return None


def route_summary_allowed(params=None):
  try:
    params = Params() if params is None else params
    return (not params.get_bool("IsOnroad") and params.get_bool("IsOffroad")
            and psutil.virtual_memory().available >= ROUTE_MIN_AVAILABLE_BYTES)
  except Exception:
    return False


def route_log_messages(path, *, budget=None):
  """Bound one local qlog, without LogReader's whole-file event cache/sort."""
  import bz2
  import zstandard as zstd

  path = Path(path)
  budget = budget or WorkBudget(route_summary_allowed, ROUTE_WORK_LIMITS)
  budget.check()
  if path.name not in LOG_FILE_CANDIDATES or not route_summary_allowed():
    raise ValueError("route summary unavailable")
  if path.is_symlink() or path.parent.is_symlink() or any(path.parent.glob("*.lock")):
    raise WorkAborted("route file is linked or still recording")
  with path.open("rb") as stream:
    compressed = stream.read(ROUTE_LOG_MAX_BYTES + 1)
  if len(compressed) > ROUTE_LOG_MAX_BYTES:
    raise ResourceLimitExceeded("route qlog exceeds compressed limit")
  source = io.BytesIO(compressed)
  if path.suffix == ".zst":
    source = zstd.ZstdDecompressor(max_window_size=ROUTE_LOG_MAX_DECOMPRESSED_BYTES).stream_reader(source)
  elif path.suffix == ".bz2":
    source = bz2.BZ2File(source)
  with source, io.BytesIO() as output:
    while True:
      if not route_summary_allowed():
        raise RuntimeError("route summary interrupted")
      budget.check()
      chunk = source.read(min(1024 * 1024, ROUTE_LOG_MAX_DECOMPRESSED_BYTES + 1 - output.tell()))
      if not chunk:
        break
      output.write(chunk)
      budget.decoded_bytes += len(chunk)
      if budget.decoded_bytes > budget.limits.max_total_decoded_bytes:
        raise ResourceLimitExceeded("route decoded byte limit exceeded")
      if output.tell() > ROUTE_LOG_MAX_DECOMPRESSED_BYTES:
        raise ResourceLimitExceeded("route qlog exceeds decompressed limit")
    raw = output.getvalue()
  for index, message in enumerate(log.Event.read_multiple_bytes(raw)):
    if index % 128 == 0 and not route_summary_allowed():
      raise RuntimeError("route summary interrupted")
    if index % 128 == 0:
      budget.check()
    budget.messages += 1
    if budget.messages > budget.limits.max_messages:
      raise ResourceLimitExceeded("route message limit exceeded")
    yield message
  budget.check()


def latest_route_gps_payload(max_age_s=DEFAULT_LATEST_ROUTE_GPS_MAX_AGE):
  try:
    from openpilot.tools.lib.logreader import LogReader

    log_root = Path(Paths.log_root())
    if not log_root.is_dir():
      return {}

    now = time.time()
    segment_dirs = sorted(
      (path for path in log_root.iterdir() if path.is_dir()),
      key=lambda path: path.stat().st_mtime,
      reverse=True,
    )

    latest = None
    for segment_dir in segment_dirs[:8]:
      log_file = segment_log_file(segment_dir)
      if log_file is None:
        continue
      if now - max(segment_dir.stat().st_mtime, log_file.stat().st_mtime) > max_age_s:
        continue

      for msg in LogReader(str(log_file), sort_by_time=True, only_union_types=True):
        which = msg.which()
        if which not in ("gpsLocationExternal", "gpsLocation"):
          continue
        gps = getattr(msg, which)
        if not gps.hasFix or abs(gps.latitude) <= 0.001 or abs(gps.longitude) <= 0.001:
          continue

        latest = {
          "latitude": float(gps.latitude),
          "longitude": float(gps.longitude),
          "altitude": float(gps.altitude),
          "bearingDeg": float(gps.bearingDeg),
          "accuracyM": float(gps.horizontalAccuracy),
          "timestampMillis": int(gps.unixTimestampMillis),
          "source": f"routeLog:{which}",
          "satellites": int(gps.satelliteCount),
        }

      if latest:
        return latest
  except Exception as exc:
    print(f"Wayon cloud: failed to read latest route GPS: {exc}")

  return {}


def recent_route_groups(max_age_s):
  log_root = Path(Paths.log_root())
  if not log_root.is_dir():
    return []

  now = time.time()
  groups = {}
  for segment_dir in log_root.iterdir():
    if not segment_dir.is_dir():
      continue

    parsed = parse_segment_dir_name(segment_dir.name)
    if parsed is None:
      continue

    route_name, segment = parsed
    log_file = segment_log_file(segment_dir)
    if log_file is None:
      continue

    mtime = max(segment_dir.stat().st_mtime, log_file.stat().st_mtime)
    if now - mtime > max_age_s:
      continue

    group = groups.setdefault(route_name, {"route_name": route_name, "segments": [], "mtime": 0.0})
    group["segments"].append((segment, segment_dir, log_file))
    group["mtime"] = max(group["mtime"], mtime)

  route_groups = []
  for group in groups.values():
    group["segments"].sort(key=lambda item: item[0])
    route_groups.append(group)

  return sorted(route_groups, key=lambda item: item["mtime"], reverse=True)


def downsample_route(route, limit):
  if len(route) <= limit:
    return route
  if limit < 2:
    return route[:limit]

  step = (len(route) - 1) / float(limit - 1)
  return [route[round(i * step)] for i in range(limit)]


def gps_point_from_log(gps, timestamp_offset_s, route_start, speed_payload):
  if not gps.hasFix or not valid_gps_coordinates(gps.latitude, gps.longitude):
    return None

  point_time = iso_from_timestamp_ms(gps.unixTimestampMillis)
  if point_time is None and route_start is not None and timestamp_offset_s is not None:
    point_time = datetime.fromtimestamp(route_start.timestamp() + timestamp_offset_s, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")

  point = {
    "time": point_time,
    "latitude": float(gps.latitude),
    "longitude": float(gps.longitude),
    "speedMps": speed_payload.get("speedMps") if speed_payload else max(0.0, float(gps.speed)),
    "speedSource": speed_payload.get("source") if speed_payload else "gps",
    "bearingDeg": float(gps.bearingDeg),
  }

  try:
    point["accuracyM"] = float(gps.horizontalAccuracy)
  except Exception:
    pass
  return point


def should_add_route_point(route, point, last_point_mono, point_mono, point_interval_s, min_distance_m):
  if not route:
    return True
  if point_mono is not None and last_point_mono is not None and point_mono - last_point_mono >= point_interval_s:
    return True
  return haversine_m(route[-1], point) >= min_distance_m


def summarize_route_from_logs(route_group, config, device_id, *, allowed=None):
  if not route_summary_allowed():
    return None
  allowed = route_summary_allowed if allowed is None else allowed
  budget = WorkBudget(allowed, ROUTE_WORK_LIMITS)
  deadline = budget.deadline
  segments = route_group["segments"]
  indices = [segment for segment, _, _ in segments]
  if (not segments or indices[0] != 0 or len(segments) > budget.limits.max_route_segments
      or any(type(index) is not int or not 0 <= index < budget.limits.max_route_segments for index in indices)
      or indices != sorted(set(indices))):
    return None

  route_name = route_group["route_name"]
  route_start = route_started_at(route_name)
  route = []
  speed_samples = []
  first_mono = None
  last_mono = None
  last_point_mono = None
  last_vehicle_speed = None
  epoch_offset = None
  timestamp_source = None
  stop_quality = StopQualityTracker()
  active_duration_s = 0.0
  last_selfdrive_mono = None
  last_selfdrive_active = False
  cutin_stage_counts = {"watch": 0, "prepare": 0, "brake": 0}
  cutin_stage_max = 0
  cutin_event_count = 0
  previous_cutin_stage = 0
  accumulator = None
  read_segments = []
  partial_reasons = set()
  decoded_bytes = 0
  message_count = 0
  started_mono = time.monotonic()

  point_interval_s = float(config.get("route_point_interval_s", DEFAULT_ROUTE_POINT_INTERVAL))
  min_distance_m = float(config.get("route_point_min_distance_m", DEFAULT_ROUTE_POINT_MIN_DISTANCE_M))
  point_limit = int(config.get("route_point_limit", DEFAULT_ROUTE_POINT_LIMIT))

  for segment, _, log_file in segments:
    # Bound each file independently; keep one route CPU deadline and preserve
    # observations with explicit coverage if a local resource bound is reached.
    budget = WorkBudget(allowed, ROUTE_WORK_LIMITS)
    budget.deadline = deadline
    read_messages = 0
    try:
      budget.check()
      for msg in route_log_messages(log_file, budget=budget):
        read_messages += 1
        which = msg.which()
        msg_mono = float(msg.logMonoTime) / 1e9
        if not msg.valid or not math.isfinite(msg_mono) or msg_mono < 0:
          continue
        first_mono = msg_mono if first_mono is None else first_mono
        last_mono = msg_mono if last_mono is None else max(last_mono, msg_mono)
        if accumulator is None:
          accumulator = DriveReportAccumulator(route_start, route_first_mono=first_mono)
        accumulator.update(msg)

        if which in ("initData", "clocks"):
          wall_ms = getattr(msg, which).wallTimeNanos / 1e6
          if epoch_offset is None and iso_from_timestamp_ms(wall_ms) is not None:
            epoch_offset = wall_ms / 1000.0 - msg_mono
            timestamp_source = which
          continue

        if which == "carState":
          try:
            last_vehicle_speed = car_state_speed_payload(msg.carState)
            if last_vehicle_speed.get("speedMps") is not None:
              speed_samples.append(float(last_vehicle_speed["speedMps"]))
              stop_quality.update(
                msg_mono,
                last_vehicle_speed["speedMps"],
                float(msg.carState.aEgo),
                bool(msg.carState.standstill),
              )
          except Exception:
            pass
          continue

        if which == "selfdriveState":
          if last_selfdrive_mono is not None and last_selfdrive_active:
            active_duration_s += min(1.0, max(0.0, msg_mono - last_selfdrive_mono))
          last_selfdrive_active = bool(msg.selfdriveState.active)
          last_selfdrive_mono = msg_mono
          continue

        if which == "radarState":
          try:
            lead = msg.radarState.leadCutInRisk
            stage = cutin_risk_stage(
              lead.score if lead.status else 0.0,
              lead.dRel if lead.status else None,
              lead.vRel if lead.status else None,
            )
            level = int(stage["level"])
            cutin_stage_max = max(cutin_stage_max, level)
            if level > 0:
              cutin_stage_counts[stage["name"]] += 1
            if level >= 2 and previous_cutin_stage < 2:
              cutin_event_count += 1
            previous_cutin_stage = level
          except Exception:
            pass
          continue

        if which not in ("gpsLocationExternal", "gpsLocation"):
          continue

        gps = getattr(msg, which)
        if not msg.valid:
          continue
        timestamp_offset_s = None if first_mono is None else msg_mono - first_mono
        point = gps_point_from_log(gps, timestamp_offset_s, route_start, last_vehicle_speed)
        if point is None:
          continue
        if epoch_offset is None and iso_from_timestamp_ms(gps.unixTimestampMillis) is not None:
          epoch_offset = gps.unixTimestampMillis / 1000.0 - msg_mono
          timestamp_source = which

        if should_add_route_point(route, point, last_point_mono, msg_mono, point_interval_s, min_distance_m):
          point["_mono"] = msg_mono
          route.append(point)
          last_point_mono = msg_mono
      read_segments.append(segment)
    except ResourceLimitExceeded:
      if not allowed() or not route_summary_allowed():
        raise WorkAborted("route work no longer allowed")
      partial_reasons.add("resource_limit")
      if time.monotonic() >= deadline:
        break
    except WorkAborted:
      raise
    except Exception as exc:
      if not allowed() or not route_summary_allowed():
        raise WorkAborted("route work no longer allowed") from exc
      partial_reasons.add("log_read_error")
    finally:
      decoded_bytes += budget.decoded_bytes
      message_count += read_messages
    if first_mono is None:
      # A surviving later segment cannot establish segment zero's origin.
      return None

  if len(route) < 2 or not allowed() or not route_summary_allowed() or accumulator is None:
    return None

  if epoch_offset is None and route_start is not None and first_mono is not None:
    epoch_offset = route_start.timestamp() - first_mono
    timestamp_source = "routeName"
  # The v1 trips API replaces absent times with upload time. Do not send an
  # undated counter-named route, or mark it uploaded, until it has an epoch.
  if epoch_offset is None:
    return None
  for point in route:
    mono = point.pop("_mono")
    if point["time"] is None:
      point["time"] = iso_from_timestamp_ms((epoch_offset + mono) * 1000)
    if point["time"] is None:
      return None

  duration_s = 0
  if first_mono is not None and last_mono is not None:
    duration_s = max(0, int(last_mono - first_mono))

  distance_m = route_distance_m(route)
  started_at = iso_from_timestamp_ms((epoch_offset + first_mono) * 1000)
  ended_at = iso_from_timestamp_ms((epoch_offset + last_mono) * 1000)
  if started_at is None or ended_at is None:
    return None
  avg_speed_mps = distance_m / duration_s if duration_s > 0 else 0.0
  max_speed_mps = max(speed_samples) if speed_samples else max(float(point.get("speedMps") or 0.0) for point in route)
  payload_route = downsample_route(route, point_limit)
  stop_summary = stop_quality.summary()
  openpilot_active_percent = round(active_duration_s / duration_s * 100.0, 1) if duration_s > 0 else 0.0
  # UTC may arrive after the first carState (GPS/clocks). Rebase bounded report
  # rows before finalize attaches GPS context; never substitute upload time.
  accumulator.route_start = datetime.fromtimestamp(epoch_offset + first_mono, timezone.utc)
  for row in [*accumulator.moments, *accumulator.blackbox_events, *accumulator.timeline]:
    row["time"] = datetime.fromtimestamp(epoch_offset + first_mono + row["offsetS"], timezone.utc).isoformat(
      timespec="milliseconds").replace("+00:00", "Z")
  analysis = accumulator.finalize(payload_route)
  missing_segments = [index for index in range(indices[-1] + 1) if index not in indices]
  unread_segments = [index for index in indices if index not in read_segments]
  if missing_segments:
    partial_reasons.add("missing_segments")
  coverage = {"partial": bool(partial_reasons or unread_segments), "partialReasons": sorted(partial_reasons),
              "allListedSegmentsRead": not unread_segments, "readSegments": read_segments,
              "unreadSegments": unread_segments, "missingSegments": missing_segments,
              "observedStartAt": started_at, "observedEndAt": ended_at,
              "decodedBytes": decoded_bytes, "messagesRead": message_count,
              "processingDurationS": round(time.monotonic() - started_mono, 3)}
  analysis["dataQuality"].update(source="sampled_qlog", **coverage)

  return {
    "id": f"{device_id}-{route_name}",
    "deviceId": device_id,
    "startedAt": started_at,
    "endedAt": ended_at,
    "timestampSource": timestamp_source,
    "durationS": duration_s,
    "distanceM": distance_m,
    "startLat": route[0]["latitude"],
    "startLon": route[0]["longitude"],
    "endLat": route[-1]["latitude"],
    "endLon": route[-1]["longitude"],
    "source": "openpilotRoute",
    "routeName": route_name,
    "segmentCount": len(route_group["segments"]),
    "routePartial": coverage["partial"],
    "avgSpeedMps": avg_speed_mps,
    "maxSpeedMps": max_speed_mps,
    "report": {
      "schemaVersion": "wayon-trip-report-v1",
      "dataQuality": coverage,
      "openpilot": {
        "activeDurationS": round(active_duration_s, 1),
        "activePercent": openpilot_active_percent,
      },
      "stopQuality": stop_summary,
      "cutInRisk": {
        "eventCount": cutin_event_count,
        "maxLevel": cutin_stage_max,
        "sampleCounts": cutin_stage_counts,
      },
    },
    "route": payload_route,
    "analysis": analysis,
  }


def blackbox_upload_limits(config):
  return (int(bounded_interval(config.get("blackbox_upload_limit_per_route"), DEFAULT_BLACKBOX_UPLOAD_LIMIT_PER_ROUTE, 0, 4)),
          int(bounded_interval(config.get("blackbox_max_upload_bytes"), DEFAULT_BLACKBOX_MAX_UPLOAD_BYTES,
                               1, DEFAULT_BLACKBOX_MAX_UPLOAD_BYTES)))


def route_report_coverage(quality):
  if not isinstance(quality, dict) or type(quality.get("partial")) is not bool or type(quality.get("allListedSegmentsRead")) is not bool:
    return None
  coverage = {key: quality[key] for key in ("partial", "allListedSegmentsRead")}
  for key in ("readSegments", "unreadSegments", "missingSegments"):
    indices = quality.get(key)
    if (not isinstance(indices, list) or len(indices) > ROUTE_WORK_LIMITS.max_route_segments
        or any(type(index) is not int or not 0 <= index < ROUTE_WORK_LIMITS.max_route_segments for index in indices)
        or len(set(indices)) != len(indices)):
      return None
    coverage[key] = sorted(indices)
  if (set(coverage["readSegments"]) & (set(coverage["unreadSegments"]) | set(coverage["missingSegments"]))
      or coverage["allListedSegmentsRead"] != (not coverage["unreadSegments"])
      or (not coverage["partial"] and (not coverage["readSegments"] or coverage["unreadSegments"] or coverage["missingSegments"]))):
    return None
  start = ambient_timestamp(quality.get("observedStartAt"))
  end = ambient_timestamp(quality.get("observedEndAt"))
  if start is None or end is None or not start <= end:
    return None
  coverage.update(observedStartAt=quality["observedStartAt"], observedEndAt=quality["observedEndAt"])
  return coverage


def route_report_would_lose_coverage(receipt, payload):
  if (not isinstance(receipt, dict) or receipt.get("schema") != ROUTE_RECEIPT_SCHEMA
      or receipt.get("tripId") != payload.get("id")):
    return False
  previous = route_report_coverage(receipt.get("coverage"))
  if previous is None:
    # Old uploaded/complete flags do not prove how much of a route was read.
    return False
  current = route_report_coverage(payload.get("analysis", {}).get("dataQuality"))
  if current is None:
    return True
  return (not previous["partial"] and current["partial"]
          or not set(previous["readSegments"]).issubset(current["readSegments"])
          or ambient_timestamp(current["observedStartAt"]) > ambient_timestamp(previous["observedStartAt"])
          or ambient_timestamp(current["observedEndAt"]) < ambient_timestamp(previous["observedEndAt"]))


def route_upload_receipt(payload, signature):
  return {"schema": ROUTE_RECEIPT_SCHEMA, "tripId": payload["id"], "signature": signature,
          "analysisSchema": payload.get("analysis", {}).get("schemaVersion"), "confirmedAt": utc_now(),
          "coverage": route_report_coverage(payload.get("analysis", {}).get("dataQuality"))}


def route_upload_signature(route_group, config, *, allowed=None, device_id=None):
  budget = WorkBudget(route_summary_allowed if allowed is None else allowed, ROUTE_WORK_LIMITS)
  if len(route_group["segments"]) > budget.limits.max_route_segments:
    raise ResourceLimitExceeded("route segment limit exceeded")
  files = []
  for segment, directory, log_file in route_group["segments"]:
    budget.check()
    directory, log_file = Path(directory), Path(log_file)
    if directory.is_symlink() or any(directory.glob("*.lock")):
      raise WorkAborted("route is linked or still recording")
    for path in (log_file, *(directory / name for name in ("rlog", "rlog.bz2", "rlog.zst", "qcamera.ts"))):
      if path.is_symlink():
        raise WorkAborted("route file is linked")
      try:
        info = path.stat()
      except FileNotFoundError:
        if path == log_file:
          raise
        continue
      files.append((segment, path.name, info.st_size, info.st_mtime_ns))
  identity = [ROUTE_UPLOAD_SCHEMA, BLACKBOX_SCHEMA, device_id, bool(config.get("blackbox_auto_upload", True)),
              blackbox_upload_limits(config), files]
  return hashlib.sha256(json.dumps(identity, separators=(",", ":")).encode()).hexdigest()


def blackbox_archive_name(trip_id, event):
  # The packager's stable identity is trip + event + requested window, not
  # mutable rlog/video inventory. Also recognizes pre-v3 confirmed checkpoints.
  window = event["captureWindow"]
  identity = json.dumps([str(trip_id), str(event["id"]), float(window["startOffsetS"]), float(window["endOffsetS"])],
                        separators=(",", ":"))
  return "wayon-" + hashlib.sha256(identity.encode()).hexdigest()[:32] + ".zip"


def prepare_blackbox_archives(route_group, payload, config, device_id, *, packages=None, checkpoint=None, allowed=None):
  analysis = payload["analysis"]
  blackbox = analysis["blackbox"]
  events = blackbox["events"]
  if not events:
    return True
  budget = WorkBudget(route_summary_allowed if allowed is None else allowed)
  def work_allowed():
    return time.monotonic() < budget.deadline and budget.allowed()

  packages = {} if packages is None else packages
  event_ids = {event["id"] for event in events}
  for event_id in list(packages):
    if event_id not in event_ids:
      packages.pop(event_id)
  root = Path(config.get("blackbox_root") or DEFAULT_BLACKBOX_ROOT)
  upload_limit, max_bytes = blackbox_upload_limits(config)
  complete = True
  for index, event in enumerate(events):
    budget.check()
    cached = packages.get(event["id"], {})
    if (cached.get("status") == "uploaded" and cached.get("captureId")
        and cached.get("archiveName") == blackbox_archive_name(payload["id"], event)):
      event["package"] = dict(cached)
      continue
    if not config.get("blackbox_auto_upload", True) or index >= upload_limit:
      event["package"] = {"status": "pending", "reason": "auto_upload_disabled" if not config.get("blackbox_auto_upload", True) else "upload_limit"}
      continue
    try:
      bundle = build_blackbox_archive(route_group, event, payload["id"], output_root=root,
                                     route_first_mono=analysis["dataQuality"]["routeFirstMonoS"],
                                     allowed=work_allowed)
      budget.check()
      if bundle is None:
        event["package"] = {"status": "unavailable", "reason": "route_files_missing"}
        continue
      package = event["package"] = dict(bundle["package"])
      path = Path(bundle["path"])
      if path.is_symlink() or not path.is_file():
        raise WorkAborted("archive file unavailable")
      if path.stat().st_size > max_bytes:
        package["reason"] = "archive_too_large"
        continue
      with path.open("rb") as stream:
        data = stream.read(max_bytes + 1)
      if not data or len(data) > max_bytes:
        raise ResourceLimitExceeded("archive upload size limit exceeded")
      budget.check()
      window = event["captureWindow"]
      duration = float(window["endOffsetS"]) - float(window["startOffsetS"])
      if not math.isfinite(duration) or not 0 < duration <= 60.001:
        raise ValueError("invalid blackbox duration")
      captured_at = event.get("time") or payload["startedAt"]
      result = upload_live_capture(config, device_id, "clip", captured_at, "application/zip",
                                   CAMERA_LAYOUT, data, duration)
      capture_id = result.get("id") if isinstance(result, dict) else None
      if (not isinstance(result, dict) or result.get("ok") is not True
          or not isinstance(capture_id, str) or not 1 <= len(capture_id) <= 128):
        raise RuntimeError("capture upload returned no confirmed id")
      # Persist confirmed delivery even if the offroad guard changes immediately
      # after the request. It is not proof that the package contains playable video.
      package.update(status="uploaded", captureId=capture_id, uploadedAt=utc_now())
      package.pop("reason", None)
      packages[event["id"]] = dict(package)
      if checkpoint is not None:
        try:
          checkpoint()
        except OSError as exc:
          raise WorkAborted("capture checkpoint unavailable") from exc
      del data
      budget.check()
    except ResourceLimitExceeded:
      if not work_allowed() or not route_summary_allowed():
        raise WorkAborted("blackbox work no longer allowed")
      # Deterministic file/output limits are terminal for this route signature.
      # Publish the analysis; only changed inputs/policy should retry this event.
      event["package"] = {"status": "unavailable", "reason": "resource_limit"}
    except WorkAborted:
      raise
    except Exception as exc:
      complete = False
      event["package"] = {**event.get("package", {}), "status": "error", "reason": type(exc).__name__}
  blackbox["uploadedEventCount"] = sum(event.get("package", {}).get("status") == "uploaded" for event in events)
  blackbox["storedEventCount"] = sum(event.get("package", {}).get("status") in ("stored", "uploaded") for event in events)
  blackbox["unavailableEventCount"] = sum(event.get("package", {}).get("status") == "unavailable" for event in events)
  prune_blackbox_archives(root, allowed=work_allowed)
  budget.check()
  return complete


def upload_recent_route_summary(config, device_id):
  if not route_summary_allowed():
    return False
  state = read_route_state()
  max_age = bounded_interval(config.get("route_summary_max_age_s"), DEFAULT_ROUTE_SUMMARY_MAX_AGE, 60, DEFAULT_ROUTE_SUMMARY_MAX_AGE)
  grace = bounded_interval(config.get("route_summary_grace_period_s"), DEFAULT_ROUTE_SUMMARY_GRACE_PERIOD, 0, 3600)
  limit = int(bounded_interval(config.get("route_summary_limit_per_cycle"), 2, 1, 4))
  deadline = time.monotonic() + bounded_interval(config.get("route_summary_work_seconds"), 90, 1, 120)

  def allowed():
    return time.monotonic() < deadline and route_summary_allowed()

  uploaded = False
  attempted = 0
  for group in recent_route_groups(max_age)[:100]:
    if attempted >= limit or not allowed():
      break
    if not grace <= time.time() - group["mtime"] <= max_age:
      continue
    name = group["route_name"]
    try:
      signature = route_upload_signature(group, config, allowed=allowed, device_id=device_id)
      previous = state.get("route_uploads", {}).get(name, {})
      if previous.get("signature") == signature:
        if previous.get("complete") or previous.get("retryAfter", 0) > time.time():
          continue
      packages = previous.get("packages", {})
      packages = {key: dict(value) for key, value in packages.items()
                  if isinstance(value, dict) and value.get("status") == "uploaded" and value.get("captureId")} if isinstance(packages, dict) else {}
      attempted += 1
      save_route_progress(name, signature, packages)
      payload = summarize_route_from_logs(group, config, device_id, allowed=allowed)
      if payload is None or not allowed():
        continue
      if route_report_would_lose_coverage(previous.get("confirmedReport"), payload):
        # Complete only this rejected attempt, not a new upload receipt. Keep the
        # confirmed signature/coverage and capture IDs before touching packages.
        save_route_progress(name, signature, packages, complete=True, skip_reason="coverage_regression")
        continue
      complete = prepare_blackbox_archives(group, payload, config, device_id, packages=packages, allowed=allowed,
                                           checkpoint=lambda: save_route_progress(name, signature, packages))
      if not allowed():
        break
      if route_upload_signature(group, config, allowed=allowed, device_id=device_id) != signature:
        continue
      result = post_json(config, "/api/trips", payload)
      if not isinstance(result, dict) or result.get("ok") is not True or result.get("id") != payload["id"]:
        raise RuntimeError("trip upload returned no confirmation")
      receipt = route_upload_receipt(payload, signature)
      if not allowed():
        save_route_progress(name, signature, packages, receipt=receipt)
        break
      mark_route_uploaded(name, signature, packages, complete=complete, receipt=receipt)
      uploaded = True
      print(f"Wayon cloud: uploaded route analysis ({len(payload['route'])} route points)")
    except WorkAborted:
      if not allowed():
        break
    except Exception as exc:
      print(f"Wayon cloud: route analysis deferred ({type(exc).__name__})")
  return uploaded



def jpeg_base64(array, quality=72, subsampling=-1):
  return base64.b64encode(jpeg_bytes(array, quality=quality, subsampling=subsampling)).decode("ascii")


def enhance_wide_snapshot(image, target_median=WAYON_WIDE_SNAPSHOT_TARGET_MEDIAN,
                          min_gamma=WAYON_WIDE_SNAPSHOT_MIN_GAMMA):
  import numpy as np

  if image is None:
    return None

  frame = np.asarray(image)
  if frame.ndim != 3 or frame.shape[2] < 3 or frame.size == 0:
    return image

  rgb = frame[:, :, :3].astype(np.float32)
  luma = 0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1] + 0.0722 * rgb[:, :, 2]
  median = float(np.median(luma))
  target = max(1.0, min(254.0, float(target_median)))
  if not math.isfinite(median) or median <= 0.0 or median >= target:
    return image

  gamma = math.log(target / 255.0) / math.log(max(1.0, median) / 255.0)
  gamma = max(0.1, min(1.0, max(float(min_gamma), gamma)))
  corrected_luma = 255.0 * np.power(np.clip(luma / 255.0, 0.0, 1.0), gamma)
  gain = corrected_luma / np.maximum(luma, 1.0)

  enhanced = frame.copy()
  enhanced[:, :, :3] = np.clip(rgb * gain[:, :, None], 0.0, 255.0).astype(frame.dtype)
  return enhanced


def upload_offroad_snapshot(config, device_id, multiframe=True, approach_frames=None):
  try:
    from openpilot.system.wayon_camera import CameraGuard, capture_pair
    params = Params()
    burst = [] if multiframe and config.get("parking_multiframe_enabled", False) else None
    wide, driver = capture_pair(params, OFFROAD_SNAPSHOT_TIMEOUT_S, wide_frames=burst)
    if wide is None and driver is None:
      return False
    guard = CameraGuard(params)
    if not guard.wait_ready():
      return False
    frames = burst or [(time.time(), wide)]
    parking = None
    approach = []
    if config.get("parking_approach_enabled", False) and burst and len(burst) == 3:
      import uuid
      from openpilot.system.wayon_parking import upload_frames
      parked_at = frames[-1][0]
      approach = upload_frames(approach_frames or [], parked_at)
      parking = {"version": 1, "eventId": uuid.uuid4().hex,
                 "parkedAt": datetime.fromtimestamp(parked_at, timezone.utc).isoformat(timespec="milliseconds"),
                 "count": len(approach) + len(frames)}
    entries = [(at, raw, "approach", None) for at, raw, _ in approach]
    entries.extend((at, image, "parked", i) for i, (at, image) in enumerate(frames))
    for index, (captured_at, image, phase, parked_index) in enumerate(entries):
      if not guard.allowed():
        return False
      payload = {
        "deviceId": device_id,
        "capturedAt": (datetime.fromtimestamp(captured_at, timezone.utc).isoformat(timespec="milliseconds") if parking else
                       datetime.fromtimestamp(captured_at, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")),
        "wideJpegBase64": (base64.b64encode(image).decode("ascii") if phase == "approach" else
                           jpeg_base64(enhance_wide_snapshot(image), quality=92, subsampling=0) if image is not None else None),
        "driverJpegBase64": jpeg_base64(driver) if parked_index == 0 and driver is not None else None,
      }
      if parking is not None:
        payload["parking"] = {**parking, "index": index, "phase": phase}
      if not guard.allowed():
        return False
      post_json(config, "/api/snapshot", payload)
    print("Wayon cloud: uploaded offroad snapshot")
    return True
  except Exception as exc:
    print(f"Wayon cloud: snapshot upload failed: {exc}")
    return False


def capture_offroad_images():
  from openpilot.system.wayon_camera import capture_pair
  wide, driver = capture_pair(Params(), OFFROAD_SNAPSHOT_TIMEOUT_S)
  return enhance_wide_snapshot(wide), driver


def main():
  from openpilot.system.wayon_parking import ApproachSession
  set_core_affinity([0, 1, 2, 3])
  params = Params()
  sm = messaging.SubMaster(TELEMETRY_SERVICES)

  config = None
  next_config_load = 0.0
  next_telemetry = 0.0
  next_route_summary = 0.0
  next_snapshot = 0.0
  parking_burst_pending = True
  parking_approach = ApproachSession()
  pending_approach = []
  next_impact_upload = 0.0
  next_vehicle_event_upload = 0.0
  ambient_delivery = AmbientCommandDelivery()
  previous_started = False
  last_telemetry_signature = None
  last_telemetry_upload_at = 0.0
  last_connection_signature = None
  telemetry_backoff = UploadBackoff()
  route_backoff = UploadBackoff(base=60.0, maximum=900.0)

  while True:
    sm.update(1000)
    now = time.monotonic()

    if config is None or now >= next_config_load:
      config = read_config()
      next_config_load = now + CONFIG_RELOAD_INTERVAL
      if config is None:
        parking_approach.clear()
        pending_approach = []
        time.sleep(5.0)
        continue

    device_id = str(config.get("device_id") or get_param_str(params, "DongleId") or "unknown")
    try:
      ambient_delivery.run(now, lambda: poll_ambient_command(config, params, device_id))
    except Exception as exc:
      print(f"Wayon cloud: ambient command delivery failed: {exc}")

    if not sm.seen["deviceState"]:
      time.sleep(LOOP_SLEEP_OFFROAD)
      continue

    started = bool(sm["deviceState"].started)
    approach_enabled = (config.get("parking_approach_enabled", False)
                        and config.get("parking_multiframe_enabled", False)
                        and config.get("snapshots_enabled", True))
    if not approach_enabled:
      parking_approach.clear()
      pending_approach = []
    elif started:
      if not previous_started:
        parking_approach.clear()
        pending_approach = []
      parking_approach.sample(sm)
    elif previous_started:
      pending_approach = parking_approach.take()

    if now >= next_impact_upload:
      try:
        uploaded_impacts = upload_pending_impacts(config, device_id)
        next_impact_upload = now + (0.2 if uploaded_impacts >= 3 else 1.0)
      except Exception as exc:
        print(f"Wayon cloud: impact upload failed: {exc}")
        next_impact_upload = now + 15.0

    if now >= next_vehicle_event_upload:
      try:
        uploaded_vehicle_events = upload_pending_vehicle_events(config, device_id)
        next_vehicle_event_upload = now + (0.2 if uploaded_vehicle_events >= 3 else 1.0)
      except Exception as exc:
        print(f"Wayon cloud: vehicle event upload failed: {exc}")
        next_vehicle_event_upload = now + 15.0

    telemetry_default = DEFAULT_TELEMETRY_INTERVAL_ONROAD if started else DEFAULT_TELEMETRY_INTERVAL_OFFROAD
    telemetry_interval = bounded_interval(config.get(
      "telemetry_interval_onroad" if started else "telemetry_interval_offroad",
      telemetry_default,
    ), telemetry_default, 5.0 if started else 15.0, 3600.0)
    route_summary_interval = bounded_interval(config.get("route_summary_interval_offroad"),
                                              DEFAULT_ROUTE_SUMMARY_INTERVAL_OFFROAD, 30.0, 3600.0)
    route_summary_grace_period = bounded_interval(config.get("route_summary_grace_period_s"),
                                                  DEFAULT_ROUTE_SUMMARY_GRACE_PERIOD, 0.0, 3600.0)
    snapshot_interval = bounded_interval(config.get("snapshot_interval_offroad"),
                                         DEFAULT_SNAPSHOT_INTERVAL_OFFROAD, 300.0, 86400.0)

    if started and not previous_started:
      print(f"Wayon cloud: using {telemetry_interval:.0f}s lightweight onroad telemetry")
      next_telemetry = now

    if not started and previous_started:
      parking_burst_pending = True
      next_telemetry = now
      route_summary_due = now + route_summary_grace_period
      next_route_summary = route_summary_due if next_route_summary <= now else min(next_route_summary, route_summary_due)
      next_snapshot = min(next_snapshot, now + 30.0)

    if now >= next_telemetry:
      try:
        payload = telemetry_payload(sm, params, device_id, started_override=started)
        signature = telemetry_signature(payload)
        heartbeat_due = now - last_telemetry_upload_at >= max(5.0, telemetry_interval)
        if signature != last_telemetry_signature or heartbeat_due:
          post_json(config, "/api/telemetry", payload)
          last_telemetry_signature = signature
          last_telemetry_upload_at = now
          write_json_atomic(UPLOAD_HEALTH_PATH, {
            "lastSuccessAt": utc_now(),
            "lastReason": "state_changed" if not heartbeat_due else "heartbeat",
          })
          health_event, last_connection_signature = health_event_from_telemetry(
            payload, last_connection_signature)
          if health_event is not None:
            post_json(config, "/api/health-event", health_event)
        telemetry_backoff.success()
        next_telemetry = now + (5.0 if started else 15.0)
      except Exception as exc:
        print(f"Wayon cloud: telemetry upload failed: {exc}")
        write_json_atomic(UPLOAD_HEALTH_PATH, {
          "lastSuccessAt": read_json_file(UPLOAD_HEALTH_PATH).get("lastSuccessAt"),
          "lastFailureAt": utc_now(),
          "lastError": str(exc)[:240],
        })
        next_telemetry = time.monotonic() + telemetry_backoff.failure_delay()

    if not started and now >= next_route_summary:
      try:
        upload_recent_route_summary(config, device_id)
        route_backoff.success()
        next_route_summary = now + route_summary_interval
      except Exception as exc:
        print(f"Wayon cloud: route summary upload failed: {exc}")
        next_route_summary = time.monotonic() + max(route_summary_interval, route_backoff.failure_delay())

    if not started and config.get("snapshots_enabled", True) and now >= next_snapshot:
      if upload_offroad_snapshot(config, device_id, multiframe=parking_burst_pending,
                                 approach_frames=pending_approach):
        parking_burst_pending = False
        pending_approach = []
      next_snapshot = now + max(300.0, snapshot_interval)

    previous_started = started
    time.sleep(LOOP_SLEEP_ONROAD if started else LOOP_SLEEP_OFFROAD)


if __name__ == "__main__":
  main()
