import io
import json
import time
import uuid
import zipfile
from datetime import UTC, datetime

import requests


USER_AGENT = "wayon-live/1"
LIVE_CAPTURE_CHUNK_BYTES = 2 * 1024 * 1024


def utc_now() -> str:
  return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def select_recent_frames(frames, duration_s: float, now_s: float):
  if not frames:
    return []

  cutoff = now_s - duration_s
  start_index = None
  for index, frame in enumerate(frames):
    received_at, _payload, key_frame, _timestamp_us = frame
    if received_at <= cutoff and key_frame:
      start_index = index
    elif received_at > cutoff:
      break

  if start_index is None:
    start_index = next((index for index, frame in enumerate(frames) if frame[0] >= cutoff and frame[2]), None)
  if start_index is None:
    return []
  return [frame for frame in list(frames)[start_index:] if frame[0] <= now_s]


def frame_duration_s(frames) -> float:
  return max(0.0, frames[-1][0] - frames[0][0]) if len(frames) > 1 else 0.0


def build_dual_h264_archive(wide_frames, driver_frames, captured_at: str,
                            requested_duration_s: int, stream_metadata: dict) -> bytes:
  manifest = {
    "schema": "wayon-live-dual-h264-v1",
    "capturedAt": captured_at,
    "requestedDurationSeconds": requested_duration_s,
    "codec": stream_metadata.get("codec", "avc1.640020"),
    "fps": stream_metadata.get("fps", 20),
    "width": stream_metadata.get("width", 1344),
    "height": stream_metadata.get("height", 760),
    "panorama": stream_metadata.get("panorama", {}),
    "cameras": {
      "wide": {
        "file": "wide.h264",
        "frameCount": len(wide_frames),
        "durationSeconds": round(frame_duration_s(wide_frames), 3),
      },
      "driver": {
        "file": "driver.h264",
        "frameCount": len(driver_frames),
        "durationSeconds": round(frame_duration_s(driver_frames), 3),
      },
    },
  }

  output = io.BytesIO()
  with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED) as archive:
    archive.writestr("manifest.json", json.dumps(manifest, ensure_ascii=True, separators=(",", ":")))
    archive.writestr("wide.h264", b"".join(frame[1] for frame in wide_frames))
    archive.writestr("driver.h264", b"".join(frame[1] for frame in driver_frames))
  return output.getvalue()


def post_capture_request(endpoint: str, payload: bytes, headers: dict) -> dict:
  for attempt in range(3):
    response = None
    try:
      response = requests.post(endpoint, data=payload, headers=headers, timeout=(60, 180))
      response.raise_for_status()
      return response.json() if response.content else {"ok": True}
    except requests.RequestException as exc:
      status = response.status_code if response is not None else None
      retryable = status is None or status in (408, 429) or status >= 500
      if not retryable or attempt == 2:
        detail = response.text[:160] if response is not None else str(exc)
        raise RuntimeError(f"live capture upload failed ({status or 'network'}): {detail}") from exc
      time.sleep(2 ** attempt)

  raise RuntimeError("live capture upload failed")


def upload_live_capture_chunked(config: dict, device_id: str, captured_at: str,
                                camera_layout: str, payload: bytes, duration_s: float) -> dict:
  root = str(config["endpoint"]).rstrip("/")
  upload_id = uuid.uuid4().hex
  part_count = (len(payload) + LIVE_CAPTURE_CHUNK_BYTES - 1) // LIVE_CAPTURE_CHUNK_BYTES
  for part_index in range(part_count):
    start = part_index * LIVE_CAPTURE_CHUNK_BYTES
    part = payload[start:start + LIVE_CAPTURE_CHUNK_BYTES]
    post_capture_request(
      f"{root}/api/live-capture-part",
      part,
      {
        "Authorization": f"Bearer {config['token']}",
        "Content-Type": "application/octet-stream",
        "User-Agent": USER_AGENT,
        "X-Wayon-Device-Id": device_id,
        "X-Wayon-Upload-Id": upload_id,
        "X-Wayon-Part-Index": str(part_index),
        "X-Wayon-Part-Count": str(part_count),
        "X-Wayon-Total-Size": str(len(payload)),
      },
    )

  commit = json.dumps({
    "uploadId": upload_id,
    "deviceId": device_id,
    "capturedAt": captured_at,
    "durationS": duration_s,
    "cameraLayout": camera_layout,
    "partCount": part_count,
    "sizeBytes": len(payload),
  }, separators=(",", ":")).encode()
  return post_capture_request(
    f"{root}/api/live-capture-commit",
    commit,
    {
      "Authorization": f"Bearer {config['token']}",
      "Content-Type": "application/json",
      "User-Agent": USER_AGENT,
    },
  )


def upload_live_capture(config: dict, device_id: str, kind: str, captured_at: str,
                        content_type: str, camera_layout: str, payload: bytes,
                        duration_s: float | None = None) -> dict:
  if kind == "clip" and len(payload) > LIVE_CAPTURE_CHUNK_BYTES:
    if duration_s is None:
      raise ValueError("clip duration is required")
    return upload_live_capture_chunked(
      config, device_id, captured_at, camera_layout, payload, duration_s,
    )

  headers = {
    "Authorization": f"Bearer {config['token']}",
    "Content-Type": content_type,
    "User-Agent": USER_AGENT,
    "X-Wayon-Device-Id": device_id,
    "X-Wayon-Capture-Kind": kind,
    "X-Wayon-Captured-At": captured_at,
    "X-Wayon-Camera-Layout": camera_layout,
  }
  if duration_s is not None:
    headers["X-Wayon-Duration-S"] = f"{duration_s:.3f}"

  endpoint = f"{str(config['endpoint']).rstrip('/')}/api/live-capture"
  return post_capture_request(endpoint, payload, headers)
