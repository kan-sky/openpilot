"""Bounded, offroad-only camera requests shared by manager and hardwared."""
import json
import math
import os
import time
from pathlib import Path

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
REQUEST_PATH = Path(os.getenv("WAYON_CAMERA_REQUEST", "/tmp/wayon_camera.active"))
REQUEST_TTL = 3.0


def configured(path=CONFIG_PATH):
  try:
    config = json.loads(path.read_text())
    return (isinstance(config, dict) and config.get("enabled", True) is not False
            and str(config.get("endpoint", "")).startswith("https://")
            and str(config.get("token", "")).startswith("wayon_"))
  except (OSError, ValueError, TypeError):
    return False


def camera_request(path=REQUEST_PATH, now=None):
  try:
    request = json.loads(path.read_text())
    age = (time.monotonic() if now is None else now) - float(request["updated"])
    pid = int(request["pid"])
    if not 0 <= age <= REQUEST_TTL or pid <= 1 or request["mode"] not in ("panorama", "snapshot"):
      return None
    os.kill(pid, 0)
    return request
  except (OSError, ValueError, KeyError, TypeError, OverflowError):
    return None


def request_camera(mode, path=REQUEST_PATH):
  if mode not in ("panorama", "snapshot"):
    raise ValueError("unsupported camera mode")
  temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
  with os.fdopen(fd, "w") as stream:
    json.dump({"pid": os.getpid(), "updated": time.monotonic(), "mode": mode}, stream)
  os.replace(temporary, path)


def release_camera(path=REQUEST_PATH):
  try:
    request = json.loads(path.read_text())
    if isinstance(request, dict) and request.get("pid") == os.getpid():
      path.unlink(missing_ok=True)
  except (OSError, ValueError, TypeError):
    pass


def sample_fresh(sm, service, now=None, max_age=3.0):
  now = time.monotonic() if now is None else now
  return (sm.seen.get(service, False) and sm.valid.get(service, False)
          and 0 <= now - sm.recv_time.get(service, 0.0) <= max_age
          and 0 <= now - sm.logMonoTime.get(service, 0) / 1e9 <= max_age)


def camera_block_reason(params, sm, now=None):
  if params.get_bool("IsOnroad") or not params.get_bool("IsOffroad"):
    return "onroad"
  if params.get_bool("IsDriverViewEnabled"):
    return "driver_view"
  if not all(sample_fresh(sm, name, now) for name in ("deviceState", "pandaStates")):
    return "state_unavailable"
  device = sm["deviceState"]
  pandas = sm["pandaStates"]
  if device.started or any(p.ignitionLine or p.ignitionCan or p.controlsAllowed for p in pandas):
    return "ignition"
  if str(device.thermalStatus) not in ("green", "yellow"):
    return "temperature"
  if not math.isfinite(float(device.memoryUsagePercent)) or device.memoryUsagePercent >= 92:
    return "low_memory"
  return None


class CameraGuard:
  def __init__(self, params):
    from openpilot.cereal import messaging
    self.params = params
    self.sm = messaging.SubMaster(["deviceState", "pandaStates"])
    self.reason = "state_unavailable"

  def allowed(self, wait_ms=0):
    self.sm.update(wait_ms)
    self.reason = camera_block_reason(self.params, self.sm)
    return self.reason is None

  def wait_ready(self, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      if self.allowed(100):
        return True
      if self.reason != "state_unavailable":
        return False
    return False


def snapshot_frame_fresh(client, now):
  # Carrot CameraBuf initializes frame ID/SOF/EOF but not VisionIpcBufExtra.valid.
  # Keep this compatibility local to photos; never change model validity rules.
  eof = float(client.timestamp_eof) / 1e9
  return math.isfinite(eof) and eof > 0 and 0 <= now - eof <= 0.5


def snapshot_width(camera_index, native_width):
  return min(1928 if camera_index == 0 else 960, native_width)


def snapshot_capture_time(client):
  return time.time() - (time.monotonic() - float(client.timestamp_eof) / 1e9)


def capture_pair(params, timeout=25.0, wide_frames=None):
  """Request manager-owned cameras without blocking ignition or taking ownership of camerad."""
  import cv2
  import numpy as np
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  from openpilot.system.camera_lease import CameraLease
  from openpilot.system.wayon_live_stream import camera_busy

  guard = CameraGuard(params)
  if not guard.wait_ready() or camera_busy(params):
    return None, None
  lease = CameraLease("wayon_snapshot", timeout + 5)
  if not lease.acquire():
    return None, None
  try:
    if not guard.allowed() or camera_busy(params):
      return None, None
    clients = [VisionIpcClient("camerad", stream, True) for stream in (
      VisionStreamType.VISION_STREAM_WIDE_ROAD, VisionStreamType.VISION_STREAM_DRIVER)]
    connected = [False, False]
    images = [None, None]
    first_frame = [None, None]
    burst = []
    last_wide_at = 0.0
    wanted = 3 if wide_frames is not None else 1
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and guard.allowed(100):
      request_camera("snapshot")
      for i, client in enumerate(clients):
        if images[i] is not None and (i != 0 or len(burst) >= wanted):
          continue
        if not connected[i]:
          connected[i] = client.connect(False)
        if not connected[i]:
          continue
        frame = client.recv(timeout_ms=100)
        if frame is None:
          continue
        now = time.monotonic()
        if not snapshot_frame_fresh(client, now):
          continue
        if first_frame[i] is None:
          first_frame[i] = now
        if now - first_frame[i] < 4.0:
          continue
        if i == 0 and burst and now - last_wide_at < 1.0:
          continue
        # Decode NV12 with a bounded native conversion, not a full float64 RGB matrix.
        y = np.asarray(frame.data[:frame.uv_offset]).reshape(-1, frame.stride)[:frame.height, :frame.width]
        uv = np.asarray(frame.data[frame.uv_offset:frame.uv_offset + frame.stride * (frame.height // 2)])
        uv = uv.reshape(-1, frame.stride)[:frame.height // 2, :frame.width]
        rgb = cv2.cvtColor(np.vstack((y, uv)), cv2.COLOR_YUV2RGB_NV12)
        width = snapshot_width(i, frame.width)
        images[i] = rgb if width == frame.width else cv2.resize(rgb, (width, round(frame.height * width / frame.width)))
        if i == 0:
          burst.append((snapshot_capture_time(client), images[i]))
          last_wide_at = now
      if all(image is not None for image in images) and len(burst) >= wanted:
        if not guard.allowed():
          return None, None
        if wide_frames is not None:
          wide_frames.extend(burst)
        return tuple(images)
    return None, None
  finally:
    release_camera()
    lease.release()


def wayon_ready(started, params, CP):
  return configured()


def wayon_offroad(started, params, CP):
  return not started and params.get_bool("IsOffroad") and configured()


def wayon_impact_ready(started, params, CP):
  return wayon_offroad(started, params, CP) and not CONFIG_PATH.with_name("disable_impact_detection").exists()


def wayon_camera_requested(started, params, CP):
  return not started and params.get_bool("IsOffroad") and camera_request() is not None


def wayon_panorama_requested(started, params, CP):
  request = camera_request()
  return not started and params.get_bool("IsOffroad") and request is not None and request["mode"] == "panorama"
