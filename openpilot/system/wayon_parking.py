"""Best-effort, bounded exterior photos of a low-speed parking approach.

This consumer never requests cameras, publishes vehicle data or runs inference.
Only the offroad uploader may transmit the retained JPEGs.
"""
from collections import deque
from dataclasses import dataclass
import io
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
import time

from openpilot.system.wayon_camera import sample_fresh, snapshot_frame_fresh, snapshot_capture_time

INTERVAL_S = 2.0
HISTORY_S = 60.0
MAX_FRAMES = 24
MAX_BYTES = 8 * 1024 * 1024
MAX_FRAME_BYTES = 1024 * 1024
MAX_SELECTED = 6
MAX_UPLOAD_AGE_S = 120.0

DIAGNOSTICS_PATH = Path("/dev/shm/wayon-parking-diagnostics.json")
DIAGNOSTICS_BYTES = 8192
DIAGNOSTICS_INTERVAL_S = 5.0
DIAGNOSTICS_MAX = 2**53 - 1
GATE_REASONS = ("device_stale", "car_stale", "not_started", "thermal", "memory", "speed", "gear")
DIAGNOSTIC_KEYS = frozenset(
  [f"{scope}_gate_{reason}" for scope in ("session", "sample") for reason in GATE_REASONS]
  + [f"history_clear_{reason}" for reason in ("external", "stale", "ineligible", "worker_dead", "heartbeat", "spawn", "take")]
  + [f"buffer_clear_{reason}" for reason in ("explicit", "speed", "thermal", "memory", "take")]
  + [f"worker_exit_{reason}" for reason in ("offroad", "ineligible", "stale", "parent_gone")]
  + [f"add_reject_{reason}" for reason in ("type", "size", "time", "age", "clock", "order")]
  + [f"take_reject_{reason}" for reason in ("worker", "stop", "manifest", "name", "file", "hash", "time", "order", "bytes", "read", "clock")]
  + [f"upload_reject_{reason}" for reason in ("age", "clock", "parked")]
  + [f"{stage}_{metric}" for stage in ("capture", "encode") for metric in ("calls", "us_total", "us_max")]
  + ["session_samples", "sample_calls", "sample_interval", "sample_backoff", "attempted", "captured", "reader_missing",
     "capture_error", "reader_connect_failed", "reader_empty", "reader_stale", "reader_dimensions", "capture_backoff",
     "buffer_added", "buffer_expired", "buffer_evicted", "buffer_cleared_frames", "take_calls", "take_selected", "take_empty",
     "take_not_selected", "upload_calls", "upload_selected", "upload_not_considered", "worker_started", "worker_retry",
     "worker_snapshot_missing", "worker_snapshot_unfinished", "worker_snapshots", "worker_discarded_frames", "worker_wait_stale"]
)


class CaptureDiagnostics:
  """Fixed-schema saturating counters; no timestamps, frame data or exception text.

  One local summary is replaced, never appended/uploaded. Termination can lose
  counts since the last checkpoint; missing snapshots are explicit.
  """
  def __init__(self):
    self.counters = {}
    self.next_write = 0.0

  def count(self, key, amount=1):
    try:
      if key in DIAGNOSTIC_KEYS and type(amount) is int and amount >= 0:
        self.counters[key] = min(DIAGNOSTICS_MAX, self.counters.get(key, 0) + amount)
    except Exception:
      pass

  def elapsed(self, stage, seconds):
    try:
      micros = min(DIAGNOSTICS_MAX, max(0, int(seconds * 1_000_000)))
      self.count(f"{stage}_calls")
      self.count(f"{stage}_us_total", micros)
      key = f"{stage}_us_max"
      if key in DIAGNOSTIC_KEYS:
        self.counters[key] = max(self.counters.get(key, 0), micros)
    except Exception:
      pass

  def write(self, path, phase, now=None):
    try:
      if now is not None:
        if now < self.next_write:
          return
        self.next_write = now + DIAGNOSTICS_INTERVAL_S
      if phase not in ("worker", "take", "clear", "upload"):
        return
      counters = {k: v for k, v in self.counters.items()
                  if k in DIAGNOSTIC_KEYS and type(v) is int and 0 <= v <= DIAGNOSTICS_MAX}
      raw = json.dumps({"version": 1, "phase": phase, "counters": counters}, separators=(",", ":"))
      if len(raw) > DIAGNOSTICS_BYTES:
        return
      temporary = path.with_suffix(".tmp")
      fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
      with os.fdopen(fd, "w") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
          return
        stream.write(raw)
      temporary.replace(path)
    except Exception:
      pass

  def collect(self, directory):
    try:
      path = directory / "diagnostics.json"
      fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
      with os.fdopen(fd, "rb") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
          raise ValueError
        raw = stream.read(DIAGNOSTICS_BYTES + 1)
      if len(raw) > DIAGNOSTICS_BYTES:
        raise ValueError
      data = json.loads(raw)
      if data["version"] != 1 or data["phase"] != "worker" or not isinstance(data["counters"], dict):
        raise ValueError
      for key, value in data["counters"].items():
        if key in DIAGNOSTIC_KEYS and type(value) is int and 0 <= value <= DIAGNOSTICS_MAX:
          if key.endswith("_us_max"):
            self.counters[key] = max(self.counters.get(key, 0), value)
          else:
            self.count(key, value)
      self.count("worker_snapshots")
      if not any(type(value) is int and 0 < value <= DIAGNOSTICS_MAX
                 for key, value in data["counters"].items() if key.startswith("worker_exit_") and key in DIAGNOSTIC_KEYS):
        self.count("worker_snapshot_unfinished")
    except Exception:
      self.count("worker_snapshot_missing")


class ApproachFrames(list):
  # Carry only the bounded counters to upload_frames, without IDs or global maps.
  def __init__(self, frames, diagnostics):
    super().__init__(frames)
    self.diagnostics = diagnostics


@dataclass(frozen=True)
class ApproachFrame:
  monotonic: float
  captured_at: float
  jpeg: bytes


def approach_reject_reason(sm, now):
  for name, reason in (("deviceState", "device_stale"), ("carState", "car_stale")):
    if not sample_fresh(sm, name, now, max_age=1.5):
      return reason
  device, car = sm["deviceState"], sm["carState"]
  speed, memory = float(car.vEgo), float(device.memoryUsagePercent)
  if not bool(device.started):
    return "not_started"
  if str(device.thermalStatus) != "green":
    return "thermal"
  if not math.isfinite(memory) or not 0 <= memory < 80:
    return "memory"
  if not math.isfinite(speed) or not 0.2 <= abs(speed) <= 25 / 3.6:
    return "speed"
  if str(car.gearShifter) not in ("drive", "reverse", "low", "sport"):
    return "gear"
  return None


def approach_allowed(sm, now):
  return approach_reject_reason(sm, now) is None


class WideReader:
  def __init__(self, diagnostics=None):
    self.client = None
    self.diagnostics = diagnostics or CaptureDiagnostics()

  def close(self):
    self.client = None

  def __call__(self, now):
    import cv2
    import numpy as np
    from PIL import Image
    from msgq.visionipc import VisionIpcClient, VisionStreamType

    cv2.setNumThreads(1)

    if self.client is None:
      client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, True)
      if not client.connect(False):
        self.diagnostics.count("reader_connect_failed")
        return None
      self.client = client
    frame = self.client.recv(timeout_ms=0)
    received = time.monotonic()
    if frame is None:
      self.diagnostics.count("reader_empty")
      return None
    if not snapshot_frame_fresh(self.client, received):
      self.diagnostics.count("reader_stale")
      return None
    if not (64 <= frame.width <= 1928 and 64 <= frame.height <= 1208):
      self.diagnostics.count("reader_dimensions")
      return None
    began = time.monotonic()
    try:
      y = np.asarray(frame.data[:frame.uv_offset]).reshape(-1, frame.stride)[:frame.height, :frame.width]
      uv = np.asarray(frame.data[frame.uv_offset:frame.uv_offset + frame.stride * (frame.height // 2)])
      uv = uv.reshape(-1, frame.stride)[:frame.height // 2, :frame.width]
      rgb = cv2.cvtColor(np.vstack((y, uv)), cv2.COLOR_YUV2RGB_NV12)
      image = Image.fromarray(rgb)
      output = io.BytesIO()
      image.save(output, "JPEG", quality=92, subsampling=0)
      raw = output.getvalue()
    finally:
      self.diagnostics.elapsed("encode", time.monotonic() - began)
    captured = float(self.client.timestamp_eof) / 1e9
    return ApproachFrame(captured, snapshot_capture_time(self.client), raw)


class ApproachBuffer:
  def __init__(self, reader=None, diagnostics=None):
    self.diagnostics = diagnostics or CaptureDiagnostics()
    self.reader = reader or WideReader(self.diagnostics)
    self.frames = deque()
    self.bytes = 0
    self.next_sample = 0.0
    self.last_elapsed = 0.0
    self.captured_count = 0
    self.skipped_count = 0

  def clear(self, reason="explicit"):
    self.diagnostics.count(f"buffer_clear_{reason}")
    self.diagnostics.count("buffer_cleared_frames", len(self.frames))
    self.frames.clear()
    self.bytes = 0
    self.reader.close()

  def expire(self, now):
    while self.frames and not 0 <= now - self.frames[0].monotonic <= HISTORY_S:
      self.bytes -= len(self.frames.popleft().jpeg)
      self.diagnostics.count("buffer_expired")

  def add(self, frame, now, wall):
    if not isinstance(frame, ApproachFrame) or not isinstance(frame.jpeg, bytes):
      self.diagnostics.count("add_reject_type")
      return False
    if not 0 < len(frame.jpeg) <= MAX_FRAME_BYTES:
      self.diagnostics.count("add_reject_size")
      return False
    if not math.isfinite(frame.monotonic) or not math.isfinite(frame.captured_at):
      self.diagnostics.count("add_reject_time")
      return False
    if not 0 <= now - frame.monotonic <= 0.75:
      self.diagnostics.count("add_reject_age")
      return False
    if abs((wall - frame.captured_at) - (now - frame.monotonic)) > 2:
      self.diagnostics.count("add_reject_clock")
      return False
    if self.frames and (frame.monotonic - self.frames[-1].monotonic < 0.5
                        or frame.captured_at <= self.frames[-1].captured_at):
      self.diagnostics.count("add_reject_order")
      return False
    self.expire(now)
    self.frames.append(frame)
    self.bytes += len(frame.jpeg)
    while len(self.frames) > MAX_FRAMES or self.bytes > MAX_BYTES:
      self.bytes -= len(self.frames.popleft().jpeg)
      self.diagnostics.count("buffer_evicted")
    self.diagnostics.count("buffer_added")
    return True

  def sample(self, sm, now=None):
    now = time.monotonic() if now is None else now
    self.diagnostics.count("sample_calls")
    self.expire(now)
    reason = approach_reject_reason(sm, now)
    if reason is not None:
      self.diagnostics.count(f"sample_gate_{reason}")
      self.reader.close()
      # A fast stretch breaks the parking approach, even if it was brief.
      if sample_fresh(sm, "carState", now) and abs(float(sm["carState"].vEgo)) > 25 / 3.6:
        self.clear("speed")
      if sample_fresh(sm, "deviceState", now):
        device = sm["deviceState"]
        if str(device.thermalStatus) != "green" or float(device.memoryUsagePercent) >= 85:
          self.clear("thermal" if str(device.thermalStatus) != "green" else "memory")
      return
    if now < self.next_sample:
      self.diagnostics.count("sample_backoff" if self.last_elapsed > 0.1 else "sample_interval")
      return
    self.next_sample = now + INTERVAL_S
    self.diagnostics.count("attempted")
    began = time.monotonic()
    try:
      frame = self.reader(now)
      if frame is None:
        self.diagnostics.count("reader_missing")
      if self.add(frame, time.monotonic(), time.time()):
        self.captured_count += 1
        self.diagnostics.count("captured")
    except Exception:
      self.reader.close()
      self.skipped_count += 1
      self.diagnostics.count("capture_error")
    self.last_elapsed = time.monotonic() - began
    if self.last_elapsed > 0.1:
      # A slow encoder is optional work: back off, never try to catch up.
      self.next_sample = time.monotonic() + 10.0
      self.diagnostics.count("capture_backoff")
    # This is the existing backoff timer, including IPC/imports/encoding and add.
    self.diagnostics.elapsed("capture", self.last_elapsed)

  def take(self, now=None, wall=None):
    now = time.monotonic() if now is None else now
    wall = time.time() if wall is None else wall
    self.expire(now)
    frames = list(self.frames)
    self.clear("take")
    if any(abs((wall - f.captured_at) - (now - f.monotonic)) > 2 for f in frames):
      self.diagnostics.count("take_reject_clock")
      return []
    if len(frames) <= MAX_SELECTED:
      return frames
    # Keep adjacent pairs at the beginning, middle and end of the approach.
    starts = (0, (len(frames) - 2) // 2, len(frames) - 2)
    return [frames[i] for i in sorted({j for i in starts for j in (i, i + 1)})]


def upload_frames(frames, parked_at, now=None, wall=None):
  now = time.monotonic() if now is None else now
  wall = time.time() if wall is None else wall
  diagnostics = frames.diagnostics if isinstance(frames, ApproachFrames) else CaptureDiagnostics()
  diagnostics.count("upload_calls")
  diagnostics.count("upload_not_considered", max(0, len(frames) - MAX_SELECTED))
  selected = []
  for f in frames[:MAX_SELECTED]:
    if not 0 <= now - f.monotonic <= MAX_UPLOAD_AGE_S:
      diagnostics.count("upload_reject_age")
      continue
    if abs((wall - f.captured_at) - (now - f.monotonic)) > 2:
      diagnostics.count("upload_reject_clock")
      continue
    if not 0.5 <= parked_at - f.captured_at <= MAX_UPLOAD_AGE_S:
      diagnostics.count("upload_reject_parked")
      continue
    selected.append((f.captured_at, f.jpeg, "approach"))
  diagnostics.count("upload_selected", len(selected))
  if isinstance(frames, ApproachFrames):
    diagnostics.write(DIAGNOSTICS_PATH, "upload")
  return selected


class ApproachSession:
  """Keep optional camera IPC/JPEG work out of the telemetry uploader process."""
  def __init__(self):
    self.process = None
    self.directory = None
    self.retry_after = 0.0
    self.started_at = 0.0
    self.diagnostics = CaptureDiagnostics()
    cleanup_abandoned()

  def stop(self):
    process = self.process
    if process is not None and process.poll() is None:
      process.terminate()
      try:
        process.wait(timeout=0.25)
      except subprocess.TimeoutExpired:
        process.kill()
        try:
          process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
          return False
    self.process = None
    return True

  def clear(self, reason="external"):
    if not self.stop():
      return
    if self.directory is not None:
      self.diagnostics.collect(self.directory)
      self.diagnostics.count(f"history_clear_{reason}")
      shutil.rmtree(self.directory, ignore_errors=True)
      self.directory = None
    if reason == "external":
      if self.diagnostics.counters:
        self.diagnostics.write(DIAGNOSTICS_PATH, "clear")
      self.diagnostics = CaptureDiagnostics()

  def sample(self, sm):
    now = time.monotonic()
    self.diagnostics.count("session_samples")
    try:
      reason = approach_reject_reason(sm, now)
      if reason is not None:
        self.diagnostics.count(f"session_gate_{reason}")
    except Exception:
      pass
    if not all(sample_fresh(sm, name, now, max_age=1.5) for name in ("deviceState", "carState")):
      self.clear("stale")
      return
    device, car = sm["deviceState"], sm["carState"]
    if (not device.started or str(device.thermalStatus) != "green"
        or not math.isfinite(float(device.memoryUsagePercent)) or device.memoryUsagePercent >= 85
        or not math.isfinite(float(car.vEgo)) or abs(car.vEgo) > 25 / 3.6):
      self.clear("ineligible")
      return
    if self.process is not None and self.process.poll() is not None:
      self.clear("worker_dead")
      self.retry_after = now + 30
    if self.process is not None and now - self.started_at > 10:
      try:
        heartbeat = self.directory / "heartbeat"
        alive = heartbeat.stat().st_size < 64 and 0 <= now - float(heartbeat.read_text()) <= 5
      except (OSError, ValueError):
        alive = False
      if not alive:
        self.clear("heartbeat")
        self.retry_after = now + 30
    if now < self.retry_after:
      self.diagnostics.count("worker_retry")
    if self.process is not None or now < self.retry_after or not approach_allowed(sm, now):
      return
    try:
      self.directory = Path(tempfile.mkdtemp(prefix=f"wayon-parking-{os.getpid()}-", dir="/dev/shm"))
      env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}
      self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--worker",
                                       str(self.directory), str(os.getpid())], env=env,
                                      stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL, start_new_session=True)
      self.started_at = now
      self.diagnostics.count("worker_started")
    except (OSError, ValueError):
      self.clear("spawn")
      self.retry_after = now + 30

  def take(self):
    self.diagnostics.count("take_calls")
    frames = self._take()
    diagnostics = self.diagnostics
    diagnostics.count("take_selected", len(frames))
    if not frames:
      diagnostics.count("take_empty")
    diagnostics.write(DIAGNOSTICS_PATH, "take")
    if self.process is None and self.directory is None:
      self.diagnostics = CaptureDiagnostics()
    return ApproachFrames(frames, diagnostics)

  def _take(self):
    if self.process is not None and self.process.poll() is not None:
      try:
        clean_offroad = (self.directory / "finished").read_text() == "offroad"
      except (OSError, TypeError):
        clean_offroad = False
      if not clean_offroad:
        self.diagnostics.count("take_reject_worker")
        self.clear("worker_dead")
        return []
    if not self.stop():
      self.diagnostics.count("take_reject_stop")
      return []
    try:
      if self.directory is None:
        return []
      manifest = self.directory / "frames.json"
      if manifest.stat().st_size > 32_768:
        self.diagnostics.count("take_reject_manifest")
        return []
      rows = json.loads(manifest.read_text())
      if not isinstance(rows, list) or len(rows) > MAX_FRAMES:
        self.diagnostics.count("take_reject_manifest")
        return []
      ring = ApproachBuffer(diagnostics=self.diagnostics)
      for row in rows:
        name = row["file"]
        if not isinstance(name, str) or len(name) != 68 or not name.endswith(".jpg"):
          self.diagnostics.count("take_reject_name")
          return []
        digest = name[:-4]
        if any(c not in "0123456789abcdef" for c in digest):
          self.diagnostics.count("take_reject_name")
          return []
        path = self.directory / name
        if path.is_symlink() or not 0 < path.stat().st_size <= MAX_FRAME_BYTES:
          self.diagnostics.count("take_reject_file")
          return []
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
          self.diagnostics.count("take_reject_hash")
          return []
        mono, wall = float(row["monotonic"]), float(row["captured_at"])
        if not math.isfinite(mono) or not math.isfinite(wall):
          self.diagnostics.count("take_reject_time")
          return []
        if ring.frames and (mono <= ring.frames[-1].monotonic or wall <= ring.frames[-1].captured_at):
          self.diagnostics.count("take_reject_order")
          return []
        ring.frames.append(ApproachFrame(mono, wall, raw))
        ring.bytes += len(raw)
        if ring.bytes > MAX_BYTES:
          self.diagnostics.count("take_reject_bytes")
          return []
      frames = ring.take()
      self.diagnostics.count("take_not_selected", max(0, len(rows) - len(frames)))
      return frames
    except (OSError, ValueError, KeyError, TypeError):
      self.diagnostics.count("take_reject_read")
      return []
    finally:
      self.clear("take")


def write_history(directory, frames):
  rows = []
  keep = set()
  for f in frames:
    name = hashlib.sha256(f.jpeg).hexdigest() + ".jpg"
    path = directory / name
    if not path.exists():
      temporary = directory / "frame.tmp"
      temporary.write_bytes(f.jpeg)
      temporary.replace(path)
    keep.add(name)
    rows.append({"file": name, "monotonic": f.monotonic, "captured_at": f.captured_at})
  temporary = directory / "frames.tmp"
  temporary.write_text(json.dumps(rows, separators=(",", ":")))
  temporary.replace(directory / "frames.json")
  for path in directory.glob("*.jpg"):
    if path.name not in keep:
      path.unlink()


def cleanup_abandoned(root=Path("/dev/shm")):
  for directory in list(root.glob("wayon-parking-*-*"))[:100]:
    try:
      parent = int(directory.name.split("-", 3)[2])
      if (parent <= 1 or directory.is_symlink() or not directory.is_dir()
          or directory.stat().st_uid != os.getuid() or directory.stat().st_mode & 0o077):
        continue
      try:
        os.kill(parent, 0)
      except ProcessLookupError:
        shutil.rmtree(directory)
    except (OSError, ValueError):
      continue


def finish_history(directory, frames, reason):
  write_history(directory, frames if reason == "offroad" else [])
  temporary = directory / "finished.tmp"
  temporary.write_text(reason)
  temporary.replace(directory / "finished")


def history_exit_reason(sm, now):
  device_fresh = sample_fresh(sm, "deviceState", now, max_age=1.5)
  car_fresh = sample_fresh(sm, "carState", now, max_age=1.5)
  if device_fresh:
    device = sm["deviceState"]
    memory = float(device.memoryUsagePercent)
    if str(device.thermalStatus) != "green" or not math.isfinite(memory) or not 0 <= memory < 85:
      return "ineligible"
  if car_fresh:
    speed = float(sm["carState"].vEgo)
    if not math.isfinite(speed) or abs(speed) > 25 / 3.6:
      return "ineligible"
  # card normally stops offroad. Fresh device state can close an existing
  # history without needing a new car sample; it never authorizes a capture.
  if device_fresh and not sm["deviceState"].started:
    return "offroad"
  return None


def worker(directory, parent):
  import ctypes
  import signal

  # Parent death must not leave a camera consumer behind.
  if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) != 0 or os.getppid() != parent:
    return
  os.nice(19)
  os.sched_setaffinity(0, {0, 1, 2, 3})
  from openpilot.cereal import messaging
  sm = messaging.SubMaster(["deviceState", "carState"])
  ring = ApproachBuffer()
  last_count = 0
  deadline = time.monotonic() + 5
  reason = "parent_gone"
  while os.getppid() == parent:
    sm.update(200)
    now = time.monotonic()
    heartbeat = directory / "heartbeat.tmp"
    heartbeat.write_text(str(now))
    heartbeat.replace(directory / "heartbeat")
    terminal = history_exit_reason(sm, now)
    if terminal:
      reason = terminal
      break
    if not all(sample_fresh(sm, name, now, max_age=1.5) for name in ("deviceState", "carState")):
      ring.diagnostics.count("worker_wait_stale")
      if now > deadline:
        reason = "stale"
        break
      ring.diagnostics.write(directory / "diagnostics.json", "worker", now=now)
      continue
    deadline = now + 5
    ring.sample(sm, now)
    if ring.captured_count != last_count:
      write_history(directory, ring.frames)
      last_count = ring.captured_count
    ring.diagnostics.write(directory / "diagnostics.json", "worker", now=now)
    time.sleep(0.2)
  finish_history(directory, ring.frames, reason)
  ring.diagnostics.count(f"worker_exit_{reason}")
  if reason != "offroad":
    ring.diagnostics.count("worker_discarded_frames", len(ring.frames))
  ring.diagnostics.write(directory / "diagnostics.json", "worker")


if __name__ == "__main__" and len(sys.argv) == 4 and sys.argv[1] == "--worker":
  worker(Path(sys.argv[2]), int(sys.argv[3]))
