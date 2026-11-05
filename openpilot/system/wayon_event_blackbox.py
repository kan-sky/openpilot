"""Offroad-only local event packages. No network, camera ownership or uploader side effects.

Port of Wayon's event package format. Pass analysis.dataQuality.routeFirstMonoS
to build_blackbox_archive so report offsets and signal extraction share an origin.
The caller owns upload retries and attaches the returned capture ID to the report.
Video is MP4 only, remuxed from whole selected segments. Playback offsets use
nominal 60-second segments, not measured camera-to-log synchronization.
"""
from __future__ import annotations

import bz2
import gzip
import hashlib
import io
import json
import math
import os
import shutil
import stat
import subprocess
import tempfile
import time
import zipfile
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path


BLACKBOX_SCHEMA = "wayon-drive-blackbox-package-v1"
CAMERA_LAYOUT = "road_blackbox"
SEGMENT_DURATION_S = 60.0
STATE_SAMPLE_INTERVAL_S = 0.1
DEFAULT_BLACKBOX_ROOT = Path(os.getenv("WAYON_BLACKBOX_ROOT", "/data/media/0/wayon-blackbox"))
DEFAULT_MAX_ARCHIVES = 40
DEFAULT_MAX_STORAGE_BYTES = 1024 * 1024 * 1024
QLOG_CANDIDATES = ("qlog.zst", "qlog.bz2", "qlog")
RLOG_CANDIDATES = ("rlog.zst", "rlog.bz2", "rlog", *QLOG_CANDIDATES)
SIGNAL_TYPES = ("can", "carState", "carControl", "longitudinalPlan", "selfdriveState", "radarState")
REPORT_TYPES = ("carState", "carControl", "longitudinalPlan", "selfdriveState", "radarState",
                "driverMonitoringState", "longitudinalPlanSP")
COPY_CHUNK_BYTES = 256 * 1024


class WorkAborted(RuntimeError):
  """No partial report/archive should be marked complete or uploaded."""


class ResourceLimitExceeded(WorkAborted):
  pass


class _LogUnavailable(ResourceLimitExceeded):
  """A file-local failure permits trying another log with the same work budget."""


class _VideoUnavailable(ResourceLimitExceeded):
  """A video-only limit may leave a usable signals-only package."""


@dataclass(frozen=True)
class WorkLimits:
  max_log_bytes: int = 32 * 1024 * 1024
  max_decoded_log_bytes: int = 64 * 1024 * 1024
  max_total_decoded_bytes: int = 128 * 1024 * 1024
  max_messages: int = 1_000_000
  max_route_segments: int = 240
  max_package_segments: int = 3
  max_signal_bytes: int = 8 * 1024 * 1024
  max_video_bytes: int = 32 * 1024 * 1024
  max_archive_bytes: int = 48 * 1024 * 1024
  max_seconds: float = 90.0
  ffmpeg_seconds: float = 45.0

  def __post_init__(self):
    if any(not math.isfinite(value) or value <= 0 for value in vars(self).values()):
      raise ValueError("work limits must be finite and positive")


def offroad_allowed():
  """Fail closed without Params or sufficient memory; evaluated throughout work."""
  try:
    import psutil
    from openpilot.common.params import Params
    params = Params()
    return (params.get_bool("IsOffroad") and not params.get_bool("IsOnroad")
            and psutil.virtual_memory().available >= 512 * 1024 * 1024)
  except Exception:
    return False


class WorkBudget:
  def __init__(self, allowed=offroad_allowed, limits=None):
    self.allowed = allowed
    self.limits = limits or WorkLimits()
    self.deadline = time.monotonic() + self.limits.max_seconds
    self.decoded_bytes = 0
    self.messages = 0

  def check(self):
    if not self.allowed():
      raise WorkAborted("offroad work no longer allowed")
    if time.monotonic() >= self.deadline:
      raise ResourceLimitExceeded("work deadline exceeded")


def _open_input(path, max_bytes):
  path = Path(path)
  if path.is_symlink() or path.parent.is_symlink() or any(path.parent.glob("*.lock")):
    raise WorkAborted("route file is linked or still recording")
  fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
  try:
    info = os.fstat(fd)
    if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= max_bytes:
      raise ResourceLimitExceeded("route file type or size is outside limits")
    return os.fdopen(fd, "rb")
  except BaseException:
    os.close(fd)
    raise


def iter_route_log(path, *, budget=None, allowed=offroad_allowed, limits=None):
  """Bounded qlog/rlog decoding, without LogReader's whole-route cache or sort."""
  budget = budget or WorkBudget(allowed, limits)
  budget.check()
  path = Path(path)
  if path.name not in RLOG_CANDIDATES:
    raise ValueError("unsupported route log filename")
  try:
    with _open_input(path, budget.limits.max_log_bytes) as source:
      compressed = source.read(budget.limits.max_log_bytes + 1)
  except ResourceLimitExceeded as error:
    raise _LogUnavailable("log_file_size_limit") from error
  if len(compressed) > budget.limits.max_log_bytes:
    raise _LogUnavailable("compressed_log_limit")
  source = io.BytesIO(compressed)
  decode_errors = (OSError, EOFError)
  if path.suffix == ".zst":
    import zstandard
    decode_errors += (zstandard.ZstdError,)
    source = zstandard.ZstdDecompressor(max_window_size=budget.limits.max_decoded_log_bytes).stream_reader(source)
  elif path.suffix == ".bz2":
    source = bz2.BZ2File(source)
  with source, io.BytesIO() as decoded:
    while True:
      budget.check()
      try:
        chunk = source.read(min(COPY_CHUNK_BYTES, budget.limits.max_decoded_log_bytes + 1 - decoded.tell()))
      except decode_errors as error:
        raise _LogUnavailable("malformed_compressed_log") from error
      if not chunk:
        break
      decoded.write(chunk)
      budget.decoded_bytes += len(chunk)
      if budget.decoded_bytes > budget.limits.max_total_decoded_bytes:
        raise ResourceLimitExceeded("total decoded log limit exceeded")
      if decoded.tell() > budget.limits.max_decoded_log_bytes:
        raise _LogUnavailable("decoded_log_limit")
    raw = decoded.getvalue()
  from openpilot.cereal import log
  import capnp
  try:
    for index, message in enumerate(log.Event.read_multiple_bytes(raw)):
      if index % 128 == 0:
        budget.check()
      budget.messages += 1
      if budget.messages > budget.limits.max_messages:
        raise ResourceLimitExceeded("message limit exceeded")
      yield message
  except capnp.KjException as error:
    raise _LogUnavailable("malformed_capnp_log") from error
  budget.check()


def route_segments(route_group, budget):
  segments = route_group.get("segments", [])
  if len(segments) > budget.limits.max_route_segments:
    raise ResourceLimitExceeded("route segment limit exceeded")
  result = []
  seen = set()
  for segment, directory, fallback in segments:
    budget.check()
    if not isinstance(segment, int) or segment < 0 or segment in seen:
      raise ValueError("invalid or duplicate route segment")
    seen.add(segment)
    result.append((segment, Path(directory), Path(fallback)))
  return sorted(result)


def route_log_file(segment_dir, *, qlog_only=False, budget):
  candidates = QLOG_CANDIDATES if qlog_only else RLOG_CANDIDATES
  for name in candidates:
    budget.check()
    candidate = Path(segment_dir) / name
    if candidate.is_file() and not candidate.is_symlink() and 0 < candidate.stat().st_size <= budget.limits.max_log_bytes:
      return candidate
  return None


def _json_safe(value):
  if isinstance(value, bytes):
    return value.hex()
  if isinstance(value, dict):
    return {str(key): _json_safe(item) for key, item in value.items()}
  if isinstance(value, (list, tuple)):
    return [_json_safe(item) for item in value]
  if isinstance(value, float) and not math.isfinite(value):
    return None
  if isinstance(value, (str, int, float, bool)) or value is None:
    return value
  return str(getattr(value, "name", value))


def _event_window(event):
  offset = float(event.get("offsetS", 0.0))
  window = event.get("captureWindow") or {}
  start = float(window.get("startOffsetS", max(0.0, offset - 30.0)))
  end = float(window.get("endOffsetS", offset + 30.0))
  if not all(math.isfinite(value) for value in (offset, start, end)) or start < 0 or not start < end or end - start > 60.001:
    raise ValueError("invalid event window")
  return start, end


def _first_mono(segments, budget):
  # A surviving later segment is not a new route origin.
  if not segments or segments[0][0] != 0:
    raise WorkAborted("route origin unavailable; pass route_first_mono")
  log_file = route_log_file(segments[0][1], qlog_only=True, budget=budget)
  if log_file is None:
    raise WorkAborted("route origin qlog unavailable")
  for msg in iter_route_log(log_file, budget=budget):
    if msg.valid and msg.which() in REPORT_TYPES:
      return float(msg.logMonoTime) / 1e9
  raise WorkAborted("route origin unavailable")


def _can_payload(msg):
  return [[int(frame.address), int(frame.src), int(getattr(frame, "busTime", 0)), bytes(frame.dat).hex()]
          for frame in msg.can]


def _segment_signal_rows(log_file, start, end, origin, last_emitted, max_bytes, budget):
  import capnp
  counts = dict.fromkeys(SIGNAL_TYPES, 0)
  last_emitted = dict(last_emitted)
  # Commit only a complete candidate; a malformed rlog prefix must not survive a qlog retry.
  with io.BytesIO() as staged:
    try:
      for msg in iter_route_log(log_file, budget=budget):
        which = msg.which()
        offset = float(msg.logMonoTime) / 1e9 - origin
        if which not in counts or not msg.valid or not start <= offset <= end:
          continue
        if which != "can" and offset - last_emitted[which] < STATE_SAMPLE_INTERVAL_S:
          continue
        payload = _can_payload(msg) if which == "can" else _json_safe(getattr(msg, which).to_dict())
        if which == "can" and not payload:
          continue
        row = json.dumps({"t": round(offset, 4), "type": which, "data": payload},
                         ensure_ascii=True, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        if staged.tell() + len(row) > max_bytes:
          raise ResourceLimitExceeded("signal output limit exceeded")
        staged.write(row)
        counts[which] += 1
        if which != "can":
          last_emitted[which] = offset
    except capnp.KjException as error:
      raise _LogUnavailable("malformed_capnp_log") from error
    return staged.getvalue(), counts, last_emitted


def _build_signal_log(selected, event, origin, destination, budget):
  start, end = _event_window(event)
  counts = dict.fromkeys(SIGNAL_TYPES, 0)
  last_emitted = {name: -math.inf for name in SIGNAL_TYPES if name != "can"}
  sources, missing, rejected, missing_can = [], [], [], []
  uncompressed_bytes = 0
  with destination.open("wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", compresslevel=6, mtime=0) as compressed:
    for segment, directory, _ in selected:
      for name in RLOG_CANDIDATES:
        budget.check()
        log_file = directory / name
        if not log_file.is_file():
          continue
        try:
          rows, segment_counts, emitted = _segment_signal_rows(
            log_file, start, end, origin, last_emitted, budget.limits.max_signal_bytes - uncompressed_bytes, budget)
        except _LogUnavailable as error:
          budget.check()
          rejected.append({"segment": segment, "file": name, "reason": str(error)})
          continue
        budget.check()
        compressed.write(rows)
        uncompressed_bytes += len(rows)
        for which, count in segment_counts.items():
          counts[which] += count
        last_emitted = emitted
        sources.append({"segment": segment, "file": name})
        if not segment_counts["can"]:
          missing_can.append(segment)
        break
      else:
        missing.append(segment)
        missing_can.append(segment)
  budget.check()
  return counts, sources, missing, rejected, missing_can


def _copy_file(source, destination, budget, max_bytes):
  total = 0
  with _open_input(source, max_bytes) as stream:
    while True:
      budget.check()
      chunk = stream.read(COPY_CHUNK_BYTES)
      if not chunk:
        break
      total += len(chunk)
      if total > max_bytes:
        raise ResourceLimitExceeded("copy limit exceeded")
      destination.write(chunk)
  return total


def _find_ffmpeg():
  candidates = [os.getenv("WAYON_FFMPEG")]
  try:
    import ffmpeg
    binary_dir = getattr(ffmpeg, "BIN_DIR", None)
    if isinstance(binary_dir, (str, os.PathLike)) and binary_dir:
      candidates.append(Path(binary_dir) / "ffmpeg")
  except ImportError:
    pass
  # comma's dependency keeps the native executable outside PATH. Do not call
  # wrapper probes here; execution belongs to the bounded, owned-child runner.
  candidates.extend((shutil.which("ffmpeg"), "/usr/local/venv/bin/ffmpeg", "/usr/bin/ffmpeg"))
  for candidate in candidates:
    if candidate and Path(candidate).is_file() and os.access(candidate, os.X_OK):
      return str(Path(candidate).resolve())
  return None


def _run_ffmpeg(command, output, budget):
  budget.check()
  deadline = min(budget.deadline, time.monotonic() + budget.limits.ffmpeg_seconds)
  process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  try:
    while process.poll() is None:
      budget.check()
      if time.monotonic() >= deadline:
        raise _VideoUnavailable("ffmpeg deadline exceeded")
      if output.exists() and output.stat().st_size > budget.limits.max_video_bytes:
        raise _VideoUnavailable("ffmpeg output limit exceeded")
      time.sleep(0.05)
    budget.check()
    if output.exists() and output.stat().st_size > budget.limits.max_video_bytes:
      raise _VideoUnavailable("ffmpeg output limit exceeded")
    return process.returncode == 0 and output.is_file() and output.stat().st_size > 0
  finally:
    # Never pkill, signal a process group, or touch an existing encoder/camerad.
    if process.poll() is None:
      process.terminate()
      try:
        process.wait(timeout=0.5)
      except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def _build_video(selected, event, temporary_dir, budget):
  start, end = _event_window(event)
  videos = [(segment, directory / "qcamera.ts") for segment, directory, _ in selected
            if (directory / "qcamera.ts").is_file()]
  info = {"available": False, "file": None, "contentType": None, "source": "qcamera",
          "sourceSegments": [segment for segment, _ in videos], "playbackStartS": 0.0,
          "playbackDurationS": 0.0, "requestedStartOffsetS": start, "requestedEndOffsetS": end,
          "synchronization": {"status": "approximate", "basis": "nominal_60s_segments", "frameAccurate": False,
                              "routeAlignmentVerified": False, "actualStartOffsetS": None, "actualEndOffsetS": None,
                              "maxErrorS": None},
          "mediaDurationS": None, "coverageVerified": False}
  if not videos:
    return None, {**info, "reason": "qcamera_missing"}
  if any(right[0] != left[0] + 1 for left, right in zip(videos, videos[1:], strict=False)):
    return None, {**info, "reason": "video_segment_gap"}
  available_start = max(start, videos[0][0] * SEGMENT_DURATION_S)
  available_end = min(end, (videos[-1][0] + 1) * SEGMENT_DURATION_S)
  playback_start = available_start - videos[0][0] * SEGMENT_DURATION_S
  duration = available_end - available_start
  info.update(selectedStartOffsetS=videos[0][0] * SEGMENT_DURATION_S,
              selectedEndOffsetS=(videos[-1][0] + 1) * SEGMENT_DURATION_S,
              availableStartOffsetS=available_start, availableEndOffsetS=available_end,
              partialWindow=available_start > start or available_end < end)
  ffmpeg = _find_ffmpeg()
  if not ffmpeg:
    return None, {**info, "reason": "ffmpeg_unavailable"}
  if sum(source.stat().st_size for _, source in videos) > budget.limits.max_video_bytes:
    return None, {**info, "reason": "video_size_limit"}
  # Private bounded copies prevent ffmpeg from reading changing/deleted route files.
  copied = []
  total = 0
  for segment, source in videos:
    target = temporary_dir / f"segment-{segment}.ts"
    with target.open("wb") as destination:
      total += _copy_file(source, destination, budget, budget.limits.max_video_bytes - total)
    copied.append(target)
  concat = temporary_dir / "qcamera-concat.txt"
  concat.write_text("".join(f"file '{file.name}'\n" for file in copied), encoding="ascii")
  output = temporary_dir / "road.mp4"
  # Retain preceding GOPs; the player seeks inside this MP4 instead of a stream-copy cut.
  command = [ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-y", "-threads", "1",
             "-f", "concat", "-safe", "1", "-i", str(concat), "-map", "0:v:0", "-an", "-c", "copy",
             "-avoid_negative_ts", "disabled", "-movflags", "+faststart", str(output)]
  reason = "mp4_remux_failed"
  try:
    if _run_ffmpeg(command, output, budget):
      return output, {**info, "available": True, "file": "road.mp4", "contentType": "video/mp4", "source": "qcamera-remux",
                      "remuxMode": "whole_selected_segments", "playbackStartS": round(playback_start, 3),
                      "playbackDurationS": round(duration, 3)}
  except _VideoUnavailable:
    reason = "video_processing_limit"
  except OSError:
    reason = "ffmpeg_execution_failed"
  budget.check()
  return None, {**info, "reason": reason}


def build_blackbox_archive(route_group, event, trip_id, output_root=DEFAULT_BLACKBOX_ROOT, *,
                           route_first_mono=None, allowed=offroad_allowed, limits=None):
  budget = WorkBudget(allowed, limits)
  budget.check()
  segments = route_segments(route_group, budget)
  start, end = _event_window(event)
  selected = [item for item in segments if (item[0] + 1) * SEGMENT_DURATION_S > start and item[0] * SEGMENT_DURATION_S < end]
  if not selected:
    return None
  if len(selected) > budget.limits.max_package_segments:
    raise ResourceLimitExceeded("package segment limit exceeded")
  origin = _first_mono(segments, budget) if route_first_mono is None else float(route_first_mono)
  if not math.isfinite(origin) or origin < 0:
    raise ValueError("invalid route origin")
  root = Path(output_root)
  if root.is_symlink():
    raise WorkAborted("archive directory must not be a symlink")
  root.mkdir(parents=True, exist_ok=True)
  identity = json.dumps([str(trip_id), str(event.get("id", "")), start, end], separators=(",", ":"))
  archive_path = root / ("wayon-" + hashlib.sha256(identity.encode()).hexdigest()[:32] + ".zip")
  with tempfile.TemporaryDirectory(prefix=".wayon-blackbox-", dir=root) as workdir:
    workdir = Path(workdir)
    signals_path = workdir / "signals.jsonl.gz"
    counts, sources, missing, rejected, missing_can = _build_signal_log(selected, event, origin, signals_path, budget)
    sampled_segments = [item["segment"] for item in sources if item["file"].startswith("qlog")]
    video_path, video_info = _build_video(selected, event, workdir, budget)
    event_fields = ("id", "time", "offsetS", "type", "severity", "title", "detail", "captureWindow", "metrics",
                    "causeCandidates", "causeStatus")
    manifest = {
      "schema": BLACKBOX_SCHEMA, "cameraLayout": CAMERA_LAYOUT, "createdAtUnixS": int(datetime.now(UTC).timestamp()),
      "tripId": str(trip_id), "routeName": str(route_group.get("route_name", "unknown-route")),
      "routeFirstMonoS": origin, "event": {key: _json_safe(event[key]) for key in event_fields if key in event},
      "video": video_info,
      "signals": {"file": "signals.jsonl.gz", "contentType": "application/gzip", "counts": counts,
                  "sampleIntervalS": STATE_SAMPLE_INTERVAL_S, "rawCanIncluded": counts["can"] > 0,
                  "rawCanComplete": False, "sources": sources, "missingSegments": missing,
                  "rejectedSources": rejected, "sampledSegments": sampled_segments, "rawCanMissingSegments": missing_can,
                  "coverage": "sampled_qlog" if sampled_segments else "recorded_only" if sources else "unavailable"},
    }
    manifest_bytes = json.dumps(manifest, ensure_ascii=True, allow_nan=False, separators=(",", ":")).encode()
    inputs = [signals_path] + ([video_path] if video_path else [])
    if len(manifest_bytes) + sum(file.stat().st_size for file in inputs) + 4096 > budget.limits.max_archive_bytes:
      raise ResourceLimitExceeded("archive size limit exceeded")
    temporary_path = workdir / "package.zip"
    with zipfile.ZipFile(temporary_path, "w", compression=zipfile.ZIP_STORED, allowZip64=False) as archive:
      archive.writestr("manifest.json", manifest_bytes)
      for file in inputs:
        with archive.open(file.name, "w") as destination:
          _copy_file(file, destination, budget, budget.limits.max_archive_bytes)
    budget.check()
    if temporary_path.stat().st_size > budget.limits.max_archive_bytes:
      raise ResourceLimitExceeded("archive size limit exceeded")
    os.replace(temporary_path, archive_path)
  return {"path": str(archive_path), "sizeBytes": archive_path.stat().st_size,
          "videoAvailable": video_info["available"], "rawCanIncluded": counts["can"] > 0, "signalCounts": counts,
          "videoUnavailableReason": video_info.get("reason"),
          "manifest": manifest, "cameraLayout": CAMERA_LAYOUT,
          "package": {"status": "stored", "archiveName": archive_path.name, "sizeBytes": archive_path.stat().st_size,
                      "videoAvailable": video_info["available"], "rawCanIncluded": counts["can"] > 0,
                      "videoUnavailableReason": video_info.get("reason"),
                      "rawCanComplete": False, "rawCanMissingSegments": missing_can,
                      "signalCoverage": manifest["signals"]["coverage"],
                      "signalCounts": counts},
          "metadata": {"schema": BLACKBOX_SCHEMA, "eventId": str(event.get("id", "")),
                       "eventType": str(event.get("type", "")), "tripId": str(trip_id)}}


def prune_blackbox_archives(output_root=DEFAULT_BLACKBOX_ROOT, max_archives=DEFAULT_MAX_ARCHIVES,
                            max_storage_bytes=DEFAULT_MAX_STORAGE_BYTES, *, allowed=offroad_allowed):
  budget = WorkBudget(allowed)
  budget.check()
  root = Path(output_root)
  if root.is_symlink() or not root.is_dir():
    return []
  archives = []
  for path in root.glob("wayon-*.zip"):
    budget.check()
    if path.is_symlink() or not path.is_file() or len(path.stem) != 38:
      continue
    if any(char not in "0123456789abcdef" for char in path.stem[6:]):
      continue
    archives.append(path)
    if len(archives) > 4096:
      raise ResourceLimitExceeded("archive inventory limit exceeded")
  archives.sort(key=lambda path: path.stat().st_mtime, reverse=True)
  removed = []
  total = 0
  for index, path in enumerate(archives):
    budget.check()
    size = path.stat().st_size
    if index < max_archives and total + size <= max_storage_bytes:
      total += size
    else:
      path.unlink(missing_ok=True)
      removed.append(str(path))
  return removed
