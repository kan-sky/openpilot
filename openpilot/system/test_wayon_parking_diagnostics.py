"""Real policy/spool/worker-loop tests; IPC, codecs and Linux setup are mocked."""
import ctypes
import itertools
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from openpilot.system import wayon_parking as parking
from openpilot.system.test_wayon_parking import Reader, State, frame


@pytest.fixture(autouse=True)
def local_summary(tmp_path, monkeypatch):
  path = tmp_path / "summary.json"
  monkeypatch.setattr(parking, "DIAGNOSTICS_PATH", path)
  monkeypatch.setattr(parking, "cleanup_abandoned", lambda: None)
  return path


@pytest.fixture
def clock(monkeypatch):
  value = [100.0]
  monkeypatch.setattr(parking.time, "monotonic", lambda: value[0])
  monkeypatch.setattr(parking.time, "time", lambda: 1000 + value[0])
  return value


@pytest.mark.parametrize("field,value,reason", [
  ("started", False, "not_started"), ("thermalStatus", "yellow", "thermal"),
  ("memoryUsagePercent", 80, "memory"), ("memoryUsagePercent", 85, "memory"),
  ("memoryUsagePercent", -1, "memory"), ("memoryUsagePercent", float("nan"), "memory"),
  ("vEgo", 0, "speed"), ("vEgo", 7, "speed"), ("vEgo", float("inf"), "speed"),
  ("gearShifter", "park", "gear"), ("deviceState", None, "device_stale"), ("carState", None, "car_stale"),
])
def test_gate_reasons_before_worker_and_inside_buffer(field, value, reason, clock, local_summary, monkeypatch):
  sm = State()
  if field in sm:
    sm.seen[field] = False
  else:
    setattr(sm["carState"] if field in ("vEgo", "gearShifter") else sm["deviceState"], field, value)
  reader = Reader()
  ring = parking.ApproachBuffer(reader)
  ring.sample(sm)
  assert reader.calls == 0
  assert ring.diagnostics.counters[f"sample_gate_{reason}"] == 1
  monkeypatch.setattr(parking.subprocess, "Popen", lambda *a, **kw: pytest.fail("ineligible worker spawn"))
  session = parking.ApproachSession()
  session.sample(sm)
  assert session.take() == []
  counts = json.loads(local_summary.read_text())["counters"]
  assert counts[f"session_gate_{reason}"] == 1 and counts["session_samples"] == 1
  assert "attempted" not in counts


def test_gate_matches_original_predicate_at_boundaries():
  import math
  speeds = (-7, -25 / 3.6, -.2, -.199, 0, .199, .2, 25 / 3.6, 7, float("nan"), float("inf"))
  memories = (-1, 0, 79.999, 80, 84.999, 85, float("nan"), float("inf"))
  for speed, memory, started, thermal, gear in itertools.product(
    speeds, memories, (True, False), ("green", "yellow"), ("drive", "reverse", "low", "sport", "park"),
  ):
    sm = State()
    sm["deviceState"].started, sm["deviceState"].thermalStatus, sm["deviceState"].memoryUsagePercent = started, thermal, memory
    sm["carState"].vEgo, sm["carState"].gearShifter = speed, gear
    expected = (bool(started) and str(thermal) == "green" and math.isfinite(memory) and 0 <= memory < 80
                and math.isfinite(speed) and .2 <= abs(speed) <= 25 / 3.6
                and str(gear) in ("drive", "reverse", "low", "sport"))
    assert parking.approach_allowed(sm, 100) == expected
  sm["deviceState"].thermalStatus = "red"
  sm.seen["deviceState"] = False
  assert parking.approach_reject_reason(sm, 100) == "device_stale"  # First failure, not every matching condition.


def test_attempts_missing_errors_latency_interval_and_backoff(clock):
  class TimedReader(Reader):
    result = "frame"
    elapsed = .025

    def __call__(self, now):
      self.calls += 1
      clock[0] += self.elapsed
      if self.result == "error":
        raise RuntimeError("private camera details must never be logged")
      return frame(now) if self.result == "frame" else None

  reader = TimedReader()
  ring = parking.ApproachBuffer(reader)
  ring.sample(State(100))
  clock[0] = 101
  ring.sample(State(101))
  clock[0] = 102
  reader.result = "missing"
  ring.sample(State(102))
  clock[0] = 104
  reader.result, reader.elapsed = "error", .2
  ring.sample(State(104))
  assert ring.next_sample == pytest.approx(114.2)
  clock[0] = 106
  ring.sample(State(106))
  counts = ring.diagnostics.counters
  assert counts["attempted"] == counts["capture_calls"] == 3
  assert counts["captured"] == ring.captured_count == 1
  assert counts["reader_missing"] == counts["add_reject_type"] == 1
  assert counts["capture_error"] == ring.skipped_count == 1
  assert counts["sample_interval"] == counts["sample_backoff"] == counts["capture_backoff"] == 1
  assert counts["capture_us_total"] == pytest.approx(250_000, abs=3)
  assert counts["capture_us_max"] == pytest.approx(200_000, abs=1)
  assert "encode_calls" not in counts  # Total reader time is not mislabeled as codec time.


@pytest.mark.parametrize("candidate,now,wall,reason", [
  (None, 100, 1100, "type"), (frame(100, bytearray(b"jpeg")), 100, 1100, "type"),
  (frame(100, b""), 100, 1100, "size"), (frame(100, b"x" * (parking.MAX_FRAME_BYTES + 1)), 100, 1100, "size"),
  (frame(float("nan")), 100, 1100, "time"), (frame(99), 100, 1100, "age"),
  (frame(101), 100, 1100, "age"), (frame(100), 100, 1200, "clock"), (frame(100), 100, 1100, "order"),
])
def test_add_rejections(candidate, now, wall, reason):
  ring = parking.ApproachBuffer(Reader())
  assert ring.add(frame(100), 100, 1100)
  assert not ring.add(candidate, now, wall)
  assert ring.diagnostics.counters[f"add_reject_{reason}"] == 1
  assert list(ring.frames) == [frame(100)]


@pytest.mark.parametrize("field,value,reason", [("vEgo", 10, "speed"), ("memoryUsagePercent", 85, "memory"),
                                                ("thermalStatus", "yellow", "thermal")])
def test_clear_and_expiry_counts_without_retention_changes(field, value, reason):
  ring = parking.ApproachBuffer(Reader())
  for at in range(100, 152, 2):
    assert ring.add(frame(at), at, at + 1000)
  assert len(ring.frames) == 24
  assert ring.diagnostics.counters["buffer_evicted"] == 2
  ring.expire(166)
  assert ring.diagnostics.counters["buffer_expired"] == 1
  sm = State(166)
  setattr(sm["carState"] if field == "vEgo" else sm["deviceState"], field, value)
  ring.sample(sm, 166)
  assert ring.diagnostics.counters[f"buffer_clear_{reason}"] == 1
  assert ring.diagnostics.counters["buffer_cleared_frames"] == 23
  assert ring.bytes == 0 and not ring.frames


def test_saturating_fixed_schema_and_private_atomic_replacement(tmp_path):
  diagnostics = parking.CaptureDiagnostics()
  for key in parking.DIAGNOSTIC_KEYS:
    diagnostics.count(key, parking.DIAGNOSTICS_MAX)
    diagnostics.count(key)
  for amount in (-1, True, 1.5, "private"):
    diagnostics.count("attempted", amount)
  diagnostics.count("frame-id/location/private", 123)
  diagnostics.counters["location"] = "must not leave memory"
  diagnostics.counters["captured"] = "must not leave memory"
  diagnostics.elapsed("encode", float("nan"))
  diagnostics.elapsed("encode", float("inf"))
  path = tmp_path / "diagnostics.json"
  diagnostics.write(path, "worker")
  data = json.loads(path.read_text())
  assert set(data) == {"version", "phase", "counters"}
  assert "location" not in data["counters"] and "captured" not in data["counters"]
  assert all(type(v) is int and v == parking.DIAGNOSTICS_MAX for v in data["counters"].values())
  assert path.stat().st_size <= parking.DIAGNOSTICS_BYTES
  assert path.stat().st_mode & 0o777 == 0o600
  parking.CaptureDiagnostics().write(path, "worker")
  assert json.loads(path.read_text())["counters"] == {}
  assert list(tmp_path.iterdir()) == [path]


def test_collect_only_sanitized_bounded_counters(tmp_path):
  diagnostics = parking.CaptureDiagnostics()
  diagnostics.count("captured", parking.DIAGNOSTICS_MAX)
  (tmp_path / "diagnostics.json").write_text(json.dumps({"version": 1, "phase": "worker", "counters": {
    "captured": 1, "encode_us_max": 25, "worker_exit_offroad": 1,
    "frame_id": "private", "location": "private", "capture_error": -1, "attempted": True, "sample_calls": 1.5,
    "encode_us_total": parking.DIAGNOSTICS_MAX + 1,
  }}))
  diagnostics.collect(tmp_path)
  assert diagnostics.counters == {"captured": parking.DIAGNOSTICS_MAX, "encode_us_max": 25,
                                  "worker_exit_offroad": 1, "worker_snapshots": 1}


def test_memory_capture_gate_still_preserves_existing_history(clock, tmp_path):
  ring = parking.ApproachBuffer(Reader())
  assert ring.add(frame(98), 98, 1098)
  sm = State()
  sm["deviceState"].memoryUsagePercent = 80
  ring.sample(sm)
  assert list(ring.frames) == [frame(98)] and ring.reader.calls == 0
  session = parking.ApproachSession()
  directory = tmp_path / "spool"
  directory.mkdir()
  session.directory = directory
  session.started_at = 100
  process = SimpleNamespace(poll=lambda: None, terminate=lambda: pytest.fail("80% must not stop an existing worker"))
  session.process = process
  session.sample(sm)
  assert session.process is process and directory.exists()
  assert session.diagnostics.counters["session_gate_memory"] == 1


def test_checkpoint_rate_limit_including_failed_writes(tmp_path, monkeypatch):
  diagnostics = parking.CaptureDiagnostics()
  opened = []

  def fail(*args):
    opened.append(args)
    raise OSError("full filesystem")

  monkeypatch.setattr(parking.os, "open", fail)
  for now in (100, 100.1, 102, 104.99, 105, 109.9):
    diagnostics.write(tmp_path / "diagnostics.json", "worker", now=now)
  assert len(opened) == 2


@pytest.mark.parametrize("bad", ["absent", "oversize", "invalid_json", "wrong_schema", "symlink", "fifo"])
def test_bad_worker_snapshot_does_not_block_or_change_take(tmp_path, clock, bad):
  directory = tmp_path / "spool"
  directory.mkdir()
  parking.write_history(directory, [frame(98)])
  path = directory / "diagnostics.json"
  if bad == "oversize":
    path.write_bytes(b" " * (parking.DIAGNOSTICS_BYTES + 1))
  elif bad == "invalid_json":
    path.write_bytes(b"{untrusted")
  elif bad == "wrong_schema":
    path.write_text('{"version":1,"phase":"worker","counters":[]}')
  elif bad == "symlink":
    path.symlink_to(tmp_path / "outside")
  elif bad == "fifo":
    parking.os.mkfifo(path)
  session = parking.ApproachSession()
  session.directory = directory
  result = session.take()
  assert result == [frame(98)]
  assert result.diagnostics.counters["worker_snapshot_missing"] == 1
  assert not directory.exists()


def test_worker_snapshots_accumulate_once_and_reset_at_session_boundary(tmp_path, clock, local_summary):
  session = parking.ApproachSession()
  for i in range(2):
    directory = tmp_path / f"worker-{i}"
    directory.mkdir()
    parking.write_history(directory, [frame(98)])
    worker = parking.CaptureDiagnostics()
    worker.count("captured", 3 + i)
    worker.elapsed("encode", .02 + i * .01)
    worker.write(directory / "diagnostics.json", "worker")
    session.directory = directory
    if i == 0:
      session.clear("ineligible")
      session.clear("ineligible")
  result = session.take()
  counts = result.diagnostics.counters
  assert counts["captured"] == 7 and counts["worker_snapshots"] == 2
  assert counts["worker_snapshot_unfinished"] == 2
  assert counts["encode_us_total"] == 50_000 and counts["encode_us_max"] == 30_000
  assert counts["history_clear_ineligible"] == counts["history_clear_take"] == 1
  assert counts["take_selected"] == 1
  assert session.diagnostics.counters == {}
  assert parking.upload_frames(result, 1100, now=100, wall=1100) == [(1098, b"jpeg", "approach")]
  data = json.loads(local_summary.read_text())
  assert data["phase"] == "upload" and data["counters"]["captured"] == 7
  assert data["counters"]["upload_selected"] == 1
  session.clear()
  assert json.loads(local_summary.read_text()) == data  # Idle clear does not erase the completed summary.


def test_upload_filter_reasons_and_no_unrelated_empty_list_overwrite(local_summary):
  diagnostics = parking.CaptureDiagnostics()
  frames = parking.ApproachFrames([
    frame(100), frame(-1), parking.ApproachFrame(100, 9999, b"jpeg"), frame(120), frame(119), frame(118), frame(117),
  ], diagnostics)
  selected = parking.upload_frames(frames, 1120, now=120, wall=1120)
  assert [at for at, _, _ in selected] == [1100, 1119, 1118]
  counts = diagnostics.counters
  assert counts["upload_calls"] == 1 and counts["upload_selected"] == 3
  assert counts["upload_reject_age"] == 1
  assert counts["upload_reject_clock"] == counts["upload_reject_parked"] == 1
  assert counts["upload_not_considered"] == 1
  saved = local_summary.read_bytes()
  assert parking.upload_frames([], 1120, now=120, wall=1120) == []
  assert local_summary.read_bytes() == saved


@pytest.mark.parametrize("failure", ["open", "serialize", "replace"])
def test_diagnostic_write_errors_do_not_change_take_or_upload(tmp_path, clock, monkeypatch, failure):
  directory = tmp_path / "spool"
  directory.mkdir()
  parking.write_history(directory, [frame(98)])

  def fail(*args, **kwargs):
    raise OSError("private details")

  if failure == "open":
    original_open = parking.os.open

    def open_file(path, *args, **kwargs):
      if str(path).endswith(("diagnostics.json", "summary.tmp")):
        return fail()
      return original_open(path, *args, **kwargs)

    monkeypatch.setattr(parking.os, "open", open_file)
  elif failure == "serialize":
    monkeypatch.setattr(parking.json, "dumps", fail)
  else:
    monkeypatch.setattr(Path, "replace", fail)
  session = parking.ApproachSession()
  session.directory = directory
  result = session.take()
  assert result == [frame(98)]
  assert parking.upload_frames(result, 1100, now=100, wall=1100) == [(1098, b"jpeg", "approach")]
  assert not directory.exists()


@pytest.mark.parametrize("reason", ["manifest", "name", "file", "hash", "time", "order", "read", "clock"])
def test_take_rejections_are_counted_without_exposing_manifest_data(tmp_path, clock, local_summary, reason):
  directory = tmp_path / "spool"
  directory.mkdir()
  parking.write_history(directory, [frame(96), frame(98, b"other")])
  manifest = directory / "frames.json"
  rows = json.loads(manifest.read_text())
  path = directory / rows[0]["file"]
  if reason == "manifest":
    rows = rows * 13
  elif reason == "name":
    rows[0]["file"] = "../private-location"
  elif reason == "file":
    path.write_bytes(b"")
  elif reason == "hash":
    path.write_bytes(b"corrupt")
  elif reason == "time":
    rows[0]["monotonic"] = float("nan")
  elif reason == "order":
    rows.reverse()
  elif reason == "clock":
    rows[1]["captured_at"] += 999
  else:
    rows[0].pop("file")
  manifest.write_text(json.dumps(rows))
  session = parking.ApproachSession()
  session.directory = directory
  assert session.take() == []
  raw = local_summary.read_text()
  assert json.loads(raw)["counters"][f"take_reject_{reason}"] == 1
  assert "private-location" not in raw and "captured_at" not in raw and path.name not in raw


@pytest.mark.parametrize("outcome", ["connect", "empty", "stale", "dimensions", "encode", "encode_error"])
def test_actual_wide_reader_paths_with_mock_ipc_and_codecs(monkeypatch, clock, outcome):
  class Array:
    def reshape(self, *args): return self
    def __getitem__(self, key): return self

  saved = []

  def save(output, format_name, **kwargs):
    saved.append((format_name, kwargs))
    clock[0] += .04
    if outcome == "encode_error":
      raise RuntimeError("codec private error")
    output.write(b"mock-jpeg")

  ipc_frame = SimpleNamespace(width=64, height=64, stride=64, uv_offset=4096, data=b"x" * 6144)
  client = SimpleNamespace(connect=lambda blocking: outcome != "connect", recv=lambda **kwargs: None if outcome == "empty" else ipc_frame,
                           timestamp_eof=int((98 if outcome == "stale" else 100) * 1e9))
  if outcome == "dimensions":
    ipc_frame.width = 2000
  monkeypatch.setitem(sys.modules, "msgq.visionipc", SimpleNamespace(VisionIpcClient=lambda *a: client,
                                                                  VisionStreamType=SimpleNamespace(VISION_STREAM_WIDE_ROAD=1)))
  monkeypatch.setitem(sys.modules, "cv2", SimpleNamespace(setNumThreads=lambda n: None, cvtColor=lambda *a: Array(), COLOR_YUV2RGB_NV12=1))
  monkeypatch.setitem(sys.modules, "numpy", SimpleNamespace(asarray=lambda a: Array(), vstack=lambda a: Array()))
  monkeypatch.setitem(sys.modules, "PIL", SimpleNamespace(Image=SimpleNamespace(fromarray=lambda a: SimpleNamespace(save=save))))
  reader = parking.WideReader()
  if outcome == "encode_error":
    with pytest.raises(RuntimeError):
      reader(100)
  else:
    result = reader(100)
    assert (result is not None) == (outcome == "encode")
  counts = reader.diagnostics.counters
  if outcome.startswith("encode"):
    assert counts["encode_calls"] == 1 and abs(counts["encode_us_total"] - 40_000) <= 1
    assert saved == [("JPEG", {"quality": 92, "subsampling": 0})]
  else:
    key = "reader_connect_failed" if outcome == "connect" else f"reader_{outcome}"
    assert counts[key] == 1 and "encode_calls" not in counts


@pytest.mark.parametrize("exit_reason", ["offroad", "ineligible", "stale", "parent_gone"])
@pytest.mark.parametrize("storage_available", [True, False])
def test_actual_worker_loop_and_final_checkpoint_with_mock_linux_ipc(tmp_path, monkeypatch, clock, exit_reason, storage_available):
  class WorkerState(State):
    updates = 0

    def update(self, timeout):
      self.updates += 1
      clock[0] = (100, 102, 105.1, 106, 112)[self.updates - 1]
      self.__init__(clock[0])
      if self.updates >= 4:
        if exit_reason == "offroad":
          self["deviceState"].started = False
        elif exit_reason == "ineligible":
          self["deviceState"].memoryUsagePercent = 85
        else:
          self.seen["carState"] = False

  sm = WorkerState()
  monkeypatch.setitem(sys.modules, "openpilot.cereal", SimpleNamespace(messaging=SimpleNamespace(SubMaster=lambda services: sm)))
  monkeypatch.setattr(ctypes, "CDLL", lambda *args: SimpleNamespace(prctl=lambda *args: 0))
  monkeypatch.setattr(parking.os, "nice", lambda value: None)
  monkeypatch.setattr(parking.os, "sched_setaffinity", lambda *args: None, raising=False)
  monkeypatch.setattr(parking.os, "getppid", lambda: 2 if exit_reason == "parent_gone" and sm.updates == 3 else 123)
  diagnostics = []

  def reader(counters):
    diagnostics.append(counters)
    return Reader()

  monkeypatch.setattr(parking, "WideReader", reader)
  monkeypatch.setattr(parking.time, "sleep", lambda seconds: None)
  if not storage_available:
    original_open = parking.os.open

    def open_file(path, *args, **kwargs):
      if str(path).endswith("diagnostics.tmp"):
        raise OSError("diagnostics storage unavailable")
      return original_open(path, *args, **kwargs)

    monkeypatch.setattr(parking.os, "open", open_file)
  parking.worker(tmp_path, 123)
  if storage_available:
    data = json.loads((tmp_path / "diagnostics.json").read_text())
    assert data["phase"] == "worker" and data["counters"] == diagnostics[0].counters
  else:
    assert not (tmp_path / "diagnostics.json").exists()
  counts = diagnostics[0].counters
  assert counts["captured"] == counts["attempted"] == 3
  assert counts[f"worker_exit_{exit_reason}"] == 1
  rows = json.loads((tmp_path / "frames.json").read_text())
  assert len(rows) == (3 if exit_reason == "offroad" else 0)
  assert (tmp_path / "finished").read_text() == exit_reason
  if exit_reason != "offroad":
    assert counts["worker_discarded_frames"] == 3
