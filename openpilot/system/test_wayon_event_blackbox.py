import bz2
import gzip
import json
import os
import subprocess
import sys
import zipfile
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.system import wayon_event_blackbox as box


def event_message(which, mono_s, payload):
  return log.Event.new_message(logMonoTime=int(mono_s * 1e9), valid=True, **{which: payload}).to_bytes()


def make_segment(root, index=0, *, video=b"video-ts-data", filename="qlog", messages=None):
  segment = root / f"route--{index}"
  segment.mkdir()
  path = segment / filename
  if messages is None:
    messages = [
      event_message("carState", 100 + index * 60, {"vEgo": 3, "aEgo": -0.2}),
      event_message("can", 100.1 + index * 60, [{"address": 0x180, "src": 0, "dat": b"\x01\x02"}]),
      event_message("carControl", 100.2 + index * 60, {"longActive": True, "actuators": {"accel": -0.8}}),
      event_message("longitudinalPlan", 100.3 + index * 60, {"aTarget": -0.7}),
    ]
  data = b"".join(messages)
  if filename.endswith(".bz2"):
    data = bz2.compress(data)
  elif filename.endswith(".zst"):
    import zstandard
    data = zstandard.ZstdCompressor().compress(data)
  path.write_bytes(data)
  if video is not None:
    (segment / "qcamera.ts").write_bytes(video)
  return index, segment, path


def event(start=0.0, end=30.0):
  return {"id": "unexplained-brake-1", "type": "unexplained_braking", "offsetS": start + 0.4,
          "captureWindow": {"startOffsetS": start, "endOffsetS": end}, "metrics": {"speedKph": 10.8},
          "causeCandidates": [], "causeStatus": "hypothesis_not_diagnosis"}


def build(tmp_path, segments, incident=None, **kwargs):
  return box.build_blackbox_archive({"route_name": "route", "segments": segments}, incident or event(), "trip-1",
                                    output_root=tmp_path / "blackbox", allowed=lambda: True, **kwargs)


@pytest.fixture(autouse=True)
def no_implicit_native_process(monkeypatch):
  finder = box._find_ffmpeg
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: None)
  return finder


def test_find_ffmpeg_prefers_official_dependency_before_path(tmp_path, monkeypatch, no_implicit_native_process):
  native = tmp_path / "install" / "bin" / "ffmpeg"
  native.parent.mkdir(parents=True)
  native.write_bytes(b"native-placeholder")
  native.chmod(0o755)
  wrapper = tmp_path / "ffmpeg-wrapper"
  wrapper.write_bytes(b"wrapper-placeholder")
  wrapper.chmod(0o755)
  monkeypatch.delenv("WAYON_FFMPEG", raising=False)
  monkeypatch.setitem(sys.modules, "ffmpeg", SimpleNamespace(BIN_DIR=str(native.parent)))
  monkeypatch.setattr(box.shutil, "which", lambda name: str(wrapper))

  def unexpected_process(*args, **kwargs):
    pytest.fail("discovery must not start an unbounded probe")

  monkeypatch.setattr(box.subprocess, "Popen", unexpected_process)
  assert no_implicit_native_process() == str(native)
  monkeypatch.setenv("WAYON_FFMPEG", str(wrapper))
  assert no_implicit_native_process() == str(wrapper)


@pytest.mark.parametrize("module", [None, SimpleNamespace(), SimpleNamespace(BIN_DIR="/missing-wayon-ffmpeg-dir")])
def test_find_ffmpeg_keeps_path_fallback_without_bundled_binary(tmp_path, monkeypatch, no_implicit_native_process, module):
  native = tmp_path / "ffmpeg"
  native.write_bytes(b"native-placeholder")
  native.chmod(0o755)
  monkeypatch.delenv("WAYON_FFMPEG", raising=False)
  monkeypatch.setitem(sys.modules, "ffmpeg", module)
  monkeypatch.setattr(box.shutil, "which", lambda name: str(native))
  assert no_implicit_native_process() == str(native)


def test_without_ffmpeg_archive_is_signals_only_with_stored_zip_headers(tmp_path):
  result = build(tmp_path, [make_segment(tmp_path)])
  assert result["cameraLayout"] == "road_blackbox"
  assert result["package"]["status"] == "stored"
  assert "captureId" not in result["package"]
  assert result["videoAvailable"] is False
  assert result["package"]["videoAvailable"] is False
  assert result["package"]["videoUnavailableReason"] == "ffmpeg_unavailable"
  assert result["metadata"] == {"schema": box.BLACKBOX_SCHEMA, "eventId": "unexplained-brake-1",
                                "eventType": "unexplained_braking", "tripId": "trip-1"}
  with zipfile.ZipFile(result["path"]) as archive:
    assert set(archive.namelist()) == {"manifest.json", "signals.jsonl.gz"}
    assert all(item.compress_type == zipfile.ZIP_STORED and not (item.flag_bits & 8) for item in archive.infolist())
    manifest = json.loads(archive.read("manifest.json"))
    assert manifest["schema"] == box.BLACKBOX_SCHEMA
    assert manifest["video"]["playbackDurationS"] == 0.0
    assert manifest["video"]["file"] is None
    assert manifest["video"]["reason"] == "ffmpeg_unavailable"
    assert manifest["signals"]["rawCanIncluded"] is True
    assert manifest["signals"]["rawCanComplete"] is False
    assert manifest["signals"]["coverage"] == "sampled_qlog"
    rows = [json.loads(line) for line in gzip.decompress(archive.read("signals.jsonl.gz")).splitlines()]
    assert any(row["type"] == "can" and row["data"][0][3] == "0102" for row in rows)
    assert any(row["type"] == "carState" for row in rows)
  assert not list((tmp_path / "blackbox").glob(".wayon-blackbox-*"))


def test_prefers_bounded_rlog_but_qlog_can_is_not_claimed_complete(tmp_path):
  segment = make_segment(tmp_path)
  (segment[1] / "rlog").write_bytes(event_message("carState", 100.2, {"vEgo": 3}))
  result = build(tmp_path, [segment])
  assert result["manifest"]["signals"]["sources"] == [{"segment": 0, "file": "rlog"}]
  assert result["rawCanIncluded"] is False
  assert result["manifest"]["signals"]["rawCanComplete"] is False


@pytest.mark.parametrize("failure", ["compressed_size", "decoded_size", "malformed_capnp", "malformed_bz2", "malformed_zst"])
def test_rlog_file_failure_falls_back_to_qlog_without_partial_rows(tmp_path, failure):
  segment = make_segment(tmp_path)
  qlog_bytes = segment[2].read_bytes()
  limits = box.WorkLimits()
  if failure == "compressed_size":
    name, data, reason = "rlog", qlog_bytes * 4, "log_file_size_limit"
    limits = replace(limits, max_log_bytes=len(qlog_bytes))
  elif failure == "decoded_size":
    name, data, reason = "rlog.bz2", bz2.compress(qlog_bytes * 4), "decoded_log_limit"
    limits = replace(limits, max_decoded_log_bytes=len(qlog_bytes) + 64)
  elif failure == "malformed_capnp":
    prefix = event_message("carState", 100, {"vEgo": 99})
    prefix += event_message("can", 100.1, [{"address": 0x999, "src": 0, "dat": b"bad-prefix"}])
    name, data, reason = "rlog", prefix + b"\xff" * 32, "malformed_capnp_log"
  else:
    name = "rlog.bz2" if failure == "malformed_bz2" else "rlog.zst"
    data, reason = b"invalid-compressed-log", "malformed_compressed_log"
  (segment[1] / name).write_bytes(data)
  result = build(tmp_path, [segment], route_first_mono=100, limits=limits)
  signals = result["manifest"]["signals"]
  assert signals["sources"] == [{"segment": 0, "file": "qlog"}]
  assert signals["rejectedSources"] == [{"segment": 0, "file": name, "reason": reason}]
  assert signals["coverage"] == "sampled_qlog" and signals["sampledSegments"] == [0]
  assert signals["rawCanIncluded"] is True and signals["rawCanComplete"] is False
  assert signals["rawCanMissingSegments"] == [] and signals["missingSegments"] == []
  assert result["signalCounts"]["carState"] == result["signalCounts"]["can"] == 1
  with zipfile.ZipFile(result["path"]) as archive:
    rows = [json.loads(line) for line in gzip.decompress(archive.read("signals.jsonl.gz")).splitlines()]
  assert next(row for row in rows if row["type"] == "carState")["data"]["vEgo"] == 3
  assert next(row for row in rows if row["type"] == "can")["data"][0][0] == 0x180


def test_fallback_is_per_segment_and_missing_raw_can_is_explicit(tmp_path):
  first = make_segment(tmp_path, 0, messages=[event_message("carState", 155, {"vEgo": 3})])
  second = make_segment(tmp_path, 1, messages=[event_message("carState", 165, {"vEgo": 4})])
  (first[1] / "rlog").write_bytes(event_message("can", 155, [{"address": 0x180, "src": 0, "dat": b"raw"}]))
  (second[1] / "rlog").write_bytes(b"\xff" * 32)
  result = build(tmp_path, [first, second], event(50, 110), route_first_mono=100)
  signals = result["manifest"]["signals"]
  assert signals["sources"] == [{"segment": 0, "file": "rlog"}, {"segment": 1, "file": "qlog"}]
  assert signals["sampledSegments"] == [1] and signals["rawCanMissingSegments"] == [1]
  assert signals["coverage"] == "sampled_qlog" and signals["rawCanComplete"] is False
  assert result["package"]["signalCoverage"] == "sampled_qlog"
  assert result["package"]["rawCanMissingSegments"] == [1] and result["package"]["rawCanComplete"] is False
  assert result["signalCounts"]["can"] == result["signalCounts"]["carState"] == 1


def test_all_bad_logs_are_reported_missing_not_complete(tmp_path):
  segment = make_segment(tmp_path)
  segment[2].write_bytes(b"\xff" * 32)
  (segment[1] / "rlog").write_bytes(b"\xff" * 32)
  result = build(tmp_path, [segment], route_first_mono=100)
  signals = result["manifest"]["signals"]
  assert signals["sources"] == [] and signals["coverage"] == "unavailable"
  assert signals["missingSegments"] == signals["rawCanMissingSegments"] == [0]
  assert signals["rawCanIncluded"] is False and signals["rawCanComplete"] is False
  assert [item["file"] for item in signals["rejectedSources"]] == ["rlog", "qlog"]


@pytest.mark.parametrize("failure", [box.WorkAborted("onroad"), box.ResourceLimitExceeded("outer deadline")])
def test_signal_fallback_does_not_swallow_global_abort(tmp_path, monkeypatch, failure):
  segment = make_segment(tmp_path)
  (segment[1] / "rlog").write_bytes(segment[2].read_bytes())
  attempted = []

  def abort(log_file, **kwargs):
    attempted.append(log_file.name)
    raise failure

  monkeypatch.setattr(box, "iter_route_log", abort)
  with pytest.raises(type(failure)):
    build(tmp_path, [segment], route_first_mono=100)
  assert attempted == ["rlog"]
  assert list((tmp_path / "blackbox").iterdir()) == []


@pytest.mark.parametrize("reason", ["onroad", "deadline"])
def test_file_failure_rechecks_guard_before_qlog_retry(tmp_path, monkeypatch, reason):
  segment = make_segment(tmp_path)
  (segment[1] / "rlog").write_bytes(segment[2].read_bytes())
  permitted = True
  attempted = []

  def fail(log_file, *, budget):
    nonlocal permitted
    attempted.append(log_file.name)
    if reason == "onroad":
      permitted = False
    else:
      budget.deadline = -1
    raise box._LogUnavailable("malformed_capnp_log")

  monkeypatch.setattr(box, "iter_route_log", fail)
  with pytest.raises(box.WorkAborted):
    box.build_blackbox_archive({"segments": [segment]}, event(), "trip", output_root=tmp_path / "blackbox",
                               route_first_mono=100, allowed=lambda: permitted)
  assert attempted == ["rlog"]
  assert list((tmp_path / "blackbox").iterdir()) == []


@pytest.mark.parametrize("limit", ["max_total_decoded_bytes", "max_messages"])
def test_fallback_never_resets_shared_work_counters(tmp_path, limit):
  segment = make_segment(tmp_path)
  prefix = event_message("carState", 100, {"vEgo": 99})
  rlog_data = prefix + b"\xff" * 32
  (segment[1] / "rlog").write_bytes(rlog_data)
  value = len(rlog_data) + segment[2].stat().st_size - 1 if limit == "max_total_decoded_bytes" else 4
  with pytest.raises(box.ResourceLimitExceeded):
    build(tmp_path, [segment], route_first_mono=100, limits=replace(box.WorkLimits(), **{limit: value}))
  assert list((tmp_path / "blackbox").iterdir()) == []


def fake_remux(monkeypatch):
  def run(command, output, budget):
    assert "-ss" not in command and "-t" not in command
    output.write_bytes(b"mock-mp4")
    return True

  monkeypatch.setattr(box, "_find_ffmpeg", lambda: "ffmpeg")
  monkeypatch.setattr(box, "_run_ffmpeg", run)


def test_missing_first_video_clips_duration_to_available_window(tmp_path, monkeypatch):
  fake_remux(monkeypatch)
  segments = [make_segment(tmp_path, 0, video=None), make_segment(tmp_path, 1)]
  result = build(tmp_path, segments, event(50, 90))
  video = result["manifest"]["video"]
  assert video["sourceSegments"] == [1]
  assert video["playbackStartS"] == 0
  assert video["playbackDurationS"] == 30  # Not the unavailable 50..60 seconds.
  assert video["partialWindow"] is True
  assert video["selectedStartOffsetS"] == 60
  assert video["selectedEndOffsetS"] == 120
  assert video["availableStartOffsetS"] == 60
  assert video["availableEndOffsetS"] == 90
  assert video["synchronization"] == {"status": "approximate", "basis": "nominal_60s_segments", "frameAccurate": False,
                                      "routeAlignmentVerified": False, "actualStartOffsetS": None, "actualEndOffsetS": None,
                                      "maxErrorS": None}
  assert video["mediaDurationS"] is None and video["coverageVerified"] is False


@pytest.mark.parametrize("failure,reason", [(None, "mp4_remux_failed"), (OSError(), "ffmpeg_execution_failed"),
                                          (box._VideoUnavailable(), "video_processing_limit")])
def test_remux_failure_keeps_signals_without_advertising_video(tmp_path, monkeypatch, failure, reason):
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: "ffmpeg")

  def fail(command, output, budget):
    output.write_bytes(b"partial-mp4")
    if failure is not None:
      raise failure
    return False

  monkeypatch.setattr(box, "_run_ffmpeg", fail)
  result = build(tmp_path, [make_segment(tmp_path)])
  assert result["videoAvailable"] is False and result["package"]["videoAvailable"] is False
  assert result["videoUnavailableReason"] == result["package"]["videoUnavailableReason"] == reason
  assert result["manifest"]["video"]["reason"] == reason
  assert result["manifest"]["video"]["file"] is None
  assert result["signalCounts"]["carState"] == 1
  with zipfile.ZipFile(result["path"]) as archive:
    assert set(archive.namelist()) == {"manifest.json", "signals.jsonl.gz"}
  assert not list((tmp_path / "blackbox").glob(".wayon-blackbox-*"))


@pytest.mark.parametrize("failure", [box.WorkAborted("onroad"), box.ResourceLimitExceeded("work deadline")])
def test_remux_does_not_swallow_global_work_revocation(tmp_path, monkeypatch, failure):
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: "ffmpeg")

  def fail(*args):
    raise failure

  monkeypatch.setattr(box, "_run_ffmpeg", fail)
  with pytest.raises(type(failure)):
    build(tmp_path, [make_segment(tmp_path)])
  assert list((tmp_path / "blackbox").iterdir()) == []


def test_oversize_video_still_allows_bounded_signals_only_package(tmp_path, monkeypatch):
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: "ffmpeg")
  result = build(tmp_path, [make_segment(tmp_path)], limits=box.WorkLimits(max_video_bytes=1))
  assert result["videoAvailable"] is False
  assert result["videoUnavailableReason"] == "video_size_limit"
  assert result["signalCounts"]["carState"] == 1


def test_missing_video_is_explicit_not_a_playable_package(tmp_path):
  result = build(tmp_path, [make_segment(tmp_path, video=None)])
  assert result["videoAvailable"] is False
  assert result["manifest"]["video"]["reason"] == "qcamera_missing"
  with zipfile.ZipFile(result["path"]) as archive:
    assert set(archive.namelist()) == {"manifest.json", "signals.jsonl.gz"}


def test_missing_route_origin_needs_explicit_origin(tmp_path):
  segment = make_segment(tmp_path, 1)
  with pytest.raises(box.WorkAborted, match="route origin"):
    build(tmp_path, [segment], event(60, 90))
  result = build(tmp_path, [segment], event(60, 90), route_first_mono=100)
  assert result["manifest"]["routeFirstMonoS"] == 100
  assert result["signalCounts"]["carState"] == 1


def test_no_unrelated_segment_is_substituted_for_event(tmp_path):
  assert build(tmp_path, [make_segment(tmp_path)], event(200, 230)) is None


@pytest.mark.parametrize("start,end", [(float("nan"), 30), (0, float("inf")), (-1, 30), (30, 20), (0, 61)])
def test_invalid_window_is_rejected_before_files_created(tmp_path, start, end):
  with pytest.raises(ValueError):
    build(tmp_path, [make_segment(tmp_path)], event(start, end))
  assert not (tmp_path / "blackbox").exists()


@pytest.mark.parametrize("filename", ["qlog", "qlog.bz2", "qlog.zst", "rlog", "rlog.bz2", "rlog.zst"])
def test_native_cereal_reader_supports_bounded_formats(tmp_path, filename):
  _, _, path = make_segment(tmp_path, filename=filename)
  assert [msg.which() for msg in box.iter_route_log(path, allowed=lambda: True)] == ["carState", "can", "carControl", "longitudinalPlan"]


@pytest.mark.parametrize("limit", ["max_log_bytes", "max_decoded_log_bytes", "max_total_decoded_bytes", "max_messages"])
def test_reader_limits_are_enforced(tmp_path, limit):
  _, _, path = make_segment(tmp_path, filename="qlog.bz2")
  with pytest.raises(box.ResourceLimitExceeded):
    list(box.iter_route_log(path, allowed=lambda: True, limits=replace(box.WorkLimits(), **{limit: 1})))


def test_reader_aborts_on_offroad_revocation(tmp_path):
  _, _, path = make_segment(tmp_path)
  checks = 0

  def allowed():
    nonlocal checks
    checks += 1
    return checks <= 2

  with pytest.raises(box.WorkAborted):
    list(box.iter_route_log(path, allowed=allowed))


def test_total_budget_is_shared_across_segments(tmp_path):
  first = make_segment(tmp_path)
  second = make_segment(tmp_path, 1)
  limits = replace(box.WorkLimits(), max_total_decoded_bytes=first[2].stat().st_size + 1)
  budget = box.WorkBudget(lambda: True, limits)
  list(box.iter_route_log(first[2], budget=budget))
  with pytest.raises(box.ResourceLimitExceeded):
    list(box.iter_route_log(second[2], budget=budget))


def test_locked_and_symlink_inputs_are_rejected(tmp_path):
  _, directory, path = make_segment(tmp_path)
  lock = directory / "rlog.lock"
  lock.touch()
  with pytest.raises(box.WorkAborted):
    list(box.iter_route_log(path, allowed=lambda: True))
  lock.unlink()
  data = directory / "original"
  path.rename(data)
  path.symlink_to(data)
  with pytest.raises(box.WorkAborted):
    list(box.iter_route_log(path, allowed=lambda: True))


@pytest.mark.parametrize("limits", [box.WorkLimits(max_signal_bytes=1), box.WorkLimits(max_archive_bytes=1)])
def test_failed_packaging_leaves_no_zip_or_temp_files(tmp_path, limits):
  with pytest.raises(box.ResourceLimitExceeded):
    build(tmp_path, [make_segment(tmp_path)], limits=limits)
  assert list((tmp_path / "blackbox").iterdir()) == []


def test_packaging_does_not_start_or_create_files_onroad(tmp_path):
  segment = make_segment(tmp_path)
  with pytest.raises(box.WorkAborted):
    box.build_blackbox_archive({"segments": [segment]}, event(), "trip", output_root=tmp_path / "out", allowed=lambda: False)
  assert not (tmp_path / "out").exists()


def test_path_names_cannot_escape_output_root(tmp_path):
  segment = make_segment(tmp_path)
  group = {"route_name": "../../other", "segments": [segment]}
  incident = {**event(), "id": "../../secret.zip"}
  result = box.build_blackbox_archive(group, incident, "../../../trip", output_root=tmp_path / "out", allowed=lambda: True)
  assert Path(result["path"]).parent == tmp_path / "out"
  assert Path(result["path"]).name.startswith("wayon-")


def test_pruning_only_removes_owned_archives(tmp_path):
  for index in range(3):
    path = tmp_path / f"wayon-{index:032x}.zip"
    path.write_bytes(bytes([index]) * 10)
    os.utime(path, (index, index))
  foreign = tmp_path / "other.zip"
  foreign.write_bytes(b"preserve")
  (tmp_path / f"wayon-{'f' * 32}.zip").symlink_to(foreign)
  removed = box.prune_blackbox_archives(tmp_path, max_archives=2, max_storage_bytes=20, allowed=lambda: True)
  assert removed == [str(tmp_path / f"wayon-{0:032x}.zip")]
  assert foreign.read_bytes() == b"preserve"
  assert (tmp_path / f"wayon-{'f' * 32}.zip").is_symlink()


def test_pruning_is_offroad_only(tmp_path):
  path = tmp_path / f"wayon-{'a' * 32}.zip"
  path.write_bytes(b"keep")
  with pytest.raises(box.WorkAborted):
    box.prune_blackbox_archives(tmp_path, max_archives=0, allowed=lambda: False)
  assert path.exists()


def test_ffmpeg_timeout_terminates_only_its_owned_child(tmp_path, monkeypatch):
  original_popen = subprocess.Popen
  owned = []

  def record(*args, **kwargs):
    process = original_popen(*args, **kwargs)
    owned.append(process)
    return process

  sibling = original_popen([sys.executable, "-c", "import time; time.sleep(60)"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  monkeypatch.setattr(box.subprocess, "Popen", record)
  try:
    budget = box.WorkBudget(lambda: True, box.WorkLimits(ffmpeg_seconds=0.1))
    with pytest.raises(box.ResourceLimitExceeded, match="ffmpeg deadline"):
      box._run_ffmpeg([sys.executable, "-c", "import time; time.sleep(60)"], tmp_path / "out.mp4", budget)
    assert owned and owned[0].poll() is not None
    assert sibling.poll() is None
  finally:
    sibling.terminate()
    sibling.wait(timeout=2)


def test_ffmpeg_revocation_kills_unresponsive_owned_child(monkeypatch, tmp_path):
  calls = []

  class Process:
    returncode = None

    def poll(self):
      return self.returncode

    def terminate(self):
      calls.append("terminate")

    def kill(self):
      calls.append("kill")
      self.returncode = -9

    def wait(self, timeout):
      calls.append(("wait", timeout))
      if self.returncode is None:
        raise subprocess.TimeoutExpired("owned", timeout)
      return self.returncode

  checks = iter([True, False])
  monkeypatch.setattr(box.subprocess, "Popen", lambda *args, **kwargs: Process())
  with pytest.raises(box.WorkAborted):
    box._run_ffmpeg(["unused"], tmp_path / "out.mp4", box.WorkBudget(lambda: next(checks)))
  assert calls == ["terminate", ("wait", 0.5), "kill", ("wait", 1.0)]


@pytest.mark.parametrize("indices,seconds,start,end", [([0], 2, 0.3, 1.3), ([0, 1], 60, 57.3, 63.3), ([1], 10, 55, 65)])
def test_native_ffmpeg_non_keyframe_seek_and_segment_boundary(tmp_path, monkeypatch, no_implicit_native_process, indices, seconds, start, end):
  executable = no_implicit_native_process()
  if not executable:
    pytest.skip("native ffmpeg not available in this test environment")
  encoders = subprocess.run([executable, "-hide_banner", "-encoders"], check=True, timeout=5, text=True,
                            stdin=subprocess.DEVNULL, capture_output=True).stdout
  available_encoders = {fields[1] for line in encoders.splitlines() if len(fields := line.split()) >= 2}
  if not {"libx264", "rawvideo"} <= available_encoders:
    pytest.skip("synthetic fixture requires libx264/rawvideo encoders; production only needs remux")
  segments = []
  frame_bytes = 64 * 48 * 3 // 2
  for index in indices:
    segment = make_segment(tmp_path, index, video=None)
    segments.append(segment)
    raw = tmp_path / f"segment-{index}.yuv"
    raw.write_bytes((bytes([16 if index == 0 else 200]) * (64 * 48) + bytes([128]) * (64 * 48 // 2)) * (seconds * 10))
    subprocess.run([executable, "-nostdin", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
                    "-pixel_format", "yuv420p", "-video_size", "64x48", "-framerate", "10", "-i", str(raw),
                    "-c:v", "libx264", "-threads", "1", "-g", "20", "-keyint_min", "20", "-sc_threshold", "0", "-bf", "2",
                    "-f", "mpegts", str(segment[1] / "qcamera.ts")], check=True, timeout=15,
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
  monkeypatch.setattr(box, "_find_ffmpeg", lambda: executable)
  result = build(tmp_path, segments, event(start, end), route_first_mono=100)
  video = result["manifest"]["video"]
  assert result["videoAvailable"] is True and result["package"]["videoUnavailableReason"] is None
  assert video["remuxMode"] == "whole_selected_segments"
  assert video["synchronization"]["frameAccurate"] is False
  assert video["playbackStartS"] == round(max(start, indices[0] * 60) - indices[0] * 60, 3)
  assert video["playbackDurationS"] == round(end - max(start, indices[0] * 60), 3)
  with zipfile.ZipFile(result["path"]) as archive:
    assert "road.mp4" in archive.namelist()
    assert "road.ts" not in archive.namelist()
    assert b"ftyp" in archive.read("road.mp4")[:32]
    assert video["contentType"] == "video/mp4"
    movie = tmp_path / "road.mp4"
    movie.write_bytes(archive.read("road.mp4"))
  decoded = subprocess.run([executable, "-nostdin", "-hide_banner", "-loglevel", "error", "-ss", str(video["playbackStartS"]),
                            "-i", str(movie), "-t", str(video["playbackDurationS"]), "-an", "-threads", "1",
                            "-pix_fmt", "yuv420p", "-f", "rawvideo", "pipe:1"], check=True, timeout=15,
                           stdin=subprocess.DEVNULL, capture_output=True).stdout
  assert len(decoded) % frame_bytes == 0
  frames = len(decoded) // frame_bytes
  assert abs(frames - video["playbackDurationS"] * 10) <= 1
  # The synthetic dark/bright transition pins the selected segment boundary.
  bright = [decoded[index * frame_bytes] > 100 for index in range(frames)]
  if indices == [0, 1]:
    assert abs(bright.index(True) - (60 - start) * 10) <= 1
    assert all(bright[bright.index(True):])
  else:
    assert all(value == (indices[0] == 1) for value in bright)
