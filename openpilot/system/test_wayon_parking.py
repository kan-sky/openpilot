from types import SimpleNamespace
import json
import os
import subprocess

import pytest

from openpilot.system import wayon_parking as parking


class State(dict):
  def __init__(self, now=100):
    super().__init__(deviceState=SimpleNamespace(started=True, thermalStatus="green", memoryUsagePercent=70),
                     carState=SimpleNamespace(vEgo=3, gearShifter="drive"))
    self.seen = {k: True for k in self}
    self.valid = {k: True for k in self}
    self.recv_time = {k: now for k in self}
    self.logMonoTime = {k: int(now * 1e9) for k in self}


class Reader:
  def __init__(self):
    self.calls = 0
    self.closed = 0
    self.fail = False

  def close(self):
    self.closed += 1

  def __call__(self, now):
    self.calls += 1
    if self.fail:
      raise RuntimeError("camera unavailable")
    return parking.ApproachFrame(now, 1000 + now, b"jpeg")


def frame(at, raw=b"jpeg"):
  return parking.ApproachFrame(at, 1000 + at, raw)


@pytest.mark.parametrize("field,value", [
  ("vEgo", 0), ("vEgo", 7), ("vEgo", float("nan")), ("vEgo", float("inf")),
  ("gearShifter", "park"), ("gearShifter", "unknown"), ("started", False),
  ("memoryUsagePercent", 85), ("memoryUsagePercent", float("nan")),
  ("thermalStatus", "yellow"), ("thermalStatus", "red"),
])
def test_ineligible_input_never_reads_camera(field, value):
  sm = State()
  setattr(sm["carState"] if field in ("vEgo", "gearShifter") else sm["deviceState"], field, value)
  reader = Reader()
  parking.ApproachBuffer(reader).sample(sm, 100)
  assert reader.calls == 0


@pytest.mark.parametrize("service", ["deviceState", "carState"])
@pytest.mark.parametrize("bad", ["unseen", "invalid", "old_receive", "old_source", "future"])
def test_untrusted_sample_never_reads(service, bad):
  sm = State()
  if bad == "unseen":
    sm.seen[service] = False
  elif bad == "invalid":
    sm.valid[service] = False
  elif bad == "old_receive":
    sm.recv_time[service] = 98
  elif bad == "old_source":
    sm.logMonoTime[service] = int(98e9)
  else:
    sm.logMonoTime[service] = int(101e9)
  reader = Reader()
  parking.ApproachBuffer(reader).sample(sm, 100)
  assert reader.calls == 0


def test_sampling_interval_failure_and_backoff(monkeypatch):
  clock = [100.0]
  monkeypatch.setattr(parking.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(parking.time, "time", lambda: clock[0] + 1000)
  reader = Reader()
  ring = parking.ApproachBuffer(reader)
  ring.sample(State(100))
  clock[0] = 101
  ring.sample(State(101))
  assert reader.calls == 1 and ring.captured_count == 1
  clock[0] = 102
  reader.fail = True
  ring.sample(State(102))
  assert ring.skipped_count == 1 and reader.closed == 1
  def slow(now):
    clock[0] += 0.2
    return frame(now)
  reader.fail = False
  class Slow:
    def __call__(self, now):
      return slow(now)
    def close(self):
      pass
  ring.reader = Slow()
  clock[0] = 104
  ring.sample(State(104))
  assert ring.next_sample == pytest.approx(114.2)


def test_bounds_age_clock_and_adjacent_pair_selection():
  ring = parking.ApproachBuffer(Reader())
  for t in range(100, 180, 2):
    assert ring.add(frame(t), t, t + 1000)
  assert len(ring.frames) == 24
  chosen = ring.take(180, 1180)
  assert [f.monotonic for f in chosen] == [132, 134, 154, 156, 176, 178]
  assert ring.bytes == 0 and not ring.frames
  ring.add(frame(180), 180, 1180)
  assert ring.take(241, 1241) == []
  ring.add(frame(250), 250, 1250)
  assert ring.take(251, 1300) == []


def test_jpeg_and_total_byte_caps():
  ring = parking.ApproachBuffer(Reader())
  assert not ring.add(frame(100, b""), 100, 1100)
  assert not ring.add(frame(100, b"x" * (parking.MAX_FRAME_BYTES + 1)), 100, 1100)
  for t in range(100, 140, 2):
    assert ring.add(frame(t, b"x" * parking.MAX_FRAME_BYTES), t, t + 1000)
  assert ring.bytes == parking.MAX_BYTES and len(ring.frames) == 8


def test_no_duplicate_stale_or_future_frames():
  ring = parking.ApproachBuffer(Reader())
  assert ring.add(frame(100), 100, 1100)
  assert not ring.add(frame(100), 100, 1100)
  assert not ring.add(frame(101), 102, 1102)
  assert not ring.add(frame(103), 102, 1102)
  assert not ring.add(parking.ApproachFrame(102, 1600, b"jpeg"), 102, 1102)


@pytest.mark.parametrize("field,value", [("vEgo", 10), ("memoryUsagePercent", 85), ("thermalStatus", "yellow")])
def test_high_speed_or_resource_pressure_releases_history(field, value):
  ring = parking.ApproachBuffer(Reader())
  ring.add(frame(100), 100, 1100)
  sm = State(101)
  setattr(sm["carState"] if field == "vEgo" else sm["deviceState"], field, value)
  ring.sample(sm, 101)
  assert not ring.frames and ring.bytes == 0


def test_pending_frames_expire_or_reject_clock_jump_before_upload():
  frames = [frame(100), frame(102)]
  assert len(parking.upload_frames(frames, 1120, now=120, wall=1120)) == 2
  assert parking.upload_frames(frames, 1120, now=240, wall=1240) == []
  assert parking.upload_frames(frames, 1120, now=120, wall=2120) == []
  assert parking.upload_frames(frames, 1000, now=120, wall=1120) == []
  assert len(parking.upload_frames([frame(t) for t in range(100, 120)], 1120, now=120, wall=1120)) == 6


def test_spool_roundtrip_is_bounded_and_removes_consumed_files(tmp_path, monkeypatch):
  monkeypatch.setattr(parking.time, 'monotonic', lambda: 120)
  monkeypatch.setattr(parking.time, 'time', lambda: 1120)
  frames = [frame(t, f'jpeg-{t}'.encode()) for t in range(100, 112, 2)]
  directory = tmp_path / 'owned'
  directory.mkdir()
  parking.write_history(directory, frames)
  (directory / 'obsolete.jpg').write_bytes(b'old')
  parking.write_history(directory, frames)
  assert not (directory / 'obsolete.jpg').exists()
  session = parking.ApproachSession()
  session.directory = directory
  assert session.take() == frames
  assert not directory.exists()


@pytest.mark.parametrize('corrupt', ['hash', 'traversal', 'symlink', 'clock', 'order', 'too_many', 'too_big'])
def test_spool_corruption_never_reaches_uploader(tmp_path, monkeypatch, corrupt):
  monkeypatch.setattr(parking.time, 'monotonic', lambda: 120)
  monkeypatch.setattr(parking.time, 'time', lambda: 1120)
  directory = tmp_path / 'owned'
  directory.mkdir()
  parking.write_history(directory, [frame(100), frame(102, b'other')])
  manifest = directory / 'frames.json'
  rows = json.loads(manifest.read_text())
  file = directory / rows[0]['file']
  if corrupt == 'hash':
    file.write_bytes(b'corrupt')
  elif corrupt == 'traversal':
    rows[0]['file'] = '../secret'
  elif corrupt == 'symlink':
    target = tmp_path / 'outside'
    target.write_bytes(b'jpeg')
    file.unlink()
    file.symlink_to(target)
  elif corrupt == 'clock':
    rows[0]['captured_at'] += 999
  elif corrupt == 'order':
    rows.reverse()
  elif corrupt == 'too_many':
    rows = rows * 13
  else:
    file.write_bytes(b'x' * (parking.MAX_FRAME_BYTES + 1))
  manifest.write_text(json.dumps(rows))
  session = parking.ApproachSession()
  session.directory = directory
  assert session.take() == []
  assert not directory.exists()


def test_parent_never_replaces_unreaped_child(tmp_path):
  class Process:
    def poll(self): return None
    def terminate(self): pass
    def kill(self): pass
    def wait(self, timeout): raise subprocess.TimeoutExpired('camera', timeout)
  session = parking.ApproachSession()
  session.process = Process()
  session.directory = tmp_path
  session.clear()
  assert session.process is not None and tmp_path.exists()
  assert session.take() == []


@pytest.mark.parametrize('reason', ['offroad', 'stale', 'ineligible', 'parent_gone', 'crash'])
def test_dead_child_history_is_accepted_only_after_clean_offroad(tmp_path, monkeypatch, reason):
  monkeypatch.setattr(parking.time, 'monotonic', lambda: 100)
  monkeypatch.setattr(parking.time, 'time', lambda: 1100)
  directory = tmp_path / 'owned'
  directory.mkdir()
  frames = [frame(96), frame(98, b'jpeg2')]
  parking.write_history(directory, frames)
  if reason != 'crash':
    parking.finish_history(directory, frames, reason)
  session = parking.ApproachSession()
  session.directory = directory
  session.process = SimpleNamespace(poll=lambda: 0 if reason != 'crash' else 1)
  assert session.take() == (frames if reason == 'offroad' else [])
  assert not directory.exists()


def test_offroad_closes_history_without_fresh_card_but_never_captures():
  sm = State(100)
  sm['deviceState'].started = False
  sm.seen['carState'] = False
  assert parking.history_exit_reason(sm, 100) == 'offroad'
  assert not parking.approach_allowed(sm, 100)
  sm.seen['deviceState'] = False
  assert parking.history_exit_reason(sm, 100) is None


@pytest.mark.parametrize('field,value', [('thermalStatus', 'yellow'), ('memoryUsagePercent', 85),
                                        ('memoryUsagePercent', float('nan')), ('vEgo', 10)])
def test_resource_or_high_speed_rejection_wins_over_offroad(field, value):
  sm = State(100)
  sm['deviceState'].started = False
  setattr(sm['carState'] if field == 'vEgo' else sm['deviceState'], field, value)
  assert parking.history_exit_reason(sm, 100) == 'ineligible'


def test_cleanup_only_dead_private_owned_spool(tmp_path, monkeypatch):
  dead = tmp_path / 'wayon-parking-123456-dead'
  alive = tmp_path / 'wayon-parking-123457-live'
  public = tmp_path / 'wayon-parking-123458-public'
  unrelated = tmp_path / 'unrelated'
  for path in (dead, alive, public, unrelated):
    path.mkdir(mode=0o700)
  public.chmod(0o755)
  def alive_check(pid, sig):
    if pid != 123457:
      raise ProcessLookupError()
  monkeypatch.setattr(parking.os, 'kill', alive_check)
  parking.cleanup_abandoned(tmp_path)
  assert not dead.exists()
  assert alive.exists() and public.exists() and unrelated.exists()


def test_session_spawns_only_eligible_and_bounds_dead_worker(tmp_path, monkeypatch):
  class Process:
    def __init__(self): self.returncode = None
    def poll(self): return self.returncode
    def terminate(self): self.returncode = -15
    def wait(self, timeout): return self.returncode
  launched = []
  def start(args, **kwargs):
    launched.append((args, kwargs))
    return Process()
  directory = tmp_path / 'owned'
  directory.mkdir()
  clock = [100]
  monkeypatch.setattr(parking.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(parking.tempfile, 'mkdtemp', lambda **kwargs: str(directory))
  monkeypatch.setattr(parking.subprocess, 'Popen', start)
  session = parking.ApproachSession()
  session.sample(State(100))
  assert len(launched) == 1
  args, options = launched[0]
  assert '--worker' in args and options['start_new_session']
  assert options['env']['OMP_NUM_THREADS'] == '1'
  clock[0] = 101
  session.sample(State(101))
  assert len(launched) == 1
  clock[0] = 111
  session.sample(State(111))  # No heartbeat; terminate optional work, then back off.
  assert session.process is None and session.retry_after == 141
  assert not directory.exists()
