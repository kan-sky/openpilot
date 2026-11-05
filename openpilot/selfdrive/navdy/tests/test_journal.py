import errno
import json
import os
import stat
import threading
import time
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import journal
from openpilot.selfdrive.navdy.diagnostics import LinkDiagnostics


def event(sequence=1, **fields):
  return {'kind': 'transport_open', 'runId': 'a' * 32, 'eventSeq': sequence,
          'at': 100. + sequence, 'wallTime': 1792782000. + sequence, 'transport': 'bluetooth', **fields}


def records(directory):
  return [json.loads(line) for path in directory.glob('events*.jsonl') for line in path.read_text().splitlines()]


def wait_for(predicate):
  until = time.perf_counter() + 2
  while not predicate():
    assert time.perf_counter() < until
    threading.Event().wait(.005)


def test_async_identity_permissions_and_restart_preserve_history(tmp_path, monkeypatch):
  directory = tmp_path.resolve() / 'logs'
  monkeypatch.setattr(journal, 'boot_id', lambda: 'b' * 32)
  for _ in range(2):
    sink = journal.LocalJournal(directory)
    diagnostics = LinkDiagnostics(lambda *a, **kw: None, journal=sink)
    diagnostics.record('bridge_started')
    diagnostics.record('bridge_stopped')
    diagnostics.close()
    assert not sink.thread.is_alive()
    assert sink.written == 2
  rows = records(directory)
  assert len(rows) == 4
  assert len({row['runId'] for row in rows}) == 2
  assert {row['bootId'] for row in rows} == {'b' * 32}
  assert {row['pid'] for row in rows} == {os.getpid()}
  assert all(row['at'] > 0 and row['wallTime'] > 0 and row['schema'] == 1 for row in rows)
  assert stat.S_IMODE(directory.stat().st_mode) == 0o700
  assert all(stat.S_IMODE(path.stat().st_mode) == 0o600 for path in directory.iterdir())


def test_boot_change_is_distinct_from_run_restart(tmp_path, monkeypatch):
  directory = tmp_path.resolve() / 'logs'
  for boot in ('b' * 32, 'c' * 32):
    monkeypatch.setattr(journal, 'boot_id', lambda boot=boot: boot)
    sink = journal.LocalJournal(directory)
    sink.submit(event())
    sink.close()
  assert {row['bootId'] for row in records(directory)} == {'b' * 32, 'c' * 32}


def test_usb_recovery_and_original_connect_failure_are_retained_without_identity(tmp_path):
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  diagnostics = LinkDiagnostics(lambda *args, **kwargs: None, journal=sink)
  diagnostics.record('usb_recovery', local_only=True, transport='usb', usbRecovery={
    'action': 'reset', 'failureAge': 31., 'resetCount': 1, 'serial': 'PRIVATE'})
  diagnostics.record('connect_failed', transport='usb', reason='Navdy USB not found')
  sink.close()
  saved = records(tmp_path / 'logs')
  assert saved[0]['usbRecovery'] == {'action': 'reset', 'failureAge': 31., 'resetCount': 1}
  assert saved[1]['reason'] == 'Navdy USB not found'
  assert 'PRIVATE' not in json.dumps(saved)


def test_feedback_counters_survive_sanitizer_and_disk_without_payloads(tmp_path):
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  diagnostics = LinkDiagnostics(lambda *a, **kw: None, journal=sink)
  diagnostics.feedback({'navigationFeedback': None, 'modelNavigation': None},
                       transport='bluetooth', received_at=100., elapsed_ms=1., wireless={})
  sink.submit(event(feedback={**diagnostics.feedback_stats, 'privatePayload': 'secret'}))
  sink.close()
  assert records(tmp_path / 'logs')[0]['feedback'] == {
    'noObservation': {'count': 1, 'lastReceivedAt': 100.},
    'observation': {'count': 0}, 'incomplete': {'count': 0},
  }


def test_transmit_metrics_survive_diagnostics_and_disk(tmp_path):
  metrics = dict.fromkeys(('ackPollDelayMs', 'txPendingBytes', 'txAgeMs', 'lastTxMs',
                          'partialWriteCount', 'blockedWriteCount'), 12)
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  clean = LinkDiagnostics.metrics({**metrics, 'payload': 'secret', 'peer': 'secret'})
  sink.submit(event(wireless=clean))
  sink.close()
  assert clean == metrics
  assert records(tmp_path / 'logs')[0]['wireless'] == metrics


@pytest.mark.parametrize('kind', ['connect_failed', 'transport_open', 'failure'])
def test_fixed_connect_timings_survive_journal_without_private_or_unknown_keys(tmp_path, kind):
  phases = ('bluetoothctl', 'sdp', 'socket_create', 'security_request', 'socket', 'security', 'hello', 'pipeline')
  assert set(journal.CONNECT_TIMINGS) == set(phases)
  expected = {phase: {'ok': phase != 'hello', 'elapsedMs': 12.5} for phase in phases}
  private = 'PRIVATE 54:ED:A3:00:48:23 /data/private token'
  dirty = {phase: {**value, 'reason': private, 'peer': private, 'command': [private]} for phase, value in expected.items()}
  dirty.update({private: {'ok': True, 'elapsedMs': 10}, 'unknown': {'reason': private}, 'steps': [private]})
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  diagnostics = LinkDiagnostics(lambda *args, **kwargs: None, journal=sink)
  diagnostics.record(kind, phase='connect', transport='bluetooth', connectTimings=dirty)
  dirty['sdp']['elapsedMs'] = 999
  diagnostics.close()
  rows = records(sink.files.directory)
  assert len(rows) == 1 and sink.dropped == 0
  assert rows[0]['connectTimings'] == expected
  assert private not in json.dumps(rows)


@pytest.mark.parametrize('value', [[], [{'phase': 'sdp', 'ok': True, 'elapsedMs': 1}], 'PRIVATE', None])
def test_connect_timings_schema_never_accepts_an_arbitrary_list_or_text(value):
  assert 'connectTimings' not in journal.sanitize(event(connectTimings=value))


def test_connect_timings_schema_rejects_invalid_values_and_bridge_uses_same_whitelist():
  from openpilot.selfdrive.navdy.bridge import connect_timings

  dirty = {'steps': [
    {'phase': 'sdp', 'ok': 'PRIVATE', 'elapsedMs': float('inf'), 'peer': 'PRIVATE'},
    {'phase': 'socket', 'ok': True, 'elapsedMs': 1.5, 'reason': 'PRIVATE'},
    {'phase': 'hello', 'ok': False, 'elapsedMs': float('nan')},
    {'phase': 'unknown', 'ok': True, 'elapsedMs': 99},
    {'phase': ['PRIVATE'], 'ok': True, 'elapsedMs': 1},
    'PRIVATE',
  ]}
  expected = {'sdp': {}, 'socket': {'ok': True, 'elapsedMs': 1.5}, 'hello': {'ok': False}}
  assert connect_timings(dirty) == expected
  assert journal.sanitize(event(connectTimings=connect_timings(dirty)))['connectTimings'] == expected
  assert connect_timings(None) is None


def test_rotation_bounds_disk_across_restarts_and_repairs_incomplete_tail(tmp_path):
  directory = tmp_path.resolve() / 'logs'
  for run in range(3):
    files = journal.JournalFiles(directory, max_bytes=512)
    try:
      for sequence in range(30):
        data = json.dumps(event(run * 30 + sequence)).encode() + b'\n'
        files.write(data)
    finally:
      files.close()
    assert len(list(directory.glob('events*.jsonl'))) == journal.FILE_COUNT
    assert all(path.stat().st_size <= 512 for path in directory.iterdir())
    assert sum(path.stat().st_size for path in directory.iterdir()) <= journal.FILE_COUNT * 512
  with (directory / 'events.jsonl').open('ab') as stream:
    stream.write(b'{"interrupted":')
  files = journal.JournalFiles(directory, max_bytes=512)
  files.write(json.dumps(event(100)).encode() + b'\n')
  files.close()
  rows = records(directory)
  assert max(row['eventSeq'] for row in rows) == 100
  assert min(row['eventSeq'] for row in rows) > 0


@pytest.mark.parametrize('target', ['directory', 'ancestor', 'events.jsonl', 'events.1.jsonl', '.lock'])
def test_symlinks_are_never_followed(tmp_path, target):
  root = tmp_path.resolve()
  outside = root / 'outside'
  outside.mkdir()
  secret = outside / 'secret'
  secret.write_text('PRIVATE')
  secret.chmod(0o644)
  directory = root / 'logs'
  if target == 'directory':
    directory.symlink_to(outside, target_is_directory=True)
  elif target == 'ancestor':
    directory.symlink_to(outside, target_is_directory=True)
    directory = directory / 'child'
  else:
    directory.mkdir()
    (directory / target).symlink_to(secret)
  files = journal.JournalFiles(directory)
  with pytest.raises(OSError):
    files.write(b'{}\n')
  files.close()
  assert secret.read_text() == 'PRIVATE'
  assert stat.S_IMODE(secret.stat().st_mode) == 0o644
  assert list(outside.iterdir()) == [secret]


@pytest.mark.parametrize('kind', ['hardlink', 'fifo'])
def test_non_regular_or_shared_files_are_rejected_without_modifying_them(tmp_path, kind):
  directory = tmp_path.resolve() / 'logs'
  directory.mkdir()
  outside = tmp_path / 'secret'
  outside.write_text('PRIVATE')
  outside.chmod(0o644)
  path = directory / 'events.jsonl'
  if kind == 'hardlink':
    os.link(outside, path)
  else:
    os.mkfifo(path)
  files = journal.JournalFiles(directory)
  with pytest.raises(OSError):
    files.write(b'{}\n')
  files.close()
  assert outside.read_text() == 'PRIVATE'
  assert stat.S_IMODE(outside.stat().st_mode) == 0o644


def test_exclusive_writer_lock_and_descriptor_cleanup(tmp_path):
  directory = tmp_path.resolve() / 'logs'
  first, second = journal.JournalFiles(directory), journal.JournalFiles(directory)
  try:
    first.write(b'{}\n')
    with pytest.raises(OSError):
      second.write(b'{}\n')
    assert second.fd is second.directory_fd is second.lock_fd is None
    first.close()
    second.write(b'{}\n')
  finally:
    first.close()
    second.close()
  assert len(records(directory)) == 2


def test_existing_permissions_and_oversize_files_are_bounded(tmp_path):
  directory = tmp_path.resolve() / 'logs'
  directory.mkdir(mode=0o755)
  for index in range(journal.FILE_COUNT):
    path = directory / journal.JournalFiles.name(index)
    path.write_bytes(b'x' * 600)
    path.chmod(0o644)
  files = journal.JournalFiles(directory, max_bytes=512)
  files.write(b'{}\n')
  files.close()
  assert stat.S_IMODE(directory.stat().st_mode) == 0o700
  assert all(path.stat().st_size <= 512 and stat.S_IMODE(path.stat().st_mode) == 0o600 for path in directory.iterdir())


def test_strict_sanitizer_excludes_payloads_private_enum_values_and_nonfinite_numbers(tmp_path):
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  private = 'PRIVATE 54:ED:A3:00:48:23 /data/private token music location'
  data = event(reason=private, errorType=private, payload={'location': private},
               wireless={'rttMs': float('nan'), 'ackAgeMs': float('inf'), 'sequence': 1 << 300, 'peer': private},
               receiver={'wireless': {'owner': private, 'session': 3}, 'power': {'state': private, 'fresh': True}},
               state={'power': 'offroad', 'ambient': 'offroad_door', 'missing': {'carState': private}})
  assert sink.submit(data)
  data['receiver']['wireless']['session'] = 999
  sink.close()
  rows = records(sink.files.directory)
  assert len(rows) == 1
  assert rows[0]['wireless'] == {}
  assert rows[0]['receiver']['wireless'] == {'session': 3}
  assert 'PRIVATE' not in json.dumps(rows)
  assert 'payload' not in rows[0]
  json.dumps(rows, allow_nan=False)


def test_all_storage_calls_are_off_producer_and_queue_saturation_stop_are_bounded(tmp_path, monkeypatch):
  entered, release = threading.Event(), threading.Event()
  clock = NS(now=1.)
  monkeypatch.setattr(journal, 'time', NS(monotonic=lambda: clock.now))
  writer_threads = []

  def blocked(_files, _data):
    writer_threads.append(threading.get_ident())
    entered.set()
    assert release.wait(3)

  monkeypatch.setattr(journal.JournalFiles, 'write', blocked)
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  try:
    assert sink.submit(event())
    assert entered.wait(1)
    started = time.perf_counter()
    for sequence in range(journal.QUEUE_SIZE + 20):
      clock.now += 1
      sink.submit(event(sequence))
    assert time.perf_counter() - started < .5
    assert sink.queue.qsize() == journal.QUEUE_SIZE
    assert sink.dropped == 20
    assert sum(len(item) for item in sink.queue.queue) <= journal.QUEUE_SIZE * journal.RECORD_BYTES
    started = time.perf_counter()
    sink.close()
    assert time.perf_counter() - started < .6
    assert sink.thread.is_alive()
    assert not sink.submit(event())
  finally:
    release.set()
    sink.thread.join(2)
  assert not sink.thread.is_alive()
  assert all(ident != threading.get_ident() for ident in writer_threads)


@pytest.mark.parametrize('error', [OSError(errno.ENOSPC, 'PRIVATE'), PermissionError('PRIVATE'), ValueError('PRIVATE')])
def test_disk_failure_backoff_and_recovery_never_log_raw_exception(tmp_path, monkeypatch, error):
  attempts = []
  original = journal.JournalFiles.write
  monkeypatch.setattr(journal, 'RETRY_SECONDS', .1)

  def fail_once(files, data):
    attempts.append(time.perf_counter())
    if len(attempts) == 1:
      raise error
    return original(files, data)

  monkeypatch.setattr(journal.JournalFiles, 'write', fail_once)
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  try:
    assert sink.submit(event())
    wait_for(lambda: sink.storage_errors == 1)
    for _ in range(100):
      assert not sink.submit(event())
    assert len(attempts) == 1
    wait_for(lambda: time.monotonic() >= sink.retry_at)
    assert sink.submit(event(2))
    wait_for(lambda: sink.written == 1)
  finally:
    sink.close()
  assert attempts[1] - attempts[0] >= .1
  assert sink.dropped == 101
  assert 'PRIVATE' not in json.dumps(records(sink.files.directory))


def test_short_write_is_repaired_on_next_open(tmp_path, monkeypatch):
  files = journal.JournalFiles(tmp_path.resolve() / 'logs')
  files.write(b'{"first":1}\n')
  original = journal.os.write
  with monkeypatch.context() as patch:
    patch.setattr(journal.os, 'write', lambda fd, data: original(fd, data[:5]))
    with pytest.raises(OSError):
      files.write(b'{"partial":2}\n')
  files.close()
  files.write(b'{"last":3}\n')
  files.close()
  assert records(files.directory) == [{'first': 1}, {'last': 3}]


def test_rate_limit_and_thread_start_failure_are_contained(tmp_path, monkeypatch):
  clock = NS(now=100.)
  monkeypatch.setattr(journal, 'time', NS(monotonic=lambda: clock.now))
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  try:
    assert sum(sink.submit(event(n)) for n in range(1000)) == journal.BURST
    clock.now += 1
    assert sum(sink.submit(event(n)) for n in range(1000)) == journal.RATE
    clock.now -= 1000  # A bad clock cannot refill the bucket.
    assert not sink.submit(event())
  finally:
    sink.close()
  def failed_start(_thread):
    raise RuntimeError('PRIVATE')
  monkeypatch.setattr(threading.Thread, 'start', failed_start)
  sink = journal.LocalJournal(tmp_path.resolve() / 'disabled')
  assert sink.disabled and not sink.submit(event())
  sink.close()
  assert not sink.files.directory.exists()


@pytest.mark.parametrize('total', [1024 * 1024 * 1024, 10 * 1024 * 1024 * 1024])
@pytest.mark.parametrize('headroom,allowed', [(0, False), (3, True)])
def test_low_space_reserves_five_percent_or_floor_before_each_write(tmp_path, monkeypatch, total, headroom, allowed):
  reserve = max(journal.FREE_FLOOR, total // 20)
  monkeypatch.setattr(journal.os, 'fstatvfs', lambda _fd: NS(f_bavail=reserve + headroom, f_blocks=total, f_frsize=1))
  files = journal.JournalFiles(tmp_path.resolve() / 'logs')
  try:
    if allowed:
      files.write(b'{}\n')
      assert files.size == 3
    else:
      with pytest.raises(OSError) as caught:
        files.write(b'{}\n')
      assert caught.value.errno == errno.ENOSPC
      assert files.size == 0
  finally:
    files.close()


def test_low_space_guard_runs_only_on_writer_and_retries_are_bounded(tmp_path, monkeypatch):
  checked = []
  def low_space(_fd):
    checked.append(threading.get_ident())
    return NS(f_bavail=0, f_blocks=10**9, f_frsize=1)
  monkeypatch.setattr(journal.os, 'fstatvfs', low_space)
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  sink.submit(event())
  wait_for(lambda: sink.storage_errors == 1)
  for _ in range(100):
    assert not sink.submit(event())
  sink.close()
  assert len(checked) == 1 and checked[0] != threading.get_ident()
  assert not sink.thread.is_alive()
  assert records(sink.files.directory) == []


@pytest.mark.parametrize('location,error', [('boot', OSError('PRIVATE')), ('boot', SystemExit('PRIVATE')),
                                          ('write', KeyboardInterrupt('PRIVATE')), ('close', SystemExit('PRIVATE'))])
def test_worker_top_level_exceptions_disable_without_escaping_thread(tmp_path, monkeypatch, location, error):
  escaped = []
  monkeypatch.setattr(threading, 'excepthook', escaped.append)
  def fail(*_args):
    raise error
  if location == 'boot':
    monkeypatch.setattr(journal, 'boot_id', fail)
  else:
    monkeypatch.setattr(journal.JournalFiles, 'write', fail if location == 'write' else lambda *_: None)
    if location == 'close':
      monkeypatch.setattr(journal.JournalFiles, 'close', fail)
  sink = journal.LocalJournal(tmp_path.resolve() / 'logs')
  sink.submit(event())
  sink.close()
  assert not sink.thread.is_alive()
  assert sink.disabled
  assert not sink.submit(event())
  assert not escaped


def test_rotation_failure_recovers_within_same_fixed_file_set(tmp_path, monkeypatch):
  files = journal.JournalFiles(tmp_path.resolve() / 'logs', max_bytes=16)
  files.write(b'{"first":1}\n')
  with monkeypatch.context() as patch:
    def fail(*_args, **_kwargs):
      raise OSError(errno.EIO, 'PRIVATE')
    patch.setattr(journal.os, 'replace', fail)
    with pytest.raises(OSError):
      files.write(b'{"second":2}\n')
  files.close()
  files.write(b'{"third":3}\n')
  files.close()
  assert sorted(records(files.directory), key=lambda row: next(iter(row))) == [{'first': 1}, {'third': 3}]
  assert sum(path.stat().st_size for path in files.directory.iterdir()) <= 16 * journal.FILE_COUNT
