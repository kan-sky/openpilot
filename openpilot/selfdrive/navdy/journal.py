"""Best-effort local diagnostics, isolated from the display/transport thread.

Four 4 MiB files survive bridge restarts. At most 256 x 4096-byte records are
queued, with 8 records/s and a 16-record burst. Storage retries are 30 s apart.
Writes stop below 5% free space or 64 MiB available, whichever is greater.
No fsync on the link thread (or per-record fsync); sudden power loss may lose
recent records. A stuck filesystem can strand one daemon writer, never the link.
"""
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import stat
import threading
import time
import uuid

from openpilot.selfdrive.navdy.diagnostics import COUNTER_MAX, ERROR_TYPES, PHASES, PUBLIC_ERRORS

DIRECTORY = Path('/data/navdy/diagnostics')
FILE_BYTES = 4 * 1024 * 1024
FILE_COUNT = 4
RECORD_BYTES = 4096
QUEUE_SIZE = 256
RETRY_SECONDS = 30.
STOP_SECONDS = .25
RATE = 8.
BURST = 16.
FREE_FLOOR = 64 * 1024 * 1024

TRANSPORT = frozenset(('usb', 'bluetooth', 'jetson'))
POWER = frozenset(('onroad', 'offroad', 'unknown'))
AMBIENT = frozenset(('disconnected', 'reverse', 'onroad', 'onroad_door', 'offroad', 'offroad_door'))
REASONS = PUBLIC_ERRORS | frozenset(('Exception', 'subprocess_timeout', 'subprocess_failed')) | frozenset(
  cls.__name__ for cls in ERROR_TYPES) | frozenset(os.strerror(n) for n in errno.errorcode)
METRICS = dict.fromkeys(('protocol', 'sequence', 'rttMs', 'receiverAgeMs', 'displayStatus',
                       'inFlight', 'skippedSnapshots', 'ackAgeMs', 'ackPollDelayMs', 'txPendingBytes',
                       'txAgeMs', 'lastTxMs', 'partialWriteCount', 'blockedWriteCount'), 'number')
RECEIVER = {'wireless': {'session': 'number', 'owner': TRANSPORT | {'none', 'unknown'}, 'clears': 'number', 'usbUpdates': 'number',
                         'receivedSeq': 'number', 'appliedSeq': 'number'},
            'power': {'state': POWER, 'fresh': 'bool', 'generation': 'number', 'invalidations': 'number'}}
SOURCE = {'missing': dict.fromkeys(('deviceState', 'pandaStates', 'carState', 'selfdriveState'),
                                  frozenset(('not_seen', 'invalid', 'invalid_timestamp', 'stale', 'can_invalid', 'can_timeout'))),
          'power': POWER, 'ambient': AMBIENT}
CONNECT_TIMINGS = dict.fromkeys(('bluetoothctl', 'sdp', 'socket_create', 'security_request', 'socket', 'security', 'hello', 'pipeline'),
                              {'ok': 'bool', 'elapsedMs': 'number'})
SCHEMA = {
  'runId': 'uuid', 'eventSeq': 'number', 'at': 'number', 'wallTime': 'number',
  'kind': frozenset(('bridge_started', 'bridge_stopped', 'policy_changed', 'usb_handover', 'connect_attempt',
                     'connect_failed', 'transport_open', 'transport_closed', 'failure', 'retry_scheduled',
                     'source_baseline', 'source_changed', 'receiver_baseline', 'receiver_changed', 'periodic_status', 'usb_recovery')),
  'phase': frozenset(PHASES), 'transport': TRANSPORT, 'connected': 'bool', 'bluetoothOnly': 'bool',
  'connectedAt': 'number', 'receivedAt': 'number', 'sourceAt': 'number', 'receiverAt': 'number',
  'phaseMs': 'number', 'roundTripMs': 'number', 'retryInSeconds': 'number', 'fastRetry': 'bool',
  'errorType': frozenset(cls.__name__ for cls in ERROR_TYPES) | {'Exception'}, 'reason': REASONS,
  'state': SOURCE, 'sourceState': SOURCE, 'receiver': RECEIVER, 'previous': RECEIVER, 'current': RECEIVER,
  'wireless': METRICS, 'batchLatestWireless': METRICS,
  'connectTimings': CONNECT_TIMINGS,
  'usbRecovery': {'action': frozenset(('absent_or_ambiguous', 'not_offline', 'no_adb_owner', 'unverified', 'not_parked',
                                      'waiting', 'reconnect', 'budget', 'changed', 'reset', 'reset_failed')),
                  'failureAge': 'number', 'resetCount': 'number'},
  'timings': dict.fromkeys(PHASES, dict.fromkeys(('count', 'totalMs', 'maxMs'), 'number')),
  'exchanges': 'number', 'replies': 'number',
  'feedback': {kind: dict.fromkeys(('count', 'lastReceivedAt'), 'number')
               for kind in ('noObservation', 'observation', 'incomplete')},
  'journal': dict.fromkeys(('queued', 'written', 'dropped', 'storageErrors', 'disabled'), 'number'),
}


def sanitize(value, schema=SCHEMA):
  # Walk the fixed schema, never arbitrary payload keys/arrays or object reprs.
  if isinstance(schema, dict):
    if type(value) is not dict:
      return None
    return {key: clean for key, spec in schema.items() if (clean := sanitize(value.get(key), spec)) is not None}
  if isinstance(schema, (set, frozenset)):
    return value if type(value) is str and len(value) <= 128 and value in schema else None
  if schema == 'number':
    return value if type(value) in (int, float, bool) and -COUNTER_MAX <= value <= COUNTER_MAX and math.isfinite(value) else None
  if schema == 'bool':
    return value if type(value) is bool else None
  if schema == 'uuid' and type(value) is str and len(value) == 32 and all(c in '0123456789abcdef' for c in value):
    return value
  return None


def boot_id():
  try:
    fd = os.open('/proc/sys/kernel/random/boot_id', os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, encoding='ascii') as source:
      return uuid.UUID(source.read(64).strip()).hex
  except (OSError, ValueError):
    return 'unavailable'


class JournalFiles:
  """Worker-only bounded file ring, using directory FDs for every operation."""
  def __init__(self, directory, max_bytes=FILE_BYTES):
    self.directory = Path(directory)
    self.max_bytes = max_bytes
    self.directory_fd = self.lock_fd = self.fd = None
    self.size = 0

  @staticmethod
  def name(index):
    return 'events.jsonl' if index == 0 else f'events.{index}.jsonl'

  def _file(self, name, create=False):
    fd = os.open(name, os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK | (os.O_CREAT if create else 0),
                 0o600, dir_fd=self.directory_fd)
    try:
      info = os.fstat(fd)
      if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.geteuid():
        raise OSError('unsafe journal file')
      os.fchmod(fd, 0o600)
      return fd
    except BaseException:
      os.close(fd)
      raise

  def open(self):
    if not self.directory.is_absolute() or len(self.directory.parts) < 2 or '..' in self.directory.parts:
      raise ValueError('journal directory must be absolute')
    parent = os.open('/', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
      for part in self.directory.parts[1:]:
        try:
          os.mkdir(part, mode=0o700, dir_fd=parent)
        except FileExistsError:
          pass
        child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
        os.close(parent)
        parent = child
      if os.fstat(parent).st_uid != os.geteuid():
        raise OSError('unsafe journal directory')
      os.fchmod(parent, 0o700)
      self.directory_fd, parent = parent, None
      self.lock_fd = self._file('.lock', create=True)
      fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
      os.ftruncate(self.lock_fd, 0)
      for index in range(FILE_COUNT):
        try:
          fd = self._file(self.name(index))
        except FileNotFoundError:
          continue
        try:
          if os.fstat(fd).st_size > self.max_bytes:
            os.ftruncate(fd, 0)
        finally:
          os.close(fd)
      self._current()
    except BaseException:
      self.close()
      raise
    finally:
      if parent is not None:
        os.close(parent)

  def _current(self):
    self.fd = self._file(self.name(0), create=True)
    self.size = os.fstat(self.fd).st_size
    # A crash/short write can leave one incomplete JSON line. Do not join it to
    # the next run's record; inspect at most one bounded record, not the full log.
    if self.size:
      offset = max(0, self.size - RECORD_BYTES)
      tail = os.pread(self.fd, self.size - offset, offset)
      if not tail.endswith(b'\n'):
        self.size = offset + tail.rfind(b'\n') + 1
        os.ftruncate(self.fd, self.size)
    os.lseek(self.fd, self.size, os.SEEK_SET)

  def write(self, data):
    if len(data) > min(RECORD_BYTES, self.max_bytes):
      raise ValueError('journal record bound')
    if self.fd is None:
      self.open()
    space = os.fstatvfs(self.directory_fd)
    available = space.f_bavail * space.f_frsize
    reserve = max(FREE_FLOOR, math.ceil(space.f_blocks * space.f_frsize * .05))
    if available - len(data) < reserve:
      raise OSError(errno.ENOSPC, 'journal free space reserve')
    if self.size + len(data) > self.max_bytes:
      os.close(self.fd)
      self.fd = None
      for index in range(FILE_COUNT - 2, -1, -1):
        try:
          os.replace(self.name(index), self.name(index + 1), src_dir_fd=self.directory_fd, dst_dir_fd=self.directory_fd)
        except FileNotFoundError:
          pass
      self._current()
    written = os.write(self.fd, data)
    self.size += written
    if written != len(data):
      raise OSError('short journal write')

  def close(self):
    for name in ('fd', 'lock_fd', 'directory_fd'):
      fd = getattr(self, name)
      setattr(self, name, None)
      if fd is not None:
        try:
          os.close(fd)
        except OSError:
          pass


class LocalJournal:
  def __init__(self, directory=None):
    self.files = JournalFiles(DIRECTORY if directory is None else directory)
    self.queue = queue.Queue(maxsize=QUEUE_SIZE)
    self.stop = threading.Event()
    self.dropped = self.written = self.storage_errors = 0
    self.disabled = False
    self.retry_at = 0.
    self.tokens, self.token_at = BURST, time.monotonic()
    self.thread = threading.Thread(target=self._worker, name='navdy-journal', daemon=True)
    try:
      self.thread.start()
    except RuntimeError:
      self.disabled = True

  def _drop(self):
    self.dropped = min(COUNTER_MAX, self.dropped + 1)
    return False

  def submit(self, event):
    # Single producer: serialization only, no storage calls or blocking queue put.
    try:
      now = time.monotonic()
      if self.disabled or self.stop.is_set() or now < self.retry_at:
        return self._drop()
      self.tokens = min(BURST, self.tokens + max(0., now - self.token_at) * RATE)
      self.token_at = max(self.token_at, now)
      if self.tokens < 1:
        return self._drop()
      self.tokens -= 1
      record = sanitize(event)
      if not record or not all(key in record for key in ('kind', 'runId', 'at', 'wallTime', 'eventSeq')):
        return self._drop()
      data = json.dumps(record, allow_nan=False, separators=(',', ':')).encode('ascii')
      # Leave space for worker-added boot ID/schema/PID, including the newline.
      if len(data) > RECORD_BYTES - 128:
        return self._drop()
      self.queue.put_nowait(data)
      return True
    except Exception:
      return self._drop()

  def _worker(self):
    try:
      identity = {'bootId': boot_id(), 'schema': 1, 'pid': os.getpid()}
      while not self.stop.is_set() or not self.queue.empty():
        if self.retry_at > time.monotonic():
          if self.stop.wait(self.retry_at - time.monotonic()):
            break
        try:
          data = self.queue.get(timeout=.1)
        except queue.Empty:
          continue
        try:
          record = {**json.loads(data), **identity}
          self.files.write(json.dumps(record, allow_nan=False, separators=(',', ':')).encode('ascii') + b'\n')
          self.written = min(COUNTER_MAX, self.written + 1)
        except Exception:
          self.storage_errors = min(COUNTER_MAX, self.storage_errors + 1)
          self.retry_at = time.monotonic() + RETRY_SECONDS
          self._drop()
          self.files.close()
          for _ in range(QUEUE_SIZE):
            try:
              self.queue.get_nowait()
              self._drop()
            except queue.Empty:
              break
    except BaseException:
      # A failed logger stays disabled; never propagate thread exceptions, spawn
      # replacement workers, or retain exception objects/messages with secrets.
      self.disabled = True
    finally:
      try:
        self.files.close()
      except BaseException:
        self.disabled = True

  def stats(self):
    return {'queued': self.queue.qsize(), 'written': self.written, 'dropped': self.dropped,
            'storageErrors': self.storage_errors, 'disabled': self.disabled}

  def close(self):
    self.stop.set()
    if self.thread.ident is not None:
      self.thread.join(timeout=STOP_SECONDS)
