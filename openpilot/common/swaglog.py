import logging
import os
import socket
import time
from pathlib import Path
from logging.handlers import BaseRotatingHandler

from openpilot.common.logging_extra import SwagLogger, SwagFormatter, SwagLogFileFormatter
from openpilot.common.hardware.hw import Paths


SWAGLOG_MAX_SIZE = 4*1024*1024

# Kans: 일반 로그는 최근 300개(최소 약 5시간)만 남긴다
SWAGLOG_BACKUP_COUNT = 300
# Kans: 시작할 때마다 파일 번호를 이 단위로 건너뛰어, 번호만으로 시작 직후 파일을 구분한다
BOOT_INDEX_STEP = 10000
# Kans: 시작 직후 파일(rlog 기록 전 오류 확인용)은 처음 5개씩, 최근 10번 시작분을 따로 남긴다
BOOT_KEEP_FILES = 5
BOOT_KEEP_COUNT = 10


def _log_index(path: str) -> int | None:
  idx = path.split(".")[-1]
  return int(idx) if idx.isdigit() else None

def get_file_handler():
  Path(Paths.swaglog_root()).mkdir(parents=True, exist_ok=True)
  base_filename = os.path.join(Paths.swaglog_root(), "swaglog")
  handler = SwaglogRotatingFileHandler(base_filename)
  return handler

class SwaglogRotatingFileHandler(BaseRotatingHandler):
  def __init__(self, base_filename, interval=60, max_bytes=1024*256, backup_count=SWAGLOG_BACKUP_COUNT, encoding=None):
    super().__init__(base_filename, mode="a", encoding=encoding, delay=True)
    self.base_filename = base_filename
    self.interval = interval # seconds
    self.max_bytes = max_bytes
    self.backup_count = backup_count
    self.log_files = self.get_existing_logfiles()
    log_indexes = [f.split(".")[-1] for f in self.log_files]
    self.last_file_idx = max([int(i) for i in log_indexes if i.isdigit()] or [-1])
    # 이번 시작의 첫 파일 번호를 BOOT_INDEX_STEP 배수로 맞춘다
    if self.last_file_idx >= 0:
      self.last_file_idx = (self.last_file_idx // BOOT_INDEX_STEP + 1) * BOOT_INDEX_STEP - 1
    self.last_rollover = 0.0
    self.doRollover()

  def _open(self):
    self.last_rollover = time.monotonic()
    self.last_file_idx += 1
    next_filename = f"{self.base_filename}.{self.last_file_idx:010}"
    stream = open(next_filename, self.mode, encoding=self.encoding)
    self.log_files.insert(0, next_filename)
    return stream

  def get_existing_logfiles(self):
    log_files = []
    base_dir = os.path.dirname(self.base_filename)
    for fn in os.listdir(base_dir):
      fp = os.path.join(base_dir, fn)
      if fp.startswith(self.base_filename) and os.path.isfile(fp):
        log_files.append(fp)
    # newest first, matching _open()'s insert(0, ...) so doRollover()'s pop() deletes the oldest
    return sorted(log_files, reverse=True)

  def shouldRollover(self, record):
    size_exceeded = self.max_bytes > 0 and self.stream.tell() >= self.max_bytes
    time_exceeded = self.interval > 0 and self.last_rollover + self.interval <= time.monotonic()
    return size_exceeded or time_exceeded

  def doRollover(self):
    if self.stream:
      self.stream.close()
    self.stream = self._open()

    if self.backup_count > 0:
      self._trim_log_files()

  def _trim_log_files(self):
    # log_files 는 최신 파일이 앞에 있다
    keep = []
    general_count = 0
    boot_groups: list[int] = []
    for fn in self.log_files:
      idx = _log_index(fn)
      if idx is not None and idx % BOOT_INDEX_STEP < BOOT_KEEP_FILES:
        # 시작 직후 파일: 최근 BOOT_KEEP_COUNT 번의 시작분만 남긴다
        group = idx // BOOT_INDEX_STEP
        if group not in boot_groups:
          boot_groups.append(group)
        if len(boot_groups) <= BOOT_KEEP_COUNT:
          keep.append(fn)
          continue
      elif general_count < self.backup_count:
        general_count += 1
        keep.append(fn)
        continue
      if os.path.exists(fn): # just being safe, should always exist
        os.remove(fn)
    self.log_files = keep

class UnixDomainSocketHandler(logging.Handler):
  def __init__(self, formatter):
    logging.Handler.__init__(self)
    self.setFormatter(formatter)
    self.pid = None
    self.sock = None

  def __del__(self):
    self.close()

  def close(self):
    if self.sock is not None:
      self.sock.close()
      self.sock = None

  def connect(self):
    self.close()
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SWAGLOG_MAX_SIZE)
    self.sock.setblocking(False)
    self.pid = os.getpid()

  def emit(self, record):
    if os.getpid() != self.pid:
      self.connect()

    msg = self.format(record).rstrip('\n')
    try:
      self.sock.sendto((chr(record.levelno)+msg).encode('utf8'), Paths.swaglog_ipc())
    except OSError:
      # logmessaged is busy, not running, or the message is too big. drop :/
      pass


class ForwardingHandler(logging.Handler):
  def __init__(self, target_logger):
    super().__init__()
    self.target_logger = target_logger

  def emit(self, record):
    self.target_logger.handle(record)


def add_file_handler(log):
  """
  Function to add the file log handler to swaglog.
  This can be used to store logs when logmessaged is not running.
  """
  handler = get_file_handler()
  handler.setFormatter(SwagLogFileFormatter(log))
  log.addHandler(handler)


cloudlog = log = SwagLogger()
log.setLevel(logging.DEBUG)


outhandler = logging.StreamHandler()

print_level = os.environ.get('LOGPRINT', 'warning')
if print_level == 'debug':
  outhandler.setLevel(logging.DEBUG)
elif print_level == 'info':
  outhandler.setLevel(logging.INFO)
elif print_level == 'warning':
  outhandler.setLevel(logging.WARNING)

ipchandler = UnixDomainSocketHandler(SwagFormatter(log))

log.addHandler(outhandler)
# logs are sent through IPC before writing to disk to prevent disk I/O blocking
log.addHandler(ipchandler)
