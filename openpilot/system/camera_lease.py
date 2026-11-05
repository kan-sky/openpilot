import fcntl
import json
import os
import time
from pathlib import Path


CAMERA_LEASE_PATH = Path(os.getenv("WAYON_CAMERA_LEASE", "/tmp/wayon_camera.lease"))


class CameraLease:
  def __init__(self, owner: str, ttl_s: float, path: Path = CAMERA_LEASE_PATH):
    self.owner = owner
    self.ttl_s = max(1.0, float(ttl_s))
    self.path = path
    self.fd: int | None = None

  @property
  def acquired(self) -> bool:
    return self.fd is not None

  def acquire(self) -> bool:
    if self.fd is not None:
      return True

    self.path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
    os.set_inheritable(fd, False)
    try:
      fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      os.close(fd)
      return False

    self.fd = fd
    try:
      self.renew()
    except Exception:
      self.fd = None
      try:
        fcntl.flock(fd, fcntl.LOCK_UN)
      finally:
        os.close(fd)
      raise
    return True

  def renew(self) -> None:
    if self.fd is None:
      return

    now = time.monotonic()
    payload = json.dumps({
      "owner": self.owner,
      "pid": os.getpid(),
      "acquiredAt": now,
      "expiresAt": now + self.ttl_s,
    }, separators=(",", ":")).encode("utf-8")
    os.ftruncate(self.fd, 0)
    os.lseek(self.fd, 0, os.SEEK_SET)
    os.write(self.fd, payload)
    os.fsync(self.fd)

  def release(self) -> None:
    if self.fd is None:
      return

    fd, self.fd = self.fd, None
    try:
      os.ftruncate(fd, 0)
      fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
      os.close(fd)

  def __enter__(self):
    self.acquire()
    return self

  def __exit__(self, _exc_type, _exc, _traceback):
    self.release()
