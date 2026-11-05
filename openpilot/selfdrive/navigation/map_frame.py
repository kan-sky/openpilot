"""Bounded tmpfs snapshot for optional model input, without a socket handshake."""
import os
from pathlib import Path
import stat
import struct
from types import SimpleNamespace

FRAME_PATH = Path('/dev/shm/carrot_nav_model.frame')
HEADER = struct.Struct('<8sQQ')
MAGIC = b'NAVMAP01'
PIXELS = 256 * 256
FRAME_BYTES = HEADER.size + PIXELS


def write_frame(image, route_id, rendered_ns, path=FRAME_PATH):
  pixels = image.tobytes()
  if image.shape != (256, 256) or str(image.dtype) != 'uint8' or len(pixels) != PIXELS:
    raise ValueError('invalid navigation image')
  temporary = path.with_suffix('.tmp')
  # Replace only complete frames. Readers holding the old inode see the old
  # complete frame, which still has its original capture time and route ID.
  with temporary.open('wb') as output:
    output.write(HEADER.pack(MAGIC, route_id, rendered_ns))
    output.write(pixels)
  os.replace(temporary, path)


class SnapshotClient:
  """MapInput-compatible reader; no listener, retry, lock or native IPC call."""
  def __init__(self, path=FRAME_PATH):
    self.path = path
    self.timestamp_sof = self.timestamp_eof = 0

  def is_connected(self):
    return True

  def recv(self, timeout=0):
    if timeout != 0:
      raise ValueError('map snapshots must never wait')
    try:
      fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    except FileNotFoundError:
      return None
    try:
      info = os.fstat(fd)
      if not stat.S_ISREG(info.st_mode) or info.st_size != FRAME_BYTES:
        return None
      payload = os.read(fd, FRAME_BYTES + 1)
    finally:
      os.close(fd)
    if len(payload) != FRAME_BYTES:
      return None
    magic, route, rendered = HEADER.unpack_from(payload)
    if magic != MAGIC:
      return None
    self.timestamp_sof, self.timestamp_eof = route, rendered
    return SimpleNamespace(width=256, height=256, data=payload[HEADER.size:])
