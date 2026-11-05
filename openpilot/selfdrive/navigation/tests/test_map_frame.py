import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest

from openpilot.selfdrive.navigation import map_frame
from openpilot.selfdrive.navigation.map_client import _make_client
from openpilot.selfdrive.navigation.map_frame import FRAME_BYTES, HEADER, MAGIC, SnapshotClient, write_frame


def test_pixel_exact_snapshot_and_atomic_replacement(tmp_path, monkeypatch):
  path = tmp_path / 'map.frame'
  client = SnapshotClient(path)
  assert client.recv() is None
  pixels = np.arange(65536, dtype=np.uint8).reshape(256, 256)
  write_frame(pixels, 123, 1000000000, path)
  before = client.recv()
  assert before.data == pixels.tobytes()
  assert (client.timestamp_sof, client.timestamp_eof) == (123, 1000000000)
  replace = os.replace

  def inspect_before_publish(src, dst):
    assert client.recv().data == pixels.tobytes()
    replace(src, dst)

  monkeypatch.setattr(map_frame.os, 'replace', inspect_before_publish)
  write_frame(np.zeros_like(pixels), 124, 2000000000, path)
  assert not any(client.recv().data)
  assert (client.timestamp_sof, client.timestamp_eof) == (124, 2000000000)
  assert before.data == pixels.tobytes()
  assert len(list(tmp_path.iterdir())) == 1


@pytest.mark.parametrize('payload', [b'', b'bad', MAGIC + bytes(FRAME_BYTES),
                                    HEADER.pack(b'WRONG001', 1, 1) + bytes(65536)])
def test_malformed_frame_rejected(tmp_path, payload):
  path = tmp_path / 'map.frame'
  path.write_bytes(payload)
  assert SnapshotClient(path).recv() is None


def test_fifo_and_symlink_cannot_block_inference(tmp_path):
  fifo = tmp_path / 'fifo'
  os.mkfifo(fifo)
  # Separate process and deadline catch a regression to a blocking open/read.
  program = f'from openpilot.selfdrive.navigation.map_frame import SnapshotClient; assert SnapshotClient({str(fifo)!r}).recv() is None'
  subprocess.run([sys.executable, '-c', program], check=True, timeout=3)
  path = tmp_path / 'map.frame'
  path.symlink_to(fifo)
  with pytest.raises(OSError):
    SnapshotClient(path).recv()


def test_model_reader_never_opens_visionipc_socket():
  assert type(_make_client()) is SnapshotClient
  source = Path(map_frame.__file__).read_text()
  assert 'VisionIpcClient' not in source and 'socket.' not in source


def test_bounded_read_and_nonblocking_open(tmp_path, monkeypatch):
  path = tmp_path / 'map.frame'
  write_frame(np.zeros((256, 256), np.uint8), 0, time.monotonic_ns(), path)
  original_open, original_read = os.open, os.read
  calls = []

  def opened(name, flags):
    assert flags & os.O_NONBLOCK and flags & os.O_NOFOLLOW
    return original_open(name, flags)

  def read(fd, size):
    calls.append(size)
    return original_read(fd, size)

  monkeypatch.setattr(map_frame.os, 'open', opened)
  monkeypatch.setattr(map_frame.os, 'read', read)
  assert SnapshotClient(path).recv() is not None
  assert calls == [FRAME_BYTES + 1]


def test_writer_rejects_wrong_image_contract(tmp_path):
  for image in (np.zeros((256, 256), np.float32), np.zeros((255, 256), np.uint8)):
    with pytest.raises(ValueError):
      write_frame(image, 1, 1, tmp_path / 'frame')
