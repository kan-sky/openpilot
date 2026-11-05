import ast
import base64
import io
from datetime import datetime, timezone
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

from openpilot.system import wayon_camera as camera


def test_snapshot_size_keeps_native_wide_without_upscaling():
  assert camera.snapshot_width(0, 1344) == 1344
  assert camera.snapshot_width(0, 1928) == 1928
  assert camera.snapshot_width(0, 3840) == 1928
  assert camera.snapshot_width(1, 1344) == 960
  assert camera.snapshot_width(0, 640) == 640


@pytest.fixture
def fake_capture(monkeypatch):
  state = SimpleNamespace(now=100.0, released=0, lease_released=0, acquired=True, blocked=False, stop_at=1000)
  data = np.full(64 * 64 * 3 // 2, 100, dtype=np.uint8)

  class Guard:
    def __init__(self, params):
      pass

    def wait_ready(self):
      return not state.blocked

    def allowed(self, *args):
      return not state.blocked and state.now < state.stop_at

  class Client:
    def __init__(self, *args):
      self.timestamp_eof = int(state.now * 1e9)

    def connect(self, *args):
      return True

    def recv(self, **kwargs):
      state.now += 0.1
      self.timestamp_eof = int(state.now * 1e9)
      return SimpleNamespace(width=64, height=64, stride=64, uv_offset=4096, data=data)

  class Lease:
    def __init__(self, *args):
      pass

    def acquire(self):
      return state.acquired

    def release(self):
      state.lease_released += 1

  monkeypatch.setattr(camera, 'CameraGuard', Guard)
  monkeypatch.setattr(camera.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(camera.time, 'time', lambda: 1793360000 + state.now)
  monkeypatch.setattr(camera, 'request_camera', lambda *args: None)
  monkeypatch.setattr(camera, 'release_camera', lambda: setattr(state, 'released', state.released + 1))
  monkeypatch.setitem(sys.modules, 'msgq.visionipc', SimpleNamespace(
    VisionIpcClient=Client, VisionStreamType=SimpleNamespace(VISION_STREAM_WIDE_ROAD=0, VISION_STREAM_DRIVER=1)))
  monkeypatch.setitem(sys.modules, 'openpilot.system.camera_lease', SimpleNamespace(CameraLease=Lease))
  monkeypatch.setitem(sys.modules, 'openpilot.system.wayon_live_stream', SimpleNamespace(camera_busy=lambda p: False))
  return state


def test_burst_has_three_spaced_frames_and_releases(fake_capture):
  frames = []
  wide, driver = camera.capture_pair(None, wide_frames=frames)
  assert len(frames) == 3
  assert all(b[0] - a[0] >= 1.0 for a, b in zip(frames, frames[1:]))
  assert wide.shape == driver.shape == (64, 64, 3)
  assert fake_capture.released == fake_capture.lease_released == 1


def test_ignition_change_discards_partial_burst(fake_capture):
  fake_capture.stop_at = 105
  frames = []
  assert camera.capture_pair(None, wide_frames=frames) == (None, None)
  assert frames == []
  assert fake_capture.released == fake_capture.lease_released == 1


def test_default_remains_one_frame_and_guard_refuses(fake_capture):
  wide, driver = camera.capture_pair(None)
  assert wide is not None and driver is not None
  assert fake_capture.now < 106
  fake_capture.blocked = True
  assert camera.capture_pair(None, wide_frames=[]) == (None, None)


@pytest.mark.parametrize('multiframe,blocked_at,count', [(True, 99, 3), (False, 99, 1), (True, 4, 1)])
def test_production_upload_preserves_timestamps_and_jpeg92(monkeypatch, multiframe, blocked_at, count):
  tree = ast.parse(Path(camera.__file__).with_name('wayon_cloud_uploader.py').read_text())
  names = {'jpeg_bytes', 'jpeg_base64', 'upload_offroad_snapshot'}
  nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
  uploaded = []
  array = np.arange(1344 * 760 * 3, dtype=np.uint8).reshape(760, 1344, 3)

  def capture(params, timeout, wide_frames=None):
    assert (wide_frames is not None) == multiframe
    if wide_frames is not None:
      wide_frames.extend([(1793360000 + i, array) for i in range(3)])
    return array, np.zeros((543, 960, 3), dtype=np.uint8)

  state = SimpleNamespace(calls=0)
  def allowed():
    state.calls += 1
    return state.calls < blocked_at

  monkeypatch.setattr(camera, 'capture_pair', capture)
  monkeypatch.setattr(camera, 'CameraGuard', lambda p: SimpleNamespace(wait_ready=lambda: True, allowed=allowed))
  scope = {'io': io, 'base64': base64, 'Params': lambda: None, 'datetime': datetime, 'timezone': timezone,
           'OFFROAD_SNAPSHOT_TIMEOUT_S': 25, 'time': camera.time, 'enhance_wide_snapshot': lambda x: x,
           'post_json': lambda config, path, payload: uploaded.append(payload)}
  exec(compile(ast.Module(nodes, []), 'production_snapshot', 'exec'), scope)
  ok = scope['upload_offroad_snapshot']({'parking_multiframe_enabled': True}, 'test', multiframe=multiframe)
  assert ok == (blocked_at == 99)
  assert len(uploaded) == count
  for payload in uploaded:
    datetime.strptime(payload['capturedAt'], '%Y-%m-%dT%H:%M:%SZ')
  if count == 3:
    assert uploaded[0]['capturedAt'] != uploaded[1]['capturedAt']
    assert uploaded[1]['driverJpegBase64'] is None
  assert uploaded[0]['driverJpegBase64']
  raw = base64.b64decode(uploaded[0]['wideJpegBase64'])
  with Image.open(io.BytesIO(raw)) as image:
    assert image.size == (1344, 760)
    assert image.layer[0][1:3] == (1, 1)
    assert image.quantization[0][0] == 3  # libjpeg luminance table at Q92.


@pytest.mark.parametrize('abort_at', [99, 7])
def test_approach_group_keeps_real_capture_times_and_offroad_guard(monkeypatch, abort_at):
  from openpilot.system import wayon_parking as parking
  tree = ast.parse(Path(camera.__file__).with_name('wayon_cloud_uploader.py').read_text())
  nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef)
           and node.name in {'jpeg_bytes', 'jpeg_base64', 'upload_offroad_snapshot'}]
  uploaded = []
  now = camera.time.time()
  mono = camera.time.monotonic()
  array = np.zeros((760, 1344, 3), dtype=np.uint8)
  stop_times = [now - 3 + i for i in range(3)]
  def capture(params, timeout, wide_frames=None):
    wide_frames.extend((t, array) for t in stop_times)
    return array, array
  state = SimpleNamespace(calls=0)
  def allowed():
    state.calls += 1
    return state.calls < abort_at
  monkeypatch.setattr(camera, 'capture_pair', capture)
  monkeypatch.setattr(camera, 'CameraGuard', lambda p: SimpleNamespace(wait_ready=lambda: True, allowed=allowed))
  scope = {'io': io, 'base64': base64, 'Params': lambda: None, 'datetime': datetime, 'timezone': timezone,
           'OFFROAD_SNAPSHOT_TIMEOUT_S': 25, 'time': camera.time, 'enhance_wide_snapshot': lambda x: x,
           'post_json': lambda config, path, payload: uploaded.append(payload)}
  exec(compile(ast.Module(nodes, []), 'production_snapshot', 'exec'), scope)
  approach = [parking.ApproachFrame(mono - age, now - age, b'jpeg') for age in (40, 38, 24, 22, 8, 6)]
  ok = scope['upload_offroad_snapshot']({'parking_multiframe_enabled': True, 'parking_approach_enabled': True},
                                       'test', approach_frames=approach)
  assert ok == (abort_at == 99)
  assert len(uploaded) == (9 if ok else 3)
  assert len({p['parking']['eventId'] for p in uploaded}) == 1
  assert all(p['parking']['count'] == 9 for p in uploaded)
  assert [p['parking']['index'] for p in uploaded] == list(range(len(uploaded)))
  assert all(p['parking']['phase'] == 'approach' and p['driverJpegBase64'] is None for p in uploaded[:6])
  for payload, source in zip(uploaded[:6], approach):
    assert payload['capturedAt'] == datetime.fromtimestamp(source.captured_at, timezone.utc).isoformat(timespec='milliseconds')
    assert base64.b64decode(payload['wideJpegBase64']) == source.jpeg
  if ok:
    assert uploaded[6]['driverJpegBase64']
    assert all(p['parking']['phase'] == 'parked' for p in uploaded[6:])
    assert uploaded[-1]['parking']['parkedAt'] == uploaded[-1]['capturedAt']


def test_photo_timestamp_uses_sensor_eof_not_encoding_finish(monkeypatch):
  monkeypatch.setattr(camera.time, 'monotonic', lambda: 100.25)
  monkeypatch.setattr(camera.time, 'time', lambda: 1100.25)
  client = SimpleNamespace(timestamp_eof=int(99.875 * 1e9))
  assert camera.snapshot_capture_time(client) == pytest.approx(1099.875)
  monkeypatch.setattr(camera.time, 'monotonic', lambda: 101.5)
  monkeypatch.setattr(camera.time, 'time', lambda: 1101.5)
  assert camera.snapshot_capture_time(client) == pytest.approx(1099.875)
