import json
import socket
import time
from types import SimpleNamespace

from openpilot.system.wayon_live_stream import (
  CLIENT_HEARTBEAT_MAGIC,
  CLIENT_CONTROL_CLIP,
  CLIENT_CONTROL_HEADER,
  CLIENT_CONTROL_MAGIC,
  ClipFrameStore,
  ClientHeartbeatMonitor,
  ClientControlState,
  FRAME_FLAG_KEY,
  FRAME_HEADER,
  FRAME_MAGIC,
  FRAME_TYPE_METADATA,
  FRAME_TYPE_DRIVER,
  FRAME_TYPE_WIDE,
  bounded_number,
  camera_busy,
  encoded_payload,
  is_offroad,
  json_frame,
  pack_frame,
  read_client_control,
  set_live_active,
  stream_metadata,
  wait_for_camera,
  wait_for_processes,
)


def unpack_frame(frame):
  header = FRAME_HEADER.unpack(frame[:FRAME_HEADER.size])
  magic, frame_type, flags, _, sequence, timestamp_us, payload_size = header
  return magic, frame_type, flags, sequence, timestamp_us, frame[FRAME_HEADER.size:FRAME_HEADER.size + payload_size]


def test_pack_frame_preserves_header_and_payload():
  frame = pack_frame(2, b"video", sequence=42, timestamp_us=123_456, key_frame=True)
  magic, frame_type, flags, sequence, timestamp_us, payload = unpack_frame(frame)

  assert magic == FRAME_MAGIC
  assert frame_type == 2
  assert flags & FRAME_FLAG_KEY
  assert sequence == 42
  assert timestamp_us == 123_456
  assert payload == b"video"


def test_json_frame_uses_compact_utf8_payload():
  frame = json_frame(FRAME_TYPE_METADATA, {"state": "live", "fps": 20})
  *_, payload = unpack_frame(frame)
  assert json.loads(payload) == {"state": "live", "fps": 20}


def test_encoded_payload_prepends_codec_header_on_key_frame():
  payload, key_frame = encoded_payload(SimpleNamespace(header=b"sps-pps", data=b"frame"))
  assert payload == b"sps-ppsframe"
  assert key_frame is True

  payload, key_frame = encoded_payload(SimpleNamespace(header=b"", data=b"delta"))
  assert payload == b"delta"
  assert key_frame is False


def test_bounded_number_clamps_and_falls_back():
  assert bounded_number("800", 10, 100, 1000) == 800
  assert bounded_number(10, 50, 20, 100) == 20
  assert bounded_number("bad", 50, 20, 100) == 50


def test_is_offroad_requires_matching_manager_params():
  class FakeParams:
    def __init__(self, values):
      self.values = values

    def get_bool(self, key):
      return self.values.get(key, False)

  assert is_offroad(FakeParams({"IsOffroad": True, "IsOnroad": False}))
  assert not is_offroad(FakeParams({"IsOffroad": False, "IsOnroad": False}))
  assert not is_offroad(FakeParams({"IsOffroad": True, "IsOnroad": True}))


def test_wait_for_camera_retries_snapshot_contention(monkeypatch):
  class FakeParams:
    def get_bool(self, key):
      return key == "IsOffroad"

  busy = iter((True, True, False, False))
  monkeypatch.setattr("openpilot.system.wayon_live_stream.camera_busy", lambda _params: next(busy))
  monkeypatch.setattr("openpilot.system.wayon_live_stream.time.sleep", lambda _seconds: None)

  assert wait_for_camera(FakeParams(), timeout_s=1.0, poll_s=0.01)


def test_camera_busy_checks_snapshot_param_before_process(monkeypatch):
  class FakeParams:
    def get_bool(self, key):
      return key == "IsTakingSnapshot"

  monkeypatch.setattr("openpilot.system.wayon_live_stream.process_running", lambda _name: False)
  assert camera_busy(FakeParams())


def test_process_running_reads_proc_comm(tmp_path):
  process_dir = tmp_path / "123"
  process_dir.mkdir()
  (process_dir / "comm").write_text("camerad\n", encoding="utf-8")

  from openpilot.system.wayon_live_stream import process_running
  assert process_running("camerad", tmp_path)
  assert not process_running("encoderd", tmp_path)


def test_wait_for_processes_waits_for_each_process(monkeypatch):
  calls = {"camerad": 0, "encoderd": 0}

  def fake_process_running(name):
    calls[name] += 1
    return calls[name] >= (2 if name == "camerad" else 3)

  monkeypatch.setattr("openpilot.system.wayon_live_stream.process_running", fake_process_running)
  monkeypatch.setattr("openpilot.system.wayon_live_stream.time.sleep", lambda _seconds: None)

  assert wait_for_processes(("camerad", "encoderd"), timeout_s=1.0)
  assert calls["camerad"] >= 2
  assert calls["encoderd"] >= 3


def test_set_live_active_creates_and_removes_marker(tmp_path):
  marker = tmp_path / "wayon_live.active"
  set_live_active(True, marker)
  assert json.loads(marker.read_text())["mode"] == "panorama"
  set_live_active(False, marker)
  assert not marker.exists()


def test_driver_camera_uses_correct_horizontal_orientation():
  metadata = stream_metadata(800_000, 300)
  assert metadata["panorama"]["driverMirror"] is False
  assert metadata["panorama"]["blendDeg"] == 24.0
  assert metadata["panorama"]["wideRadialDistortion"] == [-0.018, 0.006]


def test_panorama_uses_mici_equidistant_projection():
  panorama = stream_metadata(800_000, 300)["panorama"]

  assert panorama["projectionModel"] == "equidistant-sphere-v1"
  assert panorama["wideFocalScale"] == [0.31640625, 0.55953947]
  assert panorama["driverFocalScale"] == [0.31640625, 0.55953947]
  assert panorama["widePitchDeg"] == -6.5
  assert panorama["driverPitchDeg"] == -14.0
  assert panorama["widePositionM"] == [-0.2, 0.0, 0.0]
  assert panorama["driverPositionM"] == [0.0, -0.435, 0.03]


def test_client_control_accepts_fragmented_heartbeat():
  state = ClientControlState()

  state.feed(CLIENT_HEARTBEAT_MAGIC[:2])
  assert not state.heartbeat_seen
  assert bytes(state.buffer) == CLIENT_HEARTBEAT_MAGIC[:2]

  state.feed(CLIENT_HEARTBEAT_MAGIC[2:])
  assert state.heartbeat_seen
  assert not state.buffer


def test_frame_gate_waits_for_key_after_gap_and_discards_backlog():
  from openpilot.system.wayon_live_stream import EncodedFrameGate
  gate = EncodedFrameGate()
  def frame(number, key=False, captured=100):
    return SimpleNamespace(header=b"sps" if key else b"", idx=SimpleNamespace(encodeId=number, timestampEof=int(captured * 1e9)))
  assert not gate.accept(frame(0), 100.1)
  assert gate.accept(frame(1, True), 100.1)
  assert gate.accept(frame(2), 100.1)
  assert not gate.accept(frame(4), 100.1)
  assert not gate.accept(frame(5), 100.1)
  assert gate.accept(frame(6, True), 100.1)
  assert not gate.accept(frame(7, True), 101)
  assert not gate.accept(frame(8, captured=101), 101.1)
  assert gate.accept(frame(9, True, 101), 101.1)


def test_video_buffer_is_byte_bounded():
  store = ClipFrameStore()
  for i in range(20):
    store.append(FRAME_TYPE_WIDE, (100., b"x" * 1024 * 1024, True, i))
  assert store.bytes[FRAME_TYPE_WIDE] <= 12 * 1024 * 1024
  assert len(store.buffers[FRAME_TYPE_WIDE]) == 12
  store.clear(FRAME_TYPE_WIDE)
  assert store.bytes[FRAME_TYPE_WIDE] == 0


def test_capture_commands_are_bounded():
  state = ClientControlState()
  packet = CLIENT_CONTROL_HEADER.pack(CLIENT_CONTROL_MAGIC, CLIENT_CONTROL_CLIP, 1) + bytes([10])
  state.feed(packet * 100)
  assert len(state.pop_commands()) == 4


def test_archive_queue_is_bounded_across_viewer_sessions(monkeypatch):
  from concurrent.futures import Future
  import threading
  from openpilot.system import wayon_live_stream as live
  futures = []
  def submit(*args):
    future = Future()
    futures.append(future)
    return future
  monkeypatch.setattr(live, "ARCHIVE_EXECUTOR", SimpleNamespace(submit=submit))
  monkeypatch.setattr(live, "ARCHIVE_SLOTS", threading.BoundedSemaphore(3))
  for _ in range(3):
    assert live.submit_archive(lambda: None) is not None
  assert live.submit_archive(lambda: None) is None
  futures[0].set_result(None)
  assert live.submit_archive(lambda: None) is not None


def test_client_control_preserves_binary_command_payload():
  state = ClientControlState()
  payload = b"clip-contains-" + CLIENT_HEARTBEAT_MAGIC
  packet = CLIENT_CONTROL_HEADER.pack(CLIENT_CONTROL_MAGIC, CLIENT_CONTROL_CLIP, len(payload)) + payload

  state.feed(packet[:7])
  assert state.pop_commands() == []
  state.feed(packet[7:])
  assert state.pop_commands() == [(CLIENT_CONTROL_CLIP, payload)]
  assert not state.heartbeat_seen


def test_clip_frame_store_preserves_full_stream_and_prunes_old_frames():
  store = ClipFrameStore(max_buffer_s=12)
  for second in range(21):
    store.append(FRAME_TYPE_WIDE, (second, f"w{second}".encode(), second % 5 == 0, second * 1_000_000))
    store.append(FRAME_TYPE_DRIVER, (second, f"d{second}".encode(), second % 5 == 0, second * 1_000_000))

  wide, driver = store.recent_pair(duration_s=10, now_s=20)

  assert [item[1] for item in wide] == [f"w{second}".encode() for second in range(10, 21)]
  assert [item[1] for item in driver] == [f"d{second}".encode() for second in range(10, 21)]
  assert store.buffers[FRAME_TYPE_WIDE][0][0] == 8


def test_read_client_control_detects_heartbeat_and_disconnect():
  client, peer = socket.socketpair()
  state = ClientControlState()
  try:
    peer.sendall(CLIENT_HEARTBEAT_MAGIC)
    assert read_client_control(client, state, timeout_s=0.1)
    assert state.heartbeat_seen

    peer.close()
    assert not read_client_control(client, state, timeout_s=0.1)
  finally:
    client.close()


def test_heartbeat_monitor_closes_stale_client():
  client, peer = socket.socketpair()
  state = ClientControlState()
  monitor = ClientHeartbeatMonitor(client, state, timeout_s=0.05, poll_s=0.01)
  try:
    monitor.start()
    deadline = time.monotonic() + 0.5
    while monitor.client_alive and time.monotonic() < deadline:
      time.sleep(0.01)

    assert monitor.timed_out
    assert not monitor.client_alive
    assert peer.recv(1) == b""
  finally:
    monitor.stop()
    peer.close()
    client.close()
