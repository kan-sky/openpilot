import io
import json
import zipfile

import requests

from openpilot.system.wayon_live_archive import build_dual_h264_archive, select_recent_frames, upload_live_capture


def frame(received_at, payload, key_frame=False, timestamp_us=0):
  return received_at, payload, key_frame, timestamp_us


def test_select_recent_frames_starts_at_key_frame_before_cutoff():
  frames = [
    frame(60, b"old-key", True),
    frame(65, b"old-delta"),
    frame(70, b"start-key", True),
    frame(75, b"delta"),
    frame(80, b"end"),
  ]

  selected = select_recent_frames(frames, duration_s=8, now_s=80)
  assert [item[1] for item in selected] == [b"start-key", b"delta", b"end"]


def test_dual_h264_archive_contains_manifest_and_streams():
  wide = [frame(1, b"wide-header", True), frame(2, b"wide-frame")]
  driver = [frame(1, b"driver-header", True), frame(2, b"driver-frame")]
  payload = build_dual_h264_archive(
    wide,
    driver,
    "2026-07-21T01:00:00.000Z",
    10,
    {"codec": "avc1.640020", "fps": 20, "width": 1344, "height": 760, "panorama": {"blendDeg": 24}},
  )

  with zipfile.ZipFile(io.BytesIO(payload)) as archive:
    manifest = json.loads(archive.read("manifest.json"))
    assert manifest["schema"] == "wayon-live-dual-h264-v1"
    assert manifest["requestedDurationSeconds"] == 10
    assert manifest["cameras"]["wide"]["frameCount"] == 2
    assert archive.read("wide.h264") == b"wide-headerwide-frame"
    assert archive.read("driver.h264") == b"driver-headerdriver-frame"


def test_live_capture_upload_retries_transient_http_error(monkeypatch):
  class Response:
    def __init__(self, status_code):
      self.status_code = status_code
      self.content = b'{"ok":true,"id":"capture-id"}'
      self.text = self.content.decode()

    def raise_for_status(self):
      if self.status_code >= 400:
        raise requests.HTTPError(f"HTTP {self.status_code}", response=self)

    def json(self):
      return json.loads(self.content)

  responses = iter((Response(503), Response(200)))
  sleeps = []
  monkeypatch.setattr("openpilot.system.wayon_live_archive.requests.post", lambda *args, **kwargs: next(responses))
  monkeypatch.setattr("openpilot.system.wayon_live_archive.time.sleep", sleeps.append)

  result = upload_live_capture(
    {"endpoint": "https://wayon.test", "token": "token"},
    "device",
    "clip",
    "2026-07-21T01:00:00.000Z",
    "application/zip",
    "dual_h264_360",
    b"payload",
    30,
  )

  assert result["id"] == "capture-id"
  assert sleeps == [1]


def test_large_clip_upload_uses_idempotent_chunks(monkeypatch):
  calls = []

  def fake_post(endpoint, payload, headers):
    calls.append((endpoint, payload, headers))
    return {"ok": True, "id": "a" * 32}

  monkeypatch.setattr("openpilot.system.wayon_live_archive.LIVE_CAPTURE_CHUNK_BYTES", 4)
  monkeypatch.setattr("openpilot.system.wayon_live_archive.uuid.uuid4", lambda: type("Uuid", (), {"hex": "a" * 32})())
  monkeypatch.setattr("openpilot.system.wayon_live_archive.post_capture_request", fake_post)

  result = upload_live_capture(
    {"endpoint": "https://wayon.test", "token": "token"},
    "device",
    "clip",
    "2026-07-21T01:00:00.000Z",
    "application/zip",
    "dual_h264_360",
    b"0123456789",
    30,
  )

  assert result["id"] == "a" * 32
  assert [len(call[1]) for call in calls[:-1]] == [4, 4, 2]
  assert [call[2]["X-Wayon-Part-Index"] for call in calls[:-1]] == ["0", "1", "2"]
  assert calls[-1][0].endswith("/api/live-capture-commit")
  assert json.loads(calls[-1][1])["sizeBytes"] == 10
