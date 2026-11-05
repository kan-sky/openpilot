import base64

from openpilot.selfdrive.carrot.server.features.phone_media import (
  PHONE_MEDIA_ART_BASE64_MAX_CHARS,
  _art_diagnostics,
  _normalize_phone_media,
  _preserve_phone_media_art,
  _store_art_diagnostics,
)


def media(**overrides):
  body = {
    "title": "테스트 곡",
    "artist": "테스트 가수",
    "package": "com.example.player",
  }
  body.update(overrides)
  return _normalize_phone_media(body)


def test_preserves_art_for_same_track_progress_update():
  previous = media(
    artBase64="/9j/valid",
    artMime="image/jpeg",
    artHash="art-hash",
    artSource="metadata.bitmap",
    positionMs=1000,
  )
  update = media(positionMs=3000)

  assert _preserve_phone_media_art(update, previous)
  assert update["artBase64"] == previous["artBase64"]
  assert update["artMime"] == "image/jpeg"
  assert update["artHash"] == "art-hash"
  assert update["artSource"] == "metadata.bitmap"


def test_does_not_preserve_art_when_track_changes():
  previous = media(artBase64="/9j/old", artMime="image/jpeg")
  update = media(title="다른 곡")

  assert not _preserve_phone_media_art(update, previous)
  assert update["artBase64"] == ""


def test_new_art_replaces_previous_art():
  previous = media(artBase64="/9j/old", artMime="image/jpeg")
  update = media(artBase64="/9j/new", artMime="image/jpeg")

  assert not _preserve_phone_media_art(update, previous)
  assert update["artBase64"] == "/9j/new"


def test_reports_valid_jpeg_art_from_magic_bytes():
  art = base64.b64encode(b"\xff\xd8\xff\xe0android-jpeg").decode("ascii")
  normalized = media(artBase64=art, artMime="")

  diagnostics = _store_art_diagnostics(normalized, len(art), False)

  assert diagnostics["artStatus"] == "ok"
  assert diagnostics["artFormat"] == "jpeg"
  assert diagnostics["artBytes"] == 16
  assert normalized["artStatus"] == "ok"


def test_reports_oversize_art_drop_in_saved_diagnostics():
  oversized = "A" * (PHONE_MEDIA_ART_BASE64_MAX_CHARS + 1)
  normalized = media(artBase64=oversized, artMime="image/jpeg")

  diagnostics = _store_art_diagnostics(normalized, len(oversized), False)

  assert normalized["artBase64"] == ""
  assert diagnostics["artStatus"] == "dropped-too-large"
  assert diagnostics["artInputChars"] == len(oversized)
  assert diagnostics["artChars"] == 0


def test_art_diagnostics_rejects_invalid_base64():
  diagnostics = _art_diagnostics({"artBase64": "%%%", "artMime": "image/jpeg"})

  assert diagnostics["artStatus"].startswith("invalid-base64:")
  assert diagnostics["artBytes"] == 0
