from __future__ import annotations

import base64
import binascii
import json
import os
import time
from typing import Any

from aiohttp import web

from ..config import (
  PHONE_MEDIA_ART_BASE64_MAX_CHARS,
  PHONE_MEDIA_FALLBACK_PATH,
  PHONE_MEDIA_PATH,
  PHONE_MEDIA_TEXT_MAX_CHARS,
)


def _safe_int(value: Any) -> int | None:
  if isinstance(value, bool):
    return None
  try:
    parsed = int(value)
  except (TypeError, ValueError):
    return None
  return parsed if parsed >= 0 else None


def _safe_bool(value: Any) -> bool:
  if isinstance(value, bool):
    return value
  if isinstance(value, (int, float)):
    return value != 0
  if isinstance(value, str):
    return value.strip().lower() in ("1", "true", "yes", "on")
  return False


def _safe_text(body: dict[str, Any], key: str, limit: int = PHONE_MEDIA_TEXT_MAX_CHARS) -> str:
  value = body.get(key)
  if value is None:
    return ""
  return str(value).strip()[:limit]


def _normalize_phone_media(body: dict[str, Any]) -> dict[str, Any]:
  art_base64 = _safe_text(body, "artBase64", PHONE_MEDIA_ART_BASE64_MAX_CHARS + 1)
  if len(art_base64) > PHONE_MEDIA_ART_BASE64_MAX_CHARS:
    art_base64 = ""
  now_ms = int(time.time() * 1000.0)  # noqa: TID251 - protocol uses Unix epoch milliseconds
  return {
    "title": _safe_text(body, "title"),
    "artist": _safe_text(body, "artist"),
    "package": _safe_text(body, "package", 120),
    "isPlaying": _safe_bool(body.get("isPlaying")),
    "durationMs": _safe_int(body.get("durationMs")),
    "positionMs": _safe_int(body.get("positionMs")),
    "artBase64": art_base64,
    "artMime": _safe_text(body, "artMime", 80),
    "artHash": _safe_text(body, "artHash", 160),
    "artSource": _safe_text(body, "artSource", 160),
    "updatedAtMs": _safe_int(body.get("updatedAtMs")) or now_ms,
    "receivedAtMs": now_ms,
  }


def _art_input_chars(body: dict[str, Any]) -> int:
  value = body.get("artBase64")
  return len(str(value).strip()) if value is not None else 0


def _decode_art_base64(value: str) -> tuple[bytes, str]:
  if not value:
    return b"", "missing"
  payload = value.split(",", 1)[1] if "," in value[:96] else value
  compact = "".join(payload.split())
  if not compact:
    return b"", "missing"
  compact += "=" * (-len(compact) % 4)
  try:
    decoded = base64.b64decode(compact, altchars=b"-_", validate=True)
  except (binascii.Error, ValueError, UnicodeEncodeError) as exc:
    return b"", f"invalid-base64:{type(exc).__name__}"
  return decoded, "" if decoded else "decoded-empty"


def _image_format(data: bytes) -> str:
  if data.startswith(b"\xff\xd8\xff"):
    return "jpeg"
  if data.startswith(b"\x89PNG\r\n\x1a\n"):
    return "png"
  if data.startswith((b"GIF87a", b"GIF89a")):
    return "gif"
  if data.startswith(b"BM"):
    return "bmp"
  if len(data) >= 12 and data.startswith(b"RIFF") and data[8:12] == b"WEBP":
    return "webp"
  return "unknown"


def _art_diagnostics(media: dict[str, Any], input_chars: int | None = None) -> dict[str, Any]:
  art_base64 = str(media.get("artBase64") or "")
  decoded, decode_error = _decode_art_base64(art_base64)
  image_format = _image_format(decoded) if decoded else "none"
  if decode_error:
    status = decode_error
  elif image_format == "unknown":
    status = "unknown-image-format"
  else:
    status = "ok"
  return {
    "artInputChars": len(art_base64) if input_chars is None else input_chars,
    "artChars": len(art_base64),
    "artBytes": len(decoded),
    "artFormat": image_format,
    "artStatus": status,
    "artMime": str(media.get("artMime") or ""),
    "artHash": str(media.get("artHash") or ""),
    "artSource": str(media.get("artSource") or ""),
  }


def _store_art_diagnostics(media: dict[str, Any], input_chars: int, art_preserved: bool) -> dict[str, Any]:
  diagnostics = _art_diagnostics(media, input_chars)
  if input_chars > PHONE_MEDIA_ART_BASE64_MAX_CHARS:
    diagnostics["artStatus"] = "dropped-too-large"
  elif art_preserved:
    diagnostics["artStatus"] = "preserved"
  for key in ("artInputChars", "artBytes", "artFormat", "artStatus"):
    media[key] = diagnostics[key]
  return diagnostics


def _write_json_atomic(path: str, data: dict[str, Any]) -> None:
  directory = os.path.dirname(path)
  if directory:
    os.makedirs(directory, exist_ok=True)
  tmp_path = f"{path}.tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, separators=(",", ":"))
  os.replace(tmp_path, path)


def _write_phone_media(data: dict[str, Any]) -> str:
  primary_error: Exception | None = None
  try:
    _write_json_atomic(PHONE_MEDIA_PATH, data)
    return PHONE_MEDIA_PATH
  except Exception as exc:
    primary_error = exc

  try:
    _write_json_atomic(PHONE_MEDIA_FALLBACK_PATH, data)
    return PHONE_MEDIA_FALLBACK_PATH
  except Exception as fallback_error:
    raise OSError(
      f"phone media write failed: primary={primary_error}; fallback={fallback_error}"
    ) from fallback_error


def _latest_phone_media_path() -> str:
  candidates: list[tuple[float, str]] = []
  for path in (PHONE_MEDIA_PATH, PHONE_MEDIA_FALLBACK_PATH):
    try:
      candidates.append((os.path.getmtime(path), path))
    except OSError:
      continue
  return max(candidates, default=(0.0, PHONE_MEDIA_PATH))[1]


def _read_latest_phone_media() -> dict[str, Any]:
  with open(_latest_phone_media_path(), encoding="utf-8") as f:
    media = json.load(f)
  return media if isinstance(media, dict) else {}


def _same_media_item(left: dict[str, Any], right: dict[str, Any]) -> bool:
  left_title = str(left.get("title") or "").strip().casefold()
  right_title = str(right.get("title") or "").strip().casefold()
  if not left_title or left_title != right_title:
    return False

  left_artist = str(left.get("artist") or "").strip().casefold()
  right_artist = str(right.get("artist") or "").strip().casefold()
  if left_artist and right_artist and left_artist != right_artist:
    return False

  left_package = str(left.get("package") or "").strip().casefold()
  right_package = str(right.get("package") or "").strip().casefold()
  return not (left_package and right_package) or left_package == right_package


def _preserve_phone_media_art(media: dict[str, Any], previous: dict[str, Any]) -> bool:
  if media.get("artBase64") or not previous.get("artBase64") or not _same_media_item(media, previous):
    return False
  for key in ("artBase64", "artMime", "artHash", "artSource"):
    media[key] = previous.get(key, "")
  return True


async def health(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, "phoneMedia": True})


async def get_phone_media(request: web.Request) -> web.Response:
  try:
    media = _read_latest_phone_media()
  except FileNotFoundError:
    media = {}
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)
  return web.json_response({"ok": True, "media": media})


async def get_phone_media_diagnostics(request: web.Request) -> web.Response:
  path = _latest_phone_media_path()
  try:
    media = _read_latest_phone_media()
    stat = os.stat(path)
  except FileNotFoundError:
    return web.json_response({"ok": True, "present": False})
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)

  diagnostics = _art_diagnostics(media, _safe_int(media.get("artInputChars")))
  for key in ("artBytes", "artFormat", "artStatus"):
    if key in media:
      diagnostics[key] = media[key]
  return web.json_response({
    "ok": True,
    "present": True,
    "path": path,
    "fileBytes": stat.st_size,
    "fileAgeSeconds": max(0.0, time.time() - stat.st_mtime),  # noqa: TID251 - file mtime uses epoch time
    "title": str(media.get("title") or ""),
    "artist": str(media.get("artist") or ""),
    **diagnostics,
  })


async def set_phone_media(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)
  if not isinstance(body, dict):
    return web.json_response({"ok": False, "error": "invalid body"}, status=400)
  input_art_chars = _art_input_chars(body)
  media = _normalize_phone_media(body)
  try:
    previous = _read_latest_phone_media()
  except Exception:
    previous = {}
  art_preserved = _preserve_phone_media_art(media, previous)
  diagnostics = _store_art_diagnostics(media, input_art_chars, art_preserved)
  try:
    path = _write_phone_media(media)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)
  return web.json_response({
    "ok": True,
    "path": path,
    "artPreserved": art_preserved,
    **diagnostics,
  })


def register(app: web.Application) -> None:
  app.router.add_get("/health", health)
  app.router.add_get("/phone/media/diagnostics", get_phone_media_diagnostics)
  app.router.add_get("/phone/media", get_phone_media)
  app.router.add_post("/phone/media", set_phone_media)
