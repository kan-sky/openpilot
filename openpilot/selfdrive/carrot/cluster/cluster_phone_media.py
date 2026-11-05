"""Bounded, read-only phone metadata shared by local and Jetlink displays."""
from dataclasses import fields
import json
import math
import os
from pathlib import Path
import time

try:
    from cluster_models import PhoneMediaInfo
except ModuleNotFoundError:
    from openpilot.selfdrive.carrot.cluster.cluster_models import PhoneMediaInfo

PHONE_MEDIA_READ_INTERVAL_SECONDS = 0.5
PHONE_MEDIA_STALE_SECONDS = 15 * 60
PHONE_MEDIA_ART_BASE64_MAX_CHARS = 512 * 1024
PHONE_MEDIA_DEFAULT_PATHS = (
    "/data/media/0/carrotlink_sidecar/phone_media.json",
    "/tmp/carrotlink_phone_media.json",
)
TEXT_LIMITS = {"title": 120, "artist": 120, "package_name": 96, "art_base64": PHONE_MEDIA_ART_BASE64_MAX_CHARS,
               "art_mime": 64, "art_hash": 128, "art_source": 160, "art_format": 32, "art_status": 64}


def phone_media_from_mapping(data):
    if not isinstance(data, dict):
        return None
    values = {}
    for field in fields(PhoneMediaInfo):
        value = data.get(field.name)
        if field.name in TEXT_LIMITS:
            values[field.name] = value[:TEXT_LIMITS[field.name]].strip() if isinstance(value, str) else ""
        elif field.name == "is_playing":
            values[field.name] = value is True
        else:
            values[field.name] = value if type(value) is int and 0 <= value <= 2**53 else None
    media = PhoneMediaInfo(**values)
    return media if media.title or media.artist or media.art_base64 else None


class PhoneMediaReader:
    def __init__(self, paths=None):
        configured = os.environ.get("CARROTLINK_PHONE_MEDIA_PATH", "").strip()
        self.paths = tuple(Path(p) for p in (paths if paths is not None else
                           ((configured,) if configured else ()) + PHONE_MEDIA_DEFAULT_PATHS))
        self.next_read = 0.0
        self.cached = None

    def read(self):
        now = time.monotonic()
        if now < self.next_read:
            return self.cached
        self.next_read = now + PHONE_MEDIA_READ_INTERVAL_SECONDS
        self.cached = None
        candidates = []
        for path in self.paths:
            try:
                stat = path.stat()
                if path.is_file():
                    candidates.append((stat.st_mtime, path))
            except OSError:
                pass
        if not candidates:
            return None
        stamp, path = max(candidates, key=lambda item: item[0])
        age = time.time() - stamp
        if not math.isfinite(age) or age < -5 or age > PHONE_MEDIA_STALE_SECONDS:
            return None
        try:
            with path.open("rb") as stream:
                payload = stream.read(PHONE_MEDIA_ART_BASE64_MAX_CHARS + 32 * 1024 + 1)
            if len(payload) > PHONE_MEDIA_ART_BASE64_MAX_CHARS + 32 * 1024:
                return None
            document = json.loads(payload)
            if not isinstance(document, dict):
                return None
            data = {}
            for field in fields(PhoneMediaInfo):
                parts = field.name.split("_")
                key = parts[0] + "".join(part.title() for part in parts[1:])
                data[field.name] = document.get("package" if field.name == "package_name" else key)
            self.cached = phone_media_from_mapping(data)
        except (OSError, ValueError, UnicodeError):
            pass
        return self.cached
