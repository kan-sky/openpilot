import base64
import sys
from pathlib import Path


CLUSTER_DIR = Path(__file__).resolve().parents[1]
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from cluster_renderer import decode_phone_media_art_base64, phone_media_image_extensions


def test_decodes_android_no_wrap_jpeg_base64():
  raw = b"\xff\xd8\xff\xe0android-jpeg"
  encoded = base64.b64encode(raw).decode("ascii")

  assert decode_phone_media_art_base64(encoded) == raw
  assert phone_media_image_extensions(raw, "")[0] == ".jpg"


def test_magic_bytes_override_wrong_mime():
  raw = b"\x89PNG\r\n\x1a\ncontent"

  assert phone_media_image_extensions(raw, "image/jpeg")[:2] == (".png", ".jpg")


def test_decodes_data_uri_and_urlsafe_base64_without_padding():
  raw = b"\xff\xd8\xff\xfb\xef"
  encoded = base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")

  assert decode_phone_media_art_base64(f"data:image/jpeg;base64,{encoded}") == raw
