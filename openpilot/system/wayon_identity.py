#!/usr/bin/env python3
import json
from pathlib import Path


DEFAULT_ENDPOINT = "https://wayon-cloud.hyuklee.workers.dev"
DEFAULT_CONFIG_PATH = Path("/data/wayon_cloud/config.json")
USER_AGENT = "wayon-device-identity/1.0"


def _read_json(path: Path) -> dict:
  try:
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}
  except (OSError, TypeError, ValueError):
    return {}


def _param_text(params, key: str) -> str:
  value = params.get(key)
  if isinstance(value, bytes):
    return value.decode("utf-8", "replace").strip()
  return str(value or "").strip()


def ensure_wayon_identity(config_path: Path = DEFAULT_CONFIG_PATH, params=None) -> dict | None:
  config = _read_json(config_path)
  # Enrollment is explicit. Merely installing Carrot must never register a car.
  if not config or config.get("enabled", True) is False:
    return None
  if params is None:
    from openpilot.common.params import Params
    params = Params()
  endpoint = str(config.get("endpoint") or DEFAULT_ENDPOINT).rstrip("/")
  device_id = str(config.get("device_id") or _param_text(params, "DongleId"))
  old_token = str(config.get("token") or "")
  if not endpoint.startswith("https://") or not device_id or device_id == "UnregisteredDevice":
    return None
  if old_token.startswith("wayon_"):
    config.update(endpoint=endpoint, device_id=device_id)
    return config
  return None
