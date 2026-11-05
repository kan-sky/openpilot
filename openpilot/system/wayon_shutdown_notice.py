"""Best-effort pre-poweroff notice; networking cannot hold the manager open."""
import json
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

NOTICE_BUDGET_S = 1.0
CONFIG_PATH = Path("/data/wayon_cloud/config.json")
REASONS = ("low_voltage", "offroad_timeout")


class _ReadOnlyIdentityParams:
  def get(self, key):
    if key != "DongleId":
      return None
    try:
      return Path("/data/params/d/DongleId").read_text(encoding="utf-8").strip()
    except OSError:
      return None


class _NoRedirect(HTTPRedirectHandler):
  def redirect_request(self, *args, **kwargs):
    return None


def _https_post(url, **kwargs):
  body = json.dumps(kwargs["json"]).encode("utf-8")
  request = Request(url, data=body, method="POST", headers={**kwargs["headers"], "Content-Type": "application/json"})
  with build_opener(_NoRedirect()).open(request, timeout=kwargs["timeout"][1]) as response:
    value = json.loads(response.read(4096))
    return SimpleNamespace(status_code=response.status, json=lambda: value)


def notice_payload(reason, sm, params):
  if reason not in REASONS:
    raise ValueError("unsupported automatic shutdown reason")
  from openpilot.system.hardware.automatic_shutdown import message_is_fresh
  voltage = None
  if message_is_fresh(sm, "peripheralState", time.monotonic()):
    raw = sm["peripheralState"].voltage
    if 0 < raw < 30000:
      voltage = round(raw / 1000.0, 3)
  return {
    "id": "shutdown-" + uuid.uuid4().hex,
    "eventType": "automatic_shutdown",
    "reason": reason,
    "occurredAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    "voltageV": voltage,
    "offroadLimitMinutes": params.get_int("MaxTimeOffroadMin"),
  }


def notify_automatic_shutdown(reason, sm, params):
  payload = notice_payload(reason, sm, params)
  # An isolated child bounds DNS, TLS and imports too, unlike a socket timeout.
  # No token is placed in argv or logs. run() kills/reaps the child on timeout.
  try:
    result = subprocess.run(
      [sys.executable, "-m", "openpilot.system.wayon_shutdown_notice"],
      input=json.dumps(payload), text=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
      timeout=NOTICE_BUDGET_S, check=False,
    )
    status = "accepted" if result.returncode == 0 else "unavailable"
  except subprocess.TimeoutExpired:
    status = "timeout"
  except OSError:
    status = "unavailable"
  print(f"Automatic shutdown notification: {reason} {status}", flush=True)


def send_notice(payload, config_path=CONFIG_PATH, post=None):
  if not isinstance(payload, dict) or payload.get("eventType") != "automatic_shutdown" or payload.get("reason") not in REASONS:
    return False
  # Reuse the enrolled identity without provisioning, writing config or Params.
  # Native Params initialization is too expensive in the final poweroff budget.
  from openpilot.system.wayon_identity import ensure_wayon_identity
  config = ensure_wayon_identity(config_path, params=_ReadOnlyIdentityParams())
  if not config:
    return False
  endpoint = config.get("endpoint", "")
  parsed = urlsplit(endpoint)
  if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
    return False
  if post is None:
    post = _https_post
  response = post(
    endpoint.rstrip("/") + "/api/vehicle-event",
    json=payload,
    headers={"Authorization": "Bearer " + config["token"], "User-Agent": "wayon-shutdown-notice/1.0"},
    timeout=(0.35, 0.6), allow_redirects=False,
  )
  return response.status_code == 202 and response.json().get("accepted") is True


def main():
  try:
    payload = json.loads(sys.stdin.read(4096))
    return 0 if send_notice(payload) else 1
  except Exception:
    return 1


if __name__ == "__main__":
  sys.exit(main())
