"""Short-lived dashboard warning test with driving automation off. No CAN transport here."""
import json
import math
import os
from pathlib import Path
import secrets
import tempfile
import time

KEY = "FcwWarningTest"
DIRECTORY = Path("/dev/shm/carrot-fcw-test")
DURATION = 2.0
COOLDOWN = 5.0
STATUS_MAX_AGE = 0.5
REQUEST_MAX_AGE = 0.25


def read_json(path):
  try:
    with path.open() as stream:
      value = json.loads(stream.read(4097))
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def write_json(path, value):
  path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
  fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=".fcw-")
  try:
    with os.fdopen(fd, "w") as stream:
      json.dump(value, stream, allow_nan=False)
    os.replace(temporary, path)
  finally:
    if os.path.exists(temporary):
      os.unlink(temporary)


def recent(value, now, maximum):
  return type(value) in (int, float) and math.isfinite(value) and 0 <= now - value <= maximum


def current_status(directory=DIRECTORY, now=None):
  now = time.monotonic() if now is None else now
  status = read_json(directory / "status.json")
  if not recent(status.get("updated"), now, STATUS_MAX_AGE):
    return {"ready": False, "active": False}
  deadline = status.get("until", 0)
  status["active"] = (status.get("active") is True and type(deadline) in (int, float)
                      and math.isfinite(deadline) and 0 < deadline - now <= DURATION)
  return status


def request_test(value, directory=DIRECTORY, now=None):
  if type(value) is not int or value not in (0, 1):
    raise ValueError("FCW test requires integer 0 or 1.")
  now = time.monotonic() if now is None else now
  status = current_status(directory, now)
  if value == 1:
    if status.get("ready") is not True or status.get("active"):
      raise ValueError("FCW test unavailable: require live Traverse, valid CAN, P/D/L, no gas, stock ACC and openpilot OFF; wait 5 seconds after a test.")
    token = status.get("token")
    if not isinstance(token, str) or len(token) != 32:
      raise ValueError("FCW test controller is not ready.")
  # Cancellation remains possible even if the controller stopped publishing.
  write_json(directory / "request.json", {"token": status.get("token"), "value": value,
                                         "issued": now, "until": now + DURATION})


def manual_for_warning(cs, cc):
  try:
    return (cs.canValid and not cs.canTimeout and cs.gearShifter in ("park", "drive", "low")
            and math.isfinite(cs.vEgo) and cs.vEgo >= 0
            and not cs.gasPressed and not cs.cruiseState.enabled
            and not (cc.enabled or cc.latActive or cc.longActive))
  except (AttributeError, TypeError, ValueError):
    return False


class FcwWarningTest:
  def __init__(self, directory=DIRECTORY):
    self.directory = directory
    self.token = secrets.token_hex(16)
    self.until = self.cooldown_until = 0.0
    self.safe_since = self.last_update = None
    self.next_poll = self.next_status = 0.0
    self.previous = None

  def stop(self, now):
    if self.until:
      self.cooldown_until = max(self.cooldown_until, now + COOLDOWN)
    self.until = 0.0
    self.token = secrets.token_hex(16)

  def update(self, cs, cc, now=None):
    now = time.monotonic() if now is None else now
    continuous = self.last_update is not None and 0 <= now - self.last_update <= 0.15
    self.last_update = now
    safe = manual_for_warning(cs, cc)
    if not continuous or not safe:
      if self.safe_since is not None or self.until:
        self.stop(now)
      self.safe_since = None
    if safe and self.safe_since is None:
      self.safe_since = now
    if self.until and now >= self.until:
      self.stop(now)
    ready = safe and now - self.safe_since >= 0.5 and now >= self.cooldown_until
    try:
      if (ready or self.until) and now >= self.next_poll:
        self.next_poll = now + 0.04
        command = read_json(self.directory / "request.json")
        if command.get("token") == self.token:
          if type(command.get("value")) is not int or command["value"] not in (0, 1):
            self.stop(now)
          elif command["value"] == 0:
            self.stop(now)
          elif not self.until and command.get("value") == 1 and recent(command.get("issued"), now, REQUEST_MAX_AGE):
            deadline = command.get("until")
            if type(deadline) in (int, float) and math.isfinite(deadline) and 0 < deadline - command["issued"] <= DURATION:
              self.until = deadline
        elif self.until:
          self.stop(now)
      active = bool(safe and self.until > now)
      ready = bool(ready and not active and now >= self.cooldown_until)
      state = (ready, active, self.token)
      if state != self.previous or ((ready or active) and now >= self.next_status):
        write_json(self.directory / "status.json", {"token": self.token, "updated": now, "ready": ready,
                                                   "active": active, "until": self.until})
        self.previous, self.next_status = state, now + 0.2
      return active
    except (OSError, ValueError):
      # A broken diagnostic must neither hold a warning nor stop car control.
      self.stop(now)
      self.safe_since = None
      self.previous = None
      return False
