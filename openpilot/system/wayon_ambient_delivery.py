"""Coalesce relay notifications with a bounded offline-relay polling fallback."""
import os
from pathlib import Path
import uuid


AMBIENT_WAKE_PATH = Path(os.getenv("WAYON_AMBIENT_WAKE_PATH", "/dev/shm/wayon_ambient_wake"))
AMBIENT_NOTIFY_MESSAGE = "wayon-ambient-command-v1"
AMBIENT_POLL_INTERVAL_S = 60.0


def notify_ambient_command(path=AMBIENT_WAKE_PATH):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  temporary.write_text(uuid.uuid4().hex, encoding="ascii")
  os.replace(temporary, path)


class AmbientCommandDelivery:
  def __init__(self, path=AMBIENT_WAKE_PATH):
    self.path = Path(path)
    self.completed = None
    self.next_retry = 0.0
    self.failures = 0
    self.next_poll = 0.0

  def token(self):
    try:
      return self.path.read_text(encoding="ascii")
    except (OSError, UnicodeError):
      return "startup"

  def run(self, now, fetch_command):
    token = self.token()
    if now < self.next_retry or (token == self.completed and now < self.next_poll):
      return False
    try:
      fetch_command()
    except Exception:
      self.failures += 1
      self.next_retry = now + min(60.0, 5.0 * 2 ** min(self.failures - 1, 4))
      raise
    # A notification arriving during the request must remain pending.
    self.completed = token
    self.next_retry = 0.0
    self.next_poll = now + AMBIENT_POLL_INTERVAL_S
    self.failures = 0
    return True
