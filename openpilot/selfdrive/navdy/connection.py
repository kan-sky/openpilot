"""Bounded USB reattachment probe; it never writes display or navigation data."""
from __future__ import annotations

import os
import subprocess
import threading


class UsbProbe:
  def __init__(self, factory):
    self.factory = factory
    self.lock = threading.Lock()
    self.thread = None
    self.ready = None
    self.stopped = False
    self.next_attempt = 0.0
    self.cancel_read = self.cancel_write = None

  def poll(self, now):
    with self.lock:
      ready, self.ready = self.ready, None
      if ready is not None:
        return ready
      if self.stopped or (self.thread is not None and self.thread.is_alive()) or now < self.next_attempt:
        return None
      self.next_attempt = now + 2
      try:
        self.cancel_read, self.cancel_write = os.pipe()
        self.thread = threading.Thread(target=self._probe, name='navdy-usb-probe', daemon=True)
        self.thread.start()
      except (OSError, RuntimeError):
        for fd in (self.cancel_read, self.cancel_write):
          if fd is not None:
            os.close(fd)
        self.cancel_read = self.cancel_write = self.thread = None
    return None

  def _probe(self):
    candidate = None
    try:
      try:
        candidate = self.factory()
        candidate.recovery_allowed = lambda: not self.stopped
        candidate.recovery_fd = self.cancel_read
        candidate.connect()
      except (OSError, ValueError, subprocess.SubprocessError):
        if candidate is not None:
          candidate.close()
        return
    finally:
      with self.lock:
        if self.cancel_read is not None:
          os.close(self.cancel_read)
          self.cancel_read = None
        if self.cancel_write is not None:
          os.close(self.cancel_write)
          self.cancel_write = None
        if candidate is not None:
          candidate.recovery_fd = None
    with self.lock:
      if not self.stopped:
        self.ready = candidate
        return
    candidate.close()

  def close(self):
    with self.lock:
      self.stopped = True
      if self.cancel_write is not None:
        os.close(self.cancel_write)
        self.cancel_write = None
      if self.thread is None and self.cancel_read is not None:
        os.close(self.cancel_read)
        self.cancel_read = None
      ready, self.ready = self.ready, None
    if ready is not None:
      ready.close()
    # An in-flight ADB discovery has bounded subprocess timeouts and closes its
    # own late result. Never block the display writer on USB discovery.
