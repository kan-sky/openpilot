"""Keep bounded USB fault evidence off the transport thread, without payloads."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

from openpilot.selfdrive.navdy.journal import JournalFiles


def fault_lines(kernel, adb, serial):
  kernel_rows = [(line.replace(serial, 'NAVDY') if serial else line)[:400] for line in kernel.splitlines()
                 if any(marker in line.lower() for marker in ('usb ', 'usbpd ', 'dwc3 ', 'xhci-hcd'))][-32:]
  adb_rows = [line.replace(serial, 'NAVDY')[:400] for line in adb.splitlines()
             if serial and serial in line and ('failed:' in line or 'terminated:' in line)][-32:]
  return [('kernel', line) for line in kernel_rows] + [('adb', line) for line in adb_rows]


class UsbFaultRecorder:
  def __init__(self, directory=None, *, command=subprocess.run, clock=time.monotonic):
    if directory is None:
      from openpilot.selfdrive.navdy import journal
      directory = journal.DIRECTORY / 'usb'
    self.directory, self.command, self.clock = directory, command, clock
    self.thread = None
    self.next_capture = 0.

  def capture(self, serial):
    now = self.clock()
    if now < self.next_capture or (self.thread is not None and self.thread.is_alive()):
      return
    self.next_capture = now + 60.
    self.thread = threading.Thread(target=self._capture, args=(serial, now), name='navdy-usb-evidence', daemon=True)
    try:
      self.thread.start()
    except RuntimeError:
      self.thread = None

  def _capture(self, serial, at):
    files = JournalFiles(self.directory, max_bytes=256 * 1024)
    try:
      # Both AGNOS dmesg and BusyBox accept -s. Never collect logcat or media payloads.
      result = self.command(['dmesg', '-s', '131072'], capture_output=True, text=True, timeout=2, check=False)
      kernel = result.stdout[-131072:] if result.returncode == 0 else ''
      adb = ''
      try:
        fd = os.open(f'/tmp/adb.{os.getuid()}.log', os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as source:
          source.seek(max(0, os.fstat(source.fileno()).st_size - 32768))
          adb = source.read(32768).decode('utf-8', errors='replace')
      except OSError:
        pass
      for source, line in fault_lines(kernel, adb, serial):
        files.write((json.dumps({'capturedAt': at, 'wallTime': time.time(), 'source': source, 'line': line}) + '\n').encode())
    except (OSError, ValueError, subprocess.SubprocessError):
      pass
    finally:
      files.close()

  def close(self):
    if self.thread is not None:
      self.thread.join(timeout=.25)
