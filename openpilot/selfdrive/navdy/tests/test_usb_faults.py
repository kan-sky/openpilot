from openpilot.selfdrive.navdy.usb_faults import fault_lines, UsbFaultRecorder


def test_fault_lines_are_bounded_and_exclude_keys_media_and_unrelated_devices():
  kernel = '\n'.join('[123] usb 3-1: error -71 ' + 'x' * 900 for _ in range(100))
  adb = '\n'.join(['auth.cpp: key PRIVATE', 'media: PRIVATE',
                   'OTHER: read failed: PRIVATE', 'TESTNAVDY: read failed: Protocol error'] * 100)
  rows = fault_lines(kernel, adb, 'TESTNAVDY')
  assert len(rows) == 64 and all(len(line) <= 400 for _, line in rows)
  assert all('PRIVATE' not in line and 'TESTNAVDY' not in line for _, line in rows)


def test_unknown_serial_does_not_copy_adb_log():
  assert fault_lines('', 'any: read failed: secret', None) == []


def test_slow_snapshot_does_not_block_or_spawn_multiple_workers(monkeypatch, tmp_path):
  import threading
  started, release = threading.Event(), threading.Event()
  calls = []
  recorder = UsbFaultRecorder(tmp_path, clock=lambda: 100.)
  def capture(*args):
    calls.append(args)
    started.set()
    release.wait(2)
  monkeypatch.setattr(recorder, '_capture', capture)
  try:
    recorder.capture('test')
    assert started.wait(1)
    for _ in range(10):
      recorder.capture('test')
    assert len(calls) == 1
    recorder.close()
    assert recorder.thread.is_alive()
  finally:
    release.set()
    recorder.thread.join(1)
