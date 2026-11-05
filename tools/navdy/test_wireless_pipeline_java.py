"""Optional host-only integration with the v164 production Java parser/owner."""
import os
import socket
import subprocess
import time

import pytest

from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy.bluetooth_pipeline import SnapshotPipeline


@pytest.mark.skipif(not os.environ.get('NAVDY_JAVA_TEST_CLASSPATH'), reason='Requires compiled v164 host fixture')
def test_actual_python_pipeline_to_java_receiver():
  left, right = socket.socketpair()
  process = subprocess.Popen([os.environ.get('NAVDY_JAVA', 'java'), '-cp', os.environ['NAVDY_JAVA_TEST_CLASSPATH'],
                              'com.navdy.hud.app.openpilot.WirelessStdioHarness'],
                             stdin=right, stdout=right, stderr=subprocess.PIPE)
  right.close()
  pipeline = None
  replies = []
  try:
    hello = bt.HELLO.unpack(bt.receive(left, bt.HELLO.size, time.monotonic() + 5))
    assert hello == (b'NVH2', 0, bt.TTL_US, bt.MAX_BYTES)
    pipeline = SnapshotPipeline(left, time.monotonic_ns())
    for seq in range(15):
      captured = time.monotonic_ns()
      replies.extend(pipeline.update({'schema': 'navdy.openpilot.v1', 'active': seq % 2 == 0,
                                      'displayPower': 'onroad'}, captured_ns=captured))
      time.sleep(max(0., .2 - (time.monotonic_ns() - captured) / 1e9))
    deadline = time.monotonic() + 1
    while pipeline.pending and time.monotonic() < deadline:
      replies.extend(pipeline._read())
      time.sleep(.005)
    assert not pipeline.pending and len(replies) == 15
    assert [r[0]['wireless']['appliedSeq'] for r in replies] == list(range(15))
    assert all(r[0]['wireless']['clears'] == 0 for r in replies)
    assert all(r[0]['wireless']['owner'] == 'bluetooth' for r in replies)
    assert pipeline.diagnostics()['displayStatus'] == 0
  finally:
    if pipeline is not None:
      pipeline.close()
    left.close()
    try:
      process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      process.kill()
      process.wait(timeout=2)
    error = process.stderr.read().decode('utf-8', errors='replace')
    process.stderr.close()
  assert process.returncode == 0, error
  assert not pipeline.reader.thread.is_alive()
