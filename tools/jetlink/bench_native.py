"""Desk-only native QCOM/USB/TRT benchmark. Never publishes driving messages."""
import argparse
import json
import mmap
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace

import numpy as np


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--seconds', type=int, default=120)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if not 30 <= args.seconds <= 1800:
    parser.error('--seconds must be between 30 and 1800')

  import cereal.messaging as messaging
  from openpilot.common.params import Params
  from openpilot.common.realtime import config_realtime_process
  from openpilot.selfdrive.modeld.modeld import ModelState
  from openpilot.selfdrive.modeld.jetlink import model
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.system.hardware import HARDWARE

  params = Params()
  sm = messaging.SubMaster(['deviceState', 'pandaStates', 'managerState'])
  for _ in range(10):
    sm.update(100)

  def safe():
    sm.update(0)
    assert params.get_bool('IsOffroad') and not params.get_bool('IsOnroad'), 'not offroad'
    assert all(sm.alive.values()) and all(sm.valid.values()), 'missing live safety state'
    assert not sm['deviceState'].started, 'device started'
    assert len(sm['pandaStates']) and all(not p.controlsAllowed and not p.ignitionLine and not p.ignitionCan
      and str(p.safetyModel) == 'noOutput' for p in sm['pandaStates']), 'panda not in noOutput'
    assert not any(p.running for p in sm['managerState'].processes
                   if p.name in ('modeld', 'controlsd', 'selfdrived')), 'driving processes running'

  def checked(result):
    assert isinstance(result, dict) and result['plan'].shape == (1, 33, 15)
    assert all(np.isfinite(v).all() for v in result.values() if isinstance(v, np.ndarray))

  safe()
  assert HARDWARE.get_device_type() == 'mici', 'this benchmark currently requires C4'
  cpu_path = Path('/sys/devices/system/cpu/online')
  assert cpu_path.read_text().strip() == '0-3', 'expected initial offroad power-save state'
  logs = messaging.sub_sock('logMessage')
  records, diagnostics = [], []
  joining = None
  original_fault = model.FAULT
  report = {'scope': 'synthetic NV12 ring buffers; real native QCOM, USB, TRT and parser; no driving publications',
            'seconds': args.seconds, 'frames': records, 'diagnostics': diagnostics}
  try:
    HARDWARE.set_power_save(False)
    config_realtime_process(7, 54)
    width, height = 1344, 760
    size = get_nv12_info(width, height)[3]
    backing = [mmap.mmap(-1, (size + 4095) // 4096 * 4096) for _ in range(8)]
    for buffer in backing:
      np.frombuffer(buffer, np.uint8)[:] = 128
    ring = [{name: SimpleNamespace(data=memoryview(backing[i + offset])[:size])
             for name, offset in (('img', 0), ('big_img', 4))} for i in range(4)]
    transforms = {name: np.eye(3, dtype=np.float32) for name in ('img', 'big_img')}
    inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
              'action_t': np.array([.2, .3], np.float32)}
    small = ModelState(width, height, False)
    report['native_cold_start_ms'] = []
    for i in range(3):
      started = time.monotonic()
      checked(small.run(ring[i], transforms, inputs, False))
      report['native_cold_start_ms'].append((time.monotonic() - started) * 1000)
    joining = model.JoiningModel(small, width, height)
    joining.small_runs = 3
    joining.ready = True
    joining.join_allowed = True
    safe()
    with tempfile.TemporaryDirectory(prefix='jetlink-bench-') as directory:
      model.FAULT = Path(directory) / 'fault'
      deadline = time.monotonic() + args.seconds
      next_check = 0.
      while time.monotonic() < deadline:
        if time.monotonic() >= next_check:
          safe()
          next_check = time.monotonic() + 1
        before = ('external' if joining.active else 'connect' if joining.connection else
                  'warmup' if joining.warp else 'prepare' if joining.preparation else 'native')
        started = time.monotonic()
        checked(joining.run(ring[len(records) % 4], transforms, inputs, False))
        finished = time.monotonic()
        record = {'frame': joining.frame, 'mono': started, 'ms': (finished - started) * 1000,
                  'before': before, 'external': joining.active}
        if joining.active:
          record.update(warp_ms=[v * 1000 for v in joining.warp.timings], server_us=joining.client.timings)
        records.append(record)
        for message in messaging.drain_sock(logs):
          value = json.loads(message.logMessage).get('msg')
          if isinstance(value, dict) and value.get('component') == 'jetlinkd':
            diagnostics.append(value)
        if len(records) % 200 == 0:
          print(json.dumps({'frames': len(records), 'external': joining.active,
                            'error': joining.error, 'last_ms': record['ms']}), flush=True)
        time.sleep(max(0., started + .05 - time.monotonic()))
      assert joining.active, joining.error or 'did not join within test duration'
      joining.join_allowed = False
      joining.client.close()
      started = time.monotonic()
      checked(joining.run(ring[0], transforms, inputs, False))
      report['closed_local_socket_fallback_ms'] = (time.monotonic() - started) * 1000
      assert not joining.active and joining.client is None and not joining.output_valid
      assert model.FAULT.is_file()
      for _ in range(20):
        checked(joining.run(ring[0], transforms, inputs, False))
    safe()
    for group in ('external', 'startup'):
      values = np.array([r['ms'] for r in records if r['external'] == (group == 'external')])
      assert len(values)
      report[group] = {'frames': len(values), 'mean_ms': float(values.mean()), 'p95_ms': float(np.percentile(values, 95)),
                       'max_ms': float(values.max()), 'over_50ms': int((values > 50).sum())}
    report['completed'] = True
  finally:
    try:
      if joining is not None:
        for resource in (joining.client, joining.connection, joining.preparation):
          if resource is not None:
            resource.close()
        if joining.preparation is not None:
          joining.preparation.thread.join(timeout=5)
    finally:
      model.FAULT = original_fault
      os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
      os.sched_setaffinity(0, {0, 1, 2, 3})
      if params.get_bool('IsOffroad') and not params.get_bool('IsOnroad'):
        HARDWARE.set_power_save(True)
      report['restored_cpu'] = cpu_path.read_text().strip()
      args.output.write_text(json.dumps(report, indent=2))
      print(json.dumps({k: v for k, v in report.items() if k not in ('frames', 'diagnostics')}), flush=True)


if __name__ == '__main__':
  main()
