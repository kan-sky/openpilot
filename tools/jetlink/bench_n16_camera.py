"""Bounded, vehicle-disconnected C4 camera/N16 bench; no driving publication.

The operator must own camerad and, optionally, dmonitoringmodeld beforehand.
This file never starts/stops those services, the manager, HUD or USB daemon.
Run with --seconds 120 --output /tmp/n16-camera.json [--require-dm].
Navigation is explicitly absent (zero map/TBT), not live navigation validation.
Camera validity comes from fresh road/wide CameraState events correlated with
native frame IDs/timestamps; this producer does not populate VisionIPC.valid.
Safety sampling runs in the power-restoration watcher on SCHED_OTHER/CPU0-3.
A fixed mailbox survives blocking model initialization without queuing samples;
unsafe is latched, all checks are retained in the restoration report, and the
realtime parent requires a heartbeat younger than 500 ms from check start.

Default --calibration-source live requires fresh calibrated liveCalibration.
For offroad operation with calibrationd stopped, --calibration-source params
reads CalibrationParams once, using calibrationd's saved Event format and pose
limits. This is a recorded STATIC camera-to-road transform, not live pose or
proof of the current desk mounting. Its saved monotonic timestamp may belong to
another boot and is not treated as fresh. No calibration message is published,
no Params are changed, and the live mode never silently falls back to this mode.
"""
import argparse
import fcntl
import hashlib
import json
import math
import mmap
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
SAFETY_SERVICES = ('deviceState', 'pandaStates', 'managerState')
FORBIDDEN = ('modeld', 'controlsd', 'selfdrived', 'card')
CPU_ONLINE = Path('/sys/devices/system/cpu/online')
SAFETY_MAX_AGE = 2.5
SETUP_SECONDS = 120
CAMERA_MAX_AGE = .25
CAMERA_STATE_MAX_SKEW_NS = 100_000_000
GUARD_INTERVAL = .25
GUARD_MAX_AGE = .5
GUARD_MAILBOX_BYTES = 16384
GUARD_HEADER = struct.Struct('!QI')


def require(condition, detail):
  if not condition:
    raise RuntimeError(detail)


def save_report(path, report):
  temporary = path.with_name(path.name + '.tmp')
  temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
  temporary.replace(path)


def validate_desk(params, sm, now):
  require(params.get_bool('IsOffroad') and not params.get_bool('IsOnroad'), 'not offroad')
  for name in SAFETY_SERVICES:
    require(sm.alive[name] and sm.valid[name] and
            0 <= now - sm.recv_time[name] < SAFETY_MAX_AGE and
            0 <= now - sm.logMonoTime[name] * 1e-9 < SAFETY_MAX_AGE,
            f'missing/stale safety state: {name}')
  require(not sm['deviceState'].started, 'device started')
  pandas = sm['pandaStates']
  require(len(pandas) > 0, 'no Panda safety evidence')
  require(all(not p.controlsAllowed and not p.ignitionLine and not p.ignitionCan and
              str(p.safetyModel) == 'noOutput' and str(p.harnessStatus) == 'notConnected'
              for p in pandas), 'vehicle not disconnected: require harness notConnected, ignition off, Panda noOutput')
  processes = {p.name: p.running for p in sm['managerState'].processes}
  require(all(name in processes for name in ('modeld', 'controlsd', 'selfdrived')), 'incomplete manager process state')
  require(not any(processes.get(name, False) for name in FORBIDDEN), 'manager has a driving process running')


def check_processes():
  # Also reject unmanaged driving processes; managerState alone cannot see them.
  for entry in Path('/proc').iterdir():
    if not entry.name.isdecimal() or int(entry.name) == os.getpid():
      continue
    try:
      args = (entry / 'cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError, ProcessLookupError):
      continue
    for arg in args:
      token = arg.decode(errors='replace')
      leaf = token.rsplit('/', 1)[-1]
      require(not any(leaf in (name, name + '.py') or token.endswith('.' + name) for name in FORBIDDEN),
              f'driving process present: pid {entry.name}, {token}')


class DeskGuard:
  def __init__(self):
    from openpilot.cereal import messaging
    from openpilot.common.params import Params
    self.params = Params()
    self.sm = messaging.SubMaster(list(SAFETY_SERVICES))
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline and not all(self.sm.seen.values()):
      self.sm.update(100)
    self.check()

  def check(self):
    started = time.monotonic()
    self.sm.update(0)
    validate_desk(self.params, self.sm, time.monotonic())
    sampled = time.monotonic()
    check_processes()
    finished = time.monotonic()
    return {'mono': finished, 'check_started': started, 'sampling_ms': (sampled - started) * 1000,
            'proc_scan_ms': (finished - sampled) * 1000, 'check_ms': (finished - started) * 1000,
            'vehicle_disconnected': True, 'offroad': True,
            'started': False, 'pandas': [{'harness': str(p.harnessStatus), 'safety': str(p.safetyModel),
              'controls_allowed': bool(p.controlsAllowed), 'ignition_line': bool(p.ignitionLine),
              'ignition_can': bool(p.ignitionCan)} for p in self.sm['pandaStates']],
            'source_mono_ns': {name: self.sm.logMonoTime[name] for name in SAFETY_SERVICES},
            'source_recv_time': {name: self.sm.recv_time[name] for name in SAFETY_SERVICES}}


class GuardMailbox:
  """One bounded snapshot, no startup queue; POSIX locks are per process, not FD.

  All samples remain in the watcher's report. Unsafe is latched and the watcher
  stops immediately; it can never be overwritten by a later safe heartbeat.
  """
  def __init__(self, fd):
    self.fd = fd
    self.memory = mmap.mmap(fd, GUARD_MAILBOX_BYTES)
    self.failed = False

  def publish(self, sample):
    require(not self.failed, 'unsafe guard mailbox is latched')
    self.failed = sample['safe'] is not True
    payload = json.dumps(sample, allow_nan=False).encode()
    require(len(payload) <= GUARD_MAILBOX_BYTES - GUARD_HEADER.size, 'oversized guard heartbeat')
    fcntl.lockf(self.fd, fcntl.LOCK_EX)
    try:
      # An interrupted writer leaves an invalid snapshot, never a renewed old one.
      self.memory[:GUARD_HEADER.size] = GUARD_HEADER.pack(sample['sequence'], 0)
      self.memory[GUARD_HEADER.size:GUARD_HEADER.size + len(payload)] = payload
      self.memory[:GUARD_HEADER.size] = GUARD_HEADER.pack(sample['sequence'], len(payload))
    finally:
      fcntl.lockf(self.fd, fcntl.LOCK_UN)

  def read(self, previous_sequence):
    try:
      fcntl.lockf(self.fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
    except BlockingIOError:
      return None  # The cached heartbeat's original expiry still applies.
    try:
      sequence, size = GUARD_HEADER.unpack(self.memory[:GUARD_HEADER.size])
      if sequence == 0 or sequence == previous_sequence:
        return None
      require(sequence > previous_sequence and 0 < size <= GUARD_MAILBOX_BYTES - GUARD_HEADER.size,
              'invalid guard mailbox header')
      sample = json.loads(self.memory[GUARD_HEADER.size:GUARD_HEADER.size + size])
      require(sample['sequence'] == sequence, 'guard mailbox sequence mismatch')
      return sample
    finally:
      fcntl.lockf(self.fd, fcntl.LOCK_UN)

  def close(self):
    self.memory.close()


class GuardHeartbeat:
  """Nonblocking private IPC only; never sample Params, SubMaster or /proc here."""
  def __init__(self, child, channel):
    self.child, self.channel = child, channel
    self.current = None
    self.sequence = 0
    self.expires = 0.

  def check(self, report, power_active=True):
    started = time.monotonic()
    evidence = {'started': started, 'accepted': False}
    try:
      sample = self.channel.read(self.sequence)
      if sample is not None:
        sample['unread_safe_samples'] = sample['sequence'] - self.sequence - 1
        report['safety_checks'].append(sample)
        require(sample['sequence'] > self.sequence, 'guard heartbeat sequence replay')
        self.sequence = sample['sequence']
        require(sample['safe'] is True, f"unsafe guard heartbeat: {sample.get('error', 'unknown')}")
        require(sample['scheduler'] == 'SCHED_OTHER' and sample['cpus'] == [0, 1, 2, 3], 'guard scheduling changed')
        start, end = sample['check_started'], sample['mono']
        sources = [*sample['source_recv_time'].values(), *(v * 1e-9 for v in sample['source_mono_ns'].values())]
        require(set(sample['source_recv_time']) == set(sample['source_mono_ns']) == set(SAFETY_SERVICES),
                'incomplete guard source evidence')
        require(all(math.isfinite(v) for v in [start, end, *sources]) and start <= end and
                all(0 <= end - v < SAFETY_MAX_AGE for v in sources), 'invalid guard source timestamps')
        self.current = sample
        # Completion time must not renew evidence after a slow/blocked proc scan.
        self.expires = min(start + GUARD_MAX_AGE, *(v + SAFETY_MAX_AGE for v in sources))
      require(self.child.poll() is None, 'guard/restoration watcher exited prematurely')
      now = time.monotonic()
      require(self.current is not None and self.current['mono'] <= now < self.expires, 'stale/missing guard heartbeat')
      require(not power_active or (self.current['power_active'] and self.current['cpu_online'] == '0-7'),
              'power policy changed during benchmark')
      evidence.update(accepted=True, sequence=self.sequence, heartbeat_age_ms=(now - self.current['check_started']) * 1000)
      return evidence
    finally:
      evidence.update(finished=time.monotonic())
      evidence['check_ms'] = (evidence['finished'] - started) * 1000
      report.setdefault('guard_heartbeat_checks', []).append(evidence)

  def wait_ready(self, report, power_active=False):
    deadline = time.monotonic() + 10
    while True:
      require(self.child.poll() is None, 'guard/restoration watcher exited prematurely')
      require(time.monotonic() < deadline, 'guard heartbeat startup timeout')
      # Only startup waits; steady-state check() never sleeps or blocks on a lock.
      sample = self.channel.read(self.sequence)
      if sample is not None:
        self.check(report, power_active=False)
        if not power_active or self.current['power_active']:
          self.check(report, power_active=power_active)
          return
      time.sleep(.01)

  def close(self):
    self.channel.close()
    if getattr(self, 'backing', None) is not None:
      self.backing.close()


def process_identity(pid):
  try:
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
  except (FileNotFoundError, ProcessLookupError):
    return None


def watchdog(fd, heartbeat_fd, parent, identity, seconds, output, original_online):
  """Low-priority safety watcher and independent timeout/power restorer."""
  report = {'restored': False, 'original_cpu_online': original_online, 'guard_checks': []}
  channel = GuardMailbox(heartbeat_fd)
  normal = False
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.sched_setaffinity(0, {0, 1, 2, 3})
    os.nice(10)
    from openpilot.system.hardware import HARDWARE
    # Imports must succeed before the caller changes hardware state.
    from openpilot.common.params import Params  # noqa: F401
    from openpilot.cereal import messaging  # noqa: F401
    print('READY', flush=True)
    deadline, next_check, sequence, power_active = time.monotonic() + seconds, 0., 0, False
    guard = None
    while True:
      now = time.monotonic()
      if now >= deadline:
        report['reason'] = 'timeout_or_parent_exit'
        break
      readable, _, _ = select.select([fd], [], [], max(0., min(next_check, deadline) - now))
      if readable:
        command = os.read(fd, 1)
        if command != b'R':
          normal = command == b'D'
          report['reason'] = 'cleanup' if normal else 'timeout_or_parent_exit'
          break
        power_active, next_check = True, 0.
      if time.monotonic() < next_check:
        continue
      sequence += 1
      started = time.monotonic()
      sample = {'sequence': sequence, 'safe': False, 'check_started': started}
      try:
        guard = guard or DeskGuard()
        sample.update(guard.check())
        sample.update(cpu_online=CPU_ONLINE.read_text().strip(), power_active=power_active,
                      scheduler='SCHED_OTHER', cpus=sorted(os.sched_getaffinity(0)))
        require(os.sched_getscheduler(0) == os.SCHED_OTHER and sample['cpus'] == [0, 1, 2, 3], 'guard scheduling changed')
        require(not power_active or sample['cpu_online'] == '0-7', 'power policy changed during benchmark')
        require(time.monotonic() - sample['check_started'] < GUARD_MAX_AGE, 'guard check exceeded heartbeat lifetime')
        sample['safe'] = True
      except BaseException as error:
        sample.update(safe=False, mono=time.monotonic(), error=f'{type(error).__name__}: {error}')
        sample['check_ms'] = (sample['mono'] - sample['check_started']) * 1000
      sample['watcher_check_ms'] = (time.monotonic() - started) * 1000
      report['guard_checks'].append(sample)
      try:
        channel.publish(sample)
      except Exception as error:
        sample.update(safe=False, error=f'guard heartbeat delivery failed: {error}')
      if not sample['safe']:
        report['reason'] = 'unsafe_guard'
        report['guard_error'] = sample['error']
        break
      next_check = sample['check_started'] + GUARD_INTERVAL
    if not normal and process_identity(parent) == identity:
      try:
        os.kill(parent, signal.SIGTERM)
      except ProcessLookupError:
        pass
      deadline = time.monotonic() + 3
      while process_identity(parent) == identity and time.monotonic() < deadline:
        time.sleep(.05)
      if process_identity(parent) == identity:
        try:
          os.kill(parent, signal.SIGKILL)
        except ProcessLookupError:
          pass
    # Do not offline cores if another operator has since started driving work.
    DeskGuard().check()
    HARDWARE.set_power_save(original_online == '0-3')
    report['cpu_online'] = CPU_ONLINE.read_text().strip()
    require(report['cpu_online'] == original_online, 'CPU power restoration did not match original state')
    report['restored'] = True
  except BaseException as error:
    report['error'] = f'{type(error).__name__}: {error}'
  finally:
    channel.close()
    os.close(heartbeat_fd)
    os.close(fd)
    save_report(output, report)
  return 0 if report['restored'] else 1


def arm_watchdog(seconds, output, original_online):
  # Unlinked file-backed mmap works across exec without inheriting Python locks.
  backing = tempfile.TemporaryFile(prefix='n16-guard-')
  backing.truncate(GUARD_MAILBOX_BYTES)
  receiving = GuardMailbox(backing.fileno())
  read_fd, write_fd = os.pipe()
  try:
    child = subprocess.Popen(
      [sys.executable, '-B', str(Path(__file__).resolve()), '--restore-watchdog', str(read_fd), str(backing.fileno()),
       str(os.getpid()), process_identity(os.getpid()), str(seconds), str(output), original_online],
      pass_fds=(read_fd, backing.fileno()), start_new_session=True, stdout=subprocess.PIPE)
  except BaseException:
    os.close(write_fd)
    receiving.close()
    backing.close()
    raise
  finally:
    os.close(read_fd)
  try:
    require(bool(select.select([child.stdout], [], [], 15)[0]), 'restoration watchdog did not start')
    require(child.stdout.readline().strip() == b'READY', 'restoration watchdog initialization failed')
  except BaseException:
    os.close(write_fd)
    receiving.close()
    backing.close()
    child.wait(timeout=15)
    raise
  heartbeat = GuardHeartbeat(child, receiving)
  heartbeat.backing = backing
  return child, write_fd, heartbeat


def checked(result):
  import numpy as np
  require(isinstance(result, dict), 'model returned no parsed output')
  for name, shape in {'plan': (1, 33, 15), 'plan_stds': (1, 33, 15),
                      'lead': (1, 3, 6, 4), 'pose': (1, 6), 'lane_lines': (1, 4, 33, 2)}.items():
    require(name in result and result[name].shape == shape, f'bad parsed {name}')
  require(all(np.isfinite(value).all() for value in result.values() if isinstance(value, np.ndarray)),
          'nonfinite parsed output')


def layout(buf, width, height):
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, _, size = get_nv12_info(width, height)
  require((buf.width, buf.height, buf.stride, buf.uv_offset, len(buf.data)) ==
          (width, height, stride, stride * y_height, size), 'camera NV12 layout mismatch')


def camera_state_evidence(context, main_meta, wide_meta, now):
  evidence = {}
  for service, meta in (('roadCameraState', main_meta), ('wideRoadCameraState', wide_meta)):
    require(context.seen[service] and context.alive[service] and context.valid[service], f'invalid {service}')
    message_age = now - context.logMonoTime[service] * 1e-9
    receive_age = now - context.recv_time[service]
    require(0 <= message_age < CAMERA_MAX_AGE and 0 <= receive_age < CAMERA_MAX_AGE, f'stale {service}')
    state = context[service]
    require(0 < state.timestampSof <= state.timestampEof, f'invalid {service} exposure timestamps')
    capture_age = now - state.timestampEof * 1e-9
    require(0 <= capture_age < CAMERA_MAX_AGE, f'stale {service} capture')
    delta = int(state.frameId) - meta.frame_id
    sof_delta, eof_delta = state.timestampSof - meta.timestamp_sof, state.timestampEof - meta.timestamp_eof
    # sendState publishes VIPC first, then the same metadata as CameraState.
    # Separate transports may expose the adjacent frame (native cadence varies).
    require(abs(delta) <= 1, f'{service} frame ID mismatch')
    require((delta == 0 and sof_delta == eof_delta == 0) or
            (delta != 0 and 0 < delta * sof_delta <= CAMERA_STATE_MAX_SKEW_NS and
             0 < delta * eof_delta <= CAMERA_STATE_MAX_SKEW_NS), f'{service} exposure mismatch')
    evidence[service] = {'valid': bool(context.valid[service]), 'frame_id': int(state.frameId),
      'vipc_frame_delta': delta, 'event_mono_ns': context.logMonoTime[service],
      'message_age_ms': message_age * 1000, 'receive_age_ms': receive_age * 1000,
      'sof_ns': state.timestampSof, 'eof_ns': state.timestampEof, 'capture_age_ms': capture_age * 1000,
      'vipc_sof_delta_ns': sof_delta, 'vipc_eof_delta_ns': eof_delta}
  return evidence


def read_static_calibration(params):
  """Read the same saved Event as Calibrator.__init__, without starting it."""
  import numpy as np
  from openpilot.cereal import log
  from openpilot.selfdrive.locationd.calibrationd import INPUTS_NEEDED, is_calibration_valid

  raw = params.get('CalibrationParams')
  require(isinstance(raw, bytes) and 0 < len(raw) <= 65536, 'missing/oversized CalibrationParams')
  try:
    with log.Event.from_bytes(raw, traversal_limit_in_words=8192) as event:
      require(event.which() == 'liveCalibration' and event.valid, 'CalibrationParams is not a valid liveCalibration Event')
      saved = event.liveCalibration
      require(str(saved.calStatus) == 'calibrated' and saved.validBlocks >= INPUTS_NEEDED,
              'CalibrationParams does not contain a completed calibration')
      euler = np.array(saved.rpyCalib, dtype=np.float32)
      require(euler.shape == (3,) and np.isfinite(euler).all(), 'invalid saved calibration RPY')
      require(is_calibration_valid(euler), 'saved calibration is outside native camera pose limits')
      metadata = {'source': 'CalibrationParams', 'mode': 'static_recorded', 'live': False,
        'saved_event_mono_ns': int(event.logMonoTime), 'timestamp_boot': 'unknown; no freshness claim',
        'valid_blocks': int(saved.validBlocks), 'rpy_recorded': euler.tolist(),
        'sha256': hashlib.sha256(raw).hexdigest(),
        'limitation': 'fixed recorded camera-to-road warp; no dynamic pose or current mounting validation'}
  except Exception as error:
    raise RuntimeError(f'CalibrationParams rejected: {error}') from error
  return euler, metadata


def dm_summary(rows, now=None, started=None):
  count = len(rows)
  elapsed = (rows[-1]['mono_ns'] - rows[0]['mono_ns']) * 1e-9 if count > 1 else 0.
  consecutive = all(b['frame_id'] == a['frame_id'] + 1 for a, b in zip(rows, rows[1:]))
  hz = (count - 1) / elapsed if elapsed > 0 else None
  covers_run = (now is None or (count > 0 and 0 <= now - rows[-1]['mono_ns'] * 1e-9 < .5 and
                               (started is None or abs(rows[0]['mono_ns'] * 1e-9 - started) < .5)))
  max_gap = max(((b['mono_ns'] - a['mono_ns']) * 1e-9 for a, b in zip(rows, rows[1:])), default=0.)
  return {'received': count, 'valid': sum(r['valid'] for r in rows), 'span_seconds': elapsed,
          'hz': hz, 'consecutive': consecutive, 'covers_run': covers_run, 'max_gap_seconds': max_gap,
          'strict_20hz_pass': elapsed >= 10 and covers_run and consecutive and max_gap < .15 and
                              all(r['valid'] for r in rows) and hz >= 20.}


def bench(args, report):
  import numpy as np
  from openpilot.cereal import messaging
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  from openpilot.common.params import Params
  from openpilot.common.realtime import config_realtime_process
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.common.transformations.model import get_warp_matrix
  from openpilot.selfdrive.modeld.camera_sync import receive_camera_pair
  from openpilot.selfdrive.modeld.jetlink import link, model
  from openpilot.selfdrive.modeld.jetlink.contracts import N16_SPEC
  from openpilot.selfdrive.modeld.modeld import ModelState
  from openpilot.selfdrive.modeld.signal_shadow import ShadowClient
  from openpilot.system.hardware import HARDWARE

  require(os.path.isfile('/AGNOS') and HARDWARE.get_device_type() == 'mici', 'native C4/AGNOS required')
  params = Params()
  static_calibration = read_static_calibration(params) if args.calibration_source == 'params' else None
  if static_calibration is not None:
    report['calibration'] = static_calibration[1]
  streams = VisionIpcClient.available_streams('camerad', block=False)
  require(all(s in streams for s in (VisionStreamType.VISION_STREAM_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD)),
          'operator must start camerad with real road and wide streams first')
  original_online = CPU_ONLINE.read_text().strip()
  require(original_online in ('0-3', '0-7'), 'unrecognized original power profile')
  restore_path = args.output.with_name(args.output.name + '.restore.json')
  report.update(original_cpu_online=original_online, restoration_report=str(restore_path))
  restore, lease, heartbeat = arm_watchdog(args.seconds + SETUP_SECONDS + 15, restore_path, original_online)
  joining = None
  old_fault = model.FAULT
  old_shadow = ShadowClient.__dict__['optional']
  old_policy = os.sched_getscheduler(0)
  old_priority = os.sched_getparam(0)
  old_affinity = os.sched_getaffinity(0)
  try:
    report['guard_watcher'] = {'scheduler': 'SCHED_OTHER', 'cpus': [0, 1, 2, 3], 'nice_increment': 10,
      'interval_seconds': GUARD_INTERVAL, 'max_heartbeat_age_seconds': GUARD_MAX_AGE,
      'freshness_origin': 'check_started, additionally capped by safety source expiry',
      'transport': 'fixed 16 KiB mmap mailbox, nonblocking parent POSIX lock; unsafe latched',
      'full_check_history': 'restoration.guard_checks (includes samples superseded during native initialization)'}
    heartbeat.wait_ready(report)
    HARDWARE.set_power_save(False)
    os.write(lease, b'R')
    heartbeat.wait_ready(report, power_active=True)
    config_realtime_process(7, 54)
    require(os.sched_getscheduler(0) == os.SCHED_FIFO and os.sched_getaffinity(0) == {7}, 'RT scheduling not applied')
    # ModelState otherwise may spawn the optional signal observer from saved settings.
    ShadowClient.optional = classmethod(lambda cls, *a: None)
    main = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    extra = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
    require(main.connect(False) and extra.connect(False), 'existing camerad streams unavailable')
    require((main.width, main.height) == (extra.width, extra.height), 'road/wide dimensions differ')
    report['camera_size'] = [main.width, main.height]
    report['camera_validity'] = {'source': 'roadCameraState + wideRoadCameraState valid Events',
      'visionipc_valid': 'unused: CameraBuf::sendFrameToVipc does not populate this field',
      'max_age_seconds': CAMERA_MAX_AGE, 'max_frame_delta': 1,
      'max_adjacent_exposure_delta_ns': CAMERA_STATE_MAX_SKEW_NS,
      'same_frame_timestamps': 'exact', 'producer': 'camera_qcom2.cc CameraState::sendState'}
    context_services = ['deviceState', 'roadCameraState', 'wideRoadCameraState']
    if static_calibration is None:
      context_services.append('liveCalibration')
    context = messaging.SubMaster(context_services)
    dm = messaging.sub_sock('driverStateV2', conflate=False)
    setup_deadline = time.monotonic() + SETUP_SECONDS
    small = ModelState(main.width, main.height, False)
    joining = model.JoiningModel(small, main.width, main.height)
    inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
              'action_t': np.array([.2, .3], np.float32)}
    start = deadline = None
    first_active = None
    fallback_done = rejoined = False
    fault_until = 0.
    previous_frame = None
    with tempfile.TemporaryDirectory(prefix='n16-camera-') as directory:
      model.FAULT = Path(directory) / 'fault'
      while True:
        now = time.monotonic()
        if deadline is not None and now >= deadline:
          break
        require(start is not None or now < setup_deadline, 'N16 camera join exceeded setup deadline')
        heartbeat.check(report)
        for message in messaging.drain_sock(dm):
          report['dm_frames'].append({'mono_ns': message.logMonoTime, 'frame_id': message.driverStateV2.frameId,
                                     'valid': bool(message.valid), 'received': time.monotonic()})
        frames = receive_camera_pair(main, extra)
        heartbeat.check(report)  # A blocked camera receive cannot authorize inference on an expired guard.
        if frames is None:
          continue
        buf_main, meta_main, buf_extra, meta_extra = frames
        received = time.monotonic()
        require(all(0 < m.timestamp_sof <= m.timestamp_eof for m in (meta_main, meta_extra)), 'invalid camera timestamps')
        require(all(0 <= received - m.timestamp_eof * 1e-9 < CAMERA_MAX_AGE for m in (meta_main, meta_extra)), 'stale camera pair')
        context.update(0)
        if not all(context.seen.values()):
          continue
        require(all(context.valid.values()), 'invalid camera/calibration context')
        context_now = time.monotonic()
        require(0 <= context_now - context.logMonoTime['deviceState'] * 1e-9 < 2.5, 'stale deviceState')
        camera_evidence = camera_state_evidence(context, meta_main, meta_extra, context_now)
        if static_calibration is not None:
          # Copy before applying yaw trim: never accumulate it in the frozen pose.
          euler = static_calibration[0].copy()
          calibration_mono_ns = None
        else:
          calibration = context['liveCalibration']
          require(str(calibration.calStatus) == 'calibrated', 'existing calibrated camera pose required')
          euler = np.asarray(calibration.rpyCalib, dtype=np.float32)
          require(euler.shape == (3,) and np.isfinite(euler).all(), 'invalid calibration pose')
          calibration_mono_ns = context.logMonoTime['liveCalibration']
          require(0 <= context_now - calibration_mono_ns * 1e-9 < 5., 'stale liveCalibration')
          report['calibration'] = {'source': 'liveCalibration', 'mode': 'live', 'live': True,
                                   'source_mono_ns': calibration_mono_ns}
        dc = DEVICE_CAMERAS[(str(context['deviceState'].deviceType), str(context['roadCameraState'].sensor))]
        yaw_trim = params.get_float('CameraYawTrimDeg') * .01
        require(np.isfinite(yaw_trim), 'invalid camera yaw trim')
        euler[2] -= np.radians(yaw_trim)
        transforms = {'img': get_warp_matrix(euler, dc.fcam.intrinsics, False).astype(np.float32),
                      'big_img': get_warp_matrix(euler, dc.ecam.intrinsics, True).astype(np.float32)}
        report['camera_context'] = {'device': str(context['deviceState'].deviceType),
          'sensor': str(context['roadCameraState'].sensor), 'rpy_after_yaw_trim': euler.tolist(),
          'yaw_trim_deg': yaw_trim, 'calibration_mono_ns': calibration_mono_ns,
          'calibration_mode': report['calibration']['mode'],
          'transforms': {name: value.tolist() for name, value in transforms.items()},
          'stride': main.stride, 'uv_offset': main.uv_offset, 'buffer_len': main.buffer_len}
        for buf in (buf_main, buf_extra):
          layout(buf, main.width, main.height)
        require(previous_frame is None or meta_main.frame_id > previous_frame, 'non-increasing road frame ID')
        dropped = max(0, meta_main.frame_id - previous_frame - 1) if previous_frame is not None else 0
        previous_frame = meta_main.frame_id
        joining.ready = link.state().get('state') == 'ready'
        # Only this guarded bench supplies join permission; no vehicle messages are fabricated.
        joining.join_allowed = not fallback_done or received >= fault_until
        joining.source_sof = meta_main.timestamp_sof
        before_active = joining.active
        injected = False
        if first_active is not None and not fallback_done and received - first_active >= min(10., args.seconds / 4):
          require(joining.active and joining.client is not None, 'N16 lost before planned IPC fault')
          joining.join_allowed = False
          joining.client.close()  # Local IPC only. Do not unplug, reset or stop USB.
          injected = True
        run_start = time.monotonic()
        result = joining.run({'img': buf_main, 'big_img': buf_extra}, transforms, inputs, dropped > 0)
        finished = time.monotonic()
        if result is not None:
          checked(result)
        if joining.active:
          require(joining.spec == N16_SPEC and joining.client.spec == N16_SPEC, 'external model is not exact pinned N16')
          require(not joining.views['nav_map'].any() and not joining.views['navigation'].any(), 'navigation unexpectedly active')
          if first_active is None:
            first_active = start = finished
            deadline = start + args.seconds
            # DM coexistence is measured after startup, without SubMaster conflation.
            report['dm_frames'].clear()
            messaging.drain_sock(dm)
          if fallback_done:
            require(finished >= fault_until, 'N16 rejoined before invalid-output hold expired')
            rejoined = True
        elif before_active and not injected:
          raise RuntimeError(f'unplanned N16 fallback: {joining.error}')
        if injected:
          require(not joining.active and joining.client is None and not joining.output_valid and model.FAULT.is_file(),
                  'closed IPC did not retain local fallback and invalid-output hold')
          fault_stamp = float(model.FAULT.read_text())
          fault_until = joining.fault_until
          require(abs(fault_until - fault_stamp - link.FAULT_HOLD_SECONDS) < .001, 'fault hold duration changed')
          fallback_done = True
          report['ipc_fault'] = {'fault_at': fault_stamp, 'invalid_until': fault_until,
                                 'fallback_ms': (finished - run_start) * 1000, 'fallback_output': result is not None}
        if fallback_done and finished < fault_until:
          require(not joining.output_valid and not joining.active, 'invalid-output hold bypassed')
        report['frames'].append({'phase': 'external' if joining.active else 'hold' if fallback_done else 'startup',
          'road_frame_id': meta_main.frame_id, 'wide_frame_id': meta_extra.frame_id, 'ipc_frame': joining.frame,
          'sof_ns': meta_main.timestamp_sof, 'eof_ns': meta_main.timestamp_eof,
          'wide_sof_ns': meta_extra.timestamp_sof, 'wide_eof_ns': meta_extra.timestamp_eof,
          'camera_state_evidence': camera_evidence,
          'started': run_start, 'finished': finished, 'dropped': dropped, 'prepare_only': dropped > 0,
          'output_present': result is not None, 'output_valid': bool(joining.output_valid),
          'capture_age_start_ms': (run_start - meta_main.timestamp_eof * 1e-9) * 1000,
          'capture_age_end_ms': (finished - meta_main.timestamp_eof * 1e-9) * 1000,
          'run_ms': (finished - run_start) * 1000, 'injected_ipc_close': injected,
          'warp_ms': [v * 1000 for v in joining.warp.timings] if joining.active else None,
          'server_us': list(joining.client.timings) if joining.active else None})
        if len(report['frames']) % 200 == 0:
          print(json.dumps({'frames': len(report['frames']), 'n16_active': joining.active,
                            'rejoined': rejoined, 'last_ms': report['frames'][-1]['run_ms']}), flush=True)
      heartbeat.check(report)
      require(fallback_done and rejoined and joining.active and joining.output_valid, 'fallback/rejoin cycle incomplete')
      require(sum(r['phase'] == 'external' and r['finished'] >= fault_until for r in report['frames']) >= 20,
              'fewer than 20 post-rejoin N16 outputs')
      report['dm'] = dm_summary(report['dm_frames'], time.monotonic(), start)
      times = np.array([r['run_ms'] for r in report['frames'] if r['phase'] == 'external'])
      report['external_timing'] = {'frames': len(times), 'mean_ms': float(times.mean()),
        'p95_ms': float(np.percentile(times, 95)), 'max_ms': float(times.max()), 'over_50ms': int((times > 50).sum())}
      report.update(n16_sha256=N16_SPEC.sha256, rejoined=True)
      if args.require_dm:
        require(report['dm']['strict_20hz_pass'], 'required valid consecutive DM >=20 Hz not observed')
      report['completed'] = True
  finally:
    errors = []
    try:
      if joining is not None:
        for resource in (joining.client, joining.connection, joining.preparation):
          if resource is not None:
            try:
              resource.close()
            except Exception as error:
              errors.append(str(error))
        if joining.preparation is not None:
          joining.preparation.thread.join(timeout=5)
          if joining.preparation.thread.is_alive():
            errors.append('warp preparation supervisor did not exit')
    finally:
      model.FAULT = old_fault
      ShadowClient.optional = old_shadow
      try:
        os.sched_setscheduler(0, old_policy, old_priority)
        os.sched_setaffinity(0, old_affinity)
      except Exception as error:
        errors.append(str(error))
      try:
        os.write(lease, b'D')
      except OSError as error:
        errors.append(str(error))
      finally:
        os.close(lease)
      try:
        restore.wait(timeout=20)
        restore.stdout.close()
        report['restoration'] = json.loads(restore_path.read_text())
        if restore.returncode or not report['restoration'].get('restored'):
          errors.append('power restoration failed; inspect restoration report')
      except Exception as error:
        errors.append(str(error))
      finally:
        heartbeat.close()
      if errors:
        report.update(completed=False, cleanup_errors=errors)
        raise RuntimeError('; '.join(errors))


def main():
  if len(sys.argv) > 1 and sys.argv[1] == '--restore-watchdog':
    fd, heartbeat_fd, parent, identity, seconds, output, original = sys.argv[2:]
    return watchdog(int(fd), int(heartbeat_fd), int(parent), identity, float(seconds), Path(output), original)
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--seconds', type=int, default=120, help='measurement seconds after first N16 output (30..300)')
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--require-dm', action='store_true', help='fail unless valid consecutive driverStateV2 measures >=20 Hz')
  parser.add_argument('--calibration-source', choices=('live', 'params'), default='live',
                      help='live: fresh liveCalibration; params: explicitly frozen calibrated CalibrationParams, no live-pose claim')
  args = parser.parse_args()
  if not 30 <= args.seconds <= 300:
    parser.error('--seconds must be between 30 and 300')
  args.output = args.output.resolve()
  require(args.output.parent.is_dir(), 'output directory must already exist')
  report = {'completed': False, 'scope': 'real road/wide camera, native QCOM/Jetlink/N16; no driving publications',
            'navigation': 'absent: zero map and TBT throughout', 'seconds': args.seconds,
            'calibration_source_requested': args.calibration_source,
            'require_dm': args.require_dm, 'frames': [], 'dm_frames': [], 'safety_checks': []}
  def interrupted(signum, frame):
    raise RuntimeError(f'benchmark interrupted by signal {signum}')
  for signum in (signal.SIGINT, signal.SIGTERM):
    signal.signal(signum, interrupted)
  code = 0
  # Leave an explicit incomplete report even if a blocked native call needs SIGKILL.
  save_report(args.output, report)
  try:
    bench(args, report)
  except BaseException as error:
    report.update(completed=False, error=f'{type(error).__name__}: {error}')
    code = 1
  finally:
    if 'dm' not in report:
      report['dm'] = dm_summary(report['dm_frames'])
    save_report(args.output, report)
    print(json.dumps({k: v for k, v in report.items() if k not in ('frames', 'dm_frames', 'safety_checks', 'guard_heartbeat_checks')}), flush=True)
  return code


if __name__ == '__main__':
  sys.exit(main())
