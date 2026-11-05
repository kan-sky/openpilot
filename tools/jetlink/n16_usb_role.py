"""Offroad-only recovery of an explicitly selected N16 Jetson's reversed USB role.

Uses existing pinned SSH and the installed, hash-pinned USB-C policy command.
No boot policy, network configuration, sysfs role or upstream updater is changed.
A lost dispatch reply consumes the attempt; it never authorizes an automatic retry.
"""
from contextlib import contextmanager
import fcntl
import inspect
import json
import math
import os
from pathlib import Path
import re
import shlex
import stat
import subprocess
import time

try:
  from . import manage_jetson as management
except ImportError:
  import manage_jetson as management

MODEL = '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
BASE = '84087a5b78118acc40234bfd8ef64235421f1aab'
POLICY_SHA256 = '7e7ef891f81f79a8e482b50b474d1c0b0d7cf2e6535341526b830a9a09a587fb'
PEER = '192.168.55.1'
SOURCE = '192.168.55.100'
SELECTION = Path('/data/models/carrot-jetlink-n16.json')
MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')
LINK_STATUS = Path('/dev/shm/carrot-jetlink.json')
STATE_FILE = 'n16-usb-role.json'
MAX_ATTEMPTS = 8
LEASE_SECONDS = 15


def read_object(path, limit=16384):
  path = Path(path)
  if path.is_symlink() or not path.is_file() or path.stat().st_size > limit:
    raise ValueError('Missing/unsafe recovery input: ' + path.name)
  value = json.loads(path.read_text())
  if not isinstance(value, dict):
    raise ValueError('Recovery input must be an object')
  return value


def selection(path=SELECTION):
  path = Path(path)
  if not path.exists() and not path.is_symlink():
    return None
  value = read_object(path, 4096)
  if (type(value.get('format')) is not int or value['format'] != 1 or value.get('model') != MODEL
      or value.get('base_source_commit') != BASE
      or re.fullmatch('[a-f0-9]{64}', str(value.get('content_sha256', ''))) is None):
    raise ValueError('Invalid explicit N16 selection')
  return {key: value[key] for key in ('model', 'content_sha256', 'base_source_commit')}


def inference_idle(path=MODEL_STATUS):
  # No modeld record is normal offroad. The remote checks additionally require
  # the inference service inactive with both MainPID and ControlPID zero.
  try:
    if not Path(path).exists() and not Path(path).is_symlink():
      return True
    return read_object(path).get('active') is False
  except (OSError, ValueError):
    return False


def link_ready(path=None):
  try:
    value = read_object(LINK_STATUS if path is None else path)
    updated = value.get('updated')
    return (value.get('state') == 'ready' and type(updated) in (int, float)
            and 0 <= time.monotonic() - updated < 3)
  except (OSError, ValueError):
    return False


def usb_route(command=management.run, sys_root=Path('/sys')):
  rows = json.loads(command(['ip', '-j', '-4', 'route', 'get', PEER], timeout=5))
  if (not isinstance(rows, list) or len(rows) != 1 or not isinstance(rows[0], dict)
      or rows[0].get('gateway') is not None or rows[0].get('prefsrc') != SOURCE
      or rows[0].get('dst') != PEER or rows[0].get('type', 'unicast') != 'unicast'):
    raise ValueError('Jetson is not on the exact direct USB route')
  interface = rows[0].get('dev', '')
  if not isinstance(interface, str) or re.fullmatch('[a-zA-Z0-9_.-]{1,15}', interface) is None:
    raise ValueError('Invalid USB interface')
  net = Path(sys_root) / 'class/net' / interface
  if (net / 'carrier').read_text().strip() != '1':
    raise ValueError('USB carrier is absent')
  device = (net / 'device').resolve(strict=True)
  if not device.is_relative_to((Path(sys_root) / 'devices').resolve()):
    raise ValueError('Network interface is not a physical sysfs device')
  if (device / 'driver').resolve(strict=True).name not in ('cdc_ether', 'cdc_ncm', 'rndis_host'):
    raise ValueError('Not a supported USB Ethernet driver')
  for parent in (device, *device.parents):
    if not (parent / 'idVendor').exists():
      continue
    # Only the nearest USB device counts, never an NVIDIA or Linux parent hub.
    if ((parent / 'idVendor').read_text().strip().lower(),
        (parent / 'idProduct').read_text().strip().lower()) != ('0955', '7020'):
      raise ValueError('USB Ethernet is not the verified NVIDIA 0955:7020 device')
    numbers = [(parent / key).read_text().strip() for key in ('busnum', 'devnum')]
    if any(not number.isdigit() for number in numbers):
      raise ValueError('Invalid USB device enumeration identity')
    return {'interface': interface, 'source': SOURCE, 'device': str(parent), 'bus': numbers[0], 'number': numbers[1]}
  raise ValueError('No physical USB device identity')


def _remote_check(expected, filesystem='/'):
  """Self-contained read-only check also embedded in the independent root job."""
  import hashlib
  import importlib.util
  import json
  from pathlib import Path
  import platform
  import re
  import subprocess
  import time

  fs = Path(filesystem).resolve()
  runtime = fs / 'opt/carrot-jetlink'
  policy = fs / 'usr/local/lib/carrot-jetlink/usbc_policy.py'

  def require(condition, message):
    if not condition:
      raise ValueError(message)

  def regular(path):
    require(not path.is_symlink() and path.is_file(), 'Nonregular installed file')
    require(path.stat().st_uid == 0 and not path.stat().st_mode & 0o022, 'Installed code must be root-owned')
    return path

  def digest(path):
    value = hashlib.sha256()
    with regular(path).open('rb') as stream:
      for block in iter(lambda: stream.read(4 << 20), b''):
        value.update(block)
    return value.hexdigest()

  def status(name):
    raw = subprocess.check_output(['systemctl', 'show', name, '--property=ActiveState',
                                   '--property=MainPID', '--property=ControlPID'], text=True, timeout=5)
    return dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)

  boot = (fs / 'proc/sys/kernel/random/boot_id').read_text().strip()
  require(re.fullmatch('[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', boot), 'Invalid boot identity')
  require(expected.get('boot_id', boot) == boot, 'Jetson rebooted after authorization')
  if 'expires' in expected:
    require(time.monotonic() <= expected['expires'], 'Role authorization expired')
  require(platform.machine() == 'aarch64', 'Wrong Jetson architecture')
  compatible = (fs / 'proc/device-tree/compatible').read_bytes().decode().split('\0')
  require(any(re.fullmatch(r'nvidia,p3767-0001(?:\+[^\x00]+)?', item) for item in compatible), 'Not Orin NX 8GB')
  require(json.loads((fs / 'run/carrot-storage.json').read_text()).get('state') == 'protected', 'DATA not protected')
  transaction = runtime / 'HL_N16_TRANSACTION.json'
  require(not transaction.exists() and not transaction.is_symlink(), 'N16 transaction requires recovery')
  require((runtime / 'current').is_symlink(), 'Missing selected release')
  current = (runtime / 'current').resolve(strict=True)
  require(current.parent == runtime / 'releases', 'Release outside runtime')
  require((current / 'SOURCE_COMMIT').read_text().strip() == expected['pin']['base_source_commit'], 'Base differs')
  manifest = json.loads(regular(current / 'HL_N16_OVERLAY.json').read_text())
  require(manifest.get('kind') == 'device-local-n16' and type(manifest.get('format')) is int and manifest['format'] == 1,
          'Not a device-local N16 overlay')
  require({key: manifest.get(key) for key in expected['pin']} == expected['pin'], 'Overlay identity differs')
  content = hashlib.sha256(json.dumps(manifest['files'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
  require(content == expected['pin']['content_sha256'], 'Overlay content identity differs')
  helper = current / 'tools/jetlink/n16_overlay.py'
  require(digest(helper) == manifest['files']['tools/jetlink/n16_overlay.py'], 'Overlay verifier changed')
  module_spec = importlib.util.spec_from_file_location('_n16_role_verifier', helper)
  module = importlib.util.module_from_spec(module_spec)
  module_spec.loader.exec_module(module)
  module.verify_files(current)  # No TensorRT import, engine load or USB connection.
  require(digest(policy) == expected['policy_sha256'], 'Installed USB-C policy hash differs')
  inference = status('carrot-jetlink.service')
  require(inference == {'ActiveState': 'inactive', 'MainPID': '0', 'ControlPID': '0'}, 'Inference is not fully inactive')
  gate = status('carrot-jetlink-boot-gate.service')
  require(gate.get('ActiveState') == 'active' and gate.get('MainPID', '').isdigit()
          and int(gate['MainPID']) > 0 and gate.get('ControlPID') == '0', 'Boot gate is not running')
  ready = fs / 'run/carrot-jetlink-boot-ready.json'
  if ready.exists():
    require(json.loads(ready.read_text()).get('boot_id') != boot, 'This boot already passed the gate')
  require((runtime / 'current').resolve() == current, 'Release changed during verification')
  require((fs / 'proc/sys/kernel/random/boot_id').read_text().strip() == boot, 'Boot changed during verification')
  now = time.monotonic()
  if 'expires' in expected:
    require(now <= expected['expires'], 'Role authorization expired during verification')
  return {'boot_id': boot, 'overlay': expected['pin'], 'policy_sha256': expected['policy_sha256'],
          'selected': current.name, 'observed_monotonic': now}


def _remote_apply(expected):
  import json
  import os
  from pathlib import Path

  _remote_check(expected)
  # This does not take the updater lock: the waiting boot gate owns that lock.
  # /run is reset at boot, and O_EXCL burns the attempt even on an uncertain exit.
  marker = '/run/carrot-n16-usb-role-attempt.json'
  fd = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
  with os.fdopen(fd, 'w') as stream:
    json.dump(expected, stream)
    stream.flush()
    os.fsync(stream.fileno())
  _remote_check(expected)
  policy = Path('/usr/local/lib/carrot-jetlink/usbc_policy.py')
  os.execv('/usr/bin/python3', ['/usr/bin/python3', '-B', str(policy), '--mode', 'comma', '--parked-test'])


def remote_program(apply=False):
  code = inspect.getsource(_remote_check)
  if apply:
    code += '\n' + inspect.getsource(_remote_apply)
  code += '\nimport json, sys\nexpected = json.loads(sys.argv[1])\n'
  return code + ('_remote_apply(expected)\n' if apply else 'print(json.dumps(_remote_check(expected)))\n')


def ssh(state, route, arguments, timeout):
  # Existing host-key pin and per-pair identity, bound to the inspected USB IP.
  return management.run(['ssh', *management.ssh_options(state), '-b', route['source'],
                         'jetlink@' + PEER, shlex.join(arguments)], timeout=timeout)


@contextmanager
def locked(state):
  state = Path(state)
  info = state.stat()
  if state.is_symlink() or not state.is_dir() or info.st_uid != os.geteuid() or info.st_mode & 0o022:
    raise ValueError('Unsafe paired state directory')
  fd = os.open(state / 'n16-usb-role.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
  with os.fdopen(fd, 'a') as stream:
    info = os.fstat(stream.fileno())
    if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o022:
      raise ValueError('Unsafe recovery lock')
    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    yield


def save(state, value):
  management.private_json(Path(state) / STATE_FILE, value)
  fd = os.open(state, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def recover_once(safe, *, state=management.STATE, selection_path=SELECTION, policy_sha256=POLICY_SHA256,
                 idle=inference_idle, route_check=usb_route, remote=ssh):
  """One bounded attempt per paired Jetson boot; safe() must read fresh offroad state."""
  pin = selection(selection_path)
  if pin is None:
    return {'state': 'not-selected'}
  if re.fullmatch('[a-f0-9]{64}', str(policy_sha256)) is None:
    raise ValueError('A verified installed policy SHA256 is required')
  if safe() is not True or idle() is not True:
    return {'state': 'blocked', 'reason': 'Fresh offroad state and inactive inference required'}
  config = management.paired_config(state)
  route = route_check()
  with locked(state):
    journal_path = Path(state) / STATE_FILE
    journal = read_object(journal_path) if journal_path.exists() or journal_path.is_symlink() else {'format': 1, 'attempts': []}
    if (type(journal.get('format')) is not int or journal['format'] != 1
        or not isinstance(journal.get('attempts'), list) or len(journal['attempts']) > MAX_ATTEMPTS):
      raise ValueError('Invalid role-recovery attempt journal')
    expected = {'pin': pin, 'policy_sha256': policy_sha256}
    result = remote(state, route, ['sudo', '-n', '/usr/bin/python3', '-B', '-c', remote_program(), json.dumps(expected)], 25)
    if len(result) > 16384:
      raise ValueError('Oversized remote role snapshot')
    value = json.loads(result)
    if (not isinstance(value, dict) or value.get('overlay') != pin or value.get('policy_sha256') != policy_sha256
        or re.fullmatch('[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', str(value.get('boot_id'))) is None
        or type(value.get('observed_monotonic')) not in (int, float) or not math.isfinite(value['observed_monotonic'])
        or value['observed_monotonic'] < 0 or not str(value.get('selected', '')).startswith('hl-n16-')):
      raise ValueError('Remote N16 boot/overlay snapshot mismatch')
    boot = value['boot_id']
    for previous in journal['attempts']:
      if not isinstance(previous, dict) or not isinstance(previous.get('boot_id'), str) or not isinstance(previous.get('fingerprint'), str):
        raise ValueError('Invalid prior recovery attempt')
      if previous['boot_id'] == boot and previous['fingerprint'] == config['fingerprint']:
        return {'state': 'already-attempted', 'boot_id': boot, 'unit': previous.get('unit')}
    if (selection(selection_path) != pin or management.paired_config(state) != config or route_check() != route
        or safe() is not True or idle() is not True):
      return {'state': 'blocked', 'reason': 'Recovery prerequisites changed before dispatch'}
    expected.update(boot_id=boot, expires=value['observed_monotonic'] + LEASE_SECONDS)
    unit = 'carrot-n16-usb-role-' + boot.replace('-', '')
    attempt = {'boot_id': boot, 'fingerprint': config['fingerprint'], 'unit': unit, 'state': 'reserved'}
    journal['attempts'] = [*journal['attempts'], attempt][-MAX_ATTEMPTS:]
    save(state, journal)  # Persist before dispatch; losing SSH cannot create another job.
    if safe() is not True or idle() is not True:
      attempt['state'] = 'cancelled'
      save(state, journal)
      return {'state': 'blocked', 'reason': 'Offroad authorization revoked; attempt consumed'}
    command = ['sudo', '-n', 'systemd-run', '--unit=' + unit, '--no-block', '--collect',
               '--property=Type=oneshot', '--property=RuntimeMaxSec=60', '--property=TimeoutStartSec=60',
               '--property=TimeoutStopSec=5', '--property=KillMode=control-group', '--property=SendSIGKILL=yes',
               '--property=Restart=no', '--property=UMask=0077', '/usr/bin/python3', '-B', '-c',
               remote_program(apply=True), json.dumps(expected)]
    try:
      remote(state, route, command, 10)
      attempt['state'] = 'dispatched'
    except (OSError, RuntimeError, subprocess.SubprocessError):
      attempt['state'] = 'dispatch-unknown'
    save(state, journal)
    return {'state': attempt['state'], 'boot_id': boot, 'unit': unit,
            'verified': False}  # Native USB/model readiness is a separate observation.
