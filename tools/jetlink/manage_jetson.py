"""Paired comma -> Jetson SSH: USB management first, private LAN fallback."""
import argparse
import base64
import contextlib
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import struct
import subprocess
import sys
import time
import uuid

try:
  from .usb_management import JETSON_IP, usb_candidate
except ImportError:
  from usb_management import JETSON_IP, usb_candidate

ROOT = Path(__file__).resolve().parents[2]
STATE = Path('/data/hl-jetson')
N16_SELECTION = Path('/data/models/carrot-jetlink-n16.json')
ALIAS = 'hl-paired-jetson'
DEPLOYMENT_BRANCHES = frozenset({'cluster_jetson', 'WIP-FR'})
TERMINAL = {'succeeded', 'failed', 'rolled-back', 'needs-attention', 'interrupted', 'cancelled'}


def private_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
  temporary = path.with_name(path.name + '.' + uuid.uuid4().hex)
  try:
    with open(temporary, 'x', opener=lambda name, flags: os.open(name, flags, 0o600)) as stream:
      json.dump(value, stream)
      stream.flush()
      os.fsync(stream.fileno())
    temporary.replace(path)
  finally:
    temporary.unlink(missing_ok=True)


def read_json(path, default=None):
  try:
    return json.loads(path.read_text())
  except FileNotFoundError:
    return default


def address(value):
  ip = ipaddress.IPv4Address(value)
  networks = ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16')
  if str(ip) != JETSON_IP and not any(ip in ipaddress.IPv4Network(net) for net in networks):
    raise ValueError('Use the target Jetson USB or private LAN IPv4 address')
  return str(ip)


def public_key(value):
  parts = value.strip().split()
  if len(parts) < 2 or parts[0] != 'ssh-ed25519' or '\n' in value.strip():
    raise ValueError('Expected one Ed25519 public key, never a private key')
  raw = base64.b64decode(parts[1], validate=True)
  prefix = struct.pack('>I', 11) + b'ssh-ed25519' + struct.pack('>I', 32)
  if len(raw) != len(prefix) + 32 or not raw.startswith(prefix):
    raise ValueError('Invalid Ed25519 key encoding')
  return ' '.join(parts[:2])


def fingerprint(key):
  return 'SHA256:' + base64.b64encode(hashlib.sha256(base64.b64decode(public_key(key).split()[1])).digest()).decode().rstrip('=')


def run(args, timeout=10):
  result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
  if result.returncode:
    raise RuntimeError((result.stderr or result.stdout or f'{args[0]} failed')[-2000:].strip())
  return result.stdout


def init_key(state):
  state.mkdir(mode=0o700, parents=True, exist_ok=True)
  if state.is_symlink():
    raise ValueError('Management directory must not be a symlink')
  state.chmod(0o700)
  key = state / 'id_ed25519'
  pub = key.with_suffix('.pub')
  if key.is_symlink() or pub.is_symlink():
    raise ValueError('Key files must not be symlinks')
  if not key.exists() and not pub.exists():
    run(['ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-C', 'hl-comma-jetson', '-f', str(key)])
  if not key.is_file() or not pub.is_file():
    raise ValueError('Incomplete key pair; not replacing an existing key')
  key.chmod(0o600)
  actual = public_key(run(['ssh-keygen', '-y', '-P', '', '-f', str(key)]))
  if actual != public_key(pub.read_text()):
    raise ValueError('Private/public key mismatch')
  return actual


def ssh_options(state):
  return ['-F', '/dev/null', '-i', str(state / 'id_ed25519'), '-o', 'IdentitiesOnly=yes',
          '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
          '-o', 'HostKeyAlgorithms=ssh-ed25519', '-o', 'HostKeyAlias=' + ALIAS,
          '-o', 'UserKnownHostsFile=' + str(state / 'known_hosts'), '-o', 'GlobalKnownHostsFile=/dev/null',
          '-o', 'ConnectTimeout=4', '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=1',
          '-o', 'ForwardAgent=no', '-o', 'ClearAllForwardings=yes']


def ssh(state, host, args, timeout=10):
  return run(['ssh', *ssh_options(state), 'jetlink@' + address(host), shlex.join(args)], timeout)


def pair(state, host, host_key, expected):
  init_key(state)
  host = address(host)
  key = public_key(host_key)
  if fingerprint(key) != expected:
    raise ValueError('Host fingerprint does not match the independently verified key')
  if (state / 'config.json').exists():
    raise ValueError('Already paired; do not silently replace the target or its host key')
  known = state / 'known_hosts'
  if known.exists() and known.read_text().strip() != f'{ALIAS} {key}':
    raise ValueError('Existing host identity differs; investigate before replacing it')
  known.write_text(f'{ALIAS} {key}\n')
  known.chmod(0o600)
  # Enrollment is only complete after real, pinned-host SSH and sudo work.
  if ssh(state, host, ['id', '-un']).strip() != 'jetlink' or ssh(state, host, ['sudo', '-n', 'id', '-u']).strip() != '0':
    raise ValueError('Jetson account/sudo verification failed')
  private_json(state / 'config.json', dict(version=1, host=host, fingerprint=expected, automatic=False))


def candidates(config, state, discovered=()):
  try:
    from .usb_internet import mobile_candidates
  except ImportError:
    from usb_internet import mobile_candidates
  saved = read_json(state / 'last-host.json', {})
  usb = usb_candidate()
  mobile = mobile_candidates() if (state / 'usb-internet.enabled').is_file() else []
  values = [*usb, *mobile, *discovered, saved.get('host'), config.get('host')]
  result = []
  for value in values[:10]:
    try:
      ip = address(value)
      if ip == JETSON_IP and ip not in usb:
        continue
      if ip not in result:
        result.append(ip)
    except (ValueError, TypeError, ipaddress.AddressValueError):
      pass
  return result


def discover():
  try:
    from openpilot.common.jetlink_status import diagnostics
    value = diagnostics()
    return value.get('addresses', []) if value and value.get('fresh') and value.get('label') == 'jetSON' else []
  except (ImportError, OSError, ValueError, TypeError, KeyError):
    return []


def paired_config(state):
  config = read_json(state / 'config.json')
  if not config or config.get('version') != 1:
    raise ValueError('Pair the actual comma and Jetson first')
  known = (state / 'known_hosts').read_text().strip().split(maxsplit=1)
  if len(known) != 2 or known[0] != ALIAS or fingerprint(known[1]) != config.get('fingerprint'):
    raise ValueError('Pinned host identity/config mismatch; refusing connection')
  return config


def set_host(state, value):
  config = paired_config(state)
  host = address(value)
  ssh(state, host, ['true'])
  config['host'] = host
  private_json(state / 'config.json', config)
  private_json(state / 'last-host.json', dict(host=host))


def connect(state, allow_usb=True):
  config = paired_config(state)
  errors = []
  for host in candidates(config, state, discover()):
    if host == JETSON_IP and not allow_usb:
      continue
    try:
      ssh(state, host, ['true'])
      private_json(state / 'last-host.json', dict(host=host))
      return host
    except (RuntimeError, subprocess.SubprocessError) as error:
      errors.append(f'{host}: {error}')
  raise ConnectionError('No pinned Jetson reachable. Check USB management, hotspot isolation/IP/power. ' + ' | '.join(errors))


def connect_for_update(state):
  host = connect(state)
  if host == JETSON_IP and read_json(state / 'config.json', {}).get('usb_updates_verified') is not True:
    try:
      return connect(state, allow_usb=False)
    except ConnectionError as error:
        raise ConnectionError('USB-only service-restarting updates are not field-verified. ' +
                            'Connect the shared hotspot for this update; USB shell remains available.') from error
  return host


def remote(state, host, token, command, *args):
  if not re.fullmatch('[a-f0-9]{32}', token):
    raise ValueError('Invalid job ID')
  script = f'/var/tmp/hl-maintenance/{token}/maintenance_host.py'
  return json.loads(ssh(state, host, ['sudo', '-n', 'python3', script, command, '--job', token, *args]))


def upload(state, host, token, bundle=None):
  directory = f'/var/tmp/hl-maintenance/{token}'
  ssh(state, host, ['mkdir', '-p', '-m', '700', directory])
  for source, name in [(ROOT / 'tools/jetlink/maintenance_host.py', 'maintenance_host.py'),
                       (ROOT / 'tools/jetlink/install_cluster.py', 'install_cluster.py'),
                       *([(bundle, 'overlay.tar.gz')] if bundle else [])]:
    run(['scp', *ssh_options(state), str(source), f'jetlink@{address(host)}:{directory}/{name}'], timeout=180)


def read_road_flag(raw):
  if type(raw) is bool:
    return raw
  if raw in (b'0', '0'):
    return False
  if raw in (b'1', '1'):
    return True
  return None


def offroad_allowed(onroad, offroad, started, age, valid, driver_view):
  return (read_road_flag(onroad) is False and read_road_flag(offroad) is True
          and started is False and valid is True and 0 <= age < 3 and driver_view is False)


class Safety:
  def __init__(self):
    from openpilot.common.params import Params
    from openpilot.cereal import messaging
    self.params = Params()
    self.sm = messaging.SubMaster(['deviceState'])
    self.received = float('-inf')

  def __call__(self):
    self.sm.update(0)
    if self.sm.updated['deviceState']:
      self.received = time.monotonic()
    return offroad_allowed(self.params.get('IsOnroad'), self.params.get('IsOffroad'),
                           self.sm['deviceState'].started, time.monotonic() - self.received,
                           bool(self.sm.valid['deviceState'] and self.sm.alive['deviceState']),
                           self.params.get_bool('IsDriverViewEnabled'))


def require_safe(safe):
  if not safe():
    raise RuntimeError('Maintenance requires fresh offroad/ignition-off state; keep vehicle off until complete')


@contextlib.contextmanager
def locked(state):
  state.mkdir(parents=True, exist_ok=True, mode=0o700)
  with (state / 'client.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    yield


def build_bundle(state):
  branch = run(['git', '-C', str(ROOT), 'branch', '--show-current']).strip()
  if branch not in DEPLOYMENT_BRANCHES:
    raise ValueError('Deployment only supports reviewed branches: ' + ', '.join(sorted(DEPLOYMENT_BRANCHES)))
  if run(['git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no']).strip():
    raise ValueError('Tracked working-tree changes present; commit/review before deployment')
  commit = run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']).strip()
  from tools.jetlink.build_cluster_bundle import build
  path = state / 'overlay.tar.gz'
  build(path)
  if (run(['git', '-C', str(ROOT), 'branch', '--show-current']).strip() != branch
      or run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']).strip() != commit
      or run(['git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no']).strip()):
    raise ValueError('Source changed while packaging; wait until Git update completes')
  import tarfile
  with tarfile.open(path) as archive:
    manifest = json.load(archive.extractfile('HL_CLUSTER_MANIFEST.json'))
  tracked = set(run(['git', '-C', str(ROOT), 'ls-files', '-z']).split('\0'))
  if set(manifest['files']) - tracked:
    raise ValueError('Untracked runtime inputs present; commit/review before deployment')
  return path, path.with_suffix('.gz.sha256').read_text().strip(), manifest['content_sha256']


def save_status(directory, **value):
  private_json(directory / 'status.json', dict(updated=time.monotonic(), **value))


def follow(state, host, job, safe):
  token = job['token']
  deadline = time.monotonic() + 1560
  while time.monotonic() < deadline:
    cancelled = read_json(state / 'cancel.json', {}).get('token') == token
    permitted = safe() and not cancelled
    if cancelled:
      remote(state, host, token, 'cancel')
    remote(state, host, token, 'lease', '--safe', 'yes' if permitted else 'no')
    report = remote(state, host, token, 'status')
    if report['state'] == 'interrupted':
      private_json(state / 'cancel.json', dict(token=token))
      report = remote(state, host, token, 'cancel')
    save_status(state, **report)
    if report['state'] in TERMINAL:
      private_json(state / 'last-result.json', dict(job=job, report=report))
      (state / 'job.json').unlink(missing_ok=True)
      return report
    if report['state'] == 'not-started':
      raise RuntimeError('Job dispatch not confirmed. Run inspect; never redispatch an ambiguous job automatically')
    time.sleep(3)
  raise TimeoutError('Job still unconfirmed; use inspect. No duplicate job will be started.')


def sync(state, safe, action='sync', retry=False):
  with locked(state):
    require_safe(safe)
    host = connect_for_update(state)
    pending = read_json(state / 'job.json')
    if pending:
      if action != pending['action']:
        raise ValueError('Another action is pending; inspect/cancel it before starting ' + action)
      return follow(state, host, pending, safe)
    bundle, digest, content = build_bundle(state) if action == 'sync' else (None, '', None)
    last = read_json(state / 'last-result.json', {})
    if (action == 'sync' and last.get('job', {}).get('content') == content
        and last.get('report', {}).get('state') != 'succeeded' and not retry):
      report = {**last['report'], 'already_attempted': True}
      save_status(state, **report)
      return report
    require_safe(safe)
    token = uuid.uuid4().hex
    upload(state, host, token)
    current = remote(state, host, token, 'check')
    if current['recovery_required'] and action != 'recover':
      raise ValueError('Interrupted Jetson transaction; inspect then run recover')
    if current.get('content') == content and action == 'sync':
      report = dict(state='succeeded', action=action, unchanged=True)
      job = dict(token=token, content=content, action=action)
      private_json(state / 'last-result.json', dict(job=job, report=report))
      save_status(state, **report)
      return report
    if bundle:
      run(['scp', *ssh_options(state), str(bundle),
           f'jetlink@{address(host)}:/var/tmp/hl-maintenance/{token}/overlay.tar.gz'], timeout=180)
    require_safe(safe)
    job = dict(token=token, content=content, action=action, host=host)
    # Persist before dispatch. A lost SSH reply must never create a second job.
    private_json(state / 'job.json', job)
    remote(state, host, token, 'lease', '--safe', 'yes')
    remote(state, host, token, 'start', '--action', action, '--sha256', digest)
    return follow(state, host, job, safe)


def inspect(state):
  host = connect(state)
  job = read_json(state / 'job.json') or read_json(state / 'last-result.json', {}).get('job')
  if not job:
    return dict(host=host, state='paired', message='No maintenance job yet')
  return remote(state, host, job['token'], 'status')


def cancel_pending(state):
  job = read_json(state / 'job.json')
  if not job:
    return dict(state='no-pending-job')
  private_json(state / 'cancel.json', dict(token=job['token']))
  try:
    with locked(state):
      host = connect(state)
      remote(state, host, job['token'], 'cancel')
      return follow(state, host, job, lambda: False)
  except BlockingIOError:
    return dict(state='cancel-requested', token=job['token'])


def recover_n16_role(state, safe):
  # Optional, explicitly selected recovery must not change updater job/status.
  try:
    try:
      from . import n16_usb_role
    except ImportError:
      import n16_usb_role
    if n16_usb_role.link_ready():
      return  # Keep the last recovery diagnostic; READY is not a recovery error.
    report = n16_usb_role.recover_once(safe, state=state, selection_path=N16_SELECTION)
  except Exception as error:
    report = {'state': 'probe-error', 'reason': str(error)[-2400:]}
  try:
    private_json(state / 'n16-usb-role-status.json', dict(updated=time.monotonic(), **report))
  except Exception:
    pass  # Diagnostic disk failure must not interrupt existing jobs or leases.


def daemon():
  safe = Safety()
  stable_since = None
  completed_commit = None
  retry_commit, retry_after, retry_delay = None, 0., 30.
  n16_after = 0.
  while True:
    try:
      if (STATE / 'usb-internet.enabled').is_file():
        run(['sudo', '-n', '/usr/bin/python3', str(ROOT / 'tools/jetlink/usb_internet.py'), 'start-service'], timeout=20)
      config = read_json(STATE / 'config.json', {})
      permitted = safe()
      now = time.monotonic()
      stable_since = (now if stable_since is None else stable_since) if permitted else None
      stable_offroad = stable_since is not None and now - stable_since >= 30
      if (stable_offroad and now >= n16_after and not (STATE / 'job.json').exists()
          and (N16_SELECTION.exists() or N16_SELECTION.is_symlink())):
        n16_after = now + 30  # Bound failed/no-.55-route probes as well as dispatch.
        recover_n16_role(STATE, safe)
      if config.get('automatic') is True and stable_offroad:
        commit = run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']).strip()
        if commit != retry_commit:
          retry_commit, retry_after, retry_delay = commit, 0., 30.
        pending = (STATE / 'job.json').exists()
        if pending or (completed_commit != commit and time.monotonic() >= retry_after):
          report = sync(STATE, safe)
          retry_after, retry_delay = 0., 30.
          if report.get('state') in TERMINAL:
            completed_commit = commit
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, AttributeError, subprocess.SubprocessError) as error:
      # Keep existing jobs/leases responsive; only back off new failed builds.
      if (STATE / 'job.json').exists():
        retry_after, retry_delay = 0., 30.
      else:
        retry_after = time.monotonic() + retry_delay
        retry_delay = min(retry_delay * 2, 600.)
      save_status(STATE, state='waiting', reason=str(error)[-2400:], retry_after=retry_after)
    time.sleep(10)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--state', type=Path, default=STATE)
  sub = parser.add_subparsers(dest='command', required=True)
  sub.add_parser('init-key')
  usb = sub.add_parser('usb')
  usb.add_argument('mode', choices=('on', 'off'))
  usb_updates = sub.add_parser('usb-updates')
  usb_updates.add_argument('mode', choices=('on', 'off'))
  usb_updates.add_argument('--confirm-field-checks', action='store_true')
  pairing = sub.add_parser('pair')
  pairing.add_argument('--host', required=True)
  pairing.add_argument('--host-key-file', type=Path, required=True)
  pairing.add_argument('--fingerprint', required=True)
  target = sub.add_parser('set-host')
  target.add_argument('--host', required=True)
  auto = sub.add_parser('auto')
  auto.add_argument('mode', choices=('on', 'off'))
  auto.add_argument('--confirm-field-checks', action='store_true')
  sub.add_parser('inspect')
  sub.add_parser('cancel')
  sub.add_parser('shell')
  sub.add_parser('daemon')
  for name in ('sync', 'rollback', 'recover'):
    action = sub.add_parser(name)
    action.add_argument('--confirm-parked', action='store_true')
    action.add_argument('--retry', action='store_true')
  args = parser.parse_args()
  state = args.state
  if args.command == 'init-key':
    print(init_key(state))
  elif args.command == 'usb':
    init_key(state)
    marker = state / 'usb-management.enabled'
    if args.mode == 'on':
      marker.touch(mode=0o600)
    else:
      marker.unlink(missing_ok=True)
    print('USB management ' + args.mode + '. Takes effect on next offroad reboot; active USB link unchanged.')
  elif args.command == 'usb-updates':
    config = paired_config(state)
    if args.mode == 'on' and not args.confirm_field_checks:
      raise ValueError('Verify USB-only SSH survives inference restart and recovery before enabling updates')
    config['usb_updates_verified'] = args.mode == 'on'
    private_json(state / 'config.json', config)
    print('USB service-restarting updates: ' + args.mode)
  elif args.command == 'pair':
    pair(state, args.host, args.host_key_file.read_text(), args.fingerprint)
    print('Paired. Automatic updates remain disabled until the field checklist passes.')
  elif args.command == 'set-host':
    set_host(state, args.host)
    print('Address updated after pinned-host SSH verification.')
  elif args.command == 'auto':
    config = read_json(state / 'config.json')
    if not config:
      raise ValueError('Pair first')
    last = read_json(state / 'last-result.json', {})
    if args.mode == 'on' and (not args.confirm_field_checks or last.get('job', {}).get('action') != 'sync'
        or last.get('report', {}).get('state') != 'succeeded'):
      raise ValueError('Complete a manual sync and field verification before enabling automatic updates')
    config['automatic'] = args.mode == 'on'
    private_json(state / 'config.json', config)
    print('Automatic updates: ' + args.mode)
  elif args.command == 'inspect':
    print(json.dumps(inspect(state), ensure_ascii=False, indent=2))
  elif args.command == 'cancel':
    print(json.dumps(cancel_pending(state), ensure_ascii=False, indent=2))
  elif args.command == 'shell':
    host = connect(state)
    os.execvp('ssh', ['ssh', *ssh_options(state), '-t', 'jetlink@' + host])
  elif args.command == 'daemon':
    daemon()
  else:
    if not args.confirm_parked:
      parser.error('Park, turn ignition off, keep vehicle off until complete, then pass --confirm-parked')
    if args.command != 'sync':
      config = read_json(state / 'config.json')
      if config:
        config['automatic'] = False
        private_json(state / 'config.json', config)
    safe = Safety()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not safe():
      time.sleep(0.2)
    print(json.dumps(sync(state, safe, args.command, args.retry), ensure_ascii=False, indent=2))


if __name__ == '__main__':
  sys.path.insert(0, str(ROOT))
  main()
