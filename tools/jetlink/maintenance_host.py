"""SSH-only maintenance worker. No network listener or inference protocol changes."""
import argparse
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time

import install_cluster as installer

JOBS = Path('/var/tmp/hl-maintenance')
TRANSACTION = 'HL_MAINTENANCE_TRANSACTION.json'
LAST_GOOD = 'HL_MAINTENANCE_LAST_GOOD.json'
LEASE_SECONDS = 20
MAX_BUNDLE = 256 << 20
SAVED_FILES = ('HL_CLUSTER_PIN.json', 'updater/update_host.py', 'updater/update_host.hl-original.py')


def read_json(path):
  return json.loads(path.read_text())


def write_json(path, value):
  installer.atomic_bytes(path, json.dumps(value).encode())


def boot_id():
  return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def job_path(token):
  if not re.fullmatch('[a-f0-9]{32}', token):
    raise ValueError('Invalid job ID')
  path = JOBS / token
  if path.is_symlink() or not path.is_dir() or path.resolve() != path:
    raise ValueError('Invalid job directory')
  return path


def lease(directory, safe):
  write_json(directory / 'lease.json', dict(safe=safe, received=time.monotonic(), boot=boot_id()))


def require_lease(directory):
  value = read_json(directory / 'lease.json')
  if (value.get('safe') is not True or value.get('boot') != boot_id()
      or not 0 <= time.monotonic() - value['received'] < LEASE_SECONDS):
    raise RuntimeError('Offroad lease expired/revoked; stopping maintenance')


def preflight(runtime=installer.RUNTIME):
  if not Path('/etc/carrot-jetlink-image.json').is_file():
    raise RuntimeError('Not a supported carrot-jetson image')
  current = installer.check(runtime)
  if Path('/etc/carrot-jetlink-protected.json').exists():
    if read_json(Path('/run/carrot-storage.json')).get('state') != 'protected':
      raise RuntimeError('Protected DATA unavailable; refusing factory image changes')
  manifest = current / 'HL_CLUSTER_MANIFEST.json'
  return dict(selected=str(current), content=read_json(manifest).get('content_sha256') if manifest.exists() else None,
              recovery_required=(runtime / TRANSACTION).exists())


def snapshot(runtime):
  value = dict(selected=str(installer.check(runtime).relative_to(runtime)), files={})
  for name in SAVED_FILES:
    path = runtime / name
    value['files'][name] = base64.b64encode(path.read_bytes()).decode() if path.exists() else None
  return value


def restore(runtime, value):
  selected = installer.release_target(runtime, value['selected'])
  if set(value['files']) != set(SAVED_FILES):
    raise ValueError('Incomplete recovery record')
  data = {name: base64.b64decode(raw, validate=True) if raw is not None else None
          for name, raw in value['files'].items()}
  for name, raw in data.items():
    if raw is None:
      (runtime / name).unlink(missing_ok=True)
    else:
      installer.atomic_bytes(runtime / name, raw)
  installer.switch(runtime, selected)
  flush()


def flush():
  if hasattr(os, 'sync'):
    os.sync()


def stop_process(process):
  try:
    os.killpg(process.pid, signal.SIGTERM)
    process.wait(timeout=5)
  except subprocess.TimeoutExpired:
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5)
  except ProcessLookupError:
    process.wait()


def command(args, directory, output, timeout=60, guarded=True):
  if guarded:
    require_lease(directory)
  process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, start_new_session=True, cwd='/tmp')
  deadline = time.monotonic() + timeout
  try:
    while process.poll() is None:
      if guarded:
        require_lease(directory)
      if time.monotonic() >= deadline:
        raise TimeoutError('Maintenance command timed out: ' + args[0])
      time.sleep(0.25)
    if process.returncode:
      raise RuntimeError(f'Maintenance command failed ({process.returncode}): {args[0]}')
  finally:
    if process.poll() is None:
      stop_process(process)


def services(directory, output, guarded=True):
  command(['systemctl', 'start', *reversed(installer.SERVICES)], directory, output, guarded=guarded)
  # Require both units to remain active, not merely a successful start request.
  for _ in range(3):
    time.sleep(2)
    command(['systemctl', 'is-active', '--quiet', *installer.SERVICES], directory, output, guarded=guarded)


def apply(directory, runtime, action, digest, output):
  require_lease(directory)
  transaction = runtime / TRANSACTION
  if action == 'recover':
    if not transaction.exists():
      raise ValueError('No interrupted transaction to recover')
    before = read_json(transaction)
  else:
    if transaction.exists():
      raise ValueError('Interrupted transaction requires recover before another update')
    before = snapshot(runtime)
  if action == 'rollback' and not (runtime / LAST_GOOD).is_file():
    raise ValueError('No previous maintained version recorded')
  if action == 'sync':
    bundle = directory / 'overlay.tar.gz'
    if (not re.fullmatch('[a-f0-9]{64}', digest) or not 0 < bundle.stat().st_size <= MAX_BUNDLE
        or hashlib.sha256(bundle.read_bytes()).hexdigest() != digest):
      raise ValueError('Uploaded bundle checksum/size mismatch')
    if shutil.disk_usage(runtime).free < 512 << 20:
      raise RuntimeError('Less than 512 MiB free; preserve recovery space before updating')
  if action != 'recover':
    write_json(transaction, before)
    flush()
  try:
    command(['systemctl', 'stop', *installer.SERVICES], directory, output)
    if action == 'recover':
      restore(runtime, before)
    elif action == 'rollback':
      restore(runtime, read_json(runtime / LAST_GOOD))
    elif action == 'sync':
      def probe(candidate):
        require_lease(directory)
        installer.check_navdy_prerequisites(candidate)
        args = ['runuser', '-u', 'jetlink', '--', str(runtime / 'venv/bin/python')]
        if (candidate / installer.NAVDY_RELAY).is_file():
          command([*args, str(candidate / installer.NAVDY_RELAY), '--help'], directory, output, timeout=15)
        command([*args, str(candidate / 'tools/jetlink/hud.py'), '--help'], directory, output)
        command([*args, str(candidate / 'tools/jetlink/probe_release.py'), str(runtime / 'cache')],
                directory, output, timeout=960)

      pin = runtime / 'HL_CLUSTER_PIN.json'
      if not pin.exists():
        installer.install(runtime, bundle, digest, probe)
      else:
        old = read_json(pin)
        current = installer.check(runtime)
        if current != installer.release_target(runtime, old['installed']):
          raise ValueError('Pinned runtime changed outside installer')
        if not (runtime / 'updater/update_host.hl-original.py').is_file():
          raise ValueError('Original updater backup missing')
        target, manifest = installer.prepare_release(runtime, current, bundle, digest, probe)
        require_lease(directory)
        installer.atomic_bytes(runtime / 'updater/update_host.py', (target / 'tools/jetlink/update_guard.py').read_bytes())
        write_json(pin, dict(previous=old['previous'], installed=str(target.relative_to(runtime)), manifest=manifest))
        flush()
        installer.switch(runtime, target)
    else:
      raise ValueError('Unsupported action')
    services(directory, output)
    require_lease(directory)
    if action != 'recover':
      write_json(runtime / LAST_GOOD, before)
    transaction.unlink()
    flush()
    return dict(state='succeeded', action=action)
  except Exception as error:
    # Recovery is unconditional: an expired lease must not leave services stopped.
    try:
      command(['systemctl', 'stop', *installer.SERVICES], directory, output, guarded=False)
      restore(runtime, before)
      services(directory, output, guarded=False)
      transaction.unlink()
      flush()
      return dict(state='rolled-back', action=action, error=str(error))
    except Exception as recovery_error:
      return dict(state='needs-attention', action=action, error=str(error), recovery_error=str(recovery_error))


def run_job(directory, action, digest):
  write_json(directory / 'result.json', dict(state='running', action=action, boot=boot_id()))
  with (directory / 'install.log').open('w') as output:
    try:
      with Path('/run/carrot-jetlink-update.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preflight()
        result = apply(directory, installer.RUNTIME, action, digest, output)
    except Exception as error:
      result = dict(state='failed', action=action, error=str(error))
    result['boot'] = boot_id()
    write_json(directory / 'result.json', result)


def status(directory):
  path = directory / 'result.json'
  value = read_json(path) if path.exists() else dict(state='not-started')
  if value['state'] in ('starting', 'running'):
    unit = 'hl-maintenance-' + directory.name
    result = subprocess.run(['systemctl', 'show', unit, '--property=ActiveState', '--value'],
                            capture_output=True, text=True, timeout=5, check=True)
    latest = read_json(path)
    if latest['state'] in ('starting', 'running') and (value.get('boot') != boot_id()
        or result.stdout.strip() not in ('active', 'activating', 'reloading')):
      latest.update(state='interrupted', error='Worker not running; inspect recovery_required before retrying')
    value = latest
  log = directory / 'install.log'
  if log.is_file():
    with log.open('rb') as stream:
      stream.seek(max(0, log.stat().st_size - 16000))
      value['log'] = stream.read().decode(errors='replace')
  value['recovery_required'] = (installer.RUNTIME / TRANSACTION).exists()
  return value


def dispatch(directory, action, digest):
  # Serialize cancellation with dispatch, including a delayed SSH start request.
  with (directory / 'dispatch.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    require_lease(directory)
    preflight()
    if (directory / 'result.json').exists():
      raise ValueError('Job already dispatched; query status instead of starting again')
    write_json(directory / 'result.json', dict(state='starting', action=action, boot=boot_id()))
    subprocess.run(['systemd-run', '--unit=hl-maintenance-' + directory.name, '--property=Type=exec', '--property=UMask=0077',
                    '--property=RuntimeMaxSec=1500', sys.executable, str(Path(__file__).resolve()),
                    'run', '--job', directory.name, '--action', action, '--sha256', digest], check=True, timeout=10)
  return dict(state='dispatched')


def cancel(directory):
  with (directory / 'dispatch.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    lease(directory, False)
    if not (directory / 'result.json').exists():
      write_json(directory / 'result.json', dict(state='cancelled', boot=boot_id()))
  return status(directory)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('command', choices=('check', 'lease', 'start', 'run', 'status', 'cancel'))
  parser.add_argument('--job', required=True)
  parser.add_argument('--action', choices=('sync', 'rollback', 'recover'), default='sync')
  parser.add_argument('--sha256', default='')
  parser.add_argument('--safe', choices=('yes', 'no'), default='no')
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('Use authenticated SSH and sudo -n')
  directory = job_path(args.job)
  if args.command == 'check':
    result = preflight()
  elif args.command == 'lease':
    lease(directory, args.safe == 'yes')
    result = dict(ok=True)
  elif args.command == 'status':
    result = status(directory)
  elif args.command == 'cancel':
    result = cancel(directory)
  elif args.command == 'run':
    run_job(directory, args.action, args.sha256)
    return
  else:
    result = dispatch(directory, args.action, args.sha256)
  print(json.dumps(result))


if __name__ == '__main__':
  main()
