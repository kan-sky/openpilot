"""Durable, local systemd installation jobs for the Windows setup helper."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import install_cluster as installer


def write_report(directory, **values):
  installer.atomic_bytes(directory / 'result.json', json.dumps(values).encode())


def preflight():
  if not Path('/etc/carrot-jetlink-image.json').is_file():
    raise RuntimeError('Not a carrot-jetson image. No changes made.')
  current = installer.check(installer.RUNTIME)
  if Path('/etc/carrot-jetlink-protected.json').exists():
    state = json.loads(Path('/run/carrot-storage.json').read_text())
    if state.get('state') != 'protected':
      raise RuntimeError('Protected DATA storage is not ready. No changes made.')
  return {'current': str(current), 'pinned': (installer.RUNTIME / 'HL_CLUSTER_PIN.json').is_file()}


def start(directory, action, digest, runner=subprocess.run):
  token = directory.name.removeprefix('hl-cluster-')
  if not re.fullmatch('[a-f0-9]{32}', token) or directory.parent != Path('/var/tmp'):
    raise ValueError('Unexpected job directory')
  if (directory / 'result.json').exists():
    raise ValueError('This job has already been dispatched. Read its status, do not repeat it.')
  if action == 'install':
    bundle = directory / 'cluster-jetson-overlay.tar.gz'
    if not re.fullmatch('[a-f0-9]{64}', digest) or hashlib.sha256(bundle.read_bytes()).hexdigest() != digest:
      raise ValueError('Uploaded bundle checksum mismatch')
  unit = 'hl-cluster-' + token
  write_report(directory, state='starting', action=action, unit=unit)
  try:
    runner(['systemd-run', '--unit=' + unit, '--property=Type=exec', '--property=UMask=0077',
            sys.executable, str(directory / 'remote_helper.py'), 'run', '--action', action,
            '--sha256', digest, '--confirm-parked'], check=True, timeout=30)
  except Exception:
    # An SSH/start timeout is ambiguous; a started worker owns the report now.
    # Never overwrite a concurrently running/completed worker's result here.
    raise RuntimeError('Job dispatch was not confirmed. Query this job before trying again.') from None
  return {'state': 'dispatched', 'unit': unit}


def run_job(directory, action, digest, runner=subprocess.run):
  write_report(directory, state='running', action=action)
  result = {'state': 'failed', 'action': action}
  log = directory / 'install.log'
  with log.open('w') as output:
    log.chmod(0o600)
    try:
      command = [sys.executable, '-u', str(directory / 'install_cluster.py'), action, '--confirm-parked']
      if action == 'install':
        command += ['--bundle', str(directory / 'cluster-jetson-overlay.tar.gz'), '--sha256', digest]
      completed = runner(command, stdout=output, stderr=subprocess.STDOUT, timeout=1500)
      result['returncode'] = completed.returncode
      if completed.returncode == 0:
        healthy = False
        for _ in range(15):
          statuses = [runner(['systemctl', 'is-active', '--quiet', service]).returncode for service in installer.SERVICES]
          if not any(statuses):
            healthy = True
            break
          time.sleep(2)
        result.update(state='succeeded' if healthy else 'needs-attention',
                      selected=str((installer.RUNTIME / 'current').resolve()), services_active=healthy)
        if not healthy:
          output.write('\nRuntime changed, but services are not active. Inspect log or rollback.\n')
    except Exception as exc:
      result.update(state='needs-attention', error=str(exc))
      output.write('\n' + str(exc) + '\n')
  write_report(directory, **result)
  return result


def status(directory, runner=subprocess.run):
  path = directory / 'result.json'
  report = json.loads(path.read_text()) if path.is_file() else {'state': 'not-started'}
  if report['state'] in ('starting', 'running'):
    unit = directory.name
    check = runner(['systemctl', 'show', unit, '--property=ActiveState', '--value'], capture_output=True, text=True, check=True)
    report['unit_state'] = check.stdout.strip()
    if report['unit_state'] not in ('active', 'activating', 'reloading'):
      # Read again to handle a worker completing while systemctl was running.
      report = json.loads(path.read_text())
      if report['state'] in ('starting', 'running'):
        report['state'] = 'unknown'
        report['error'] = 'Worker is not running. Interrupted or dispatch failed; inspect before retrying.'
  log = directory / 'install.log'
  report['log'] = ''
  if log.is_file():
    with log.open('rb') as stream:
      stream.seek(max(0, log.stat().st_size - 24000))
      report['log'] = stream.read().decode(errors='replace')
  return report


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('command', choices=('check', 'start', 'run', 'status'))
  parser.add_argument('--action', choices=('install', 'rollback'), default='install')
  parser.add_argument('--sha256', default='')
  parser.add_argument('--confirm-parked', action='store_true')
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('Use sudo -n. The stock image provides passwordless sudo to its SSH account.')
  directory = Path(__file__).resolve().parent
  if args.command == 'status':
    result = status(directory)
  elif args.command == 'check':
    result = preflight()
  else:
    if not args.confirm_parked:
      parser.error('Confirm parked, assistance disengaged and comma disconnected first')
    preflight()
    result = start(directory, args.action, args.sha256) if args.command == 'start' else run_job(directory, args.action, args.sha256)
  print(json.dumps(result))


if __name__ == '__main__':
  main()
