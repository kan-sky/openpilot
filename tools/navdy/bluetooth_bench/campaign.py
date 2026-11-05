#!/usr/bin/env python3
"""Run bounded, sequential trials on comma, retaining each result as JSON."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--ssh-host', required=True)
  parser.add_argument('--identity', type=Path, required=True)
  parser.add_argument('--peer', required=True)
  parser.add_argument('--channel', type=int, required=True)
  parser.add_argument('--out', type=Path, required=True)
  args = parser.parse_args()
  if not args.ssh_host.replace('@', '').replace('.', '').replace('-', '').isalnum():
    parser.error('Invalid SSH target')
  args.out.mkdir(parents=True, exist_ok=False)
  ssh = ['ssh', '-F', '/dev/null', '-i', str(args.identity), '-o', 'IdentitiesOnly=yes',
         '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6', '-o', 'ServerAliveInterval=5',
         '-o', 'ServerAliveCountMax=2', '-o', 'StrictHostKeyChecking=yes', args.ssh_host]
  import re
  if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.peer) or not 1 <= args.channel <= 30:
    parser.error('Invalid Bluetooth target')
  trials = [('full-size-180s', 180, 5, 2409, 1932),
            ('reduced-size-120s', 120, 5, 1200, 1000),
            ('reconnect-1', 5, 5, 1200, 1000),
            ('reconnect-2', 5, 5, 1200, 1000),
            ('reconnect-3', 5, 5, 1200, 1000)]
  for name, seconds, hz, tx, rx in trials:
    cmd = ['python3', '/tmp/client.py', '--peer', args.peer, '--channel', str(args.channel),
           '--seconds', str(seconds), '--hz', str(hz), '--tx', str(tx), '--rx', str(rx)]
    print(json.dumps({'starting': name}), flush=True)
    try:
      result = subprocess.run(ssh + cmd, capture_output=True, text=True, timeout=seconds + 30)
      try:
        record = json.loads(result.stdout)
      except ValueError:
        record = {'error': 'No complete JSON result', 'stdout': result.stdout, 'stderr': result.stderr}
      record.update(trial=name, exit_code=result.returncode)
      if result.returncode and not record.get('error'):
        record['error'] = f'SSH trial exited with code {result.returncode}'
    except subprocess.TimeoutExpired:
      record = {'trial': name, 'error': 'SSH trial deadline exceeded'}
    (args.out / (name + '.json')).write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)
    if record.get('error'):
      raise SystemExit(1)


if __name__ == '__main__':
  main()
