#!/usr/bin/env python3
"""Sequential raw/compressed A/B and fault trials on the verified radio pair."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--ssh-host', required=True)
  parser.add_argument('--identity', type=Path, required=True)
  parser.add_argument('--peer', required=True)
  parser.add_argument('--channel', type=int, required=True)
  parser.add_argument('--out', type=Path, required=True)
  args = parser.parse_args()
  if not re.fullmatch(r'[A-Za-z0-9_.@-]+', args.ssh_host) or args.ssh_host.startswith('-'):
    parser.error('Invalid SSH target')
  if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.peer) or not 1 <= args.channel <= 30:
    parser.error('Invalid paired Bluetooth target')
  args.out.mkdir(parents=True, exist_ok=False)
  ssh = ['ssh', '-F', '/dev/null', '-i', str(args.identity), '-o', 'IdentitiesOnly=yes',
         '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6', '-o', 'ServerAliveInterval=5',
         '-o', 'ServerAliveCountMax=2', '-o', 'StrictHostKeyChecking=yes', args.ssh_host]
  trials = [('raw-a', 60, ['--raw']), ('zlib-a', 120, []),
            ('raw-b', 60, ['--raw']), ('zlib-b', 60, []),
            ('sender-stall', 20, ['--stall-every', '7']),
            ('stale-injection', 20, ['--stale-every', '7'])]
  for name, seconds, extra in trials:
    print(json.dumps({'starting': name}), flush=True)
    cmd = ['python3', '/tmp/snapshot.py', '--peer', args.peer, '--channel', str(args.channel),
           '--fixture', '/tmp/Navdy-Bluetooth-Snapshot-20261019-fixture.json', '--seconds', str(seconds), *extra]
    try:
      result = subprocess.run(ssh + cmd, capture_output=True, text=True, timeout=seconds + 30)
      try:
        record = json.loads(result.stdout)
      except ValueError:
        record = {'error': 'Incomplete trial result', 'stdout': result.stdout, 'stderr': result.stderr}
      record.update(trial=name, exit_code=result.returncode)
      if result.returncode and not record.get('error'):
        record['error'] = f'Trial exit code {result.returncode}'
      if name == 'sender-stall' and not record.get('coalesced_unsent'):
        record['error'] = 'Stall did not exercise coalescing'
      if name == 'sender-stall' and not record.get('receiver_expired_clears'):
        record['error'] = 'Stall did not exercise receiver expiry'
      if name == 'stale-injection' and not record.get('stale_injected'):
        record['error'] = 'Stale injection was not exercised'
    except subprocess.TimeoutExpired:
      record = {'trial': name, 'error': 'SSH trial deadline exceeded'}
    (args.out / (name + '.json')).write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)
    if record.get('error'):
      raise SystemExit(1)
    time.sleep(1)


if __name__ == '__main__':
  main()
