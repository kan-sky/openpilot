"""Prepare CARROTSETUP on macOS; never mount Linux DATA, format, or copy private keys."""
import argparse
import json
import os
from pathlib import Path
import plistlib
import re
import subprocess
import tempfile
import uuid


def validate_public_key(value):
  if not isinstance(value, str) or len(value) > 16384 or '\n' in value.strip() or '\r' in value:
    raise ValueError('Expected one SSH public key per line')
  parts = value.split()
  if len(parts) < 2 or parts[0] not in ('ssh-ed25519', 'ssh-rsa', 'ecdsa-sha2-nistp256'):
    raise ValueError('Only supported public keys are accepted, never private keys')
  key = ' '.join(parts[:2])
  with tempfile.NamedTemporaryFile(mode='w') as file:
    file.write(key + '\n')
    file.flush()
    subprocess.run(['ssh-keygen', '-l', '-f', file.name], capture_output=True, check=True, timeout=5)
  return key


def merge(existing, keys):
  value = json.loads(existing) if existing is not None else {}
  if not isinstance(value, dict) or set(value) - {'hostname', 'wifi', 'ssh_public_keys'}:
    raise ValueError('Unrecognized setup format; preserve it and inspect before editing')
  hostname = value.get('hostname')
  if hostname is not None and (not isinstance(hostname, str) or not re.fullmatch('[a-z]([a-z0-9-]{0,61}[a-z0-9])?', hostname)):
    raise ValueError('Existing hostname is invalid; no changes made')
  wifi = value.get('wifi')
  if wifi is not None:
    if (not isinstance(wifi, dict) or set(wifi) != {'ssid', 'password'}
        or not all(isinstance(wifi[k], str) for k in ('ssid', 'password'))
        or not 1 <= len(wifi['ssid'].encode()) <= 32 or not 8 <= len(wifi['password']) <= 63
        or any(c in wifi['ssid'] + wifi['password'] for c in '\x00\r\n')):
      raise ValueError('Existing Wi-Fi settings are invalid; no changes made')
  old = value.get('ssh_public_keys', [])
  if not isinstance(old, list):
    raise ValueError('Existing ssh_public_keys is not a list')
  combined = list(dict.fromkeys(validate_public_key(key) for key in [*old, *keys]))
  if not 1 <= len(combined) <= 16:
    raise ValueError('Keep between 1 and 16 public keys')
  value['ssh_public_keys'] = combined
  return value


def inspect_volume(volume):
  result = subprocess.run(['diskutil', 'info', '-plist', str(volume)], capture_output=True, check=True, timeout=10)
  info = plistlib.loads(result.stdout)
  if (info.get('VolumeName') != 'CARROTSETUP' or info.get('MountPoint') != str(volume)
      or info.get('FilesystemType') != 'msdos' or info.get('Writable') is False or info.get('Internal') is not False):
    raise ValueError('Expected the mounted, writable CARROTSETUP FAT partition; nothing was written')
  return {'device': info.get('DeviceIdentifier'), 'volume': str(volume), 'filesystem': info.get('FilesystemType')}


def prepare(volume, keys, write=False, complete=False):
  identity = inspect_volume(volume)
  path = volume / 'setup.json'
  if path.is_symlink():
    raise ValueError('setup.json must not be a symlink')
  original = path.read_bytes() if path.exists() else None
  value = merge(original, keys)
  summary = dict(**identity, public_keys=len(value['ssh_public_keys']),
                 preserves_wifi='wifi' in value, written=False)
  if not write:
    return summary
  if not complete:
    raise ValueError('Confirm all existing keys to retain are included; enrollment replaces authorized_keys')
  temporary = volume / ('hl-setup-' + uuid.uuid4().hex + '.tmp')
  try:
    with temporary.open('x') as stream:
      json.dump(value, stream, ensure_ascii=False, indent=2)
      stream.write('\n')
      stream.flush()
      os.fsync(stream.fileno())
    if (path.read_bytes() if path.exists() else None) != original:
      raise RuntimeError('setup.json changed during preparation; refusing overwrite')
    temporary.replace(path)
    if json.loads(path.read_bytes()) != value:
      raise RuntimeError('Read-back verification failed; do not eject until inspected')
    summary['written'] = True
  finally:
    temporary.unlink(missing_ok=True)
  return summary


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--volume', type=Path, default=Path('/Volumes/CARROTSETUP'))
  parser.add_argument('--public-key', action='append', type=Path, required=True)
  parser.add_argument('--write', action='store_true')
  parser.add_argument('--existing-keys-complete', action='store_true')
  args = parser.parse_args()
  keys = [path.read_text() for path in args.public_key]
  print(json.dumps(prepare(args.volume, keys, args.write, args.existing_keys_complete), ensure_ascii=False, indent=2))


if __name__ == '__main__':
  main()
