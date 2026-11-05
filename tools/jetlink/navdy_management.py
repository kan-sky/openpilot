"""On-demand Navdy ADB/relay tools kept outside the protected inference runtime."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path('/run/carrot-data/hl-navdy')
PORT = 5038
PACKAGE = 'com.navdy.hud.app'
COMMANDS = {'shell', 'pull', 'push', 'install', 'uninstall', 'logcat', 'reboot', 'root', 'remount', 'get-state'}


def verify(root=ROOT, owner=0):
  """Check the private installation before loading code or native libraries."""
  if root.is_symlink() or not root.is_dir():
    raise ValueError('Navdy management installation is missing')
  manifest_path = root / 'manifest.json'
  for path in (root, manifest_path):
    info = path.lstat()
    if path.is_symlink() or info.st_uid != owner or info.st_mode & 0o022:
      raise ValueError('Untrusted installation ownership or permissions')
  if manifest_path.stat().st_size > 131072:
    raise ValueError('Oversized installation manifest')
  manifest = json.loads(manifest_path.read_text())
  files = manifest.get('files', {})
  required = {'navdy_management.py', 'transport.py', 'bin/adb', 'sdk/usr/bin/adb'}
  if manifest.get('format') != 1 or not required <= files.keys() or len(files) > 256:
    raise ValueError('Incomplete management manifest')
  for name, expected in files.items():
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts or not re.fullmatch('[0-9a-f]{64}', expected):
      raise ValueError('Unsafe manifest entry')
    path = root / relative
    target = path.resolve(strict=True)
    if not target.is_relative_to(root.resolve()) or not target.is_file():
      raise ValueError('Manifest path leaves the installation')
    for component in (path, *path.parents):
      if component == root:
        break
      info = component.stat()
      if info.st_uid != owner or info.st_mode & 0o022:
        raise ValueError('Untrusted installed file ownership or permissions')
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
      raise ValueError('Installed file checksum mismatch: ' + name)
  return manifest


def environment(root=ROOT, inherited=None):
  env = dict(os.environ if inherited is None else inherited)
  for key in ('ADB_VENDOR_KEYS', 'ADB_TRACE', 'LD_PRELOAD', 'PYTHONPATH', 'PYTHONHOME'):
    env.pop(key, None)
  env.update(HOME=str(root / 'state'), ANDROID_SDK_HOME=str(root / 'state'),
             ANDROID_USER_HOME=str(root / 'state/.android'),
             ADB_SERVER_SOCKET=f'tcp:localhost:{PORT}',
             LD_LIBRARY_PATH=str(root / 'sdk/usr/lib/aarch64-linux-gnu/android'),
             PATH=str(root / 'bin') + ':/usr/sbin:/usr/bin:/sbin:/bin',
             PYTHONDONTWRITEBYTECODE='1')
  return env


def native(args, root=ROOT):
  return [str(root / 'sdk/usr/bin/adb'), '-P', str(PORT), *args]


def usb_devices(output):
  found = {}
  for line in output.splitlines():
    fields = line.split()
    if (len(fields) >= 2 and re.fullmatch('[A-Za-z0-9_.-]{1,128}', fields[0])
        and any(field.startswith('usb:') for field in fields[2:])):
      found[fields[0]] = fields[1]
  return found


def select_navdy(serial, root=ROOT, command=subprocess.run):
  if not re.fullmatch('[A-Za-z0-9_.-]{1,128}', serial or '') or serial.startswith('-'):
    raise ValueError('Select a USB serial from the devices command, not an IP address')
  def read(args):
    return command(native(args, root), capture_output=True, text=True,
                   timeout=5, check=True, env=environment(root)).stdout
  if usb_devices(read(['devices', '-l'])).get(serial) != 'device':
    raise ValueError('Selected USB device is absent or not authorized')
  if not read(['-s', serial, 'shell', 'pm', 'path', PACKAGE]).strip().startswith('package:'):
    raise ValueError('Selected USB device is not running the Navdy HUD package')
  return serial


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  commands = parser.add_subparsers(dest='command', required=True)
  for name in ('status', 'version', 'devices', 'stop', 'relay'):
    commands.add_parser(name)
  adb = commands.add_parser('adb')
  adb.add_argument('--serial', required=True)
  adb.add_argument('arguments', nargs=argparse.REMAINDER)
  args = parser.parse_args(argv)
  verify()
  if os.geteuid() != 0:
    parser.error('Run through sudo; ADB USB access and its persistent key are private to root')
  if not os.path.ismount(ROOT.parent):
    parser.error('Protected DATA partition is not mounted')
  state = ROOT / 'state'
  info = state.stat()
  if state.is_symlink() or info.st_uid != 0 or info.st_mode & 0o077:
    parser.error('ADB identity directory is not private')
  if args.command == 'status':
    print(json.dumps({'installed': True, 'on_demand_only': True,
                      'adb_port': PORT, 'adb_listen': 'localhost',
                      'adb_key_present': (state / '.android/adbkey').is_file(),
                      'comma_paired': 'not_checked', 'navdy_connected': 'not_checked'}))
    return
  if args.command == 'relay':
    os.execve('/usr/bin/python3', ['python3', '-u', str(ROOT / 'transport.py'), '--stdio'], environment())
  if args.command == 'adb':
    if not args.arguments or args.arguments[0] not in COMMANDS:
      parser.error('Unsupported command; TCP ADB and network listener changes are not exposed')
    select_navdy(args.serial)
    command = native(['-s', args.serial, *args.arguments])
  else:
    command = native({'version': ['version'], 'devices': ['devices', '-l'], 'stop': ['kill-server']}[args.command])
  os.execve(command[0], command, environment())


if __name__ == '__main__':
  try:
    main()
  except (OSError, ValueError, subprocess.SubprocessError) as error:
    print('Navdy management: ' + str(error), file=sys.stderr)
    raise SystemExit(1) from error
