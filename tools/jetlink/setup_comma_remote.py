"""Opt-in, persistent Tailscale installation for an AGNOS comma (not Jetson)."""
import argparse
import hashlib
import io
import os
from pathlib import Path
import platform
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = Path('/data/hl-comma-remote')
UNIT = 'hl-comma-tailscaled.service'
SOCKET = '/run/hl-comma-tailscale/tailscaled.sock'
VERSION = '1.104.1'
SHA256 = 'f60294374967f3dfd8cf57bbbd474d6cfce32d123e3a8ddeab82a87940daa806'
URL = f'https://pkgs.tailscale.com/stable/tailscale_{VERSION}_arm64.tgz'
MAX_DOWNLOAD = 128 << 20


def unit_text():
  return f'''[Unit]
Description=HL comma remote access (Tailscale)
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
User=root
ExecStart={DIRECTORY}/bin/tailscaled --state={DIRECTORY}/state/tailscaled.state --socket={SOCKET} --tun=hlts0
Restart=on-failure
RestartSec=10
RuntimeDirectory=hl-comma-tailscale
RuntimeDirectoryMode=0700
UMask=0077
Nice=10
CPUAffinity=0 1
'''


def trusted(path, directory=False):
  info = path.lstat()
  if (path.is_symlink() or info.st_uid != 0 or info.st_mode & 0o022
      or not (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode))):
    raise ValueError('Remote access path must be root-owned and not group/world-writable: ' + str(path))


def run(args):
  return subprocess.run(args, check=True, capture_output=True, text=True, timeout=30).stdout.strip()


def unpack(data, digest=SHA256):
  if not 0 < len(data) <= MAX_DOWNLOAD or hashlib.sha256(data).hexdigest() != digest:
    raise ValueError('Tailscale archive SHA256/size mismatch')
  result = {}
  with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
    for name in ('tailscale', 'tailscaled'):
      path = f'tailscale_{VERSION}_arm64/{name}'
      matches = [member for member in archive if member.name == path]
      if len(matches) != 1 or not matches[0].isfile() or not 0 < matches[0].size <= MAX_DOWNLOAD:
        raise ValueError('Invalid static binary in Tailscale archive')
      result[name] = archive.extractfile(matches[0]).read()
  return result


def ensure_service(command=run):
  trusted(DIRECTORY, directory=True)
  for name in ('bin', 'state'):
    trusted(DIRECTORY / name, directory=True)
  for name in ('tailscale', 'tailscaled'):
    trusted(DIRECTORY / 'bin' / name)
  unit = DIRECTORY / UNIT
  trusted(unit)
  if unit.read_text() != unit_text():
    raise ValueError('Unexpected remote access service definition')
  # AGNOS system files may be read-only; runtime registration is reconstructed
  # by the openpilot manager after each boot. Identity stays under /data.
  runtime = Path('/run/systemd/system') / UNIT
  if runtime.exists() or runtime.is_symlink():
    if not runtime.is_symlink() or runtime.resolve() != unit.resolve():
      raise ValueError('Existing systemd unit has a different owner')
    try:
      command(['systemctl', 'is-active', '--quiet', UNIT])
      return
    except subprocess.CalledProcessError:
      pass
  command(['systemctl', 'link', '--runtime', str(unit)])
  command(['systemctl', 'daemon-reload'])
  command(['systemctl', 'start', UNIT])
  command(['systemctl', 'is-active', '--quiet', UNIT])


def install():
  if not Path('/TICI').is_file() or not Path('/AGNOS').is_file() or platform.machine() not in ('aarch64', 'arm64'):
    raise ValueError('Install only on the intended AGNOS comma, not this Mac or Jetson')
  if not Path('/dev/net/tun').exists():
    raise ValueError('TUN device missing; do not change USB or fall back silently')
  if DIRECTORY.exists():
    raise ValueError('Remote access already prepared; preserving binaries and account identity')
  previous = subprocess.run(['systemctl', 'is-active', 'tailscaled.service'],
                            capture_output=True, text=True, timeout=10)
  if previous.returncode == 0 or Path('/run/tailscale/tailscaled.sock').exists():
    raise ValueError('Existing Tailscale installation found; reuse it instead')


def prepare_files(data, destination=DIRECTORY):
  binaries = unpack(data)
  if destination.exists() or destination.is_symlink():
    raise ValueError('Destination already exists; refusing to replace identity')
  with tempfile.TemporaryDirectory(prefix='.hl-remote-', dir=destination.parent) as temporary:
    stage = Path(temporary) / 'install'
    stage.mkdir(mode=0o755)
    (stage / 'bin').mkdir(mode=0o755)
    (stage / 'state').mkdir(mode=0o700)
    for name, binary in binaries.items():
      path = stage / 'bin' / name
      path.write_bytes(binary)
      path.chmod(0o755)
    (stage / UNIT).write_text(unit_text())
    (stage / UNIT).chmod(0o644)
    (stage / 'enabled').touch(mode=0o644)
    stage.rename(destination)


def client_args(*args):
  return [str(DIRECTORY / 'bin/tailscale'), '--socket=' + SOCKET, *args]


def login_args():
  # Use ordinary key-authenticated sshd over Tailscale so shared-node access
  # does not depend on a separate Tailscale SSH policy. No LAN subnet/exit node.
  return client_args('up', '--accept-dns=false', '--accept-routes=false', '--ssh=false',
                     '--advertise-routes=', '--exit-node=', '--advertise-exit-node=false')


def background_priority():
  # Called only by AGNOS CLI helpers. Their downloads/service checks are not
  # part of a driving deadline; children inherit this placement and priority.
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, {0, 1})
  os.nice(10)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=('install', 'start', 'login', 'status', 'disable'))
  parser.add_argument('--confirm-parked', action='store_true')
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('Run with sudo on the comma')
  background_priority()
  if args.action == 'install':
    if not args.confirm_parked:
      parser.error('Park and turn ignition off, then pass --confirm-parked')
    sys.path.insert(0, str(ROOT))
    from tools.jetlink.manage_jetson import Safety, require_safe
    safety = Safety()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not safety():
      time.sleep(0.2)
    require_safe(safety)
    install()
    with urllib.request.urlopen(URL, timeout=60) as response:
      data = response.read(MAX_DOWNLOAD + 1)
    require_safe(safety)
    prepare_files(data)
    ensure_service()
    print('Installed. Account enrollment still required: run the login action and open its URL.')
  elif args.action == 'start':
    if (DIRECTORY / 'enabled').is_file():
      ensure_service()
  elif args.action == 'login':
    ensure_service()
    os.execv(login_args()[0], login_args())
  elif args.action == 'status':
    os.execv(client_args()[0], client_args('status'))
  else:
    (DIRECTORY / 'enabled').unlink(missing_ok=True)
    run(['systemctl', 'stop', UNIT])
    print('Remote access stopped; identity retained. Jetlink is unchanged.')


if __name__ == '__main__':
  main()
