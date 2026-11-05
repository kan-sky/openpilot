"""Overlay an installed carrot-jetson runtime without changing its OS or engine."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import uuid

RUNTIME = Path('/opt/carrot-jetlink')
SERVICES = ('carrot-jetlink-hud.service', 'carrot-jetlink.service')
NAVDY_RELAY = 'openpilot/selfdrive/navdy/transport.py'


def atomic_bytes(path, data):
  temporary = path.with_name(path.name + '.hl-new')
  with temporary.open('wb') as f:
    f.write(data)
    f.flush()
    os.fsync(f.fileno())
  temporary.chmod(0o644)
  temporary.replace(path)


def release_target(runtime, name):
  path = runtime / name
  target = path.resolve(strict=True)
  if target.parent != (runtime / 'releases').resolve() or not target.is_dir():
    raise ValueError('Runtime target must be a release directory')
  return target


def extract_verified(bundle, destination, expected_sha256):
  if hashlib.sha256(bundle.read_bytes()).hexdigest() != expected_sha256:
    raise ValueError('Bundle SHA256 mismatch')
  with tarfile.open(bundle, 'r:gz') as archive:
    members = archive.getmembers()
    names = [m.name for m in members]
    if len(set(names)) != len(names) or len(names) > 3000 or sum(m.size for m in members) > 256 << 20:
      raise ValueError('Invalid bundle size/duplicate paths')
    for member in members:
      path = destination / member.name
      if (not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts
          or not path.resolve().is_relative_to(destination.resolve())):
        raise ValueError('Unsafe bundle path')
    manifest_file = archive.extractfile('HL_CLUSTER_MANIFEST.json')
    if manifest_file is None:
      raise ValueError('Missing cluster manifest')
    manifest = json.load(manifest_file)
    if (manifest.get('format') != 2 or manifest.get('branch') != 'cluster_jetson'
        or manifest.get('package_layout') != 'openpilot' or manifest.get('hud_capability') != 'carrot_hud_v1'):
      raise ValueError('Wrong runtime format')
    expected = manifest['files']
    if set(names) != set(expected) | {'HL_CLUSTER_MANIFEST.json'}:
      raise ValueError('Bundle file manifest mismatch')
    identity = hashlib.sha256(json.dumps(expected, sort_keys=True).encode()).hexdigest()
    if identity != manifest.get('content_sha256'):
      raise ValueError('Bundle content identity mismatch')
    for member in members:
      data = archive.extractfile(member).read()
      if member.name in expected and hashlib.sha256(data).hexdigest() != expected[member.name]:
        raise ValueError('Bundle member checksum mismatch')
      target = destination / member.name
      target.parent.mkdir(parents=True, exist_ok=True)
      target.write_bytes(data)
      target.chmod(0o644)
  return manifest


def check(runtime):
  current = release_target(runtime, 'current')
  for path in ('venv/bin/python', 'cache/last-loaded.json', 'updater/update_host.py'):
    if not (runtime / path).is_file():
      raise ValueError('Not a prepared carrot-jetson runtime: ' + path)
  for path in ('SOURCE_COMMIT', 'tools/jetlink/server.py', 'tools/jetlink/hud.py', 'tools/jetlink/probe_release.py',
               'openpilot/__init__.py', 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json'):
    if not (current / path).is_file():
      raise ValueError('Incomplete upstream runtime: ' + path)
  if (runtime / 'boot-update-required').exists():
    raise ValueError('Boot-gated image requires a separately validated installer; this overlay targets carrot-jetson R2 runtime')
  if (runtime / 'updates/transaction.json').exists():
    raise ValueError('Finish or recover the pending upstream update transaction first')
  return current


def check_navdy_prerequisites(candidate):
  """Check tooling only; never enumerate, pair or start a connected device."""
  if (candidate / NAVDY_RELAY).is_file() and shutil.which('adb') is None:
    raise ValueError('Navdy relay requires adb on the Jetson (install the adb package before this overlay)')


def switch(runtime, release):
  target = runtime / 'current.hl-next'
  target.unlink(missing_ok=True)
  target.symlink_to(release.relative_to(runtime))
  os.replace(target, runtime / 'current')


def prepare_release(runtime, previous, bundle, expected_sha256, probe):
  """Build and probe a new release without changing the selected runtime."""
  with tempfile.TemporaryDirectory(prefix='hl-stage-', dir=runtime / 'releases') as temporary:
    staging = Path(temporary)
    overlay = staging / 'overlay'
    overlay.mkdir()
    manifest = extract_verified(bundle, overlay, expected_sha256)
    candidate = staging / 'runtime'
    shutil.copytree(previous, candidate, symlinks=True, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    old_manifest = candidate / 'HL_CLUSTER_MANIFEST.json'
    if old_manifest.is_file():
      for name in set(json.loads(old_manifest.read_text())['files']) - set(manifest['files']):
        removed = candidate / name
        if not removed.resolve().is_relative_to(candidate.resolve()):
          raise ValueError('Previous manifest redirects outside release')
        removed.unlink(missing_ok=True)
    for source in overlay.rglob('*'):
      if source.is_file():
        dest = candidate / source.relative_to(overlay)
        if not dest.resolve().is_relative_to(candidate.resolve()):
          raise ValueError('Upstream path redirects outside release')
        dest.parent.mkdir(parents=True, exist_ok=True)
        # The root maintenance worker uses umask 077; jetlink must traverse
        # newly added code/asset directories in this immutable release.
        for parent in (dest.parent, *dest.parent.parents):
          if parent == candidate:
            break
          parent.chmod(0o755)
        shutil.copyfile(source, dest)
        dest.chmod(0o644)
    candidate.chmod(0o755)
    # runuser needs traversal through the TemporaryDirectory as well.
    staging.chmod(0o755)
    remembered = runtime / 'cache/last-loaded.json'
    original_model = remembered.read_bytes()
    try:
      probe(candidate)
    finally:
      if not remembered.exists() or remembered.read_bytes() != original_model:
        atomic_bytes(remembered, original_model)
    target = runtime / 'releases' / ('hl-' + manifest['content_sha256'][:20])
    if target.exists():
      # Rollback retains releases. Reinstall from the freshly probed candidate,
      # never reuse or overwrite a retained runtime that may have been modified.
      target = target.with_name(target.name + '-' + uuid.uuid4().hex)
    candidate.rename(target)
  return target, manifest


def install(runtime, bundle, expected_sha256, probe):
  previous = check(runtime)
  pin = runtime / 'HL_CLUSTER_PIN.json'
  if pin.exists():
    raise ValueError('A custom runtime is already installed; rollback before replacing it')
  updater = runtime / 'updater/update_host.py'
  backup = updater.with_name('update_host.hl-original.py')
  if backup.exists():
    raise ValueError('Previous updater backup exists; resolve it before installing')
  target, manifest = prepare_release(runtime, previous, bundle, expected_sha256, probe)
  atomic_bytes(backup, updater.read_bytes())
  atomic_bytes(updater, (target / 'tools/jetlink/update_guard.py').read_bytes())
  atomic_bytes(pin, json.dumps({'previous': str(previous.relative_to(runtime)),
                               'installed': str(target.relative_to(runtime)), 'manifest': manifest}).encode())
  if hasattr(os, 'sync'):
    os.sync()
  switch(runtime, target)
  if hasattr(os, 'sync'):
    os.sync()
  return target


def rollback(runtime):
  pin = runtime / 'HL_CLUSTER_PIN.json'
  record = json.loads(pin.read_text())
  previous = release_target(runtime, record['previous'])
  installed = release_target(runtime, record['installed'])
  if (runtime / 'current').resolve() not in (previous, installed):
    raise ValueError('Current release changed outside this installer; refusing rollback')
  backup = runtime / 'updater/update_host.hl-original.py'
  if not backup.is_file():
    raise ValueError('Original updater backup is missing')
  switch(runtime, previous)
  atomic_bytes(runtime / 'updater/update_host.py', backup.read_bytes())
  pin.unlink()
  backup.unlink()
  if hasattr(os, 'sync'):
    os.sync()
  return previous


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=('check', 'install', 'rollback'))
  parser.add_argument('--bundle', type=Path)
  parser.add_argument('--sha256')
  parser.add_argument('--confirm-parked', action='store_true')
  args = parser.parse_args()
  if not Path('/etc/carrot-jetlink-image.json').is_file():
    raise RuntimeError('This installer is for the prepared ajouatom/carrot-jetson image, not a generic JetPack host')
  current = check(RUNTIME)
  print('Current carrot-jetson runtime:', current)
  if args.action == 'check':
    return
  if os.geteuid() != 0 or not args.confirm_parked:
    parser.error('Use sudo and --confirm-parked with the vehicle parked and comma disconnected')
  if args.action == 'install' and (args.bundle is None or args.sha256 is None):
    parser.error('--bundle and --sha256 are required')
  marker = Path('/etc/carrot-jetlink-protected.json')
  if marker.exists() and json.loads(Path('/run/carrot-storage.json').read_text()).get('state') != 'protected':
    raise RuntimeError('DATA unavailable: never modify the immutable factory recovery image')
  # Share the original updater lock, including across the atomic current switch.
  import fcntl
  with Path('/run/carrot-jetlink-update.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    subprocess.run(['systemctl', 'stop', *SERVICES], check=True)
    try:
      def probe(candidate):
        check_navdy_prerequisites(candidate)
        python = str(RUNTIME / 'venv/bin/python')
        command = ['runuser', '-u', 'jetlink', '--', python]
        if (candidate / NAVDY_RELAY).is_file():
          subprocess.run([*command, str(candidate / NAVDY_RELAY), '--help'], check=True, timeout=15, cwd='/tmp')
        subprocess.run([*command, str(candidate / 'tools/jetlink/hud.py'), '--help'], check=True, timeout=60, cwd='/tmp')
        # The image's native probe loads its existing engine and executes test tensors.
        subprocess.run([*command, str(candidate / 'tools/jetlink/probe_release.py'), str(RUNTIME / 'cache')],
                       check=True, timeout=960, cwd='/tmp')
      result = (install(RUNTIME, args.bundle.resolve(), args.sha256, probe) if args.action == 'install' else rollback(RUNTIME))
      print('Selected:', result)
    finally:
      subprocess.run(['systemctl', 'start', *reversed(SERVICES)], check=True)


if __name__ == '__main__':
  main()
