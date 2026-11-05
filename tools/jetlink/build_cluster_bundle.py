"""Package only cluster runtime inputs, including hydrated fonts and icons."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
HOST_TOOLS = ('server.py', 'hud.py', 'hud_protocol.py', 'host_reader.py', 'host_usb.py', 'host_health.py',
              'display_owner.py', 'wifi_protocol.py', 'install_cluster.py', 'update_guard.py')


def source_files(root=ROOT):
  tracked = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z', '--',
    'openpilot/selfdrive/carrot/cluster', 'openpilot/selfdrive/navdy', 'openpilot/selfdrive/assets/fonts',
    'openpilot/selfdrive/assets/icons_mici']).decode().split('\0')
  paths = {name for name in tracked if name and ('/tests/' not in name) and not name.endswith(('.md', '.html'))}
  paths.update('tools/jetlink/' + name for name in HOST_TOOLS)
  paths.add('openpilot/selfdrive/navdy/transport.py')
  paths.update(p.relative_to(root).as_posix() for p in (root / 'third_party/jetlink').rglob('*')
               if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc')
  paths.add('LICENSE')
  return sorted(paths)


def build(output, root=ROOT):
  entries = {}
  for name in source_files(root):
    path = root / name
    if path.is_symlink() or not path.is_file():
      raise ValueError('Missing/non-regular runtime asset: ' + name)
    data = path.read_bytes()
    if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
      raise ValueError('Git LFS object not hydrated: ' + name)
    entries[name] = data
  files = {name: hashlib.sha256(data).hexdigest() for name, data in entries.items()}
  identity = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
  commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
  manifest = dict(format=2, branch='cluster_jetson', package_layout='openpilot', hud_capability='carrot_hud_v1',
                  source_commit=commit, content_sha256=identity, files=files)
  entries['HL_CLUSTER_MANIFEST.json'] = json.dumps(manifest, sort_keys=True).encode()
  output = Path(output)
  output.parent.mkdir(parents=True, exist_ok=True)
  with tarfile.open(output, 'w:gz') as archive:
    for name, data in entries.items():
      info = tarfile.TarInfo(name)
      info.size, info.mode = len(data), 0o644
      archive.addfile(info, io.BytesIO(data))
  digest = hashlib.sha256(output.read_bytes()).hexdigest()
  output.with_suffix(output.suffix + '.sha256').write_text(digest + '\n')
  print(json.dumps({'bundle': str(output), 'sha256': digest, 'content_sha256': identity, 'files': len(files)}))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('output', type=Path)
  build(parser.parse_args().output)
