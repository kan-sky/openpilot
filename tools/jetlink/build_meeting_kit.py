"""Build a credential-free Mac field kit plus verified offline runtime bundle."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

from build_cluster_bundle import ROOT, build


def build_kit(output, root=ROOT):
  output.parent.mkdir(parents=True, exist_ok=True)
  bundle = output.parent / 'cluster-jetson-overlay.tar.gz'
  build(bundle, root)
  names = ('prepare_ssd.py', 'manage_jetson.py', 'maintenance_host.py', 'install_cluster.py',
           'usb_management.py', 'setup_comma_remote.py', 'REMOTE_ACCESS.md',
           'MEETING_CHECKLIST.md', 'MEETING-START.txt')
  entries = {name: (root / 'tools/jetlink' / name).read_bytes() for name in names}
  entries[bundle.name] = bundle.read_bytes()
  entries[bundle.name + '.sha256'] = bundle.with_suffix('.gz.sha256').read_bytes()
  files = {name: hashlib.sha256(data).hexdigest() for name, data in entries.items()}
  commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
  dirty = bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no']))
  entries['package.json'] = json.dumps(dict(format=1, source_commit=commit, tracked_source_dirty=dirty,
                                           field_validated=False, files=files), indent=2).encode()
  with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    for name, data in entries.items():
      archive.writestr(name, data)
  digest = hashlib.sha256(output.read_bytes()).hexdigest()
  output.with_suffix('.zip.sha256').write_text(digest + '\n')
  print(json.dumps(dict(kit=str(output), sha256=digest, files=len(entries), field_validated=False)))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('output', type=Path)
  build_kit(parser.parse_args().output)
