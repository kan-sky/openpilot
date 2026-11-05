"""Build the self-contained Windows menu helper; no Python install needed on PC."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from build_cluster_bundle import ROOT, build


def build_windows(output, root=ROOT):
  output = Path(output)
  output.parent.mkdir(parents=True, exist_ok=True)
  bundle = output.parent / 'cluster-jetson-overlay.tar.gz'
  build(bundle, root)
  windows = root / 'tools/jetlink/windows'
  entries = {}
  for name in ('Start.cmd', 'Core.ps1', 'Helper.ps1', 'README.txt', 'remote_helper.py'):
    data = (windows / name).read_bytes()
    if name.endswith(('.ps1', '.txt')):
      # Windows PowerShell 5.1 otherwise reads Korean text using the ANSI codepage.
      data = b'\xef\xbb\xbf' + data.decode('utf-8-sig').replace('\r\n', '\n').replace('\n', '\r\n').encode()
    elif name.endswith('.cmd'):
      data = data.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    entries[name] = data
  entries['install_cluster.py'] = (root / 'tools/jetlink/install_cluster.py').read_bytes()
  entries[bundle.name] = bundle.read_bytes()
  entries[bundle.name + '.sha256'] = bundle.with_suffix('.gz.sha256').read_bytes()
  hashes = {name: hashlib.sha256(data).hexdigest() for name, data in entries.items()}
  entries['package.json'] = json.dumps({'format': 1, 'files': hashes}, indent=2).encode()
  with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    for name, data in entries.items():
      archive.writestr('HL-Cluster-Windows/' + name, data)
  digest = hashlib.sha256(output.read_bytes()).hexdigest()
  output.with_suffix('.zip.sha256').write_text(digest + '\n')
  print(json.dumps({'windows_zip': str(output), 'sha256': digest, 'files': len(entries)}))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('output', type=Path)
  build_windows(parser.parse_args().output)
