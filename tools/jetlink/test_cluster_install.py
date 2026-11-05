import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

import install_cluster as installer


def bundle_at(tmp_path):
  guard = Path(__file__).with_name('update_guard.py').read_bytes()
  files = {'tools/jetlink/server.py': b'# custom', 'tools/jetlink/update_guard.py': guard,
           'openpilot/selfdrive/carrot/cluster/cluster_models.py': b'# models'}
  sums = {name: hashlib.sha256(data).hexdigest() for name, data in files.items()}
  identity = hashlib.sha256(json.dumps(sums, sort_keys=True).encode()).hexdigest()
  files['HL_CLUSTER_MANIFEST.json'] = json.dumps(dict(format=2, branch='cluster_jetson', files=sums,
                                                    package_layout='openpilot', hud_capability='carrot_hud_v1',
                                                    content_sha256=identity)).encode()
  bundle = tmp_path / 'bundle.tar.gz'
  with tarfile.open(bundle, 'w:gz') as archive:
    for name, data in files.items():
      member = tarfile.TarInfo(name)
      member.size = len(data)
      archive.addfile(member, io.BytesIO(data))
  return bundle, hashlib.sha256(bundle.read_bytes()).hexdigest()


def runtime_at(tmp_path):
  root = tmp_path / 'runtime'
  old = root / 'releases/original'
  old.mkdir(parents=True)
  for name in ('SOURCE_COMMIT', 'tools/jetlink/server.py', 'tools/jetlink/hud.py', 'tools/jetlink/probe_release.py',
               'openpilot/__init__.py', 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json'):
    path = old / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('original')
  for name in ('venv/bin/python', 'cache/last-loaded.json', 'updater/update_host.py'):
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('original')
  (root / 'current').symlink_to('releases/original')
  return root


def test_install_preserves_image_engine_and_original_then_rollback(tmp_path, monkeypatch):
  monkeypatch.setattr(installer.os, 'sync', lambda: None)
  runtime = runtime_at(tmp_path)
  bundle, digest = bundle_at(tmp_path)
  inspected = []
  target = installer.install(runtime, bundle, digest, lambda candidate: inspected.append(candidate))
  assert inspected and (runtime / 'current').resolve() == target
  assert (runtime / 'current/tools/jetlink/server.py').read_text() == '# custom'
  assert (runtime / 'releases/original/tools/jetlink/server.py').read_text() == 'original'
  assert (runtime / 'cache/last-loaded.json').read_text() == 'original'
  assert (runtime / 'HL_CLUSTER_PIN.json').exists()
  result = subprocess.run([sys.executable, str(runtime / 'updater/update_host.py'), 'activate'], capture_output=True, text=True)
  assert result.returncode == 0 and 'pinned' in result.stdout
  installer.rollback(runtime)
  assert (runtime / 'current').resolve() == runtime / 'releases/original'
  assert (runtime / 'updater/update_host.py').read_text() == 'original'
  assert not (runtime / 'HL_CLUSTER_PIN.json').exists()
  assert target.exists()


def test_failed_probe_leaves_selection_and_updater_intact(tmp_path, monkeypatch):
  runtime = runtime_at(tmp_path)
  bundle, digest = bundle_at(tmp_path)
  def fail(candidate):
    raise RuntimeError('engine failed')
  with pytest.raises(RuntimeError, match='engine failed'):
    installer.install(runtime, bundle, digest, fail)
  assert (runtime / 'current').resolve() == runtime / 'releases/original'
  assert (runtime / 'updater/update_host.py').read_text() == 'original'
  assert not (runtime / 'HL_CLUSTER_PIN.json').exists()


def test_restrictive_worker_umask_keeps_overlay_traversable(tmp_path, monkeypatch):
  monkeypatch.setattr(installer.os, 'sync', lambda: None)
  runtime = runtime_at(tmp_path)
  bundle, digest = bundle_at(tmp_path)
  def probe(candidate):
    for path in (candidate.parent, candidate, candidate / 'openpilot', candidate / 'openpilot/selfdrive',
                 candidate / 'openpilot/selfdrive/carrot', candidate / 'openpilot/selfdrive/carrot/cluster'):
      assert path.stat().st_mode & 0o777 == 0o755
    assert (candidate / 'openpilot/selfdrive/carrot/cluster/cluster_models.py').stat().st_mode & 0o777 == 0o644
  mask = os.umask(0o077)
  try:
    installer.install(runtime, bundle, digest, probe)
  finally:
    os.umask(mask)


def test_same_bundle_reinstall_after_rollback_preserves_retained_release(tmp_path, monkeypatch):
  monkeypatch.setattr(installer.os, 'sync', lambda: None)
  runtime = runtime_at(tmp_path)
  bundle, digest = bundle_at(tmp_path)
  probes = []
  first = installer.install(runtime, bundle, digest, probes.append)
  installer.rollback(runtime)
  (first / 'tools/jetlink/server.py').write_text('# retained local change')
  second = installer.install(runtime, bundle, digest, probes.append)
  assert len(probes) == 2
  assert first != second
  assert (runtime / 'current').resolve() == second
  assert (second / 'tools/jetlink/server.py').read_text() == '# custom'
  assert (first / 'tools/jetlink/server.py').read_text() == '# retained local change'
  installer.rollback(runtime)
  assert (runtime / 'current').resolve() == runtime / 'releases/original'
  assert (runtime / 'updater/update_host.py').read_text() == 'original'
  assert first.exists() and second.exists()


def test_wrong_digest_and_archive_traversal_rejected(tmp_path):
  bundle, _ = bundle_at(tmp_path)
  target = tmp_path / 'out'
  target.mkdir()
  with pytest.raises(ValueError, match='SHA256'):
    installer.extract_verified(bundle, target, '0' * 64)
  with tarfile.open(bundle, 'w:gz') as archive:
    member = tarfile.TarInfo('../escape')
    member.size = 1
    archive.addfile(member, io.BytesIO(b'x'))
  digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
  with pytest.raises(ValueError, match='Unsafe'):
    installer.extract_verified(bundle, target, digest)
  assert not (tmp_path / 'escape').exists()


def test_boot_gate_and_pending_transaction_require_explicit_recovery(tmp_path):
  runtime = runtime_at(tmp_path)
  marker = runtime / 'boot-update-required'
  marker.touch()
  with pytest.raises(ValueError, match='Boot-gated'):
    installer.check(runtime)
  marker.unlink()
  (runtime / 'updates').mkdir()
  (runtime / 'updates/transaction.json').write_text('{}')
  with pytest.raises(ValueError, match='transaction'):
    installer.check(runtime)
