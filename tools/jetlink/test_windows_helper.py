import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import zipfile

import pytest

import build_windows_helper as builder
import install_cluster

SPEC = importlib.util.spec_from_file_location('remote_helper', Path(__file__).parent / 'windows/remote_helper.py')
remote = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(remote)


@pytest.fixture(autouse=True)
def no_global_sync(monkeypatch):
  monkeypatch.setattr(install_cluster.os, 'fsync', lambda fd: None)


def test_bundle_has_no_keys_and_preserves_windows_encoding(tmp_path, monkeypatch):
  def fake_build(output, root):
    output.write_bytes(b'bundle fixture')
    output.with_suffix('.gz.sha256').write_text(hashlib.sha256(output.read_bytes()).hexdigest())
  monkeypatch.setattr(builder, 'build', fake_build)
  output = tmp_path / 'helper.zip'
  builder.build_windows(output)
  with zipfile.ZipFile(output) as archive:
    entries = {Path(name).name: archive.read(name) for name in archive.namelist()}
  assert set(entries) == {'Start.cmd', 'Core.ps1', 'Helper.ps1', 'README.txt', 'remote_helper.py', 'install_cluster.py',
                          'cluster-jetson-overlay.tar.gz', 'cluster-jetson-overlay.tar.gz.sha256', 'package.json'}
  for name, data in entries.items():
    if name.endswith(('.ps1', '.txt')):
      assert data.startswith(b'\xef\xbb\xbf') and b'\r\n' in data
      assert '�' not in data.decode('utf-8-sig')
  manifest = json.loads(entries['package.json'])
  assert all(hashlib.sha256(entries[name]).hexdigest() == digest for name, digest in manifest['files'].items())
  assert output.with_suffix('.zip.sha256').read_text().strip() == hashlib.sha256(output.read_bytes()).hexdigest()


@pytest.mark.parametrize('code,expected', [(0, 'succeeded'), (1, 'failed')])
def test_remote_job_success_and_failure(tmp_path, code, expected):
  calls = []
  def run(args, **kwargs):
    calls.append(args)
    if args[0] != 'systemctl':
      kwargs['stdout'].write('installer test log\n')
      return subprocess.CompletedProcess(args, code)
    return subprocess.CompletedProcess(args, 0)
  result = remote.run_job(tmp_path, 'install', 'a' * 64, runner=run)
  assert result['state'] == expected
  assert json.loads((tmp_path / 'result.json').read_text()) == result
  assert '--confirm-parked' in calls[0] and '--sha256' in calls[0]
  assert remote.status(tmp_path)['log'] == 'installer test log\n'
  assert len(calls) == (3 if code == 0 else 1)


def test_service_failure_is_not_success(tmp_path, monkeypatch):
  monkeypatch.setattr(remote.time, 'sleep', lambda seconds: None)
  def run(args, **kwargs):
    return subprocess.CompletedProcess(args, 1 if args[0] == 'systemctl' else 0)
  result = remote.run_job(tmp_path, 'rollback', '', runner=run)
  assert result['state'] == 'needs-attention' and not result['services_active']


def test_interrupted_worker_and_log_tail(tmp_path):
  remote.write_report(tmp_path, state='running')
  (tmp_path / 'install.log').write_text('x' * 100000)
  def run(args, **kwargs):
    return subprocess.CompletedProcess(args, 0, stdout='failed\n')
  result = remote.status(tmp_path, runner=run)
  assert result['state'] == 'unknown' and len(result['log']) == 24000


def test_completion_race_does_not_report_false_failure(tmp_path):
  remote.write_report(tmp_path, state='running')
  def run(args, **kwargs):
    remote.write_report(tmp_path, state='succeeded')
    return subprocess.CompletedProcess(args, 0, stdout='inactive\n')
  assert remote.status(tmp_path, runner=run)['state'] == 'succeeded'


def test_remote_job_exception_reported(tmp_path):
  def run(*args, **kwargs):
    raise OSError('test failure')
  result = remote.run_job(tmp_path, 'install', 'b' * 64, runner=run)
  assert result['state'] == 'needs-attention' and 'test failure' in result['error']


def test_dispatch_guard_and_detachment(tmp_path, monkeypatch):
  # Map only the job path operations to a temporary fixture, not a real /var/tmp job.
  directory = Path('/var/tmp/hl-cluster-' + 'a' * 32)
  original_exists = Path.exists
  original_read = Path.read_bytes
  monkeypatch.setattr(Path, 'exists', lambda self: False if self.parent == directory else original_exists(self))
  monkeypatch.setattr(Path, 'read_bytes', lambda self: b'bundle' if self.parent == directory else original_read(self))
  reports = []
  monkeypatch.setattr(remote, 'write_report', lambda directory, **fields: reports.append(fields))
  calls = []
  def run(args, **kwargs):
    assert reports[0]['state'] == 'starting'
    calls.append(args)
  result = remote.start(directory, 'install', hashlib.sha256(b'bundle').hexdigest(), runner=run)
  assert result['state'] == 'dispatched'
  assert calls[0][0] == 'systemd-run' and '--property=Type=exec' in calls[0]
  assert 'run' in calls[0] and '--confirm-parked' in calls[0]
  with pytest.raises(ValueError, match='checksum'):
    remote.start(directory, 'install', '0' * 64, runner=run)
  with pytest.raises(ValueError, match='directory'):
    remote.start(tmp_path, 'rollback', '', runner=run)
  monkeypatch.setattr(Path, 'exists', lambda self: self.parent == directory)
  with pytest.raises(ValueError, match='already'):
    remote.start(directory, 'rollback', '', runner=run)


def test_menu_safety_contract():
  script = (Path(__file__).parent / 'windows/Helper.ps1').read_text()
  assert script.index('Write-Utf8 $script:SessionPath') < script.index('remote_helper.py start')
  assert 'Format-Volume' not in script and 'Clear-Disk' not in script
  assert 'Upload-Package' in script and 'SETUP-RESULT.json' in script
