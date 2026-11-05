import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.jetlink import navdy_management as management


@pytest.fixture
def installed(tmp_path):
  files = {}
  for name in ('navdy_management.py', 'transport.py', 'bin/adb', 'sdk/usr/bin/adb'):
    path = tmp_path / name
    path.parent.mkdir(parents=True, exist_ok=True)
    for parent in (path.parent, *path.parent.parents):
      if parent == tmp_path:
        break
      parent.chmod(0o755)
    path.write_bytes(b'fixture')
    path.chmod(0o644)
    files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
  (tmp_path / 'manifest.json').write_text(json.dumps({'format': 1, 'files': files}))
  (tmp_path / 'manifest.json').chmod(0o644)
  return tmp_path


def test_installed_manifest(installed):
  assert management.verify(installed, os.getuid())['format'] == 1


def test_corruption_rejected(installed):
  (installed / 'transport.py').write_text('changed')
  with pytest.raises(ValueError, match='checksum'):
    management.verify(installed, os.getuid())


def test_writable_payload_rejected(installed):
  (installed / 'bin/adb').chmod(0o666)
  with pytest.raises(ValueError, match='permissions'):
    management.verify(installed, os.getuid())


def test_writable_parent_directory_rejected(installed):
  (installed / 'bin').chmod(0o775)
  with pytest.raises(ValueError, match='permissions'):
    management.verify(installed, os.getuid())


def test_escape_rejected(installed, tmp_path_factory):
  outside = tmp_path_factory.mktemp('other') / 'adb'
  outside.write_text('fixture')
  (installed / 'bin/adb').unlink()
  (installed / 'bin/adb').symlink_to(outside)
  with pytest.raises(ValueError, match='leaves'):
    management.verify(installed, os.getuid())


def test_path_traversal_rejected(installed):
  path = installed / 'manifest.json'
  manifest = json.loads(path.read_text())
  manifest['files']['../escape'] = 'a' * 64
  path.write_text(json.dumps(manifest))
  with pytest.raises(ValueError, match='Unsafe'):
    management.verify(installed, os.getuid())


def test_environment_does_not_use_ephemeral_home_or_public_listener():
  env = management.environment(Path('/data/test'), {'ADB_VENDOR_KEYS': '/tmp/foreign',
      'ADB_SERVER_SOCKET': 'tcp:0.0.0.0:5037', 'LD_PRELOAD': '/tmp/foreign', 'PYTHONPATH': '/tmp/foreign'})
  assert env['HOME'] == '/data/test/state'
  assert env['ADB_SERVER_SOCKET'] == 'tcp:localhost:5038'
  assert not {'ADB_VENDOR_KEYS', 'LD_PRELOAD', 'PYTHONPATH'} & env.keys()


def test_network_and_unauthorized_devices_rejected():
  output = 'navdy device usb:1-2 product:navdy\nphone unauthorized usb:1-3\n192.168.1.2:5555 device\n'
  assert management.usb_devices(output) == {'navdy': 'device', 'phone': 'unauthorized'}


@pytest.mark.parametrize('serial', ['192.168.0.1:5555', '', '../device', '-a'])
def test_serial_rejected_before_adb(serial):
  def fail(*args, **kwargs):
    raise AssertionError('ADB must not be called')
  with pytest.raises(ValueError):
    management.select_navdy(serial, command=fail)


@pytest.mark.parametrize('package, accepted', [('package:/system/priv-app/Hud/Hud.apk', True), ('', False)])
def test_verify_navdy_identity(package, accepted):
  calls = []
  def run(command, **kwargs):
    calls.append(command)
    assert command[1:3] == ['-P', '5038']
    assert kwargs['timeout'] == 5
    return SimpleNamespace(stdout='navdy device usb:1-2\n' if 'devices' in command else package)
  if accepted:
    assert management.select_navdy('navdy', command=run) == 'navdy'
  else:
    with pytest.raises(ValueError, match='not running'):
      management.select_navdy('navdy', command=run)
  assert len(calls) == 2


def test_device_missing_never_sends_shell():
  calls = []
  def run(command, **kwargs):
    calls.append(command)
    return SimpleNamespace(stdout='navdy unauthorized usb:1-2\n')
  with pytest.raises(ValueError, match='not authorized'):
    management.select_navdy('navdy', command=run)
  assert len(calls) == 1
