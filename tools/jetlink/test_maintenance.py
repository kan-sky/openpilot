import base64
import io
import json
import struct
import subprocess
import sys
import shutil
import tarfile
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

import install_cluster as installer
import maintenance_host as host
import manage_jetson as client
import prepare_ssd
from test_cluster_install import bundle_at, runtime_at


@pytest.fixture
def packaging(tmp_path, monkeypatch):
  from tools.jetlink import build_cluster_bundle
  source = {'branch': 'WIP-FR', 'commit': 'reviewed-commit', 'status': '', 'tracked': ''}
  built = []
  def run(args):
    assert args[:3] == ['git', '-C', str(client.ROOT)]
    operation = tuple(args[3:])
    field = {('branch', '--show-current'): 'branch', ('rev-parse', 'HEAD'): 'commit',
             ('status', '--porcelain', '--untracked-files=no'): 'status', ('ls-files', '-z'): 'tracked'}[operation]
    return source[field]
  def build(path):
    built.append(path)
    bundle, digest = bundle_at(tmp_path)
    with tarfile.open(bundle) as archive:
      manifest = json.load(archive.extractfile('HL_CLUSTER_MANIFEST.json'))
    source['tracked'] = '\0'.join(manifest['files']) + '\0'
    bundle.rename(path)
    path.with_suffix('.gz.sha256').write_text(digest + '\n')
    source.update(source.pop('after_build', {}))
  monkeypatch.setattr(client, 'run', run)
  monkeypatch.setattr(build_cluster_bundle, 'build', build)
  return source, built


@pytest.mark.parametrize('branch', ['WIP-FR', 'cluster_jetson'])
def test_sync_packages_only_reviewed_branches(tmp_path, packaging, branch):
  source, built = packaging
  source['branch'] = branch
  path, digest, content = client.build_bundle(tmp_path)
  assert built == [path] and path.exists() and len(digest) == len(content) == 64


@pytest.mark.parametrize('branch', ['', 'carrot-wip', 'codex/wip-fr-carrot-navdy-20261016', 'wip-fr', 'WIP-FR-unreviewed'])
def test_sync_rejects_detached_and_unapproved_branches_before_packaging(tmp_path, packaging, branch):
  source, built = packaging
  source['branch'] = branch
  with pytest.raises(ValueError, match='reviewed branches'):
    client.build_bundle(tmp_path)
  assert not built


@pytest.mark.parametrize('status', [' M tools/jetlink/server.py', 'M  tools/jetlink/server.py'])
def test_sync_rejects_uncommitted_sources_before_packaging(tmp_path, packaging, status):
  source, built = packaging
  source['status'] = status
  with pytest.raises(ValueError, match='Tracked working-tree changes'):
    client.build_bundle(tmp_path)
  assert not built


@pytest.mark.parametrize('change', [{'branch': 'cluster_jetson'}, {'branch': ''}, {'branch': 'other'},
                                  {'commit': 'different-commit'}, {'status': ' M tools/jetlink/server.py'}])
def test_sync_rejects_source_changes_during_packaging(tmp_path, packaging, change):
  source, built = packaging
  source['after_build'] = change
  with pytest.raises(ValueError, match='Source changed while packaging'):
    client.build_bundle(tmp_path)
  assert len(built) == 1


def test_sync_does_not_deploy_untracked_overlay_inputs(tmp_path, packaging):
  source, _ = packaging
  source['after_build'] = {'tracked': 'LICENSE\0'}
  with pytest.raises(ValueError, match='Untracked runtime inputs'):
    client.build_bundle(tmp_path)


def test_safety_uses_current_cereal_package_without_creating_real_ipc(monkeypatch):
  params, sm = object(), object()
  subscribe = Mock(return_value=sm)
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: params))
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', NS(messaging=NS(SubMaster=subscribe)))
  safety = client.Safety()
  assert safety.params is params and safety.sm is sm
  subscribe.assert_called_once_with(['deviceState'])


@pytest.mark.parametrize('paired', [False, True])
def test_navdy_relay_uses_exact_installed_path_and_pinned_ssh_only(tmp_path, monkeypatch, paired):
  from openpilot.selfdrive.navdy import bridge
  from tools.jetlink import manage_jetson
  state = tmp_path
  manage_jetson.private_json(state / 'config.json', dict(version=1, fingerprint=client.fingerprint(key())))
  (state / 'known_hosts').write_text(f'{client.ALIAS} {key(1 if paired else 2)}\n')
  process = NS(stdin=NS(fileno=lambda: 31), stdout=NS(fileno=lambda: 32))
  launch, blocking = Mock(return_value=process), Mock()
  monkeypatch.setattr(bridge.subprocess, 'Popen', launch)
  monkeypatch.setattr(bridge.os, 'set_blocking', blocking)
  if not paired:
    with pytest.raises(ValueError, match='Pinned host identity/config mismatch'):
      bridge.SshTransport('192.168.1.7', state)
    launch.assert_not_called()
    blocking.assert_not_called()
    return
  transport = bridge.SshTransport('192.168.1.7', state)
  expected = '/opt/carrot-jetlink/venv/bin/python -u /opt/carrot-jetlink/current/openpilot/selfdrive/navdy/transport.py --stdio'
  launch.assert_called_once_with(
    ['ssh', *manage_jetson.ssh_options(state), '-T', 'jetlink@192.168.1.7', expected],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
  assert transport.process is process
  assert [call.args for call in blocking.call_args_list] == [(31, False), (32, False)]


@pytest.mark.parametrize('changed,pending', [(False, False), (True, False), (False, True)])
@pytest.mark.parametrize('error', [ValueError, KeyError, TypeError])
def test_daemon_backs_off_failed_builds_but_not_new_commits_or_pending_jobs(tmp_path, monkeypatch, changed, pending, error):
  class Finished(BaseException):
    pass
  now, attempts = [100.], []
  monkeypatch.setattr(client, 'STATE', tmp_path)
  monkeypatch.setattr(client, 'Safety', lambda: lambda: True)
  monkeypatch.setattr(client, 'read_json', lambda *args: {'automatic': True})
  monkeypatch.setattr(client.time, 'monotonic', lambda: now[0])
  monkeypatch.setattr(client, 'run', lambda args: 'new' if changed and now[0] >= 140 else 'old')
  monkeypatch.setattr(client, 'save_status', lambda *args, **kwargs: None)
  if pending:
    (tmp_path / 'job.json').touch()
  def sync(*args):
    attempts.append(now[0])
    raise error('Invalid bundle or journal')
  def sleep(seconds):
    now[0] += seconds
    if now[0] > 250:
      raise Finished
  monkeypatch.setattr(client, 'sync', sync)
  monkeypatch.setattr(client.time, 'sleep', sleep)
  with pytest.raises(Finished):
    client.daemon()
  assert attempts == (list(range(130, 251, 10)) if pending else [130., 140., 170., 230.] if changed else [130., 160., 220.])


def key(byte=1):
  raw = struct.pack('>I', 11) + b'ssh-ed25519' + struct.pack('>I', 32) + bytes([byte]) * 32
  return 'ssh-ed25519 ' + base64.b64encode(raw).decode()


@pytest.fixture
def runtime(tmp_path, monkeypatch):
  monkeypatch.setattr(installer.os, 'sync', lambda: None)
  monkeypatch.setattr(host, 'boot_id', lambda: 'test-boot')
  monkeypatch.setattr(host, 'command', lambda *a, **k: None)
  monkeypatch.setattr(host, 'services', lambda *a, **k: None)
  root = runtime_at(tmp_path)
  usage = shutil.disk_usage(root)
  monkeypatch.setattr(host.shutil, 'disk_usage', lambda _: type(usage)(2 << 30, 0, 2 << 30))
  monkeypatch.setattr(installer, 'RUNTIME', root)
  return root


def job(tmp_path):
  directory = tmp_path / 'job'
  directory.mkdir()
  bundle, digest = bundle_at(directory)
  bundle.rename(directory / 'overlay.tar.gz')
  host.lease(directory, True)
  return directory, digest


def perform(directory, runtime, digest='', action='sync'):
  return host.apply(directory, runtime, action, digest, io.StringIO())


def test_initial_install_update_and_one_step_rollback(runtime, tmp_path):
  directory, digest = job(tmp_path)
  assert perform(directory, runtime, digest)['state'] == 'succeeded'
  first = (runtime / 'current').resolve()
  assert perform(directory, runtime, digest)['state'] == 'succeeded'
  second = (runtime / 'current').resolve()
  assert first != second and first.exists()
  assert perform(directory, runtime, action='rollback')['state'] == 'succeeded'
  assert (runtime / 'current').resolve() == first
  assert (runtime / 'HL_CLUSTER_PIN.json').exists()
  assert not (runtime / host.TRANSACTION).exists()
  installer.rollback(runtime)
  assert (runtime / 'current').resolve() == runtime / 'releases/original'


def test_probe_failure_restores_existing_custom_runtime(runtime, tmp_path, monkeypatch):
  directory, digest = job(tmp_path)
  perform(directory, runtime, digest)
  before = host.snapshot(runtime)
  def fail(args, *a, **k):
    if 'runuser' in args:
      raise RuntimeError('engine probe failed')
  monkeypatch.setattr(host, 'command', fail)
  result = perform(directory, runtime, digest)
  assert result['state'] == 'rolled-back' and 'engine probe failed' in result['error']
  assert host.snapshot(runtime) == before
  assert not (runtime / host.TRANSACTION).exists()


def test_service_failure_after_switch_restores_selection(runtime, tmp_path, monkeypatch):
  directory, digest = job(tmp_path)
  before = host.snapshot(runtime)
  calls = []
  def fail_once(*args, guarded=True):
    calls.append(guarded)
    if guarded:
      raise RuntimeError('new HUD crashed')
  monkeypatch.setattr(host, 'services', fail_once)
  assert perform(directory, runtime, digest)['state'] == 'rolled-back'
  assert host.snapshot(runtime) == before and calls == [True, False]


def test_recovery_failure_keeps_journal(runtime, tmp_path, monkeypatch):
  directory, digest = job(tmp_path)
  def fail(*a, **k):
    raise RuntimeError('service failure')
  monkeypatch.setattr(host, 'services', fail)
  result = perform(directory, runtime, digest)
  assert result['state'] == 'needs-attention'
  assert (runtime / host.TRANSACTION).exists()
  with pytest.raises(ValueError, match='recover'):
    perform(directory, runtime, digest)


def test_interrupted_switch_can_be_recovered(runtime, tmp_path):
  directory, digest = job(tmp_path)
  before = host.snapshot(runtime)
  host.write_json(runtime / host.TRANSACTION, before)
  installer.install(runtime, directory / 'overlay.tar.gz', digest, lambda p: None)
  assert (runtime / 'current').resolve() != runtime / before['selected']
  assert perform(directory, runtime, action='recover')['state'] == 'succeeded'
  assert host.snapshot(runtime) == before


@pytest.mark.parametrize('value', [dict(safe=False, received=100, boot='test-boot'),
                                    dict(safe=True, received=80, boot='test-boot'),
                                    dict(safe=True, received=101, boot='test-boot'),
                                    dict(safe=True, received=100, boot='previous-boot')])
def test_stale_onroad_future_and_prior_boot_leases_rejected(runtime, tmp_path, monkeypatch, value):
  directory, digest = job(tmp_path)
  host.write_json(directory / 'lease.json', value)
  monkeypatch.setattr(host.time, 'monotonic', lambda: 100)
  with pytest.raises(RuntimeError, match='lease'):
    perform(directory, runtime, digest)
  assert not (runtime / host.TRANSACTION).exists()


def test_lease_expiring_during_probe_restores_runtime(runtime, tmp_path, monkeypatch):
  directory, digest = job(tmp_path)
  before = host.snapshot(runtime)
  def revoke(args, *a, **k):
    if 'runuser' in args:
      host.lease(directory, False)
  monkeypatch.setattr(host, 'command', revoke)
  result = perform(directory, runtime, digest)
  assert result['state'] == 'rolled-back' and 'lease' in result['error']
  assert host.snapshot(runtime) == before


def test_bad_checksum_has_no_runtime_side_effects(runtime, tmp_path):
  directory, _ = job(tmp_path)
  before = host.snapshot(runtime)
  with pytest.raises(ValueError, match='checksum'):
    perform(directory, runtime, '0' * 64)
  assert before == host.snapshot(runtime) and not (runtime / host.TRANSACTION).exists()


def test_cancel_prevents_late_dispatch(runtime, tmp_path, monkeypatch):
  directory, _ = job(tmp_path)
  monkeypatch.setattr(host, 'preflight', dict)
  assert host.cancel(directory)['state'] == 'cancelled'
  host.lease(directory, True)
  with pytest.raises(ValueError, match='already dispatched'):
    host.dispatch(directory, 'sync', '')


def test_real_subprocess_is_reaped_after_lease_loss(tmp_path, monkeypatch):
  monkeypatch.setattr(host, 'boot_id', lambda: 'boot')
  directory = tmp_path
  host.lease(directory, True)
  program = "import pathlib,time;pathlib.Path('" + str(directory / 'lease.json') + "').write_text('{}');time.sleep(30)"
  with (tmp_path / 'log').open('w') as output:
    with pytest.raises(RuntimeError, match='lease'):
      host.command([sys.executable, '-c', program], directory, output, timeout=2)


@pytest.mark.parametrize('overrides', [dict(onroad=None), dict(offroad=None), dict(onroad=True), dict(offroad=False),
                                     dict(started=True), dict(age=3), dict(age=-1), dict(valid=False), dict(driver_view=True)])
def test_road_safety_fails_closed(overrides):
  args = dict(onroad=False, offroad=True, started=False, age=0.5, valid=True, driver_view=False)
  assert client.offroad_allowed(**args)
  args.update(overrides)
  assert not client.offroad_allowed(**args)


@pytest.mark.parametrize('on,off', [(False, True), (b'0', b'1'), ('0', '1')])
def test_typed_and_legacy_params(on, off):
  assert client.offroad_allowed(on, off, False, 0, True, False)


@pytest.mark.parametrize('ip', ['127.0.0.1', '0.0.0.0', '8.8.8.8', '169.254.1.1', 'host;touch /tmp/x', '-oProxyCommand=bad', '::1'])
def test_host_address_rejects_unsafe_or_public_destinations(ip):
  with pytest.raises(ValueError):
    client.address(ip)


def test_real_key_generation_is_idempotent_and_private(tmp_path):
  state = tmp_path / 'state'
  public = client.init_key(state)
  private = (state / 'id_ed25519').read_bytes()
  assert client.init_key(state) == public
  assert (state / 'id_ed25519').read_bytes() == private
  assert (state / 'id_ed25519').stat().st_mode & 0o777 == 0o600
  assert state.stat().st_mode & 0o777 == 0o700
  assert public.startswith('ssh-ed25519 ') and 'PRIVATE' not in public


def test_no_replacement_of_incomplete_key(tmp_path):
  (tmp_path / 'id_ed25519.pub').write_text(key())
  with pytest.raises(ValueError, match='Incomplete'):
    client.init_key(tmp_path)
  assert not (tmp_path / 'id_ed25519').exists()


def test_pair_requires_verified_fingerprint_and_real_account(tmp_path, monkeypatch):
  monkeypatch.setattr(client, 'init_key', lambda state: key())
  with pytest.raises(ValueError, match='fingerprint'):
    client.pair(tmp_path, '192.168.1.3', key(), 'wrong')
  assert not (tmp_path / 'config.json').exists()
  monkeypatch.setattr(client, 'ssh', lambda state, ip, args: 'jetlink' if args == ['id', '-un'] else '0')
  client.pair(tmp_path, '192.168.1.3', key(), client.fingerprint(key()))
  config = client.read_json(tmp_path / 'config.json')
  assert config['automatic'] is False
  with pytest.raises(ValueError, match='Already paired'):
    client.pair(tmp_path, '192.168.1.4', key(2), client.fingerprint(key(2)))


def test_changed_ip_uses_same_pinned_identity(tmp_path, monkeypatch):
  client.private_json(tmp_path / 'config.json', dict(version=1, host='192.168.1.2', fingerprint=client.fingerprint(key())))
  (tmp_path / 'known_hosts').write_text(client.ALIAS + ' ' + key())
  monkeypatch.setattr(client, 'discover', lambda: ['192.168.42.5'])
  monkeypatch.setattr(client, 'usb_candidate', list)
  attempts = []
  monkeypatch.setattr(client, 'ssh', lambda state, ip, args: attempts.append(ip))
  assert client.connect(tmp_path) == '192.168.42.5'
  assert attempts == ['192.168.42.5']
  options = client.ssh_options(tmp_path)
  assert 'StrictHostKeyChecking=yes' in options and 'HostKeyAlias=' + client.ALIAS in options
  assert 'BatchMode=yes' in options and 'ForwardAgent=no' in options


def test_host_key_mismatch_never_falls_back_to_auto_accept(tmp_path, monkeypatch):
  client.private_json(tmp_path / 'config.json', dict(version=1, host='192.168.1.2', fingerprint=client.fingerprint(key())))
  (tmp_path / 'known_hosts').write_text(client.ALIAS + ' ' + key(2))
  monkeypatch.setattr(client, 'ssh', lambda *a: pytest.fail('must not connect'))
  with pytest.raises(ValueError, match='identity/config'):
    client.connect(tmp_path)


def test_lost_start_reply_preserves_job_no_duplicate(tmp_path, monkeypatch):
  monkeypatch.setattr(client, 'connect', lambda state: '192.168.1.3')
  monkeypatch.setattr(client, 'build_bundle', lambda state: (tmp_path / 'bundle', 'digest', 'content'))
  monkeypatch.setattr(client, 'upload', lambda *a: None)
  monkeypatch.setattr(client, 'run', lambda *a, **k: '')
  calls = []
  def remote(state, ip, token, command, *args):
    calls.append(command)
    if command == 'check':
      return dict(recovery_required=False, content=None)
    if command == 'start':
      raise subprocess.TimeoutExpired('ssh', 10)
    if command == 'status':
      return dict(state='succeeded')
    return {}
  monkeypatch.setattr(client, 'remote', remote)
  with pytest.raises(subprocess.TimeoutExpired):
    client.sync(tmp_path, lambda: True)
  assert (tmp_path / 'job.json').exists()
  assert client.sync(tmp_path, lambda: True)['state'] == 'succeeded'
  assert calls.count('start') == 1 and not (tmp_path / 'job.json').exists()


def test_failed_content_not_automatically_retried(tmp_path, monkeypatch):
  monkeypatch.setattr(client, 'connect', lambda state: '192.168.1.3')
  monkeypatch.setattr(client, 'build_bundle', lambda state: (tmp_path / 'bundle', '', 'content'))
  client.private_json(tmp_path / 'last-result.json', dict(job=dict(content='content'), report=dict(state='rolled-back')))
  monkeypatch.setattr(client, 'upload', lambda *a: pytest.fail('must not retry'))
  assert client.sync(tmp_path, lambda: True)['already_attempted']


def test_onroad_refuses_any_connection(tmp_path, monkeypatch):
  monkeypatch.setattr(client, 'connect', lambda *a: pytest.fail('must not connect'))
  with pytest.raises(RuntimeError, match='offroad'):
    client.sync(tmp_path, lambda: False)


def test_ssd_dry_run_then_write_preserves_settings_and_keys(tmp_path, monkeypatch):
  monkeypatch.setattr(prepare_ssd, 'inspect_volume', lambda path: dict(device='disk-test', volume=str(path)))
  settings = dict(wifi=dict(ssid='retained', password='not-printed'), hostname='jetson', ssh_public_keys=[key()])
  (tmp_path / 'setup.json').write_text(json.dumps(settings))
  original = (tmp_path / 'setup.json').read_bytes()
  assert not prepare_ssd.prepare(tmp_path, [key(2)])['written']
  assert (tmp_path / 'setup.json').read_bytes() == original
  with pytest.raises(ValueError, match='Confirm'):
    prepare_ssd.prepare(tmp_path, [key(2)], write=True)
  report = prepare_ssd.prepare(tmp_path, [key(2)], write=True, complete=True)
  result = json.loads((tmp_path / 'setup.json').read_bytes())
  assert result['wifi'] == settings['wifi'] and result['hostname'] == 'jetson'
  assert result['ssh_public_keys'] == [key(), key(2)]
  assert 'not-printed' not in json.dumps(report) and report['written']


def test_ssd_rejects_private_key_and_unknown_configuration():
  with pytest.raises(ValueError):
    prepare_ssd.merge(None, ['-----BEGIN OPENSSH PRIVATE KEY-----'])
  with pytest.raises(ValueError, match='Unrecognized'):
    prepare_ssd.merge('{"unknown": 1}', [key()])


@pytest.mark.parametrize('value', [dict(hostname='BAD NAME'), dict(wifi=dict(ssid='', password='password')),
                                 dict(wifi=dict(ssid='valid', password='short')), dict(ssh_public_keys='bad')])
def test_ssd_invalid_existing_settings_not_overwritten(value):
  with pytest.raises(ValueError):
    prepare_ssd.merge(json.dumps(value), [key()])


def test_manager_entrypoint_is_callable():
  from openpilot.selfdrive.carrot.jetson_maintenance import main
  assert callable(main)


def test_service_checks_require_multiple_stable_samples(tmp_path, monkeypatch):
  calls = []
  monkeypatch.setattr(host.time, 'sleep', lambda seconds: None)
  monkeypatch.setattr(host, 'command', lambda args, *a, **k: calls.append(args))
  host.services(tmp_path, io.StringIO())
  assert len(calls) == 4 and calls[0][1] == 'start'
  assert all(call[1] == 'is-active' for call in calls[1:])


def test_model_cache_restored_when_probe_deletes_it(runtime, tmp_path):
  directory, digest = job(tmp_path)
  cache = runtime / 'cache/last-loaded.json'
  original = cache.read_bytes()
  def probe(candidate):
    cache.unlink()
    raise RuntimeError('bad probe')
  with pytest.raises(RuntimeError, match='bad probe'):
    installer.prepare_release(runtime, installer.check(runtime), directory / 'overlay.tar.gz', digest, probe)
  assert cache.read_bytes() == original


def test_cancel_during_client_lock_is_persisted(tmp_path):
  client.private_json(tmp_path / 'job.json', dict(token='a' * 32))
  with client.locked(tmp_path):
    assert client.cancel_pending(tmp_path)['state'] == 'cancel-requested'
  assert client.read_json(tmp_path / 'cancel.json')['token'] == 'a' * 32


def test_client_revokes_lease_on_onroad(tmp_path, monkeypatch):
  calls = []
  def remote(state, ip, token, command, *args):
    calls.append((command, args))
    return dict(state='rolled-back') if command == 'status' else {}
  monkeypatch.setattr(client, 'remote', remote)
  report = client.follow(tmp_path, '192.168.1.3', dict(token='a' * 32), lambda: False)
  assert calls[0] == ('lease', ('--safe', 'no')) and report['state'] == 'rolled-back'


def test_job_identity_and_successful_content_rechecked(tmp_path, monkeypatch):
  monkeypatch.setattr(client, 'connect', lambda state: '192.168.1.3')
  monkeypatch.setattr(client, 'build_bundle', lambda state: (tmp_path / 'bundle', '', 'content'))
  monkeypatch.setattr(client, 'upload', lambda *a: None)
  client.private_json(tmp_path / 'last-result.json', dict(job=dict(content='content'), report=dict(state='succeeded')))
  calls = []
  def remote(state, ip, token, command, *args):
    calls.append(command)
    return dict(content='content', recovery_required=False)
  monkeypatch.setattr(client, 'remote', remote)
  monkeypatch.setattr(client, 'run', lambda *a, **k: pytest.fail('unchanged bundle must not upload'))
  assert client.sync(tmp_path, lambda: True)['unchanged']
  assert calls == ['check']


def test_pending_action_cannot_be_mistaken_for_requested_rollback(tmp_path, monkeypatch):
  client.private_json(tmp_path / 'job.json', dict(action='sync', token='a' * 32))
  monkeypatch.setattr(client, 'connect', lambda state: '192.168.1.3')
  with pytest.raises(ValueError, match='Another action'):
    client.sync(tmp_path, lambda: True, action='rollback')


def test_full_client_host_flow_with_real_packages_and_simulated_ssh(runtime, tmp_path, monkeypatch):
  # Only hardware/network/process boundaries are replaced. The client journal,
  # remote command contract, checksum, installer, selection and rollback are real.
  state = tmp_path / 'comma'
  state.mkdir()
  jobs = tmp_path / 'jobs'
  jobs.mkdir()
  bundle, digest = bundle_at(state)
  calls = []
  monkeypatch.setattr(client, 'connect', lambda _: '192.168.1.3')
  monkeypatch.setattr(client, 'build_bundle', lambda _: (bundle, digest, 'content'))
  def upload(_, ip, token, bundle=None):
    (jobs / token).mkdir()
  def transfer(args, **kwargs):
    assert args[0] == 'scp'
    token = args[-1].split('/')[-2]
    shutil.copyfile(args[-2], jobs / token / 'overlay.tar.gz')
    return ''
  def remote(_, ip, token, command, *args):
    calls.append(command)
    directory = jobs / token
    values = dict(zip(args[::2], args[1::2], strict=True))
    if command == 'check':
      return dict(content=None, recovery_required=(runtime / host.TRANSACTION).exists())
    if command == 'lease':
      host.lease(directory, values['--safe'] == 'yes')
      return dict(ok=True)
    if command == 'start':
      result = perform(directory, runtime, values['--sha256'], values['--action'])
      host.write_json(directory / 'result.json', result)
      return dict(state='dispatched')
    if command == 'status':
      return host.status(directory)
    raise AssertionError(command)
  monkeypatch.setattr(client, 'upload', upload)
  monkeypatch.setattr(client, 'run', transfer)
  monkeypatch.setattr(client, 'remote', remote)
  assert client.sync(state, lambda: True)['state'] == 'succeeded'
  selected = (runtime / 'current').resolve()
  assert selected != runtime / 'releases/original'
  assert client.sync(state, lambda: True, action='rollback')['state'] == 'succeeded'
  assert (runtime / 'current').resolve() == runtime / 'releases/original'
  assert selected.exists() and calls.count('start') == 2
  assert not (state / 'job.json').exists()


def test_low_disk_blocks_mutation(runtime, tmp_path, monkeypatch):
  directory, digest = job(tmp_path)
  usage = shutil.disk_usage(runtime)
  monkeypatch.setattr(host.shutil, 'disk_usage', lambda _: type(usage)(usage.total, usage.used, 10))
  with pytest.raises(RuntimeError, match='512 MiB'):
    perform(directory, runtime, digest)
  assert not (runtime / host.TRANSACTION).exists()


def test_readonly_or_wrong_volume_rejected(tmp_path, monkeypatch):
  import plistlib
  for fields in (dict(VolumeName='wrong', Internal=False), dict(VolumeName='CARROTSETUP', Internal=True),
                 dict(VolumeName='CARROTSETUP', Internal=False, Writable=False)):
    info = dict(MountPoint=str(tmp_path), FilesystemType='msdos', **fields)
    data = plistlib.dumps(info)
    monkeypatch.setattr(prepare_ssd.subprocess, 'run',
                        lambda *a, data=data, **k: subprocess.CompletedProcess([], 0, stdout=data))
    with pytest.raises(ValueError, match='CARROTSETUP'):
      prepare_ssd.inspect_volume(tmp_path)


def test_interrupted_dispatch_is_fenced_before_clearing_client_journal(tmp_path, monkeypatch):
  calls = []
  def remote(state, ip, token, command, *args):
    calls.append(command)
    return dict(state='interrupted') if command in ('status', 'cancel') else {}
  monkeypatch.setattr(client, 'remote', remote)
  job = dict(token='a' * 32)
  client.private_json(tmp_path / 'job.json', job)
  assert client.follow(tmp_path, '192.168.1.3', job, lambda: True)['state'] == 'interrupted'
  assert calls == ['lease', 'status', 'cancel']
  assert client.read_json(tmp_path / 'cancel.json') == job
  assert not (tmp_path / 'job.json').exists()


def test_auto_requires_explicit_field_confirmation(tmp_path, monkeypatch):
  client.private_json(tmp_path / 'config.json', dict(automatic=False))
  client.private_json(tmp_path / 'last-result.json', dict(report=dict(state='succeeded')))
  monkeypatch.setattr(sys, 'argv', ['manage_jetson.py', '--state', str(tmp_path), 'auto', 'on'])
  with pytest.raises(ValueError, match='field verification'):
    client.main()
  assert client.read_json(tmp_path / 'config.json')['automatic'] is False


def test_manual_address_change_must_authenticate_same_host(tmp_path, monkeypatch):
  config = dict(version=1, host='192.168.1.2', fingerprint=client.fingerprint(key()), automatic=False)
  client.private_json(tmp_path / 'config.json', config)
  (tmp_path / 'known_hosts').write_text(client.ALIAS + ' ' + key())
  def fail(*args):
    raise RuntimeError('host key mismatch')
  monkeypatch.setattr(client, 'ssh', fail)
  with pytest.raises(RuntimeError, match='host key'):
    client.set_host(tmp_path, '192.168.42.5')
  assert client.read_json(tmp_path / 'config.json') == config
  monkeypatch.setattr(client, 'ssh', lambda *args: '')
  client.set_host(tmp_path, '192.168.42.5')
  assert client.read_json(tmp_path / 'config.json')['host'] == '192.168.42.5'


def test_recovery_success_alone_does_not_enable_automatic_update(tmp_path, monkeypatch):
  client.private_json(tmp_path / 'config.json', dict(automatic=False))
  client.private_json(tmp_path / 'last-result.json', dict(job=dict(action='recover'), report=dict(state='succeeded')))
  monkeypatch.setattr(sys, 'argv', ['manage_jetson.py', '--state', str(tmp_path), 'auto', 'on', '--confirm-field-checks'])
  with pytest.raises(ValueError, match='field verification'):
    client.main()
  assert client.read_json(tmp_path / 'config.json')['automatic'] is False
