import asyncio
import base64
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from tools.jetlink import remote_support as support

SPEC = importlib.util.spec_from_file_location('remote_support_api',
  Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/server/features/remote_support.py')
api = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(api)


def test_helper_loads_repo_dependencies_without_pythonpath(tmp_path):
  helpers = tmp_path / 'tools/jetlink'
  helpers.mkdir(parents=True)
  (tmp_path / 'pydeps').mkdir()
  (tmp_path / 'pydeps/serial.py').write_text('origin = "repo-local"\n')
  (helpers / 'manage_jetson.py').write_text('')
  (helpers / 'setup_comma_remote.py').write_text(
    'from pathlib import Path\nDIRECTORY = Path("/unused")\nSOCKET = "/unused/socket"\n')
  target = helpers / 'remote_support.py'
  shutil.copyfile(Path(support.__file__), target)
  code = ('import runpy, sys; runpy.run_path(sys.argv[1]); import serial; '
          'assert serial.origin == "repo-local"; '
          'assert sys.path[:2] == [sys.argv[2], sys.argv[3]]')
  subprocess.run([sys.executable, '-I', '-c', code, str(target),
                  str(tmp_path / 'pydeps'), str(tmp_path)], check=True, timeout=15)


def test_support_helpers_stay_off_driving_cores_at_background_priority(monkeypatch):
  calls = []
  monkeypatch.setattr(support.setup.os, 'sched_setscheduler', lambda *args: calls.append('normal'), raising=False)
  monkeypatch.setattr(support.setup.os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(support.setup.os, 'sched_param', lambda priority: priority, raising=False)
  monkeypatch.setattr(support.setup.os, 'sched_setaffinity', lambda pid, cpus: calls.append(cpus), raising=False)
  monkeypatch.setattr(support.setup.os, 'nice', lambda value: calls.append(value))
  support.setup.background_priority()
  assert calls == ['normal', {0, 1}, 10]


def key(n=1):
  raw = b'\0\0\0\x0bssh-ed25519\0\0\0\x20' + bytes([n]) * 32
  return 'ssh-ed25519 ' + base64.b64encode(raw).decode()


class Params:
  def __init__(self, **values):
    self.values = values

  def get(self, name):
    return self.values.get(name, '')

  def get_bool(self, name):
    return self.values.get(name, False) is True

  def get_int(self, name):
    return int(self.values.get(name, 0))

  def put(self, name, value):
    self.values[name] = value

  put_bool = put
  put_int = put


def test_key_preview_rejects_private_key_and_options():
  for value in ('-----BEGIN OPENSSH PRIVATE KEY-----', 'command="bad" ' + key(), key() + '\n' + key(2)):
    with pytest.raises(ValueError):
      support.preview_key({'public_key': value})


def test_key_grant_is_explicit_and_preserves_owner(monkeypatch):
  p = Params(GithubSshKeys=key() + ' owner\n')
  monkeypatch.setattr(support, 'params', lambda: p)
  preview = support.preview_key({'public_key': key(2)})
  with pytest.raises(ValueError):
    support.change_key('add-key', {'public_key': key(2)})
  support.change_key('add-key', preview)
  assert p.get('GithubSshKeys').startswith(key() + ' owner\n')
  assert p.get_bool('SshEnabled')
  assert support.support_keys(support.key_lines(p.get('GithubSshKeys'))) == [{'fingerprint': preview['fingerprint']}]
  support.change_key('remove-key', preview)
  assert p.get('GithubSshKeys') == key() + ' owner\n'


def test_existing_key_never_becomes_removable_support_key(monkeypatch):
  p = Params(GithubSshKeys=key() + ' owner\n')
  monkeypatch.setattr(support, 'params', lambda: p)
  preview = support.preview_key({'public_key': key()})
  assert support.change_key('add-key', preview)['already_authorized']
  assert p.get_bool('SshEnabled')
  with pytest.raises(ValueError):
    support.change_key('remove-key', preview)
  assert p.get('GithubSshKeys') == key() + ' owner\n'


@pytest.mark.parametrize('value', ['http://login.tailscale.com/a/abc', 'https://evil.test/a/abc',
  'https://login.tailscale.com.evil.test/a/abc', 'https://evil@login.tailscale.com/a/abc',
  'https://login.tailscale.com:bad/a/abc', 'javascript:alert(1)', None])
def test_auth_url_restricted(value):
  assert support.auth_url(value) == ''


def test_status_returns_only_own_account(monkeypatch):
  data = {'BackendState': 'Running', 'Self': {'Online': True, 'HostName': 'comma', 'UserID': 1,
          'TailscaleIPs': ['100.1.2.3']}, 'User': {'1': {'LoginName': 'owner@example.test'},
          '2': {'LoginName': 'other@example.test'}}, 'Peer': {'secret': 'never expose'}}
  monkeypatch.setattr(support, 'client', lambda *a: (0, json.dumps(data), ''))
  result = support.network_status(('wayon', None, None))
  assert result['account'] == 'owner@example.test' and result['online']
  assert 'other@example.test' not in json.dumps(result) and 'secret' not in json.dumps(result)


def test_connected_login_never_logs_out(monkeypatch):
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: {'backend': 'Running', 'account': 'owner'})
  monkeypatch.setattr(support, 'client', lambda *a, **k: pytest.fail('Must not change an existing account'))
  assert support.login()['changed'] is False


def test_stopped_cold_boot_uses_only_selected_saved_account(monkeypatch):
  calls = []
  def client(entry, *args):
    calls.append(args)
    if args == ('status', '--json'):
      return 0, json.dumps({'BackendState': 'Stopped', 'Self': {'TailscaleIPs': None}}), ''
    return 0, json.dumps([{'account': 'other-owner', 'selected': False},
                          {'account': 'saved-owner', 'selected': True}]), ''
  monkeypatch.setattr(support, 'client', client)
  result = support.network_status(('wayon', None, None))
  assert result['account'] == 'saved-owner' and result['addresses'] == []
  assert not result['online'] and 'other-owner' not in json.dumps(result)
  assert calls == [('status', '--json'), ('switch', '--list', '--json')]


@pytest.mark.parametrize('code,profiles', [(1, ''), (0, 'invalid'), (0, '{}'), (0, '[null]')])
def test_stopped_profile_query_failure_keeps_status_readable(monkeypatch, code, profiles):
  def client(entry, *args):
    if args == ('status', '--json'):
      return 0, '{"BackendState":"Stopped"}', ''
    return code, profiles, ''
  monkeypatch.setattr(support, 'client', client)
  result = support.network_status(('system', None, None))
  assert result['backend'] == 'Stopped' and not result['account']


def test_account_switch_requires_exact_current_account(monkeypatch):
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: {'backend': 'Running', 'account': 'owner'})
  monkeypatch.setattr(support, 'client', lambda *a, **k: pytest.fail('No logout on stale confirmation'))
  with pytest.raises(ValueError):
    support.login(True, 'old-owner')


def test_account_switch_logs_out_before_new_login(monkeypatch):
  calls = []
  states = iter([{'backend': 'Running', 'account': 'owner'}, {'backend': 'NeedsLogin', 'auth_url': ''}])
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: next(states))

  def client(entry, *args, **kwargs):
    calls.append(args)
    if args == ('debug', 'prefs'):
      return 0, '{"NetfilterMode":0}', ''
    return (0, '', '') if args == ('logout',) else (1, '{"AuthURL":"https://login.tailscale.com/a/test-only"}\n{}', '')
  monkeypatch.setattr(support, 'client', client)
  result = support.login(True, 'owner')
  assert calls[:2] == [('debug', 'prefs'), ('logout',)] and calls[2][0] == 'up'
  assert '--accept-dns=false' in calls[2] and '--ssh=false' in calls[2]
  assert '--accept-routes=false' in calls[2] and '--netfilter-mode=off' in calls[2]
  assert result['network']['auth_url'].endswith('/test-only')


@pytest.mark.parametrize('mode,label', [(0, 'off'), (1, 'nodivert'), (2, 'on')])
def test_wayon_login_preserves_existing_firewall_mode(monkeypatch, mode, label):
  calls = []
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda entry: {'backend': 'NeedsLogin', 'auth_url': ''})
  def client(entry, *args, **kwargs):
    calls.append(args)
    return (0, json.dumps({'NetfilterMode': mode}), '') if args == ('debug', 'prefs') else (0, '{}', '')
  monkeypatch.setattr(support, 'client', client)
  support.login()
  assert calls[0] == ('debug', 'prefs')
  assert '--netfilter-mode=' + label in calls[1]


@pytest.mark.parametrize('code,raw', [(1, '{}'), (0, '{}'), (0, 'invalid'), (0, 'null'),
                                    (0, '{"NetfilterMode":true}'), (0, '{"NetfilterMode":3}')])
def test_wayon_firewall_read_failure_never_logs_out(monkeypatch, code, raw):
  calls = []
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda entry: {'backend': 'Running', 'account': 'owner'})
  monkeypatch.setattr(support, 'client', lambda entry, *args: (calls.append(args) or code, raw, ''))
  with pytest.raises(RuntimeError, match='account unchanged'):
    support.login(True, 'owner')
  assert calls == [('debug', 'prefs')]


def test_hl_login_does_not_disable_firewall():
  assert not any('--netfilter-mode=' in arg for arg in support.setup.login_args())


def test_existing_provider_install_is_noop(monkeypatch):
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support.setup, 'install', lambda: pytest.fail('Must preserve existing installation'))
  assert support.install() == {'changed': False}


@pytest.mark.parametrize('enabled', [False, True])
def test_network_toggle_preserves_account_and_preferences(monkeypatch, enabled):
  before, after = ('Stopped', 'Running') if enabled else ('Running', 'Stopped')
  states = iter([{'backend': before, 'account': 'owner'}, {'backend': after, 'account': 'owner'}])
  calls = []
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: next(states))
  monkeypatch.setattr(support, 'client', lambda e, *a, **k: (calls.append(a) or 0, '', ''))
  monkeypatch.setattr(support, 'params', lambda: pytest.fail('Do not change Params or Jetson settings'))
  monkeypatch.setattr(support.setup, 'ensure_service', lambda: pytest.fail('Do not replace or restart a daemon'))
  result = support.set_network(enabled)
  assert calls == [('up',) if enabled else ('down',)]
  assert result == {'changed': True, 'network': {'backend': after, 'account': 'owner'}}


@pytest.mark.parametrize('enabled,backend', [(False, 'Stopped'), (True, 'Running')])
def test_network_toggle_idempotent(monkeypatch, enabled, backend):
  monkeypatch.setattr(support, 'provider', lambda: ('hl', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: {'backend': backend})
  monkeypatch.setattr(support, 'client', lambda *a, **k: pytest.fail('Already in requested state'))
  assert support.set_network(enabled)['changed'] is False


@pytest.mark.parametrize('backend', ['NeedsLogin', 'NoState', 'Unavailable', 'Unknown'])
def test_network_resume_never_installs_or_reauthenticates(monkeypatch, backend):
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: {'backend': backend})
  monkeypatch.setattr(support, 'client', lambda *a, **k: pytest.fail('No logout or implicit login'))
  with pytest.raises((ValueError, RuntimeError)):
    support.set_network(True)


def test_network_toggle_requires_installation(monkeypatch):
  monkeypatch.setattr(support, 'provider', lambda: None)
  with pytest.raises(ValueError, match='Install'):
    support.set_network(True)


@pytest.mark.parametrize('enabled,code,backend', [(False, 1, 'Running'), (True, 1, 'Stopped'),
  (True, 0, 'Starting'), (False, 0, 'Running')])
def test_network_toggle_does_not_report_failure_as_success(monkeypatch, enabled, code, backend):
  states = iter([{'backend': 'Stopped' if enabled else 'Running'}, {'backend': backend}])
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: next(states))
  monkeypatch.setattr(support, 'client', lambda *a, **k: (code, 'private output', 'private error'))
  with pytest.raises(RuntimeError, match='requested state') as error:
    support.set_network(enabled)
  assert 'private' not in str(error.value)


@pytest.mark.parametrize('backend', ['Running', 'Starting'])
def test_network_resume_timeout_uses_actual_daemon_state(monkeypatch, backend):
  states = iter([{'backend': 'Stopped'}, {'backend': backend}])
  monkeypatch.setattr(support, 'provider', lambda: ('wayon', None, None))
  monkeypatch.setattr(support, 'network_status', lambda e: next(states))
  def client(entry, *args, **kwargs):
    assert args == ('up',) and kwargs == {'timeout': 10}
    raise subprocess.TimeoutExpired(['tailscale', 'up'], 10)
  monkeypatch.setattr(support, 'client', client)
  if backend == 'Running':
    assert support.set_network(True)['network']['backend'] == 'Running'
  else:
    with pytest.raises(RuntimeError, match='requested state'):
      support.set_network(True)


@pytest.mark.parametrize('action', ['network-on', 'network-off'])
def test_onroad_network_toggle_only_dispatches_fixed_action(monkeypatch, tmp_path, action):
  calls = []
  monkeypatch.setattr(support, 'supported', lambda: True)
  monkeypatch.setattr(support, 'STATE', tmp_path)
  monkeypatch.setattr(support.setup, 'trusted', lambda *a, **k: None)
  monkeypatch.setattr(support, 'params', lambda: Params(IsOnroad=True))
  monkeypatch.setattr(support.management, 'Safety', lambda: pytest.fail('No vehicle-mode gating or mutation'))
  monkeypatch.setattr(support, 'set_network', lambda enabled: calls.append(enabled))
  support.execute(action, {})
  assert calls == [action == 'network-on']


def test_onroad_does_not_gate_remote_login(monkeypatch, tmp_path):
  monkeypatch.setattr(support, 'supported', lambda: True)
  monkeypatch.setattr(support, 'STATE', tmp_path)
  monkeypatch.setattr(support.setup, 'trusted', lambda *a, **k: None)
  monkeypatch.setattr(support, 'params', lambda: Params(IsOnroad=True))
  monkeypatch.setattr(support, 'login', lambda *a: {'changed': True})
  monkeypatch.setattr(support.management, 'Safety', lambda: pytest.fail('No road-mode mutation or gating'))
  assert support.execute('login', {})['changed']


def test_jetson_check_uses_pinned_usb_first(monkeypatch):
  monkeypatch.setattr(support.management, 'paired_config', lambda s: {})
  monkeypatch.setattr(support.management, 'discover', list)
  monkeypatch.setattr(support.management, 'candidates', lambda *a: ['169.254.77.1', '192.168.0.21'])
  calls = []
  def ssh(state, host, cmd):
    calls.append(host)
    return 'jetson' if cmd == ['hostname'] else 'jetlink'
  monkeypatch.setattr(support.management, 'ssh', ssh)
  assert support.jetson_check()['transport'] == 'USB'
  assert calls == ['169.254.77.1', '169.254.77.1']


def request(peer='192.168.0.2', host='192.168.0.10:7000', **headers):
  return SimpleNamespace(remote=peer, host=host, scheme='http', headers=headers,
    transport=SimpleNamespace(get_extra_info=lambda key: ('192.168.0.10', 7000)))


@pytest.mark.parametrize('req', [request(peer='100.79.53.117'), request(peer='8.8.8.8'),
  request(host='evil.test:7000'), request(**{'Sec-Fetch-Site': 'cross-site'}),
  request(**{'X-Forwarded-For': '127.0.0.1'}), request(Origin='http://evil.test')])
def test_requests_reject_remote_and_cross_origin(req):
  with pytest.raises(web.HTTPForbidden):
    api.validate_request(req)


def test_post_requires_origin():
  with pytest.raises(web.HTTPForbidden):
    api.validate_request(request(), mutate=True)
  api.validate_request(request(Origin='http://192.168.0.10:7000'), mutate=True)


def test_real_http_csrf_actions_and_busy_guard():
  async def scenario():
    entered, release = asyncio.Event(), asyncio.Event()
    calls = []
    async def runner(action, body):
      calls.append(action)
      if action == 'status':
        return {'ok': True, 'supported': True, 'network': {'backend': 'Running'}}
      entered.set()
      await release.wait()
      return {'ok': True}
    app = web.Application()
    api.register(app)
    app[api.STATE_KEY] = api.SupportState(runner)
    async with TestClient(TestServer(app)) as client:
      result = await client.get('/api/remote_support')
      value = await result.json()
      assert result.headers['Cache-Control'] == 'no-store'
      origin = str(client.make_url('/')).rstrip('/')
      assert (await client.post('/api/remote_support', json={'action': 'login'}, headers={'Origin': origin})).status == 403
      headers = {'Origin': origin, 'X-HL-CSRF': value['csrf']}
      assert (await client.post('/api/remote_support', json={'action': 'login'}, headers=headers)).status == 400
      assert (await client.post('/api/remote_support', json={'action': 'shell', 'confirmed': True}, headers=headers)).status == 400
      assert (await client.post('/api/remote_support', json={'action': 'login', 'confirmed': True}, headers=headers)).status == 202
      await entered.wait()
      assert (await client.post('/api/remote_support', json={'action': 'login', 'confirmed': True}, headers=headers)).status == 409
      release.set()
      await app[api.STATE_KEY].task
      assert app[api.STATE_KEY].operation['state'] == 'done'
      assert calls.count('login') == 1
  asyncio.run(scenario())


def test_chunked_json_request():
  async def scenario():
    async def runner(action, body):
      return {'ok': True, 'fingerprint': 'test'}
    app = web.Application()
    api.register(app)
    app[api.STATE_KEY] = api.SupportState(runner)
    async with TestClient(TestServer(app)) as client:
      async def chunks():
        for chunk in (b'{"action":', b'"key-preview",', b'"public_key":"x"}'):
          yield chunk
          await asyncio.sleep(0.01)
      headers = {'Origin': str(client.make_url('/')).rstrip('/'), 'X-HL-CSRF': app[api.STATE_KEY].csrf, 'Content-Type': 'application/json'}
      result = await client.post('/api/remote_support', data=chunks(), headers=headers)
      assert result.status == 200 and (await result.json())['fingerprint'] == 'test'
  asyncio.run(scenario())


@pytest.mark.parametrize('action', ['network-on', 'network-off'])
def test_network_http_confirmation_and_login_cache(action):
  async def scenario():
    calls = []
    async def runner(name, body):
      calls.append(name)
      return {'ok': True, 'network': {'backend': 'Stopped' if name == 'network-off' else 'Running', 'auth_url': ''}}
    app = web.Application()
    api.register(app)
    state = app[api.STATE_KEY] = api.SupportState(runner)
    state.login_url, state.login_deadline = 'https://login.tailscale.com/a/old', 1000.
    async with TestClient(TestServer(app)) as client:
      headers = {'Origin': str(client.make_url('/')).rstrip('/'), 'X-HL-CSRF': state.csrf}
      assert (await client.post('/api/remote_support', json={'action': action}, headers=headers)).status == 400
      assert not calls
      assert (await client.post('/api/remote_support', json={'action': action, 'confirmed': True}, headers=headers)).status == 202
      await state.task
      assert calls == [action] and state.operation['state'] == 'done'
      assert 'auth_url' not in state.operation['result']['network']
      if action == 'network-off':
        assert state.login_url == '' and state.login_deadline == 0.
  asyncio.run(scenario())


@pytest.mark.parametrize('values', [None, {}, {'ClusterHud': True}, {'ClusterHud': 3},
  {'ClusterHudTheme': -1}, {'usb_management': 1}, {'IsOnroad': True}, {'Model': 'other'}])
def test_jetson_settings_validate_before_writing(monkeypatch, values):
  monkeypatch.setattr(support, 'params', lambda: pytest.fail('Invalid settings must not reach Params'))
  with pytest.raises(ValueError):
    support.jetson_settings({'settings': values})


def test_jetson_settings_no_live_usb_or_model_restart(monkeypatch, tmp_path):
  p = Params(IsOnroad=True, ClusterHud=0, ClusterHudTheme=0)
  monkeypatch.setattr(support, 'params', lambda: p)
  monkeypatch.setattr(support.management, 'STATE', tmp_path)
  monkeypatch.setattr(support, 'init_management_key', lambda: None)
  monkeypatch.setattr(support, 'command', lambda *a: pytest.fail('Never reset live USB or inference'))
  result = support.jetson_settings({'settings': {'ClusterHud': 2, 'ClusterHudTheme': 1, 'usb_management': True}})
  assert p.get_int('ClusterHud') == 2 and p.get_int('ClusterHudTheme') == 1
  assert p.get_bool('IsOnroad') and result['usb_restart_required']
  assert (tmp_path / 'usb-management.enabled').exists()
  monkeypatch.setattr(support, 'init_management_key', lambda: pytest.fail('Disabling must not create keys'))
  support.jetson_settings({'settings': {'usb_management': False}})
  assert not (tmp_path / 'usb-management.enabled').exists()


def test_web_params_and_management_writes_as_comma(monkeypatch, tmp_path):
  calls = []
  monkeypatch.setattr(support, 'supported', lambda: True)
  monkeypatch.setattr(support, 'STATE', tmp_path)
  monkeypatch.setattr(support.setup, 'trusted', lambda *a, **k: None)
  monkeypatch.setattr(support.pwd, 'getpwnam', lambda name: SimpleNamespace(pw_uid=1000, pw_gid=1000))
  monkeypatch.setattr(support.os, 'geteuid', lambda: 0)
  monkeypatch.setattr(support.os, 'getegid', lambda: 0)
  monkeypatch.setattr(support.os, 'seteuid', lambda uid: calls.append(('uid', uid)))
  monkeypatch.setattr(support.os, 'setegid', lambda gid: calls.append(('gid', gid)))
  monkeypatch.setattr(support, 'jetson_settings', lambda body: calls.append(('write', body)))
  support.execute('jetson-settings', {'settings': {'ClusterHudTheme': 1}})
  assert calls == [('gid', 1000), ('uid', 1000), ('write', {'settings': {'ClusterHudTheme': 1}}), ('uid', 0), ('gid', 0)]


def test_jetson_web_uses_live_snapshot_not_cached_hello(monkeypatch, tmp_path):
  from openpilot.common import jetlink_status as status
  monkeypatch.setattr(status, 'LINK_STATUS', tmp_path / 'link.json')
  monkeypatch.setattr(status, 'MODEL_STATUS', tmp_path / 'model.json')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 100.)
  link = {'state': 'ready', 'updated': 100., 'telemetry_updated': 100., 'model': 'Cinque v2',
    'peer': {'carrot_host': 'jetson', 'engine_state': 'none', 'carrot_hud_connected': True},
    'telemetry': {'engine_state': 'ready', 'carrot_hud_connected': False,
      'carrot_health': {'severity': 'ok', 'age_s': 0., 'temp_c': 50.,
        'addresses': [{'address': '192.168.0.21'}]}}}
  status.LINK_STATUS.write_text(json.dumps(link))
  value = status.web_status()
  assert value['badge'] == 'jetSON READY' and not value['active']
  assert value['engine_state'] == 'ready' and value['hud_connected'] is False
  assert value['health']['temp_c'] == 50.
  status.MODEL_STATUS.write_text(json.dumps({'active': True, 'updated': 100.}))
  assert status.web_status()['active']
  link['telemetry_updated'] = 95.
  status.LINK_STATUS.write_text(json.dumps(link))
  value = status.web_status()
  assert value['badge'] == 'jetSON DATA?' and value['engine_state'] is None
  assert value['hud_connected'] is None and value['health']['temp_c'] is None
  link['updated'] = 95.
  status.LINK_STATUS.write_text(json.dumps(link))
  status.MODEL_STATUS.unlink()
  value = status.web_status()
  assert not value['fresh'] and not value['active']
  assert value['health']['addresses'] == [] and value['health']['temp_c'] is None


def test_jetson_api_and_cached_login_expiry(monkeypatch):
  async def scenario():
    async def runner(action, body):
      if action == 'jetson-status':
        return {'ok': True, 'jetlink': {'badge': 'jetSON READY'}, 'settings': {'ClusterHud': 2}}
      if action == 'login':
        return {'ok': True, 'network': {'backend': 'NeedsLogin', 'auth_url': 'https://login.tailscale.com/a/test-only'}}
      return {'ok': True, 'network': {'backend': 'NeedsLogin'}}
    app = web.Application()
    api.register(app)
    state = app[api.STATE_KEY] = api.SupportState(runner)
    monkeypatch.setattr(api.time, 'monotonic', lambda: 100.)
    await state.run('login', {})
    assert 'auth_url' not in state.operation['result']['network']
    async with TestClient(TestServer(app)) as client:
      value = await (await client.get('/api/jetson')).json()
      assert value['csrf'] == state.csrf and value['settings']['ClusterHud'] == 2
      assert value['jetlink']['badge'] == 'jetSON READY'
      value = await (await client.get('/api/remote_support')).json()
      assert value['network']['auth_url'].endswith('/test-only')
      state.login_deadline = 99.
      value = await (await client.get('/api/remote_support')).json()
      assert not value['network'].get('auth_url')
  asyncio.run(scenario())
