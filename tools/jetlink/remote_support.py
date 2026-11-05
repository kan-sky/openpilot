"""Fixed-action helper for Carrot Web remote support; never runs browser commands."""
import argparse
import contextlib
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import platform
import pwd
import subprocess
import sys
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
# sudo removes PYTHONPATH; use the same repo-local dependencies as the launcher.
sys.path[:0] = [str(ROOT / 'pydeps'), str(ROOT)]
from tools.jetlink import manage_jetson as management
from tools.jetlink import setup_comma_remote as setup

STATE = Path('/data/hl-remote-support')
KEY_MARKER = 'hl-remote-support'
PROVIDERS = (
  ('hl', setup.DIRECTORY / 'bin/tailscale', setup.SOCKET),
  ('wayon', Path('/data/wayon-tailscale/bin/tailscale'), '/run/wayon-tailscale/tailscaled.sock'),
  ('system', Path('/usr/bin/tailscale'), '/var/run/tailscale/tailscaled.sock'),
)
HUD_SETTINGS = {'ClusterHud': range(3), 'ClusterHudTheme': range(3)}


def supported():
  return Path('/TICI').is_file() and Path('/AGNOS').is_file() and platform.machine() in ('aarch64', 'arm64')


def command(args, timeout=15):
  result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
  return result.returncode, result.stdout, result.stderr


def provider():
  found = [entry for entry in PROVIDERS if entry[1].is_file()]
  active = [entry for entry in found if Path(entry[2]).exists()]
  if len(active) > 1:
    raise ValueError('Multiple Tailscale daemons found. Resolve the duplicate installation first.')
  if active:
    return active[0]
  if len(found) > 1:
    raise ValueError('Multiple Tailscale installations found; refusing to select an account implicitly.')
  if found:
    return found[0]
  code, _, _ = command(['pgrep', '-x', 'tailscaled'])
  if code == 0:
    raise ValueError('An unsupported Tailscale installation is already running; it was left unchanged.')
  return None


def client(entry, *args, timeout=15):
  setup.trusted(entry[1])
  return command([str(entry[1]), '--socket=' + entry[2], *args], timeout)


def auth_url(value):
  if not isinstance(value, str):
    return ''
  try:
    url = urllib.parse.urlsplit(value)
    if (url.scheme == 'https' and url.hostname == 'login.tailscale.com'
        and url.path.startswith('/a/') and not url.username and not url.password and not url.port):
      return value
  except ValueError:
    pass
  return ''


def network_status(entry):
  if not entry:
    return {'installed': False, 'backend': 'NotInstalled', 'online': False, 'auth_url': ''}
  code, output, _ = client(entry, 'status', '--json')
  try:
    data = json.loads(output)
  except (ValueError, TypeError):
    return {'installed': True, 'provider': entry[0], 'backend': 'Unavailable', 'online': False,
            'auth_url': '', 'error': 'Tailscale daemon is not reachable.'}
  own = data.get('Self') or {}
  user = (data.get('User') or {}).get(str(own.get('UserID')), {})
  account = user.get('LoginName', '')
  if data.get('BackendState') == 'Stopped' and not account:
    # After a stopped daemon reboots there is no network map/User entry yet.
    # Read only the selected saved profile; never return other accounts.
    profile_code, profiles, _ = client(entry, 'switch', '--list', '--json')
    try:
      profiles = json.loads(profiles) if not profile_code else []
      if isinstance(profiles, list):
        account = next((p.get('account', '') for p in profiles if isinstance(p, dict) and p.get('selected') is True), '')
    except (ValueError, TypeError):
      pass
  return {'installed': True, 'provider': entry[0], 'backend': data.get('BackendState', 'Unknown'),
          'online': data.get('BackendState') == 'Running' and own.get('Online') is True,
          'hostname': own.get('HostName', ''), 'account': account,
          'addresses': own.get('TailscaleIPs') or [], 'key_expiry': own.get('KeyExpiry', ''),
          'health': data.get('Health') or [], 'auth_url': auth_url(data.get('AuthURL', '')),
          'status_code': code}


def params():
  from openpilot.common.params import Params
  return Params()


def key_lines(value):
  if isinstance(value, bytes):
    value = value.decode('utf-8')
  return str(value or '').splitlines()


def support_keys(lines):
  result = []
  for line in lines:
    parts = line.split()
    if len(parts) == 3 and parts[2] == KEY_MARKER:
      try:
        result.append({'fingerprint': management.fingerprint(' '.join(parts[:2]))})
      except ValueError:
        pass
  return result


def status():
  if not supported():
    return {'supported': False, 'network': {'installed': False}, 'ssh': {'keys': []}}
  p = params()
  return {'supported': True, 'network': network_status(provider()),
          'ssh': {'enabled': p.get_bool('SshEnabled'), 'keys': support_keys(key_lines(p.get('GithubSshKeys')))},
          'onroad': p.get_bool('IsOnroad'),
          'jetson_paired': (management.STATE / 'config.json').is_file()}


@contextlib.contextmanager
def mutation_lock():
  STATE.mkdir(mode=0o700, exist_ok=True)
  setup.trusted(STATE, directory=True)
  with (STATE / 'operation.lock').open('a') as stream:
    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    yield


@contextlib.contextmanager
def comma_identity():
  """Params and pairing files must stay readable by the normal manager user."""
  owner = pwd.getpwnam('comma')
  uid, gid = os.geteuid(), os.getegid()
  try:
    os.setegid(owner.pw_gid)
    os.seteuid(owner.pw_uid)
    yield
  finally:
    os.seteuid(uid)
    os.setegid(gid)


def install():
  if provider():
    return {'changed': False}
  # This opt-in web path changes no USB, inference, DNS or vehicle-control state.
  # The legacy CLI retains its parked-only maintenance workflow.
  setup.install()
  with urllib.request.urlopen(setup.URL, timeout=60) as response:
    data = response.read(setup.MAX_DOWNLOAD + 1)
  if provider():
    raise ValueError('A Tailscale installation appeared while downloading; nothing was replaced.')
  setup.prepare_files(data)
  setup.ensure_service()
  return {'changed': True}


def login(switch=False, expected_account=None):
  entry = provider()
  if not entry:
    raise ValueError('Install Tailscale first.')
  before = network_status(entry)
  if switch:
    if expected_account != before.get('account', ''):
      raise ValueError('The signed-in account changed. Refresh and confirm the current account again.')
  elif before['backend'] == 'Running':
    return {'changed': False, 'network': before}
  firewall_args = []
  if entry[0] == 'wayon':
    # Wayon can own an SSH-only firewall separately. Preserve that policy when
    # explicit `up` flags are used, including after an approved account switch.
    code, raw, _ = client(entry, 'debug', 'prefs')
    try:
      mode = json.loads(raw)['NetfilterMode']
      if code or type(mode) is not int or mode not in (0, 1, 2):
        raise ValueError
    except (ValueError, KeyError, TypeError):
      raise RuntimeError('Could not read the existing Wayon firewall policy; account unchanged.') from None
    firewall_args = ['--netfilter-mode=' + {0: 'off', 1: 'nodivert', 2: 'on'}[mode]]
  if switch:
    code, _, _ = client(entry, 'logout')
    if code:
      raise RuntimeError('Could not sign out. The account was not replaced.')
  if entry[0] == 'hl':
    (setup.DIRECTORY / 'enabled').touch(mode=0o644)
    setup.ensure_service()
  args = setup.login_args()[2:] + firewall_args + ['--json', '--timeout=5s']
  code, output, _ = client(entry, *args)
  after = network_status(entry)
  # `up --json` emits a stream of objects and may time out while the user logs in.
  # Never return raw CLI output: it can contain private authentication material.
  decoder = json.JSONDecoder()
  remaining = output.lstrip()
  while remaining:
    try:
      value, end = decoder.raw_decode(remaining)
    except ValueError:
      break
    if isinstance(value, dict):
      after['auth_url'] = auth_url(value.get('AuthURL')) or after.get('auth_url', '')
    remaining = remaining[end:].lstrip()
  if code and not after.get('auth_url') and after['backend'] != 'Running':
    raise RuntimeError('Login link unavailable. Check the comma Internet connection and retry.')
  return {'changed': True, 'network': after}


def set_network(enabled):
  entry = provider()
  if not entry:
    raise ValueError('Install Tailscale first.')
  before = network_status(entry)
  target = 'Running' if enabled else 'Stopped'
  if before['backend'] == target:
    return {'changed': False, 'network': before}
  if before['backend'] in ('Unavailable', 'Unknown'):
    raise RuntimeError('Tailscale service is unavailable. Check the service before retrying.')
  if enabled and before['backend'] in ('NeedsLogin', 'NoState'):
    raise ValueError('Sign in to Tailscale before enabling the connection.')
  # Even --timeout makes `up` require all non-default preference flags. Use no
  # flags and bound the subprocess instead, preserving the saved configuration.
  try:
    code, _, _ = client(entry, 'up' if enabled else 'down', timeout=10)
  except subprocess.TimeoutExpired:
    # The daemon may have accepted the request before the CLI timed out.
    code = 0
  after = network_status(entry)
  if code or after['backend'] != target:
    raise RuntimeError('Tailscale connection did not reach the requested state. Check status and retry.')
  return {'changed': True, 'network': after}


def preview_key(body):
  key = management.public_key(str(body.get('public_key', '')))
  return {'public_key': key, 'fingerprint': management.fingerprint(key)}


def change_key(action, body):
  p = params()
  lines = key_lines(p.get('GithubSshKeys'))
  if action == 'add-key':
    value = preview_key(body)
    if body.get('fingerprint') != value['fingerprint']:
      raise ValueError('Confirm the SSH public-key fingerprint before granting access.')
    if any(' '.join(line.split()[:2]) == value['public_key'] for line in lines):
      # An existing owner's key is never re-labelled as a removable support key.
      enabled = p.get_bool('SshEnabled')
      p.put_bool('SshEnabled', True)
      return {'changed': not enabled, 'already_authorized': True}
    if len(support_keys(lines)) >= 5:
      raise ValueError('Remove an old support key before adding another (maximum 5).')
    lines.append(value['public_key'] + ' ' + KEY_MARKER)
    p.put('GithubSshKeys', '\n'.join(lines) + '\n')
    p.put_bool('SshEnabled', True)
  else:
    wanted = body.get('fingerprint')
    if wanted not in {item['fingerprint'] for item in support_keys(lines)}:
      raise ValueError('This is not a registered support key; owner keys are unchanged.')
    lines = [line for line in lines if not any(item['fingerprint'] == wanted for item in support_keys([line]))]
    p.put('GithubSshKeys', '\n'.join(lines) + ('\n' if lines else ''))
  return {'changed': True}


def jetson_check():
  config = management.paired_config(management.STATE)
  failures = []
  for host in management.candidates(config, management.STATE, management.discover()):
    try:
      name = management.ssh(management.STATE, host, ['hostname']).strip()
      user = management.ssh(management.STATE, host, ['id', '-un']).strip()
      return {'connected': True, 'host': host, 'hostname': name, 'user': user,
              'transport': 'USB' if ipaddress.ip_address(host).is_link_local else 'LAN'}
    except (RuntimeError, subprocess.SubprocessError):
      failures.append(host)
  return {'connected': False, 'tried': failures}


def jetson_status():
  from openpilot.common.jetlink_status import web_status
  p = params()
  settings = {name: p.get_int(name) for name in HUD_SETTINGS}
  settings['usb_management'] = (management.STATE / 'usb-management.enabled').exists()
  usb = management.read_json(Path('/dev/shm/hl-jetson-usb.json'), {})
  paired = (management.STATE / 'config.json').is_file()
  return {'supported': supported(), 'jetlink': web_status(), 'settings': settings,
          'usb': {key: usb.get(key) for key in ('state', 'interface', 'address', 'peer')}, 'paired': paired}


def init_management_key():
  # The normal comma manager must retain access to newly created management keys.
  management.init_key(management.STATE)


def jetson_settings(body):
  values = body.get('settings')
  if not isinstance(values, dict) or not values or set(values) - (set(HUD_SETTINGS) | {'usb_management'}):
    raise ValueError('Unsupported Jetson setting')
  for name, value in values.items():
    if name == 'usb_management':
      if type(value) is not bool:
        raise ValueError('USB management must be true or false')
    elif type(value) is not int or value not in HUD_SETTINGS[name]:
      raise ValueError('Invalid HUD setting')
  p = params()
  if values.get('usb_management'):
    init_management_key()
  for name, value in values.items():
    if name == 'usb_management':
      marker = management.STATE / 'usb-management.enabled'
      if value:
        marker.touch(mode=0o600)
      else:
        marker.unlink(missing_ok=True)
    else:
      p.put_int(name, value)
  return {'changed': True, 'usb_restart_required': 'usb_management' in values}


def execute(action, body):
  if action == 'status':
    return status()
  if action == 'jetson-status':
    return jetson_status()
  if action == 'key-preview':
    return preview_key(body)
  if not supported():
    raise ValueError('Remote support is available only on an AGNOS comma.')
  with mutation_lock():
    if action == 'install':
      return install()
    if action in ('login', 'switch-account'):
      return login(action == 'switch-account', body.get('account'))
    if action in ('network-on', 'network-off'):
      return set_network(action == 'network-on')
    if action in ('add-key', 'remove-key'):
      with comma_identity():
        return change_key(action, body)
    if action == 'jetson-check':
      return jetson_check()
    if action == 'jetson-settings':
      with comma_identity():
        return jetson_settings(body)
    raise ValueError('Unsupported action')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=('status', 'install', 'login', 'switch-account', 'network-on', 'network-off',
                                       'key-preview', 'add-key', 'remove-key', 'jetson-check',
                                       'jetson-status', 'jetson-settings'))
  args = parser.parse_args()
  try:
    if os.geteuid() != 0:
      raise ValueError('Run the fixed-action helper using sudo.')
    setup.background_priority()
    raw = sys.stdin.read(8193)
    if len(raw) > 8192:
      raise ValueError('Request too large')
    body = json.loads(raw or '{}')
    if not isinstance(body, dict):
      raise ValueError('Expected an object')
    print(json.dumps({'ok': True, **execute(args.action, body)}))
  except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as error:
    # Authentication URLs, command arguments and account-state files never enter logs.
    message = str(error) if isinstance(error, (ValueError, RuntimeError)) else type(error).__name__
    print(json.dumps({'ok': False, 'error': message}))
    sys.exit(1)


if __name__ == '__main__':
  main()
