"""LAN-only, same-origin remote-support operations using a fixed sudo helper."""
import asyncio
import contextlib
import hmac
import ipaddress
import json
from pathlib import Path
import secrets
import sys
import time
from urllib.parse import urlsplit

from aiohttp import web

ROOT = Path(__file__).resolve().parents[5]
HELPER = ROOT / 'tools/jetlink/remote_support.py'
NETWORKS = tuple(ipaddress.ip_network(value) for value in ('127.0.0.0/8', '10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))
ACTIONS = {'install', 'login', 'switch-account', 'network-on', 'network-off',
           'key-preview', 'add-key', 'remove-key', 'jetson-check', 'jetson-settings'}
STATE_KEY = 'hl_remote_support'


def validate_request(request, mutate=False):
  try:
    peer = ipaddress.ip_address(request.remote)
    local = ipaddress.ip_address(request.transport.get_extra_info('sockname')[0])
    host = urlsplit('http://' + request.host)
    hostname = host.hostname
    if hostname == 'localhost' and local.is_loopback:
      target = local
    else:
      target = ipaddress.ip_address(hostname)
    if target != local or not (peer == ipaddress.ip_address('::1') or any(peer in net for net in NETWORKS)):
      raise ValueError('Not local')
    if request.headers.get('Sec-Fetch-Site', 'same-origin') not in ('same-origin', 'none'):
      raise ValueError('Cross-site request')
    if any(name in request.headers for name in ('Forwarded', 'X-Forwarded-For', 'X-Forwarded-Host')):
      raise ValueError('Forwarded request')
    origin = request.headers.get('Origin')
    if (mutate or origin) and origin != f'{request.scheme}://{request.host}':
      raise ValueError('Origin mismatch')
  except (ValueError, TypeError, AttributeError):
    raise web.HTTPForbidden(text='Remote setup requires the comma local IP on the same trusted network.') from None


async def helper(action, body):
  process = await asyncio.create_subprocess_exec(
    'sudo', '-n', sys.executable, str(HELPER), action,
    cwd=str(ROOT), env={'PATH': '/usr/local/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin',
                       'PYTHONPATH': f'{ROOT}/pydeps:{ROOT}'},
    stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
  try:
    output, _ = await asyncio.wait_for(process.communicate(json.dumps(body).encode()), timeout=150)
  except (TimeoutError, asyncio.CancelledError):
    with contextlib.suppress(ProcessLookupError):
      process.kill()
    await process.wait()
    raise
  try:
    result = json.loads(output)
  except ValueError:
    raise RuntimeError('Remote support helper failed; check installation and sudo access.') from None
  if process.returncode or result.get('ok') is not True:
    raise RuntimeError(result.get('error', 'Remote support operation failed.'))
  return result


class SupportState:
  def __init__(self, runner=helper):
    self.runner = runner
    self.csrf = secrets.token_urlsafe(32)
    self.task = None
    self.operation = {'state': 'idle'}
    self.login_url = ''
    self.login_deadline = 0.
    self.jetson = None
    self.status_lock = asyncio.Lock()

  async def run(self, action, body):
    self.operation = {'state': 'running', 'action': action}
    if action in ('login', 'switch-account', 'network-off'):
      self.login_url = ''
      self.login_deadline = 0.
    try:
      result = await self.runner(action, body)
      if action in ('login', 'switch-account'):
        self.login_url = result.get('network', {}).get('auth_url', '')
        self.login_deadline = time.monotonic() + 600
      if action in ('login', 'switch-account', 'network-on', 'network-off'):
        result.get('network', {}).pop('auth_url', None)
      if action == 'jetson-check':
        self.jetson = result
      self.operation = {'state': 'done', 'action': action, 'result': result}
    except Exception as error:
      self.operation = {'state': 'error', 'action': action, 'error': str(error)}


def response(body, status=200):
  return web.json_response(body, status=status, headers={'Cache-Control': 'no-store', 'Referrer-Policy': 'no-referrer'})


async def get_status(request):
  validate_request(request)
  state = request.app[STATE_KEY]
  try:
    async with state.status_lock:
      value = await state.runner('status', {})
    network = value.get('network', {})
    if network.get('backend') == 'Running':
      state.login_url = ''
    elif not network.get('auth_url') and time.monotonic() < state.login_deadline:
      network['auth_url'] = state.login_url
    return response({**value, 'csrf': state.csrf, 'operation': state.operation, 'jetson': state.jetson})
  except Exception as error:
    return response({'ok': False, 'error': str(error)}, 503)


async def get_jetson(request):
  validate_request(request)
  state = request.app[STATE_KEY]
  try:
    async with state.status_lock:
      value = await state.runner('jetson-status', {})
    return response({**value, 'csrf': state.csrf, 'operation': state.operation, 'ssh': state.jetson})
  except Exception as error:
    return response({'ok': False, 'error': str(error)}, 503)


async def action(request):
  validate_request(request, mutate=True)
  state = request.app[STATE_KEY]
  if not hmac.compare_digest(request.headers.get('X-HL-CSRF', ''), state.csrf):
    raise web.HTTPForbidden(text='Refresh this page before continuing.')
  if request.content_type != 'application/json' or (request.content_length or 0) > 8192:
    raise web.HTTPBadRequest(text='Expected a small JSON request.')
  try:
    raw = bytearray()
    async for chunk in request.content.iter_chunked(4096):
      raw.extend(chunk)
      if len(raw) > 8192:
        raise ValueError('Too large')
    body = json.loads(raw)
    name = body.pop('action')
    if name not in ACTIONS:
      raise ValueError('Unsupported action')
  except (ValueError, TypeError, KeyError, AttributeError):
    raise web.HTTPBadRequest(text='Invalid operation.') from None
  if state.task and not state.task.done():
    return response({'ok': False, 'error': 'Another remote-support operation is in progress.'}, 409)
  if name == 'key-preview':
    try:
      return response(await state.runner(name, body))
    except Exception as error:
      return response({'ok': False, 'error': str(error)}, 400)
  if body.pop('confirmed', False) is not True:
    raise web.HTTPBadRequest(text='Explicit confirmation is required.')
  state.operation = {'state': 'running', 'action': name}
  state.task = asyncio.create_task(state.run(name, body))
  return response({'ok': True, 'operation': state.operation}, 202)


async def cleanup(app):
  task = app[STATE_KEY].task
  if task and not task.done():
    task.cancel()
    with contextlib.suppress(asyncio.CancelledError):
      await task


def register(app):
  app[STATE_KEY] = SupportState()
  app.router.add_get('/api/remote_support', get_status)
  app.router.add_post('/api/remote_support', action)
  app.router.add_get('/api/jetson', get_jetson)
  app.on_cleanup.append(cleanup)
