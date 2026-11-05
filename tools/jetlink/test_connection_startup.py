import json
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock  # noqa: TID251

import pytest

from openpilot.common import jetlink_status as status
from openpilot.selfdrive.modeld.jetlink import daemon, startup
from jetlink.transport.base import LinkError, LinkTimeout


@pytest.mark.parametrize('error', [TimeoutError('HELLO timeout'), LinkTimeout('HELLO timeout'),
                                 LinkError('gadget write had no reader for 3.000s'),
                                 LinkError('gadget write had no reader before deadline; link abandoned')])
@pytest.mark.parametrize('elapsed,ready,preparing,expected', [
  (0, False, False, True), (119.9, False, False, True),
  (120, False, False, False), (-1, False, False, False),
  (float('inf'), False, False, False), (float('nan'), False, False, False),
  (1, True, False, False), (1, False, True, False),
])
def test_only_bounded_first_handshake_timeout_is_waiting(error, elapsed, ready, preparing, expected):
  assert startup.initial_connection_wait(error, elapsed=elapsed, ready_once=ready, preparing=preparing) is expected


@pytest.mark.parametrize('error', [ValueError('signature'), LinkError('partial payload'), OSError('USB failure'),
                                 LinkError('hello response is not an object'), LinkError('server selected invalid protocol value 9'),
                                 RuntimeError('engine failed'), ConnectionError('disconnected'),
                                 daemon.UnsafeCleanupError('active reader'), startup.RuntimeHandoff('gate ended')])
def test_real_failures_are_not_initial_waits(error):
  assert not startup.initial_connection_wait(error, elapsed=1, ready_once=False, preparing=False)


@pytest.fixture
def gate(tmp_path, monkeypatch):
  release = tmp_path / 'release.json'
  release.write_text(json.dumps({'source_commit': 'test'}))
  monkeypatch.setattr(startup, 'RELEASE', release)
  client = NS(t=NS(send_json=Mock()), _next_seq=lambda: 1,
              state=Mock(return_value={'carrot_update': {'state': 'ready'}}))
  return client, {'carrot_host': 'jetson', startup.CAPABILITY: True}


@pytest.mark.parametrize('reply', [{'carrot_update': {'state': 'ready'}}, {}, {'carrot_update': None}])
def test_boot_gate_handoff_reconnects_once_without_changing_release(gate, reply):
  client, peer = gate
  client.state.return_value = reply
  with pytest.raises(startup.RuntimeHandoff, match='reconnecting to runtime'):
    startup.wait_for_boot_update(client, peer, None, lambda: True, Mock())
  client.t.send_json.assert_called_once_with(startup.MESSAGE, 1, {'source_commit': 'test'})


@pytest.mark.parametrize('attached', [[False, False], [True, False]])
def test_boot_gate_disconnect_is_not_a_handoff(gate, attached):
  client, peer = gate
  connected = iter(attached)
  with pytest.raises(ConnectionError, match='disconnected') as error:
    startup.wait_for_boot_update(client, peer, None, lambda: next(connected), Mock())
  assert not isinstance(error.value, startup.RuntimeHandoff)


@pytest.mark.parametrize('failure', [TimeoutError('state timeout'), LinkError('USB failure')])
def test_boot_gate_transport_failure_is_not_a_handoff(gate, failure):
  client, peer = gate
  client.state.side_effect = failure
  with pytest.raises(type(failure), match=str(failure)):
    startup.wait_for_boot_update(client, peer, None, lambda: True, Mock())


@pytest.fixture
def reports(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20.)
  def write(state, *, error='', model_error='', active=False, updated=20.):
    link.write_text(json.dumps({'updated': updated, 'state': state, 'peer': {'carrot_host': 'jetson'},
                               'error': error, 'preparation': {'stage': 'waiting-host', 'message': error}}))
    model.write_text(json.dumps({'updated': 20., 'active': active, 'error': model_error}))
  return write


@pytest.mark.parametrize('state,label', [('connecting', '연결 대기'), ('loading', '모델 준비 중')])
def test_wait_and_model_preparation_preserve_web_and_display_details(reports, state, label):
  reports(state, error='last unanswered initial HELLO')
  badge, detail = status.display_status()
  assert badge == (f'jetSON {label}', 'loading')
  assert 'last unanswered initial HELLO' in detail
  web = status.web_status()
  assert (web['badge'], web['style'], web['detail']) == (*badge, detail)


@pytest.mark.parametrize('state,model_error', [
  ('blocked', ''), ('retrying', ''), ('connecting', 'inference failed'), ('loading', 'warp validation failed'),
])
def test_real_error_overrides_startup_progress_and_keeps_details(reports, state, model_error):
  reports(state, error='link failed', model_error=model_error)
  badge, detail = status.display_status()
  assert badge == ('jetSON ERROR', 'error')
  assert 'Link: link failed' in detail
  if model_error:
    assert f'Model: {model_error}' in detail
  web = status.web_status()
  assert (web['badge'], web['style'], web['detail']) == (*badge, detail)


def test_stale_progress_does_not_claim_current_connection(reports):
  reports('connecting', updated=17.)
  assert status.badge() == ('jetSON ERROR', 'error')
  assert 'stale' in status.display_status()[1]
  assert not status.web_status()['fresh']


@pytest.fixture
def run_daemon(tmp_path, monkeypatch):
  """Run the actual retry loop with fake time, transports and OS boundaries."""
  class StopDaemon(BaseException):
    pass

  current, now, published = {}, [0.], []
  udc = tmp_path / 'udc/controller'
  udc.mkdir(parents=True)
  (udc / 'current_speed').write_text('super-speed')
  real_path = daemon.Path
  monkeypatch.setattr(daemon, 'Path', lambda value: tmp_path / 'udc' if value == '/sys/class/udc' else real_path(value))
  monkeypatch.setattr(daemon, 'SOCKET', str(tmp_path / 'socket'))
  monkeypatch.setattr(daemon, 'open', Mock(), raising=False)
  monkeypatch.setattr(daemon.fcntl, 'flock', Mock())
  monkeypatch.setattr(daemon.os, 'sched_setscheduler', Mock(), raising=False)
  monkeypatch.setattr(daemon.os, 'sched_setaffinity', Mock(), raising=False)
  monkeypatch.setattr(daemon.os, 'sched_param', Mock(), raising=False)
  monkeypatch.setattr(daemon.os, 'chmod', Mock())
  monkeypatch.setattr(daemon, 'gc', Mock())
  monkeypatch.setattr(daemon, 'socket', NS(socket=Mock(), AF_UNIX=0, SOCK_STREAM=0))
  monkeypatch.setattr(daemon, 'time', NS(monotonic=lambda: now[0], sleep=lambda _: None))
  monkeypatch.setattr(daemon, 'transport_mode', lambda: 'usb')
  monkeypatch.setattr(daemon, 'initialize_management', lambda: False)
  monkeypatch.setattr(daemon, 'host_attached', lambda: current.get('attached', True))
  monkeypatch.setattr(daemon, 'publish', lambda state, **kw: published.append({'state': state, **kw}))
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS()))

  def step(stage):
    if current.get('stage') == stage:
      raise current['error']

  peer = {'carrot_host': 'jetson'}
  def close():
    if current.get('cleanup_error'):
      raise current['cleanup_error']

  client = NS(last_state=None, spec=NS(warped_shape=(1,), packed_nelem=1),
              infer=lambda *a, **kw: step('warmup'), close=close)
  monkeypatch.setattr(daemon, 'setup_gadget', lambda *a, **kw: step('setup'))
  monkeypatch.setattr(daemon, 'CarrotTransport', lambda *a, **kw: NS(bound_udc='controller', close=Mock()))
  monkeypatch.setattr(daemon, 'JetlinkClient', lambda *a, **kw: client)
  monkeypatch.setattr(daemon, 'greet', lambda *a, **kw: step('hello') or peer)
  monkeypatch.setattr(daemon, 'wait_for_boot_update', lambda *a, **kw: step('gate'))
  monkeypatch.setattr(daemon, 'wait_for_selection', lambda *a: peer)
  monkeypatch.setattr(daemon, 'prepare', lambda *a, **kw: step('prepare'))
  monkeypatch.setattr(daemon, 'serve_local', lambda *a: step('serve'))

  def run(attempts):
    pending = iter(attempts)
    def advance():
      try:
        attempt = next(pending)
      except StopIteration:
        raise StopDaemon from None
      current.clear()
      current.update(attempt)
      now[0] = attempt['at']
    monkeypatch.setattr(daemon, 'update_affinity', advance)
    with pytest.raises(StopDaemon):
      daemon.main()
    return published
  return run


def test_daemon_retries_and_usb_detach_do_not_renew_initial_deadline(run_daemon):
  failure = {'stage': 'hello', 'error': LinkTimeout('HELLO timeout')}
  reports = run_daemon([{'at': 0., **failure}, {'at': 119.9, **failure}, {'at': 120., 'attached': False},
                        {'at': 121., **failure}])
  waits = [r for r in reports if r.get('preparation', {}).get('stage') == 'waiting-host']
  assert len(waits) == 2
  assert all(r['error'] == r['preparation']['message'] == 'HELLO timeout' for r in waits)
  assert [r['error'] for r in reports if r['state'] == 'retrying'] == ['HELLO timeout']


@pytest.mark.parametrize('stage', ['setup', 'gate', 'prepare', 'warmup', 'serve'])
def test_daemon_only_grants_grace_during_hello(run_daemon, stage):
  reports = run_daemon([{'at': 0., 'stage': stage, 'error': TimeoutError(f'{stage} failed')}])
  assert not any(r.get('preparation', {}).get('stage') == 'waiting-host' for r in reports)
  assert [r['error'] for r in reports if r['state'] == 'retrying'] == [f'{stage} failed']


def test_daemon_never_grants_initial_grace_after_a_ready_session(run_daemon):
  reports = run_daemon([{'at': 0.}, {'at': 1., 'stage': 'hello', 'error': LinkTimeout('reconnect failed')}])
  assert not any(r.get('preparation', {}).get('stage') == 'waiting-host' for r in reports)
  assert [r['error'] for r in reports if r['state'] == 'retrying'] == ['reconnect failed']


def test_daemon_handoff_gets_one_separate_deadline_after_a_long_update(run_daemon):
  handoff = {'stage': 'gate', 'error': startup.RuntimeHandoff('reconnecting to runtime')}
  timeout = {'stage': 'hello', 'error': LinkTimeout('runtime HELLO timeout')}
  reports = run_daemon([{'at': 0., 'stage': 'hello', 'error': LinkTimeout('initial HELLO timeout')},
                        {'at': 300., **handoff}, {'at': 301., 'attached': False}, {'at': 419., **timeout},
                        {'at': 419.9, **handoff}, {'at': 420., **handoff}, {'at': 421., **timeout}])
  assert len([r for r in reports if r.get('preparation', {}).get('stage') == 'runtime-handoff']) == 2
  assert len([r for r in reports if r.get('preparation', {}).get('stage') == 'waiting-host']) == 2
  failures = [r['error'] for r in reports if r['state'] == 'retrying']
  assert len(failures) == 2 and 'runtime handoff exceeded 120s' in failures[0]
  assert failures[1] == 'runtime HELLO timeout'


def test_daemon_unsafe_cleanup_remains_blocked(run_daemon):
  reports = run_daemon([{'at': 0., 'stage': 'hello', 'error': daemon.UnsafeCleanupError('reader still active')},
                        {'at': 1.}])
  assert not any(r.get('preparation') for r in reports)
  assert len([r for r in reports if r['state'] == 'blocked']) == 2


def test_daemon_cleanup_failure_overrides_initial_wait_on_next_iteration(run_daemon):
  reports = run_daemon([{'at': 0., 'stage': 'hello', 'error': LinkTimeout('HELLO timeout'),
                        'cleanup_error': RuntimeError('reader still active')}, {'at': 1.}, {'at': 2.}])
  assert len([r for r in reports if r.get('preparation', {}).get('stage') == 'waiting-host']) == 1
  blocked = [r for r in reports if r['state'] == 'blocked']
  assert len(blocked) == 2 and all('reader still active' in r['error'] for r in blocked)
