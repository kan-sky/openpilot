"""Device-free transaction/cache tests; never invoke sudo, SSH, CUDA or systemd."""
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import pwd
import signal
import subprocess
import sys
import tarfile
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import n16_overlay as overlay


@pytest.fixture
def case(tmp_path, monkeypatch):
  monkeypatch.setattr(overlay, 'RELEASE_UID', os.getuid())
  source = tmp_path / 'source'
  source.mkdir()
  model, plan, meta = [tmp_path / name for name in ('model.onnx', 'engine.plan', 'engine.json')]
  model.write_bytes(b'verified synthetic model')
  plan.write_bytes(b'verified synthetic plan')
  monkeypatch.setattr(overlay, 'MODEL', overlay.digest(model))
  monkeypatch.setattr(overlay, 'MODEL_BYTES', model.stat().st_size)
  monkeypatch.setattr(overlay, 'PLAN_BYTES', plan.stat().st_size)
  spec = {'sha256': overlay.MODEL, 'nbytes': overlay.MODEL_BYTES, 'frame_skip': 4,
          'input_shapes': {'state_img_q': [1]}, 'output_shapes': {'outputs': [1, 1]}}
  for name in overlay.REQUIRED:
    path = source / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(spec) if name == overlay.SPEC else '# reviewed runtime ' + name)
  monkeypatch.setattr(overlay, 'SPEC_SHA256', overlay.digest(source / overlay.SPEC))
  meta.write_text(json.dumps({'backend': 'trt', 'trt_version': '10.3.0', 'device': 'Orin-sm87', 'spec': spec}))
  revision = 'a' * 40
  tracked = sorted(name for name in overlay.REQUIRED if name.startswith('third_party/'))

  def git(command, **kwargs):
    if command[-2:] == ['rev-parse', 'HEAD']:
      return revision
    assert command[-4:] == ['ls-files', '-z', '--', 'third_party/jetlink']
    return '\0'.join(tracked) + '\0'

  monkeypatch.setattr(overlay.subprocess, 'check_output', git)
  ident = overlay.source_identity(source)
  bundle = tmp_path / 'bundle.tar.gz'
  built = overlay.prepare(source, bundle, revision, ident['content_sha256'])
  runtime = tmp_path / 'runtime'
  current = runtime / 'releases' / overlay.BASE
  current.mkdir(parents=True)
  for name, data in {'SOURCE_COMMIT': overlay.BASE, 'tools/jetlink/hud.py': '# unchanged HUD',
                     'tools/jetlink/server.py': '# original inference', '.bundle-sha256': 'signed bundle identity'}.items():
    path = current / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data)
  for directory in ('cache/models', 'cache/engines', 'updater'):
    (runtime / directory).mkdir(parents=True)
  (runtime / 'updater/update_host.py').write_text('# unchanged signed updater')
  (runtime / 'updater/release-signing-public.pem').write_text('unchanged public key')
  (runtime / 'cache/models/cinque.onnx').write_bytes(b'preserve Cinque')
  (runtime / 'cache/last-loaded.json').write_text(json.dumps({'sha256': 'c' * 64, 'frame_skip': 4, 'backend': 'trt'}))
  (runtime / 'current').symlink_to('releases/' + overlay.BASE)
  events = []
  active = set(overlay.SERVICES)

  def service(action, names):
    events.append((action, tuple(names)))
    if action == 'stop':
      active.difference_update(names)
    elif action == 'start':
      active.update(names)
    else:
      assert action == 'is-active' and set(names) <= active

  monkeypatch.setattr(overlay, 'service', service)
  monkeypatch.setattr(overlay, 'active_services', lambda: [s for s in overlay.SERVICES if s in active])
  monkeypatch.setattr(pwd, 'getpwnam', lambda name: NS(pw_uid=os.getuid(), pw_gid=os.getgid()))
  monkeypatch.setattr(overlay, 'platform_check', lambda: None)
  monkeypatch.setattr(overlay, 'native_backend', lambda: None)
  def probe(candidate, runtime):
    assert not active
    assert overlay.selected(runtime) == current
    assert read(runtime / 'cache/last-loaded.json')['sha256'] == 'c' * 64
    overlay.verify_files(candidate)
    events.append(('probe',))
  return NS(source=source, tracked=tracked, revision=revision, spec=spec, bundle=bundle, built=built,
            model=model, plan=plan, meta=meta, runtime=runtime, current=current, probe=probe,
            events=events, active=active, service=service, before=overlay.snapshot(runtime))


def read(path):
  return json.loads(path.read_text())


def install(case, **overrides):
  arguments = {'runtime': case.runtime, 'bundle': case.bundle, 'bundle_sha256': overlay.digest(case.bundle),
               'sources': (case.model, case.plan, case.meta), 'plan_sha256': overlay.digest(case.plan),
               'meta_sha256': overlay.digest(case.meta), 'expected_current': case.current.name,
               'expected_tree': overlay.tree_hash(case.current), 'probe': case.probe}
  arguments.update(overrides)
  return overlay.install(**arguments)


def test_prepare_exact_identity_and_no_hud_or_controls(case):
  with tarfile.open(case.bundle) as archive:
    names = archive.getnames()
    assert 'tools/jetlink/hud.py' not in names
    assert all(overlay.allowed(name) for name in names if name != overlay.MANIFEST)
  for kwargs in ({'expected_revision': 'b' * 40}, {'expected_content': 'b' * 64}):
    arguments = {'root': case.source, 'output': case.source / 'other.tar.gz',
                 'expected_revision': case.revision, 'expected_content': case.built['content_sha256']}
    arguments.update(kwargs)
    with pytest.raises(ValueError, match='Source revision/content'):
      overlay.prepare(**arguments)
    assert not arguments['output'].exists()


@pytest.mark.parametrize('name', ['../escape.py', '/tmp/escape.py', 'tools/jetlink/hud.py',
                                'openpilot/selfdrive/controls/controlsd.py', 'tools/jetlink/update_host.py',
                                'third_party/jetlink/jetlink/tests/test.py'])
def test_allowlist_rejects_unrelated_paths(name):
  assert not overlay.allowed(name)


def test_untracked_required_runtime_is_not_silently_packaged(case):
  case.tracked.remove('third_party/jetlink/jetlink/server/backends/trt/state_buffers.py')
  with pytest.raises(ValueError, match='must be tracked'):
    overlay.source_identity(case.source)


def test_source_mutation_rejected(case):
  (case.source / overlay.SERVER).write_text('# changed')
  with pytest.raises(ValueError, match='Source revision/content'):
    overlay.prepare(case.source, case.source / 'new.tar.gz', case.revision, case.built['content_sha256'])


def test_install_copies_cache_preserves_base_hud_updater_and_reports_local_identity(case):
  updater = (case.runtime / 'updater/update_host.py').read_bytes()
  result = install(case)
  selected = overlay.selected(case.runtime)
  assert selected != case.current and selected.name.startswith('hl-n16-')
  assert (selected / 'SOURCE_COMMIT').read_text() == overlay.BASE
  assert (selected / 'tools/jetlink/hud.py').read_bytes() == (case.current / 'tools/jetlink/hud.py').read_bytes()
  assert (case.runtime / 'updater/update_host.py').read_bytes() == updater
  assert (case.runtime / 'updater/release-signing-public.pem').read_text() == 'unchanged public key'
  assert (case.runtime / 'cache/models/cinque.onnx').read_bytes() == b'preserve Cinque'
  assert overlay.tree_hash(case.current) == case.before['tree_sha256']
  assert not (case.runtime / overlay.TRANSACTION).exists()
  assert case.active == set(overlay.SERVICES)
  for src, dst in zip((case.model, case.plan, case.meta), overlay.cache_paths(case.runtime), strict=True):
    assert src.read_bytes() == dst.read_bytes()
    assert src.stat().st_ino != dst.stat().st_ino
  assert result == overlay.verify_runtime(selected) == {
    'model': overlay.MODEL, 'content_sha256': case.built['content_sha256'], 'base_source_commit': overlay.BASE}
  assert case.events.index(('probe',)) < next(i for i, call in enumerate(case.events) if call[0] == 'start')


def test_unoverlaid_verify_does_not_import_runtime_or_modify_files(case, monkeypatch):
  monkeypatch.setattr(overlay, 'platform_check', lambda: pytest.fail('unoverlaid host touched'))
  before = overlay.tree_hash(case.runtime)
  assert overlay.verify_runtime(case.current) is None
  assert before == overlay.tree_hash(case.runtime)


@pytest.mark.parametrize('kind', ['owner', 'permissions'])
def test_startup_requires_root_owned_nonwritable_overlay(case, monkeypatch, kind):
  install(case)
  current = overlay.selected(case.runtime)
  if kind == 'owner':
    monkeypatch.setattr(overlay, 'RELEASE_UID', os.getuid() + 1000)
  else:
    (current / overlay.SERVER).chmod(0o666)
  with pytest.raises(ValueError, match='root-owned'):
    overlay.verify_runtime(current)


@pytest.mark.parametrize('kind', ['overlay', 'base', 'plan', 'meta', 'model', 'selection'])
def test_startup_rejects_corrupt_files_or_cache(case, kind):
  install(case)
  current = overlay.selected(case.runtime)
  model, plan, meta = overlay.cache_paths(case.runtime)
  targets = {'overlay': current / overlay.SERVER, 'base': current / 'tools/jetlink/hud.py',
             'plan': plan, 'meta': meta, 'model': model, 'selection': case.runtime / 'cache/last-loaded.json'}
  targets[kind].write_bytes(b'corrupt')
  with pytest.raises((ValueError, KeyError)):
    overlay.verify_runtime(current)


def test_probe_failure_restores_model_services_and_current(case):
  def fail(*args):
    raise RuntimeError('probe failed')
  with pytest.raises(RuntimeError, match='probe failed'):
    install(case, probe=fail)
  assert overlay.snapshot(case.runtime) == case.before
  assert not (case.runtime / overlay.TRANSACTION).exists()


def test_keyboard_interrupt_recovers(case):
  def interrupt(*args):
    raise KeyboardInterrupt()
  with pytest.raises(KeyboardInterrupt):
    install(case, probe=interrupt)
  assert overlay.snapshot(case.runtime) == case.before


def test_service_start_failure_after_selection_rolls_back(case, monkeypatch):
  starts = []
  def fail_once(action, names):
    if action == 'start':
      starts.append(overlay.selected(case.runtime).name)
      if len(starts) == 1:
        raise RuntimeError('service failed')
    return case.service(action, names)
  monkeypatch.setattr(overlay, 'service', fail_once)
  with pytest.raises(RuntimeError, match='service failed'):
    install(case)
  assert starts[0].startswith('hl-n16-') and starts[1] == overlay.BASE
  assert overlay.snapshot(case.runtime) == case.before


def test_inactive_hud_stays_inactive(case):
  case.active.remove(overlay.SERVICES[0])
  install(case)
  assert case.active == {'carrot-jetlink.service'}


def test_explicit_rollback_retains_version_and_cinque(case):
  install(case)
  current = overlay.selected(case.runtime)
  overlay.rollback(case.runtime, current.name, overlay.tree_hash(current))
  assert overlay.snapshot(case.runtime) == case.before
  assert current.exists()
  assert all(path.exists() for path in overlay.cache_paths(case.runtime))


@pytest.mark.parametrize('point', ['before_select', 'after_select'])
def test_durable_interruption_requires_recover(case, point):
  install(case)
  current = overlay.selected(case.runtime)
  journal = {'format': 1, 'restore': case.before, 'allowed_current': [case.current.name, current.name]}
  overlay.atomic_json(case.runtime / overlay.TRANSACTION, journal)
  if point == 'before_select':
    overlay.switch(case.runtime, case.current)
  else:
    with pytest.raises(ValueError, match='Interrupted'):
      overlay.verify_runtime(current)
  overlay.recover(case.runtime)
  assert overlay.snapshot(case.runtime) == case.before
  assert not (case.runtime / overlay.TRANSACTION).exists()


def test_failed_recovery_keeps_durable_journal(case, monkeypatch):
  def fail(*args):
    raise RuntimeError('cannot stop')
  monkeypatch.setattr(overlay, 'service', fail)
  with pytest.raises(RuntimeError, match='cannot stop'):
    install(case)
  assert read(case.runtime / overlay.TRANSACTION)['restore'] == case.before


def test_new_signed_base_supersedes_overlay_and_is_never_rolled_back(case):
  install(case)
  old = overlay.selected(case.runtime)
  new = case.runtime / 'releases' / ('d' * 40)
  new.mkdir()
  (new / 'SOURCE_COMMIT').write_text('d' * 40)
  overlay.switch(case.runtime, new)
  assert overlay.verify_runtime(new) is None
  with pytest.raises(ValueError, match='superseded'):
    overlay.rollback(case.runtime, old.name, overlay.tree_hash(old))
  overlay.atomic_json(case.runtime / overlay.TRANSACTION, {'restore': case.before,
                       'allowed_current': [case.current.name, old.name]})
  with pytest.raises(ValueError, match='superseded'):
    overlay.recover(case.runtime)
  assert overlay.selected(case.runtime) == new


@pytest.mark.parametrize('field,value', [('backend', 'ort'), ('trt_version', '10.4.0'), ('device', 'Orin-sm90'),
                                       ('spec', {}), ('source_sha256', 'f' * 64)])
def test_cache_metadata_mismatch_before_service_stop(case, field, value):
  facts = read(case.meta)
  facts[field] = value
  case.meta.write_text(json.dumps(facts))
  with pytest.raises(ValueError, match='metadata|source'):
    install(case)
  assert case.events == []
  assert overlay.snapshot(case.runtime) == case.before


def test_native_sidecar_without_source_sha_is_supported(case):
  assert 'source_sha256' not in read(case.meta)
  overlay.check_cache(case.model, case.plan, case.meta, case.spec, overlay.digest(case.plan), overlay.digest(case.meta))


@pytest.mark.parametrize('arg', ['plan_sha256', 'meta_sha256', 'expected_tree', 'bundle_sha256'])
def test_expected_hashes_cannot_be_skipped(case, arg):
  with pytest.raises(ValueError):
    install(case, **{arg: 'f' * 64})
  assert case.events == []


def test_existing_corrupt_cache_is_not_replaced(case):
  _, plan, _ = overlay.cache_paths(case.runtime)
  plan.write_bytes(b'old unknown cache')
  with pytest.raises(ValueError, match='Existing N16 cache differs'):
    install(case)
  assert plan.read_bytes() == b'old unknown cache'
  assert overlay.snapshot(case.runtime) == case.before


def test_symlink_cache_destination_rejected(case):
  _, plan, _ = overlay.cache_paths(case.runtime)
  plan.symlink_to(case.plan)
  with pytest.raises(ValueError, match='regular file'):
    install(case)
  assert overlay.snapshot(case.runtime) == case.before


@pytest.mark.parametrize('unsafe', ['../escape.py', 'tools/jetlink/hud.py', 'symlink', 'duplicate'])
def test_unsafe_archive_rejected(case, tmp_path, unsafe):
  bundle = tmp_path / 'unsafe.tar.gz'
  with tarfile.open(case.bundle) as original, tarfile.open(bundle, 'w:gz') as archive:
    for member in original:
      archive.addfile(member, original.extractfile(member))
    member = tarfile.TarInfo(overlay.SERVER if unsafe in ('symlink', 'duplicate') else unsafe)
    if unsafe == 'symlink':
      member.type, member.linkname = tarfile.SYMTYPE, '/tmp/escape'
      archive.addfile(member)
    else:
      member.size = 1
      archive.addfile(member, io.BytesIO(b'x'))
  with pytest.raises(ValueError, match='Unsafe'):
    overlay.extract(bundle, tmp_path / 'out', overlay.digest(bundle))


@pytest.mark.parametrize('failure', ['timeout', 'interrupt', 'nonzero'])
def test_bounded_child_failure_kills_group_and_reaps(monkeypatch, failure):
  calls = []
  error = subprocess.TimeoutExpired('probe', 3) if failure == 'timeout' else KeyboardInterrupt()
  class Process:
    pid = 123
    def wait(self, timeout):
      calls.append(('wait', timeout))
      if len(calls) == 1:
        if failure == 'nonzero':
          return 7
        raise error
      return -9
  def spawn(command, **kwargs):
    assert kwargs == {'start_new_session': True, 'cwd': '/tmp'}
    return Process()
  monkeypatch.setattr(overlay.subprocess, 'Popen', spawn)
  monkeypatch.setattr(overlay.os, 'killpg', lambda pid, sig: calls.append(('kill', pid, sig)))
  with pytest.raises((subprocess.TimeoutExpired, KeyboardInterrupt, subprocess.CalledProcessError)):
    overlay.run_child(['native-probe'], 3)
  assert calls == [('wait', 3), ('kill', 123, signal.SIGKILL), ('wait', 10)]


def test_invalid_probe_timeout_never_spawns(monkeypatch):
  monkeypatch.setattr(overlay.subprocess, 'Popen', lambda *a, **k: pytest.fail('spawned'))
  for timeout in (0, -1, overlay.MAX_PROBE_SECONDS + 1):
    with pytest.raises(ValueError, match='timeout'):
      overlay.run_child(['probe'], timeout)


def test_full_spec_change_rejected_even_with_model_identity(case):
  spec = deepcopy(case.spec)
  spec['input_shapes'] = {'wrong': [1]}
  with pytest.raises(ValueError, match='full spec'):
    overlay.check_cache(case.model, case.plan, case.meta, spec, overlay.digest(case.plan), overlay.digest(case.meta))
