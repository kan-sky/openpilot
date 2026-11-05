"""Bench-only cache preparation, without TensorRT, CUDA or device access."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.jetlink import bench_n16 as bench
from jetlink.server.cache import CacheEntry
from jetlink.server.session import EngineHost, Job, Request
from jetlink.spec import ModelSpec


@pytest.fixture
def case(tmp_path, monkeypatch):
  model = tmp_path / 'n16.onnx'
  model.write_bytes(b'mocked N16 model; never executed')
  pin = ModelSpec.from_dict(json.loads((Path(__file__).parent / 'fixtures/n16.json').read_text()))
  spec = replace(pin, sha256=hashlib.sha256(model.read_bytes()).hexdigest(), nbytes=model.stat().st_size)
  cache = tmp_path / 'experiment'
  stem = cache / 'engines' / f'{spec.sha256[:16]}.trt-test'
  entry = CacheEntry(Path(str(stem) + '.plan'), Path(str(stem) + '.json'))
  events, calls = [], []
  state = NS(model=model, spec=spec, cache=cache, entry=entry, events=events, calls=calls,
             result=0, error=None, reaped=False)

  def write_plan(meta=None):
    entry.path.parent.mkdir(parents=True, exist_ok=True)
    entry.path.write_bytes(b'mocked plan')
    entry.write_meta({'backend': 'trt', 'spec': spec.to_dict()} if meta is None else meta)

  state.write_plan = write_plan
  state.build = write_plan

  def run(command, **kwargs):
    calls.append((command, kwargs))
    events.append('spawn')
    assert kwargs['check'] is True and kwargs['timeout'] > 0
    # subprocess.run's timeout contract kills and waits before it raises.
    if state.build is not None and state.error is None:
      state.build()
    state.reaped = True
    events.append('reaped')
    if state.error is not None:
      raise state.error
    if state.result:
      raise subprocess.CalledProcessError(state.result, command)
    return subprocess.CompletedProcess(command, 0)

  class Backend:
    name = 'trt'
    suffix = '.plan'

    def __init__(self):
      assert state.reaped, 'Parent touched TRT before reaping the builder'
      events.append('backend')

    def tag(self):
      return 'trt-test'

    def build(self, *args, **kwargs):
      pytest.fail('Parent must never build')

  native = ModuleType('jetlink.server.backends.trt')
  native.TrtBackend = Backend
  monkeypatch.setitem(sys.modules, native.__name__, native)
  monkeypatch.setattr(bench.subprocess, 'run', run)
  monkeypatch.setattr(bench, 'PRODUCTION_CACHE', tmp_path / 'production')
  return state


def test_prepare_success_reaps_before_backend_and_host_load(case, monkeypatch):
  monkeypatch.setenv('PYTHONPATH', '/existing/python')
  cache = bench.prepare_cache(case.model, case.cache, case.spec, timeout=23)
  command, options = case.calls[0]
  target = cache.model_path(case.spec.sha256)
  assert command == [sys.executable, '-m', 'jetlink.server.main', '--backend', 'trt',
                     '--cache', str(case.cache), '--build', str(target),
                     '--parent-pid', str(bench.os.getpid())]
  assert target.samefile(case.model)
  assert options['check'] is True and options['timeout'] == 23
  assert options['cwd'] == ROOT
  assert options['env']['PYTHONPATH'].split(bench.os.pathsep) == [
    str(ROOT / 'third_party/jetlink'), str(ROOT), '/existing/python']
  assert 'stdout' not in options and 'stderr' not in options
  assert case.events == ['spawn', 'reaped', 'backend']

  host = bench.cache_only_host(cache)
  engine = NS(close=lambda: None)

  def load(path):
    assert case.reaped and path == case.entry.path
    case.events.append('load')
    return engine

  monkeypatch.setattr(host, '_load_engine', load)
  monkeypatch.setattr(host, '_warm', lambda engine, spec: NS(sha256=spec.sha256, spec=spec, engine=engine))
  try:
    job = Job(case.spec.sha256, load_only=True)
    host._run(job, Request(case.spec.sha256, case.spec.nbytes, case.spec.frame_skip),
              case.entry, target, case.spec)
    assert job.state == 'ready'
    assert case.events.index('reaped') < case.events.index('load')
    assert cache.last_loaded() == (case.spec.sha256, case.spec.frame_skip)
  finally:
    host.close()


def test_cache_hit_still_validated_without_rewriting(case):
  case.write_plan()
  before = [(p.read_bytes(), p.stat().st_mtime_ns) for p in (case.entry.path, case.entry.meta_path)]
  # The existing --build entrypoint returns immediately for a complete hit.
  case.build = None
  cache = bench.prepare_cache(case.model, case.cache, case.spec)
  assert cache.entry(case.spec.sha256) == case.entry
  assert [(p.read_bytes(), p.stat().st_mtime_ns) for p in (case.entry.path, case.entry.meta_path)] == before
  assert len(case.calls) == 1
  assert case.events.index('reaped') < case.events.index('backend')


@pytest.mark.parametrize('error', [subprocess.TimeoutExpired('build', 3), KeyboardInterrupt()])
def test_interrupted_build_does_not_load(case, error):
  case.error = error
  expected = TimeoutError if isinstance(error, subprocess.TimeoutExpired) else KeyboardInterrupt
  with pytest.raises(expected):
    bench.prepare_cache(case.model, case.cache, case.spec, timeout=3)
  assert case.reaped
  assert case.events == ['spawn', 'reaped']
  assert case.calls[0][1]['timeout'] == 3


@pytest.mark.parametrize('code', [1, -9])
def test_failed_child_never_loads_even_with_complete_cache(case, code):
  case.result = code
  with pytest.raises(RuntimeError, match=f'exited with status {code}'):
    bench.prepare_cache(case.model, case.cache, case.spec)
  assert case.reaped and 'backend' not in case.events


@pytest.mark.parametrize('field,value', [
  ('sha256', '0' * 64), ('nbytes', 1), ('frame_skip', 1), ('checkpoint', 'different'),
  ('input_shapes', {}), ('output_shapes', {}), ('output_slices', {}),
])
def test_full_spec_mismatch_rejected_for_build_and_cache_hit(case, field, value):
  meta = {'backend': 'trt', 'spec': deepcopy(case.spec.to_dict())}
  meta['spec'][field] = value
  case.build = lambda: case.write_plan(meta)
  with pytest.raises(ValueError, match='Full model spec'):
    bench.prepare_cache(case.model, case.cache, case.spec)
  case.build = None
  with pytest.raises(ValueError, match='Full model spec'):
    bench.prepare_cache(case.model, case.cache, case.spec)


@pytest.mark.parametrize('bad', ['no_plan', 'empty_plan', 'no_meta', 'invalid_json', 'not_object', 'no_spec', 'wrong_backend'])
def test_incomplete_or_bad_metadata_rejected(case, bad):
  def build():
    case.write_plan()
    if bad == 'no_plan':
      case.entry.path.unlink()
    elif bad == 'empty_plan':
      case.entry.path.write_bytes(b'')
    elif bad == 'no_meta':
      case.entry.meta_path.unlink()
    elif bad == 'invalid_json':
      case.entry.meta_path.write_text('{')
    elif bad == 'not_object':
      case.entry.meta_path.write_text('[]')
    elif bad == 'no_spec':
      case.entry.write_meta({'backend': 'trt'})
    elif bad == 'wrong_backend':
      case.entry.write_meta({'backend': 'ort', 'spec': case.spec.to_dict()})

  case.build = build
  with pytest.raises(ValueError, match='Invalid prepared N16 cache'):
    bench.prepare_cache(case.model, case.cache, case.spec)
  assert case.reaped


@pytest.mark.parametrize('kind', [
  'root', 'descendant', 'root_alias', 'storage_alias', 'engines', 'models', 'last-loaded.json', 'plan',
])
def test_production_cache_guard_before_any_process_or_model_link(case, kind):
  production = bench.PRODUCTION_CACHE
  production.mkdir()
  marker = production / 'untouched'
  marker.write_bytes(b'production')
  path = case.cache
  if kind == 'root':
    path = production
  elif kind == 'descendant':
    path = production / 'experiment'
  elif kind == 'root_alias':
    path.symlink_to(production, target_is_directory=True)
  elif kind == 'storage_alias':
    path.symlink_to(production / 'engines', target_is_directory=True)
  else:
    path.mkdir()
    if kind == 'plan':
      case.entry.path.parent.mkdir()
      case.entry.path.symlink_to(marker)
    else:
      (path / kind).symlink_to(production, target_is_directory=True)
  with pytest.raises(ValueError, match='(separate experiment cache|alias another cache)'):
    bench.prepare_cache(case.model, path, case.spec)
  assert case.calls == []
  assert list(production.iterdir()) == [marker]
  assert marker.read_bytes() == b'production'


def test_nested_experiment_cache_accepted_without_touching_production_storage(case):
  production = bench.PRODUCTION_CACHE
  (production / 'engines').mkdir(parents=True)
  (production / 'models').mkdir()
  sentinel = production / 'engines/cinque.plan'
  sentinel.write_bytes(b'production plan')
  remembered = production / 'last-loaded.json'
  remembered.write_text(json.dumps({'sha256': 'c' * 64, 'frame_skip': 4}))
  before = remembered.read_bytes()
  nested = production / 'experiments/n16-20261021/cold-cache'
  entry = CacheEntry(nested / 'engines' / case.entry.path.name, nested / 'engines' / case.entry.meta_path.name)

  def build():
    entry.path.parent.mkdir(parents=True, exist_ok=True)
    entry.path.write_bytes(b'candidate plan')
    entry.write_meta({'backend': 'trt', 'spec': case.spec.to_dict()})

  case.build = build
  result = bench.prepare_cache(case.model, nested, case.spec)
  assert result.root == nested
  assert result.entry(case.spec.sha256) == entry
  assert sentinel.read_bytes() == b'production plan'
  assert remembered.read_bytes() == before
  assert list((production / 'models').iterdir()) == []
  assert '--cache' in case.calls[0][0] and str(nested) in case.calls[0][0]


def test_foreign_model_cache_rejected_before_process(case):
  case.cache.mkdir()
  (case.cache / 'last-loaded.json').write_text(json.dumps({'sha256': '0' * 64}))
  with pytest.raises(ValueError, match='different model'):
    bench.prepare_cache(case.model, case.cache, case.spec)
  assert case.calls == []


def test_model_integrity_checked_before_cache_creation(case):
  case.model.write_bytes(b'changed')
  with pytest.raises(ValueError, match='integrity mismatch'):
    bench.prepare_cache(case.model, case.cache, case.spec)
  assert not case.cache.exists() and not case.calls


@pytest.mark.parametrize('timeout', [0, -1, 1201, float('nan'), float('inf')])
def test_invalid_timeout_rejected_before_process(case, timeout):
  with pytest.raises(ValueError, match='build timeout'):
    bench.prepare_cache(case.model, case.cache, case.spec, timeout=timeout)
  assert case.calls == []


def test_cache_only_host_does_not_fall_back_to_parent_build(case):
  cache = bench.prepare_cache(case.model, case.cache, case.spec)
  original_build = EngineHost._build_job
  host = bench.cache_only_host(cache)
  try:
    with pytest.raises(RuntimeError, match='forbids in-process builds'):
      host._build_job(None, None, None, None)
    assert EngineHost._build_job is original_build
    job = Job(case.spec.sha256, load_only=False)
    host._run(job, Request(case.spec.sha256, case.spec.nbytes, case.spec.frame_skip),
              case.entry, case.model, case.spec)
    assert job.state == 'failed' and 'forbids in-process builds' in job.detail
    assert host.loaded is None
  finally:
    host.close()
