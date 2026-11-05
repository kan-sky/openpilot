"""Isolated N16 TensorRT bench; no vehicle messages, model selection or USB owner."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]
PRODUCTION_CACHE = Path('/opt/carrot-jetlink/cache')
BUILD_TIMEOUT = 1200


def _experiment_cache(cache, spec):
  cache = cache.resolve()
  production = PRODUCTION_CACHE.resolve()
  experiments = production / 'experiments'
  storage = tuple((production / name).resolve() for name in ('engines', 'models', 'last-loaded.json'))
  if (cache == production or cache == experiments or
      (cache.is_relative_to(production) and not cache.is_relative_to(experiments)) or
      any(cache.is_relative_to(path) for path in storage)):
    raise ValueError('Use a separate experiment cache, not the live service cache')
  for path in (cache, *(cache / name for name in ('engines', 'models', 'last-loaded.json'))):
    if path.resolve() == production or any(path.resolve().is_relative_to(target) for target in storage):
      raise ValueError('Use a separate experiment cache, not the live service cache')
    if path.is_symlink():
      raise ValueError('Experiment cache must not share service files or directories')
  previous = cache / 'last-loaded.json'
  if previous.exists():
    previous = json.loads(previous.read_text())
    if previous.get('sha256') != spec.sha256:
      raise ValueError('Experiment cache belongs to a different model')
  for path in (cache / 'engines').glob(f'{spec.sha256[:16]}.*'):
    if path.is_symlink():
      raise ValueError('Experiment engine must not alias another cache')
  return cache


def _check_prepared(entry, spec):
  try:
    if not entry.path.is_file() or entry.path.stat().st_size == 0:
      raise ValueError('Missing or empty TensorRT plan')
    meta = entry.meta()
    if meta.get('backend') != 'trt' or meta.get('spec') != spec.to_dict():
      raise ValueError('Full model spec does not match the N16 pin')
  except (OSError, ValueError, TypeError, AttributeError) as error:
    raise ValueError(f'Invalid prepared N16 cache: {error}') from error


def prepare_cache(model, cache, spec, timeout=BUILD_TIMEOUT):
  """Build only in an exec child, including cache lookup, before touching CUDA."""
  from jetlink.server.cache import EngineCache

  if not 0 < timeout <= BUILD_TIMEOUT:
    raise ValueError('N16 build timeout must be positive and at most 1200 seconds')
  cache = _experiment_cache(cache, spec)
  hasher = hashlib.sha256()
  with model.open('rb') as stream:
    for block in iter(lambda: stream.read(4 << 20), b''):
      hasher.update(block)
  if hasher.hexdigest() != spec.sha256 or model.stat().st_size != spec.nbytes:
    raise ValueError('N16 model integrity mismatch')
  # EngineCache without a backend can locate the model without importing TRT.
  target = EngineCache(cache).model_path(spec.sha256)
  if target.is_symlink():
    raise ValueError('Experiment model must not alias another cache')
  if not target.exists():
    os.link(model, target)
  elif not os.path.samefile(target, model):
    raise ValueError('Unexpected experiment model already exists')

  env = os.environ.copy()
  env['PYTHONPATH'] = os.pathsep.join(filter(None, (
    str(ROOT / 'third_party/jetlink'), str(ROOT), env.get('PYTHONPATH'))))
  command = [sys.executable, '-m', 'jetlink.server.main', '--backend', 'trt',
             '--cache', str(cache), '--build', str(target), '--parent-pid', str(os.getpid())]
  # Inherit the log streams: no pipe can fill or retain the build log in RAM.
  try:
    subprocess.run(command, cwd=ROOT, env=env, check=True, timeout=timeout)
  except subprocess.TimeoutExpired as error:
    # subprocess.run kills and waits for the child before raising.
    raise TimeoutError(f'N16 cache build exceeded {timeout:g} seconds') from error
  except subprocess.CalledProcessError as error:
    raise RuntimeError(f'N16 cache build exited with status {error.returncode}') from error

  # The builder (and its native allocator) is fully gone before the parent
  # imports TensorRT or resolves the device-specific cache key.
  from jetlink.server.backends.trt import TrtBackend
  engine_cache = EngineCache(cache, TrtBackend())
  _check_prepared(engine_cache.entry(spec.sha256), spec)
  return engine_cache


def cache_only_host(engine_cache):
  from jetlink.server.session import EngineHost

  class CacheOnlyHost(EngineHost):
    def _build_job(self, *args, **kwargs):
      raise RuntimeError('N16 bench forbids in-process builds; prepare the cache again')

  return CacheOnlyHost(engine_cache)


def summarize(samples):
  import numpy as np
  values = np.asarray(samples)
  return {'frames': len(values), 'mean_ms': float(values.mean()), 'p95_ms': float(np.percentile(values, 95)),
          'max_ms': float(values.max()), 'over_50ms': int((values > 50).sum())}


def run(model, cache, frames):
  from jetlink.spec import ModelSpec
  from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec, parse_outputs

  spec = validate_model_spec(ModelSpec.from_dict(json.loads(
    (ROOT / 'openpilot/selfdrive/modeld/jetlink/n16.json').read_text())))
  started = time.monotonic()
  engine_cache = prepare_cache(model, cache, spec)
  prepared = time.monotonic()

  import numpy as np
  from jetlink.server.session import Request
  from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser

  host = cache_only_host(engine_cache)
  report = {'scope': 'synthetic inputs on native TensorRT; no USB, camera or driving validation',
            'model': spec.sha256, 'frames_requested': frames, 'completed': False,
            'cache_prepare_seconds': prepared - started}
  try:
    host.request(Request(spec.sha256, spec.nbytes, spec.frame_skip), None)
    deadline = time.monotonic() + 1200
    while time.monotonic() < deadline:
      state = host.status(spec.sha256, spec.frame_skip)
      if state['state'] == 'ready':
        break
      if state['state'] in ('failed', 'need_upload'):
        raise RuntimeError(state.get('detail', state['state']))
      time.sleep(1)
    else:
      raise TimeoutError('N16 cached load exceeded 20 minutes')
    report['build_load_seconds'] = time.monotonic() - started
    loaded = host.loaded
    if loaded.spec != spec:
      raise ValueError('Compiled model spec does not match the N16 pin')
    parser = Parser()
    packed = np.zeros(spec.packed_nelem, np.float32)
    views = {name: array.reshape(shape) for (name, shape), array in zip(
      spec.packed_shapes.items(), np.split(packed, np.cumsum(spec.packed_sizes[:-1])), strict=True)}
    prev_desire = np.zeros(8, np.float32)
    images = np.full(spec.warped_shape, 128, np.uint8)
    inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
              'action_t': np.array([.2, .3], np.float32)}

    def frame():
      fill_inputs(views, inputs, prev_desire)
      begin = time.monotonic()
      loaded.queues.step_into(images, packed, loaded.host_inputs)
      outputs = loaded.engine.run()
      loaded.queues.feedback(outputs)
      values = np.asarray(outputs['outputs'], dtype=np.float32).reshape(-1)
      if values.shape != (spec.output_nelem,) or not np.isfinite(values).all():
        raise ValueError('Invalid native N16 output')
      parsed = parse_outputs(parser, spec, values)
      if parsed['plan'].shape != (1, 33, 15) or not all(np.isfinite(v).all() for v in parsed.values()):
        raise ValueError('Invalid parsed driving output')
      return (time.monotonic() - begin) * 1000, values.copy()

    loaded.queues.reset()
    _, first = frame()
    for _ in range(30):
      frame()
    rows = []
    for index in range(frames):
      # Exercise absent, active, then removed navigation without synthetic IPC.
      phase = (index * 3) // frames
      if phase == 1:
        inputs['nav_map'] = np.full((256, 256), index % 256, np.uint8)
        inputs['navigation'] = np.zeros(47, np.float32)
        inputs['navigation'][0] = 1
      else:
        inputs.pop('nav_map', None)
        inputs.pop('navigation', None)
      elapsed, _ = frame()
      rows.append(elapsed)
      if index % 100 == 0:
        print(json.dumps({'frame': index, 'ms': elapsed}), flush=True)
    loaded.queues.reset()
    prev_desire.fill(0)
    inputs.pop('nav_map', None)
    inputs.pop('navigation', None)
    _, reset = frame()
    report.update(timing=summarize(rows), reset_max_abs=float(np.max(np.abs(first - reset))),
                  all_finite=True, completed=True)
    if not np.allclose(first, reset, rtol=1e-5, atol=1e-5):
      raise ValueError('Native recurrent reset did not reproduce the first frame')
    return report
  finally:
    host.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--model', type=Path, required=True)
  parser.add_argument('--cache', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--frames', type=int, default=600)
  args = parser.parse_args()
  if not 60 <= args.frames <= 6000:
    parser.error('--frames must be between 60 and 6000')
  report = {'completed': False, 'error': 'benchmark interrupted before completion'}
  try:
    report = run(args.model, args.cache, args.frames)
  except Exception as error:
    report = {'completed': False, 'error': str(error)}
    raise
  finally:
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
  main()
