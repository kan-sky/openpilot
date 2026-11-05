"""Comma packing and fallback regression; no hardware or driving publication."""
from concurrent.futures import Future
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import warnings

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink'), str(Path(__file__).parent)]
from jetlink.spec import ModelSpec
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from openpilot.selfdrive.modeld.jetlink import model
from test_hotplug_resources import joining_model

SPEC = ModelSpec.from_dict(json.loads((ROOT / 'openpilot/selfdrive/modeld/jetlink/n16.json').read_text()))


def buffers():
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  views = {name: array.reshape(shape) for (name, shape), array in zip(
    SPEC.packed_shapes.items(), np.split(packed, np.cumsum(SPEC.packed_sizes[:-1])), strict=True)}
  return packed, views


def frame_inputs():
  return {'desire_pulse': np.eye(8, dtype=np.float32)[2], 'traffic_convention': np.array([1, 0], np.float32),
          'action_t': np.array([.2, .3], np.float32),
          'nav_map': np.arange(65536, dtype=np.uint8).reshape(256, 256),
          'navigation': np.arange(47, dtype=np.float32)}


def test_map_pixels_unnormalized_and_missing_fields_clear():
  packed, views = buffers()
  previous = np.zeros(8, np.float32)
  inputs = frame_inputs()
  fill_inputs(views, inputs, previous)
  assert packed.size == 65595
  np.testing.assert_array_equal(views['nav_map'][0, 0], inputs['nav_map'])
  assert views['nav_map'].max() == 255 and views['desire'][2] == 1
  del inputs['nav_map']
  fill_inputs(views, inputs, previous)
  assert not views['nav_map'].any() and not views['desire'].any()
  assert views['navigation'].any()
  del inputs['navigation']
  fill_inputs(views, inputs, previous)
  assert not packed[12:].any()


@pytest.mark.parametrize('name,value', [
  ('nav_map', np.ones(1)), ('navigation', np.ones(46)),
  ('nav_map', np.full((256, 256), 256.)), ('nav_map', np.full((256, 256), -1.)),
  ('nav_map', np.full((256, 256), np.inf)), ('navigation', np.full(47, np.nan)),
])
def test_bad_inputs_rejected(name, value):
  _, views = buffers()
  inputs = frame_inputs()
  inputs[name] = value
  with pytest.raises(ValueError):
    fill_inputs(views, inputs, np.zeros(8, np.float32))


@pytest.mark.parametrize('name', ['nav_map', 'navigation'])
@pytest.mark.parametrize('dtype,value', [
  (np.complex64, 1 + 2j), (np.complex128, 1 + 0j),
  (np.bool_, True), (object, 1), (str, '1'),
  ('timedelta64[s]', 1), ('datetime64[s]', 1),
  (np.float64, 1e300), (np.float64, -1e300),
])
def test_invalid_optional_conversion_clears_old_buffer_without_warnings(name, dtype, value):
  _, views = buffers()
  previous = np.zeros(8, np.float32)
  inputs = frame_inputs()
  fill_inputs(views, inputs, previous)
  assert views[name].any()
  inputs[name] = np.full(views[name].shape, value, dtype=dtype)
  with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter('always')
    with pytest.raises(ValueError, match=f'Jetlink .*{name}|Jetlink map pixels'):
      fill_inputs(views, inputs, previous)
  assert not caught
  assert not views[name].any()


@pytest.mark.parametrize('dtype', [np.uint8, np.int64, np.float16, np.float64])
def test_real_optional_numeric_inputs_remain_supported(dtype):
  _, views = buffers()
  inputs = frame_inputs()
  for name in ('nav_map', 'navigation'):
    inputs[name] = inputs[name].astype(dtype)
  fill_inputs(views, inputs, np.zeros(8, np.float32))
  for name in ('nav_map', 'navigation'):
    np.testing.assert_array_equal(views[name].ravel(), inputs[name].ravel())
    assert views[name].dtype == np.float32


def test_joining_adapter_nav_clear_and_usb_fault_fallback(monkeypatch, tmp_path):
  joining = joining_model(monkeypatch)
  received = []

  def infer(images, packed, frame, reset, source_sof):
    received.append(packed.copy())
    if len(received) == 3:
      raise ConnectionError('lost USB')
    return np.zeros(SPEC.output_nelem, np.float32)

  client = NS(spec=SPEC, infer=infer, timings=(0, 0, 0), close=lambda: None)
  future = Future()
  future.set_result(client)
  joining.connection = NS(future=future)

  class Warp:
    timings = (0, 0, 0)

    def __call__(self, *args):
      return np.zeros(SPEC.warped_shape, np.uint8)

  joining.warp = Warp()
  joining.small.reset = lambda: None
  monkeypatch.setattr(model, 'FAULT', tmp_path / 'fault')
  inputs = frame_inputs()
  assert joining.run({}, {}, inputs, False) == 'external'
  np.testing.assert_array_equal(received[0][12:12 + 65536], inputs['nav_map'].ravel())
  np.testing.assert_array_equal(received[0][-47:], inputs['navigation'])
  del inputs['nav_map'], inputs['navigation']
  assert joining.run({}, {}, inputs, False) == 'external'
  assert not received[1][12:].any()
  assert joining.run({}, {}, inputs, False) == 'local'
  assert not joining.active and joining.client is None and not joining.output_valid
  assert (tmp_path / 'fault').exists()


def test_status_write_failure_does_not_interrupt_model(monkeypatch, tmp_path):
  joining = joining_model(monkeypatch)
  joining.spec = SPEC
  joining.phase_enabled = False
  joining.active = True
  monkeypatch.setattr(model, 'MODEL_STATUS', tmp_path / 'missing' / 'state.json')
  monkeypatch.setattr(model, 'state', lambda: {'state': 'ready'})
  monkeypatch.setattr(model, 'may_join', lambda *a: True)

  class SM(dict):
    valid = alive = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), True)
    recv_time = {}

  joining.update(SM.fromkeys(SM.valid, NS()), NS(timestamp_sof=0))
  assert joining.active and joining.ready


def test_raw_prediction_is_copied_before_destructive_parser(monkeypatch):
  joining = joining_model(monkeypatch)
  original = np.arange(SPEC.output_nelem, dtype=np.float32)
  joining.client = NS(infer=lambda *a, **kw: original.copy(), timings=(0, 0, 0))
  joining.spec = SPEC
  joining.packed, joining.views = buffers()
  class Warp:
    timings = (0, 0, 0)
    def __call__(self, *args):
      return np.zeros(SPEC.warped_shape, np.uint8)
  joining.warp = Warp()
  def parse(parser, spec, values):
    values[:] = 0
    return {}
  monkeypatch.setattr(model, 'parse_outputs', parse)
  monkeypatch.setenv('SEND_RAW_PRED', '1')
  output = joining.run({}, {}, frame_inputs(), False)
  np.testing.assert_array_equal(output['raw_pred'], original)
