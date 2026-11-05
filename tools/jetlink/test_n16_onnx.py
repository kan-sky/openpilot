"""Byte-queue patch equivalence; deliberately excludes model-weight/GPU parity."""
from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

onnx = pytest.importorskip('onnx')
from onnx import TensorProto as T, helper as H, numpy_helper
from onnx.reference import ReferenceEvaluator

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'third_party/jetlink')]
from jetlink.onnx_patch import needs_patch, patch_uint8_inputs


def queue_graph():
  # Bounded copy of the upstream byte-queue dataflow, with all 256 byte values.
  init = [numpy_helper.from_array(np.array(value, np.int64), name) for name, value in
          [('start', [1]), ('end', [5]), ('axis', [1]), ('step', [1]), ('zero', 0), ('flat', [5, 256])]]
  nodes = [H.make_node('Slice', ['state_img_q', 'start', 'end', 'axis', 'step'], ['old']),
           H.make_node('Unsqueeze', ['new_img', 'axis'], ['new']),
           H.make_node('Concat', ['old', 'new'], ['next_state_img_q'], axis=1),
           H.make_node('Gather', ['next_state_img_q', 'zero'], ['one_camera'], axis=0),
           H.make_node('Reshape', ['one_camera', 'flat'], ['pixels']),
           H.make_node('Cast', ['pixels'], ['outputs'], to=T.FLOAT16)]
  inputs = [H.make_tensor_value_info('new_img', T.UINT8, [2, 256]),
            H.make_tensor_value_info('state_img_q', T.UINT8, [2, 5, 256])]
  outputs = [H.make_tensor_value_info('outputs', T.FLOAT16, [5, 256]),
             H.make_tensor_value_info('next_state_img_q', T.UINT8, [2, 5, 256])]
  for name, shape in [('nav_map', [1, 1, 256, 256]), ('navigation', [1, 47]),
                      ('state_map_features_q', [128, 1, 64]), ('state_navigation_features_q', [128, 1, 64])]:
    inputs.append(H.make_tensor_value_info(name, T.FLOAT, shape))
    output = 'next_' + name
    outputs.append(H.make_tensor_value_info(output, T.FLOAT, shape))
    nodes.append(H.make_node('Identity', [name], [output]))
  return H.make_model(H.make_graph(nodes, 'n16_queue_fixture', inputs, outputs, init),
                      opset_imports=[H.make_opsetid('', 18)])


def test_n16_image_patch_exact_across_frames_and_resets_with_untouched_map_nav():
  original = queue_graph()
  patched = patch_uint8_inputs(deepcopy(original))
  assert needs_patch(original) and not needs_patch(patched)
  onnx.checker.check_model(patched, full_check=True)
  before, after = ReferenceEvaluator(original), ReferenceEvaluator(patched)
  state = np.zeros((2, 5, 256), np.uint8)
  other = {vi.name: np.linspace(-1, 1, int(np.prod([d.dim_value for d in vi.type.tensor_type.shape.dim])), dtype=np.float32)
           .reshape(tuple(d.dim_value for d in vi.type.tensor_type.shape.dim))
           for vi in original.graph.input if vi.name not in ('new_img', 'state_img_q')}
  for frame in range(12):
    pixels = np.roll(np.tile(np.arange(256, dtype=np.uint8), (2, 1)), frame, axis=1)
    if frame == 6:
      state.fill(0)
    expected = before.run(None, dict(other, new_img=pixels, state_img_q=state))
    actual = after.run(None, dict(other, new_img=pixels.astype(np.float16), state_img_q=state.astype(np.float16)))
    for a, b in zip(actual, expected, strict=True):
      np.testing.assert_array_equal(a, b)
    state = expected[1]
  for vi in [*patched.graph.input, *patched.graph.output]:
    if 'map' in vi.name or 'navigation' in vi.name:
      assert vi.type.tensor_type.elem_type == T.FLOAT


@pytest.mark.parametrize('failure', ['arithmetic', 'domain', 'cast', 'mixed_concat', 'output_dtype'])
def test_image_patch_rejects_unproven_changes_without_mutation(failure):
  model = queue_graph()
  if failure == 'arithmetic':
    model.graph.node.insert(0, H.make_node('Add', ['new_img', 'new_img'], ['bad']))
  elif failure == 'domain':
    model.graph.node[0].domain = 'unknown'
  elif failure == 'cast':
    model.graph.node.insert(0, H.make_node('Cast', ['new_img'], ['bad'], to=T.INT32))
  elif failure == 'mixed_concat':
    model.graph.node.insert(0, H.make_node('Concat', ['new_img', 'navigation'], ['bad'], axis=0))
  else:
    model.graph.output[1].type.tensor_type.elem_type = T.FLOAT
  serialized = model.SerializeToString()
  with pytest.raises(ValueError):
    patch_uint8_inputs(model)
  assert model.SerializeToString() == serialized


def test_legacy_head_cast_patch_is_unchanged():
  nodes = [H.make_node('Concat', ['img', 'big_img'], ['cat'], axis=0),
           H.make_node('Cast', ['cat'], ['cast'], to=T.FLOAT16),
           H.make_node('Identity', ['cast'], ['outputs'])]
  graph = H.make_graph(nodes, 'legacy', [H.make_tensor_value_info(n, T.UINT8, [1, 256]) for n in ('img', 'big_img')],
                       [H.make_tensor_value_info('outputs', T.FLOAT16, [2, 256])])
  original = H.make_model(graph, opset_imports=[H.make_opsetid('', 18)])
  patched = patch_uint8_inputs(deepcopy(original))
  onnx.checker.check_model(patched, full_check=True)
  images = {n: np.arange(256, dtype=np.uint8).reshape(1, 256) for n in ('img', 'big_img')}
  expected = ReferenceEvaluator(original).run(None, images)
  actual = ReferenceEvaluator(patched).run(None, {n: a.astype(np.float16) for n, a in images.items()})
  np.testing.assert_array_equal(actual[0], expected[0])
