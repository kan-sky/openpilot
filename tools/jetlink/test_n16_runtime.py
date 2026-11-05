"""N16 contracts and real host runtime; CUDA calls are simulated, not GPU parity.

Fixture: 59eb0f82fa8bf5f2d85106afa446d33197c52df8:
openpilot/selfdrive/modeld/jetlink/jetson_model.json. No device access.
"""
from copy import deepcopy
from contextlib import contextmanager
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import socket
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]

from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.queues import PolicyQueues, StatefulQueues, make_queues
from jetlink.server.session import EngineHost, Session, _check_shapes
from jetlink.spec import ModelSpec
from jetlink.transport.base import LinkError
from jetlink.transport.tcp import TcpTransport
from openpilot.selfdrive.modeld.jetlink.contracts import DEFAULT_SPEC, N16_SPEC, model_name, validate_model_spec

SPEC = ModelSpec.from_dict(json.loads((Path(__file__).parent / 'fixtures/n16.json').read_text()))


def inputs(spec=SPEC, image_dtype=np.float16):
  return {name: np.zeros(shape, image_dtype if name in ('new_img', 'state_img_q') else np.float32)
          for name, shape in spec.input_shapes.items()}


def advanced(dest, value=1):
  # State first ensures the session selects the named driving output.
  return {**{output: np.full_like(dest[name], value + i) for i, (name, output) in enumerate(SPEC.state_pairs.items())},
          'outputs': np.full(SPEC.output_shapes['outputs'], value, np.float32)}


def assert_reset(queues):
  assert all(not value.any() for value in queues.state.values())


def test_n16_exact_pin_and_legacy_default_unchanged():
  assert N16_SPEC == SPEC
  assert validate_model_spec(SPEC) is SPEC
  assert SPEC.sha256 == '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
  assert SPEC.nbytes == 814591107 and len(SPEC.state_pairs) == 5
  assert SPEC.output_nelem == 18580 and SPEC.packed_nelem == 65595
  assert SPEC.packed_nbytes == 262380 and SPEC.warped_nbytes == 393216
  assert list(SPEC.packed_shapes) == ['desire', 'traffic_convention', 'action_t', 'nav_map', 'navigation']
  assert SPEC.packed_shapes['nav_map'] == (1, 1, 256, 256)
  assert SPEC.packed_shapes['navigation'] == (1, 47)
  assert validate_model_spec(DEFAULT_SPEC) is DEFAULT_SPEC
  # The old pin ignores a redundant unit step when serializing slices.
  assert validate_model_spec(replace(DEFAULT_SPEC, output_slices=dict(DEFAULT_SPEC.output_slices, action=slice(2062, 2066, 1))))
  assert model_name(DEFAULT_SPEC) == 'Cinque v2' and model_name(SPEC) == 'N16 Nav (96244b1f)'
  assert not DEFAULT_SPEC.stateful and DEFAULT_SPEC.packed_nbytes == 65584
  for name, at in DEFAULT_SPEC.output_slices.items():
    if name != 'pad':
      assert SPEC.output_slices[name] == at


@pytest.mark.parametrize('change', [
  {'sha256': 'a' * 64}, {'nbytes': 814591108}, {'checkpoint': 'another-export'}, {'frame_skip': True},
  {'input_shapes': {k: v for k, v in SPEC.input_shapes.items() if k != 'nav_map'}},
  {'input_shapes': {k: v for k, v in SPEC.input_shapes.items() if k != 'navigation'}},
  {'input_shapes': dict(SPEC.input_shapes, navigation=(47,))},
  {'input_shapes': dict(SPEC.input_shapes, nav_map=(1, 1, 128, 512))},
  {'input_shapes': dict(SPEC.input_shapes, desire=(1, 8))},
  {'input_shapes': dict(SPEC.input_shapes, state_map_features_q=(64, 1, 128))},
  {'input_shapes': dict(SPEC.input_shapes, state_navigation_features_q=(128, True, 64))},
  {'input_shapes': dict(SPEC.input_shapes, state_extra=(1,))},
  {'output_shapes': dict(SPEC.output_shapes, next_state_navigation_features_q=(1, 128, 64))},
  {'output_slices': dict(SPEC.output_slices, action=slice(2062, 2066, 1))},
  {'output_slices': dict(SPEC.output_slices, navigation_features=slice(18514, 18577))},
])
def test_n16_contract_is_not_a_generic_navigation_allowlist(change):
  with pytest.raises(ValueError):
    validate_model_spec(replace(SPEC, **change))


@pytest.mark.parametrize('dtype', [np.uint8, np.float16])
def test_five_states_feedback_and_packed_inputs(dtype):
  dest = inputs(image_dtype=dtype)
  q = make_queues(SPEC, dest)
  assert isinstance(q, StatefulQueues)
  warped = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  packed = np.arange(SPEC.packed_nelem, dtype=np.float32)
  packed[12:12 + 65536] %= 256
  for value in (1, 2, 3):
    q.step_into(warped, packed, dest)
    np.testing.assert_array_equal(dest['new_img'], warped)
    offset = 0
    for name, size in zip(SPEC.packed_shapes, SPEC.packed_sizes, strict=True):
      np.testing.assert_array_equal(dest[name].ravel(), packed[offset:offset + size])
      offset += size
    result = advanced(dest, value)
    q.feedback(result)
    for i, (name, output) in enumerate(SPEC.state_pairs.items()):
      np.testing.assert_array_equal(dest[name], result[output])
      result[output].fill(99)
      assert np.all(dest[name] == value + i)
  q.reset()
  assert_reset(q)


@pytest.mark.parametrize('name', SPEC.output_shapes)
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize('shared', [False, True])
def test_every_nonfinite_output_clears_all_five_states(name, bad, shared):
  dest = inputs()
  result = advanced(dest, 3)
  if shared:
    for state, output in SPEC.state_pairs.items():
      dest[state] = result[output]
  q = StatefulQueues(SPEC, dest)
  for value in result.values():
    value.fill(3)
  result[name].flat[-1] = bad
  with pytest.raises(FloatingPointError, match='Non-finite'):
    q.feedback(result)
  assert_reset(q)


@pytest.mark.parametrize('failure', ['missing', 'extra', 'shape', 'dtype'])
def test_feedback_validates_all_outputs_before_copying(failure):
  dest = inputs()
  q = StatefulQueues(SPEC, dest)
  q.feedback(advanced(dest, 1))
  result = advanced(dest, 2)
  name = 'next_state_navigation_features_q'
  if failure == 'missing':
    del result[name]
  elif failure == 'extra':
    result['unexpected'] = np.zeros(1)
  elif failure == 'shape':
    result[name] = result[name].reshape(1, 128, 64)
  else:
    result[name] = result[name].astype(np.float16)
  with pytest.raises(ValueError):
    q.feedback(result)
  assert_reset(q)


def test_half_finite_mask_covers_every_binary16_value():
  q = StatefulQueues(SPEC, inputs())
  values = np.arange(65536, dtype=np.uint16).view(np.float16)
  assert not q._all_finite(values)
  np.testing.assert_array_equal(q._finite_bad[:values.size], ~np.isfinite(values))
  assert q._all_finite(values[np.isfinite(values)])
  for bits in (0x7c00, 0xfc00, 0x7c01, 0x7fff, 0xfc01, 0xffff):
    assert not q._all_finite(np.array([0, bits, 0x8000], np.uint16).view(np.float16))
    assert q._all_finite(np.array([0, -0., 65504., -65504.], np.float16))


@pytest.mark.parametrize('offset', [0, 8, 10, 12, 65548])
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_invalid_packed_field_resets_history_before_inference(offset, bad):
  dest = inputs()
  q = StatefulQueues(SPEC, dest)
  q.feedback(advanced(dest))
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  packed[offset] = bad
  with pytest.raises(FloatingPointError):
    q.step_into(np.zeros(SPEC.warped_shape, np.uint8), packed, dest)
  assert_reset(q)


@pytest.mark.parametrize('failure', ['image_type', 'image_shape', 'packed_type', 'packed_shape'])
def test_invalid_frame_contract_resets_history(failure):
  dest = inputs()
  q = StatefulQueues(SPEC, dest)
  q.feedback(advanced(dest))
  warped, packed = np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32)
  if failure == 'image_type':
    warped = warped.astype(np.float32)
  elif failure == 'image_shape':
    warped = warped.reshape(-1)
  elif failure == 'packed_type':
    packed = packed.astype(np.float16)
  else:
    packed = packed.reshape(1, -1)
  with pytest.raises(ValueError):
    q.step_into(warped, packed, dest)
  assert_reset(q)


def fake_engine():
  dest = inputs()
  engine = NS(inputs={name: NS(shape=shape) for name, shape in SPEC.input_shapes.items()},
              outputs={name: NS(shape=shape) for name, shape in reversed(list(SPEC.output_shapes.items()))},
              host_input=dest.__getitem__, last_gpu_us=1, close=lambda: None, calls=0, fail=None)
  def run():
    engine.calls += 1
    result = advanced(dest, float(dest['state_img_q'].flat[0]) + 1)
    if engine.fail == 'raise':
      raise RuntimeError('execution failed')
    if engine.fail == 'numeric_raise':
      raise FloatingPointError('execution failed')
    if engine.fail in result:
      result[engine.fail].flat[-1] = np.nan
    return result
  def warm():
    run()
    for name in SPEC.state_pairs:
      dest[name].fill(42)
    return 'simulated warm'
  engine.run, engine.warm = run, warm
  return engine, dest


def local_session(engine):
  loaded = EngineHost._warm(None, engine, SPEC)
  sent = []
  transport = NS(send=lambda kind, seq, parts, flags: sent.append(b''.join(memoryview(p).cast('B') for p in parts)))
  host = NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None))
  return Session(transport, host), loaded, sent


def request(seq, flags=0):
  return NS(seq=seq, payload=memoryview(P.pack_infer_req(seq, flags) + bytes(SPEC.warped_nbytes + SPEC.packed_nbytes)))


def test_real_session_named_output_feedback_reset_and_failed_state():
  engine, dest = fake_engine()
  session, loaded, sent = local_session(engine)
  assert_reset(loaded.queues)  # warm state must never leak to the first frame
  for seq, flags, expected in ((1, 0, 1), (2, 0, 2), (3, P.Flag.RESET_QUEUES, 1)):
    session._infer(loaded, request(seq, flags))
    response = memoryview(sent[-1])
    assert P.unpack_infer_resp(response)[:2] == (seq, P.Status.OK)
    assert len(response) == SPEC.infer_resp_nbytes
    np.testing.assert_array_equal(np.frombuffer(response, np.float32, offset=P.INFER_RESP_SIZE), expected)
  engine.fail = 'next_state_navigation_features_q'
  session._infer(loaded, request(4))
  assert P.unpack_infer_resp(memoryview(sent[-1]))[1] == P.Status.NOT_FINITE
  assert_reset(loaded.queues)
  engine.fail = None
  session._infer(loaded, request(5))
  assert np.all(np.frombuffer(sent[-1], np.float32, offset=P.INFER_RESP_SIZE) == 1)
  assert np.all(dest['state_map_features_q'] == 4)


@pytest.mark.parametrize('failure', ['raise', 'numeric_raise'])
def test_engine_exception_does_not_reuse_shared_or_previous_state(failure):
  engine, _ = fake_engine()
  session, loaded, _ = local_session(engine)
  session._infer(loaded, request(1))
  engine.fail = failure
  with pytest.raises((RuntimeError, FloatingPointError), match='execution failed'):
    session._infer(loaded, request(2))
  assert_reset(loaded.queues)


def test_bad_wire_size_never_runs_or_advances_queues():
  engine, _ = fake_engine()
  session, loaded, sent = local_session(engine)
  session._infer(loaded, request(1))
  before = {name: a.copy() for name, a in loaded.queues.state.items()}
  calls = engine.calls
  msg = request(2)
  msg.payload = msg.payload[:-4]
  session._infer(loaded, msg)
  assert P.unpack_infer_resp(memoryview(sent[-1]))[1] == P.Status.BAD_SHAPE and engine.calls == calls
  for name, a in before.items():
    np.testing.assert_array_equal(loaded.queues.state[name], a)


@pytest.mark.parametrize('name', [*SPEC.input_shapes, *SPEC.output_shapes])
def test_stateful_engine_rejects_same_size_wrong_shape(name):
  engine, _ = fake_engine()
  bindings = engine.inputs if name in engine.inputs else engine.outputs
  bindings[name].shape = (int(np.prod(bindings[name].shape)),)
  if name == 'desire':
    bindings[name].shape = (1, 8)
  with pytest.raises(ValueError, match=name):
    _check_shapes(engine, SPEC)


@pytest.mark.parametrize('kind', ['missing_input', 'missing_output', 'extra_output'])
def test_stateful_engine_rejects_wrong_bindings(kind):
  engine, _ = fake_engine()
  if kind == 'missing_input':
    del engine.inputs['state_map_features_q']
  elif kind == 'missing_output':
    del engine.outputs['next_state_map_features_q']
  else:
    engine.outputs['unexpected'] = NS(shape=(1,))
  with pytest.raises(ValueError):
    _check_shapes(engine, SPEC)


def test_legacy_queue_frame_and_feedback_are_unchanged():
  spec = DEFAULT_SPEC
  q = make_queues(spec, {})
  reference = PolicyQueues(spec)
  assert isinstance(q, PolicyQueues)
  for frame in range(7):
    warped = np.full(spec.warped_shape, frame, np.uint8)
    packed = np.full(spec.packed_nelem, frame, np.float32)
    expected = reference.step(warped, packed)
    actual = q.step(warped, packed)
    q.feedback({'outputs': np.ones(spec.output_shapes['outputs'], np.float32)})
    for name in expected:
      np.testing.assert_array_equal(actual[name], expected[name])
  q.reset()
  assert all(not queue.buf.any() for queue in (q.img_q, q.big_img_q, q.feat_q, q.desire_q))
  # Queued engines historically permit flattened bindings with equal size.
  engine = NS(inputs={n: NS(shape=(int(np.prod(s)),)) for n, s in spec.input_shapes.items()},
              outputs={'outputs': NS(shape=(spec.output_nelem,))})
  _check_shapes(engine, spec)


def test_real_v2_hello_n16_inference_feedback_and_reset():
  engine, dest = fake_engine()
  backend = NS(describe=lambda: {'backend': 'test', 'runtime_version': 'test', 'device': 'CPU'})
  host = EngineHost(NS(backend=backend, inventory=list))
  host.loaded = host._warm(engine, SPEC)
  ours, peer = socket.socketpair()
  transport = TcpTransport(peer)
  transport.protocol_version = 2
  session = Session(transport, host)
  thread = threading.Thread(target=session.serve_forever, daemon=True)
  thread.start()
  stream = TcpTransport(ours)
  stream.protocol_version = 2
  client = JetlinkClient(stream, deadline=3)
  try:
    hello = client.hello()
    assert hello['protocol'] == 2 and hello['stateful_v2'] is True
    assert 'carrot_nav_u8_v1' not in hello and 'carrot_lz4_v1' not in hello
    assert client._stateful_v2 and client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=4) == SPEC
    warped = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
    packed = np.arange(SPEC.packed_nelem, dtype=np.float32) % 256
    for frame, reset, expected in ((1, True, 1), (2, False, 2), (3, True, 1)):
      result = client.infer(warped, packed, frame, reset=reset)
      np.testing.assert_array_equal(result, np.full(SPEC.output_nelem, expected, np.float32))
      np.testing.assert_array_equal(dest['new_img'], warped)
      np.testing.assert_array_equal(dest['nav_map'].ravel(), packed[12:65548])
      np.testing.assert_array_equal(dest['navigation'].ravel(), packed[-47:])
    assert session.frames == 3
    # Re-HELLO starts another session but must explicitly grant capability again.
    assert client.hello()['stateful_v2'] is True
    assert client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=4) == SPEC
    np.testing.assert_array_equal(client.infer(warped, packed, 4, reset=True), 1)
  finally:
    client.close()
    thread.join(5)
    transport.close()
    session.close()
    host.close()
  assert not thread.is_alive() and P.VERSION == 2


@contextmanager
def hello_client(responses):
  """Real v2 framing with scripted older-host HELLO metadata."""
  ours, peer = socket.socketpair()
  transport = TcpTransport(peer)
  errors, sent = [], []

  def serve():
    try:
      for response in responses:
        message = transport.recv(timeout=3)
        assert message.msg_type == P.Msg.HELLO_REQ
        payload = response if isinstance(response, bytes) else json.dumps(response).encode()
        transport.send(P.Msg.HELLO_RESP, message.seq, (payload,))
    except BaseException as exc:
      errors.append(exc)
    finally:
      transport.close()

  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  stream = TcpTransport(ours)
  send = stream.send
  def record_send(kind, seq, parts=(), flags=0):
    sent.append(kind)
    return send(kind, seq, parts, flags)
  stream.send = record_send
  client = JetlinkClient(stream, deadline=3)
  client.spec = SPEC
  try:
    yield client, sent
  finally:
    client.close()
    thread.join(5)
  assert not thread.is_alive() and not errors


@pytest.mark.parametrize('capability', [{}, {'stateful_v2': False}, {'stateful_v2': None},
                                       {'stateful_v2': 1}, {'stateful_v2': 'true'}])
@pytest.mark.parametrize('previously_enabled', [False, True])
def test_legacy_v2_host_rejects_n16_and_rehello_revokes_capability(capability, previously_enabled):
  responses = ([{'protocol': 2, 'stateful_v2': True}] if previously_enabled else []) + [{'protocol': 2, **capability}]
  with hello_client(responses) as (client, sent):
    assert not client._stateful_v2
    if previously_enabled:
      client.hello()
      assert client._stateful_v2
    client.hello()
    assert not client._stateful_v2
    before = len(sent)
    with pytest.raises(LinkError, match='stateful models require'):
      client.infer_begin(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 1)
    assert len(sent) == before and P.Msg.INFER_REQ not in sent


@pytest.mark.parametrize('response', [
  {'protocol': 3, 'stateful_v2': True}, {'protocol': True, 'stateful_v2': True},
  {'stateful_v2': True}, [], b'not JSON',
])
def test_failed_rehello_cannot_retain_stateful_v2_permission(response):
  with hello_client([{'protocol': 2, 'stateful_v2': True}, response]) as (client, sent):
    client.hello()
    assert client._stateful_v2
    with pytest.raises((LinkError, ValueError)):
      client.hello()
    assert not client._stateful_v2
    before = len(sent)
    with pytest.raises(LinkError, match='stateful models require'):
      client.infer_begin(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 1)
    assert len(sent) == before


def test_real_v3_socket_inference_feedback_and_reset_without_hello_change():
  engine, _ = fake_engine()
  backend = NS(describe=lambda: {'backend': 'test', 'runtime_version': 'test', 'device': 'CPU'})
  host = EngineHost(NS(backend=backend, inventory=list))
  host.loaded = host._warm(engine, SPEC)
  ours, peer = socket.socketpair()
  transport = TcpTransport(peer)
  transport.protocol_version = 3
  session = Session(transport, host)
  thread = threading.Thread(target=session.serve_forever, daemon=True)
  thread.start()
  stream = TcpTransport(ours)
  stream.protocol_version = 3
  client = JetlinkClient(stream, deadline=3)
  try:
    # Test existing v3 framing independently of the v2 capability handshake.
    assert client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=4) == SPEC
    for frame, reset, expected in ((1, True, 1), (2, False, 2), (3, True, 1)):
      result = client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), frame, reset=reset)
      np.testing.assert_array_equal(result, np.full(SPEC.output_nelem, expected, np.float32))
    engine.fail = 'next_state_map_features_q'
    with pytest.raises(LinkError):
      client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 4)
    assert_reset(host.loaded.queues)
  finally:
    client.close()
    thread.join(5)
    transport.close()
    session.close()
    host.close()
  assert not thread.is_alive()


@pytest.mark.parametrize('failure', [None, 'nonfinite', 'raise'])
def test_legacy_session_still_returns_client_queued_driving_output(failure):
  spec = DEFAULT_SPEC
  dest = {n: np.zeros(s, np.float16) for n, s in spec.input_shapes.items()}
  result = np.arange(spec.output_nelem, dtype=np.float32).reshape(spec.output_shapes['outputs'])
  engine = NS(inputs={n: NS(shape=s) for n, s in spec.input_shapes.items()},
              outputs={'outputs': NS(shape=result.shape)}, host_input=dest.__getitem__,
              warm=lambda: 'warm', run=lambda: {'outputs': result}, last_gpu_us=1)
  loaded = EngineHost._warm(None, engine, spec)
  sent = []
  session = Session(NS(send=lambda kind, seq, parts, flags: sent.append(b''.join(memoryview(p).cast('B') for p in parts))),
                    NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None)))
  packed = np.arange(spec.packed_nelem, dtype=np.float32)
  msg = NS(seq=1, payload=memoryview(P.pack_infer_req(1, 0) + bytes(spec.warped_nbytes) + packed.tobytes()))
  if failure == 'nonfinite':
    result.flat[0] = np.nan
  if failure == 'raise':
    def failed():
      raise FloatingPointError('failed legacy inference')
    engine.run = failed
    with pytest.raises(FloatingPointError):
      session._infer(loaded, msg)
    assert not sent
  else:
    session._infer(loaded, msg)
    response = memoryview(sent[-1])
    assert P.unpack_infer_resp(response)[:2] == (1, P.Status.NOT_FINITE if failure else P.Status.OK)
    np.testing.assert_array_equal(np.frombuffer(response, np.float32, offset=P.INFER_RESP_SIZE), result.ravel())
  np.testing.assert_array_equal(loaded.queues.feat_q.logical()[-1].ravel(), packed[12:].astype(np.float16))


@pytest.fixture
def trt_module(monkeypatch):
  """Import the real engine without TensorRT/CUDA or device initialization."""
  directory = ROOT / 'third_party/jetlink/jetlink/server/backends/trt'
  package = ModuleType('jetlink.server.backends.trt')
  package.__path__ = [str(directory)]
  package.cudart = NS()
  fake_trt = NS(DataType=NS(**dict(zip(
    ('FLOAT', 'HALF', 'INT8', 'INT32', 'INT64', 'BOOL', 'UINT8'), range(7), strict=True))), Logger=NS(WARNING=1))
  monkeypatch.setitem(sys.modules, 'tensorrt', fake_trt)
  monkeypatch.setitem(sys.modules, 'jetlink.server.backends.trt', package)
  spec = importlib.util.spec_from_file_location('n16_trt_engine_test', directory / 'engine.py')
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, spec.name, module)
  spec.loader.exec_module(module)
  return module, package.cudart


def test_real_trt_enqueue_uses_all_five_previous_outputs_before_writes(trt_module):
  module, cuda = trt_module
  dest = inputs()
  result = advanced(dest, 1)
  engine = module.TrtEngine.__new__(module.TrtEngine)
  engine.bindings = {}
  for i, (name, host) in enumerate([*dest.items(), *result.items()]):
    engine.bindings[name] = module.Binding(name, host.shape, host.dtype, host.nbytes, i + 1,
                                          host.ctypes.data, host, name in dest)
  engine.inputs = {n: b for n, b in engine.bindings.items() if b.is_input}
  engine.outputs = {n: b for n, b in engine.bindings.items() if not b.is_input}
  engine._state_host_sources, engine.graph_exec, engine.stream = {}, None, 7
  original = {n: b.device_ptr for n, b in engine.bindings.items()}
  engine.configure_state_feedback(SPEC.state_pairs)
  for name, output in SPEC.state_pairs.items():
    assert engine.host_input(name) is result[output]
  q = StatefulQueues(SPEC, {n: engine.host_input(n) for n in engine.inputs})
  assert_reset(q)
  events = []
  cuda.memcpy_h2d_async = lambda *a: events.append(('h2d', *a))
  cuda.memcpy_d2h_async = lambda *a: events.append(('d2h', *a))
  engine.context = NS(execute_async_v3=lambda stream: events.append(('execute', stream)) or True)
  engine._enqueue()
  assert [e[0] for e in events] == ['h2d'] * len(dest) + ['execute'] + ['d2h'] * len(result)
  for event, (name, binding) in zip(events, engine.inputs.items(), strict=False):
    source = engine.outputs[SPEC.state_pairs[name]] if name in SPEC.state_pairs else binding
    assert event == ('h2d', binding.device_ptr, source.host_ptr, binding.nbytes, 7)
  assert {n: b.device_ptr for n, b in engine.bindings.items()} == original
  for a in result.values():
    a.fill(3)
  result['next_state_map_features_q'].flat[-1] = np.inf
  with pytest.raises(FloatingPointError):
    q.feedback(result)
  assert_reset(q)
  engine.graph_exec = object()
  with pytest.raises(RuntimeError, match='before CUDA graph'):
    engine.configure_state_feedback(SPEC.state_pairs)


@pytest.mark.parametrize('failure', ['shape', 'dtype', 'nbytes', 'missing', 'duplicate'])
def test_trt_feedback_binding_contracts(trt_module, failure):
  from jetlink.server.backends.trt.state_buffers import feedback_sources
  target = NS(shape=(128, 1, 64), dtype=np.dtype(np.float32), nbytes=32768)
  source = deepcopy(target)
  pairs = {'state_map_features_q': 'next_state_map_features_q'}
  if failure == 'missing':
    pairs['state_navigation_features_q'] = 'next_state_navigation_features_q'
  elif failure == 'duplicate':
    pairs['state_navigation_features_q'] = pairs['state_map_features_q']
  else:
    setattr(source, failure, {'shape': (1, 128, 64), 'dtype': np.dtype(np.float16), 'nbytes': 16384}[failure])
  with pytest.raises(ValueError):
    feedback_sources({'state_map_features_q': target}, {'next_state_map_features_q': source}, pairs)
