import io
import pickle
import struct

import numpy as np
import pytest

from openpilot.common.file_chunker import get_chunk_name, get_manifest_path, open_file_chunked
from openpilot.selfdrive.modeld import helpers
from openpilot.selfdrive.modeld.helpers import dump_oob, load_oob


def test_warp_buffer_roundtrip_does_not_change_native_pickle_format():
  value = {'identity': (1344, 760, 123456, 'model'), 'arrays': [np.arange(20, dtype=np.uint8), np.eye(3, dtype=np.float32)]}
  stream = io.BytesIO()
  dump_oob(value, stream)
  stream.seek(0)
  result = load_oob(stream)
  assert result['identity'] == value['identity']
  for old, new in zip(value['arrays'], result['arrays'], strict=True):
    np.testing.assert_array_equal(old, new)
    assert old.dtype == new.dtype


@pytest.mark.parametrize('payload', [b'', b'x', struct.pack('<q', -1), struct.pack('<q', 100) + b'x', struct.pack('<q', 65 << 20)])
def test_truncated_or_oversized_warp_pickle_is_rejected(payload):
  with pytest.raises((EOFError, ValueError)):
    load_oob(io.BytesIO(payload))


@pytest.mark.parametrize('length', range(8))
def test_short_metadata_header_is_rejected_before_deserialization(monkeypatch, length):
  def unexpected_load(*args, **kwargs):
    pytest.fail('malformed framing reached the pickle loader')
  monkeypatch.setattr(helpers.pickle, 'load', unexpected_load)
  with pytest.raises(ValueError, match='metadata header'):
    load_oob(io.BytesIO(struct.pack('<q', 1)[:length]))


@pytest.mark.parametrize('length', [-1, 0, 100, (64 << 20) + 1, (1 << 63) - 1])
def test_invalid_metadata_length_cannot_hide_a_valid_pickle(monkeypatch, length):
  opcodes = pickle.dumps({'probe': 1}, protocol=5)
  def unexpected_load(*args, **kwargs):
    pytest.fail('malformed framing reached the pickle loader')
  monkeypatch.setattr(helpers.pickle, 'load', unexpected_load)
  with pytest.raises(ValueError, match='model metadata'):
    load_oob(io.BytesIO(struct.pack('<q', length) + opcodes))


@pytest.mark.parametrize('opcodes', [b'x', b'\x80\x05', b'\x80\x06N.', b'\x80\x05\x95' + b'\xff' * 8])
def test_malformed_metadata_is_value_error(opcodes):
  with pytest.raises(ValueError):
    load_oob(io.BytesIO(struct.pack('<q', len(opcodes)) + opcodes))


def test_metadata_limit_is_inclusive(monkeypatch):
  opcodes = pickle.dumps({'probe': 1}, protocol=5)
  payload = struct.pack('<q', len(opcodes)) + opcodes
  monkeypatch.setattr(helpers, 'MAX_OOB_METADATA_SIZE', len(opcodes))
  assert load_oob(io.BytesIO(payload)) == {'probe': 1}
  monkeypatch.setattr(helpers, 'MAX_OOB_METADATA_SIZE', len(opcodes) - 1)
  with pytest.raises(ValueError, match='metadata size'):
    load_oob(io.BytesIO(payload))


def oob_payload(buffer_header, buffer_data=b''):
  opcodes = pickle.dumps(pickle.PickleBuffer(b'warp'), protocol=5, buffer_callback=lambda _: None)
  return struct.pack('<q', len(opcodes)) + opcodes + buffer_header + buffer_data


@pytest.mark.parametrize('length', range(1, 8))
def test_short_buffer_header_is_rejected(length):
  with pytest.raises(EOFError, match='incomplete model buffer header'):
    load_oob(io.BytesIO(oob_payload(struct.pack('<q', 4)[:length])))


def test_missing_buffer_is_rejected():
  with pytest.raises(ValueError, match='metadata'):
    load_oob(io.BytesIO(oob_payload(b'')))


@pytest.mark.parametrize('length', [-1, helpers.MAX_OOB_BUFFER_SIZE + 1, (1 << 63) - 1])
def test_invalid_buffer_length_is_rejected_before_allocation(monkeypatch, length):
  def unexpected_allocation(*args, **kwargs):
    pytest.fail('invalid buffer length reached allocation')
  monkeypatch.setattr(helpers, 'bytearray', unexpected_allocation, raising=False)
  with pytest.raises(ValueError, match='buffer size'):
    load_oob(io.BytesIO(oob_payload(struct.pack('<q', length))))


@pytest.mark.parametrize('length', range(4))
def test_truncated_buffer_still_raises_eof(length):
  with pytest.raises(EOFError, match='incomplete model buffer'):
    load_oob(io.BytesIO(oob_payload(struct.pack('<q', 4), b'warp'[:length])))


@pytest.mark.parametrize('data', [b'', b'warp'])
def test_buffer_limit_is_inclusive_and_allows_empty_buffers(monkeypatch, data):
  monkeypatch.setattr(helpers, 'MAX_OOB_BUFFER_SIZE', len(data))
  assert bytes(load_oob(io.BytesIO(oob_payload(struct.pack('<q', len(data)), data)))) == data


def test_metadata_and_buffers_preserve_file_and_nonseekable_chunk_api(tmp_path):
  value = {'identity': (1344, 760, 123456, 'model'), 'warp': np.eye(3, dtype=np.float32), 'empty': np.empty(0, dtype=np.uint8)}
  path = tmp_path / 'warp.pkl'
  with path.open('wb') as stream:
    dump_oob(value, stream)
  payload = path.read_bytes()
  with path.open('rb') as stream:
    result = load_oob(stream)
    assert stream.read(1) == b''
  assert result['identity'] == value['identity']
  np.testing.assert_array_equal(result['warp'], value['warp'])
  assert result['empty'].size == 0

  chunks = [payload[i:i + 3] for i in range(0, len(payload), 3)]
  tmp_path.joinpath(get_manifest_path(path.name)).write_text(str(len(chunks)))
  for i, chunk in enumerate(chunks):
    tmp_path.joinpath(get_chunk_name(path.name, i, len(chunks))).write_bytes(chunk)
  with open_file_chunked(path) as stream:
    assert not stream.seekable()
    result = load_oob(stream)
    assert stream.read(1) == b''
  assert result['identity'] == value['identity']
  np.testing.assert_array_equal(result['warp'], value['warp'])
  assert result['empty'].size == 0


def test_loader_leaves_trailing_data_for_callers_to_validate():
  stream = io.BytesIO()
  dump_oob({'probe': 1}, stream)
  stream.write(b'trailing')
  stream.seek(0)
  assert load_oob(stream) == {'probe': 1}
  assert stream.read() == b'trailing'
