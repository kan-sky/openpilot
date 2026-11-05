from types import SimpleNamespace

import pytest

from openpilot.selfdrive.navdy.policy import PARAM, read_transport_mode, transport_policy


@pytest.mark.parametrize('mode,legacy,expected', [
  (0, False, 'usb_first'), (0, True, 'bluetooth_only'),
  (1, False, 'usb_only'), (1, True, 'usb_only'),
  (2, False, 'bluetooth_only'), (2, True, 'bluetooth_only'),
])
def test_explicit_mode_overrides_only_transport_not_peer(mode, legacy, expected):
  assert transport_policy(mode, legacy) == expected


@pytest.mark.parametrize('mode', [None, True, False, '1', 1.0, -1, 3])
def test_invalid_mode_never_silently_enables_a_transport(mode):
  with pytest.raises(ValueError):
    transport_policy(mode, True)


@pytest.mark.parametrize('raw,expected', [(None, 0), (b'0', 0), (b'1', 1), (b'2', 2), (b'1\n', 1), (b'\t2 ', 2)])
def test_reads_current_param_directory_without_native_key_rebuild(tmp_path, raw, expected):
  def path(key):
    assert key == PARAM
    return str(tmp_path / key)
  params = SimpleNamespace(get_param_path=path)
  if raw is not None:
    (tmp_path / PARAM).write_bytes(raw)
  assert read_transport_mode(params) == expected


@pytest.mark.parametrize('raw', [b'', b' ', b'1.0', b'true', b'3', b'-1', b'1' * 1000, b'1' + b' ' * 100])
def test_bad_persistent_value_is_rejected(tmp_path, raw):
  (tmp_path / PARAM).write_bytes(raw)
  with pytest.raises(ValueError):
    read_transport_mode(SimpleNamespace(get_param_path=lambda _: str(tmp_path / PARAM)))
