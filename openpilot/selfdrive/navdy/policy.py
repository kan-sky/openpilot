"""Persistent transport choice, independent of the paired Bluetooth identity."""
from pathlib import Path

PARAM = 'NavdyConnectionMode'


def read_transport_mode(params=None):
  if params is None:
    from openpilot.common.params import Params
    params = Params()
  # A running native Params table can predate a source-only update. Use its
  # resolved directory, not a second configuration file or a hard-coded root.
  path = Path(params.get_param_path(PARAM))
  try:
    with path.open('rb') as stream:
      raw = stream.read(17)
  except FileNotFoundError:
    return 0
  if len(raw) > 16 or raw.strip() not in (b'0', b'1', b'2'):
    raise ValueError('NavdyConnectionMode must be 0, 1 or 2')
  return int(raw.strip())


def transport_policy(mode, legacy_bluetooth_only):
  if type(mode) is not int or mode not in (0, 1, 2):
    raise ValueError('Invalid Navdy connection mode')
  if mode == 1:
    return 'usb_only'
  if mode == 2 or legacy_bluetooth_only:
    return 'bluetooth_only'
  return 'usb_first'
