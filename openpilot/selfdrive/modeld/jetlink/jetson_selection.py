"""Explicit device-local N16 selection; no implicit model switch on reconnect."""
import json
from pathlib import Path
import re

from openpilot.selfdrive.modeld.jetlink.contracts import N16_SPEC, validate_model_spec

SELECTION = Path('/data/models/carrot-jetlink-n16.json')
CAPABILITY = 'carrot_n16_overlay'


def selected_jetson(peer, path=None):
  if not isinstance(peer, dict) or peer.get('carrot_host') != 'jetson':
    return None
  path = SELECTION if path is None else Path(path)
  try:
    selected = json.loads(path.read_text())
  except FileNotFoundError:
    if CAPABILITY in peer:
      raise ValueError('N16 host requires matching comma installation; retaining internal model')
    return None
  if (not isinstance(selected, dict) or type(selected.get('format')) is not int or selected['format'] != 1
      or selected.get('model') != N16_SPEC.sha256
      or re.fullmatch('[a-f0-9]{64}', str(selected.get('content_sha256', ''))) is None
      or re.fullmatch('[a-f0-9]{40}', str(selected.get('base_source_commit', ''))) is None):
    raise ValueError('Invalid installed N16 selection; retaining internal model')
  expected = {name: selected[name] for name in ('model', 'content_sha256', 'base_source_commit')}
  if (peer.get(CAPABILITY) != expected or peer.get('protocol') != 2
      or peer.get('stateful_v2') is not True or peer.get('backend') != 'trt'
      or peer.get('runtime_version') != '10.3.0' or peer.get('device') != 'Orin-sm87'
      or N16_SPEC.sha256 not in peer.get('cached_models', [])):
    raise ValueError('Selected N16 host/cache identity mismatch; retaining internal model')
  return N16_SPEC


def prepare_selected(client, peer):
  spec = selected_jetson(peer)
  if spec is None:
    return False
  result = client.ensure_engine(spec.sha256, spec.nbytes, frame_skip=spec.frame_skip, build_timeout=30)
  if validate_model_spec(result) != spec:
    raise ValueError('Selected N16 engine contract mismatch')
  return True
