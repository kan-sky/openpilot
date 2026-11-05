import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]
from openpilot.selfdrive.modeld.jetlink import jetson_selection as selection
from openpilot.selfdrive.modeld.jetlink.contracts import DEFAULT_SPEC, N16_SPEC
from openpilot.selfdrive.modeld.jetlink import mac


def record():
  return dict(format=1, model=N16_SPEC.sha256, content_sha256='a' * 64, base_source_commit='b' * 40)


def peer():
  return dict(carrot_host='jetson', backend='trt', runtime_version='10.3.0', device='Orin-sm87',
              protocol=2, stateful_v2=True, cached_models=[N16_SPEC.sha256],
              carrot_n16_overlay={key: value for key, value in record().items() if key != 'format'})


def test_exact_installed_pin_selects_n16(tmp_path, monkeypatch):
  pin = tmp_path / 'selected.json'
  pin.write_text(json.dumps(record()))
  monkeypatch.setattr(selection, 'SELECTION', pin)
  requests = []
  def ensure(*args, **kwargs):
    requests.append((args, kwargs))
    return N16_SPEC
  mac.prepare(SimpleNamespace(ensure_engine=ensure), peer(), lambda: False, lambda: True, lambda *args: None)
  assert requests == [((N16_SPEC.sha256, N16_SPEC.nbytes), dict(frame_skip=4, build_timeout=30))]


@pytest.mark.parametrize('change', [dict(carrot_n16_overlay=None), dict(stateful_v2=1), dict(protocol=3),
                                  dict(backend='ort'), dict(runtime_version='10.4.0'), dict(device='Orin-sm89'),
                                  dict(cached_models=[]), dict(carrot_n16_overlay=dict(model='c' * 64))])
def test_pinned_n16_never_silently_falls_back_to_cinque(tmp_path, change):
  pin = tmp_path / 'selected.json'
  pin.write_text(json.dumps(record()))
  with pytest.raises(ValueError, match='identity mismatch'):
    selection.selected_jetson(dict(peer(), **change), pin)


def test_uninstalled_host_does_not_opt_in(tmp_path):
  pin = tmp_path / 'missing'
  with pytest.raises(ValueError, match='matching comma installation'):
    selection.selected_jetson(peer(), pin)
  assert selection.selected_jetson(dict(carrot_host='jetson'), pin) is None
  pin.write_text(json.dumps(record()))
  assert selection.selected_jetson(dict(carrot_host='mac'), pin) is None


@pytest.mark.parametrize('bad', [[], {}, dict(record(), format=True), dict(record(), model=DEFAULT_SPEC.sha256),
                               dict(record(), content_sha256='../bad'), dict(record(), base_source_commit='')])
def test_malformed_pin_refuses(tmp_path, bad):
  pin = tmp_path / 'selected.json'
  pin.write_text(json.dumps(bad))
  with pytest.raises(ValueError, match='Invalid installed'):
    selection.selected_jetson(peer(), pin)


def test_mismatched_reply_refuses(monkeypatch):
  monkeypatch.setattr(selection, 'selected_jetson', lambda peer: N16_SPEC)
  with pytest.raises(ValueError, match='contract mismatch'):
    selection.prepare_selected(SimpleNamespace(ensure_engine=lambda *a, **kw: DEFAULT_SPEC), peer())
