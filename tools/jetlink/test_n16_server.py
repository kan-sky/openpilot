import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'third_party/jetlink'), str(Path(__file__).parent)]
import server


def test_import_for_unselected_candidate_does_not_verify_or_advertise():
  assert server.N16_OVERLAY is None


def test_prepared_n16_never_rebuilds_in_server(monkeypatch):
  model = 'a' * 64
  monkeypatch.setattr(server, 'N16_OVERLAY', {'model': model})
  host = object.__new__(server.CachedN16Host)
  with pytest.raises(RuntimeError, match='automatic rebuilding is disabled'):
    host._build_job(SimpleNamespace(sha256=model))


@pytest.mark.parametrize('overlay, requested', [(None, 'a' * 64), ({'model': 'a' * 64}, 'b' * 64)])
def test_other_models_keep_existing_build_path(monkeypatch, overlay, requested):
  monkeypatch.setattr(server, 'N16_OVERLAY', overlay)
  monkeypatch.setattr(server.server.EngineHost, '_build_job', lambda self, request: request.sha256)
  assert object.__new__(server.CachedN16Host)._build_job(SimpleNamespace(sha256=requested)) == requested


def test_verified_overlay_identity_only_added_to_jetson_hello(monkeypatch):
  identity = {'model': 'a' * 64, 'content_sha256': 'b' * 64, 'base_source_commit': 'c' * 40}
  monkeypatch.setattr(server, 'N16_OVERLAY', identity)
  monkeypatch.setattr(server.sys, 'platform', 'linux')
  monkeypatch.setattr(Path, 'is_file', lambda self: str(self) == '/etc/nv_tegra_release')
  monkeypatch.setattr(Path, 'read_text', lambda self: 'c' * 40)
  sent = []
  monkeypatch.setattr(server.Session, '_send_json', lambda self, kind, seq, obj, flags=0: sent.append(obj))
  instance = object.__new__(server.CarrotSession)
  instance._send_json(server.P.Msg.HELLO_RESP, 1, {'stateful_v2': True})
  assert sent[0]['carrot_n16_overlay'] == identity
  assert sent[0]['carrot_host'] == 'jetson'
  monkeypatch.setattr(server.sys, 'platform', 'darwin')
  instance._send_json(server.P.Msg.HELLO_RESP, 2, {})
  assert 'carrot_n16_overlay' not in sent[1]
