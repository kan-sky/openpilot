"""Phone extensions preserve the current cereal HUD protocol."""
import base64
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).parent))
import hud
import hud_protocol as wire
from cluster_models import PhoneMediaInfo
from cluster_layout import extrapolated_media_position_ms
from cluster_phone_media import PhoneMediaReader


def media(**values):
  return PhoneMediaInfo(**{'title': '이등병의 편지', 'artist': '김광석', 'is_playing': True,
                           'duration_ms': 200000, 'position_ms': 1000, 'received_at_ms': 99000,
                           'art_base64': 'YWJj' * 10000, 'art_mime': 'image/jpeg', **values})


def receive(encoder, receiver, value, sequence=0):
  record = encoder.encode(value, wall_time=100.)
  snapshot = (10. + sequence * .1, {'version': 1, 'phone_media': record})
  return receiver.decode(snapshot, wall_time=500000., monotonic=snapshot[0] + .02), record


def test_phone_unicode_art_and_clock_transfer_are_bounded():
  encoder, receiver = wire.PhoneMediaEncoder(), wire.PhoneMediaReceiver()
  original = media()
  for sequence in range(5):
    result, record = receive(encoder, receiver, original, sequence)
    assert len(record['art']['data']) <= 8192
    assert result.title == original.title and result.artist == original.artist
    assert result.position_ms == 2000
    assert 2019 <= extrapolated_media_position_ms(result.position_ms, result.duration_ms, True,
                                                  result.received_at_ms, 500000000) <= 2020
    assert bool(result.art_base64) == (sequence == 4)
  assert result.art_base64 == original.art_base64


def test_missing_fragment_recovers_and_track_switch_discards_art():
  encoder, receiver = wire.PhoneMediaEncoder(), wire.PhoneMediaReceiver()
  original = media()
  receive(encoder, receiver, original)
  encoder.encode(original, 100.)
  for sequence in range(2, 5):
    result, _ = receive(encoder, receiver, original, sequence)
    assert not result.art_base64
  for sequence in range(5, 10):
    result, _ = receive(encoder, receiver, original, sequence)
  assert result.art_base64 == original.art_base64
  result, _ = receive(encoder, receiver, media(title='Next', art_base64='eHl6' * 10000), 10)
  assert result.title == 'Next' and not result.art_base64
  assert receiver.decode(None) is None
  assert receiver.art == receiver.parts == ''


def test_corrupt_art_not_displayed_and_oversized_art_not_sent():
  encoder, receiver = wire.PhoneMediaEncoder(), wire.PhoneMediaReceiver()
  record = encoder.encode(media(art_base64='YWJj'), 100.)
  record['art']['data'] = 'eHl6'
  result = receiver.decode((10., {'phone_media': record}), wall_time=100., monotonic=10.)
  assert not result.art_base64
  for art in ('a' * (wire.MAX_PHONE_ART_CHARS + 1), '한글'):
    assert 'art' not in encoder.encode(media(art_base64=art), 100.)


@pytest.mark.parametrize('age', [-1, .5, 1, float('nan'), float('inf')])
def test_phone_receiver_rejects_stale_or_nonfinite_snapshot(age):
  record = wire.PhoneMediaEncoder().encode(media(), 100.)
  assert wire.PhoneMediaReceiver().decode((10., {'phone_media': record}), monotonic=10. + age) is None


@pytest.mark.parametrize('fragment', [None, {}, {'offset': -1, 'total': 1, 'data': 'a'},
  {'offset': 0, 'total': wire.MAX_PHONE_ART_CHARS + 1, 'data': 'a'},
  {'offset': 0, 'total': 9000, 'data': 'a' * 9000}, {'offset': False, 'total': 1, 'data': 'a'}])
def test_malformed_fragments_do_not_hide_metadata_or_allocate_large_buffers(fragment):
  record = wire.PhoneMediaEncoder().encode(media(), 100.)
  record['art'] = fragment
  receiver = wire.PhoneMediaReceiver()
  result = receiver.decode((10., {'phone_media': record}), monotonic=10.)
  assert result.title == media().title and not result.art_base64
  assert not receiver.parts


def test_current_cereal_snapshot_roundtrip_is_unchanged(monkeypatch):
  from openpilot.cereal import log
  event = log.Event.new_message(logMonoTime=1000000000, valid=True,
                                carState={'vEgo': 24., 'leftBlindspot': True})
  record = {'version': 1, 'sent': 1., 'events': {'carState': base64.b64encode(event.to_bytes()).decode()},
            'params': {'ClusterHudTheme': base64.b64encode(b'2').decode()},
            'mono': {'carState': 1000000000}, 'received': {'carState': 1.},
            'valid': {'carState': True}, 'alive': {'carState': True},
            'phone_media': wire.PhoneMediaEncoder().encode(media(), 100.)}
  record = json.loads(json.dumps(record))
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (10., record))
  sm = hud.RemoteSubMaster(['carState'])
  sm.update()
  assert sm['carState'].vEgo == 24. and sm['carState'].leftBlindspot
  assert sm.updated['carState'] and sm.valid['carState'] and sm.alive['carState']
  assert sm.recv_time['carState'] == 10. and sm.logMonoTime['carState'] == 10000000000
  assert hud.DisplayParams().get_int('ClusterHudTheme') == 2
  sm.update()
  assert not sm.updated['carState']
  monkeypatch.setattr(hud, 'read_snapshot', lambda: None)
  sm.update()
  assert not sm.valid['carState'] and not sm.alive['carState']
  assert wire.HUD_CAPABILITY == 'carrot_hud_v1' and wire.HUD_MESSAGE == 0x4000


def test_actual_snapshot_builder_keeps_cereal_and_navigation_media(monkeypatch):
  from openpilot.cereal import log
  import openpilot.common.jetlink_status as status
  monkeypatch.setattr(status, 'badge', lambda: None)
  monkeypatch.setattr(wire.time, 'monotonic', lambda: 10.)
  builder = object.__new__(wire.SnapshotBuilder)
  builder.next_send = 0.
  builder.next_params = 100.
  builder.settings = {'ClusterHud': 'MQ=='}
  builder.host_release = None
  builder.network = SimpleNamespace(address=lambda: '')
  event = log.Event.new_message(carState={'vEgo': 24.})
  builder.cached = {'carState': base64.b64encode(event.to_bytes()).decode()}
  builder.sm = SimpleNamespace(update=lambda _: None, updated=dict.fromkeys(wire.SERVICES, False),
                               recv_time={}, logMonoTime={}, valid={}, alive={})
  builder.camera_previews = lambda: {'road': 'a' * (96 * 1024)}
  builder.phone_media = SimpleNamespace(read=lambda: media())
  builder.phone_encoder = wire.PhoneMediaEncoder()
  packet = builder.packet()
  record = json.loads(packet)
  assert len(packet) <= 96 * 1024 and record['events'] == builder.cached
  assert record['cameras'] == {} and record['phone_media']['metadata']['title'] == media().title
  assert 'media_socket.recv' in Path(wire.__file__).read_text()


def test_sidecar_reader_is_bounded_and_clears_invalid_or_removed_content(tmp_path, monkeypatch):
  path = tmp_path / 'phone.json'
  tick = [10.]
  monkeypatch.setattr(wire.time, 'monotonic', lambda: tick[0])
  path.write_text(json.dumps({'title': '한글 제목', 'isPlaying': True, 'positionMs': 1000,
                              'receivedAtMs': 99000, 'artBase64': 'YWJj', 'package': 'player'}))
  reader = PhoneMediaReader([path])
  assert reader.read().title == '한글 제목'
  assert reader.read().package_name == 'player'
  tick[0] += 1
  path.write_text('x' * (wire.MAX_PHONE_ART_CHARS + 32769))
  assert reader.read() is None
  tick[0] += 1
  path.write_text('{bad')
  assert reader.read() is None
  tick[0] += 1
  path.unlink()
  assert reader.read() is None


def test_c4_preview_rejects_headless_before_graphics_initialization(monkeypatch):
  import preview_cluster
  import cluster_graphics_cleanup
  monkeypatch.setattr(preview_cluster.Path, 'is_file', lambda path: str(path) == '/TICI')
  monkeypatch.setenv('RAYLIB_BACKEND', 'headless')
  monkeypatch.setattr(cluster_graphics_cleanup, 'install_graphics_cleanup',
                      lambda: pytest.fail('unsupported backend must not initialize'))
  with pytest.raises(RuntimeError, match='headless is unsupported'):
    preview_cluster.configure_preview_graphics()


def test_preview_configures_native_cleanup_before_renderer_import():
  import ast
  import preview_cluster
  tree = ast.parse(Path(preview_cluster.__file__).read_text())
  assert not any(isinstance(node, ast.ImportFrom) and node.module == 'cluster_renderer' for node in tree.body)
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
  configure = next(i for i, node in enumerate(main.body) if isinstance(node, ast.Expr)
                   and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name)
                   and node.value.func.id == 'configure_preview_graphics')
  render_import = next(i for i, node in enumerate(main.body)
                       if isinstance(node, ast.ImportFrom) and node.module == 'cluster_renderer')
  assert configure < render_import


@pytest.mark.parametrize('failure', [RuntimeError, KeyboardInterrupt])
def test_preview_closes_native_window_if_open_is_interrupted(tmp_path, monkeypatch, failure):
  import preview_cluster
  import types
  calls = []
  class Renderer:
    def __init__(self, **kwargs):
      pass
    def open(self, *, hidden):
      assert hidden
      calls.append('open')
      raise failure('font setup failed after context creation')
    def close(self):
      calls.append('close')
  module = types.ModuleType('cluster_renderer')
  module.ClusterUiRenderer = Renderer
  monkeypatch.setitem(sys.modules, 'cluster_renderer', module)
  monkeypatch.setattr(preview_cluster, 'configure_preview_graphics', lambda: calls.append('configure'))
  monkeypatch.setattr(sys, 'argv', ['preview_cluster.py', str(tmp_path)])
  with pytest.raises(failure, match='font setup failed'):
    preview_cluster.main()
  assert calls == ['configure', 'open', 'close']
