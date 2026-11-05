import json
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

import openpilot.cereal as cereal
from openpilot.selfdrive.navigation import mapsd, preview
from openpilot.selfdrive.navigation.tests.test_map_client import Context


@pytest.mark.parametrize('replacement', [None, 'source', 'generation', 'cancel'])
def test_renderer_diagnostics_use_post_render_context_without_changing_model_frames(monkeypatch, tmp_path, replacement):
  context = Context()
  context['navRoute'].navigationSource = context['navInstructionCarrot'].navigationSource = 'navdy'
  context['navRoute'].coordinates *= 2823
  frames, diagnostics, renders = [], [], []
  class Done(Exception):
    pass
  def stop():
    raise Done
  def update(timeout):
    context.updates.append(timeout)
    if len(context.updates) == 2:
      if replacement == 'source':
        context['navRoute'].navigationSource = context['navInstructionCarrot'].navigationSource = 'network-v2'
        context['navRoute'].coordinates = context['navRoute'].coordinates[:2]
      elif replacement in ('generation', 'cancel'):
        context['navRoute'].carrotRouteId = context['navInstructionCarrot'].carrotRouteId = 124 if replacement == 'generation' else 0
  context.update = update
  messaging = NS(SubMaster=lambda _: context)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal.messaging', messaging)
  # `from openpilot.cereal import messaging` can reuse the preloaded package attribute.
  monkeypatch.setattr(cereal, 'messaging', messaging, raising=False)
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(
    get_bool=lambda _: False, get_int=lambda _: 7, get=lambda _: '')))
  monkeypatch.setitem(sys.modules, 'msgq.visionipc', NS(
    VisionIpcServer=lambda _: NS(create_buffers_with_sizes=lambda *_: None, start_listener=lambda: None,
                                send=lambda *args: frames.append(args)),
    VisionStreamType=NS(VISION_STREAM_MAP=1)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.realtime', NS(Ratekeeper=lambda *_args, **_kw: NS(frame=1, keep_time=stop)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.runtime_diagnostics', NS(
    RuntimeDiagnostics=lambda *_args, **_kw: NS(record=lambda **record: diagnostics.append(record))))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(event=lambda *_: None, warning=lambda *_: None)))
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.navigation.tiles', NS(RemoteTiles=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.navigation.mapbox_tiles', NS(MapboxTiles=lambda _: None))
  monkeypatch.setattr(mapsd, 'ParkedPosition', lambda: NS(held=False, update=lambda _: NS(longitude=127., latitude=37., speed=0., time=10.)))
  monkeypatch.setattr(mapsd, 'RouteTracker', lambda _: NS(update_pose=lambda *_: (127., 37., 0.)))
  monkeypatch.setattr(mapsd, 'PMTiles', lambda _: NS(close=lambda: None))
  def render(_position, route):
    renders.append(len(route))
    return NS(valid=True, image=np.full((256, 256), 128, np.uint8))
  monkeypatch.setattr(mapsd, 'MapRenderer', lambda _: NS(render=render))
  monkeypatch.setattr(mapsd, 'jetlink_map_demand', lambda **_: False)
  monkeypatch.setattr(mapsd, 'write_frame', lambda *_: None)
  monkeypatch.setattr(mapsd.time, 'monotonic', lambda: 10.)
  monkeypatch.delattr(mapsd.os, 'sched_setaffinity', raising=False)
  monkeypatch.delenv('MAPBOX_TOKEN', raising=False)
  path = tmp_path / 'map.png'
  monkeypatch.setattr(mapsd, 'write_preview', lambda image, **kwargs: preview.write_preview(image, path, **kwargs))
  with pytest.raises(Done):
    mapsd.main()
  metadata = json.loads(path.with_name(preview.STATUS_PATH.name).read_text())
  assert renders == [5646]
  assert metadata['rendered_at'] == 10.
  assert metadata['reason'] == ('ready' if replacement is None else 'context_changed')
  if replacement == 'cancel':
    assert metadata['provenance'] == {}
  else:
    assert metadata['provenance']['source'] == ('network-v2' if replacement == 'source' else 'navdy')
    assert metadata['provenance']['route_id'] == ('124' if replacement == 'generation' else '123')
  # Diagnostic source checking does not alter the existing route-ID model gate.
  valid = replacement not in ('generation', 'cancel')
  assert frames[0][3:] == (123 if valid else 0, 10_000_000_000)
  assert bool(any(frames[0][1])) == valid
  assert diagnostics[0]['context']['map_valid'] == valid
