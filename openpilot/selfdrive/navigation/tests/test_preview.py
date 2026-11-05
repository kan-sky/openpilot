import hashlib
import json

import numpy as np
import pytest
from PIL import Image

from openpilot.selfdrive.navigation import preview
from openpilot.selfdrive.navigation.tests.test_map_client import Context


def test_preview_binds_actual_png_to_original_context_and_render_start(tmp_path, monkeypatch):
  monkeypatch.setattr(preview.time, 'monotonic', lambda: 10.)
  context = Context()
  context['navRoute'].navigationSource = context['navInstructionCarrot'].navigationSource = 'navdy'
  context['navRoute'].carrotRouteId = context['navInstructionCarrot'].carrotRouteId = 2**64 - 1
  context['navRoute'].coordinates *= 2823  # Complete 5646-point route; metadata never substitutes geometry.
  provenance = preview.diagnostic_provenance(context, 10.)
  assert provenance == {'source': 'navdy', 'route_id': str(2**64 - 1), 'updated': 10.,
                        'route_expires': 12.6, 'context_expires': 11.7}
  path = tmp_path / 'map.png'
  image = np.arange(65536, dtype=np.uint8).reshape(256, 256)
  preview.write_preview(image, path, provenance=provenance, rendered_at=9.8)
  metadata = json.loads(path.with_name(preview.STATUS_PATH.name).read_text())
  assert metadata['provenance'] == provenance
  assert metadata['rendered_at'] == 9.8 and metadata['updated'] == 10.
  assert metadata['png_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
  np.testing.assert_array_equal(np.asarray(Image.open(path)), image)
  assert not list(tmp_path.glob('*.tmp'))


@pytest.mark.parametrize('change', ['route_stale', 'route_invalid', 'route_future', 'source', 'generation', 'oversize', 'missing'])
def test_invalid_route_cannot_create_provenance(change):
  context = Context()
  if change == 'route_stale':
    context.recv_time['navRoute'] = 7.
  elif change == 'route_future':
    context.recv_time['navRoute'] = 11.
  elif change == 'route_invalid':
    context.valid['navRoute'] = False
  elif change == 'source':
    context['navRoute'].navigationSource = 'unknown'
  elif change == 'generation':
    context['navRoute'].carrotRouteId = 0
  elif change == 'oversize':
    context['navRoute'].coordinates *= 4097
  else:
    del context['navRoute'].carrotRouteId
  assert preview.diagnostic_provenance(context, 10.) == {}


@pytest.mark.parametrize('change', ['expired', 'invalid', 'different_source', 'different_generation'])
def test_guidance_deadlines_are_not_renewed_by_new_diagnostics(change):
  context = Context()
  if change == 'expired':
    context['navInstructionCarrot'].instructionMonoTime = 8_000_000_000
  elif change == 'invalid':
    context.valid['navInstructionCarrot'] = False
  elif change == 'different_source':
    context['navInstructionCarrot'].navigationSource = 'navdy'
  else:
    context['navInstructionCarrot'].carrotRouteId += 1
  provenance = preview.diagnostic_provenance(context, 10.)
  assert provenance['route_id'] == '123'
  assert provenance['context_expires'] == 0.


def test_legacy_writer_cannot_invent_context(tmp_path):
  path = tmp_path / 'map.png'
  preview.write_preview(np.zeros((256, 256), np.uint8), path)
  status = json.loads(path.with_name(preview.STATUS_PATH.name).read_text())
  assert status['provenance'] == {} and status['rendered_at'] is None
