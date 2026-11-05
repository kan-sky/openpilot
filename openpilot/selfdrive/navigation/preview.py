"""One small tmpfs image shared by Web and USB display (never model input)."""
import os
import json
import hashlib
from io import BytesIO
from pathlib import Path
import time

PREVIEW_PATH = Path('/dev/shm/carrot_nav_map.png')
PREVIEW_MAX_AGE = 2.
WEB_DEMAND_PATH = Path('/dev/shm/carrot_nav_web')
STATUS_PATH = Path('/dev/shm/carrot_nav_status.json')


def preview_fresh(path=PREVIEW_PATH):
  try:
    return 0 <= time.time() - path.stat().st_mtime < PREVIEW_MAX_AGE  # noqa: TID251 - filesystem mtime is wall clock
  except OSError:
    return False


def diagnostic_provenance(sm, now):
  """Snapshot original context deadlines, never renew guidance for a status file."""
  from openpilot.selfdrive.navigation.provenance import SOURCES, ROUTE_MAX_AGE, TBT_MAX_AGE, context_ready, fresh, route_point_limit
  try:
    route = sm['navRoute']
    received = sm.recv_time['navRoute']
    if (not sm.valid['navRoute'] or route.navigationSource not in SOURCES or not route.carrotRouteId
        or not 2 <= len(route.coordinates) <= route_point_limit(route.navigationSource)
        or not fresh(received, now, ROUTE_MAX_AGE)):
      return {}
    expires = received + ROUTE_MAX_AGE
    instruction = sm['navInstructionCarrot']
    context_expires = min(expires, sm.recv_time['navInstructionCarrot'] + TBT_MAX_AGE,
                          instruction.instructionMonoTime / 1e9 + TBT_MAX_AGE,
                          instruction.distanceMonoTime / 1e9 + TBT_MAX_AGE) if context_ready(sm, now) else 0.
    # UInt64 route generations must survive JSON/JavaScript without rounding.
    return {'source': route.navigationSource, 'route_id': str(route.carrotRouteId), 'updated': now,
            'route_expires': expires, 'context_expires': context_expires}
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return {}


def write_preview(image, path=PREVIEW_PATH, *, reason='ready', provenance=None, rendered_at=None):
  from PIL import Image
  buffer = BytesIO()
  Image.fromarray(image).save(buffer, format='PNG', compress_level=1)
  png = buffer.getvalue()
  temporary = path.with_suffix('.tmp')
  temporary.write_bytes(png)
  os.replace(temporary, path)
  status = path.with_name(STATUS_PATH.name)
  temporary = status.with_suffix('.tmp')
  temporary.write_text(json.dumps({'reason': reason, 'provenance': provenance or {},
                                   'updated': time.monotonic(), 'rendered_at': rendered_at,
                                   'png_sha256': hashlib.sha256(png).hexdigest()}), encoding='utf-8')
  os.replace(temporary, status)


def preview_reason():
  try:
    return str(json.loads(STATUS_PATH.read_text(encoding='utf-8'))['reason'])
  except (OSError, ValueError, KeyError):
    return 'waiting'
