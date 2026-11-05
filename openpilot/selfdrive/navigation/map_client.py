"""Nonblocking optional N16 inputs, paired by source, route and observation time."""
import time

import numpy as np

from openpilot.selfdrive.navigation.helpers import finite_number, navigation_input
from openpilot.selfdrive.navigation.provenance import context_ready, fresh, route_point_limit
from openpilot.selfdrive.navigation.preview import diagnostic_provenance

MAP_SHAPE = (1, 1, 256, 256)
NAVIGATION_SHAPE = (1, 47)
MAP_MAX_AGE = .5
TBT_MAX_AGE = 2.
ROUTE_MAX_AGE = 3.  # upstream selected-route heartbeats are two seconds apart
RETRY_SECONDS = 1.


def _make_context():
  from openpilot.cereal.messaging import SubMaster
  return SubMaster(['navInstructionCarrot', 'navRoute'])


def _make_client():
  # VisionIPC connect(False) can still block in its native FD handshake.
  # Optional input must not make camera inference wait for that listener.
  from openpilot.selfdrive.navigation.map_frame import SnapshotClient
  return SnapshotClient()


def _supports(model):
  views = getattr(model, 'views', {})
  if not getattr(model, 'active', False) or getattr(model, 'usbgpu', False) or not all(
      getattr(views.get(name), 'shape', None) == shape
      for name, shape in (('nav_map', MAP_SHAPE), ('navigation', NAVIGATION_SHAPE))):
    return False
  from openpilot.selfdrive.modeld.jetlink.contracts import N16_SPEC
  return getattr(model, 'spec', None) == N16_SPEC


class MapInput:
  def __init__(self):
    self.sm = self.client = None
    self.retry_at = self.context_retry_at = 0.
    self.image = None
    self.image_time = 0.
    self.route_time = 0
    self.selected_route_id = 0
    self.route_changed_at = 0.
    self.map = self.navigation = None
    self.status = {}
    self._status(False, 'unsupported_model')

  def _status(self, supported, reason, *, route=False, tbt=False, image=False):
    self.status = {
      'nav_input_supported': supported,
      'nav_input_not_yet_available': supported and not (image or tbt),
      'nav_route_available': route,
      'nav_tbt_available': tbt,
      'nav_map_available': image,
      'nav_input_reason': reason,
      'nav_provenance': diagnostic_provenance(self.sm, time.monotonic()) if route and self.sm is not None else {},
    }

  def _read_map(self, now):
    try:
      if self.client is None:
        if now < self.retry_at:
          return
        self.client = _make_client()
      if not self.client.is_connected() and now >= self.retry_at:
        self.client.connect(False)
        self.retry_at = now + RETRY_SECONDS
      if not self.client.is_connected():
        self.image = None
        return
      buf = self.client.recv(0)
      if buf is not None:
        self.image = None
        if (buf.width, buf.height) == (256, 256) and len(buf.data) >= 65536:
          self.image = np.frombuffer(buf.data, np.uint8, count=65536).reshape(MAP_SHAPE).copy()
          self.image_time = self.client.timestamp_eof * 1e-9
          self.route_time = self.client.timestamp_sof
    except Exception:
      # Optional map IPC must never interrupt camera inference or its fallback.
      self.client = self.image = None
      self.retry_at = time.monotonic() + RETRY_SECONDS

  def update(self, model, speed=0.):
    if not _supports(model):
      self.sm = self.client = self.image = None
      self.map = self.navigation = None
      self.selected_route_id = 0
      self.retry_at = self.context_retry_at = 0.
      self._status(False, 'unsupported_model')
      return {}
    if self.map is None:
      self.map = np.zeros(MAP_SHAPE, np.float32)
      self.navigation = np.zeros(NAVIGATION_SHAPE, np.float32)
    self.map.fill(0)
    self.navigation.fill(0)
    result = {'nav_map': self.map, 'navigation': self.navigation}
    self._status(True, 'navigation_not_yet_available')
    now = time.monotonic()
    try:
      if self.sm is None:
        if now < self.context_retry_at:
          return result
        self.sm = _make_context()
      self.sm.update(0)
      # SubMaster assigns receive timestamps during update, after our entry clock.
      now = time.monotonic()
      sm = self.sm
      route = sm['navRoute']
      route_id = getattr(route, 'carrotRouteId', 0)
      self._status(True, 'route_unavailable')
      if (not sm.valid['navRoute'] or not route_id
          or not 2 <= len(route.coordinates) <= route_point_limit(route.navigationSource)
          or not fresh(sm.recv_time['navRoute'], now, ROUTE_MAX_AGE)):
        self.image = None
        self.selected_route_id = 0
        return result
      if route_id != self.selected_route_id:
        self.selected_route_id = route_id
        self.route_changed_at = sm.recv_time['navRoute']
        self.image = None
      self._status(True, 'tbt_unavailable', route=True)
      if (not sm.valid['navInstructionCarrot']
          or not fresh(sm.recv_time['navInstructionCarrot'], now, TBT_MAX_AGE)
          or not context_ready(sm, now)
          or not finite_number(speed) or speed < 0):
        self.image = None
        return result
      self._read_map(now)
      # Even nonblocking native calls have overhead; age at consumption, not entry.
      now = time.monotonic()
      if not context_ready(sm, now):
        self.image = None
        self._status(True, 'context_expired')
        return result
      navigation = navigation_input(sm['navInstructionCarrot'].allManeuvers, speed * (now - sm['navInstructionCarrot'].distanceMonoTime / 1e9))
      if not navigation[:-1].any():
        self.image = None
        return result
      image_valid = (self.image is not None and self.route_time == route_id
                     and fresh(self.image_time, now, MAP_MAX_AGE))
      self.navigation[0] = navigation
      if image_valid:
        self.map[:] = self.image
      self._status(True, 'ready' if image_valid else 'map_not_yet_available', route=True, tbt=True, image=image_valid)
    except Exception:
      self.map.fill(0)
      self.navigation.fill(0)
      self.sm = self.client = self.image = None
      self.selected_route_id = 0
      self.context_retry_at = self.retry_at = time.monotonic() + RETRY_SECONDS
      self._status(True, 'navigation_unavailable')
    return result
