#!/usr/bin/env python3
"""Carrot's shared CPU map renderer; model, Web and USB reuse the same image."""
import os
import time

import numpy as np

from openpilot.selfdrive.navigation.helpers import ParkedPosition
from openpilot.selfdrive.navigation.jetlink_demand import jetlink_map_demand
from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.preview import diagnostic_provenance, write_preview, preview_fresh, WEB_DEMAND_PATH
from openpilot.selfdrive.navigation.renderer import SIZE, MapRenderer, Position
from openpilot.selfdrive.navigation.route import RouteTracker, route_coordinates
from openpilot.selfdrive.navigation.provenance import context_ready, route_point_limit
from openpilot.selfdrive.navigation.map_frame import write_frame


def render_interval(cpu_seconds):
  # At most 10 Hz / approximately 10% of one CPU. A >40 ms render can cause
  # the model's unchanged 500 ms freshness gate to reject the map.
  return max(.1, cpu_seconds * 10)


def main():
  from openpilot.selfdrive.navigation.tiles import RemoteTiles
  from openpilot.selfdrive.navigation.mapbox_tiles import MapboxTiles
  import openpilot.cereal.messaging as messaging
  from openpilot.common.params import Params
  from msgq.visionipc import VisionIpcServer, VisionStreamType
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
  from openpilot.common.swaglog import cloudlog

  # Auxiliary work shares only the little cores. Never compete at FIFO priority
  # with camera/control, radar, driving model or DM on cores 4..7.
  if hasattr(os, 'sched_setaffinity'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    allowed = os.sched_getaffinity(0) & {0, 1, 2, 3}
    if allowed:
      os.sched_setaffinity(0, allowed)
    os.nice(15)
  sm = messaging.SubMaster(['carrotMan', 'gpsLocation', 'gpsLocationExternal', 'navRoute', 'navInstructionCarrot', 'carState'])
  position = ParkedPosition()
  server = VisionIpcServer('mapsd')
  server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_MAP, 4, SIZE, SIZE, SIZE * SIZE, SIZE, SIZE * SIZE)
  server.start_listener()
  params = Params()
  demand_checked, demand = 0., False
  mapbox_token = ''
  archive_created = 0.
  diagnostics = RuntimeDiagnostics('mapsd', cloudlog.event, interval=5.)
  rk = Ratekeeper(10., print_delay_threshold=None)
  route, tracker, archive, renderer = np.empty((0, 2)), None, None, None
  retry_at = next_render = next_preview = 0.
  was_ready = False
  route_id = 0
  image = np.zeros((SIZE, SIZE), np.uint8)
  try:
    while True:
      sm.update(0)
      now = time.monotonic()
      if sm['navRoute'].carrotRouteId != route_id:
        route_id = sm['navRoute'].carrotRouteId
        route = route_coordinates(sm['navRoute']) if sm.valid['navRoute'] else np.empty((0, 2))
        tracker = RouteTracker(route) if (route_id and 2 <= len(route) <= route_point_limit(sm['navRoute'].navigationSource)
                                         and np.isfinite(route).all()) else None
        next_render = 0.
      gps = position.update(sm)
      fresh_tbt = context_ready(sm, now)
      # Navigation without usable position/instruction immediately publishes black.
      if now >= demand_checked:
        demand = (params.get_bool('UsbGpuActive') or jetlink_map_demand(now=now) or
                  params.get_int('ClusterHudScreenMode') == 7 or preview_fresh(WEB_DEMAND_PATH))
        token = os.environ.get('MAPBOX_TOKEN') or params.get('MapboxPublicKey') or ''
        if isinstance(token, bytes):
          token = token.decode('utf-8')
        if token != mapbox_token or (archive is not None and now - archive_created >= 3600):
          if archive is not None:
            archive.close()
          archive = renderer = None
          retry_at = next_render = 0.
          mapbox_token = token
        demand_checked = now + 1.
      ready = demand and tracker is not None and gps is not None and fresh_tbt
      if now >= next_render or (was_ready and not ready):
        started, cpu_started = time.monotonic(), time.process_time()
        rendered_context = diagnostic_provenance(sm, started)
        image.fill(0)
        valid = False
        reason = 'idle' if not demand else 'route' if tracker is None else 'position' if gps is None else 'instruction' if not fresh_tbt else 'tiles'
        try:
          if ready and now >= retry_at:
            if renderer is None:
              archive = MapboxTiles(mapbox_token) if mapbox_token else PMTiles(RemoteTiles())
              archive_created = now
              renderer = MapRenderer(archive)
            pose = tracker.update_pose(gps.longitude, gps.latitude, gps.speed * (now - gps.time))
            # Fetch only the current view; do not prefetch a whole country/route.
            if pose is not None:
              lon, lat, bearing = pose
            result = renderer.render(Position(lat, lon, bearing), route) if pose is not None else None
            valid = result is not None and result.valid
            reason = 'ready' if valid else 'route_position' if pose is None else 'tiles'
            if valid:
              image = result.image
        except (OSError, ValueError, RuntimeError):
          cloudlog.warning('mapsd: map unavailable; retry in 30 seconds')
          if archive is not None:
            archive.close()
          archive = renderer = None
          retry_at = time.monotonic() + 30.
        finished = time.monotonic()
        # A tile request may outlive route cancellation or source replacement.
        sm.update(0)
        if not context_ready(sm, time.monotonic()) or sm['navRoute'].carrotRouteId != route_id:
          image.fill(0)
          valid = False
          reason = 'context_changed'
        if finished - started >= .5:
          image.fill(0)
          valid = False
          reason = 'slow'
        cpu = time.process_time() - cpu_started
        next_render = finished + (render_interval(cpu) if valid else max(1., render_interval(cpu)))
        # Timestamp at render START prevents a slow tile request looking fresh.
        server.send(VisionStreamType.VISION_STREAM_MAP, image.tobytes(), rk.frame,
                    route_id if valid else 0, int(started * 1e9))
        try:
          write_frame(image, route_id if valid else 0, int(started * 1e9))
        except OSError:
          cloudlog.warning('mapsd: model snapshot unavailable')
        if finished >= next_preview:
          current_context = diagnostic_provenance(sm, time.monotonic())
          same_context = all(rendered_context.get(key) == current_context.get(key) for key in ('source', 'route_id'))
          write_preview(image, reason=reason if same_context else 'context_changed',
                        provenance=current_context, rendered_at=started)
          next_preview = finished + .5
        diagnostics.record(render_ms=(finished - started) * 1000, cpu_ms=cpu * 1000,
                           context={'map_valid': valid, 'route_points': len(route), 'interval': render_interval(cpu),
                                    'parked_position_hold': position.held})
      was_ready = ready
      rk.keep_time()
  finally:
    if archive is not None:
      archive.close()


if __name__ == '__main__':
  main()
