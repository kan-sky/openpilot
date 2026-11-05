"""Bind model navigation to one selected route and original guidance observation."""
from dataclasses import dataclass
import threading
import time

from openpilot.selfdrive.navigation.helpers import finite_number

ROUTE_MAX_AGE = 3.
TBT_MAX_AGE = 2.
SOURCES = frozenset(('mapbox', 'network-v1', 'network-v2', 'navdy'))
NAVDY_ROUTE_MAX_POINTS = 8192


def route_point_limit(source):
  return NAVDY_ROUTE_MAX_POINTS if source == 'navdy' else 4096


def fresh(stamp, now, limit):
  return finite_number(stamp) and stamp > 0 and 0 <= now - stamp < limit


def context_ready(sm, now):
  route, instruction = sm['navRoute'], sm['navInstructionCarrot']
  return bool(sm.valid['navRoute'] and sm.valid['navInstructionCarrot']
              and route.carrotRouteId > 0 and 2 <= len(route.coordinates) <= route_point_limit(route.navigationSource)
              and route.navigationSource in SOURCES
              and instruction.carrotRouteId == route.carrotRouteId
              and instruction.navigationSource == route.navigationSource
              and fresh(sm.recv_time['navRoute'], now, ROUTE_MAX_AGE)
              and fresh(sm.recv_time['navInstructionCarrot'], now, TBT_MAX_AGE)
              and fresh(instruction.instructionMonoTime / 1e9, now, TBT_MAX_AGE)
              and fresh(instruction.distanceMonoTime / 1e9, now, TBT_MAX_AGE)
              and instruction.instructionMonoTime <= instruction.distanceMonoTime)


@dataclass(frozen=True)
class SelectedRoute:
  source: str = ''
  generation: int = 0
  observed: int = 0
  points: tuple = ()
  session: str = ''


class RoutePublication:
  """Only the main publisher emits routes; network threads offer immutable data."""
  def __init__(self):
    self.lock = threading.Lock()
    self.candidates = {}
    self.selected = SelectedRoute()
    self.next_publish = 0.
    self.previous_source = ''

  def offer(self, source, points, *, session='', observed=0, generation=0):
    if source not in SOURCES:
      return
    values = []
    if len(points) <= route_point_limit(source):
      for point in points:
        lat, lon = ((point['latitude'], point['longitude']) if isinstance(point, dict)
                    else (point.latitude, point.longitude))
        if (not finite_number(lat) or not finite_number(lon)
            or abs(lat) > 85.05112878 or abs(lon) > 180):
          values = []
          break
        values.append((float(lat), float(lon)))
    points = tuple(values) if len(values) >= 2 else ()
    with self.lock:
      old = self.candidates.get(source)
      if old and old.points == points and old.session == session and (source != 'mapbox' or old.generation == generation):
        return
      now = time.monotonic_ns()
      if source == 'mapbox' and not generation:
        points = ()
      self.candidates[source] = SelectedRoute(source, (generation or now) if points else 0,
                                              observed if 0 < observed <= now else now, points, session)

  def choose(self, source):
    with self.lock:
      # An external disconnect must not resurrect an old route on reconnection.
      if self.previous_source and self.previous_source != source:
        self.candidates.pop(self.previous_source, None)
      self.previous_source = source
      self.selected = self.candidates.get(source, SelectedRoute(source=source))
      return self.selected

  def publish(self, source, pm):
    from openpilot.cereal import messaging
    previous = self.selected
    selected = self.choose(source)
    now = time.monotonic()
    if selected != previous or now >= self.next_publish:
      msg = messaging.new_message('navRoute', valid=True)
      msg.navRoute.carrotRouteId = selected.generation
      msg.navRoute.navigationSource = selected.source
      msg.navRoute.coordinates = [{'latitude': lat, 'longitude': lon} for lat, lon in selected.points]
      pm.send('navRoute', msg)
      self.next_publish = now + 2.
    return selected
