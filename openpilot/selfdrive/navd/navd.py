#!/usr/bin/env python3
import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor

import requests
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.navd.helpers import (Coordinate, coordinate_from_param,
                                    distance_along_geometry, maxspeed_to_ms,
                                    minimum_distance,
                                    parse_banner_instructions)
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import ParkedPosition


REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
NAV_ROUTE_MAX_POINTS = 4096
ROUTE_RESPONSE_MAX_BYTES = 8 * 1024 * 1024
ROUTE_REQUEST_TIMEOUT = 15.


def limit_route_points(points, max_points=NAV_ROUTE_MAX_POINTS):
  if max_points <= 0:
    return []
  count = len(points)
  if count <= max_points:
    return list(points)

  limited = []
  last_index = count - 1
  previous_index = -1
  for i in range(max_points):
    source_index = round(i * last_index / max(1, max_points - 1))
    if source_index == previous_index:
      continue
    limited.append(points[source_index])
    previous_index = source_index
  return limited


class RouteEngine:
  def __init__(self, sm, pm):
    self.name = "RouteEngine Instance"
    self.sm = sm
    self.pm = pm

    self.params = Params()

    # A saved position is not fresh GPS evidence, including at startup in Park.
    self.last_position = None
    self.last_bearing = None

    self.gps_ok = False
    self.localizer_valid = False
    self.position = ParkedPosition()

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None
    self.route_id = 0
    self.generation = 0
    self.context = None

    self.recompute_backoff = 0
    self.recompute_countdown = 0

    self.ui_pid = None

    self.reroute_counter = 0


    self.mapbox_token = ""
    self.mapbox_host = "https://api.mapbox.com"
    self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mapbox-route")
    self.pending = None
    self.pending_destination = None
    self.pending_token = None
    self.pending_waypoints = None
    self.pending_generation = None
    self.pending_started = None
    self.resend_at = time.monotonic() + 5.

  def refresh_token(self):
    token = os.environ.get("MAPBOX_TOKEN") or self.params.get("MapboxPublicKey") or ""
    if isinstance(token, bytes):
      token = token.decode('utf-8')
    if token != self.mapbox_token:
      self.mapbox_token = token
      self.clear_route()
      self.reset_recompute_limits()

  def sync_route_context(self):
    self.refresh_token()
    try:
      destination, place_name = coordinate_from_param('NavDestination', self.params)
    except (ValueError, TypeError, KeyError):
      destination, place_name = None, None
    external = (self.sm.valid['carrotMan'] and self.sm.alive['carrotMan']
                and self.sm['carrotMan'].activeCarrot > 0)
    source = self.params.get_int('CarrotNaviSource')
    waypoints = self.params.get('NavDestinationWaypoints') or ''
    context = (source, external, self.mapbox_token,
               None if destination is None else (destination.latitude, destination.longitude),
               place_name == 'External Navi', waypoints)
    if context != self.context:
      self.context = context
      self.clear_route()
      self.reset_recompute_limits()
    # Explicit Navdy selection never falls back to Mapbox, even on disconnect.
    allowed = source == 0 and not external and place_name != 'External Navi' and bool(self.mapbox_token)
    return destination, allowed

  def update(self):
    self.sm.update(0)

    if self.sm.updated["managerState"]:
      ui_pid = [p.pid for p in self.sm["managerState"].processes if p.name == "ui" and p.running]
      if ui_pid:
        if self.ui_pid and self.ui_pid != ui_pid[0]:
          cloudlog.warning("UI restarting, sending route")
          print("########## UI restarting, sending route")
          self.resend_at = time.monotonic() + 5.
        self.ui_pid = ui_pid[0]

    self.update_location()
    self.sync_route_context()
    self.poll_route()
    if self.resend_at and time.monotonic() >= self.resend_at:
      self.send_route()
      self.resend_at = time.monotonic() + 5.

    try:
      self.recompute_route()
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
      self.send_invalid_instruction()

  def update_location(self):
    gps = self.position.update(self.sm, max_poll_gap=1.5)
    self.localizer_valid = self.gps_ok = gps is not None
    if gps is not None:
      self.last_position = Coordinate(gps.latitude, gps.longitude)
      self.last_bearing = gps.bearing

  def recompute_route(self):
    new_destination, allowed = self.sync_route_context()
    if not allowed or new_destination is None or not self.gps_ok or self.last_position is None:
      return
    should_recompute = self.should_recompute()
    if new_destination != self.nav_destination:
      if self.nav_destination is not None:
        self.clear_route()
        self.reset_recompute_limits()
      should_recompute = True

    if self.pending is None and self.recompute_countdown == 0 and should_recompute:
      self.recompute_countdown = 2**self.recompute_backoff
      self.recompute_backoff = min(6, self.recompute_backoff + 1)
      self.calculate_route(new_destination)
      self.reroute_counter = 0
    else:
      self.recompute_countdown = max(0, self.recompute_countdown - 1)

  @staticmethod
  def request_route(url, params):
    # Network work is isolated; the 1 Hz instruction publisher never waits on it.
    try:
      deadline = time.monotonic() + ROUTE_REQUEST_TIMEOUT
      with requests.get(url, params=params, timeout=(3, 8), stream=True, allow_redirects=False) as response:
        if response.status_code != 200:
          return None
        response.raise_for_status()
        body = bytearray()
        for chunk in response.iter_content(65536):
          body.extend(chunk)
          if len(body) > ROUTE_RESPONSE_MAX_BYTES or time.monotonic() >= deadline:
            return None
        return json.loads(body)
    except (requests.RequestException, ValueError, RecursionError):
      # Exceptions may contain access_token in the URL. Never log them.
      return None

  def calculate_route(self, destination):
    current_destination, allowed = self.sync_route_context()
    if (self.pending is not None or not allowed or not self.gps_ok or self.last_position is None
        or destination is None or destination != current_destination):
      return
    if destination != self.nav_destination:
      self.clear_route()
    params = {
      'access_token': self.mapbox_token, 'annotations': 'maxspeed',
      'geometries': 'geojson', 'overview': 'full', 'steps': 'true',
      'banner_instructions': 'true', 'alternatives': 'false',
      'language': (self.params.get('LanguageSetting') or 'en').replace('main_', ''),
    }
    waypoints = self.params.get('NavDestinationWaypoints') or ''
    try:
      middle = json.loads(waypoints) if waypoints else []
      if (not isinstance(middle, list) or len(middle) > 23
          or any(len(p) != 2 or not all(math.isfinite(v) for v in p) or abs(p[0]) > 180 or abs(p[1]) > 85.05112878 for p in middle)):
        return
    except (ValueError, TypeError):
      return
    points = [(self.last_position.longitude, self.last_position.latitude), *middle, (destination.longitude, destination.latitude)]
    coords = ';'.join(f'{lon},{lat}' for lon, lat in points)
    params['waypoints'] = f'0;{len(points)-1}'
    if self.last_bearing is not None:
      params['bearings'] = f'{self.last_bearing % 360:.0f},90' + ';' * (len(points)-1)
    self.pending_waypoints = waypoints
    self.pending_destination, self.pending_token = destination, self.mapbox_token
    self.pending_generation = self.generation
    self.pending_started = time.monotonic()
    self.pending = self.executor.submit(self.request_route, self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords, params)

  def poll_route(self):
    destination, allowed = self.sync_route_context()
    if self.pending is None or not self.pending.done():
      return
    pending = self.pending
    self.pending = None
    if (pending.cancelled() or not allowed or not self.gps_ok or self.pending_generation != self.generation
        or not 0 <= time.monotonic() - self.pending_started < ROUTE_REQUEST_TIMEOUT
        or destination != self.pending_destination or self.mapbox_token != self.pending_token
        or (self.params.get('NavDestinationWaypoints') or '') != self.pending_waypoints):
      return
    try:
      response = pending.result()
    except Exception:
      # A worker exception can embed the authenticated request URL.
      cloudlog.warning('navd: Mapbox route request failed')
      return
    try:
      if not response or not response.get('routes'):
        return
      steps = response['routes'][0]['legs'][0]['steps']
      if not steps or len(steps) > 2048:
        raise ValueError('invalid route step count')
      geometry = []
      maxspeeds = response['routes'][0]['legs'][0].get('annotation', {}).get('maxspeed', [])
      maxspeed_idx = 0
      total = 0
      for step in steps:
        points = step['geometry']['coordinates']
        total += len(points)
        if total > 50000 or not points:
          raise ValueError('invalid route geometry size')
        converted = []
        for lon, lat in points:
          if not math.isfinite(lat) or not math.isfinite(lon) or abs(lat) > 85.05112878 or abs(lon) > 180:
            raise ValueError('invalid route coordinate')
          coord = Coordinate(lat, lon)
          if maxspeed_idx < len(maxspeeds):
            speed = maxspeeds[maxspeed_idx]
            if 'speed' in speed and speed.get('unit') in ('km/h', 'mph'):
              converted_speed = maxspeed_to_ms(speed)
              if not math.isfinite(converted_speed) or converted_speed < 0:
                raise ValueError('invalid route speed limit')
              coord.annotations['maxspeed'] = converted_speed
          maxspeed_idx += 1
          converted.append(coord)
        maxspeed_idx -= 1
        for key in ('distance', 'duration', 'duration_typical'):
          if key == 'duration_typical' and step.get(key) is None:
            continue
          if not isinstance(step[key], (int, float)) or not math.isfinite(step[key]) or step[key] < 0:
            raise ValueError('invalid route distance/duration')
        geometry.append(converted)
        step.setdefault('bannerInstructions', [])
        step.setdefault('duration_typical', None)
        # Validate banners before publishing geometry paired with instructions.
        if not isinstance(step['bannerInstructions'], list):
          raise ValueError('invalid route instructions')
        for banner in step['bannerInstructions']:
          if not math.isfinite(banner['distanceAlongGeometry']):
            raise ValueError('invalid route instruction distance')
          parse_banner_instructions([banner])
      self.sync_route_context()
      if self.pending_generation != self.generation:
        return
      self.route, self.route_geometry = steps, geometry
      self.nav_destination, self.step_idx = destination, 0
      self.route_id = max(1, time.monotonic_ns())
      if self.pending_waypoints:
        self.params.remove('NavDestinationWaypoints')
        self.context = (*self.context[:-1], '')
      self.send_route()
    except (ValueError, KeyError, TypeError, IndexError, AttributeError, OverflowError):
      cloudlog.warning('navd: invalid Mapbox route response')

  def send_instruction(self):
    _, allowed = self.sync_route_context()
    self.update_location()
    calculated_at = time.monotonic_ns()
    msg = messaging.new_message('navInstruction', valid=True)
    if not allowed or not self.gps_ok or self.last_position is None or self.step_idx is None or not self.route_id:
      self.send_invalid_instruction()
      return
    msg.navInstruction.carrotRouteId = self.route_id
    msg.navInstruction.navigationSource = 'mapbox'
    msg.navInstruction.instructionMonoTime = calculated_at
    msg.navInstruction.distanceMonoTime = calculated_at
    step = self.route[self.step_idx]
    geometry = self.route_geometry[self.step_idx]
    along_geometry = distance_along_geometry(geometry, self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)
    if instruction is not None:
      for k,v in instruction.items():
        setattr(msg.navInstruction, k, v)

    # All instructions
    maneuvers = []
    cumulative = np.cumsum([0.] + [float(step_i['distance']) for step_i in self.route])
    for i, step_i in enumerate(self.route):
      distance_to_maneuver = cumulative[i + 1] - cumulative[self.step_idx] - along_geometry

      instruction = parse_banner_instructions(step_i['bannerInstructions'], distance_to_maneuver)
      if instruction is None:
        continue
      # cereal's list-of-struct conversion rejects NumPy scalar distances.
      maneuver = {'distance': float(distance_to_maneuver)}
      if 'maneuverType' in instruction:
        maneuver['type'] = instruction['maneuverType']
      if 'maneuverModifier' in instruction:
        maneuver['modifier'] = instruction['maneuverModifier']
      maneuvers.append(maneuver)

    msg.navInstruction.allManeuvers = maneuvers

    # Compute total remaining time and distance
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    total_distance = step['distance'] * remaining
    total_time = step['duration'] * remaining

    if step['duration_typical'] is None:
      total_time_typical = total_time
    else:
      total_time_typical = step['duration_typical'] * remaining

    # Add up totals for future steps
    for i in range(self.step_idx + 1, len(self.route)):
      total_distance += self.route[i]['distance']
      total_time += self.route[i]['duration']
      if self.route[i]['duration_typical'] is None:
        total_time_typical += self.route[i]['duration']
      else:
        total_time_typical += self.route[i]['duration_typical']

    msg.navInstruction.distanceRemaining = total_distance
    msg.navInstruction.timeRemaining = total_time
    msg.navInstruction.timeRemainingTypical = total_time_typical

    # Speed limit
    closest_idx, closest = min(enumerate(geometry), key=lambda p: p[1].distance_to(self.last_position))
    if closest_idx > 0:
      # If we are not past the closest point, show previous
      if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
        closest = geometry[closest_idx - 1]

    if ('maxspeed' in closest.annotations) and self.localizer_valid:
      msg.navInstruction.speedLimit = closest.annotations['maxspeed']
      print(closest.annotations)


    # Speed limit sign type
    if 'speedLimitSign' in step:
      if step['speedLimitSign'] == 'mutcd':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd
      elif step['speedLimitSign'] == 'vienna':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.reset_recompute_limits()
      else:
        cloudlog.warning("Destination reached")
        print("Destination reached")

        # Clear route if driving away from destination
        dist = self.nav_destination.distance_to(self.last_position)
        if dist > REROUTE_DISTANCE:
          destination, allowed = self.sync_route_context()
          if allowed and destination == self.nav_destination:
            self.params.remove("NavDestination")
          self.clear_route()

  def send_route(self):
    self.sync_route_context()
    self._publish_route()

  def _publish_route(self):
    coords = []

    if self.route is not None:
      for path in self.route_geometry:
        coords.extend(c.as_dict() for c in path)

    original_count = len(coords)
    coords = limit_route_points(coords, NAV_ROUTE_MAX_POINTS)
    if original_count > len(coords):
      cloudlog.warning(f"navd route limited from {original_count} to {len(coords)} points")
      print(f"navd route limited: {original_count} -> {len(coords)}")

    #print("$$$$$$$$$$$$$$$$$$coords=",coords)
    msg = messaging.new_message('navRouteNavd', valid=bool(coords) and bool(self.route_id))
    msg.navRouteNavd.coordinates = coords
    msg.navRouteNavd.carrotRouteId = self.route_id
    msg.navRouteNavd.navigationSource = 'mapbox'
    self.pm.send('navRouteNavd', msg)

  def send_invalid_instruction(self):
    msg = messaging.new_message('navInstruction', valid=False)
    msg.navInstruction.navigationSource = 'mapbox'
    self.pm.send('navInstruction', msg)

  def clear_route(self):
    had_route = self.route is not None or self.route_geometry is not None or self.step_idx is not None or self.nav_destination is not None
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None
    self.route_id = 0
    self.generation += 1
    # Keep the in-flight future until it finishes: source churn cannot queue
    # unbounded jobs behind a running HTTP request. The generation rejects it.
    if self.pending is not None:
      self.pending.cancel()
    if had_route:
      self._publish_route()
      self.send_invalid_instruction()

  def reset_recompute_limits(self):
    self.recompute_backoff = 0
    self.recompute_countdown = 0

  def should_recompute(self):
    if self.step_idx is None or self.route is None:
      return True

    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # Compute closest distance to all line segments in the current path
    min_d = REROUTE_DISTANCE + 1
    path = self.route_geometry[self.step_idx]
    for i in range(len(path) - 1):
      a = path[i]
      b = path[i + 1]

      if a.distance_to(b) < 1.0:
        continue

      min_d = min(min_d, minimum_distance(a, b, self.last_position))

    if min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN
    # TODO: Check for going wrong way in segment

def main():
  pm = messaging.PubMaster(['navInstruction', 'navRouteNavd'])
  sm = messaging.SubMaster(['managerState', 'carrotMan', 'gpsLocation', 'gpsLocationExternal', 'carState'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
