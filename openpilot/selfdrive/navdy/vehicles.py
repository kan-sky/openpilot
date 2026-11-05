"""Bounded Navdy-only radar overlay. Never publishes or modifies control inputs."""
from __future__ import annotations

from itertools import islice
import math

from openpilot.selfdrive.navdy.display import (
  _lead_lane, finite_float, model_line_y_at, navdy_project_point, rounded,
)

MAX_TRACKS = 64
MAX_VEHICLES = 8
TRACK_MAX_AGE = 0.35
MAX_DISTANCE = 80.0


def vehicle_lane(model, distance, lateral):
  lane = _lead_lane(model, distance, lateral)
  if lane:
    return lane
  # Wayon's display fallback uses the path and measured/default lane width.
  path_y = model_line_y_at(getattr(model, 'position', None), distance)
  if path_y is None:
    return ''
  lanes = list(islice(getattr(model, 'laneLines', ()), 4))
  probs = list(islice(getattr(model, 'laneLineProbs', ()), 4))
  boundaries = [model_line_y_at(line, distance) for line in lanes]
  if len(boundaries) >= 4 and len(probs) >= 4:
    if boundaries[0] is not None and lateral < boundaries[0] and min(probs[:2]) >= 0.2:
      return ''
    if boundaries[3] is not None and lateral > boundaries[3] and min(probs[2:4]) >= 0.2:
      return ''
  width = 3.6
  if len(boundaries) >= 3 and boundaries[1] is not None and boundaries[2] is not None:
    measured = boundaries[2] - boundaries[1]
    if 2.4 <= measured <= 4.8:
      width = measured
  offset = lateral - path_y
  if abs(offset) <= width / 2:
    return 'center'
  if -1.5 * width <= offset < -width / 2:
    return 'left'
  if width / 2 < offset <= 1.5 * width:
    return 'right'
  return ''


def vehicle_yaw(model, lane, distance, screen_x):
  offset = abs(screen_x - 160.0)
  angle = next((angle for bound, angle in ((24, 0), (48, 8), (72, 12), (104, 16)) if offset < bound), 24)
  yaw = -angle if lane == 'left' else angle if lane == 'right' else 0
  near, far = max(0.0, distance - 4.0), min(MAX_DISTANCE, distance + 4.0)
  lanes = list(islice(getattr(model, 'laneLines', ()), 4))
  indices = {'left': (0, 1), 'right': (2, 3)}.get(lane)

  def center(d):
    if indices and len(lanes) > indices[1]:
      ys = [model_line_y_at(lanes[index], d) for index in indices]
      if all(y is not None for y in ys):
        return sum(ys) / 2
    return model_line_y_at(getattr(model, 'position', None), d)

  y0, y1 = center(near), center(far)
  if y0 is not None and y1 is not None and far - near >= 1.0:
    yaw -= math.degrees(math.atan2(y1 - y0, far - near))
  return rounded(max(-24.0, min(24.0, yaw)))


class VehicleOverlay:
  def __init__(self):
    self.tracks = {}
    self.timestamp = 0.0
    self.flipped = None
    self.display_data = None

  def clear(self):
    self.tracks.clear()
    self.timestamp = 0.0
    self.flipped = None
    self.display_data = None

  def update(self, payload, model, data, *, timestamp, now):
    errors = getattr(data, 'errors', None)
    if (not payload['active'] or not payload.get('navPathLeft') or data is None or errors is None
        or any(bool(getattr(errors, name, False)) for name in
               ('canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'))
        or not math.isfinite(timestamp) or not math.isfinite(now) or timestamp <= 0
        or not 0 <= now - timestamp <= TRACK_MAX_AGE):
      self.clear()
      return
    flipped = bool(getattr(data, 'radarTrackFlipped', False))
    display_data = bool(getattr(data, 'hasDisplayData', False))
    if timestamp < self.timestamp or flipped != self.flipped or display_data != self.display_data:
      self.clear()
    self.flipped = flipped
    self.display_data = display_data
    self.tracks = {key: value for key, value in self.tracks.items() if now - value['time'] <= TRACK_MAX_AGE}
    if timestamp > self.timestamp:
      self.timestamp = timestamp
      current = {}
      # An advertised display stream is authoritative even when empty.
      points = getattr(data, 'displayPoints', ()) if display_data else getattr(data, 'points', ())
      for point in islice(points, MAX_TRACKS):
        ident = finite_float(getattr(point, 'trackId', None), math.nan)
        distance = finite_float(getattr(point, 'dRel', None), math.nan)
        lateral = finite_float(getattr(point, 'yRel', None), math.nan)
        speed = finite_float(getattr(point, 'vRel', None), math.nan)
        if (not math.isfinite(ident) or not 0 <= ident <= 2**53 - 1 or ident != int(ident)
            or not 2.0 <= distance <= MAX_DISTANCE or not abs(lateral) <= 15.0 or not abs(speed) <= 100.0):
          continue
        ident = int(ident)
        if ident in current:
          continue
        previous = self.tracks.get(ident)
        # Published liveTracks already includes RadarTrackFlip. Convert only
        # radar-left-positive to model-right-positive; never flip a second time.
        values = {'dRel': distance, 'yRel': -lateral, 'vRel': speed}
        if (previous and timestamp - previous['time'] <= TRACK_MAX_AGE
            and abs(previous['dRel'] - distance) <= max(8.0, distance * 0.35)
            and abs(previous['yRel'] + lateral) <= 2.5):
          values = {key: previous[key] + 0.55 * (value - previous[key]) for key, value in values.items()}
          samples = min(2, previous['samples'] + 1)
        else:
          samples = 1
        current[ident] = dict(values, time=timestamp, samples=samples)
      # A fresh complete radar snapshot may explicitly remove a target.
      self.tracks = current

    leads = payload['navVehicles']
    surrounding = []
    for ident, track in self.tracks.items():
      if track['samples'] < 2:
        continue
      distance, lateral = track['dRel'], track['yRel']
      lane = vehicle_lane(model, distance, lateral)
      path_y = model_line_y_at(getattr(model, 'position', None), distance)
      if not lane:
        continue
      # Preserve actual lead positions/highlights; raw points only fill in
      # additional vehicles. Display streams may assign independent IDs; use
      # tight geometric matching there, never ID equality across namespaces.
      if any((not display_data and lead['trackId'] == ident) or
             ((display_data or lead['source'] == 'vision') and lead['lane'] == lane
              and abs(lead['distanceM'] - distance) <= (max(2.5, distance * 0.06) if display_data else max(5.0, distance * 0.25))
              and abs(lead['lateralM'] - lateral) <= (1.0 if display_data else 1.5)
              and abs(lead['relativeSpeedMps'] - track['vRel']) <= (3.0 if display_data else 5.0)) for lead in leads):
        continue
      screen = navdy_project_point(distance, lateral, path_y)
      if not screen:
        continue
      surrounding.append({
        'trackId': ident, 'screenX': rounded(max(4.0, min(316.0, screen[0]))), 'screenY': screen[1],
        'yawDeg': vehicle_yaw(model, lane, distance, screen[0]), 'distanceM': rounded(distance),
        'relativeSpeedMps': rounded(track['vRel']), 'lateralM': rounded(lateral, 2),
        'lane': lane, 'source': 'radar', 'longitudinalLead': False, 'confidence': 0.65,
      })
    surrounding.sort(key=lambda vehicle: vehicle['distanceM'])
    selected = leads + surrounding[:max(0, MAX_VEHICLES - len(leads))]
    payload['navVehicles'] = sorted(selected, key=lambda vehicle: vehicle['distanceM'], reverse=True)
