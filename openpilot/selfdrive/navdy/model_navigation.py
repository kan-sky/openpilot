"""Bounded, independently timed model inputs. Never interprets display turn IDs."""
from dataclasses import dataclass, replace
import hashlib
import math
import re
import uuid

from openpilot.selfdrive.navigation.helpers import finite_number, valhalla_type
from openpilot.selfdrive.navigation.provenance import NAVDY_ROUTE_MAX_POINTS

MAX_POINTS = NAVDY_ROUTE_MAX_POINTS
MAX_ENCODED = 65536
CHUNK_SIZE = 1024
MAX_AGE = 2.
ASSEMBLY_AGE = 30.
QUARANTINE_AGE = 10.
HASH = re.compile(r'[0-9a-f]{64}\Z')


def integer(value, low, high):
  return type(value) is int and low <= value <= high


def bounded(value, low, high):
  return finite_number(value) and low <= value <= high


def identity(value):
  if not isinstance(value, dict) or value.get('source') not in ('tmap', 'naver'):
    return None
  parts = tuple(value.get(k) for k in ('source', 'session', 'routeId'))
  return parts if all(isinstance(p, str) and 1 <= len(p) <= 64 for p in parts) else None


def decode_route(encoded, count, digest):
  if (not isinstance(encoded, str) or not 2 <= len(encoded) <= MAX_ENCODED
      or not integer(count, 2, MAX_POINTS) or not isinstance(digest, str) or not HASH.fullmatch(digest)
      or any(not 63 <= ord(c) <= 126 for c in encoded)):
    raise ValueError('invalid route manifest')
  if hashlib.sha256(encoded.encode('ascii')).hexdigest() != digest:
    raise ValueError('route checksum mismatch')
  values, accumulator, shift = [], 0, 0
  for character in encoded:
    byte = ord(character) - 63
    accumulator |= (byte & 31) << shift
    if byte & 32:
      shift += 5
      if shift > 30:
        raise ValueError('polyline integer overflow')
    else:
      if accumulator > 0xffffffff:
        raise ValueError('polyline integer overflow')
      values.append(~(accumulator >> 1) if accumulator & 1 else accumulator >> 1)
      accumulator = shift = 0
      if len(values) > count * 2:
        raise ValueError('too many coordinates')
  if shift or len(values) != count * 2:
    raise ValueError('incomplete polyline')
  lat = lon = 0
  points = []
  for index in range(0, len(values), 2):
    lat += values[index]
    lon += values[index + 1]
    if abs(lat) > 85051128 or abs(lon) > 180000000:
      raise ValueError('invalid coordinates')
    points.append((lat / 1e6, lon / 1e6))
  if len(set(points)) < 2:
    raise ValueError('empty path')
  return tuple(points)


@dataclass(frozen=True)
class ModelNavigation:
  identity: tuple
  points: tuple
  observed: float
  maneuver_type: str
  modifier: str
  distance: float
  position: tuple | None

  @property
  def session(self):
    # Length-prefixed identity cannot collide through delimiters in a source ID.
    return ''.join(f'{len(part)}:{part}' for part in self.identity)


@dataclass(frozen=True)
class _QuarantinedRoute:
  identity: tuple
  manifest: tuple
  points: tuple
  encoded: str
  expired: float


class ModelNavigationTracker:
  def __init__(self):
    self.high_water = {}
    self.source_sessions = {}
    self.retired_producers = set()
    self.current = None
    self.sequence = -1
    self.observed = self.deadline = self.received = -math.inf
    self.value_received = -math.inf
    self.value = None
    self.position_key = None
    self.position_stamp = -math.inf
    self.manifest = None
    self.chunks = {}
    self.points = ()
    self.encoded = None
    self.assembly_started = -math.inf
    self.reason = 'model_feedback_missing'
    self.cache_generation = 0
    self._quarantine = None

  def clear(self, reason='model_feedback_missing'):
    self._quarantine = None
    self.cache_generation += 1
    self.current = self.value = self.manifest = self.encoded = None
    self.chunks = {}
    self.points = ()
    self.position_key = None
    self.position_stamp = -math.inf
    self.sequence = -1
    self.reason = reason

  def clear_geometry(self):
    self._quarantine = None
    self.manifest = self.encoded = None
    self.chunks = {}
    self.points = ()

  def _expire_quarantine(self, now):
    if self._quarantine is not None and (not 0 <= now - self.received < MAX_AGE
        or not self._quarantine.expired <= now < self._quarantine.expired + QUARANTINE_AGE):
      self._quarantine = None

  def _guidance_gap(self, reason, now, *, deadline=None):
    self._expire_quarantine(now)
    saved = self._quarantine
    if self.points and 0 <= now - self.received < MAX_AGE and now >= self.observed:
      saved = _QuarantinedRoute(self.current, self.manifest, self.points, self.encoded, min(now, self.deadline))
    if saved is not None and deadline is not None:
      saved = replace(saved, expired=min(saved.expired, deadline))
    self.clear(reason)
    self._quarantine = saved
    self._expire_quarantine(now)

  def _expired_tail(self, value, key, received, now):
    saved = self._quarantine
    if (key is None or not integer(value.get('sequence'), 1, 2**53 - 1)
        or self.high_water.get(key[:2]) != (value['sequence'], key)):
      return False
    if saved is None:
      if key != self.current or value['sequence'] != self.sequence or not self.points:
        return False
      saved = _QuarantinedRoute(self.current, self.manifest, self.points, self.encoded, min(now, self.deadline))
    if key != saved.identity:
      return False
    age, valid_for = value.get('ageMs'), value.get('validForMs')
    # A valid HUD response can cross expiry when the complete ACK RTT is charged.
    if not bounded(age, 0, 4000) or not bounded(valid_for, 0, 2000):
      return False
    deadline = min(received - age / 1000. + MAX_AGE, received + valid_for / 1000.)
    if deadline > now:
      return False
    maneuver, position, chunk = value.get('maneuver'), value.get('position'), value.get('routeChunk')
    parsed = self._validated_chunk(chunk) if chunk is not None else None
    kind = maneuver.get('type') if isinstance(maneuver, dict) else None
    modifier = maneuver.get('modifier') if isinstance(maneuver, dict) else None
    command = valhalla_type(kind, modifier) if isinstance(kind, str) and isinstance(modifier, str) else None
    valid_position = position is None or (isinstance(position, dict)
      and bounded(position.get('latitude'), -85.05112878, 85.05112878)
      and bounded(position.get('longitude'), -180, 180)
      and bounded(position.get('bearingDegrees'), 0, 359.999999)
      and bounded(position.get('speedMps'), 0, 90)
      and bounded(position.get('ageMs'), 0, 4000)
      and integer(position.get('observedAtMs'), 1, 2**53 - 1))
    if (type(value.get('version')) is not int or value['version'] != 1
        or value.get('active') is not True or value.get('offRoute') is not False
        or age >= 4000 or (age >= 2000 and valid_for != 0)
        or command is None or command <= 6 or not bounded(maneuver.get('distanceMeters'), 0, 1000000)
        or not valid_position or (chunk is not None and (parsed is None or not self._matches_quarantine(*parsed, candidate=saved)))):
      self.clear()
    else:
      self._guidance_gap('model_navigation_stale', now, deadline=deadline)
      self.received = received
    return True

  def accept(self, value, received, now):
    if not bounded(received, 0, now) or now - received >= MAX_AGE:
      self.clear('model_link_stale')
      return
    self._expire_quarantine(now)
    if self.value is not None:
      self._fresh(now)
    if value is None:
      self._guidance_gap('model_feedback_missing', now)
      self.received = received
      return
    key = identity(value)
    if self._expired_tail(value, key, received, now):
      return
    if (key is None or type(value.get('version')) is not int or value['version'] != 1
        or not integer(value.get('sequence'), 1, 2**53 - 1)
        or not bounded(value.get('ageMs'), 0, 2000)
        or not bounded(value.get('validForMs'), 0, 2000)):
      self.clear()
      return
    if self._quarantine is not None and (key != self._quarantine.identity
        or value.get('active') is not True or value.get('offRoute') is not False):
      self._quarantine = None
    if self._quarantine is not None and value.get('routeChunk') is not None:
      parsed = self._validated_chunk(value['routeChunk'])
      if parsed is None or not self._matches_quarantine(*parsed):
        self._quarantine = None
    sequence = value['sequence']
    producer = key[:2]
    if producer in self.retired_producers:
      return
    previous = self.high_water.get(producer)
    if previous and (sequence < previous[0] or (sequence == previous[0] and key != previous[1])):
      return
    # A cleared generation cannot be revived by replaying an old observation.
    if self.current != key and previous and sequence <= previous[0]:
      return
    if producer not in self.high_water and len(self.high_water) >= 16:
      self.clear('model_session_limit')
      return
    previous_session = self.source_sessions.get(key[0])
    if previous_session is not None and previous_session != key[1]:
      self.retired_producers.add((key[0], previous_session))
    self.source_sessions[key[0]] = key[1]
    self.high_water[producer] = sequence, key
    observed = received - value['ageMs'] / 1000.
    deadline = min(observed + MAX_AGE, received + value['validForMs'] / 1000.)
    if (value.get('active') is not True or value.get('offRoute') is not False
        or not 0 <= now - observed < MAX_AGE or deadline <= now):
      self.clear('model_navigation_inactive')
      return
    if key != self.current:
      saved = self._quarantine
      self.clear('model_route_pending')
      self._quarantine = saved
      self.current = key
      self.assembly_started = now
    if sequence == self.sequence:
      self.observed = min(self.observed, observed)
      self.deadline = min(self.deadline, deadline)
    else:
      self.sequence, self.observed, self.deadline = sequence, observed, deadline
      self.value = value
      self.value_received = received
    self.received = received
    if not self.points and now - self.assembly_started > ASSEMBLY_AGE:
      self.chunks = {}
      self.manifest = None
      self.assembly_started = now
    chunk = value.get('routeChunk')
    if chunk is not None:
      self._chunk(chunk, restore=previous is not None and sequence > previous[0])
    if self.current is not None:
      self.reason = '' if self.points else 'model_route_pending'

  @staticmethod
  def _validated_chunk(chunk):
    if not isinstance(chunk, dict):
      return None
    digest, size, count, point_count = (chunk.get(k) for k in ('sha256', 'encodedLength', 'count', 'pointCount'))
    index, data = chunk.get('index'), chunk.get('data')
    if (chunk.get('encoding') != 'polyline6' or not isinstance(digest, str) or not HASH.fullmatch(digest)
        or not integer(size, 2, MAX_ENCODED) or not integer(count, 1, 64)
        or count != (size + CHUNK_SIZE - 1) // CHUNK_SIZE or not integer(point_count, 2, MAX_POINTS)
        or not integer(index, 0, count - 1) or not isinstance(data, str)
        or len(data) != min(CHUNK_SIZE, size - index * CHUNK_SIZE)
        or any(not 63 <= ord(c) <= 126 for c in data)):
      return None
    return (digest, size, count, point_count), index, data

  def _matches_quarantine(self, manifest, index, data, *, candidate=None):
    saved = self._quarantine if candidate is None else candidate
    return (saved is not None and saved.manifest == manifest
            and data == saved.encoded[index * CHUNK_SIZE:(index + 1) * CHUNK_SIZE])

  def _chunk(self, chunk, *, restore=False):
    parsed = self._validated_chunk(chunk)
    if parsed is None:
      self.clear('model_route_invalid')
      return
    manifest, index, data = parsed
    digest, _, count, point_count = manifest
    saved = self._quarantine
    if saved is not None:
      if not self._matches_quarantine(manifest, index, data):
        self._quarantine = None
      elif restore:
        # A new native observation must prove the exact immutable route before
        # any quarantined bytes become active or publishable again.
        self.manifest, self.points, self.encoded = saved.manifest, saved.points, saved.encoded
        self.chunks = {i: saved.encoded[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE] for i in range(count)}
        self._quarantine = None
        return
    if self.manifest is not None and self.manifest != manifest:
      self.clear('model_route_changed_without_id')
      return
    self.manifest = manifest
    if index in self.chunks and self.chunks[index] != data:
      self.clear('model_chunk_conflict')
      return
    self.chunks[index] = data
    if len(self.chunks) == count and not self.points:
      encoded = ''.join(self.chunks[i] for i in range(count))
      try:
        self.points = decode_route(encoded, point_count, digest)
      except ValueError:
        self.clear('model_route_invalid')
        return
      self.encoded = encoded
      self._quarantine = None

  def request(self, now):
    if not self._fresh(now):
      return {'routeId': '', 'index': 0}
    count = self.manifest[2] if self.manifest else 1
    return {'routeId': self.current[2], 'index': next((i for i in range(count) if i not in self.chunks), -1)}

  def route_cache(self):
    if not self.points:
      return None
    return dict(zip(('source', 'session', 'routeId'), self.current),
                sha256=self.manifest[0], pointCount=len(self.points), polyline6=self.encoded)

  def import_route(self, cache):
    if self.current is None or identity(cache) != self.current or self.points:
      return
    try:
      points = decode_route(cache.get('polyline6'), cache.get('pointCount'), cache.get('sha256'))
    except (ValueError, AttributeError):
      return
    encoded = cache['polyline6']
    manifest = cache['sha256'], len(encoded), (len(encoded) + CHUNK_SIZE - 1) // CHUNK_SIZE, len(points)
    if self.manifest is not None and self.manifest != manifest:
      return
    self.manifest, self.points, self.encoded = manifest, points, encoded
    self._quarantine = None
    self.reason = ''

  def is_fresh(self, now):
    return (self.value is not None and 0 <= now - self.received < MAX_AGE
            and 0 <= now - self.observed < MAX_AGE and now < self.deadline)

  def _fresh(self, now):
    self._expire_quarantine(now)
    if self.value is None:
      return False
    if not self.is_fresh(now):
      self._guidance_gap('model_navigation_stale', now)
      return False
    return True

  def snapshot(self, now):
    if not self._fresh(now) or not self.points:
      return None
    maneuver = self.value.get('maneuver')
    if not isinstance(maneuver, dict):
      return None
    kind, modifier, distance = (maneuver.get(k) for k in ('type', 'modifier', 'distanceMeters'))
    if not isinstance(kind, str) or not isinstance(modifier, str):
      return None
    command = valhalla_type(kind, modifier)
    if command is None or command <= 6 or not bounded(distance, 0, 1000000):
      return None
    position = self.value.get('position')
    fix = None
    if isinstance(position, dict):
      lat, lon, heading, speed, age, original = (position.get(k) for k in
          ('latitude', 'longitude', 'bearingDegrees', 'speedMps', 'ageMs', 'observedAtMs'))
      if (bounded(lat, -85.05112878, 85.05112878) and bounded(lon, -180, 180)
          and bounded(heading, 0, 359.999999) and bounded(speed, 0, 90)
          and bounded(age, 0, 2000) and integer(original, 1, 2**53 - 1)):
        stamp = self.value_received - age / 1000.
        if original == self.position_key:
          stamp = min(stamp, self.position_stamp)
        elif self.position_key is not None and original < self.position_key:
          return ModelNavigation(self.current, self.points, self.observed, kind, modifier, float(distance), None)
        self.position_key, self.position_stamp = original, stamp
        if 0 <= now - stamp < MAX_AGE:
          fix = lat, lon, heading, speed, stamp
    return ModelNavigation(self.current, self.points, self.observed, kind, modifier, float(distance), fix)


class ModelRouteCache:
  """Bind each complete bridge assembly to the corresponding local feedback."""
  def __init__(self, path, writer):
    self.path, self.writer = path, writer
    self.key = self.token = None
    self.path.unlink(missing_ok=True)

  def update(self, tracker):
    cache = tracker.route_cache()
    key = (tracker.current, cache['sha256'], tracker.cache_generation) if cache else None
    if key != self.key or (key is not None and not self.path.exists()):
      token = uuid.uuid4().hex if cache else None
      if cache is None:
        self.path.unlink(missing_ok=True)
      else:
        self.writer(self.path, dict(cache, bridgeCacheToken=token))
      self.key, self.token = key, token
    return self.token


def age_model_feedback(value, elapsed_ms):
  """Conservatively charge the complete return exchange; no source re-dating."""
  component = value.get('modelNavigation')
  if not bounded(elapsed_ms, 0, 2000):
    value['modelNavigation'] = None
    return value
  if isinstance(component, dict):
    for item in (component, component.get('position')):
      if isinstance(item, dict) and bounded(item.get('ageMs'), 0, 2000):
        item['ageMs'] += elapsed_ms
    if bounded(component.get('validForMs'), 0, 2000):
      component['validForMs'] = max(0, component['validForMs'] - math.ceil(elapsed_ms))
  return value
