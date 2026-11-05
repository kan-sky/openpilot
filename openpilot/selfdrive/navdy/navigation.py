"""Bounded Navdy feedback, separate from vehicle control and USB ownership."""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
import threading
import time

from openpilot.selfdrive.carrot.carrot_navi_control import (
  CarrotNaviControl, NaviGuidanceControl, NaviRouteControl, NaviSpeedControl,
  NaviTrafficControl, NaviVehicleControl,
)
from openpilot.selfdrive.navdy.model_navigation import ModelNavigationTracker
from openpilot.selfdrive.navdy.feedback import is_no_observation
from openpilot.selfdrive.navdy.section import SectionTracker

FEEDBACK_PATH = Path('/dev/shm/navdy-feedback.json')
STATUS_PATH = Path('/dev/shm/navdy-status.json')
MAX_FRAME_BYTES = 32768
LINK_TTL = 2.0
SOURCE_TTL = 4.0


def number(value, minimum, maximum):
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return None
  return value if math.isfinite(value) and minimum <= value <= maximum else None


def camera_distance(value):
  if not isinstance(value, str):
    return None
  match = re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*(m|km)\s*', value.lower())
  if match is None:
    return None
  distance = float(match[1]) * (1000 if match[2] == 'km' else 1)
  return int(distance) if 0 < distance <= 10000 else None


def route_traffic(guidance):
  total = guidance.get('trafficTotalMeters')
  items = guidance.get('trafficSegments')
  if type(total) is not int or not 1 <= total <= 10000000 or not isinstance(items, list) or not 1 <= len(items) <= 64:
    return 0, ()
  segments = []
  previous = 0
  for item in items:
    if not isinstance(item, dict):
      return 0, ()
    end, status = item.get('endM'), item.get('status')
    if type(end) is not int or not previous < end <= total or status not in ('G', 'O', 'R'):
      return 0, ()
    segments.append((end, status))
    previous = end
  return total, tuple(segments)


class FeedbackTracker:
  def __init__(self):
    self.received_at = -math.inf
    self.components = {}
    self.sequences = {}
    self.revision = 0
    self.revisions = {'road': 0, 'guidance': 0, 'camera': 0}
    self.camera = None
    self.extended = False
    self.speed_signature = None
    self.speed_sequence = 0
    self.section = SectionTracker()

  def _component(self, name, value, received_at):
    if value is None:
      if self.components.pop(name, None) is not None:
        self.revision += 1
        self.revisions[name] += 1
      return
    if not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] != 1:
      return
    session, sequence = value.get('session'), value.get('sequence')
    age = number(value.get('ageMs'), 0, SOURCE_TTL * 1000)
    if (not isinstance(session, str) or not 1 <= len(session) <= 64
        or type(sequence) is not int or not 0 <= sequence <= 2**53 or age is None):
      return
    valid_for = value.get('validForMs')
    if valid_for is not None and (type(valid_for) is not int or not 0 <= valid_for <= SOURCE_TTL * 1000):
      return
    deadline = received_at + valid_for / 1000 if valid_for is not None else received_at - age / 1000 + SOURCE_TTL
    previous = self.sequences.get(name)
    if previous is not None and previous[0] == session and sequence <= previous[1]:
      if sequence == previous[1] and name in self.components:
        old_value, captured, old_deadline = self.components[name]
        self.components[name] = old_value, captured, min(old_deadline, deadline)
      return
    self.sequences[name] = (session, sequence)
    self.components[name] = (value, received_at - age / 1000, deadline)
    self.revision += 1
    self.revisions[name] += 1

  def accept(self, packet, received_at, now):
    if (not isinstance(packet, dict) or is_no_observation(packet) or number(received_at, 0, now) is None
        or now - received_at > LINK_TTL or received_at <= self.received_at):
      return False
    if received_at - self.received_at >= LINK_TTL:
      self.section.clear()
    self.received_at = received_at
    self.extended = 'navigationFeedback' in packet
    self._component('road', packet.get('roadPreview'), received_at)
    old_session = self.sequences.get('guidance', ('',))[0]
    self._component('guidance', packet.get('navigationFeedback'), received_at)
    if self.sequences.get('guidance', ('',))[0] != old_session:
      self.section.clear()
      self.section.supported = False
    raw_guidance, captured, deadline = self.components.get('guidance', (None, received_at, received_at))
    self.section.accept(raw_guidance.get('guidance') if raw_guidance else None,
                        received_at - captured, deadline, received_at)
    speed = packet.get('cameraSpeedKph')
    distance = camera_distance(packet.get('cameraDistance'))
    kind_value = packet.get('cameraType')
    kind = {'fixed': 1, 'mobile': 7, 'section': 4}.get(kind_value) if isinstance(kind_value, str) else None
    camera = None
    if (packet.get('cameraSource') == 'trafficNotification' and kind is not None
        and type(speed) is int and 20 <= speed <= 140 and distance is not None):
      camera = (kind, speed, distance)
    if self.camera != camera:
      self.camera = camera
      self.revision += 1
      self.revisions['camera'] += 1
    return True

  def _fresh(self, name, now):
    value, captured_at, deadline = self.components.get(name, (None, -math.inf, -math.inf))
    if 0 <= now - captured_at < SOURCE_TTL and now < deadline:
      return value, now - captured_at
    if self.components.pop(name, None) is not None:
      self.revision += 1
      self.revisions[name] += 1
    return None, 0

  def snapshot(self, now):
    if not 0 <= now - self.received_at < LINK_TTL:
      self.section.clear()
      return None
    road, _ = self._fresh('road', now)
    raw_guidance, age = self._fresh('guidance', now)
    guidance = raw_guidance.get('guidance') if raw_guidance else None
    if not isinstance(guidance, dict) or guidance.get('active') is not True:
      guidance = {}
    road_limit = road.get('roadLimitKph') if road else None
    if type(road_limit) is not int or not 10 <= road_limit <= 200 or road_limit % 10:
      road_limit = None
    camera = self.camera
    section = self.section.snapshot(now)
    if not guidance:
      self.section.clear()
      section = None
    # An upgraded source owns section validity, including its explicit clear.
    if self.section.supported and camera is not None and camera[0] == 4:
      camera = None
    revision = self.revisions['guidance']
    sdi = guidance.get('sdi')
    bump_distance = None
    bump_selected = isinstance(sdi, dict) and type(sdi.get('type')) is int and sdi['type'] == 22
    if bump_selected:
      bump_distance = number(sdi.get('distanceMeters'), 1, 10000)
    # TMAP keeps a separate bump when another sign owns the single SDI slot.
    # Both independent components must still be fresh and guidance active.
    if not bump_selected and guidance and road and road.get('guidance') is True:
      bump_distance = number(road.get('bumpDistanceM'), 1, 2000)
    # A route/road heartbeat must not rewind Carrot's ego-motion distance estimate.
    signature = (road_limit, camera, bump_distance, section)
    if signature != self.speed_signature:
      self.speed_signature = signature
      self.speed_sequence += 1

    def turn(prefix):
      kind = guidance.get(prefix + 'Type')
      distance = number(guidance.get(prefix + 'DistanceMeters'), 0, 100000)
      present = type(kind) is int and 1 <= kind <= 2000 and distance is not None
      title = guidance.get('roadName', '') if prefix == 'primary' else ''
      return NaviGuidanceControl(present, revision, int(distance or 0), kind if present else -1,
                                 title[:256] if isinstance(title, str) else '',
                                 received_mono_time_ns=max(0, int((now - age) * 1e9)) if present else 0)

    signal = guidance.get('trafficSignal')
    traffic = NaviTrafficControl(False, revision)
    if isinstance(signal, dict) and signal.get('visible') is True and raw_guidance.get('validForMs') is not None:
      remaining = number(signal.get('remainingSeconds'), 0, 600)
      # The HUD exports its shortest signal/source deadline. Polling cannot
      # extend it; an older APK without that bound cannot supply a signal.
      signal_age = max(age, 2.5 - (self.components['guidance'][2] - now))
      remaining = max(0, int(remaining - signal_age)) if remaining is not None else 0
      lamps = [name for name in ('red', 'left', 'green') if signal.get(name) is True]
      lamp = lamps[0] if lamps else ''
      if lamp and remaining > 0:
        traffic = NaviTrafficControl(True, revision, True, int(number(guidance.get('primaryDistanceMeters'), 0, 100000) or 0),
                                     'navdy', lamp, remaining)
    speed = NaviSpeedControl(road_limit is not None or camera is not None or bump_distance is not None or section is not None,
                            self.speed_sequence, road_limit,
                            sdi_present=camera is not None, sdi_type=camera[0] if camera else -1,
                            sdi_speed_limit_kph=camera[1] if camera else 0, sdi_distance_m=camera[2] if camera else 0,
                            secondary_sdi_present=bump_distance is not None, secondary_sdi_type=22 if bump_distance is not None else -1,
                            secondary_sdi_distance_m=int(bump_distance or 0),
                            section_active=section is not None, section_session_id=section[0] if section else '',
                            section_speed_limit_kph=section[1] if section else 0,
                            section_remaining_distance_m=section[2] if section else 0)
    current, upcoming = turn('primary'), turn('secondary')
    traffic_total, traffic_segments = route_traffic(guidance)
    route = NaviRouteControl(bool(guidance), revision,
                             int(number(guidance.get('remainingDistanceMeters'), 0, 10000000) or 0),
                             int(number(guidance.get('remainingTimeSeconds'), 0, 604800) or 0),
                             traffic_total_m=traffic_total, traffic_segments=traffic_segments)
    if not (speed.present or current.present or upcoming.present or route.present or traffic.present):
      return None
    # Preserve producer identity after component expiry, but reset Carrot's
    # distance history when either producer actually restarts.
    session = 'navdy:' + json.dumps([self.sequences.get(name, ('',))[0] for name in ('road', 'guidance')], separators=(',', ':'))
    return CarrotNaviControl(session, NaviVehicleControl(False, revision), speed, current, upcoming, route, traffic,
                             guidance_active=bool(guidance))


class NavigationReader:
  def __init__(self, path=FEEDBACK_PATH):
    self.path = Path(path)
    self.tracker = FeedbackTracker()
    self.model_tracker = ModelNavigationTracker()
    self.model_route_path = self.path.with_name(self.path.stem + '-model-route.json')
    self.model_route_stat = None
    self.model_route_token = None
    self.last_poll = -math.inf
    self.last_stat = None
    self.lock = threading.Lock()

  def read(self, now=None):
    with self.lock:
      return self._read(now)

  def _read(self, now):
    realtime = now is None
    now = time.monotonic() if realtime else now
    # A crossed deadline gets one last read even if another check already cleared it.
    model_expiring = self.last_poll < self.model_tracker.deadline <= now
    if now - self.last_poll >= 0.1 or model_expiring:
      self.last_poll = now
      try:
        info = self.path.stat()
        stamp = (info.st_mtime_ns, info.st_ino)
        if stamp != self.last_stat:
          self.last_stat = stamp
          with self.path.open('rb') as stream:
            data = stream.read(MAX_FRAME_BYTES + 1025)
          if len(data) <= MAX_FRAME_BYTES + 1024:
            value = json.loads(data)
            now = time.monotonic() if realtime else now
            packet = value.get('feedback')
            if not is_no_observation(packet):
              self.tracker.accept(packet, value.get('receivedAt'), now)
              token = value.get('modelRouteCacheToken')
              token = token if isinstance(token, str) and re.fullmatch(r'[0-9a-f]{32}', token) else None
              if token != self.model_route_token:
                self.model_tracker.clear_geometry()
                self.model_route_stat = None
              self.model_route_token = token
              self.model_tracker.accept(packet.get('modelNavigation') if isinstance(packet, dict) else None,
                                        value.get('receivedAt'), now)
              if self.model_tracker.current is None:
                self._invalidate_model(self.model_tracker.reason)
          else:
            self._invalidate_model('model_feedback_oversized')
      except (OSError, ValueError, TypeError, AttributeError):
        self._invalidate_model('model_feedback_missing')
    self.model_tracker._fresh(time.monotonic() if realtime else now)
    return self.tracker.snapshot(now)

  def _invalidate_model(self, reason):
    self.model_tracker.clear(reason)
    self.model_route_stat = None

  def model_read(self, now=None):
    realtime = now is None
    with self.lock:
      self._read(now)
      now = time.monotonic() if realtime else now
      if not self.model_tracker._fresh(now):
        return None
      if self.model_route_token is None:
        self.model_tracker.clear_geometry()
        self.model_tracker.reason = 'model_route_unverified'
        return None
      try:
        info = self.model_route_path.stat()
        stamp = info.st_ino, info.st_mtime_ns, self.model_tracker.current, self.model_route_token
        if stamp != self.model_route_stat or not self.model_tracker.points:
          with self.model_route_path.open('rb') as stream:
            data = stream.read(140001)
          cache = json.loads(data) if len(data) <= 140000 else None
          self.model_tracker.clear_geometry()
          if not isinstance(cache, dict) or cache.get('bridgeCacheToken') != self.model_route_token:
            self.model_tracker.reason = 'model_route_cache_mismatch'
            return None
          self.model_tracker.import_route(cache)
          self.model_route_stat = stamp
      except FileNotFoundError:
        # No sidecar is expected while the bridge is still assembling chunks.
        if self.model_tracker.points:
          self._invalidate_model('model_route_cache_missing')
      except (OSError, ValueError, TypeError, AttributeError):
        self._invalidate_model('model_route_cache_missing')
      return self.model_tracker.snapshot(time.monotonic() if realtime else now)

  def status(self):
    with self.lock:
      state = self._read(None)
      now = time.monotonic()
      connected = 0 <= now - self.tracker.received_at < LINK_TTL
      return {'connected': connected, 'extendedFeedback': self.tracker.extended,
              'modelRouteReady': bool(self.model_tracker.points),
              'modelNavigationReason': self.model_tracker.reason,
              'navigationFresh': state is not None,
              'reason': ('navdy_disconnected' if not connected else 'navdy_apk_feedback_update_required'
                         if not self.tracker.extended else 'navdy_navigation_stale' if state is None else '')}
