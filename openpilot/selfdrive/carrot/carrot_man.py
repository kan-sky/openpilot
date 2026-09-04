import asyncio
import errno
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
import traceback
from datetime import datetime
from typing import Any, Dict, List, Optional

import ipaddress
import numpy as np
import psutil
import zmq
from aiohttp import web

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.common.hardware import PC
from openpilot.selfdrive.navd.helpers import Coordinate

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ
from openpilot.selfdrive.carrot.carrot_navi_control import CarrotNaviControl, parse_carrot_navi_control
from openpilot.common.gps import get_gps_location_service

try:
  from shapely.geometry import LineString
  SHAPELY_AVAILABLE = True
except ImportError:
  SHAPELY_AVAILABLE = False

NetworkType = log.DeviceState.NetworkType
NAVI_HTTP_PORT = 7713
NAVI_HTTP_MAX_BODY_SIZE = 16 * 1024 * 1024
NAVI_EVENT_TYPES = ("complexCrossroad", "rgdata", "vrtx", "ssinf", "sinf", "route")
NAVI_DEBUG_PARAM = "CarrotNaviDebug"
NAVI_IMAGE_PARAM = "CarrotNaviImage"
NAVI_IMAGE_BASE64_MAX_CHARS = 6 * 1024 * 1024
NAVI_ROUTE_MAX_POINTS = 4096
NAVI_ROUTE_SUMMARY_MAX_SCAN = 20000
BROADCAST_INTERVAL = 1.0
BROADCAST_REMOTE_INTERVAL = 0.2
BROADCAST_NETWORK_ERROR_RETRY_INTERVAL = 5.0
BROADCAST_NETWORK_ERROR_LOG_INTERVAL = 30.0


def limit_route_points(points, max_points=NAVI_ROUTE_MAX_POINTS):
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


_carrot_exception_tmux_send_lock = threading.Lock()
_carrot_exception_tmux_send_queued = False


def reset_carrot_exception_tmux_send_queue() -> None:
  global _carrot_exception_tmux_send_queued
  with _carrot_exception_tmux_send_lock:
    _carrot_exception_tmux_send_queued = False


def queue_carrot_exception_tmux_send(context: str = "", reason: str = "tmux_send") -> bool:
  global _carrot_exception_tmux_send_queued
  with _carrot_exception_tmux_send_lock:
    try:
      params = Params()
      current = params.get("CarrotException")
      if current in (None, "", b""):
        put_nonblocking = getattr(params, "put_nonblocking", None)
        if callable(put_nonblocking):
          put_nonblocking("CarrotException", reason)
        else:
          params.put("CarrotException", reason)
        _carrot_exception_tmux_send_queued = True
        print(f"[carrot_man] CarrotException {reason} queued: {context or 'exception'}")
        return True
      elif current == reason:
        _carrot_exception_tmux_send_queued = True
        return True
      return False
    except Exception as e:
      print(f"[carrot_man] failed to queue CarrotException {reason}: {e}")
      return False


def carrot_can_error_send_ready(detected_at: float | None, now: float, is_onroad: bool) -> bool:
  return is_onroad and detected_at is not None and now - detected_at >= 5.0


def carrot_can_error_sources(car_name: str | bytes | None, car_state_current: bool, car_state,
                             radar_state_current: bool, radar_state) -> tuple[bool, bool]:
  if isinstance(car_name, bytes):
    car_name = car_name.decode("utf-8", errors="ignore")
  if not car_name or car_name.strip().upper() == "MOCK":
    return False, False
  car_can_error = car_state_current and (car_state.canTimeout or not car_state.canValid)
  radar_can_error = radar_state_current and radar_state.radarErrors.canError
  return car_can_error, radar_can_error


def carrot_can_error(car_name: str | bytes | None, car_state_current: bool, car_state,
                     radar_state_current: bool, radar_state) -> bool:
  return any(carrot_can_error_sources(car_name, car_state_current, car_state, radar_state_current, radar_state))

################ CarrotNavi
## 국가법령정보센터: 도로설계기준
#V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 15, 5]

# Haversine formula to calculate distance between two GPS coordinates
def haversine(lon1, lat1, lon2, lat2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# Get the closest point on a segment between two coordinates
def closest_point_on_segment(p1, p2, current_position):
    x1, y1 = p1
    x2, y2 = p2
    px, py = current_position
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return p1
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    return (x1 + t * dx, y1 + t * dy)


# Get path after a certain distance from the current position
def get_path_after_distance(start_index, coordinates, current_position, distance_m):
    total_distance = 0
    path_after_distance = []
    closest_index = -1
    closest_point = None
    min_distance = float('inf')
    start_index = max(0, start_index - 2)

    for i in range(start_index, len(coordinates) - 1):
        p1 = coordinates[i]
        p2 = coordinates[i + 1]
        candidate_point = closest_point_on_segment(p1, p2, current_position)
        distance = haversine(current_position[0], current_position[1], candidate_point[0], candidate_point[1])
        if distance < min_distance:
            min_distance = distance
            closest_point = candidate_point
            closest_index = i
        elif distance > min_distance and min_distance < 10:
            break

    start_index = closest_index
    if closest_index != -1:
        path_after_distance.append(closest_point)
        path_after_distance.append(coordinates[closest_index + 1])
        total_distance = haversine(closest_point[0], closest_point[1], coordinates[closest_index + 1][0], coordinates[closest_index + 1][1])
        for i in range(closest_index + 1, len(coordinates) - 1):
            coord1 = coordinates[i]
            coord2 = coordinates[i + 1]
            segment_distance = haversine(coord1[0], coord1[1], coord2[0], coord2[1])
            if total_distance + segment_distance >= distance_m and segment_distance > 0:
                remaining_distance = distance_m - total_distance
                ratio = remaining_distance / segment_distance
                path_after_distance.append((coord1[0] + ratio * (coord2[0] - coord1[0]), coord1[1] + ratio * (coord2[1] - coord1[1])))
                break
            total_distance += segment_distance
            path_after_distance.append(coord2)

    return path_after_distance, start_index, closest_point


def calculate_angle(point1, point2):
    delta_lon = point2[0] - point1[0]
    delta_lat = point2[1] - point1[1]
    return math.degrees(math.atan2(delta_lat, delta_lon))

# Convert GPS coordinates to relative x, y coordinates based on a reference point and heading
def gps_to_relative_xy(gps_path, reference_point, heading_deg):
    ref_lon, ref_lat = reference_point
    relative_coordinates = []
    heading_rad = math.radians(heading_deg)
    for lon, lat in gps_path:
        x = (lon - ref_lon) * 40008000 * math.cos(math.radians(ref_lat)) / 360
        y = (lat - ref_lat) * 40008000 / 360
        x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
        y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)
        relative_coordinates.append((y_rot, x_rot))
    return relative_coordinates


# Calculate curvature given three points using a faster vector-based method
def calculate_curvature(p1, p2, p3):
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
    if len_v1 * len_v2 == 0:
        return 0
    return cross_product / (len_v1 * len_v2 * len_v1)


class CarrotMan:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.gps_location_service = get_gps_location_service(self.params)

    self.sm = messaging.SubMaster([
      "deviceState", "carState", "controlsState", "radarState", "longitudinalPlan", "modelV2",
      "selfdriveState", "carControl", "navRouteNavd", self.gps_location_service, "navInstruction", "carrotNavi",
    ])

    self.pm = messaging.PubMaster(["carrotMan", "navRoute", "navInstructionCarrot"])
    self.carrot_serv = CarrotServ()

    # devel CarrotMan network state. Keep it for compatibility with CarrotMan/legacy clients.
    # Current TMAP discovery/WebSocket remains owned by standalone carrot_navi.
    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 7705
    self.carrot_man_port = 7706
    self.carrot_navi_http_port = NAVI_HTTP_PORT
    self.connection = None
    self.ip_address = "0.0.0.0"
    self.remote_addr = None
    self.is_running = True
    self.is_metric = self.params.get_bool("IsMetric")

    self.turn_speed_last = 250
    self.curvatureFilter = MyMovingAverage(20)

    self.navi_points = []
    self.navi_points_start_index = 0
    self.navi_points_active = False
    self.navd_active = False

    self.carrot_navi_route_session_id = ""
    self.carrot_navi_route_sequence = -1
    self.carrot_navi_route_owned = False

    self.active_carrot_last = False

    self._rgdata_ts_lock = threading.Lock()
    self._last_rgdata_timestamp_ms = 0
    self._navi_event_lock = threading.Lock()
    self._last_navi_event: Optional[Dict[str, Any]] = None
    self._last_navi_event_by_type: Dict[str, Dict[str, Any]] = {}
    self._last_complex_crossroad: Dict[str, Any] = {}

    # tizi: legacy UDP/TCP/ZMQ threads are preserved below but not auto-started here.
    self.autoCurveSpeedFactor = 1.0
    self.autoCurveSpeedLowerLimit = 30

    self.carrot_curve_speed_params()

    # devel background services. tizi keeps TMAP protocol-v2 in the separate carrot_navi process,
    # while these legacy/CarrotMan services use different ports and remain compatible.
    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[], daemon=True)
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[], daemon=True)
    self.carrot_panda_debug_thread.start()

    self.carrot_route_thread = threading.Thread(target=self.carrot_route, args=[], daemon=True)
    self.carrot_route_thread.start()

    threading.Thread(target=self.broadcast_version_info, daemon=True).start()

  def get_broadcast_address(self):
    try:
      local_ip = self.get_local_ip()
      ipv4_addrs = [addr for addresses in psutil.net_if_addrs().values() for addr in addresses
                    if addr.family == socket.AF_INET and not addr.address.startswith("127.")]
      ipv4_addrs.sort(key=lambda addr: addr.address != local_ip)
      for addr in ipv4_addrs:
        if addr.broadcast:
          return addr.broadcast
        if addr.netmask:
          return str(ipaddress.ip_network(f"{addr.address}/{addr.netmask}", strict=False).broadcast_address)
    except Exception:
      pass
    return "255.255.255.255"

  def get_local_ip(self):
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
      return None

  def broadcast_version_info(self):
    # tizi-safe variant of devel broadcast: only advertise current state.
    # Do not call self.sm.update() here; the main run loop owns the SubMaster.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
      while self.is_running:
        try:
          self.broadcast_ip = self.get_broadcast_address() if self.remote_addr is None else self.remote_addr[0]
          self.ip_address = self.get_local_ip() or "0.0.0.0"
          self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)
          sock.sendto(self.make_send_message().encode("utf-8"), (self.broadcast_ip, self.broadcast_port))
        except OSError:
          self.connection = None
          self.remote_addr = None
          self.ip_address = "0.0.0.0"
        except Exception as e:
          print(f"[carrot_man] broadcast error: {e}")
        time.sleep(1.0)

  def _update_carrot_navi_route(self, navi: CarrotNaviControl | None, force: bool = False):
    previous_session = self.carrot_navi_route_session_id
    route_owned = self.carrot_navi_route_owned

    if navi is None:
      if not previous_session:
        return

      self.carrot_navi_route_session_id = ""
      self.carrot_navi_route_sequence = -1

      if force:
        self.carrot_navi_route_owned = False
        return
      if not route_owned:
        return

      points = ()
    else:
      new_session = navi.session_id != previous_session
      route = navi.route

      if not force and not new_session and route.sequence == self.carrot_navi_route_sequence:
        route_available = route.present and bool(route.polyline)
        if not route_available or self.navi_points_active or self.params.get_bool("IsOffroad"):
          return

      self.carrot_navi_route_session_id = navi.session_id
      self.carrot_navi_route_sequence = route.sequence
      points = route.polyline if route.present else ()

      if not points and force:
        self.carrot_navi_route_owned = False
        return
      if not points and not route_owned:
        return

    coords = [{"latitude": latitude, "longitude": longitude} for latitude, longitude in points]
    self.navi_points = [(point["longitude"], point["latitude"]) for point in coords]
    self.navi_points_start_index = 0
    self.navi_points_active = bool(self.navi_points)
    self.carrot_navi_route_owned = self.navi_points_active
    self.navd_active = self.navi_points_active

    self.send_routes(coords)

  def carrot_navi_route(self):

    if self.carrot_serv.active_carrot > 1:
      if False and self.navd_active:  # mabox always active
        self.navd_active = False
        self.params.remove("NavDestination")
    is_onroad = not self.params.get_bool("IsOffroad")

    if not is_onroad or not self.navi_points_active or not SHAPELY_AVAILABLE or (self.carrot_serv.active_carrot <= 1 and not self.navd_active):
      if self.navi_points_active:
        print("navi_points_active: ", self.navi_points_active, "active_carrot: ", self.carrot_serv.active_carrot, "navd_active: ", self.navd_active)
        self.navi_points = []
        self.navi_points_active = False
        if self.active_carrot_last > 1:
          pass
      self.active_carrot_last = self.carrot_serv.active_carrot
      return [],[],300

    current_position = (self.carrot_serv.vpPosPointLon, self.carrot_serv.vpPosPointLat)
    heading_deg = self.carrot_serv.bearing

    if current_position[0] == 0.0 and current_position[1] == 0.0:
      return [], [], 300.0

    distance_interval = 10.0
    out_speed = 300.0

    path, self.navi_points_start_index, start_point = get_path_after_distance(
      self.navi_points_start_index, self.navi_points, current_position, 300.0
    )

    if not path or start_point is None:
      return [], [], out_speed

    relative_coords = gps_to_relative_xy(path, start_point, heading_deg)
    if len(relative_coords) < 2:
      return [], [], out_speed

    line = LineString(relative_coords)
    resampled_points = []
    resampled_distances = []
    current_distance = 0.0

    while current_distance <= line.length:
      point = line.interpolate(current_distance)
      resampled_points.append((point.x, point.y))
      resampled_distances.append(current_distance)
      current_distance += distance_interval

    sample = 4

    if len(resampled_points) < sample * 2 + 1:
      return resampled_points, resampled_distances, out_speed

    speeds = []
    distances = []
    distance = 10.0

    for i in range(len(resampled_points) - sample * 2):
      distance += distance_interval

      p1 = resampled_points[i]
      p2 = resampled_points[i + sample]
      p3 = resampled_points[i + sample * 2]

      curvature = calculate_curvature(p1, p2, p3)
      speed = float(np.interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS))

      if abs(curvature) < 0.02:
        speed = max(speed, self.carrot_serv.nRoadLimitSpeed)

      speeds.append(speed)
      distances.append(distance)

    if not speeds:
      return resampled_points, resampled_distances, out_speed

    accel_limit = max(0.1, self.carrot_serv.autoNaviSpeedDecelRate)
    accel_limit_kmh = accel_limit * 3.6
    out_speeds = [0.0 for _ in speeds]
    out_speeds[-1] = speeds[-1]

    v_ego_kph = self.sm["carState"].vEgo * 3.6
    time_delay = self.carrot_serv.autoNaviSpeedCtrlEnd
    time_wait = 0.0

    for i in range(len(speeds) - 2, -1, -1):
      target_speed = speeds[i]
      next_out_speed = out_speeds[i + 1]

      if target_speed < next_out_speed:
        time_delay = max(0.0, (v_ego_kph - target_speed) / accel_limit_kmh)
        time_wait = -time_delay

      time_interval = distance_interval / (next_out_speed / 3.6) if next_out_speed > 0.0 else 0.0
      time_apply = min(time_interval, max(0.0, time_interval + time_wait))
      max_allowed_speed = next_out_speed + accel_limit_kmh * time_apply

      out_speeds[i] = min(target_speed, max_allowed_speed)
      time_wait += min(2.0, time_interval)

    out_speed = out_speeds[0]
    return resampled_points, resampled_distances, out_speed

  def make_send_message(self):
    is_onroad = not self.params.get_bool("IsOffroad")
    msg = {"Carrot2": self.params.get("Version") or "", "IsOnroad": is_onroad, "CarrotRouteActive": self.navi_points_active,
           "ip": self.ip_address, "port": self.carrot_man_port, "navi_debug": 0, "navi_http_port": self.carrot_navi_http_port}
    controls_active = False
    x_state = traffic_state = 0
    v_ego_kph = v_cruise_kph = car_cruise_speed = 0
    log_carrot = ""

    if is_onroad:
      if self.sm.alive["carState"]:
        cs = self.sm["carState"]
        v_ego_kph = int(getattr(cs, "vEgoCluster", cs.vEgo) * 3.6 + 0.5)
        log_carrot = getattr(cs, "logCarrot", "")
        v_cruise_kph = cs.vCruise
        car_cruise_speed = cs.cruiseState.speed * 3.6
      if self.sm.alive["selfdriveState"]:
        controls_active = self.sm["selfdriveState"].active
      if self.sm.alive["longitudinalPlan"]:
        lp = self.sm["longitudinalPlan"]
        x_state = getattr(lp, "xState", 0)
        traffic_state = getattr(lp, "trafficState", 0)

    msg.update({"log_carrot": log_carrot, "v_cruise_kph": v_cruise_kph, "carcruiseSpeed": car_cruise_speed,
                "v_ego_kph": v_ego_kph, "tbt_dist": self.carrot_serv.xDistToTurn, "sdi_dist": self.carrot_serv.xSpdDist,
                "active": controls_active, "xState": x_state, "trafficState": traffic_state})
    return json.dumps(msg)

  def receive_fixed_length_data(self, sock, length):
    buffer = b""
    while len(buffer) < length:
      data = sock.recv(length - len(buffer))
      if not data:
        raise ConnectionError("Connection closed before receiving all data")
      buffer += data
    return buffer

  def carrot_man_thread(self):
    # Legacy JSON UDP input (7706): nRoadLimitSpeed/nSdiType/nSdiDist/nTBTDist/nTBTTurnType
    # from a nav-bridge app (e.g. KakaoNavi bridges) -> carrot_serv.update(). Started as a
    # daemon thread in main(); harmless no-op for users who don't run a bridge app.
    while self.is_running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)
          sock.bind(("0.0.0.0", self.carrot_man_port))
          while self.is_running:
            try:
              data, remote_addr = sock.recvfrom(4096)
              if not data:
                continue
              self.remote_addr = remote_addr
              obj = json.loads(data.decode("utf-8"))
              self.carrot_serv.update(obj)
            except TimeoutError:
              self.remote_addr = None
            except Exception as e:
              print(f"[carrot_man] UDP recv error: {e}")
              self.remote_addr = None
              break
      except Exception as e:
        print(f"[carrot_man] UDP server error: {e}")
        time.sleep(2)

  def parse_kisa_data(self, data: bytes):
    result = {}
    try:
      decoded = data.decode("utf-8")
    except UnicodeDecodeError:
      return result
    for part in decoded.split("/"):
      if ":" not in part:
        continue
      key, value = part.split(":", 1)
      try:
        result[key] = int(value)
      except ValueError:
        result[key] = value
    return result

  def kisa_app_thread(self):
    # Legacy KISA/Waze UDP input. Disabled by default.
    while self.is_running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
          sock.settimeout(10)
          sock.bind(("", 12345))
          while self.is_running:
            try:
              data, _ = sock.recvfrom(4096)
              if data:
                self.carrot_serv.update_kisa(self.parse_kisa_data(data))
            except TimeoutError:
              continue
            except Exception as e:
              print(f"[carrot_man] kisa recv error: {e}")
              break
      except Exception as e:
        print(f"[carrot_man] kisa server error: {e}")
        time.sleep(2)

  def carrot_panda_debug(self):
    while self.is_running:
      if self.show_panda_debug:
        self.show_panda_debug = False
        try:
          subprocess.run("/data/openpilot/openpilot/selfdrive/debug/debug_console_carrot.py", shell=True)
        except Exception as e:
          print(f"debug_console error: {e}")
      time.sleep(1)

  def carrot_cmd_zmq(self):
    # devel-compatible non-web command socket. Web/tmux upload diagnostics are intentionally omitted.
    context = zmq.Context()
    sock = context.socket(zmq.REP)
    sock.bind("tcp://*:7710")
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    print("#########carrot_cmd_zmq: thread started...")
    while self.is_running:
      try:
        socks = dict(poller.poll(100))
        if sock not in socks or socks[sock] != zmq.POLLIN:
          continue
        message = sock.recv(zmq.NOBLOCK)
        json_obj = json.loads(message.decode())
        network_type = self.sm['deviceState'].networkType
        network_connected = network_type != NetworkType.none
        if isinstance(json_obj, dict):
          if json_obj.get("show_panda_debug"):
            self.show_panda_debug = True
          if "CarrotException" in json_obj:
            self.params.put("CarrotException", str(json_obj["CarrotException"]))
        reply = {"ok": True, "networkConnected": network_connected, "ip": self.ip_address}
        sock.send(json.dumps(reply).encode())
      except Exception as e:
        print(f"carrot_cmd_zmq error: {e}")
        try:
          sock.send(json.dumps({"ok": False, "error": str(e)}).encode())
        except Exception:
          pass

  def recvall(sock, n):
    data = bytearray()
    while len(data) < n:
      packet = sock.recv(n - len(data))
      if not packet:
        return None
      data.extend(packet)
    return bytes(data)

  def receive_double(self, sock):
    double_data = self.recvall(sock, 8)
    return struct.unpack('!d', double_data)[0]

  def receive_float(self, sock):
    float_data = self.recvall(sock, 4)
    return struct.unpack('!f', float_data)[0]

  def send_routes(self, coords, from_navd=False):
    coords = limit_route_points(coords, NAVI_ROUTE_MAX_POINTS)

    if from_navd:
      if len(coords) > 0:
        self.navi_points = [(float(c.longitude), float(c.latitude)) for c in coords]
        self.navi_points_start_index = 0
        self.navi_points_active = True
        self.navd_active = True
        out_coords = [{"latitude": float(c.latitude), "longitude": float(c.longitude)} for c in coords]
      else:
        self.navi_points = []
        self.navi_points_start_index = 0
        self.navi_points_active = False
        self.navd_active = False
        out_coords = []
    else:
      out_coords = list(coords)

    msg = messaging.new_message("navRoute", valid=True)
    msg.navRoute.coordinates = out_coords
    self.pm.send("navRoute", msg)

  def carrot_route(self):
    # Legacy route TCP server (7709). Disabled by default; carrotNavi.route is the normal tizi path.
    while self.is_running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
          server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
          server.bind(("0.0.0.0", 7709))
          server.listen()
          conn, _ = server.accept()
          with conn:
            total_size_bytes = self.recvall(conn, 4)
            if not total_size_bytes:
              continue
            total_size = struct.unpack("!I", total_size_bytes)[0]
            all_data = self.recvall(conn, total_size)
            if all_data is None:
              continue

            points = []
            navi_points = []
            for i in range(0, len(all_data) - 7, 8):
              x, y = struct.unpack("!ff", all_data[i:i + 8])
              navi_points.append((x, y))
              coord = Coordinate.from_mapbox_tuple((x, y))
              points.append(coord)

            coords = [c.as_dict() for c in points]
            self.navi_points = navi_points
            self.navi_points_start_index = 0
            self.navi_points_active = bool(coords)
            self.navd_active = bool(coords)
            self.send_routes(coords)

            if coords:
              dest = dict(coords[-1])
              dest["place_name"] = "External Navi"
              self.params.put("NavDestination", json.dumps(dest))
            else:
              self.params.remove("NavDestination")
      except Exception as e:
        print(f"[carrot_man] route server error: {e}")
        time.sleep(2)

  def carrot_curve_speed_params(self):
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor") * 0.01
    self.autoCurveSpeedLowerLimit = self.params.get_int("AutoCurveSpeedLowerLimit")

    if self.autoCurveSpeedFactor <= 0.0:
      self.autoCurveSpeedFactor = 1.0
    if self.autoCurveSpeedLowerLimit <= 0:
      self.autoCurveSpeedLowerLimit = 30

  def carrot_curve_speed(self, sm):
    self.carrot_curve_speed_params()

    if not sm.alive["carState"] or not sm.alive["modelV2"]:
      return 250.0
    if len(sm["modelV2"].orientationRate.z) == 0 or len(sm["modelV2"].velocity.x) == 0:
      return 250.0

    return self.vturn_speed(sm["carState"], sm)

  def vturn_speed(self, CS, sm):
    target_lat_a = 1.9
    model_data = sm["modelV2"]

    # Kans: max_curve divides by v_ego**2 below - near a stop, the old 0.1
    # m/s floor made that denominator tiny (0.01), so ordinary noise in
    # orientation_rate/model velocity got hugely amplified into a wildly
    # swinging turn_speed (observed flickering ~120-200kph while stopped,
    # which flips carrot_serv.py's model-turn-speed candidate in and out of
    # the desiredSpeed min() and made v_cruise itself flicker while stopped -
    # suspected trigger for GM ACC faults while stopped behind a lead).
    # Curve-speed limiting isn't physically meaningful this close to a stop
    # anyway, so bail out to "no limit" instead of computing a noisy value.
    if CS.vEgo < 2.0:
      return 250.0
    v_ego = CS.vEgo

    orientation_rate = np.asarray(model_data.orientationRate.z, dtype=float) * self.autoCurveSpeedFactor
    velocity = np.asarray(model_data.velocity.x, dtype=float)

    if len(orientation_rate) == 0 or len(velocity) == 0:
      return 250.0

    pred_lat_acc = np.abs(orientation_rate) * velocity
    max_index = int(np.argmax(np.abs(orientation_rate)))
    curv_direction = np.sign(orientation_rate[max_index])
    max_pred_lat_acc = float(np.amax(pred_lat_acc))
    max_curve = max_pred_lat_acc / (v_ego ** 2)

    if not np.isfinite(max_curve) or max_curve < 1e-6:
      return 250.0

    turn_speed = abs(target_lat_a / max_curve) ** 0.5 * 3.6
    turn_speed = max(turn_speed, self.autoCurveSpeedLowerLimit)
    turn_speed = min(turn_speed, 250.0)
    return turn_speed * curv_direction

  def carrot_navi_thread(self):
    self.carrot_navi_tcp_server(7712)

  def _route_point_to_lon_lat(self, point: Any):
    if isinstance(point, dict):
      if not point.get("valid", True):
        return None
      lon_value = point.get("x")
      lat_value = point.get("y")
      if lon_value is None:
        lon_value = point.get("lon", point.get("longitude"))
      if lat_value is None:
        lat_value = point.get("lat", point.get("latitude"))
      if lon_value is None or lat_value is None:
        return None
      try:
        lon, lat = float(lon_value), float(lat_value)
      except Exception:
        return None
      if not math.isfinite(lon) or not math.isfinite(lat) or not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
        return None
      return lon, lat
    if isinstance(point, (list, tuple)) and len(point) >= 2:
      try:
        lon, lat = float(point[0]), float(point[1])
      except Exception:
        return None
      if not math.isfinite(lon) or not math.isfinite(lat) or not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
        return None
      return lon, lat
    return None

  def _extract_route_points(self, payload: Any, depth: int = 0):
    if payload is None:
      return []
    if depth > 8:
      return None
    if isinstance(payload, dict):
      for key in ("vrtx", "vertices", "vertexes", "coordinates", "coords", "points", "path", "route"):
        value = payload.get(key)
        if value is not None:
          return self._extract_route_points(value, depth + 1)
      point = self._route_point_to_lon_lat(payload)
      return [point] if point is not None else None
    if not isinstance(payload, list):
      return None
    points = []
    for point in payload:
      lon_lat = self._route_point_to_lon_lat(point)
      if lon_lat is not None:
        points.append(lon_lat)
    return points

  def _limited_route_points(self, points: List[tuple]):
    return limit_route_points(points, NAVI_ROUTE_MAX_POINTS)

  def handle_route(self, payload: Any):
    points = self._extract_route_points(payload)
    if points is None:
      print(f"Received route: unsupported payload type={type(payload).__name__}")
      return
    points = self._limited_route_points(points)
    coords = [{"latitude": lat, "longitude": lon} for lon, lat in points]
    if not coords:
      self.navi_points = []
      self.navi_points_start_index = 0
      self.navi_points_active = False
      self.navd_active = False
      return
    self.navi_points = [(p["longitude"], p["latitude"]) for p in coords]
    self.navi_points_start_index = 0
    self.navi_points_active = True
    self.navd_active = True
    self.send_routes(coords)

  def _put_traffic_light(self, lamp: str, remain: Any, distance: Any = 0, lat: Any = None, lon: Any = None):
    try:
      remain_int = int(float(remain or 0))
    except Exception:
      remain_int = 0

    if remain_int <= 0:
      return

    try:
      distance_int = int(float(distance or 0))
    except Exception:
      distance_int = 0

    traffic_light = {
      "distance": distance_int,
      "lamp": lamp,
      "remain": remain_int,
      "ts": time.monotonic(),
    }

    try:
      if lat is not None:
        traffic_light["lat"] = float(lat)
      if lon is not None:
        traffic_light["lon"] = float(lon)
    except Exception:
      pass

    self.params_memory.put_nonblocking("TrafficLight", json.dumps(traffic_light))

  def handle_traffic_light(self, d: dict):
    if not isinstance(d, dict):
      return

    # {'distance': 120, 'greenLightRemainTime': 0, 'leftLightRemainTime': 0, 'location': {'coordString': 'x:127.045286, y:37.477032', 'latitude': 37.47703188722564, 'longitude': 127.04528634430659},
    #       'redLightRemainTime': 15, 'rightLightRemainTime': 0, 'uturnLightRemainTime': 0, 'greenLightOn': False, 'leftLightOn': False, 'redLightOn': True, 'rightLightOn': False, 'uturnLightOn': False}
    lamp = None
    remain = 0

    if d.get("redLightOn"):
      lamp = "red"
      remain = d.get("redLightRemainTime", 0)
    elif d.get("leftLightOn"):
      lamp = "left"
      remain = d.get("leftLightRemainTime", 0)
    elif d.get("greenLightOn"):
      lamp = "green"
      remain = d.get("greenLightRemainTime", 0)
    elif d.get("rightLightOn"):
      lamp = "right"
      remain = d.get("rightLightRemainTime", 0)
    elif d.get("uturnLightOn"):
      lamp = "uturn"
      remain = d.get("uturnLightRemainTime", 0)

    if lamp is None:
      return

    location = d.get("location", {})
    lat = None
    lon = None
    try:
      if isinstance(location, dict):
        if location.get("latitude") is not None:
          lat = float(location.get("latitude"))
        if location.get("longitude") is not None:
          lon = float(location.get("longitude"))
    except Exception:
      pass
    self._put_traffic_light(lamp, remain, d.get("distance", 0), lat, lon)

  def handle_traffic_light_detail(self, d: dict):
    if not isinstance(d, dict):
      return

    green_checks = (
      ("left", "left", "left_remain_time"),
      ("straight", "green", "straight_remain_time"),
      ("right", "right", "right_remain_time"),
      ("uturn", "uturn", "uturn_remain_time"),
    )
    for field, lamp, remain_field in green_checks:
      if str(d.get(field, "")).upper() == "GREEN_LIGHT_ON":
        self._put_traffic_light(lamp, d.get(remain_field, 0), d.get("distance", 0), d.get("lat"), d.get("lon"))
        return

    red_remain = 0
    for field in ("straight", "left", "right", "uturn"):
      if str(d.get(field, "")).upper() == "RED_LIGHT_ON":
        try:
          red_remain = max(red_remain, int(d.get(f"{field}_remain_time", 0) or 0))
        except Exception:
          pass

    if red_remain > 0:
      self._put_traffic_light("red", red_remain, d.get("distance", 0), d.get("lat"), d.get("lon"))

  def _safe_int_or_none(self, value: Any, minimum: Optional[int] = None, maximum: Optional[int] = None) -> Optional[int]:
    try:
      int_value = int(float(value))
    except Exception:
      return None
    if not math.isfinite(int_value):
      return None
    if minimum is not None and int_value < minimum:
      return None
    if maximum is not None and int_value > maximum:
      return None
    return int_value

  def _safe_float_or_none(self, value: Any, minimum: Optional[float] = None, maximum: Optional[float] = None) -> Optional[float]:
    try:
      float_value = float(value)
    except Exception:
      return None
    if not math.isfinite(float_value):
      return None
    if minimum is not None and float_value < minimum:
      return None
    if maximum is not None and float_value > maximum:
      return None
    return float_value

  def _remaining_time(self, value: Any) -> Optional[int]:
    return self._safe_int_or_none(value, minimum=1, maximum=999)

  def _traffic_light_debug_from_sinf(self, sinf: dict) -> Dict[str, Any]:
    return {
      "distanceM": self._safe_int_or_none(sinf.get("distance"), minimum=0),
      "redS": self._remaining_time(sinf.get("redLightRemainTime")),
      "straightS": self._remaining_time(sinf.get("greenLightRemainTime")),
      "leftS": self._remaining_time(sinf.get("leftLightRemainTime")),
      "rightS": self._remaining_time(sinf.get("rightLightRemainTime")),
      "uturnS": self._remaining_time(sinf.get("uturnLightRemainTime")),
      "redOn": bool(sinf.get("redLightOn")),
      "straightOn": bool(sinf.get("greenLightOn")),
      "leftOn": bool(sinf.get("leftLightOn")),
      "rightOn": bool(sinf.get("rightLightOn")),
      "uturnOn": bool(sinf.get("uturnLightOn")),
    }

  def _traffic_light_debug_from_ssinf(self, ssinf: dict) -> Dict[str, Any]:
    red_remaining = []
    for signal_key, remain_key in (
      ("straight", "straight_remain_time"),
      ("left", "left_remain_time"),
      ("right", "right_remain_time"),
      ("uturn", "uturn_remain_time"),
    ):
      if str(ssinf.get(signal_key, "")).upper() == "RED_LIGHT_ON":
        remaining = self._remaining_time(ssinf.get(remain_key))
        if remaining is not None:
          red_remaining.append(remaining)
    return {
      "distanceM": self._safe_int_or_none(ssinf.get("distance"), minimum=0),
      "redS": max(red_remaining) if red_remaining else None,
      "straightS": self._remaining_time(ssinf.get("straight_remain_time")),
      "leftS": self._remaining_time(ssinf.get("left_remain_time")),
      "rightS": self._remaining_time(ssinf.get("right_remain_time")),
      "uturnS": self._remaining_time(ssinf.get("uturn_remain_time")),
      "redOn": bool(red_remaining),
      "straightOn": str(ssinf.get("straight", "")).upper() == "GREEN_LIGHT_ON",
      "leftOn": str(ssinf.get("left", "")).upper() == "GREEN_LIGHT_ON",
      "rightOn": str(ssinf.get("right", "")).upper() == "GREEN_LIGHT_ON",
      "uturnOn": str(ssinf.get("uturn", "")).upper() == "GREEN_LIGHT_ON",
    }

  def handle_carrot_state(self, d: dict):
    try:
      self.carrot_serv.update(d)
    except Exception as e:
      print("carrot_state update error:", e)

  def handle_unknown(self, obj: Any):
    print("[UNKNOWN]", str(obj)[:200])

  def _detect_navi_event_type(self, obj: Any) -> str:
    if not isinstance(obj, dict):
      return "unknown"

    for key in NAVI_EVENT_TYPES:
      if obj.get(key) is not None:
        return key
    return "unknown"

  def _get_timestamp_ms(self, obj: Any) -> int:
    if not isinstance(obj, dict):
      return 0
    try:
      return int(obj.get("timestamp_ms") or obj.get("timestamp") or 0)
    except Exception:
      return 0

  def _summarize_navi_event(self, event_type: str, obj: Any) -> Dict[str, Any]:
    summary: Dict[str, Any] = {"type": event_type}
    if not isinstance(obj, dict):
      return summary

    if event_type == "rgdata" and isinstance(obj.get("rgdata"), dict):
      rgdata = obj["rgdata"]
      summary.update({
        "lat": rgdata.get("vpPosPointLat"),
        "lon": rgdata.get("vpPosPointLon"),
        "speed": rgdata.get("nPosSpeed"),
        "roadLimitSpeed": rgdata.get("nRoadLimitSpeed"),
        "tbtDist": rgdata.get("nTBTDist"),
        "tbtTurnType": rgdata.get("nTBTTurnType"),
        "sdiType": rgdata.get("nSdiType"),
        "sdiDist": rgdata.get("nSdiDist"),
      })
    elif event_type == "sinf" and isinstance(obj.get("sinf"), dict):
      sinf = obj["sinf"]
      summary.update({
        "distance": sinf.get("distance"),
        "redLightOn": sinf.get("redLightOn"),
        "greenLightOn": sinf.get("greenLightOn"),
        "leftLightOn": sinf.get("leftLightOn"),
      })
    elif event_type == "ssinf" and isinstance(obj.get("ssinf"), dict):
      ssinf = obj["ssinf"]
      summary.update({
        "distance": ssinf.get("distance"),
        "straight": ssinf.get("straight"),
        "left": ssinf.get("left"),
        "straightRemain": ssinf.get("straight_remain_time"),
        "leftRemain": ssinf.get("left_remain_time"),
      })
    else:
      summary["keys"] = list(obj.keys())[:10]
    return summary

  def _navi_debug_line(self, label: str, value: Any) -> str:
    text = "" if value is None else str(value)
    return f"{label}: {text}"[:120]

  def _navi_debug_from_event(self, obj: Any, event_type: str, event_time_ms: int) -> Dict[str, Any]:
    title = f"NAVI {event_type}"
    severity = "normal"
    lines: List[str] = []
    speed_limit_kph: Optional[int] = None
    traffic_light: Optional[Dict[str, Any]] = None

    if isinstance(obj, dict) and event_type == "sinf" and isinstance(obj.get("sinf"), dict):
      sinf = obj["sinf"]
      title = "Traffic light"
      traffic_light = self._traffic_light_debug_from_sinf(sinf)
      if sinf.get("redLightOn"):
        severity = "stop"
      elif sinf.get("leftLightOn") or sinf.get("greenLightOn"):
        severity = "go"
      lines.extend((
        self._navi_debug_line("Distance", f"{sinf.get('distance', '--')}m"),
        self._navi_debug_line("Red", f"{sinf.get('redLightOn')} {sinf.get('redLightRemainTime', '--')}s"),
        self._navi_debug_line("Green", f"{sinf.get('greenLightOn')} {sinf.get('greenLightRemainTime', '--')}s"),
        self._navi_debug_line("Left", f"{sinf.get('leftLightOn')} {sinf.get('leftLightRemainTime', '--')}s"),
      ))

    elif isinstance(obj, dict) and event_type == "ssinf" and isinstance(obj.get("ssinf"), dict):
      ssinf = obj["ssinf"]
      title = "Traffic light detail"
      traffic_light = self._traffic_light_debug_from_ssinf(ssinf)
      red_active = any(str(ssinf.get(key, "")).upper() == "RED_LIGHT_ON" for key in ("straight", "left", "right", "uturn"))
      green_active = any(str(ssinf.get(key, "")).upper() == "GREEN_LIGHT_ON" for key in ("straight", "left", "right", "uturn"))
      severity = "stop" if red_active else "go" if green_active else "normal"
      lines.extend((
        self._navi_debug_line("Distance", f"{ssinf.get('distance', '--')}m"),
        self._navi_debug_line("Straight", f"{ssinf.get('straight', '--')} {ssinf.get('straight_remain_time', '--')}s"),
        self._navi_debug_line("Left", f"{ssinf.get('left', '--')} {ssinf.get('left_remain_time', '--')}s"),
        self._navi_debug_line("Right", f"{ssinf.get('right', '--')} {ssinf.get('right_remain_time', '--')}s"),
      ))
    else:
      keys = list(obj.keys())[:10] if isinstance(obj, dict) else []
      lines.append(self._navi_debug_line("Keys", ", ".join(keys)))

    return {
      "receivedMono": time.monotonic(),
      "eventTimeMs": event_time_ms,
      "type": event_type,
      "title": title,
      "severity": severity,
      "lines": [line for line in lines if line],
      "speedLimitKph": speed_limit_kph,
      "trafficLight": traffic_light,
    }

  def _write_navi_debug_param(self, obj: Any, event_type: str, event_time_ms: int):
    try:
      debug = self._navi_debug_from_event(obj, event_type, event_time_ms)
      self.params_memory.put_nonblocking(NAVI_DEBUG_PARAM, json.dumps(debug, ensure_ascii=False))
    except Exception as e:
      print(f"navi debug param error: {e}")

  def _store_navi_event(self, obj: Any, event_type: str, event_time_ms: int):
    event = {
      "receivedAt": datetime.now().astimezone().isoformat(timespec="milliseconds"),
      "eventTimeMs": event_time_ms,
      "type": event_type,
      "summary": self._summarize_navi_event(event_type, obj),
    }
    with self._navi_event_lock:
      self._last_navi_event = event
      self._last_navi_event_by_type[event_type] = event

  def _normalize_rgdata(self, rgdata: Any):
    if not isinstance(rgdata, dict):
      return rgdata

    merged = dict(rgdata)
    for group_key in ("guidance", "sdi", "lane"):
      group = rgdata.get(group_key)
      if isinstance(group, dict):
        for key, value in group.items():
          merged.setdefault(key, value)
    return merged

  def _is_stale_rgdata(self, timestamp_ms: int):
    if timestamp_ms <= 0:
      return False, 0

    with self._rgdata_ts_lock:
      last_ts = self._last_rgdata_timestamp_ms
      if timestamp_ms <= last_ts:
        return True, last_ts

      self._last_rgdata_timestamp_ms = timestamp_ms
      return False, last_ts

  def _safe_dispatch_handler(self, label: str, handler: Any, *args: Any):
    try:
      return handler(*args)
    except Exception as e:
      print(f"navi {label} handler error: {e}")
      traceback.print_exc()
      queue_carrot_exception_tmux_send(f"navi {label} handler")
      return None

  def _dispatch_obj(self, obj: Any):
    if obj is None:
      return

    if isinstance(obj, str):
      s = obj.strip()
      if not s:
        return
      try:
        obj = json.loads(s)
      except Exception:
        return self.handle_unknown(s[:200])

    if not isinstance(obj, dict):
      return self.handle_unknown(obj)

    event_type = self._detect_navi_event_type(obj)
    event_time_ms = self._get_timestamp_ms(obj)
    try:
      self._store_navi_event(obj, event_type, event_time_ms)
    except Exception as e:
      print(f"navi event store error: {e}")

    handled = False

    if "rgdata" in obj:
      stale, last_ts = self._is_stale_rgdata(event_time_ms)
      if stale:
        print(f"[STALE DROP] rgdata ts={event_time_ms} <= last={last_ts}")
      else:
        self._safe_dispatch_handler("rgdata", self.handle_carrot_state, self._normalize_rgdata(obj["rgdata"]))
      handled = True

    if "vrtx" in obj:
      self._safe_dispatch_handler("vrtx", self.handle_route, obj["vrtx"])
      handled = True

    if "ssinf" in obj:
      print(f"[NAVI ssinf RX] {json.dumps(obj['ssinf'], ensure_ascii=False)}", flush=True)
      self._safe_dispatch_handler("ssinf", self.handle_traffic_light_detail, obj["ssinf"])
      handled = True

    if "sinf" in obj:
      print(f"[NAVI sinf RX] {json.dumps(obj['sinf'], ensure_ascii=False)}", flush=True)
      self._safe_dispatch_handler("sinf", self.handle_traffic_light, obj["sinf"])
      handled = True

    if "route" in obj:
      self._safe_dispatch_handler("route", self.handle_route, obj["route"])
      handled = True

    if handled:
      self._write_navi_debug_param(obj, event_type, event_time_ms)

    if not handled:
      self.handle_unknown({"type": event_type, "keys": list(obj.keys())[:10]})

  def carrot_navi_tcp_server(self, port: int = 7712):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(5)
    print("TCP server listening", port)
    while self.is_running:
      conn, addr = server.accept()
      self.remote_addr = addr
      try:
        f = conn.makefile("r", encoding="utf-8", errors="ignore")
        while True:
          line = f.readline()
          if not line:
            break
          s = line.strip()
          if s:
            self._dispatch_obj(s)
      except Exception as e:
        print("TCP error:", e)
      finally:
        try:
          conn.close()
        except Exception:
          pass
        self.remote_addr = None

  def carrot_navi_http_thread(self):
    # Kans (carrot-wip-0721): the constants (NAVI_HTTP_PORT/NAVI_HTTP_MAX_BODY_SIZE)
    # and _dispatch_obj() this feeds were already present in tz, but these four
    # methods themselves and the thread start below were missing - the HTTP POST
    # path (used by some Android nav-bridge apps instead of the raw TCP-7712
    # socket) was never actually reachable. Nothing else needed porting.
    while True:
      try:
        asyncio.run(self.carrot_navi_http_server(self.carrot_navi_http_port))
      except Exception as e:
        print(f"navi http server error: {e}")
        traceback.print_exc()
        queue_carrot_exception_tmux_send("navi http server")
        time.sleep(2)

  async def carrot_http_post(self, request: web.Request):
    tmap_version = request.match_info.get("tmap_version", "")

    try:
      peer = request.transport.get_extra_info("peername")
    except Exception:
      peer = None

    try:
      raw_body = (await request.text()).strip()
      if not raw_body:
        raise ValueError("empty body")
      obj = json.loads(raw_body)
    except Exception as e:
      print(f"[HTTP] json parse error: {e}")
      return web.json_response({
        "ok": False,
        "error": f"invalid json: {e}"
      }, status=400)

    if isinstance(obj, dict):
      obj["_tmap_version"] = tmap_version
    if isinstance(peer, tuple) and len(peer) >= 1 and peer[0]:
      self.remote_addr = (peer[0], self.broadcast_port)

    try:
      self._dispatch_obj(obj)
      return web.json_response({
        "ok": True,
        "tmap_version": tmap_version
      })
    except Exception as e:
      print(f"[HTTP] dispatch error: {e}")
      traceback.print_exc()
      queue_carrot_exception_tmux_send("navi http dispatch")
      return web.json_response({
        "ok": False,
        "error": str(e),
        "tmap_version": tmap_version
      }, status=500)

  async def carrot_http_health(self, request: web.Request):
    with self._navi_event_lock:
      last_event = self._last_navi_event
      by_type = dict(self._last_navi_event_by_type)

    last_summary = None
    if last_event is not None:
      last_summary = {
        "receivedAt": last_event["receivedAt"],
        "eventTimeMs": last_event["eventTimeMs"],
        "summary": last_event.get("summary", {}),
      }

    return web.json_response({
      "ok": True,
      "service": "carrot_navi_http",
      "lastEvent": last_summary,
      "receivedTypes": sorted(by_type.keys()),
    })

  async def carrot_navi_http_server(self, port: int = NAVI_HTTP_PORT):
    app = web.Application(client_max_size=NAVI_HTTP_MAX_BODY_SIZE)

    app.router.add_post("/api/navi/{tmap_version}", self.carrot_http_post)
    app.router.add_get("/health", self.carrot_http_health)

    runner = web.AppRunner(app, access_log=None)
    await runner.setup()

    site = web.TCPSite(runner, "0.0.0.0", port)
    await site.start()

    print("HTTP server listening", port)

    while True:
      await asyncio.sleep(3600)

  def run(self):
    rk = Ratekeeper(20, print_delay_threshold=None)

    while True:
      self.sm.update(0)

      navd_route_updated = self.sm.updated["navRouteNavd"]

      if navd_route_updated:
        self.send_routes(self.sm["navRouteNavd"].coordinates, from_navd=True)

      carrot_navi_service_active = self.sm.alive["carrotNavi"] and self.sm.valid["carrotNavi"]

      if (self.sm.updated["carrotNavi"] or navd_route_updated or
          (self.carrot_navi_route_session_id and not carrot_navi_service_active)):
        carrot_navi = parse_carrot_navi_control(self.sm["carrotNavi"]) if carrot_navi_service_active else None
        self._update_carrot_navi_route(carrot_navi, force=navd_route_updated)

      vturn_speed = self.carrot_curve_speed(self.sm)
      coords, distances, route_speed = self.carrot_navi_route()

      remote_ip = self.remote_addr[0] if self.remote_addr is not None else ""
      self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed, coords, distances, route_speed, self.gps_location_service)

      rk.keep_time()

def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    print("[carrot_man] failed to set core affinity")

  print("CarrotManager Started")
  carrot_man = CarrotMan()
  print(f"CarrotMan {carrot_man}")

  # tizi: current TMAP protocol-v2 discovery/WebSocket runs in the separate carrot_navi
  # process, but that process is TMAP-specific and has no rgdata/nRoadLimitSpeed handling
  # of its own - it does not overlap with the legacy nav-bridge input paths below, so
  # starting these here isn't a duplicate.
  # Kans: carrot_man_thread (7706 JSON UDP: nRoadLimitSpeed/nSdiType/nSdiDist/nTBTDist/
  # nTBTTurnType) feeds carrot_serv.update(), the same state consumed by the existing
  # camera/nav speed-control engine (xSpdType/xSpdDist/xDistToTurn) - it just was never
  # started here. devel runs it as the blocking main-loop call; tz's main loop is
  # already carrot_man.run(), so start this as a daemon thread like the others instead.
  threading.Thread(target=carrot_man.carrot_man_thread, daemon=True).start()
  threading.Thread(target=carrot_man.kisa_app_thread, daemon=True).start()
  threading.Thread(target=carrot_man.carrot_navi_thread, daemon=True).start()
  threading.Thread(target=carrot_man.carrot_navi_http_thread, daemon=True).start()

  while True:
    try:
      carrot_man.run()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
