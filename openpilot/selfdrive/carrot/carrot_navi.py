#!/usr/bin/env python3
from __future__ import annotations

import base64
import hashlib
import json
import secrets
import socket
import struct
import threading
import time
from typing import Any
from urllib.parse import urlparse


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 7714

DISCOVERY_PORT = 7705
DISCOVERY_INTERVAL_S = 1.0

PROTOCOL_VERSION = 2
CATALOG_REVISION = 1

MAX_MESSAGE_BYTES = 4 * 1024 * 1024

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


# TMAP protocol v2 catalog.
#
# IMPORTANT:
# TMAP requires the complete catalog to be present during negotiation.
# Do not remove image/render entries just because we don't use them.
# Non-web operation is controlled by ENABLED_JSON_STREAMS below.
JSON_NAMES = ("vehicle", "guidance_current", "guidance_next", "lane_current", "lane_ahead", "speed", "traffic_signal",
  "crossroad", "route", "navigation_status", "app_status", "camera_state", "composition_state",)

IMAGE_NAMES = ("tbt_current_compact", "tbt_current_full", "tbt_next", "traffic_signal", "lane_top", "lane_bottom", "safety_primary",
  "safety_secondary", "safety_section", "crossroad_minimized", "crossroad_expanded", "center_tbt_icon", "center_tbt_text", "center_tbt_fee",)

RENDER_NAMES = ("map_main",)


CATALOG = (tuple(("json", name) for name in JSON_NAMES)
  + tuple(("image", name) for name in IMAGE_NAMES) + tuple(("render", name) for name in RENDER_NAMES))

CATALOG_SET = frozenset(CATALOG)
JSON_ARRAY_NAMES = frozenset(("lane_ahead",))


# Kans: non-web Carrot Navi.
#
# Only JSON streams required for driving/navigation control are enabled.
#
# Enabled:
#   vehicle            : GPS / heading / vehicle navigation state
#   guidance_current   : current TBT
#   guidance_next      : next TBT
#   lane_current       : current lane guidance
#   lane_ahead         : upcoming lane guidance
#   speed              : SDI / camera / section control
#   traffic_signal     : traffic-light information
#   crossroad          : crossroad/navigation metadata
#   route              : TMAP route polyline
#   navigation_status  : guidance/off-route state
#
# Disabled:
#   app_status
#   camera_state
#   composition_state
#   all image streams
#   all render streams
#
# The disabled streams remain in CATALOG so TMAP protocol negotiation
# still sees the complete 28-item catalog.
ENABLED_JSON_STREAMS = frozenset(("vehicle", "guidance_current", "guidance_next", "lane_current", "lane_ahead", "speed", "traffic_signal",
  "crossroad", "route", "navigation_status",))


def detect_advertise_ip() -> str:
  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  try:
    # No packet is actually sent here.
    # This is only used to determine the local interface/IP
    # selected by the routing table.
    probe.connect(("8.8.8.8", 80))
    return str(probe.getsockname()[0])

  except OSError:
    try:
      return socket.gethostbyname(socket.gethostname())
    except OSError:
      return "127.0.0.1"

  finally:
    probe.close()


def build_manifest(session_id: str) -> dict[str, Any]:

  streams = []

  for handle, (kind, name) in enumerate(CATALOG, start=1):
    # Kans: non-web mode.
    # Only selected JSON streams are enabled.
    enabled = (kind == "json" and name in ENABLED_JSON_STREAMS)

    streams.append({"kind": kind, "name": name, "schema_version": 1,
      "stream_handle": handle, "enabled": enabled, "params": {"delivery_mode": "on_change",
        "interval_ms": 100, "stale_timeout_ms": 10000} if kind == "json" else {}})

  return {"type": "subscription_manifest", "protocol_version": PROTOCOL_VERSION, "session_id": session_id,
    "revision": CATALOG_REVISION, "metrics_enabled": False, "limit_adjustments": [], "streams": streams}


class CarrotNaviReceiver:
  def __init__(self) -> None:
    self._lock = threading.RLock()
    self._state_changed = threading.Event()

    self._session_id = ""
    self._connected = False

    self._generation = 0

    self._records: dict[str, dict[str, Any]] = {}

    self._manifest_by_key: dict[str, dict[str, Any]] = {}

  def negotiate(self, requirements: dict[str, Any], app_version: str) -> dict[str, Any]:

    if requirements.get("type") != "requirements_query":
      raise ValueError("invalid requirements query")

    if requirements.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("unsupported protocol version")

    if requirements.get("catalog_revision") != CATALOG_REVISION:
      raise ValueError("unsupported catalog revision")

    offered_streams = requirements.get("streams")

    if not isinstance(offered_streams, list) or len(offered_streams) != len(CATALOG):
      raise ValueError(f"TMAP catalog must contain exactly " f"{len(CATALOG)} items")

    offered = []

    for stream in offered_streams:
      if not isinstance(stream, dict) or stream.get("schema_version") != 1:
        raise ValueError("invalid catalog entry")

      offered.append((str(stream.get("kind")), str(stream.get("name"))))

    if len(set(offered)) != len(offered) or set(offered) != CATALOG_SET:
      raise ValueError("TMAP catalog mismatch")

    session_id = secrets.token_hex(8)

    manifest = build_manifest(session_id)

    with self._lock:
      self._session_id = session_id
      self._connected = True

      self._generation += 1

      # New control session:
      # remove records belonging to the previous session.
      self._records.clear()

      self._manifest_by_key = {f"{stream['kind']}:{stream['name']}": stream for stream in manifest["streams"]}

      self._state_changed.set()

    print(f"[carrot_navi] TMAP connected " f"session={session_id} " f"app={app_version}", flush=True)

    return manifest

  def control_disconnected(self) -> None:
    with self._lock:
      self._connected = False

      self._generation += 1
      self._state_changed.set()

  def validate_json_stream(self, session_id: str, name: str) -> None:

    with self._lock:
      if session_id != self._session_id:
        raise ValueError("stale session")

      stream = self._manifest_by_key.get(f"json:{name}")

      if stream is None:
        raise ValueError(f"unknown JSON stream: {name}")

      if not stream.get("enabled", False):
        raise ValueError(f"disabled JSON stream: {name}")

  def record_json(self, session_id: str, name: str, envelope: dict[str, Any]) -> None:
    if name == "speed":
      value = envelope.get("value") or {}
      print(f"[carrot_navi][speed] seq={envelope.get('sequence')} present={envelope.get('present')} "
        f"road={value.get('road_limit_kph')} sdi_present={value.get('sdi_present')} "
        f"sdi_type={value.get('sdi_type')} sdi_limit={value.get('sdi_speed_limit_kph')} "
        f"sdi_dist={value.get('sdi_distance_m')} plus_present={value.get('secondary_sdi_present')} "
        f"plus_type={value.get('secondary_sdi_type')} plus_limit={value.get('secondary_sdi_speed_limit_kph')} "
        f"plus_dist={value.get('secondary_sdi_distance_m')}", flush=True)

    self.validate_json_stream(session_id, name)
    if envelope.get("type") != "item_update":
      raise ValueError("invalid item type")
    if envelope.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("invalid item protocol")
    if envelope.get("session_id") != session_id:
      raise ValueError("item session mismatch")
    if envelope.get("kind") != "json" or envelope.get("name") != name:
      raise ValueError("item path mismatch")

    present = envelope.get("present")
    if not isinstance(present, bool):
      raise ValueError("present must be bool")

    value = envelope.get("value")
    if present:
      if name in JSON_ARRAY_NAMES:
        if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
          raise ValueError(f"{name} value must be array of objects")
      elif not isinstance(value, dict):
        raise ValueError(f"{name} value must be object")
    else:
      value = None

    try:
      sequence = int(envelope.get("sequence", 0))
    except (TypeError, ValueError):
      sequence = 0

    try:
      source_timestamp_ms = int(envelope.get("source_timestamp_ms", 0))
    except (TypeError, ValueError):
      source_timestamp_ms = 0

    sequence = max(0, sequence)
    source_timestamp_ms = max(0, source_timestamp_ms)

    with self._lock:
      previous = self._records.get(name)
      if previous is not None and sequence <= int(previous.get("sequence", -1)):
        return

      self._records[name] = {"present": present, "sequence": sequence, "source_timestamp_ms": source_timestamp_ms,
        "received_mono_ns": time.monotonic_ns(), "value": value}
      self._generation += 1
      self._state_changed.set()

  def cereal_snapshot(self) -> dict[str, Any]:

    with self._lock:
      # Copy each record so the cereal publisher
      # never observes a partially-updated dictionary.
      items = {name: dict(record) for name, record in self._records.items()}

      return {"generation": self._generation, "session_id": self._session_id, "connected": self._connected, "items": items}

  def wait_for_state_change(self, timeout: float) -> bool:

    changed = self._state_changed.wait(max(0.0, timeout))

    self._state_changed.clear()

    return changed

class DiscoveryBeacon:
  def __init__(self) -> None:
    self.ip = detect_advertise_ip()
    self.stop_event = threading.Event()

  @staticmethod
  def get_broadcast_ip(ip: str) -> str:
    try:
      parts = ip.split(".")
      return f"{parts[0]}.{parts[1]}.{parts[2]}.255" if len(parts) == 4 else "255.255.255.255"
    except Exception:
      return "255.255.255.255"

  def run(self) -> None:
    while not self.stop_event.is_set():
      current_ip = detect_advertise_ip()
      if current_ip != self.ip:
        self.ip = current_ip

      body = json.dumps({"ip": self.ip, "navi_debug": 1}, separators=(",", ":")).encode("utf-8")
      sock = None
      try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_ip = self.get_broadcast_ip(self.ip)
        sock.bind((self.ip, 0))
        sock.sendto(body, (broadcast_ip, DISCOVERY_PORT))
        print(f"[carrot_navi] discovery {self.ip} -> {broadcast_ip}:{DISCOVERY_PORT}", flush=True)
      except OSError as exc:
        print(f"[carrot_navi] discovery error: {exc}", flush=True)
      finally:
        if sock is not None:
          sock.close()

      self.stop_event.wait(DISCOVERY_INTERVAL_S)


def _recv_exact(conn: socket.socket, count: int,) -> bytes:

  data = bytearray()

  while len(data) < count:
    chunk = conn.recv(count - len(data))

    if not chunk:
      raise ConnectionError("socket closed")

    data.extend(chunk)

  return bytes(data)


def _read_http_request(conn: socket.socket,) -> tuple[str, dict[str, str]]:

  data = bytearray()

  while b"\r\n\r\n" not in data:
    chunk = conn.recv(4096)

    if not chunk:
      raise ConnectionError("http socket closed")

    data.extend(chunk)

    if len(data) > 65536:
      raise ValueError("http header too large")

  header = (bytes(data) .split(b"\r\n\r\n", 1,)[0] .decode("latin-1"))

  lines = header.split("\r\n")

  request_line = (lines[0].split())

  if len(request_line) < 2 or request_line[0] != "GET":
    raise ValueError("only GET supported")

  headers: dict[str, str] = {}

  for line in lines[1:]:
    if ":" not in line:
      continue

    key, value = line.split(":", 1,)

    headers[key.strip().lower()] = value.strip()

  return (request_line[1], headers,)


def _websocket_handshake(conn: socket.socket, headers: dict[str, str],) -> None:

  key = headers.get("sec-websocket-key")

  if not key:
    raise ValueError("missing websocket key")

  accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")

  response = ("HTTP/1.1 101 Switching Protocols\r\n" "Upgrade: websocket\r\n" "Connection: Upgrade\r\n"
    f"Sec-WebSocket-Accept: {accept}\r\n" "\r\n")

  conn.sendall(response.encode("ascii"))


def _read_ws_frame(conn: socket.socket,) -> tuple[int, bytes]:

  first, second = _recv_exact(conn, 2,)

  opcode = first & 0x0F

  masked = bool(second & 0x80)

  length = second & 0x7F

  if length == 126:
    length = struct.unpack(">H", _recv_exact(conn, 2,),)[0]

  elif length == 127:
    length = struct.unpack(">Q", _recv_exact(conn, 8,),)[0]

  if length > MAX_MESSAGE_BYTES:
    raise ValueError("websocket message too large")

  mask = (_recv_exact(conn, 4,) if masked else b"")

  payload = (_recv_exact(conn, length,) if length else b"")

  if masked:
    payload = bytes(value ^ mask[i % 4] for i, value in enumerate(payload))

  return (opcode, payload,)


def _send_ws_frame(conn: socket.socket, opcode: int, payload: bytes = b"",) -> None:

  length = len(payload)

  header = bytearray([0x80 | (opcode & 0x0F)])

  if length < 126:
    header.append(length)

  elif length <= 0xFFFF:
    header.append(126)

    header.extend(struct.pack(">H", length,))

  else:
    header.append(127)

    header.extend(struct.pack(">Q", length,))

  conn.sendall(bytes(header) + payload)


def _send_json(conn: socket.socket, value: dict[str, Any],) -> None:

  payload = json.dumps(value, ensure_ascii=False, separators=(",", ":"),).encode("utf-8")

  _send_ws_frame(conn, 0x1, payload,)


def _send_close(conn: socket.socket, code: int, reason: str = "",) -> None:

  payload = (struct.pack(">H", code,) + reason.encode("utf-8", errors="replace",)[:123])

  try:
    _send_ws_frame(conn, 0x8, payload,)
  except OSError:
    pass


def _serve_control_websocket(conn: socket.socket, app_version: str, receiver: CarrotNaviReceiver,) -> None:

  try:
    while True:
      opcode, payload = _read_ws_frame(conn)

      # Close
      if opcode == 0x8:
        return

      # Ping
      if opcode == 0x9:
        _send_ws_frame(conn, 0xA, payload,)
        continue

      # Pong
      if opcode == 0xA:
        continue

      # We only consume text frames.
      if opcode != 0x1:
        continue

      try:
        message = json.loads(payload.decode("utf-8"))
      except (UnicodeDecodeError, json.JSONDecodeError,):
        _send_close(conn, 1007, "invalid JSON",)
        return

      if message.get("type") == "requirements_query":
        manifest = receiver.negotiate(message, app_version,)

        _send_json(conn, manifest,)

  finally:
    receiver.control_disconnected()


def _serve_json_websocket(conn: socket.socket, session_id: str, name: str, receiver: CarrotNaviReceiver,) -> None:

  receiver.validate_json_stream(session_id, name,)

  while True:
    opcode, payload = _read_ws_frame(conn)

    # Close
    if opcode == 0x8:
      return

    # Ping
    if opcode == 0x9:
      _send_ws_frame(conn, 0xA, payload,)
      continue

    # Pong
    if opcode == 0xA:
      continue

    if opcode != 0x1:
      continue

    try:
      envelope = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError,):
      _send_close(conn, 1007, "invalid JSON",)
      return

    receiver.record_json(session_id, name, envelope,)


def _serve_websocket(conn: socket.socket, path: str, receiver: CarrotNaviReceiver,) -> None:

  parts = (urlparse(path) .path .strip("/") .split("/"))

  # /api/navi/ws/v2/control/<app_version>
  if len(parts) == 6 and parts[:5] == ["api", "navi", "ws", "v2", "control",]:
    app_version = parts[5]

    _serve_control_websocket(conn, app_version, receiver,)

    return

  # /api/navi/ws/v2/json/<session_id>/<name>
  if len(parts) == 7 and parts[:5] == ["api", "navi", "ws", "v2", "json",]:
    session_id = parts[5]
    name = parts[6]

    _serve_json_websocket(conn, session_id, name, receiver,)

    return

  _send_close(conn, 1008, "unsupported path",)


def _client_thread(conn: socket.socket, addr: tuple[str, int], receiver: CarrotNaviReceiver) -> None:
  try:
    conn.settimeout(30.0)
    path, headers = _read_http_request(conn)
    print(f"[carrot_navi] HTTP request {addr[0]}:{addr[1]} path={path} upgrade={headers.get('upgrade', '')}", flush=True)

    if headers.get("upgrade", "").lower() != "websocket":
      body = b'{"name":"Carrot Navi","ok":true}'
      response = ("HTTP/1.1 200 OK\r\n" "Content-Type: application/json\r\n" f"Content-Length: {len(body)}\r\n"
        "Connection: close\r\n" "\r\n").encode("ascii") + body
      conn.sendall(response)
      return

    _websocket_handshake(conn, headers)
    conn.settimeout(None)
    _serve_websocket(conn, path, receiver)

  except (ConnectionError, TimeoutError, OSError) as exc:
    print(f"[carrot_navi] client {addr[0]}:{addr[1]} disconnect/error: {exc}", flush=True)
  except Exception as exc:
    print(f"[carrot_navi] client {addr[0]}:{addr[1]} error: {exc}", flush=True)
  finally:
    try:
      conn.close()
    except OSError:
      pass


def run_server(receiver: CarrotNaviReceiver,) -> None:

  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM,)

  server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1,)

  server.bind((DEFAULT_HOST, DEFAULT_PORT,))

  server.listen(16)

  print(f"[carrot_navi] " f"TMAP receiver :{DEFAULT_PORT}", flush=True)

  while True:
    conn, addr = server.accept()
    print(f"[carrot_navi] TCP client connected {addr[0]}:{addr[1]}", flush=True)

    threading.Thread(target=_client_thread, args=(conn, addr, receiver,), daemon=True, name="carrot_navi_ws",).start()


def main() -> None:
  receiver = CarrotNaviReceiver()

  # Import here so carrot_navi remains independent of cereal
  # until the actual process starts.
  from openpilot.selfdrive.carrot.carrot_navi_cereal import (CarrotNaviCerealPublisher,)

  cereal_publisher = CarrotNaviCerealPublisher(receiver)

  cereal_publisher.start()

  beacon = DiscoveryBeacon()

  threading.Thread(target=beacon.run, daemon=True, name="carrot_navi_discovery",).start()

  run_server(receiver)


if __name__ == "__main__":
  main()
