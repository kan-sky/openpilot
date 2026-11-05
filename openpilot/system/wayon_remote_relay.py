#!/usr/bin/env python3
import base64
import binascii
import json
import re
import socket
import subprocess
import threading
import time

from websocket import ABNF, WebSocketException, WebSocketTimeoutException, create_connection

from openpilot.system.wayon_identity import DEFAULT_CONFIG_PATH, ensure_wayon_identity
from openpilot.system.wayon_ambient_delivery import AMBIENT_NOTIFY_MESSAGE, notify_ambient_command


USER_AGENT = "wayon-device-relay/1.0"
RELAY_TARGETS = {"ssh": ("127.0.0.1", 22), "live": ("127.0.0.1", 8765)}
HEARTBEAT_INTERVAL_SECONDS = 20
RECONNECT_DELAY_SECONDS = 1
SSH_AUTHORIZE_PREFIX = "wayon-ssh-authorize-v1."
SSH_AUTHORIZATION_ID_RE = re.compile(r"^[0-9a-f]{32}$", re.IGNORECASE)
SSH_KEY_RE = re.compile(r"^(ssh-rsa|ssh-ed25519) ([A-Za-z0-9+/]{40,4096}={0,2})(?: [^\r\n]{1,128})?$")
SSH_SESSION_KEY_TTL_MIN_SECONDS = 60
SSH_SESSION_KEY_TTL_MAX_SECONDS = 120


def decode_ssh_authorization(command: str) -> dict | None:
  if not command.startswith(SSH_AUTHORIZE_PREFIX):
    return None
  encoded = command[len(SSH_AUTHORIZE_PREFIX):]
  try:
    encoded += "=" * (-len(encoded) % 4)
    payload = json.loads(base64.urlsafe_b64decode(encoded).decode("utf-8"))
  except (binascii.Error, UnicodeDecodeError, json.JSONDecodeError, ValueError):
    return None
  if not isinstance(payload, dict):
    return None

  public_key = str(payload.get("publicKey") or "").strip()
  authorization_id = str(payload.get("authorizationId") or "")
  try:
    ttl_seconds = int(payload.get("ttlSeconds"))
  except (TypeError, ValueError):
    return None
  match = SSH_KEY_RE.fullmatch(public_key)
  if (match is None or SSH_AUTHORIZATION_ID_RE.fullmatch(authorization_id) is None
      or not SSH_SESSION_KEY_TTL_MIN_SECONDS <= ttl_seconds <= SSH_SESSION_KEY_TTL_MAX_SECONDS):
    return None
  try:
    base64.b64decode(match.group(2), validate=True)
  except (binascii.Error, ValueError):
    return None
  return {
    "public_key": f"{match.group(1)} {match.group(2)} wayon-session-{authorization_id}",
    "ttl_seconds": ttl_seconds,
  }


class TemporarySshAuthorizer:
  def __init__(self, params=None, timer_factory=threading.Timer):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.timer_factory = timer_factory
    self.lock = threading.Lock()

  def _current_lines(self) -> list[str]:
    value = self.params.get("GithubSshKeys") or b""
    if isinstance(value, bytes):
      value = value.decode("utf-8", "replace")
    return [line.strip() for line in str(value).splitlines() if line.strip()]

  def _write_lines(self, lines: list[str]) -> None:
    if lines:
      self.params.put("GithubSshKeys", "\n".join(lines) + "\n")
    else:
      self.params.remove("GithubSshKeys")

  def remove(self, authorized_key: str) -> None:
    with self.lock:
      self._write_lines([line for line in self._current_lines() if line != authorized_key])

  def authorize(self, command: str) -> bool:
    authorization = decode_ssh_authorization(command)
    if authorization is None:
      return False
    authorized_key = authorization["public_key"]
    with self.lock:
      lines = self._current_lines()
      if authorized_key not in lines:
        lines.append(authorized_key)
        self._write_lines(lines)
    timer = self.timer_factory(authorization["ttl_seconds"], self.remove, args=(authorized_key,))
    timer.daemon = True
    timer.start()
    return True


class RelayChannel:
  def __init__(self, kind: str, endpoint: str, token: str, ssh_authorizer: TemporarySshAuthorizer | None = None,
               allow_ssh: bool = False):
    self.kind = kind
    self.endpoint = endpoint
    self.token = token
    self.ssh_authorizer = ssh_authorizer
    self.allow_ssh = allow_ssh
    self.local: socket.socket | None = None
    self.local_lock = threading.Lock()
    self.retiring_websocket = None

  def close_local(self, expected=None, failed_websocket=None) -> bool:
    with self.local_lock:
      if expected is not None and self.local is not expected:
        return False
      local, self.local = self.local, None
      if local is not None and failed_websocket is not None:
        self.retiring_websocket = failed_websocket
    if local is not None:
      try:
        local.shutdown(socket.SHUT_RDWR)
      except OSError:
        pass
      local.close()
    return local is not None

  def open_local(self, websocket) -> None:
    self.close_local()
    if self.kind == "ssh":
      if not self.allow_ssh:
        return
      subprocess.run(["sudo", "-n", "systemctl", "start", "ssh"], check=False,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    local = socket.create_connection(RELAY_TARGETS[self.kind], timeout=10)
    local.settimeout(1.0)
    with self.local_lock:
      if self.retiring_websocket is websocket:
        local.close()
        raise WebSocketException("live relay session ended")
      self.local = local

    def forward() -> None:
      try:
        while True:
          try:
            data = local.recv(64 * 1024)
          except TimeoutError:
            continue
          if not data:
            break
          websocket.send(data, opcode=ABNF.OPCODE_BINARY)
      except (OSError, WebSocketException) as exc:
        print(f"Wayon relay {self.kind}: local forwarding ended {type(exc).__name__}", flush=True)
      finally:
        failed_websocket = websocket if self.kind == "live" else None
        if self.close_local(expected=local, failed_websocket=failed_websocket) and failed_websocket is not None:
          # End the Cloud session too; otherwise the viewer retains a dead LIVE
          # socket. Explicit peer-close/replacement already detached this owner.
          print("Wayon relay live: local stream ended, closing cloud session", flush=True)
          websocket.close(timeout=1)

    threading.Thread(target=forward, name=f"wayon-{self.kind}-local", daemon=True).start()

  def relay_connected(self, websocket) -> None:
    websocket.settimeout(HEARTBEAT_INTERVAL_SECONDS)
    if self.kind == "ssh":
      notify_ambient_command()

    while True:
      try:
        opcode, data = websocket.recv_data(control_frame=True)
      except WebSocketTimeoutException:
        # Cloudflare answers protocol pings at the edge and does not always
        # expose the pong to this client. Keep the healthy idle socket until a
        # ping/send actually fails instead of forcing periodic reconnects.
        websocket.ping(f"wayon-{self.kind}".encode())
        continue

      if opcode == ABNF.OPCODE_TEXT:
        command = data.decode("utf-8", "replace") if isinstance(data, bytes) else str(data)
        if self.kind == "ssh" and command.startswith(SSH_AUTHORIZE_PREFIX):
          if not self.allow_ssh:
            continue
          if self.ssh_authorizer is None:
            self.ssh_authorizer = TemporarySshAuthorizer()
          if not self.ssh_authorizer.authorize(command):
            raise WebSocketException("invalid Wayon SSH authorization")
        elif self.kind == "ssh" and command == AMBIENT_NOTIFY_MESSAGE:
          notify_ambient_command()
        elif command == "wayon-peer-open":
          self.open_local(websocket)
        elif command == "wayon-peer-close":
          self.close_local()
      elif opcode == ABNF.OPCODE_BINARY:
        if self.kind == "live" and data.startswith(b"WLC1"):
          print(f"Wayon relay live: capture bytes={len(data)}", flush=True)
        with self.local_lock:
          local = self.local
        if local is not None:
          local.sendall(data)
      elif opcode == ABNF.OPCODE_CLOSE:
        code = int.from_bytes(data[:2], "big") if len(data) >= 2 else 0
        print(f"Wayon relay {self.kind}: peer close code={code}", flush=True)
        return

  def run(self) -> None:
    websocket_url = self.endpoint.replace("https://", "wss://", 1).replace("http://", "ws://", 1)
    websocket_url = f"{websocket_url}/api/device/relay/{self.kind}"
    while True:
      websocket = None
      try:
        websocket = create_connection(
          websocket_url,
          header=[f"Authorization: Bearer {self.token}", f"User-Agent: {USER_AGENT}"],
          timeout=30,
          enable_multithread=True,
        )
        self.relay_connected(websocket)
      except (OSError, WebSocketException) as exc:
        print(f"Wayon relay {self.kind}: reconnecting after {type(exc).__name__}", flush=True)
      finally:
        self.close_local()
        try:
          websocket.close()
        except Exception:
          pass
      time.sleep(RECONNECT_DELAY_SECONDS)


def main() -> None:
  while True:
    try:
      config = ensure_wayon_identity(DEFAULT_CONFIG_PATH)
      if config is None:
        raise RuntimeError("Wayon identity unavailable")
      endpoint = str(config["endpoint"]).rstrip("/")
      token = str(config["token"])
      # The historical 'ssh' channel also carries ambient wake notifications.
      # Subscribe to those, but do not grant shell access through a viewing key.
      channels = [RelayChannel(kind, endpoint, token, allow_ssh=False) for kind in RELAY_TARGETS]
      threads = [threading.Thread(target=channel.run, name=f"wayon-relay-{channel.kind}") for channel in channels]
      for thread in threads:
        thread.start()
      for thread in threads:
        thread.join()
    except Exception as exc:
      print(f"Wayon relay: startup failed: {exc}", flush=True)
      time.sleep(10)


if __name__ == "__main__":
  main()
