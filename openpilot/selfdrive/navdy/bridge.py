"""Read-only vehicle telemetry to Navdy; bounded navigation feedback to Carrot."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import select
import shlex
import signal
import subprocess
import time

from openpilot.selfdrive.navdy.navigation import FEEDBACK_PATH, STATUS_PATH
from openpilot.selfdrive.navdy.transport import AdbTransport, LineReader, encode_frame
from openpilot.selfdrive.navdy.bluetooth import BluetoothTransport, CONFIG_PATH, configured_connection
from openpilot.selfdrive.navdy.connection import UsbProbe
from openpilot.selfdrive.navdy.ambient import DoorMonitor, read_override, road_state
from openpilot.selfdrive.navdy.model_navigation import ModelNavigationTracker, ModelRouteCache, age_model_feedback
from openpilot.selfdrive.navdy.diagnostics import LinkDiagnostics, public_error
from openpilot.selfdrive.navdy.feedback import is_no_observation
from openpilot.selfdrive.navdy.journal import CONNECT_TIMINGS, LocalJournal, sanitize
from openpilot.selfdrive.navdy.vehicles import VehicleOverlay
from openpilot.selfdrive.navdy.policy import read_transport_mode, transport_policy
from openpilot.selfdrive.navdy.usb_health import RecoveryGuard, UsbHealth, recovery_messages
from openpilot.selfdrive.navdy.usb_faults import UsbFaultRecorder

MAINTENANCE_STATE = Path('/data/hl-jetson')
JETSON_PYTHON = '/opt/carrot-jetlink/venv/bin/python'
JETSON_RELAY = '/opt/carrot-jetlink/current/openpilot/selfdrive/navdy/transport.py'
SERVICES = ['deviceState', 'pandaStates', 'carState', 'selfdriveState', 'controlsState', 'carControl',
            'carOutput', 'modelV2', 'radarState', 'liveTracks', 'longitudinalPlan', 'carrotMan']
DISPLAY_INTERVAL = 0.2
DISPLAY_MAX_AGE = 0.5
DEVICE_MAX_AGE = 1.5  # deviceState is published at 2 Hz.
BLUETOOTH_FAST_RETRY = 0.2
BLUETOOTH_FAST_RETRY_COOLDOWN = 30.0
BLUETOOTH_HEALTHY_SECONDS = 2.0
BLUETOOTH_ACK_MAX_GAP = 1.0
BLUETOOTH_ACK_MAX_AGE = 2.0  # Covers the 1.5 s response deadline plus display polling.


class BluetoothReconnect:
  """One quick retry for a recently healthy radio, not just an open socket."""
  def __init__(self):
    self.next_fast_retry = 0.0
    self.reset_session()

  def reset_session(self):
    self.first_ack = None
    self.last_ack = None

  def acknowledge(self, received_at, now):
    if not math.isfinite(received_at) or not 0 <= now - received_at <= BLUETOOTH_ACK_MAX_AGE:
      self.reset_session()
      return
    if self.last_ack is not None and received_at <= self.last_ack:
      return
    if self.last_ack is None or received_at - self.last_ack > BLUETOOTH_ACK_MAX_GAP:
      self.first_ack = received_at
    self.last_ack = received_at

  def healthy(self, now):
    return (self.first_ack is not None and self.last_ack - self.first_ack >= BLUETOOTH_HEALTHY_SECONDS
            and 0 <= now - self.last_ack <= BLUETOOTH_ACK_MAX_AGE)

  def take_fast_retry(self, now):
    if not self.healthy(now) or now < self.next_fast_retry:
      return False
    self.next_fast_retry = now + BLUETOOTH_FAST_RETRY_COOLDOWN
    return True


def atomic_json(path, value):
  temporary = path.with_suffix('.tmp')
  temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
  temporary.replace(path)


def connect_timings(connection):
  if not isinstance(connection, dict) or not isinstance(connection.get('steps'), list):
    return None
  steps = {step['phase']: step for step in connection['steps']
           if type(step) is dict and type(step.get('phase')) is str and step['phase'] in CONNECT_TIMINGS}
  return sanitize(steps, CONNECT_TIMINGS)


class SshTransport:
  def __init__(self, host, state=MAINTENANCE_STATE):
    from tools.jetlink.manage_jetson import address, paired_config, ssh_options  # noqa: TID251 -- operational tools live at repository root
    config = paired_config(state)
    relay = (["sudo", "-n", "/usr/bin/python3", "-u", "/run/carrot-data/hl-navdy/navdy_management.py", "relay"]
             if config.get('navdy_relay') == 'protected-tools-v1' else [JETSON_PYTHON, '-u', JETSON_RELAY, '--stdio'])
    remote = shlex.join(relay)
    self.process = subprocess.Popen(['ssh', *ssh_options(state), '-T', 'jetlink@' + address(host), remote],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    os.set_blocking(self.process.stdin.fileno(), False)
    os.set_blocking(self.process.stdout.fileno(), False)
    self.reader = LineReader()
    self.deadline = 0.0

  def connect(self):
    from openpilot.selfdrive.navdy.display import payload_from_messages
    # Old relays have no readiness handshake. An inactive bootstrap is compatible
    # with them and cannot forward active telemetry captured before SSH/ADB setup.
    self.exchange(payload_from_messages(seq=0, onroad=False))

  def recv(self, size):
    timeout = self.deadline - time.monotonic()
    if timeout <= 0 or not select.select([self.process.stdout], [], [], timeout)[0]:
      raise TimeoutError('Jetson Navdy relay response timed out')
    return os.read(self.process.stdout.fileno(), size)

  def exchange(self, payload):
    # First SSH setup may take longer; subsequent display/feedback exchanges are bounded.
    self.deadline = time.monotonic() + (6 if not getattr(self, 'connected', False) else 1)
    data = encode_frame(payload)
    while data:
      timeout = self.deadline - time.monotonic()
      if timeout <= 0 or not select.select([], [self.process.stdin], [], timeout)[1]:
        raise TimeoutError('Jetson Navdy relay write timed out')
      sent = os.write(self.process.stdin.fileno(), data)
      data = data[sent:]
    result = self.reader.receive(self)
    self.connected = True
    return result

  def close(self):
    self.process.terminate()
    try:
      self.process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=2)
    self.process.stdin.close()
    self.process.stdout.close()


def jetson_candidates():
  try:
    from tools.jetlink.manage_jetson import candidates, discover, paired_config  # noqa: TID251 -- operational tools live at repository root
    config = paired_config(MAINTENANCE_STATE)
    return candidates(config, MAINTENANCE_STATE, discover())
  except (OSError, ValueError, ImportError, subprocess.SubprocessError):
    return []


def display_sources(sm):
  now = time.monotonic()
  sources = {}
  for service in SERVICES:
    received_age = now - sm.recv_time[service]
    published_age = now - sm.logMonoTime[service] / 1e9
    clock_ok = (sm.logMonoTime[service] > 0 and math.isfinite(received_age) and math.isfinite(published_age)
                and received_age >= 0 and published_age >= 0)
    age = max(received_age, published_age) if clock_ok else None
    limit = DEVICE_MAX_AGE if service in ('deviceState', 'pandaStates') else DISPLAY_MAX_AGE
    if not sm.seen[service]:
      reason = 'not_seen'
    elif not sm.valid[service]:
      reason = 'invalid'
    elif age is None:
      reason = 'invalid_timestamp'
    elif age >= limit:
      reason = 'stale'
    else:
      reason = ''
    sources[service] = {'usable': not reason, 'reason': reason, 'alive': bool(sm.alive[service]),
                        'valid': bool(sm.valid[service]), 'ageSeconds': round(age, 4) if age is not None else None}
  return sources


def usb_recovery_allowed(sm):
  sm.update(0)
  return recovery_messages(sm, time.monotonic())


def live_payload(sm, sequence, *, sources=None, door_open=False, ambient_override=None, vehicles=None):
  from openpilot.selfdrive.navdy.display import payload_from_messages, resolve_operating_state
  sources = display_sources(sm) if sources is None else sources

  def recent(service):
    # SubMaster's 100 ms liveness cutoff is shorter than this display's 200 ms
    # tick. Tolerate a missed tick, not an invalid message or an old queued frame.
    return sm[service] if sources[service]['usable'] else None

  device = recent('deviceState')
  onroad = device is not None and bool(device.started)
  car_state = recent('carState')
  known_road = road_state(device, recent('pandaStates'))
  if known_road is True and car_state is not None and car_state.canValid and not car_state.canTimeout:
    door_open = bool(car_state.doorOpen)
  payload = payload_from_messages(
    seq=sequence, onroad=onroad, car_state=car_state, selfdrive_state=recent('selfdriveState'),
    controls_state=recent('controlsState'), car_control=recent('carControl'), car_output=recent('carOutput'),
    model=recent('modelV2'), radar_state=recent('radarState'), longitudinal_plan=recent('longitudinalPlan'),
    carrot_man=recent('carrotMan'), door_open=door_open,
  )
  if vehicles is not None:
    vehicles.update(payload, recent('modelV2'), recent('liveTracks'),
                    timestamp=sm.logMonoTime['liveTracks'] / 1e9, now=time.monotonic())
  # Offroad has no card/selfdrived messages. Ambient/power state instead comes
  # from fresh device/panda state; never grant active telemetry from this path.
  payload['displayPower'] = 'unknown' if known_road is None else 'onroad' if known_road else 'offroad'
  if known_road is not None:
    payload['onroad'] = known_road
    payload['ambientState'] = resolve_operating_state(connected=True, onroad=known_road,
                                                     door_open=door_open, gear=payload['gear'] if known_road else 'unknown')
  else:
    payload['ambientState'] = 'disconnected'
  if ambient_override is not None:
    payload['ambientOverride'] = ambient_override
  if payload['active'] and 'navPathLeft' not in payload:
    # The HUD retains geometry when keys are omitted; empty arrays explicitly clear it.
    payload.update(navPathLeft=[], navPathRight=[], navLaneLeft=[], navLaneRight=[])
  return payload


def run(*, feedback=FEEDBACK_PATH, status=STATUS_PATH, duration=None, bluetooth_config=CONFIG_PATH):
  from openpilot.cereal import messaging
  from openpilot.common.swaglog import cloudlog
  sm = messaging.SubMaster(SERVICES)
  lock_path = feedback.with_suffix('.lock')
  with lock_path.open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    transport = None
    transport_name = 'usb'
    connected_at = 0.0
    sequence = 0
    start = time.monotonic()
    next_attempt = 0.0
    candidate_index = 0
    failures = 0
    bluetooth_reconnect = BluetoothReconnect()
    bluetooth_connect = None
    peer = None
    bluetooth_only = False
    legacy_bluetooth_only = False
    policy = None
    connection_mode = 0
    config_error = ''
    config_checked = -math.inf
    last_handover = None
    last_connected_transport = None
    previous_available = False
    extended_feedback = False
    feedback_received_at = -math.inf
    last_display_dropout = None
    model_navigation = ModelNavigationTracker()
    model_route_path = feedback.with_name(feedback.stem + '-model-route.json')
    model_route_cache = ModelRouteCache(model_route_path, atomic_json)
    vehicles = VehicleOverlay()
    doors = DoorMonitor()
    diagnostics = LinkDiagnostics(cloudlog.event, journal=LocalJournal())
    recovery_events = queue.Queue(maxsize=32)
    def record_recovery(*args, **kwargs):
      try:
        recovery_events.put_nowait((args, kwargs))
      except queue.Full:
        pass
    usb_health = UsbHealth(status.with_suffix('.usb-recovery.json'), RecoveryGuard(), record_recovery)
    def usb_transport():
      candidate = AdbTransport()
      candidate.usb_health = usb_health
      return candidate
    usb_probe = UsbProbe(usb_transport)
    usb_faults = UsbFaultRecorder()
    try:
      diagnostics.record('bridge_started')
      doors.start()
      while duration is None or time.monotonic() - start < duration:
        tick = time.monotonic()
        for _ in range(32):
          try:
            args, kwargs = recovery_events.get_nowait()
          except queue.Empty:
            break
          diagnostics.record(*args, **kwargs)
        sm.update(0)
        sequence += 1
        if tick - config_checked >= 5:
          config_checked = tick
          old_policy = policy
          try:
            peer, legacy_bluetooth_only = configured_connection(bluetooth_config)
            config_error = ''
          except (OSError, ValueError) as error:
            peer, config_error = None, str(error)[:160]
          try:
            connection_mode = read_transport_mode()
            policy = transport_policy(connection_mode, legacy_bluetooth_only)
          except (OSError, ValueError) as error:
            connection_mode, policy, config_error = None, 'invalid', str(error)[:160]
          bluetooth_only = policy == 'bluetooth_only'
          if old_policy != policy or (transport is not None and transport_name == 'bluetooth' and peer != transport.peer):
            diagnostics.record('policy_changed', transport=transport_name, bluetoothOnly=bluetooth_only)
            if transport is not None:
              transport.close()
              diagnostics.record('transport_closed', transport=transport_name)
            transport = None
            bluetooth_reconnect.reset_session()
            usb_probe.close()
            usb_probe = UsbProbe(usb_transport)
            feedback.unlink(missing_ok=True)
            extended_feedback = False
            model_navigation.clear('model_link_changed')
            model_route_cache.update(model_navigation)
            next_attempt, failures, candidate_index = 0.0, 0, 0
            if old_policy is not None:
              atomic_json(status, {'updatedAt': time.monotonic(), 'connected': False,
                                   'transportPolicy': policy, 'connectionMode': connection_mode,
                                   'reason': 'connection_mode_changed'})
        if not bluetooth_only and transport_name in ('bluetooth', 'jetson') and transport is not None:
          ready = usb_probe.poll(tick)
          if ready is not None:
            diagnostics.record('usb_handover', transport=transport_name)
            transport.close()
            diagnostics.record('transport_closed', transport=transport_name)
            usb_probe.close()
            usb_probe = UsbProbe(usb_transport)
            transport = ready
            transport.usb_health = usb_health
            usb_health.verified(getattr(transport, 'serial', None))
            bluetooth_reconnect.reset_session()
            feedback.unlink(missing_ok=True)
            extended_feedback = False
            model_navigation.clear('model_link_changed')
            model_route_cache.update(model_navigation)
            if ready is not None:
              transport_name, connected_at = 'usb', time.monotonic()
              diagnostics.record('transport_open', transport=transport_name, connectedAt=connected_at, local_only=True)
            next_attempt, failures = 0.0, 0
            sm.update(0)
        diagnostics.periodic(connected=transport is not None, transport=transport_name, bluetooth_only=bluetooth_only,
                             retry_in=next_attempt - tick, wireless=getattr(transport, 'last_exchange', None))
        if transport is None and tick < next_attempt:
          time.sleep(0.1)
          continue
        phase = 'connect'
        diagnostics.begin_phase(phase)
        try:
          if transport is None:
            vehicles.clear()
            bluetooth_reconnect.reset_session()
            if policy == 'invalid':
              raise ValueError(config_error)
            transport_name = 'bluetooth' if bluetooth_only else 'usb'
            diagnostics.record('connect_attempt', transport=transport_name)
            try:
              if bluetooth_only:
                if peer is None:
                  raise ConnectionError(config_error or 'Bluetooth-only mode requires an enabled paired peer')
                transport = BluetoothTransport(peer)
              else:
                transport = usb_transport()
              transport.connect()
            except (OSError, ValueError, subprocess.SubprocessError) as connect_error:
              if transport_name == 'bluetooth':
                bluetooth_connect = getattr(transport, 'connect_diagnostics', None)
              diagnostics.record('connect_failed', transport=transport_name,
                                  reason=public_error(connect_error),
                                  phaseMs=(time.monotonic() - diagnostics.phase_started) * 1000,
                                  connectTimings=connect_timings(bluetooth_connect) if transport_name == 'bluetooth' else None)
              if transport is not None:
                transport.close()
              transport = None
              if bluetooth_only:
                raise
              hosts = jetson_candidates()
              options = ([('bluetooth', peer)] if peer and policy != 'usb_only' else [])
              options.extend(('jetson', host) for host in hosts)
              if not options:
                if policy == 'usb_only':
                  raise ConnectionError('Navdy USB not found; no paired Jetson relay (wired mode)') from None
                raise ConnectionError(config_error or 'Navdy USB not found; no paired Jetson or enabled Bluetooth peer') from None
              transport_name, target = options[candidate_index % len(options)]
              candidate_index += 1
              diagnostics.record('connect_attempt', transport=transport_name)
              if transport_name == 'bluetooth':
                transport = BluetoothTransport(target)
                transport.connect()
              else:
                transport = SshTransport(target)
                transport.connect()
            if transport_name == 'bluetooth':
              bluetooth_connect = getattr(transport, 'connect_diagnostics', None)
            connected_at = time.monotonic()
            diagnostics.record('transport_open', transport=transport_name, connectedAt=connected_at,
                                connectTimings=connect_timings(bluetooth_connect) if transport_name == 'bluetooth' else None)
            # Discovery/service startup can block. Drain fresh messages before sending.
            sm.update(0)
          phase = 'payload'
          diagnostics.begin_phase(phase)
          captured_ns = time.monotonic_ns()
          sources = display_sources(sm)
          payload = live_payload(sm, sequence, sources=sources, door_open=doors.door_open,
                                 ambient_override=read_override(), vehicles=vehicles)
          payload['modelRoute'] = model_navigation.request(time.monotonic())
          car_state = sm['carState']
          diagnostics.sources(sources, payload,
                              can_valid=car_state.canValid if sources['carState']['usable'] else True,
                              can_timeout=car_state.canTimeout if sources['carState']['usable'] else False)
          phase = 'exchange'
          diagnostics.begin_phase(phase)
          exchange_started = time.monotonic()
          if transport_name == 'bluetooth':
            replies = transport.exchange_batch(payload, captured_ns=captured_ns)
          else:
            response = transport.exchange(payload)
            received = time.monotonic()
            replies = [(response, received, (received - exchange_started) * 1000.)]
          received = time.monotonic()
          diagnostics.exchanged(len(replies))
          phase = 'feedback'
          diagnostics.begin_phase(phase)
          latest_feedback = None
          for response, reply_received, elapsed_ms in replies:
            diagnostics.feedback(response, transport=transport_name, received_at=reply_received, elapsed_ms=elapsed_ms,
                                 wireless=getattr(transport, 'last_exchange', None))
            if transport_name == 'bluetooth':
              bluetooth_reconnect.acknowledge(reply_received, received)
            if is_no_observation(response):
              continue
            if (type(reply_received) not in (int, float) or not math.isfinite(reply_received)
                or not 0 <= reply_received <= received or reply_received <= feedback_received_at):
              continue
            had_model_field = 'modelNavigation' in response
            age_model_feedback(response, elapsed_ms)
            if not had_model_field or is_no_observation(response):
              # Aging may insert or replace the model with null. This was a
              # real observation, so preserve its navigation clear, not a
              # synthetic two-null discard sentinel in the reader's file.
              response.pop('modelNavigation', None)
            model_navigation.accept(response.get('modelNavigation'), reply_received, received)
            latest_feedback = response, reply_received
            feedback_received_at = reply_received
          model_navigation._fresh(received)
          cache_token = model_route_cache.update(model_navigation)
          if transport_name == 'usb':
            # Every new cable loss starts with the configured wireless peer,
            # not the next failed relay left over from an earlier handover.
            candidate_index = 0
          if last_connected_transport is not None and last_connected_transport != transport_name:
            last_handover = {'at': received, 'from': last_connected_transport, 'to': transport_name}
          last_connected_transport = transport_name
          healthy = bluetooth_reconnect.healthy(received) if transport_name == 'bluetooth' else received - connected_at >= 2
          if healthy:
            failures = 0
            if transport_name == 'bluetooth':
              # Retry a proven wireless link first after a transient loss. A
              # failed reconnect still advances to the paired Jetson relays.
              candidate_index = 0
          missing = {service: sources[service]['reason'] for service in ('deviceState', 'carState', 'selfdriveState')
                     if not sources[service]['usable']}
          car_state = sm['carState']
          if sources['carState']['usable'] and (not car_state.canValid or car_state.canTimeout):
            missing['carState'] = 'can_invalid' if not car_state.canValid else 'can_timeout'
          if previous_available and missing:
            last_display_dropout = {'at': received, 'sources': missing}
          previous_available = not missing
          # No observation (including discard ACKs) must not refresh the old
          # receipt or clear it. A later malformed/real-clear ACK still wins.
          if latest_feedback is not None:
            response, reply_received = latest_feedback
            extended_feedback = 'navigationFeedback' in response
            atomic_json(feedback, {'receivedAt': reply_received, 'feedback': response, 'modelRouteCacheToken': cache_token})
          phase = 'status'
          diagnostics.begin_phase(phase)
          atomic_json(status, {'updatedAt': received, 'connected': True, 'transport': transport_name,
                               'connectedAt': connected_at, 'extendedFeedback': extended_feedback,
                               'display': {'sequence': sequence, 'onroad': payload['onroad'], 'active': payload['active'],
                                           'ambientState': payload['ambientState'], 'sources': sources,
                                           'vehicleCount': len(payload['navVehicles'])},
                               'power': payload['displayPower'],
                               'ambient': {'doorOpen': payload['doorOpen'], 'gmDoorMonitor': doors.gm, 'error': doors.error},
                               'lastDisplayDropout': last_display_dropout,
                               'lastHandover': last_handover, 'wirelessConfigured': peer is not None,
                               'transportPolicy': policy, 'connectionMode': connection_mode,
                               'wireless': getattr(transport, 'last_exchange', None),
                               'bluetoothConnect': bluetooth_connect,
                               'modelNavigation': {'routeReady': bool(model_navigation.points),
                                                   'reason': model_navigation.reason},
                               'diagnostics': diagnostics.snapshot(),
                               'reason': ''})
          diagnostics.end_phase()
        except (OSError, ValueError, subprocess.SubprocessError) as error:
          if transport_name == 'usb' and phase == 'exchange':
            usb_faults.capture(getattr(transport, 'serial', None))
          if transport_name == 'bluetooth' and transport is not None and phase == 'connect':
            bluetooth_connect = getattr(transport, 'connect_diagnostics', None)
            # The USB-first fallback bypasses the initial connect_failed handler.
            diagnostics.record('connect_failed', transport=transport_name,
                                phaseMs=(time.monotonic() - diagnostics.phase_started) * 1000,
                                connectTimings=connect_timings(bluetooth_connect))
          fast_retry = (transport_name == 'bluetooth' and phase == 'exchange' and isinstance(error, OSError)
                        and bluetooth_reconnect.take_fast_retry(time.monotonic()))
          bluetooth_reconnect.reset_session()
          diagnostics.failure(phase=phase, transport=transport_name, error=error, connected_at=connected_at,
                              wireless=getattr(transport, 'last_exchange', None))
          diagnostics.end_phase()
          if transport_name in ('bluetooth', 'jetson'):
            usb_probe.close()
            usb_probe = UsbProbe(usb_transport)
          if transport is not None:
            transport.close()
            diagnostics.record('transport_closed', transport=transport_name)
            transport = None
          feedback.unlink(missing_ok=True)
          extended_feedback = False
          model_navigation.clear('model_disconnected')
          model_route_cache.update(model_navigation)
          failures += 1
          retry_delay = BLUETOOTH_FAST_RETRY if fast_retry else min(10, 2 ** min(failures, 4))
          diagnostics.record('retry_scheduled', transport=transport_name, retryInSeconds=retry_delay, fastRetry=fast_retry)
          atomic_json(status, {'updatedAt': time.monotonic(), 'connected': False, 'transport': transport_name,
                               'transportPolicy': policy, 'connectionMode': connection_mode,
                               'retryInSeconds': retry_delay, 'wirelessConfigured': peer is not None,
                               'bluetoothConnect': bluetooth_connect,
                               'diagnostics': diagnostics.snapshot(),
                               'reason': public_error(error) if transport_name == 'bluetooth' else str(error)[:200]})
          next_attempt = time.monotonic() + retry_delay
        time.sleep(max(0, DISPLAY_INTERVAL - (time.monotonic() - tick)))
    finally:
      # Every cleanup runs even if an earlier resource or status write fails.
      # Journal close is last and only joins its daemon for a bounded interval.
      with ExitStack() as cleanup:
        cleanup.callback(usb_faults.close)
        cleanup.callback(diagnostics.close)
        cleanup.callback(lambda: atomic_json(status, {'updatedAt': time.monotonic(), 'connected': False, 'reason': 'bridge_stopped',
                                                      'bluetoothConnect': bluetooth_connect,
                                                      'diagnostics': diagnostics.snapshot()}))
        cleanup.callback(diagnostics.record, 'bridge_stopped', timings=diagnostics.timings,
                         exchanges=diagnostics.exchanges, replies=diagnostics.replies, feedback=diagnostics.feedback_stats)
        cleanup.callback(model_route_path.unlink, missing_ok=True)
        cleanup.callback(feedback.unlink, missing_ok=True)
        if transport is not None:
          cleanup.callback(transport.close)
        cleanup.callback(usb_probe.close)
        cleanup.callback(doors.close)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--feedback-file', type=Path, default=FEEDBACK_PATH)
  parser.add_argument('--status-file', type=Path, default=STATUS_PATH)
  parser.add_argument('--duration', type=float)
  args = parser.parse_args()

  def stop(*_args):
    raise SystemExit(0)

  signal.signal(signal.SIGTERM, stop)
  run(feedback=args.feedback_file, status=args.status_file, duration=args.duration)


if __name__ == '__main__':
  main()
