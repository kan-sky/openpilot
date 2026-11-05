"""Read-only ambient lifecycle inputs, including doors after card stops offroad."""
from __future__ import annotations

from datetime import datetime
from itertools import groupby
import json
from pathlib import Path
import threading
import time

DOOR_PATH = Path('/dev/shm/params/d/OffroadDoorOpen')
OVERRIDE_PATHS = (Path('/dev/shm/params/d/WayonAmbientOverride'), Path('/data/params/d/WayonAmbientOverride'))
WATCH_BUSES = (0, 1, 2, 128, 130)
DOOR_SIGNALS = {
  'Door_Open_Switch_Status_LS': ('DrDoorOpenSwAct', 'PsDoorOpenSwAct'),
  'DriverDoorStatus': ('DriverDoorOpened',),
  'BCMDoorBeltStatus': ('FrontLeftDoor', 'FrontRightDoor', 'RearLeftDoor', 'RearRightDoor'),
}


def read_override(paths=OVERRIDE_PATHS, now=None):
  now = time.time() if now is None else now
  for path in paths:
    try:
      with path.open('rb') as stream:
        raw = stream.read(16385)
      if not raw.strip():
        continue
      if len(raw) > 16384:
        return None
      value = json.loads(raw)
      if not isinstance(value, dict) or value.get('mode') not in ('manual', 'auto', 'profile'):
        return None
      # Wayon profiles are durable configuration; manual overrides expire.
      if value.get('expiresAt') and value['mode'] != 'profile':
        expiry = datetime.fromisoformat(str(value['expiresAt']).replace('Z', '+00:00'))
        if expiry.tzinfo is None or now >= expiry.timestamp():
          return None
      json.dumps(value, allow_nan=False)
      return value
    except OSError:
      continue
    except (ValueError, TypeError, OverflowError, RecursionError):
      return None
  return None


def road_state(device, pandas):
  """None is unknown, never an instruction to sleep or start offroad timers."""
  if (device is not None and device.started) or (pandas is not None and any(p.ignitionLine or p.ignitionCan for p in pandas)):
    return True
  return False if device is not None and pandas is not None else None


def build_door_parsers():
  from opendbc.can.parser import CANParser
  specs = [('gm_global_a_lowspeed_1818125', 'Door_Open_Switch_Status_LS', WATCH_BUSES),
           ('gm_global_a_lowspeed', 'DriverDoorStatus', WATCH_BUSES),
           ('gm_global_a_powertrain_generated', 'BCMDoorBeltStatus', (0, 128))]
  return [CANParser(dbc, [(message, float('nan'))], bus) for dbc, message, buses in specs for bus in buses]


def updated_door_state(parsers, frames):
  result = None
  for _, group in groupby(sorted(frames, key=lambda event: event[0]), key=lambda event: event[0]):
    batch = list(group)
    updated = False
    door_open = False
    for parser in parsers:
      parser.update(batch)
      # Aggregate simultaneous signals, not an old open and a newer close
      # collected in one drain. Quiet buses have no authority over a new edge.
      for message, signals in DOOR_SIGNALS.items():
        samples = parser.vl_all.get(message, {})
        if any(samples.get(signal) for signal in signals):
          updated = True
          door_open |= any(bool(samples[signal][-1]) for signal in signals if samples.get(signal))
    if updated:
      result = door_open
  return result


class DoorMonitor:
  """Keep offroad CAN reception independent of slow USB/BT discovery."""
  def __init__(self, path=DOOR_PATH):
    self.path = path
    try:
      self.door_open = path.read_bytes().strip() == b'1'
      self.saved_value = self.door_open
    except OSError:
      self.door_open = False
      self.saved_value = None
    self.error = ''
    self.gm = False
    self.stop = threading.Event()
    self.thread = None

  def remember(self, value):
    value = bool(value)
    self.door_open = value
    if value == self.saved_value:
      return
    try:
      self.path.parent.mkdir(parents=True, exist_ok=True)
      temporary = self.path.with_suffix('.navdy-tmp')
      temporary.write_bytes(b'1' if value else b'0')
      temporary.replace(self.path)
      self.saved_value = value
    except OSError as error:
      self.error = str(error)[:160]

  def start(self):
    self.thread = threading.Thread(target=self._run, name='navdy-doors', daemon=True)
    self.thread.start()

  def close(self):
    self.stop.set()
    if self.thread is not None:
      self.thread.join(timeout=2)

  def _run(self):
    while not self.stop.is_set():
      try:
        self._listen()
      except Exception as error:
        self.error = f'{type(error).__name__}: {error}'[:160]
        self.stop.wait(1)

  def _listen(self):
    from openpilot.cereal import car, messaging
    from openpilot.common.params import Params
    params = Params()
    sm = messaging.SubMaster(['deviceState', 'pandaStates', 'carState'])
    can = messaging.sub_sock('can', conflate=False)
    parsers = []
    checked = -10.0
    while not self.stop.wait(0.1):
      try:
        self.error = ''
        now = time.monotonic()
        if now - checked >= 5:
          checked = now
          raw = params.get('CarParamsPersistent')
          self.gm = False
          if raw:
            with car.CarParams.from_bytes(raw) as cp:
              self.gm = cp.brand == 'gm'
          if self.gm and not parsers:
            parsers = build_door_parsers()
        sm.update(0)
        now = time.monotonic()

        def recent(name, age):
          return sm[name] if (sm.seen[name] and sm.valid[name] and
                             0 <= now - sm.recv_time[name] < age and
                             0 <= now - sm.logMonoTime[name] / 1e9 < age) else None

        state = road_state(recent('deviceState', 1.5), recent('pandaStates', 1.5))
        live = recent('carState', 0.5)
        if state is True and live is not None and live.canValid and not live.canTimeout:
          self.remember(live.doorOpen)
        frames = []
        # Bounded draining also prevents an old door edge replay after a stall.
        for _ in range(512):
          event = messaging.recv_one_or_none(can)
          if event is None:
            break
          age = time.monotonic() - event.logMonoTime / 1e9
          if self.gm and state is False and event.valid and 0 <= age < 1.0:
            frames.append((event.logMonoTime, [(m.address, bytes(m.dat), m.src) for m in event.can]))
        if frames and parsers:
          value = updated_door_state(parsers, frames)
          if value is not None:
            self.remember(value)
      except Exception as error:
        # Ambient input must not take down telemetry or navigation forwarding.
        self.error = f'{type(error).__name__}: {error}'[:160]
        self.stop.wait(1)
