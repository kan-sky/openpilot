import os
import fcntl
import signal
import json
import weakref
from enum import Enum

class LateralType(Enum):
  NONE = 0
  TORQUE = 1

class GroupType:
  NONE = "none"

  TORQUE = "torque"

CONF_PATH = '/data/tuning/'

CONF_LAT_TORQUE_FILE = '/data/tuning/'+GroupType.TORQUE+'.json'

torq_tunes = {}

def file_watch_handler(signum, frame):
  global torq_tunes
  for torq_tune in torq_tunes.values():
    torq_tune.handle()


class torqTune():

  def get_ctrl(self):
    return self.ctrl() if self.ctrl is not None else None

  def __del__(self):
    if self.group is None:
      torq_tunes[self.key] = None
    print('__del__', self)

  def __init__(self, CP=None, ctrl=None, group=None):

    self.invalidated = False
    self.CP = CP
    self.ctrl = weakref.ref(ctrl) if ctrl is not None else None
    self.type = LatType.NONE
    self.group = GroupType.NONE if group is None else group
    self.config = {}
    self.key = str(self)
    self.disable_lateral_live_tuning = False

    if "LatControlTorque" in str(type(ctrl)):
      self.type = LatType.TORQUE
      self.file = CONF_LAT_TORQUE_FILE
    else:
      self.file = CONF_PATH + group + ".json"

    if not os.path.exists(CONF_PATH):
      os.makedirs(CONF_PATH)

    self.read()

    if self.group == GroupType.NONE:
      torq_tunes[self.key] = self

    try:
      signal.signal(signal.SIGIO, file_watch_handler)
      fd = os.open(CONF_PATH, os.O_RDONLY)
      fcntl.fcntl(fd, fcntl.F_SETSIG, 0)
      fcntl.fcntl(fd, fcntl.F_NOTIFY, fcntl.DN_MODIFY | fcntl.DN_CREATE | fcntl.DN_MULTISHOT)
    except Exception as ex:
      print("exception", ex)
      pass

  def handle(self):
    try:
      if os.path.getsize(self.file) > 0:
        with open(self.file, 'r') as f:
          self.config = json.load(f)

        if self.checkValid():
          self.write_config(self.config)

        self.invalidated = True

    except:
      pass

  def check(self):
    if self.invalidated:
      self.invalidated = False
      self.update()

  def read(self):
    success = False
    try:
      if os.path.getsize(self.file) > 0:
        with open(self.file, 'r') as f:
          self.config = json.load(f)

        if self.checkValid():
          self.write_config(self.config)

        self.update()
        success = True
    except:
      pass

    if not success:
      try:
        self.write_default()
        with open(self.file, 'r') as f:
          self.config = json.load(f)
          if self.checkValid():
            self.write_config(self.config)
          self.update()
      except:
        pass

  def checkValue(self, key, min_, max_, default_):
    updated = False

    if key not in self.config:
      self.config.update({key: default_})
      updated = True
    elif min_ > self.config[key]:
      self.config.update({key: min_})
      updated = True
    elif max_ < self.config[key]:
      self.config.update({key: max_})
      updated = True

    return updated

  def checkValid(self):

    if self.type == LatType.TORQUE or self.group == GroupType.TORQUE:
      return self.checkValidTorque()

    return False

  def update(self):

    if self.disable_lateral_live_tuning:
      return

    if self.type == LatType.TORQUE:
      self.updateTorque()


  def checkValidTorque(self):
    updated = False

    if self.checkValue("useSteeringAngle", 0., 1., 1.):
      updated = True
    if self.checkValue("latAccelFactor", 0.5, 4.5, 2.7):
      updated = True
    if self.checkValue("friction", 0.0, 0.2, 0.08):
      updated = True
    if self.checkValue("angle_deadzone_v2", 0.0, 2.0, 0.0):
      updated = True

    return updated

  def updateTorque(self):
    torque = self.get_ctrl()
    if torque is not None:
      torque.use_steering_angle = float(self.config["useSteeringAngle"]) > 0.5
      torque.steering_angle_deadzone_deg = float(self.config["angle_deadzone_v2"])
      torque.torque_params.latAccelFactor = float(self.config["latAccelFactor"])
      torque.torque_params.friction = float(self.config["friction"])

  def read_cp(self):

    try:
      if self.CP is not None:
        if self.type == LatType.TORQUE:
          self.config["useSteeringAngle"] = 1. if self.CP.lateralTuning.torque.useSteeringAngle else 0.
          self.config["latAccelFactor"] = self.CP.lateralTuning.torque.latAccelFactor
          self.config["friction"] = round(self.CP.lateralTuning.torque.friction, 3)
          self.config["angle_deadzone_v2"] = round(self.CP.lateralTuning.torque.steeringAngleDeadzoneDeg, 1)

    except:
      pass

  def write_default(self):

    try:
      self.read_cp()
      self.checkValid()
      self.write_config(self.config)
    except:
      pass

  def write_config(self, conf):
    try:
      with open(self.file, 'w') as f:
        json.dump(conf, f, indent=2, sort_keys=False)
        os.chmod(self.file, 0o666)
    except IOError:

      try:
        if not os.path.exists(CONF_PATH):
          os.makedirs(CONF_PATH)

        with open(self.file, 'w') as f:
          json.dump(conf, f, indent=2, sort_keys=False)
          os.chmod(self.file, 0o666)
      except:
        pass


def torq_tune_get(group, key):
  global torq_tunes
  if group not in torq_tunes:
    torq_tunes[group] = torqTune(group=group)

  torqtune = torq_tunes[group]

  if torqtune.config == None or key not in torqtune.config:
    torqtune.read()

  v = torqtune.config[key]

  if v is None:
    torqtune.read()
    v = torqtune.config[key]

  return v


def torque_get(key):
  return torq_tune_get(GroupType.TORQUE, key)

def tuning_init():
  group_values = [value for key, value in GroupType.__dict__.items() if not key.startswith('__') and not callable(value)]
  for group in group_values:
    torqTune(group=group)

if __name__ == "__main__":
  tuning_init()