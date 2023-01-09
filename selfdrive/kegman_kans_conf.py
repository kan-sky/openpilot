import json
import os

class kegman_kans_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['tuneGernby'] != "1":
      self.conf['tuneGernby'] = str(1)
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/kegman_kans.json'):
      with open('/data/kegman_kans.json', 'r') as f:
        self.config = json.load(f)

      if "battPercOff" not in self.config:
        self.config.update({"battPercOff":"80"})
        self.config.update({"carVoltageMinEonShutdown":"12000"})
        self.element_updated = True

      if "tuneGernby" not in self.config:
        self.config.update({"tuneGernby":"1"})
        self.element_updated = True

      if "liveParams" not in self.config:
        self.config.update({"liveParams":"1"})
        self.element_updated = True


      if "nTune" not in self.config:
        self.config.update({"nTune":"1"})
        self.element_updated = True

      if "steerLimitTimer" not in self.config:
        self.config.update({"steerLimitTimer":"5.0"})
        self.element_updated = True

      if "CruiseDelta" not in self.config:
        self.config.update({"CruiseDelta":"5"})
        self.element_updated = True

      if "CruiseEnableMin" not in self.config:
        self.config.update({"CruiseEnableMin":"10"})
        self.element_updated = True

      if "epsModded" not in self.config:
        self.config.update({"epsModded":"0"})
        self.element_updated = True

      if "CAMERA_SPEED_FACTOR" not in self.config:
        self.config.update({"CAMERA_SPEED_FACTOR":"0.99"})
        self.element_updated = True

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"battChargeMin":"65", "battChargeMax":"85", "wheelTouchSeconds":"18000", \
                     "battPercOff":"55", "carVoltageMinEonShutdown":"12000", \
                     "brakeStoppingTarget":"0.65", "tuneGernby":"1", "AutoHold":"1", "steerLimitTimer":"5.0", \
                     "STOPPING_DISTANCE":"2.0", "CruiseDelta":"5", \
                     "CruiseEnableMin":"10", "epsModded": "0", "CAMERA_SPEED_FACTOR":"0.99"}


      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/kegman_kans.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman_kans.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kegman_kans.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman_kans.json", 0o764)