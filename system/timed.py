#!/usr/bin/env python3
import datetime
import subprocess
import time
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.time import system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def set_time(new_time):
  diff = datetime.datetime.now() - new_time
  if diff < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    print(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  print(f"Setting time to {new_time}, diff={diff}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


def main() -> NoReturn:
  """
    timed has one responsibility:
    - getting the current time

    GPS directly gives time.
    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue

    # set time
    # TODO: account for unixTimesatmpMillis being a (usually short) time in the past
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000.)
    set_time(gps_time)

    time.sleep(10)

if __name__ == "__main__":
  main()
