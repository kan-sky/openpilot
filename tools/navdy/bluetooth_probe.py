#!/usr/bin/env python3
"""Read-only native Bluetooth preflight. Never scans, pairs, or changes the OS."""

import gzip
import json
from pathlib import Path


CONFIG_KEYS = ("CONFIG_BT", "CONFIG_BT_BREDR", "CONFIG_BT_RFCOMM",
               "CONFIG_BT_LE", "CONFIG_BT_HCIUART", "CONFIG_BT_HCIUART_QCA",
               "CONFIG_MSM_BT_POWER")


def read_text(path):
  try:
    return path.read_text().replace("\0", "").strip()
  except (OSError, UnicodeError):
    return None


def parse_config(text):
  values = {}
  for line in text.splitlines():
    if line.startswith("# CONFIG_") and line.endswith(" is not set"):
      values[line[2:-11]] = "n"
    elif line.startswith("CONFIG_") and "=" in line:
      key, value = line.split("=", 1)
      values[key] = value
  return {key: values.get(key) for key in CONFIG_KEYS}


def probe(root=Path("/")):
  config_error = None
  try:
    with gzip.open(root / "proc/config.gz", "rt") as stream:
      config = parse_config(stream.read(2 * 1024 * 1024))
  except (OSError, EOFError, UnicodeError) as error:
    config = {key: None for key in CONFIG_KEYS}
    config_error = type(error).__name__

  def exists(path):
    return (root / path).exists()

  def installed(name):
    return any(exists(f"{directory}/{name}") for directory in ("usr/bin", "usr/sbin", "bin", "sbin"))

  checks = {
    "gps_uart_present": exists("dev/ttyHS0"),
    "bluetooth_uart_present": exists("dev/ttyHS1"),
    "bluetooth_power_device_present": exists("dev/btpower"),
    "bluetoothctl_installed": installed("bluetoothctl"),
    "btattach_installed": installed("btattach"),
    "radio_service_installed": any(exists(f"{directory}/carrot-bluetooth-radio.service") for directory in (
      "etc/systemd/system", "lib/systemd/system", "usr/lib/systemd/system")),
    "radio_opted_in": exists("data/bluetooth/ENABLED"),
  }
  missing = [key for key, value in config.items() if value not in ("y", "m")]
  missing += [key for key, value in checks.items() if not value and key != "radio_opted_in"]
  controllers = sorted(path.name for path in (root / "sys/class/bluetooth").glob("hci*"))
  status = "missing_prerequisites" if missing else "radio_not_initialized"
  if not missing and controllers:
    status = "controller_present_unverified"

  return {
    "schema_version": 1,
    "read_only": True,
    "agnos_version": read_text(root / "VERSION"),
    "board": read_text(root / "sys/firmware/devicetree/base/model"),
    "kernel_config": config,
    "kernel_config_error": config_error,
    "checks": checks,
    "controllers": controllers,
    "status": status,
    "missing_or_unknown_prerequisites": missing,
    "observed_params": {name: read_text(root / "data/params/d" / name) for name in ("IsOnroad", "IsEngaged")},
    "navdy_link_verified": False,
    "phone_ambient_concurrency_verified": False,
    "remaining_runtime_checks": ["controller_power_and_capabilities", "explicit_target_pairing",
                                 "dedicated_navdy_service", "phone_and_ambient_concurrency",
                                 "disconnect_reconnect_and_stale_data"],
  }


if __name__ == "__main__":
  print(json.dumps(probe(), indent=2))
