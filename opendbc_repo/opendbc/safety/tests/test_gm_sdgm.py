"""Compile the production C hooks; no USB, CAN device, or vehicle I/O."""
import ctypes
import os
from pathlib import Path
import subprocess

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm import gmcan
from opendbc.car.gm.values import CAR, DBC, GMSafetyFlags

CAM_LONG = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG)
SDGM_LONG = CAM_LONG | int(GMSafetyFlags.HW_SDGM)


@pytest.fixture(scope='module')
def native_safety(tmp_path_factory):
  root = Path(__file__).resolve().parents[1]
  output = tmp_path_factory.mktemp('gm-safety') / 'libsafety.so'
  subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2', '-Wall', '-Werror',
                  '-Wno-pointer-to-int-cast', '-fno-builtin', '-std=gnu11',
                  '-I', str(root), '-I', str(root / 'board'),
                  str(root / 'tests/libsafety/gm_sdgm.c'), '-o', str(output)], check=True, capture_output=True)
  lib = ctypes.CDLL(str(output))
  lib.set_safety_hooks.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
  lib.set_safety_hooks.restype = ctypes.c_int
  lib.set_controls_allowed.argtypes = [ctypes.c_bool]
  lib.get_controls_allowed.restype = ctypes.c_bool
  lib.set_relay_malfunction.argtypes = [ctypes.c_bool]
  lib.get_relay_malfunction.restype = ctypes.c_bool
  lib.set_alternative_experience.argtypes = [ctypes.c_int]
  lib.gm_test_frame.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]
  lib.gm_test_frame.restype = ctypes.c_int
  return lib


@pytest.fixture
def safety(native_safety):
  assert native_safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, SDGM_LONG) == 0
  native_safety.init_tests()
  return native_safety


def frame(lib, operation, message):
  address, data, bus = message
  return lib.gm_test_frame(operation, address, bus, data, len(data))


def brake(amount, bus=2, length=5):
  raw = (-amount) & 0xfff
  return 0x315, (bytes([0xa0 | (raw >> 8), raw & 0xff]) + bytes(6))[:length], bus


def gas(amount, active=True):
  return gmcan.create_gas_regen_command(CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt]), 0, amount, 0, active, False)


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('bus', [0, 1, 2, 3])
def test_all_brake_values_and_buses(safety, enabled, bus):
  safety.set_controls_allowed(enabled)
  for amount in range(4096):
    allowed = bus == 2 and (amount == 0 or enabled and amount <= 400)
    assert bool(frame(safety, 1, brake(amount, bus))) == allowed, (enabled, bus, amount)


@pytest.mark.parametrize('length', [0, 1, 2, 3, 4, 6, 7, 8])
def test_wrong_brake_length_rejected(safety, length):
  safety.set_controls_allowed(True)
  message = brake(100, length=length)
  assert len(message[1]) == length
  assert not frame(safety, 1, message)


@pytest.mark.parametrize('amount', [-541, -540, -500, 0, 1346, 1347])
@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('enabled', [False, True])
def test_gas_bounds_and_no_tx_permission_grant(safety, amount, active, enabled):
  safety.set_controls_allowed(enabled)
  expected = (enabled or not active) and (amount == -500 or enabled and -540 <= amount <= 1346)
  assert bool(frame(safety, 1, gas(amount, active))) == expected
  assert safety.get_controls_allowed() == enabled
  assert bool(frame(safety, 1, brake(100))) == enabled


@pytest.mark.parametrize('cause', ['brake', 'gas', 'cancel', 'relay'])
def test_disengagement_survives_late_commands(safety, cause):
  # Engage through the actual RX button path, never a TX enable request.
  buttons = CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt])
  assert frame(safety, 0, gmcan.create_buttons(buttons, 0, 0, 2))
  assert safety.get_controls_allowed()
  if cause == 'brake':
    assert frame(safety, 0, (0xC9, bytes([0, 0, 0, 0x20, 0, 1, 0, 0]), 0))
  elif cause == 'gas':
    assert frame(safety, 0, (0x1C4, bytes([0, 0, 0, 0, 0, 1, 0, 0]), 0))
  elif cause == 'cancel':
    assert frame(safety, 0, gmcan.create_buttons(buttons, 0, 1, 6))
  else:
    safety.set_relay_malfunction(True)
  if cause != 'relay':
    assert not safety.get_controls_allowed()
  assert not frame(safety, 1, gas(0))
  assert not frame(safety, 1, brake(100))
  if cause != 'relay':
    assert not safety.get_controls_allowed()


@pytest.mark.parametrize('bus', [1, 2, 3])
def test_rejected_gas_bus_cannot_grant_permission(safety, bus):
  address, data, _ = gas(0)
  assert not frame(safety, 1, (address, data, bus))
  assert not safety.get_controls_allowed()
  assert not frame(safety, 1, brake(100))


@pytest.mark.parametrize('length', range(8))
def test_rejected_gas_length_cannot_grant_permission(safety, length):
  address, data, bus = gas(0)
  assert not frame(safety, 1, (address, data[:length], bus))
  assert not safety.get_controls_allowed()
  assert not frame(safety, 1, brake(100))


@pytest.mark.parametrize('address,length', [(0x180, 4), (0x2CB, 8)])
def test_stock_ecu_detection_still_blocks_output(safety, address, length):
  safety.set_controls_allowed(True)
  assert frame(safety, 0, (address, bytes(length), 0))
  assert safety.get_relay_malfunction()
  assert not frame(safety, 1, brake(100))
  assert not frame(safety, 1, gas(0))


def test_gas_override_still_blocks_longitudinal_output(safety):
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS)
  safety.set_controls_allowed(True)
  assert frame(safety, 0, (0x1C4, bytes([0, 0, 0, 0, 0, 1, 0, 0]), 0))
  assert safety.get_controls_allowed()
  assert not frame(safety, 1, brake(100))
  assert not frame(safety, 1, gas(0))
  assert frame(safety, 1, brake(0))
  assert frame(safety, 1, gas(-500, False))


@pytest.mark.parametrize('parameter,bus', [(0, 2), (1, None), (CAM_LONG, 0), (SDGM_LONG, 2),
                                        (512, 2), (513, None), (5, None), (519, None)])
def test_mode_switch_resets_routing(safety, parameter, bus):
  for previous in (5, CAM_LONG, SDGM_LONG):
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, previous) == 0
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, parameter) == 0
    safety.set_controls_allowed(True)
    for candidate_bus in range(3):
      assert bool(frame(safety, 1, brake(100, candidate_bus))) == (candidate_bus == bus)


@pytest.mark.parametrize('address', [0x180, 0x315, 0x2CB, 0x370, 0x184, 0x100])
def test_forwarding_matches_existing_camera_long(safety, address):
  for bus in (0, 1, 2):
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, CAM_LONG) == 0
    previous = frame(safety, 2, (address, bytes(8), bus))
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, SDGM_LONG) == 0
    assert frame(safety, 2, (address, bytes(8), bus)) == previous


def test_new_allowlist_only_moves_brake(safety):
  allowed = [(0x180, 0, 4), (0x2CB, 0, 8), (0x370, 0, 6), (0x200, 0, 6), (0x1E1, 0, 7), (0x184, 2, 8)]
  for address, bus, length in allowed:
    data = gas(-500, False)[1] if address == 0x2CB else bytes(length)
    if address == 0x1E1:
      data = gmcan.create_buttons(CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt]), 0, 0, 1)[1]
    safety.set_controls_allowed(True)
    assert frame(safety, 1, (address, data, bus))
    assert not frame(safety, 1, (address, data, 1))
  assert not frame(safety, 1, (0x123, bytes(8), 0))
