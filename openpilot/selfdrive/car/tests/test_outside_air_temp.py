from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.outside_air_temp import (
  OAT_MAX_AGE_NS, TRAVERSE_OAT_ADDRESS, TRAVERSE_OAT_BUS,
  TraverseOutsideAirTemp, decode_outside_air_temp, outside_air_temp_c,
)


def frame(raw, *, mirror=None, bus=TRAVERSE_OAT_BUS, address=TRAVERSE_OAT_ADDRESS, length=8):
  data = bytearray(length)
  if length >= 8:
    data[3] = (raw + 3) & 0xFF  # unfiltered sensor value differs from the displayed one
    data[4] = raw
    data[7] = raw if mirror is None else mirror
  return address, bytes(data), bus


@pytest.mark.parametrize("raw,expected", [(110, 15.0), (108, 14.0), (0, -40.0), (89, 4.5), (250, 85.0)])
def test_decode_uses_displayed_byte(raw, expected):
  assert decode_outside_air_temp(frame(raw)[1]) == expected


@pytest.mark.parametrize("data", [frame(110, mirror=111)[1], frame(251)[1], frame(255)[1], bytes(7)])
def test_decode_rejects_mismatch_out_of_range_and_short(data):
  assert decode_outside_air_temp(data) is None


def test_only_traverse_bus0_address_is_used():
  oat = TraverseOutsideAirTemp()
  now = 1_000_000_000
  wrong = [frame(110, bus=1), frame(110, bus=128), frame(110, address=0x3F9), frame(110, length=7)]
  assert oat.update([(now, wrong)], now) is None
  assert oat.update([(now, [frame(110)])], now) == 15.0


def test_freshness_and_stale_reordered_future_frames():
  oat = TraverseOutsideAirTemp()
  now = 10_000_000_000
  assert oat.update([(now - OAT_MAX_AGE_NS - 1, [frame(100)])], now) is None
  assert oat.update([(now, [frame(110)]), (now - 1, [frame(100)]), (now + 1, [frame(90)])], now) == 15.0
  assert oat.update([], now + OAT_MAX_AGE_NS) == 15.0
  assert oat.update([], now + OAT_MAX_AGE_NS + 1) is None
  later = now + OAT_MAX_AGE_NS + 2
  assert oat.update([(later, [frame(109)])], later) == 14.5


def test_invalid_frame_keeps_last_good_value_until_it_expires():
  oat = TraverseOutsideAirTemp()
  now = 1_000_000_000
  assert oat.update([(now, [frame(110)])], now) == 15.0
  assert oat.update([(now + 1, [frame(110, mirror=0)])], now + 1) == 15.0
  assert oat.update([], now + OAT_MAX_AGE_NS + 1) is None


@pytest.mark.parametrize("state,expected", [
  (NS(), None),
  (NS(outsideAirTemp=NS(valid=False, celsius=15.0)), None),
  (NS(outsideAirTemp=NS(valid=True, celsius=15.0)), 15.0),
  (NS(outsideAirTemp=NS(valid=True, celsius=float("nan"))), None),
  (NS(outsideAirTemp=NS(valid=True, celsius=120.0)), None),
])
def test_car_state_reader(state, expected):
  assert outside_air_temp_c(state) == expected


def test_capnp_field_round_trip():
  from opendbc.car import structs
  cs = structs.CarState.new_message()
  assert cs.outsideAirTemp.valid is False
  cs.outsideAirTemp.valid = True
  cs.outsideAirTemp.celsius = 14.5
  with structs.CarState.from_bytes(cs.to_bytes()) as copy:
    assert outside_air_temp_c(copy) == 14.5
