import pytest

from openpilot.selfdrive.car.stock_lane_markings import (
  STOCK_LANE_MAX_AGE_NS, TRAVERSE_LANE_ADDRESS, TRAVERSE_LANE_BUS,
  UNKNOWN_LANES, TraverseStockLaneMarkings, lane_boottime_ns, stock_lane_codes, stock_lane_sample_fresh,
)


def frame(left, right, *, bus=TRAVERSE_LANE_BUS, address=TRAVERSE_LANE_ADDRESS):
  return address, bytes([0, 0, 0, 0, 0, 0, 0xC0 | left << 3 | right, 0]), bus


@pytest.mark.parametrize("left,right,expected", [
  (1, 2, (1, 0)), (2, 1, (0, 1)), (0, 1, (-1, 1)), (2, 0, (0, -1)),
  (3, 7, UNKNOWN_LANES), (0, 0, UNKNOWN_LANES),
])
def test_stock_markings_decode_each_side(left, right, expected):
  lane = TraverseStockLaneMarkings()
  assert lane.update([(10, [frame(left, right)])], 10) == expected


def test_freshness_is_independent_of_other_can_frames():
  lane = TraverseStockLaneMarkings()
  now = 1_000_000_000
  assert lane.update([], now) == UNKNOWN_LANES
  assert lane.update([(now, [frame(1, 2)])], now) == (1, 0)
  expired = now + STOCK_LANE_MAX_AGE_NS + 1
  assert lane.update([], expired - 1) == (1, 0)
  bad_length = TRAVERSE_LANE_ADDRESS, bytes(7), TRAVERSE_LANE_BUS
  wrong = [frame(2, 1, bus=0), frame(2, 1, bus=129), frame(2, 1, address=0x354), bad_length]
  assert lane.update([(expired, wrong)], expired) == UNKNOWN_LANES
  assert lane.update([(expired, [frame(2, 1)])], expired) == (0, 1)


def test_batch_rejects_stale_reordered_and_future_frames():
  lane = TraverseStockLaneMarkings()
  now = 1_000_000_000
  assert lane.update([(now - STOCK_LANE_MAX_AGE_NS - 1, [frame(2, 1)])], now) == UNKNOWN_LANES
  assert lane.update([(now, [frame(1, 2)]), (now - 1, [frame(2, 1)]), (now + 1, [frame(2, 1)])], now) == (1, 0)
  assert lane.update([(now + 2, [frame(2, 1), frame(0, 0)])], now + 2) == UNKNOWN_LANES


def test_publisher_rebases_recorded_sample_age_without_renewing_it():
  lane = TraverseStockLaneMarkings()
  sample_ns = 10_000_000_000
  live_ns = 900_000_000_000
  assert lane.update([(sample_ns, [frame(1, 2)])], sample_ns + 80_000_000) == (1, 0)
  assert lane.publisher_sample_ns(sample_ns + 80_000_000, live_ns) == live_ns - 80_000_000
  assert lane.publisher_sample_ns(sample_ns + STOCK_LANE_MAX_AGE_NS + 1, live_ns) == 0


def test_receive_clock_matches_pandad_boot_clock(monkeypatch):
  from openpilot.selfdrive.car import stock_lane_markings

  monkeypatch.setattr(stock_lane_markings.time, "CLOCK_BOOTTIME", 42, raising=False)
  monkeypatch.setattr(stock_lane_markings.time, "clock_gettime_ns", lambda clock: 900_000 if clock == 42 else 100_000)
  assert lane_boottime_ns() == 900_000


def test_optional_capnp_field_preserves_legacy_and_control_lane_values():
  from opendbc.car import structs

  state = structs.CarState.new_message(leftLaneLine=24, rightLaneLine=10)
  assert stock_lane_codes(state) is None
  state.stockLaneMarkings.supported = True
  assert stock_lane_codes(state) == UNKNOWN_LANES
  state.stockLaneMarkings.left, state.stockLaneMarkings.right = 0, 1
  assert not stock_lane_sample_fresh(state, 1)
  state.stockLaneMarkings.sampleMonoTime = 10
  with structs.CarState.from_bytes(state.to_bytes()) as decoded:
    assert stock_lane_codes(decoded) == (0, 1)
    assert (decoded.leftLaneLine, decoded.rightLaneLine) == (24, 10)
    assert not stock_lane_sample_fresh(decoded, 9)
    assert stock_lane_sample_fresh(decoded, 10 + STOCK_LANE_MAX_AGE_NS)
    assert not stock_lane_sample_fresh(decoded, 11 + STOCK_LANE_MAX_AGE_NS)
