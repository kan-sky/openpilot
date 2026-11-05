from types import SimpleNamespace as NS

import pytest

from openpilot.common import fcw_warning_test as test


def states(moving=False):
  return (NS(canValid=True, canTimeout=False, gearShifter="drive" if moving else "park",
             parkingBrake=not moving, standstill=not moving,
             vEgo=54.5 / 3.6 if moving else 0., gasPressed=False, cruiseState=NS(enabled=False)),
          NS(enabled=False, latActive=False, longActive=False))


def warm(controller, cs, cc, start=100.):
  for tick in range(61):
    assert not controller.update(cs, cc, start + tick * .01)
  return start + .6


@pytest.fixture(params=[False, True], ids=["parked", "moving"])
def ready(tmp_path, request):
  controller = test.FcwWarningTest(tmp_path)
  cs, cc = states(request.param)
  now = warm(controller, cs, cc)
  assert test.current_status(tmp_path, now)["ready"]
  return controller, cs, cc, now


def begin(ready):
  controller, cs, cc, now = ready
  test.request_test(1, controller.directory, now)
  for tick in range(1, 10):
    if controller.update(cs, cc, now + tick * .01):
      return now + tick * .01
  raise AssertionError("test request never acknowledged")


def test_two_second_limit_cooldown_and_no_persistent_replay(ready):
  controller, cs, cc, now = ready
  begin(ready)
  for tick in range(10, 720):
    at = now + tick * .01
    assert controller.update(cs, cc, at) == (at < now + test.DURATION)
    status = test.current_status(controller.directory, at)
    if test.DURATION <= at - now < test.DURATION + test.COOLDOWN:
      assert not status["ready"]
  assert test.current_status(controller.directory, now + 7.19)["ready"]
  assert not test.current_status(controller.directory, now + 7.19)["active"]


@pytest.mark.parametrize("field,value", [
  ("canValid", False), ("canTimeout", True), ("gearShifter", "reverse"), ("gearShifter", "unknown"),
  ("gearShifter", "neutral"), ("vEgo", -.02),
  ("vEgo", float("nan")), ("vEgo", float("inf")), ("gasPressed", True),
])
def test_every_vehicle_guard_cancels_immediately_and_does_not_resume(ready, field, value):
  controller, cs, cc, now = ready
  start = begin(ready)
  previous = getattr(cs, field)
  setattr(cs, field, value)
  assert not controller.update(cs, cc, start + .01)
  setattr(cs, field, previous)
  warm(controller, cs, cc, start + .02)
  assert not test.current_status(controller.directory, start + .62)["active"]


@pytest.mark.parametrize("field", ["enabled", "latActive", "longActive"])
def test_engagement_cancels_test(ready, field):
  controller, cs, cc, _ = ready
  start = begin(ready)
  setattr(cc, field, True)
  assert not controller.update(cs, cc, start + .01)


def test_stock_acc_blocks_test(ready):
  controller, cs, cc, _ = ready
  start = begin(ready)
  cs.cruiseState.enabled = True
  assert not controller.update(cs, cc, start + .01)


def test_cancel_zero(ready):
  controller, cs, cc, _ = ready
  start = begin(ready)
  test.request_test(0, controller.directory, start + .01)
  assert not controller.update(cs, cc, start + .06)


def test_restart_never_replays_request(ready):
  controller, cs, cc, now = ready
  begin(ready)
  restarted = test.FcwWarningTest(controller.directory)
  warm(restarted, cs, cc, now + .1)
  assert not test.current_status(controller.directory, now + .7)["active"]


@pytest.mark.parametrize("gap", [.151, 1., -1.])
def test_controller_gap_or_clock_reversal_aborts(ready, gap):
  controller, cs, cc, _ = ready
  start = begin(ready)
  assert not controller.update(cs, cc, start + gap)


@pytest.mark.parametrize("change", [
  {"issued": 99.}, {"issued": 102.}, {"until": 999999.}, {"until": float("nan")},
  {"until": "102"}, {"token": "old-session"}, {"issued": True},
])
def test_stale_future_malformed_and_old_session_requests_never_fire(ready, change):
  controller, cs, cc, now = ready
  command = {"token": controller.token, "value": 1, "issued": now, "until": now + 2}
  command.update(change)
  # Malformed external data, including nonstandard JSON NaN.
  import json
  (controller.directory / "request.json").write_text(json.dumps(command))
  for tick in range(1, 21):
    assert not controller.update(cs, cc, now + .01 * tick)


def test_disconnected_controller_cannot_be_armed(ready):
  controller, _, _, now = ready
  with pytest.raises(ValueError):
    test.request_test(1, controller.directory, now + 1)
  test.request_test(0, controller.directory, now + 1)


def test_repeated_on_never_extends_pulse(ready):
  controller, cs, cc, now = ready
  begin(ready)
  with pytest.raises(ValueError):
    test.request_test(1, controller.directory, now + .05)
  # Even another local writer cannot extend the active lease with the same token.
  test.write_json(controller.directory / "request.json", {
    "token": controller.token, "value": 1, "issued": now + .1, "until": now + 2.1})
  for tick in range(10, 210):
    at = now + tick * .01
    assert controller.update(cs, cc, at) == (at < now + 2)


def test_io_failure_does_not_crash_controller_or_keep_warning(ready, monkeypatch):
  controller, cs, cc, _ = ready
  start = begin(ready)
  def fail(*args):
    raise OSError("test failure")
  monkeypatch.setattr(test, "write_json", fail)
  controller.next_status = 0
  assert not controller.update(cs, cc, start + .01)
  assert not controller.until


def test_missing_state_fails_closed():
  assert not test.manual_for_warning(NS(), NS())


@pytest.mark.parametrize("gear", ["drive", "low"])
def test_manual_motion_does_not_require_park_or_stationary(tmp_path, gear):
  controller = test.FcwWarningTest(tmp_path)
  cs, cc = states(moving=True)
  cs.gearShifter = gear
  now = warm(controller, cs, cc)
  assert test.current_status(tmp_path, now)["ready"]
  begin((controller, cs, cc, now))


def test_stop_start_motion_does_not_cancel_or_extend_test(ready):
  controller, cs, cc, now = ready
  start = begin(ready)
  for tick in range(1, 220):
    at = start + tick * .01
    cs.gearShifter = "drive"
    cs.parkingBrake = False
    cs.vEgo = 0. if tick < 30 else 10.
    cs.standstill = cs.vEgo == 0
    assert controller.update(cs, cc, at) == (at < now + test.DURATION)
