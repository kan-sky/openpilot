"""Real GM encoder/controller comparison; never opens CAN/IPC/vehicle Params."""
import pytest

from openpilot.common import fcw_warning_test
from opendbc.car import structs
from opendbc.car.gm.values import CAR, DBC
from opendbc.car.gm.tests.test_display_speed import runtime, build_cp, controller_state  # noqa: F401


@pytest.mark.parametrize("real_fcw", [False, True])
@pytest.mark.parametrize("moving", [False, True], ids=["parked", "moving"])
@pytest.mark.parametrize("finish", ["timeout", "cancel", "engage", "lateral", "longitudinal", "can_timeout"])
def test_only_dashboard_warning_bits_change(runtime, monkeypatch, tmp_path, real_fcw, moving, finish):
  clock = [100.]
  monkeypatch.setattr(fcw_warning_test.time, "monotonic", lambda: clock[0])
  helper = fcw_warning_test.FcwWarningTest(tmp_path)
  monkeypatch.setattr(runtime.controller, "FcwWarningTest", lambda: helper)
  cp = build_cp(runtime)
  tested = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  baseline = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  baseline.fcw_warning_test = None
  cs = controller_state()
  cs.out = structs.CarState(canValid=True, gearShifter="drive" if moving else "park",
                           parkingBrake=not moving, standstill=not moving,
                           vEgo=54.5 / 3.6 if moving else 0.)
  cc = structs.CarControl()
  cc.hudControl.visualAlert = "fcw" if real_fcw else "none"
  changed_frames = 0
  for tick in range(400):
    clock[0] = 100 + tick * .01
    if tick == 70:
      fcw_warning_test.request_test(1, tmp_path, clock[0])
    if tick == 120:
      if finish == "cancel":
        fcw_warning_test.request_test(0, tmp_path, clock[0])
      elif finish == "engage":
        cc.enabled = True
      elif finish == "lateral":
        cc.latActive = True
      elif finish == "longitudinal":
        cc.longActive = True
      elif finish == "can_timeout":
        cs.out.canTimeout = True
    now = int(clock[0] * 1e9)
    output, sends = tested.update(cc.as_reader(), cs, now)
    old_output, old_sends = baseline.update(cc.as_reader(), cs, now)
    assert output.to_dict() == old_output.to_dict()
    if tick >= 126 and finish != "timeout":
      assert not helper.until
    assert len(sends) == len(old_sends)
    for (address, data, bus), (old_address, old_data, old_bus) in zip(sends, old_sends, strict=True):
      assert (address, bus) == (old_address, old_bus)
      if address == 0x370 and helper.until > clock[0] and not real_fcw:
        assert bus == 0
        assert data[:5] == old_data[:5]
        assert data[5] & ~3 == old_data[5] & ~3
        assert data[5] & 3 == 3 and old_data[5] & 3 == 0
        changed_frames += 1
      else:
        assert data == old_data
  assert changed_frames > 0 if not real_fcw else changed_frames == 0


@pytest.mark.parametrize("candidate,alpha", [
  (CAR.CHEVROLET_TRAVERSE, False), (CAR.CHEVROLET_BOLT_EUV, True), (CAR.CHEVROLET_VOLT, True),
])
def test_unsupported_configuration_does_not_create_test(runtime, monkeypatch, candidate, alpha):
  def fail():
    raise AssertionError("unsupported vehicle must not create a warning test")
  monkeypatch.setattr(runtime.controller, "FcwWarningTest", fail)
  cp = build_cp(runtime, candidate, alpha)
  controller = runtime.controller.CarController(DBC[candidate], cp)
  assert controller.fcw_warning_test is None
