"""Traverse controller/interface and native safety agreement, with no CAN I/O."""
import pytest

pytest.importorskip('openpilot.common.params_pyx')

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.values import CAR, DBC, CanBus, GMSafetyFlags
from opendbc.safety.tests.test_gm_sdgm import frame, native_safety  # noqa: F401
from openpilot.selfdrive.car.tests.test_gm_radar_detection import build_cp, controller_state  # noqa: F401


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('candidate', list(CAR))
def test_sdgm_flag_is_exclusive_to_traverse_long(build_cp, candidate, alpha):
  if candidate == CAR.GMC_YUKON_CC:
    pytest.skip('Existing GMC_YUKON_CC torque parameters are missing')
  cp = build_cp(candidate, alpha=alpha)
  assert bool(cp.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_SDGM) == (candidate == CAR.CHEVROLET_TRAVERSE and alpha)


@pytest.mark.parametrize('candidate,bus', [(CAR.CHEVROLET_TRAVERSE, CanBus.CAMERA),
                                         (CAR.CHEVROLET_BOLT_EUV, CanBus.POWERTRAIN),
                                         (CAR.CHEVROLET_VOLT, CanBus.CHASSIS)])
@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('accel', [-3.27, -1.0, 0.0, 1.0])
def test_controller_brakes_match_native_panda(build_cp, native_safety, candidate, bus, active, accel):
  cp = build_cp(candidate)
  assert native_safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  native_safety.init_tests()
  native_safety.set_controls_allowed(active)
  controller = CarController(DBC[candidate], cp)
  state = controller_state()
  cc = structs.CarControl.new_message(enabled=active, longActive=active)
  cc.actuators.accel = accel
  cc.actuators.longControlState = 'pid' if active else 'off'
  brake_count = 0
  for tick in range(100):
    output, messages = controller.update(cc.as_reader(), state, 1_000_000_000 + tick * 10_000_000)
    for address, data, message_bus in messages:
      if address not in (0x315, 0x2CB):
        continue
      assert frame(native_safety, 1, (address, data, message_bus))
      assert native_safety.get_controls_allowed() == active
      if address == 0x315:
        assert message_bus == bus
        assert len(data) == 5
        amount = (-int.from_bytes(data[:2], 'big')) & 0xfff
        assert amount == output.brake
        assert 0 <= amount <= 400
        assert active or amount == 0
        counter = data[4] & 3
        assert counter == brake_count % 4
        assert (int.from_bytes(data[:2], 'big') + int.from_bytes(data[2:4], 'big') + counter) & 0xffff == 0
        brake_count += 1
  assert brake_count == 25
  if candidate == CAR.CHEVROLET_TRAVERSE and active and accel < 0:
    assert output.brake > 0


def test_stock_traverse_long_does_not_send_actuator_commands(build_cp):
  cp = build_cp(alpha=False)
  assert cp.safetyConfigs[0].safetyParam == GMSafetyFlags.HW_CAM
  controller = CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  for tick in range(100):
    _, messages = controller.update(structs.CarControl.new_message().as_reader(), controller_state(), 1_000_000_000 + tick * 10_000_000)
    assert not any(address in (0x315, 0x2CB) for address, _, _ in messages)


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('interceptor', [False, True])
def test_interceptor_and_alpha_use_same_bus_in_controller_and_safety(build_cp, native_safety, alpha, interceptor):
  fingerprint = gen_empty_fingerprint()
  if interceptor:
    fingerprint[0][0x201] = 6
  cp = build_cp(fingerprint=fingerprint, alpha=alpha)
  assert native_safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  native_safety.init_tests()
  native_safety.set_controls_allowed(True)
  controller = CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  cc = structs.CarControl.new_message(enabled=True, longActive=cp.openpilotLongitudinalControl)
  cc.actuators.accel = -1.0
  _, messages = controller.update(cc.as_reader(), controller_state(), 1_000_000_000)
  brakes = [msg for msg in messages if msg[0] == 0x315]
  assert bool(brakes) == cp.openpilotLongitudinalControl
  for msg in brakes:
    assert msg[2] == (CanBus.CAMERA if alpha else CanBus.POWERTRAIN)
    assert frame(native_safety, 1, msg)
