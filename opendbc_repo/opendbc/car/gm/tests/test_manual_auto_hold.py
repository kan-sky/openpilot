"""Manual hold state, real CAN output and compiled Panda hooks; no vehicle I/O."""
# ruff: noqa: INP001
import ctypes
import itertools
from time import monotonic_ns

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.gm.manual_auto_hold import ManualAutoHold, manual_hold_supported
from opendbc.car.gm.values import CAR, DBC, GMSafetyFlags
from opendbc.car.gm.tests.test_display_speed import MemoryParams, build_cp, controller_state, runtime as desktop_runtime


@pytest.fixture(scope="module")
def runtime():
  yield from desktop_runtime.__wrapped__()


def state(**changes):
  values = dict(canValid=True, gearShifter="drive", standstill=True, brakePressed=True, vEgo=0., vEgoRaw=0.)
  values.update(changes)
  return structs.CarState(**values)


def armed_hold():
  hold = ManualAutoHold()
  hold.update(state(), 80, False)
  assert hold.active and hold.command(state(), False) == 400
  return hold


@pytest.mark.parametrize("pedal", [0, 10, 79, 80, 100, 255])
@pytest.mark.parametrize("active", [False, True])
def test_strong_pedal_only_while_disengaged(pedal, active):
  hold = ManualAutoHold()
  hold.update(state(), pedal, active)
  assert hold.active == (pedal >= 80 and not active)


def test_arm_before_stop_but_never_brake_while_moving():
  hold = ManualAutoHold()
  hold.update(state(standstill=False, vEgo=1., vEgoRaw=1.), 90, False)
  assert hold.armed and not hold.active
  assert hold.command(state(standstill=False, vEgo=1., vEgoRaw=1.), False) == 0
  hold.update(state(), 30, False)
  assert hold.active


def test_release_before_stop_cancels_arming():
  hold = ManualAutoHold()
  hold.update(state(standstill=False, vEgo=1., vEgoRaw=1.), 90, False)
  hold.update(state(standstill=False, vEgo=.5, vEgoRaw=.5, brakePressed=False), 0, False)
  hold.update(state(), 30, False)
  assert not hold.active and not hold.armed


@pytest.mark.parametrize("peak,remains", [(10, False), (79, False), (80, True), (100, True)])
def test_initial_release_and_next_tap(peak, remains):
  hold = armed_hold()
  for _ in range(100):
    hold.update(state(), 20, False)
    assert hold.active
  for _ in range(100):
    hold.update(state(brakePressed=False), 0, False)
    assert hold.active
  hold.update(state(), 10, False)
  assert hold.active
  hold.update(state(), peak, False)
  hold.update(state(brakePressed=False), 0, False)
  assert hold.active == remains


@pytest.mark.parametrize("changes", [dict(gasPressed=True), dict(regenBraking=True), dict(parkingBrake=True),
  dict(gearShifter="park"), dict(gearShifter="reverse"), dict(gearShifter="neutral"), dict(canValid=False),
  dict(canTimeout=True), dict(accFaulted=True), dict(cruiseState={"enabled": True}), dict(standstill=False),
  dict(vEgo=.05), dict(vEgoRaw=.05), dict(vEgo=-.05), dict(vEgo=float("nan")), dict(vEgoRaw=float("inf"))])
def test_ineligible_state_clears(changes):
  hold = armed_hold()
  cs = state(**changes)
  assert hold.command(cs, False) == 0
  hold.update(cs, 0, False)
  assert not hold.active and not hold.armed


@pytest.mark.parametrize("pedal", [-1, 256, float("nan"), float("inf")])
def test_invalid_pedal_clears(pedal):
  hold = armed_hold()
  hold.update(state(), pedal, False)
  assert not hold.active


def test_engaged_state_cannot_carry_manual_hold():
  hold = armed_hold()
  assert hold.command(state(), True) == 0
  hold.update(state(), 100, True)
  hold.update(state(brakePressed=False), 0, False)
  assert not hold.armed and not hold.active


@pytest.mark.parametrize("candidate,alpha,pedal", [(CAR.CHEVROLET_TRAVERSE, True, False),
  (CAR.CHEVROLET_TRAVERSE, False, False), (CAR.CHEVROLET_TRAVERSE, True, True),
  (CAR.CHEVROLET_BOLT_EUV, True, False), (CAR.CHEVROLET_VOLT, True, False)])
def test_feature_flag_scope(runtime, candidate, alpha, pedal):
  fingerprint = gen_empty_fingerprint()
  if pedal:
    fingerprint[0][0x201] = 6
  cp = runtime.interface.CarInterface.get_params(candidate, fingerprint, [], alpha, True, False)
  assert manual_hold_supported(cp) == (candidate == CAR.CHEVROLET_TRAVERSE and alpha and not pedal)


@pytest.mark.parametrize("long_state", ["off", "stopping", "starting"])
def test_real_controller_brakes_only_no_set_resume(runtime, long_state):
  cp = build_cp(runtime)
  controller = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  controller.v_cruise_carrot.autoCruiseControl = 2
  cs = controller_state()
  cs.out = state(activateCruise=1)
  cs.manual_auto_hold = armed_hold()
  cc = structs.CarControl()
  cc.actuators.longControlState = long_state
  decoder = runtime.CANParser(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt], [("ASCMGasRegenCmd", 25)], 0)
  for tick in range(100):
    now = 1_000_000_000 + tick * 10_000_000
    out, packets = controller.update(cc.as_reader(), cs, now)
    assert all(addr != 0x1E1 for addr, _, _ in packets)
    for addr, data, bus in packets:
      if addr == 0x315:
        assert bus == 2 and len(data) == 5 and data[0] >> 4 == 0xA
        assert (-int.from_bytes(data[:2], "big")) & 0xFFF == 400
        assert out.brake == 400
      if addr == 0x2CB:
        decoder.update([(now, [(addr, data, bus)])])
        assert not decoder.vl["ASCMGasRegenCmd"]["GasRegenCmdActive"]
        assert decoder.vl["ASCMGasRegenCmd"]["GasRegenCmd"] == controller.params.INACTIVE_REGEN


@pytest.mark.parametrize("enabled,long_active", [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize("long_state", ["off", "pid", "stopping", "starting"])
def test_no_manual_trigger_leaves_output_unchanged(runtime, enabled, long_active, long_state):
  cp = build_cp(runtime)
  before_cp = structs.CarParams(**cp.to_dict())
  before_cp.safetyConfigs[0].safetyParam &= ~int(GMSafetyFlags.MANUAL_AUTO_HOLD)
  before = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], before_cp)
  after = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  before_cs, after_cs = controller_state(), controller_state()
  after_cs.manual_auto_hold = ManualAutoHold()
  cc = structs.CarControl(enabled=enabled, longActive=long_active)
  cc.actuators.longControlState = long_state
  cc.actuators.accel = -1.
  for tick in range(100):
    now = 1_000_000_000 + tick * 10_000_000
    old_out, old_frames = before.update(cc.as_reader(), before_cs, now)
    new_out, new_frames = after.update(cc.as_reader(), after_cs, now)
    assert old_out.to_dict() == new_out.to_dict() and old_frames == new_frames


def test_real_interface_update_apply_and_freshness(runtime, monkeypatch, safety):
  monkeypatch.setattr(MemoryParams, "put_int", MemoryParams.put_int_nonblocking, raising=False)
  cp = build_cp(runtime)
  ci = runtime.interface.CarInterface(cp)
  ci.CS.update(ci.can_parsers)
  packer = runtime.CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt])
  drive = next(k for k, v in ci.CS.shifter_values.items() if v == "D")
  values = {"ECMPRDNL2": {"PRNDL2": drive}, "ECMEngineStatus": {"BrakePressed": 1},
            "ECMAcceleratorPos": {"BrakePedalPos": 80}, "BCMDoorBeltStatus": {"LeftSeatBelt": 1}}
  cc = structs.CarControl()

  def packets():
    return [packer.make_can_msg(msg.name, parser.bus, values.get(msg.name, {}))
            for parser in ci.can_parsers.values() for msg in parser.message_states.values()]
  for tick in range(5):
    now = monotonic_ns()
    can_frames = packets()
    for addr, data, bus in can_frames:
      if bus == 0 and addr in (0x34A, 0xC9, 0x1C4, 0xBE):
        assert rx(safety, addr, data)
    ret = ci.update([(now, can_frames)])
    assert ret.canValid and not ret.canTimeout
    _, sends = ci.apply(cc.as_reader(), now, model_v2="model", radar_state="radar")
    for addr, data, bus in sends:
      if addr in (0x315, 0x2CB):
        accepted = safety.gm_test_frame(1, addr, bus, data, len(data))
        assert accepted, (tick, addr, can_frames)
  assert ci.CS.out.brakeHoldActive and ci.CS.manual_auto_hold.active
  assert ci.CS.modelV2 == "model" and ci.CS.radarState == "radar"
  values["ECMEngineStatus"]["BrakePressed"] = 0
  values["ECMAcceleratorPos"]["BrakePedalPos"] = 0
  now = monotonic_ns()
  assert ci.update([(now, packets())]).brakeHoldActive
  ci.apply(cc.as_reader(), now)
  assert ci.CS.out.brakeHoldActive
  ci.apply(cc.as_reader(), now + 500_000_001)
  assert not ci.CS.out.brakeHoldActive and not ci.CS.manual_auto_hold.active


@pytest.fixture(scope="module")
def native(runtime, tmp_path_factory):
  from opendbc.safety.tests.test_gm_sdgm import native_safety
  lib = native_safety.__wrapped__(tmp_path_factory)
  lib.set_timer.argtypes = [ctypes.c_uint32]
  return lib


HOLD_FLAGS = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.HW_SDGM | GMSafetyFlags.MANUAL_AUTO_HOLD)


@pytest.fixture
def safety(native):
  assert native.set_safety_hooks(structs.CarParams.SafetyModel.gm, HOLD_FLAGS) == 0
  native.init_tests()
  return native


def rx(lib, addr, data):
  return lib.gm_test_frame(0, addr, 0, data, len(data))


def inputs(lib, pedal=80, pressed=True, speed=0, gas=0, cruise=0, order=(0, 1, 2, 3), omit=None):
  messages = [(0x34A, speed.to_bytes(2, "big") * 2 + b"\x00"),
              (0xC9, bytes([0, 0, 0, 0x20, 0, int(pressed), 0, 0])),
              (0x1C4, bytes([0, cruise << 5, 0, 0, 0, gas, 0, 0])),
              (0xBE, bytes([0, pedal, 0, 0, 0, 0]))]
  for i in order:
    addr, data = messages[i]
    if addr != omit:
      assert rx(lib, addr, data)


def tx_brake(lib, value=400, bus=2, mode=0xA, length=5):
  raw = (-value) & 0xFFF
  data = (bytes([mode << 4 | raw >> 8, raw & 255]) + bytes(6))[:length]
  return bool(lib.gm_test_frame(1, 0x315, bus, data, len(data)))


@pytest.mark.parametrize("order", list(itertools.permutations(range(4))))
def test_native_initial_arm_packet_order(safety, order):
  assert not tx_brake(safety)
  inputs(safety, order=order)
  inputs(safety, order=order)
  assert tx_brake(safety)
  assert not safety.get_controls_allowed()


def test_native_all_brake_values_buses_modes_lengths(safety):
  inputs(safety)
  for bus in range(4):
    for value in range(4096):
      assert tx_brake(safety, value, bus) == (bus == 2 and value <= 400)
  for mode in range(16):
    assert tx_brake(safety, mode=mode) == (mode == 0xA)
  for length in range(9):
    assert tx_brake(safety, length=length) == (length == 5)
  assert not safety.get_controls_allowed()


@pytest.mark.parametrize("omit", [0x34A, 0xC9, 0x1C4, 0xBE])
def test_native_missing_stale_inputs(safety, omit):
  inputs(safety, omit=omit)
  assert not tx_brake(safety)
  inputs(safety)
  assert tx_brake(safety)
  safety.set_timer(500001)
  inputs(safety, pedal=0, pressed=False, omit=omit)
  assert not tx_brake(safety)
  inputs(safety, pedal=0, pressed=False)
  assert not tx_brake(safety)


@pytest.mark.parametrize("change", [dict(speed=11), dict(gas=1), dict(cruise=1), dict(cruise=3)])
def test_native_motion_gas_cruise_clears(safety, change):
  inputs(safety)
  inputs(safety, pedal=0, pressed=False, **change)
  assert not tx_brake(safety)
  inputs(safety, pedal=0, pressed=False)
  assert not tx_brake(safety)


@pytest.mark.parametrize("peak", [10, 79, 80, 100])
@pytest.mark.parametrize("order", list(itertools.permutations(range(4))))
def test_native_retains_permission_across_asynchronous_pedal_frames(safety, peak, order):
  inputs(safety)
  inputs(safety, pedal=0, pressed=False, order=order)
  assert tx_brake(safety)
  inputs(safety, pedal=peak, order=order)
  inputs(safety, pedal=0, pressed=False, order=order)
  assert tx_brake(safety)
  assert tx_brake(safety, 0)  # Host light-tap cancellation remains permitted.


def test_native_weak_or_moving_brake(safety):
  inputs(safety, pedal=79)
  assert not tx_brake(safety)
  inputs(safety, speed=174)
  assert not tx_brake(safety)
  inputs(safety, pedal=0, pressed=False, speed=174)
  inputs(safety, pedal=0, pressed=False)
  assert not tx_brake(safety)
  inputs(safety, speed=100)
  assert not tx_brake(safety)
  inputs(safety, pedal=30)
  assert tx_brake(safety)


def test_native_stale_tx_and_timer_wrap(safety):
  safety.set_timer(0xFFFFFF00)
  inputs(safety)
  safety.set_timer(100000)
  assert tx_brake(safety)
  safety.set_timer(500001)
  assert not tx_brake(safety)


def test_native_relay_failure_and_regen_block(safety):
  inputs(safety)
  assert rx(safety, 0xBD, bytes([0x10]) + bytes(6))
  assert not tx_brake(safety)
  assert rx(safety, 0xBD, bytes(7))
  inputs(safety)
  assert tx_brake(safety)
  safety.set_relay_malfunction(True)
  assert not tx_brake(safety)


@pytest.mark.parametrize("flag", [GMSafetyFlags.HW_CAM, GMSafetyFlags.HW_CAM_LONG,
                                 GMSafetyFlags.HW_SDGM, GMSafetyFlags.MANUAL_AUTO_HOLD])
def test_native_feature_flag_and_mode_reset(safety, flag):
  inputs(safety)
  assert tx_brake(safety)
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, HOLD_FLAGS & ~int(flag)) == 0
  inputs(safety)
  assert not tx_brake(safety)
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, HOLD_FLAGS) == 0
  inputs(safety, pedal=0, pressed=False)
  assert not tx_brake(safety)


def test_native_no_acceleration_permission(safety, runtime):
  from opendbc.car.gm import gmcan
  inputs(safety)
  packer = runtime.CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.pt])
  for amount, active in [(-500, False), (-500, True), (0, False), (100, True)]:
    addr, data, bus = gmcan.create_gas_regen_command(packer, 0, amount, 0, active, False)
    assert bool(safety.gm_test_frame(1, addr, bus, data, len(data))) == (amount == -500 and not active)
    assert not safety.get_controls_allowed()


@pytest.mark.parametrize("address,length", [(0x34A, 4), (0xC9, 7), (0x1C4, 7), (0xBE, 5)])
def test_native_malformed_input_drops_permission(safety, address, length):
  inputs(safety)
  assert tx_brake(safety)
  rx(safety, address, bytes(length))
  assert not tx_brake(safety)


@pytest.mark.parametrize("extra", [GMSafetyFlags.CC_LONG, GMSafetyFlags.NO_ACC, GMSafetyFlags.PEDAL_LONG,
                                  GMSafetyFlags.GAS_INTERCEPTOR, GMSafetyFlags.HW_ASCM_LONG, GMSafetyFlags.EV])
def test_native_incompatible_mode_cannot_enable_hold(safety, extra):
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, HOLD_FLAGS | int(extra)) == 0
  inputs(safety)
  assert not tx_brake(safety)


def test_host_release_produces_zero_brake_accepted_by_panda(runtime, safety):
  cp = build_cp(runtime)
  controller = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp)
  cs = controller_state()
  cs.out = state()
  cs.manual_auto_hold = armed_hold()
  inputs(safety)
  cc = structs.CarControl().as_reader()
  for tick, (pedal, pressed) in enumerate([(80, True), (0, False), (20, True), (0, False)]):
    cs.out.brakePressed = pressed
    cs.manual_auto_hold.update(cs.out, pedal, False)
    inputs(safety, pedal=pedal, pressed=pressed)
    controller.frame = tick * 4
    _, frames = controller.update(cc, cs, 1_000_000_000 + tick * 40_000_000)
    brakes = [msg for msg in frames if msg[0] == 0x315]
    assert len(brakes) == 1
    addr, data, bus = brakes[0]
    assert (-int.from_bytes(data[:2], "big")) & 0xFFF == (0 if tick == 3 else 400)
    assert safety.gm_test_frame(1, addr, bus, data, len(data))
    assert not safety.get_controls_allowed()


@pytest.mark.parametrize("raw_first", [True, False])
def test_single_strong_sample_and_pressed_switch_order(safety, raw_first):
  inputs(safety, pedal=0, pressed=False)
  strong = (0xBE, bytes([0, 80, 0, 0, 0, 0]))
  pressed = (0xC9, bytes([0, 0, 0, 0x20, 0, 1, 0, 0]))
  for message in ([strong, pressed] if raw_first else [pressed, strong]):
    assert rx(safety, *message)
  assert rx(safety, 0xBE, bytes([0, 79, 0, 0, 0, 0]))
  assert tx_brake(safety)
  inputs(safety, pedal=0, pressed=False)
  assert tx_brake(safety)
  assert not safety.get_controls_allowed()
