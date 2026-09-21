import math
from types import SimpleNamespace

from parameterized import parameterized

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.gm.carstate import create_stock_long_cancel_button_events
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.gmcan import (apply_driver_gas_override, apply_stock_longitudinal_gate, create_acc_dashboard_command,
                                 create_friction_brake_command, create_gas_regen_command, get_acc_dashboard_enabled,
                                 get_longitudinal_command_timing, get_longitudinal_sync_messages,
                                 is_trailblazer_camera_longitudinal)
from opendbc.car.gm.values import CAR, CAMERA_ACC_CAR, GM_RX_OFFSET

CAMERA_DIAGNOSTIC_ADDRESS = 0x24B
NetworkLocation = structs.CarParams.NetworkLocation


class TestGMFingerprint:
  @parameterized.expand(FINGERPRINTS.items())
  def test_can_fingerprints(self, car_model, fingerprints):
    assert len(fingerprints) > 0

    assert all(len(finger) for finger in fingerprints)

    # The camera can sometimes be communicating on startup
    if car_model in CAMERA_ACC_CAR:
      for finger in fingerprints:
        for required_addr in (CAMERA_DIAGNOSTIC_ADDRESS, CAMERA_DIAGNOSTIC_ADDRESS + GM_RX_OFFSET):
          assert finger.get(required_addr) == 8, required_addr


class TestTrailblazerLongitudinalIntegrity:
  def test_stock_long_cancel_is_a_complete_momentary_button_event(self):
    events = create_stock_long_cancel_button_events()

    assert [(event.type, event.pressed) for event in events] == [
      (structs.CarState.ButtonEvent.Type.cancel, True),
      (structs.CarState.ButtonEvent.Type.cancel, False),
    ]

  @parameterized.expand([
    ("trailblazer_gas", CAR.CHEVROLET_TRAILBLAZER, True, (-500, 0, False, False)),
    ("trailblazer_released", CAR.CHEVROLET_TRAILBLAZER, False, (-540, 143, True, True)),
    ("other_gm_gas", CAR.CHEVROLET_EQUINOX, True, (-540, 143, True, True)),
  ])
  def test_driver_gas_override_values(self, _, car_fingerprint, gas_pressed, expected):
    result = apply_driver_gas_override(car_fingerprint, gas_pressed, -500, -540, 143, True, True)
    assert result == expected

  def test_driver_gas_override_keeps_counter_group_transmittable(self):
    packer = CANPacker("gm_global_a_powertrain_volt")
    CP = SimpleNamespace(carFingerprint=CAR.CHEVROLET_TRAILBLAZER)
    apply_gas, apply_brake, at_full_stop, near_stop = apply_driver_gas_override(
      CP.carFingerprint, True, -500, -540, 143, False, True,
    )

    gas_msg = create_gas_regen_command(packer, 0, apply_gas, 3, True, at_full_stop, CP.carFingerprint)
    brake_msg = create_friction_brake_command(packer, 0, apply_brake, 3, True, near_stop, at_full_stop, CP)

    assert gas_msg[1].hex() == "c142b09000bd4f6d"
    assert brake_msg[1].hex() == "1000effd03"

  @parameterized.expand(
    [
      # Captured stock Trailblazer frames. The first two exercise the lower
      # and upper 24-bit carry/borrow boundaries missed by the old byte-wise
      # checksum implementation.
      ("counter_0_low_byte_zero", 346, 0, True, "0142cb0000bd3500"),
      ("counter_3_low_byte_overflow", 249.75, 3, True, "c142c7fe00bd37ff"),
      ("inactive_checksum_bit", -500, 2, False, "8042b09001bd4f6e"),
    ]
  )
  def test_gas_regen_checksum_matches_stock(self, _, throttle, counter, enabled, expected_payload):
    packer = CANPacker("gm_global_a_powertrain_volt")
    msg = create_gas_regen_command(packer, 0, throttle, counter, enabled, False, CAR.CHEVROLET_TRAILBLAZER)
    assert msg[1].hex() == expected_payload

  @parameterized.expand([
    ("counter_0_low_byte_zero", 346, 0, True, "0142cb0000bd3400"),
    ("counter_3_low_byte_overflow", 249.75, 3, True, "c142c7fe00bd38ff"),
    ("inactive", -500, 2, False, "8042b09001bd4f6e"),
  ])
  def test_other_gm_keeps_legacy_gas_regen_checksum(self, _, throttle, counter, enabled, expected_payload):
    packer = CANPacker("gm_global_a_powertrain_volt")
    msg = create_gas_regen_command(packer, 0, throttle, counter, enabled, False, CAR.CHEVROLET_EQUINOX)
    default_msg = create_gas_regen_command(packer, 0, throttle, counter, enabled, False)
    assert msg[1].hex() == expected_payload
    assert default_msg[1] == msg[1]

  @parameterized.expand([
    ("inactive_counter_0", "0042b09001bd4f70", 0, 0),
    ("inactive_counter_1", "4042b09001bd4f6f", 1, 0),
    ("inactive_counter_2", "8042b09001bd4f6e", 2, 0),
    ("inactive_counter_3", "c042b09001bd4f6d", 3, 0),
    ("active_counter_0", "0142cb0000bd3500", 0, 1),
  ])
  def test_stock_gas_regen_counter_and_active_decode(self, _, payload, expected_counter, expected_active):
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCMGasRegenCmd", float('nan'))], 2)
    parser.update([(1_000_000_000, [(0x2CB, bytes.fromhex(payload), 2)])])
    assert parser.vl["ASCMGasRegenCmd"]["RollingCounter"] == expected_counter
    assert parser.vl["ASCMGasRegenCmd"]["GasRegenCmdActive"] == expected_active

  @parameterized.expand([
    ("active", True),
    ("inactive", False),
  ])
  def test_uses_actual_stock_gas_regen_counter(self, _, stock_active):
    CP = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_TRAILBLAZER,
                         networkLocation=NetworkLocation.fwdCamera)
    CS = SimpleNamespace(cam_ascm_2cb_counter_ts_nanos=1, cam_ascm_2cb_counter_updated=True,
                         cam_ascm_2cb_counter=3, cam_stock_long_active=stock_active, cam_acc_status={})
    assert get_longitudinal_command_timing(CP, CS, frame=41) == (True, 3)

    CS.cam_ascm_2cb_counter_updated = False
    assert get_longitudinal_command_timing(CP, CS, frame=44) == (False, 3)

  @parameterized.expand([
    ("neither_reference", 0, None, None),
    ("no_stock_active", 1, None, {}),
    ("no_acc_status", 1, False, None),
  ])
  def test_waits_for_both_stock_references(self, _, counter_ts, stock_active, acc_status):
    CP = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_TRAILBLAZER,
                         networkLocation=NetworkLocation.fwdCamera)
    CS = SimpleNamespace(cam_ascm_2cb_counter_ts_nanos=counter_ts, cam_stock_long_active=stock_active,
                         cam_acc_status=acc_status)
    assert get_longitudinal_command_timing(CP, CS, frame=4) == (False, 0)

  def test_other_gm_uses_original_clock(self):
    CP = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_EQUINOX,
                         networkLocation=NetworkLocation.fwdCamera)
    assert get_longitudinal_command_timing(CP, SimpleNamespace(), frame=4) == (True, 1)
    assert get_longitudinal_command_timing(CP, SimpleNamespace(), frame=5) == (False, 1)

  def test_trailblazer_camera_longitudinal_scope(self):
    CP = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_TRAILBLAZER,
                         networkLocation=NetworkLocation.fwdCamera)
    assert is_trailblazer_camera_longitudinal(CP)

    CP.carFingerprint = CAR.CHEVROLET_TRAILBLAZER_CC
    assert not is_trailblazer_camera_longitudinal(CP)

    CP.carFingerprint = CAR.CHEVROLET_TRAILBLAZER
    CP.openpilotLongitudinalControl = False
    assert not is_trailblazer_camera_longitudinal(CP)

  def test_sync_messages_do_not_require_alive_frequency(self):
    CP = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_TRAILBLAZER,
                         networkLocation=NetworkLocation.fwdCamera)
    messages = get_longitudinal_sync_messages(CP)
    assert [name for name, _ in messages] == ["ASCMGasRegenCmd", "ASCMActiveCruiseControlStatus"]
    assert all(math.isnan(frequency) for _, frequency in messages)

    parser = CANParser("gm_global_a_powertrain_volt", messages, 2)
    assert parser.message_states[0x2CB].ignore_alive
    assert parser.message_states[0x370].ignore_alive

  @parameterized.expand([
    ("trailblazer_stock_inactive", CAR.CHEVROLET_TRAILBLAZER, False, (-500, 0, False, False, False)),
    ("trailblazer_stock_missing", CAR.CHEVROLET_TRAILBLAZER, None, (-500, 0, False, False, False)),
    ("trailblazer_stock_active", CAR.CHEVROLET_TRAILBLAZER, True, (-540, 143, True, True, True)),
    ("other_gm", CAR.CHEVROLET_EQUINOX, False, (-540, 143, True, True, True)),
  ])
  def test_stock_longitudinal_gate(self, _, car_fingerprint, stock_active, expected):
    result = apply_stock_longitudinal_gate(car_fingerprint, stock_active, -500, -540, 143, True, True, True)
    assert result == expected

  def test_stock_veto_creates_inactive_command_group(self):
    packer = CANPacker("gm_global_a_powertrain_volt")
    CP = SimpleNamespace(carFingerprint=CAR.CHEVROLET_TRAILBLAZER)
    values = apply_stock_longitudinal_gate(CP.carFingerprint, False, -500, -540, 143, True, True, True)
    apply_gas, apply_brake, at_full_stop, near_stop, acc_engaged = values

    gas_msg = create_gas_regen_command(packer, 0, apply_gas, 1, acc_engaged, at_full_stop, CP.carFingerprint)
    brake_msg = create_friction_brake_command(packer, 0, apply_brake, 1, acc_engaged, near_stop, at_full_stop, CP)

    assert gas_msg[1].hex() == "4042b09001bd4f6f"
    assert brake_msg[1].hex() == "1000efff01"

  @parameterized.expand([
    ("all_active", True, True, True, {"ACCCmdActive": 1}, True),
    ("not_in_drive", True, False, True, {"ACCCmdActive": 1}, False),
    ("stock_long_veto", True, True, False, {"ACCCmdActive": 1}, False),
    ("stock_status_veto", True, True, True, {"ACCCmdActive": 0}, False),
    ("missing_stock_status", True, True, True, None, False),
    ("openpilot_disabled", False, True, True, {"ACCCmdActive": 1}, False),
  ])
  def test_trailblazer_dashboard_gate(self, _, enabled, in_drive, stock_active, stock_status, expected):
    assert get_acc_dashboard_enabled(
      CAR.CHEVROLET_TRAILBLAZER, enabled, in_drive, stock_active, stock_status,
    ) == expected

  def test_other_gm_dashboard_state_is_unchanged(self):
    assert get_acc_dashboard_enabled(CAR.CHEVROLET_EQUINOX, True, False, False, None)

  @parameterized.expand([
    ("inactive", "000231790000"),
    ("unknown_bit_3", "0802b29f0000"),
    ("unknown_bit_5", "200233290000"),
  ])
  def test_inactive_acc_status_matches_stock_exactly(self, _, stock_payload_hex):
    packer = CANPacker("gm_global_a_powertrain_volt")
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCMActiveCruiseControlStatus", 25)], 2)
    stock_payload = bytes.fromhex(stock_payload_hex)
    parser.update([(1_000_000_000, [(0x370, stock_payload, 2)])])

    hud_control = SimpleNamespace(leadDistanceBars=0, leadVisible=False)
    msg = create_acc_dashboard_command(packer, 0, False, 0, hud_control, False,
                                       dict(parser.vl["ASCMActiveCruiseControlStatus"]))
    assert msg[1] == stock_payload

  def test_active_acc_status_preserves_stock_protocol_state(self):
    packer = CANPacker("gm_global_a_powertrain_volt")
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCMActiveCruiseControlStatus", 25)], 0)
    stock_payload = bytes.fromhex("200233290000")
    parser.update([(1_000_000_000, [(0x370, stock_payload, 0)])])
    stock_values = dict(parser.vl["ASCMActiveCruiseControlStatus"])

    hud_control = SimpleNamespace(leadDistanceBars=2, leadVisible=True)
    msg = create_acc_dashboard_command(packer, 0, True, 100, hud_control, True, stock_values)
    parser.update([(2_000_000_000, [(0x370, msg[1], 0)])])
    values = parser.vl["ASCMActiveCruiseControlStatus"]

    assert values["ACCCruiseState"] == stock_values["ACCCruiseState"] == 2
    assert values["ACCAlwaysOne"] == stock_values["ACCAlwaysOne"] == 0
    assert values["ACCAlwaysOne2"] == stock_values["ACCAlwaysOne2"] == 0
    assert values["ACCUnknownBit5"] == stock_values["ACCUnknownBit5"] == 1
    assert values["ACCCmdActive"] == 1
    assert values["ACCSpeedSetpoint"] == 100
    assert values["ACCGapLevel"] == 2
    assert values["ACCLeadCar"] == 1
    assert values["FCWAlert"] == 3
