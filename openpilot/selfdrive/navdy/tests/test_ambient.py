from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy.ambient import DoorMonitor, build_door_parsers, read_override, road_state, updated_door_state


def test_override_expiry_and_durable_profile(tmp_path):
  path = tmp_path / 'override'
  path.write_text('{"mode":"manual","expiresAt":"2020-01-01T00:00:00Z"}')
  assert read_override([path], now=1700000000) is None
  path.write_text('{"mode":"profile","expiresAt":"2020-01-01T00:00:00Z","id":"saved"}')
  assert read_override([path], now=1700000000)['id'] == 'saved'
  path.write_text('{"mode":"auto"}')
  assert read_override([path]) == {'mode': 'auto'}


@pytest.mark.parametrize('raw', ['[]', '{}', '{"mode":"bogus"}', '{"mode":"auto","x":NaN}',
                              '{"mode":"manual","expiresAt":"bad"}',
                              '{"mode":"manual","expiresAt":"2020-01-01"}', ' ' * 16385])
def test_malformed_override_is_ignored(tmp_path, raw):
  path = tmp_path / 'override'
  path.write_text(raw)
  assert read_override([path]) is None


def test_override_tries_persistent_path_when_memory_absent(tmp_path):
  path = tmp_path / 'persistent'
  path.write_text('{"mode":"profile"}')
  assert read_override([tmp_path / 'missing', path])['mode'] == 'profile'


def test_door_memory_survives_bridge_restart_but_is_not_a_disk_boot_setting(tmp_path):
  path = tmp_path / 'memory' / 'OffroadDoorOpen'
  monitor = DoorMonitor(path)
  assert not monitor.door_open
  monitor.remember(True)
  assert DoorMonitor(path).door_open
  monitor.remember(False)
  assert not DoorMonitor(path).door_open


def test_power_requires_positive_evidence_for_offroad():
  assert road_state(None, None) is None
  assert road_state(NS(started=False), None) is None
  assert road_state(None, []) is None
  assert road_state(NS(started=False), []) is False
  assert road_state(NS(started=True), None) is True
  assert road_state(None, [NS(ignitionCan=True, ignitionLine=False)]) is True


@pytest.mark.parametrize('dbc,message,signal,bus', [
  ('gm_global_a_lowspeed_1818125', 'Door_Open_Switch_Status_LS', 'DrDoorOpenSwAct', 1),
  ('gm_global_a_lowspeed', 'DriverDoorStatus', 'DriverDoorOpened', 130),
  ('gm_global_a_powertrain_generated', 'BCMDoorBeltStatus', 'RearRightDoor', 0),
])
def test_real_dbc_door_open_close_and_silent_bus(dbc, message, signal, bus):
  from opendbc.can.packer import CANPacker
  parsers = build_door_parsers()
  packer = CANPacker(dbc)
  assert updated_door_state(parsers, [(1, [packer.make_can_msg(message, bus, {signal: 1})])]) is True
  assert updated_door_state(parsers, [(2, [])]) is None
  assert updated_door_state(parsers, [(3, [packer.make_can_msg(message, bus, {signal: 0})])]) is False


def test_quiet_cached_open_cannot_override_new_close_on_another_bus():
  from opendbc.can.packer import CANPacker
  parsers = build_door_parsers()
  modern = CANPacker('gm_global_a_lowspeed_1818125')
  bcm = CANPacker('gm_global_a_powertrain_generated')
  assert updated_door_state(parsers, [(1, [modern.make_can_msg('Door_Open_Switch_Status_LS', 1, {'DrDoorOpenSwAct': 1})])])
  assert updated_door_state(parsers, [(2, [bcm.make_can_msg('BCMDoorBeltStatus', 0, {'FrontLeftDoor': 0})])]) is False


def test_unwatched_bus_cannot_change_door():
  from opendbc.can.packer import CANPacker
  frame = CANPacker('gm_global_a_powertrain_generated').make_can_msg('BCMDoorBeltStatus', 3, {'FrontLeftDoor': 1})
  assert updated_door_state(build_door_parsers(), [(1, [frame])]) is None
