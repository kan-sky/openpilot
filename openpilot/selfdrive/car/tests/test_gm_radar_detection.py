"""GM radar detection and transmit regressions without Params writes or CAN I/O."""
from collections import defaultdict
from types import SimpleNamespace

import pytest

pytest.importorskip('openpilot.common.params_pyx')

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.gm.radar_interface import LAST_RADAR_MSG, RADAR_HEADER_MSG, RadarInterface
from opendbc.car.gm.values import CAR, DBC, CanBus


def radar_fingerprint(bus=CanBus.OBSTACLE):
  fingerprint = gen_empty_fingerprint()
  fingerprint[bus] = dict.fromkeys(range(RADAR_HEADER_MSG, LAST_RADAR_MSG + 1), 8)
  return fingerprint


@pytest.fixture
def build_cp(monkeypatch):
  from opendbc.car.gm.interface import CarInterface
  params = SimpleNamespace(get_bool=lambda key: False, get_int=lambda key: 0, get_float=lambda key: 50.0)
  monkeypatch.setattr('opendbc.car.gm.interface.Params', lambda: params)
  monkeypatch.setattr('opendbc.car.interfaces.Params', lambda: params)
  monkeypatch.setattr('opendbc.car.gm.carcontroller.Params', lambda: params)
  monkeypatch.setattr('opendbc.car.gm.carcontroller.VCruiseCarrot', lambda cp: SimpleNamespace(autoCruiseControl=0))

  def build(candidate=CAR.CHEVROLET_TRAVERSE, fingerprint=None, alpha=True):
    return CarInterface.get_params(candidate, fingerprint if fingerprint is not None else radar_fingerprint(), [], alpha, True, False)

  return build


@pytest.mark.parametrize('alpha', [False, True])
def test_traverse_radar_detection_preserves_longitudinal_and_safety(build_cp, alpha):
  with_radar = build_cp(alpha=alpha)
  without_radar = build_cp(fingerprint=gen_empty_fingerprint(), alpha=alpha)
  assert not with_radar.radarUnavailable
  assert without_radar.radarUnavailable
  assert RadarInterface(with_radar).rcp is not None
  assert RadarInterface(without_radar).rcp is None
  expected = without_radar.to_dict()
  expected['radarUnavailable'] = False
  assert with_radar.to_dict() == expected


@pytest.mark.parametrize('missing', range(RADAR_HEADER_MSG, LAST_RADAR_MSG + 1))
def test_incomplete_radar_fingerprint_keeps_vision_only(build_cp, missing):
  fingerprint = radar_fingerprint()
  del fingerprint[CanBus.OBSTACLE][missing]
  assert build_cp(fingerprint=fingerprint).radarUnavailable


@pytest.mark.parametrize('bus', [CanBus.POWERTRAIN, CanBus.CAMERA])
def test_radar_messages_on_wrong_bus_do_not_enable_parser(build_cp, bus):
  assert build_cp(fingerprint=radar_fingerprint(bus)).radarUnavailable


@pytest.mark.parametrize('message', [RADAR_HEADER_MSG, RADAR_HEADER_MSG + 1, LAST_RADAR_MSG])
@pytest.mark.parametrize('length', [0, 7, 12])
def test_wrong_message_length_keeps_vision_only(build_cp, message, length):
  fingerprint = radar_fingerprint()
  fingerprint[CanBus.OBSTACLE][message] = length
  assert build_cp(fingerprint=fingerprint).radarUnavailable


@pytest.mark.parametrize('candidate', list(CAR))
def test_other_gm_platforms_are_unchanged_by_radar_fingerprint(build_cp, candidate):
  if candidate == CAR.CHEVROLET_TRAVERSE:
    return
  if candidate == CAR.GMC_YUKON_CC:
    pytest.skip('Existing GMC_YUKON_CC torque parameters are missing')
  assert build_cp(candidate).to_dict() == build_cp(candidate, gen_empty_fingerprint()).to_dict()


def radar_frames(packer, distance=25.0):
  frames = [packer.make_can_msg(RADAR_HEADER_MSG, CanBus.OBSTACLE, {'FLRRNumValidTargets': 1})]
  for address in range(RADAR_HEADER_MSG + 1, LAST_RADAR_MSG + 1):
    values = {'TrkRange': distance, 'TrkRangeRate': -1.5, 'TrkAzimuth': 0.0, 'TrkObjectID': 7} if address == RADAR_HEADER_MSG + 1 else {}
    frames.append(packer.make_can_msg(address, CanBus.OBSTACLE, values))
  return frames


def test_enabled_parser_decodes_and_removes_targets(build_cp):
  ri = RadarInterface(build_cp())
  packer = CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.radar])
  ri.v_ego = 10.0
  rr = ri.update([(1_000_000_000, radar_frames(packer))])
  assert rr is not None and not any(rr.errors.to_dict().values())
  assert len(rr.points) == 1
  point = rr.points[0]
  assert point.trackId == 7 and point.measured
  assert point.dRel == pytest.approx(25.0)
  assert point.vRel == pytest.approx(-1.5)
  assert point.vLead == pytest.approx(8.5)
  rr = ri.update([(1_067_000_000, radar_frames(packer, distance=0.0))])
  assert rr is not None and not rr.points


def test_enabled_parser_reports_lost_radar_frames(build_cp):
  ri = RadarInterface(build_cp())
  packer = CANPacker(DBC[CAR.CHEVROLET_TRAVERSE][Bus.radar])
  ri.update([(1_000_000_000, radar_frames(packer))])
  # Keep the trigger alive while the header and the other slots stop arriving.
  for index in range(10):
    trigger = packer.make_can_msg(LAST_RADAR_MSG, CanBus.OBSTACLE, {})
    rr = ri.update([(3_000_000_000 + index * 67_000_000, [trigger])])
  assert rr is not None and rr.errors.canError


def controller_state():
  out = structs.CarState.new_message(vEgo=10.0, standstill=False)
  out.cruiseState.enabled = True
  return SimpleNamespace(out=out, loopback_lka_steering_cmd_ts_nanos=0,
                         loopback_lka_steering_cmd_updated=False, cam_lka_steering_cmd_counter=0,
                         pt_lka_steering_cmd_counter=0, buttons_counter=0, pcm_acc_status=0,
                         pscm_status=defaultdict(int))


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('active', [False, True])
def test_traverse_radar_does_not_change_can_commands(build_cp, alpha, active):
  from opendbc.car.gm.carcontroller import CarController
  cp_radar = build_cp(alpha=alpha)
  cp_vision = build_cp(fingerprint=gen_empty_fingerprint(), alpha=alpha)
  radar_controller = CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp_radar)
  vision_controller = CarController(DBC[CAR.CHEVROLET_TRAVERSE], cp_vision)
  cc = structs.CarControl.new_message(enabled=active, latActive=active, longActive=active and alpha)
  cc.actuators.accel = 0.5
  cc.actuators.torque = 0.1
  cc = cc.as_reader()
  state = controller_state()
  for frame in range(200):
    now = 1_000_000_000 + frame * 10_000_000
    radar_actuators, radar_sends = radar_controller.update(cc, state, now)
    vision_actuators, vision_sends = vision_controller.update(cc, state, now)
    assert radar_sends == vision_sends
    assert radar_actuators.to_dict() == vision_actuators.to_dict()
    assert not any(bus == CanBus.OBSTACLE for _, _, bus in radar_sends)


def test_ascm_still_sends_radar_status(build_cp):
  from opendbc.car.gm.carcontroller import CarController
  cp = build_cp(CAR.CHEVROLET_VOLT, gen_empty_fingerprint())
  controller = CarController(DBC[CAR.CHEVROLET_VOLT], cp)
  _, messages = controller.update(structs.CarControl.new_message().as_reader(), controller_state(), 1_000_000_000)
  assert any(bus == CanBus.OBSTACLE for _, _, bus in messages)
