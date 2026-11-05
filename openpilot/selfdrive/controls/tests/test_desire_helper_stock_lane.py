from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState


@pytest.fixture
def scenario(monkeypatch):
  helper = DesireHelper()
  helper._update_params_periodic = lambda: None
  helper._check_desire_state = lambda *_: None
  helper.bluetooth_commands = NS(read=lambda **_: None)
  helper.laneLineCheck = 1
  helper.laneChangeBsd = 1
  helper.lane_change_state = LaneChangeState.preLaneChange
  helper.lane_change_delay = 0
  helper.left.object_detected_count = helper.right.object_detected_count = -10
  state = NS(
    stockLaneMarkings=NS(supported=True, left=0, right=1), leftLaneLine=1, rightLaneLine=0,
    canValid=True, canTimeout=False, leftBlinker=True, rightBlinker=False,
    leftBlindspot=False, rightBlindspot=False, vEgo=22.0, aEgo=0.0,
    trailerConnected=False, steeringPressed=False, steeringTorque=0.0,
  )
  model = NS(laneLines=[None] * 4, laneLineProbs=[0.9] * 4, roadEdges=[None] * 2)
  nav = NS(atcType="", carrotCmdIndex=0, carrotCmd="", carrotArg="")
  radar = NS(leadLeft=NS(status=False), leadRight=NS(status=False), leadsLeft=[], leadsRight=[])
  geometry = NS(available=True)

  def update_geometry(side, *_args, **_kwargs):
    side.lane_available = geometry.available
    side.edge_available = False
    side.dist_to_edge_far = 2.0
    side.lane_exist_count.counter = 10 if geometry.available else 0

  monkeypatch.setattr(SideState, "update_lane_geometry", update_geometry)

  def step(fresh=True):
    return helper.update(state, model, True, 0.1, nav, radar, stock_sample_fresh=fresh)

  return NS(helper=helper, state=state, nav=nav, radar=radar, geometry=geometry, step=step)


@pytest.mark.parametrize("driver_left,expected", [(True, True), (False, False)])
def test_matching_side_uses_stock_instead_of_opposite_onnx_code(scenario, driver_left, expected):
  s = scenario
  s.state.leftBlinker, s.state.rightBlinker = driver_left, not driver_left
  s.step()
  assert (s.helper.lane_change_state == LaneChangeState.laneChangeStarting) == expected
  assert (s.state.leftLaneLine, s.state.rightLaneLine) == (1, 0)


@pytest.mark.parametrize("block", ["delay", "blindspot", "radar", "geometry", "torque", "trailer"])
def test_fresh_dashed_does_not_remove_other_guards(scenario, block):
  s = scenario
  if block == "delay":
    s.helper.lane_change_delay = 0.5
  if block == "blindspot":
    s.state.leftBlindspot = True
  if block == "radar":
    s.radar.leadLeft = NS(status=True, dRel=4.0, vRel=-1.0, vLead=21.0, dPath=-3.4)
  if block == "geometry":
    s.geometry.available = False
  if block == "torque":
    s.helper.laneChangeNeedTorque = 1
  if block == "trailer":
    s.state.trailerConnected = True
  s.step()
  assert s.helper.lane_change_state != LaneChangeState.laneChangeStarting


@pytest.mark.parametrize("reason", ["missing", "expired", "invalid", "timeout"])
@pytest.mark.parametrize("mode", [1, 2])
def test_missing_or_stale_stock_never_falls_back_to_legacy_dashes(scenario, reason, mode):
  s = scenario
  s.helper.laneLineCheck = mode
  s.state.leftLaneLine = 0
  s.state.steeringTorque, s.state.steeringPressed = 2.0, True
  if reason == "missing":
    s.state.stockLaneMarkings.left = -1
  if reason == "invalid":
    s.state.canValid = False
  if reason == "timeout":
    s.state.canTimeout = True
  s.step(fresh=reason != "expired")
  assert not s.helper.lane_change_available_left
  assert s.helper.lane_change_state == LaneChangeState.preLaneChange


@pytest.mark.parametrize("driver,torque,bsd,expected", [
  (False, False, False, False), (True, False, False, False),
  (True, True, True, False), (True, True, False, True),
])
def test_known_solid_mode_two_requires_clear_side_and_driver_torque_even_with_atc(scenario, driver, torque, bsd, expected):
  s = scenario
  s.helper.laneLineCheck = 2
  s.state.stockLaneMarkings.left = 1
  s.state.leftBlinker = driver
  s.state.steeringTorque, s.state.steeringPressed = 2.0, torque
  s.state.leftBlindspot = bsd
  s.helper.atc_type = s.nav.atcType = "fork left"
  s.step()
  assert (s.helper.lane_change_state == LaneChangeState.laneChangeStarting) == expected


@pytest.mark.parametrize("atc,side", [("fork left", "left"), ("atc right", "right")])
def test_restored_dashed_is_not_an_atc_start_trigger(scenario, atc, side):
  s = scenario
  s.state.leftBlinker = False
  s.helper.atc_type = s.nav.atcType = atc
  s.helper.auto_lane_change_enable = True
  setattr(s.state.stockLaneMarkings, side, 0)
  s.step(fresh=False)
  s.step(fresh=True)
  assert not getattr(s.helper, side).lane_line_info_edge_detect
  assert not getattr(s.helper, side).lane_change_available_released
  assert s.helper.lane_change_state == LaneChangeState.preLaneChange


def test_mode_two_does_not_allow_solid_with_side_radar_even_with_driver_torque(scenario):
  s = scenario
  s.helper.laneLineCheck = 2
  s.state.stockLaneMarkings.left = 1
  s.state.steeringTorque, s.state.steeringPressed = 2.0, True
  s.radar.leadLeft = NS(status=True, dRel=4.0, vRel=-1.0, vLead=21.0, dPath=-3.4)
  s.step()
  assert s.helper.lane_change_state == LaneChangeState.preLaneChange


@pytest.mark.parametrize("mode,supported,legacy,expected", [
  (0, True, 0, True), (0, True, 20, False), (1, False, 0, True), (1, False, 1, False),
])
def test_disabled_check_and_unsupported_cars_keep_existing_policy(scenario, mode, supported, legacy, expected):
  s = scenario
  s.helper.laneLineCheck = mode
  s.state.stockLaneMarkings.supported = supported
  s.state.stockLaneMarkings.left = 1
  s.state.leftLaneLine = legacy
  s.step(fresh=False)
  assert (s.helper.lane_change_state == LaneChangeState.laneChangeStarting) == expected
