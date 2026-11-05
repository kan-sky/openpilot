import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock  # noqa: TID251 -- mocks only; tests use pytest

import pytest

from openpilot.selfdrive.carrot.tests.test_navigation_source import services as services
from openpilot.selfdrive.navdy.navigation import FeedbackTracker
from openpilot.selfdrive.navdy.tests.test_navigation import packet


@pytest.fixture
def navigation_publication(services):
  cls, values = services
  values.update(CarrotNaviSource=1, AutoNaviSpeedCtrlMode=2, AutoNaviSpeedSafetyFactor=100,
                AutoNaviSpeedBumpSpeed=35, AutoNaviSpeedDecelRate=135, AutoRoadSpeedLimitOffset=-1)
  svc = cls()
  svc.params_memory = Mock()
  svc.navdy_navigation = Mock()
  svc._update_gps = Mock(return_value=0.)
  svc._publish_model_route = Mock()
  svc._stamp_model_instruction = Mock()
  svc.update_nav_instruction = Mock()
  svc.update_auto_turn = Mock(return_value=(250., 'none', 250., 0.))
  scope = cls.update_navi.__globals__
  scope['messaging'] = SimpleNamespace(new_message=lambda service, **kw:
                                      SimpleNamespace(**kw, **{service: SimpleNamespace()}))
  sm = SimpleNamespace(alive={'carState': False, 'selfdriveState': False})
  sent = {}
  publisher = SimpleNamespace(send=lambda name, msg: sent.update({name: msg}))
  tracker = FeedbackTracker()

  def apply(value, now):
    assert tracker.accept(value, now, now)
    svc.navdy_navigation.read.return_value = tracker.snapshot(now)
    svc.update_navi('old-network-host', sm, publisher, 200., [], [], 200., 'gpsLocationExternal')
    return sent['carrotMan'].carrotMan, sent['navInstructionCarrot'].navInstructionCarrot

  return svc, values, apply


def test_live_camera_and_bump_remain_separate(navigation_publication):
  svc, _, apply = navigation_publication
  value = packet()
  value['cameraSpeedKph'], value['cameraDistance'] = 30, '112 m'
  value['navigationFeedback']['guidance']['sdi'] = {'type': 22, 'distanceMeters': 96}
  cm, _ = apply(value, 10)
  assert (svc.nSdiPlusType, svc.nSdiPlusDist) == (22, 96)
  assert (cm.xSpdType, cm.xSpdDist, cm.xSpdLimit, cm.desiredSource) == (1, 112, 30, 'cam')


def test_bump_only_has_correct_description(navigation_publication):
  svc, _, apply = navigation_publication
  value = packet()
  value['cameraSpeedKph'] = 0
  value['navigationFeedback']['guidance']['sdi'] = {'type': 22, 'distanceMeters': 96}
  cm, _ = apply(value, 10)
  assert (cm.xSpdType, cm.xSpdDist, cm.xSpdLimit) == (22, 96, 35)
  assert cm.szSdiDescr == svc._get_sdi_descr(22)
  assert cm.szSdiDescr


@pytest.mark.parametrize('clear', ['null', 'invalid', 'expired', 'disconnected'])
def test_navdy_expired_limit_is_not_published_to_cruise_or_ui(navigation_publication, clear):
  svc, _, apply = navigation_publication
  assert apply(packet(), 10)[0].nRoadLimitSpeed == 60
  value = packet(sequence=2)
  now = 11
  if clear == 'null':
    value['roadPreview'] = None
  elif clear == 'invalid':
    value['roadPreview']['roadLimitKph'] = 0
  elif clear == 'expired':
    value['roadPreview']['sequence'] = 1
    now = 14
  else:
    value = {'navigationFeedback': None, 'roadPreview': None}
  cm, inst = apply(value, now)
  assert not svc.carrot_navi_road_limit_valid
  assert cm.nRoadLimitSpeed == 0
  assert not cm.roadLimitSpeedValid
  if hasattr(inst, 'speedLimit'):
    assert inst.speedLimit == 0


def test_navdy_connection_flag_uses_accepted_data_not_old_remote(navigation_publication):
  _, _, apply = navigation_publication
  cm, _ = apply(packet(), 10)
  assert (cm.externalNavigationSource, cm.externalNavigationActive) == ('navdy', True)
  assert cm.roadLimitSpeedValid
  cm, _ = apply({'navigationFeedback': None, 'roadPreview': None}, 11)
  assert (cm.externalNavigationSource, cm.externalNavigationActive) == ('navdy', False)
  assert not cm.roadLimitSpeedValid


def test_traffic_segments_publish_then_clear_with_guidance(navigation_publication):
  _, _, apply = navigation_publication
  value = packet()
  value['navigationFeedback']['guidance'].update(
    trafficTotalMeters=1500, trafficSegments=[{'endM': 500, 'status': 'G'}, {'endM': 1500, 'status': 'R'}])
  cm, _ = apply(value, 10)
  assert cm.routeTrafficTotalMeters == 1500
  assert cm.routeTrafficSegments == [{'endM': 500, 'status': 'G'}, {'endM': 1500, 'status': 'R'}]
  cm, _ = apply(packet(sequence=2, active=False), 11)
  assert cm.routeTrafficSegments == [] and cm.routeTrafficTotalMeters == 0


def test_traffic_schema_roundtrip_and_web_clear():
  from openpilot.cereal import custom
  from openpilot.selfdrive.carrot.server.live_runtime.snapshot import _build_carrot

  msg = custom.CarrotMan.new_message(
    externalNavigationSource='navdy', externalNavigationActive=True, roadLimitSpeedValid=True,
    routeTrafficTotalMeters=1500,
    routeTrafficSegments=[{'endM': 500, 'status': 'G'}, {'endM': 1500, 'status': 'R'}])
  with custom.CarrotMan.from_bytes(msg.to_bytes()) as decoded:
    payload = _build_carrot(decoded)
  assert payload['externalNavigationSource'] == 'navdy'
  assert payload['externalNavigationActive'] is True
  assert payload['roadLimitSpeedValid'] is True
  assert payload['routeTrafficTotalMeters'] == 1500
  assert payload['routeTrafficSegments'] == [{'endM': 500, 'status': 'G'}, {'endM': 1500, 'status': 'R'}]
  cleared = _build_carrot(custom.CarrotMan.new_message(), previous=payload)
  assert cleared['routeTrafficSegments'] == [] and cleared['routeTrafficTotalMeters'] == 0
  assert cleared['externalNavigationActive'] is False
  assert cleared['roadLimitSpeedValid'] is False


@pytest.mark.parametrize('age,limit,current,expected', [
  (0.1, 80, 80, True), (4.0, 80, 80, True), (4.001, 80, 80, False),
  (-0.01, 80, 80, False), (0.1, 0, 30, False), (0.1, 60, 80, False),
])
def test_legacy_road_limit_has_its_own_freshness(services, monkeypatch, age, limit, current, expected):
  cls, _ = services
  svc = cls()
  monkeypatch.setattr(cls._road_limit_speed_valid.__globals__['time'], 'monotonic', lambda: 100.)
  svc.navi_source = 0
  svc.external_navigation_active = True
  svc.nRoadLimitSpeed = current
  svc.legacy_road_limit_kph = limit
  svc.legacy_road_limit_received_at = 100. - age
  assert svc._road_limit_speed_valid() is expected
  svc.external_navigation_active = False
  assert not svc._road_limit_speed_valid()


def test_default_or_unselected_limit_cannot_enable_set_speed(services):
  cls, _ = services
  svc = cls()
  svc.external_navigation_active = True
  assert svc.nRoadLimitSpeed == 30
  assert not svc._road_limit_speed_valid()
  svc.navi_source = 1
  assert not svc._road_limit_speed_valid()


def test_legacy_limit_updates_do_not_treat_unknown_as_default_30(services, monkeypatch):
  cls, _ = services
  svc = cls()
  monkeypatch.setattr(cls._road_limit_speed_valid.__globals__['time'], 'monotonic', lambda: 100.)
  for _ in range(6):
    svc.update({'carrotIndex': 1, 'nRoadLimitSpeed': 80}, source=0)
  svc._update_navigation_source()
  assert svc._road_limit_speed_valid()
  svc.update({'carrotIndex': 2, 'nRoadLimitSpeed': 0}, source=0)
  assert not svc._road_limit_speed_valid()
  svc.update({'carrotIndex': 3, 'nTBTTurnType': 12}, source=0)
  assert not svc._road_limit_speed_valid()


def test_kisa_limit_is_converted_to_kph_and_invalidated(services, monkeypatch):
  cls, _ = services
  svc = cls()
  svc.is_metric = False
  monkeypatch.setattr(cls._road_limit_speed_valid.__globals__['time'], 'monotonic', lambda: 100.)
  svc.update_kisa({'kisawazeroadspdlimit': 50})
  svc._update_navigation_source()
  assert svc.nRoadLimitSpeed == pytest.approx(80.4672)
  assert svc._road_limit_speed_valid()
  svc.update_kisa({'kisawazeroadspdlimit': 0})
  assert not svc._road_limit_speed_valid()


@pytest.fixture
def network_publication(navigation_publication, monkeypatch):
  svc, values, _ = navigation_publication
  values['CarrotNaviSource'] = svc.navi_source = 0
  clock = [100_000_000_000]
  monkeypatch.setattr(svc._road_limit_speed_valid.__globals__['time'], 'monotonic_ns', lambda: clock[0])
  data = {
    'schemaVersion': 1, 'connected': True, 'sessionId': 'network-session',
    'speed': {
      'meta': {'present': True, 'sequence': 1, 'receivedMonoTimeNanos': clock[0]},
      'roadLimitValid': True, 'roadLimitKph': 80,
      'sdiPresent': True, 'sdiType': 1, 'sdiDistanceM': 420, 'sdiSpeedLimitKph': 60,
    },
  }
  sm = MagicMock()
  sm.alive = {'carState': False, 'selfdriveState': False, 'carrotNavi': True}
  sm.valid = {'carrotNavi': True}
  sm.updated = {'carrotNavi': True}
  sm.__getitem__.return_value = data
  sent = {}
  publisher = SimpleNamespace(send=lambda name, msg: sent.update({name: msg}))

  def publish(now_ns, updated=True):
    clock[0] = now_ns
    sm.updated['carrotNavi'] = updated
    svc.update_navi('network-host', sm, publisher, 200., [], [], 200., 'gpsLocationExternal')
    return sent['carrotMan'].carrotMan

  return svc, data, publish


@pytest.mark.parametrize('received,expected', [
  (None, False), (0, False), (-1, False), ('invalid', False),
  (100_000_000_001, False), (100_000_000_000, True),
  (99_000_000_000, True), (90_000_000_001, True),
  (90_000_000_000, False), (1, False),
])
def test_network_set_limit_uses_original_speed_timestamp(network_publication, received, expected):
  svc, data, publish = network_publication
  if received is None:
    del data['speed']['meta']['receivedMonoTimeNanos']
  else:
    data['speed']['meta']['receivedMonoTimeNanos'] = received
  cm = publish(100_000_000_000)
  assert cm.roadLimitSpeedValid is expected
  assert cm.externalNavigationActive and cm.externalNavigationSource == 'network'
  # The SET boost gate must not change existing road-limit or camera control.
  assert svc.carrot_navi_road_limit_valid and cm.nRoadLimitSpeed == 80
  assert (cm.xSpdType, cm.xSpdDist, cm.xSpdLimit) == (1, 420, 60)


@pytest.mark.parametrize('updated', [False, True])
def test_network_heartbeats_cannot_renew_set_limit(network_publication, updated):
  svc, data, publish = network_publication
  original = publish(100_000_000_000)
  assert original.roadLimitSpeedValid
  assert publish(109_999_999_999, updated=updated).roadLimitSpeedValid
  data['guidanceCurrent'] = {
    'meta': {'present': True, 'sequence': 2, 'receivedMonoTimeNanos': 110_000_000_000},
    'distanceM': 131, 'turnType': 117,
  }
  expired = publish(110_000_000_000, updated=updated)
  assert not expired.roadLimitSpeedValid
  assert svc.carrot_navi_control.speed.received_mono_time_ns == 100_000_000_000
  assert expired.nRoadLimitSpeed == original.nRoadLimitSpeed
  assert expired.desiredSpeed == original.desiredSpeed
  assert (expired.xSpdType, expired.xSpdDist, expired.xSpdLimit) == (1, 420, 60)

  data['speed']['meta'].update(sequence=2, receivedMonoTimeNanos=110_000_000_000)
  assert publish(110_000_000_000).roadLimitSpeedValid


@pytest.fixture
def cruise_auto_speed():
  path = Path(__file__).resolve().parents[2] / 'car/cruise.py'
  tree = ast.parse(path.read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'VCruiseCarrot')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '_auto_speed_up')
  scope = {}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
  return scope['_auto_speed_up']


@pytest.mark.parametrize('model_speed_factor', [-1., 0., 1.])
@pytest.mark.parametrize('road_adjust', [-1., 0., 0.5])
def test_unknown_road_limit_keeps_cruise_setting(cruise_auto_speed, model_speed_factor, road_adjust):
  state = SimpleNamespace(nRoadLimitSpeed=0, _pause_auto_speed_up=False,
                          applyModelSpeed=model_speed_factor, model_v_kph=80.,
                          autoSpeedUptoRoadSpeedLimit=1., autoRoadSpeedAdjust=road_adjust)
  assert cruise_auto_speed(state, 65.) == 65.


@pytest.mark.parametrize(('factor', 'expected'), [(-1., 66.), (0., 65.), (1., 66.)])
def test_valid_road_limit_keeps_existing_model_speed_behavior(cruise_auto_speed, factor, expected):
  state = SimpleNamespace(nRoadLimitSpeed=60, nRoadLimitSpeed_last=60, _pause_auto_speed_up=False,
                          applyModelSpeed=factor, model_v_kph=80., autoSpeedUptoRoadSpeedLimit=1.,
                          autoRoadSpeedAdjust=0., autoRoadSpeedLimitOffset=-1, v_lead_kph=0., d_rel=100.)
  assert cruise_auto_speed(state, 65.) == pytest.approx(expected)
