import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car, custom
from openpilot.selfdrive.carrot import carrot_serv
from openpilot.selfdrive.carrot.deceleration_source import deceleration_candidates


@pytest.mark.parametrize('source', ['cam', 'section', 'bump', 'police', 'waze', 'atc', 'atc2', 'hda',
                                   'hda_bump', 'school', 'hda_section', 'road', 'route', 'vturn'])
def test_candidate_preserves_unrounded_cap_and_endpoint(source):
  values = [(110.75, source), (200, 'road'), (250, 'vturn')]
  targets = {source: 80.25}
  candidates = deceleration_candidates(values[:1], targets)
  assert candidates == [{'source': source, 'speedKph': 110.75, 'targetKph': 80.25, 'distanceM': -1.0}]
  assert values[0] == (110.75, source) and targets[source] == 80.25


def test_distant_valid_event_is_not_a_neutral_cap():
  assert deceleration_candidates([(250., 'cam'), (200., 'road'), (250., 'vturn')], {'cam': 80., 'vturn': 250.}) == [
    {'source': 'cam', 'speedKph': 250., 'targetKph': 80., 'distanceM': -1.0},
  ]
  assert deceleration_candidates([(250., 'cam')], {}) == []
  assert deceleration_candidates([(70., 'vturn')], {'vturn': 0.})[0]['targetKph'] == 0.


@pytest.mark.parametrize('speed,target', [(0, 80), (-10, 80), (300, 80), (float('nan'), 80), (90, float('nan'))])
def test_invalid_candidate_is_not_a_display_event(speed, target):
  assert deceleration_candidates([(speed, 'cam')], {'cam': target}) == []


def test_display_candidates_survive_real_schema_round_trip_and_old_messages_are_empty():
  payload = deceleration_candidates([(110.75, 'cam'), (70, 'vturn')], {'cam': 80.25, 'vturn': 0.})
  message = custom.CarrotMan.new_message(desiredSource='cam', desiredSpeed=110, decelerationCandidates=payload)
  with custom.CarrotMan.from_bytes(message.to_bytes()) as reader:
    assert reader.desiredSpeed == 110 and reader.desiredSource == 'cam'
    assert reader.to_dict()['decelerationCandidates'] == payload
  assert list(custom.CarrotMan.new_message().decelerationCandidates) == []


def make_serv(monkeypatch, cls=None, **settings):
  values = {'AutoCurveSpeedLowerLimit': 20, 'MapTurnSpeedFactor': 100, 'IsMetric': 1,
            'AutoNaviSpeedCtrlMode': 3, 'AutoNaviSpeedCtrlEnd': 2, 'AutoNaviSpeedDecelRate': 100,
            'AutoNaviSpeedSafetyFactor': 100, 'AutoNaviSpeedBumpSpeed': 20, 'AutoNaviSpeedBumpTime': 2,
            'AutoTurnControlSpeedTurn': 30, 'AutoTurnControlTurnEnd': 4,
            'VehicleNaviCanControl': 1, 'VehicleSpeedCameraControlMode': 1, 'VehicleNaviSchoolZoneControl': 1,
            'AutoRoadSpeedLimitOffset': -1, **settings}

  class Params:
    def __init__(self, *_args):
      pass

    def get(self, key):
      return str(values.get(key, 0))

    def get_int(self, key):
      return int(self.get(key))

    def get_float(self, key):
      return float(self.get(key))

    def get_bool(self, key):
      return bool(self.get_int(key))

  cls = cls or carrot_serv.CarrotServ
  monkeypatch.setitem(cls.__init__.__globals__, 'Params', Params)
  serv = cls()
  for name, replacement in {
    '_update_carrot_navi': lambda _sm: False,
    '_publish_model_route': lambda *_a: NS(generation=0),
    '_update_gps': lambda *_a: 0.,
    'update_nav_instruction': lambda *_a: None,
    '_update_navigation_source': lambda: False,
    '_road_limit_speed_valid': lambda: True,
  }.items():
    monkeypatch.setattr(serv, name, replacement)

  class SubMaster(dict):
    alive = {'carState': True, 'selfdriveState': True, 'navInstruction': False}
    updated = {'navRouteNavd': False}

  cs = car.CarState.new_message(vEgo=30., vCruise=120.)
  sm = SubMaster(carState=cs, selfdriveState=NS(distanceTraveled=0.))
  sent = {}
  pm = NS(send=lambda name, message: sent.update({name: message}))
  return serv, sm, pm, sent


def publish(serv, sm, pm, sent, **kwargs):
  serv.update_navi('', sm, pm, 250., [], [], 300., 'gpsLocationExternal', **kwargs)
  return sent['carrotMan'].carrotMan


def external_event(serv, kind=1, speed=80., distance=500.):
  serv.external_navigation_active = True
  serv.active_count = serv.active_sdi_count = 100
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = kind, speed, distance


@pytest.mark.parametrize('kind,source,final', [(1, 'cam', 80), (4, 'section', 80), (22, 'bump', 20),
                                            (100, 'police', 80), (101, 'waze', 80)])
def test_external_sources_publish_each_existing_endpoint(monkeypatch, kind, source, final):
  serv, sm, pm, sent = make_serv(monkeypatch)
  external_event(serv, kind, final)
  result = publish(serv, sm, pm, sent)
  candidate = result.decelerationCandidates[0]
  assert (candidate.source, candidate.targetKph) == (source, final)
  expected = final if kind == 4 else serv.calculate_current_speed(500, final, 2, 1)
  assert candidate.speedKph == pytest.approx(expected)
  assert result.desiredSpeed == min(200, int(expected))


def test_camera_disappears_and_new_camera_does_not_inherit_old_endpoint(monkeypatch):
  serv, sm, pm, sent = make_serv(monkeypatch)
  for speed, distance in [(80, 50), (60, 400), (60, 0)]:
    external_event(serv, speed=speed, distance=distance)
    result = publish(serv, sm, pm, sent)
    if distance:
      assert result.decelerationCandidates[0].targetKph == speed
    else:
      assert not list(result.decelerationCandidates)


@pytest.mark.parametrize('source,field,value,final', [
  ('hda', 'speedLimit', 80, 88), ('hda_bump', 'speedBumpDistance', 200, 20),
  ('school', 'schoolZoneActive', True, 30), ('hda_section', 'vehicleNaviSectionActive', True, 88),
])
def test_vehicle_sources_publish_safety_adjusted_or_configured_endpoint(monkeypatch, source, field, value, final):
  serv, sm, pm, sent = make_serv(monkeypatch, AutoNaviSpeedSafetyFactor=110)
  cs = sm['carState']
  cs.vehicleNaviActive = True
  cs.vehicleNaviAvailable = True
  cs.vehicleNaviSpeed = 80
  if source == 'hda':
    cs.speedLimitDistance = 500
  setattr(cs, field, value)
  result = publish(serv, sm, pm, sent)
  candidates = {candidate.source: candidate for candidate in result.decelerationCandidates}
  assert candidates[source].targetKph == pytest.approx(final)


@pytest.mark.parametrize('enabled', [False, True])
def test_turn_candidates_use_their_own_final_targets_and_setting_gate(monkeypatch, enabled):
  serv, sm, pm, sent = make_serv(monkeypatch, AutoTurnControl=2 if enabled else 0)
  external_event(serv)
  serv.xSpdDist = 0
  serv.xTurnInfo, serv.xDistToTurn = 1, 300
  serv.xTurnInfoNext, serv.xDistToTurnNext = 7, 500
  result = publish(serv, sm, pm, sent)
  candidates = {candidate.source: candidate for candidate in result.decelerationCandidates}
  if enabled:
    assert candidates['atc'].targetKph == serv.atcSpeed
    assert candidates['atc2'].targetKph != candidates['atc'].targetKph
  else:
    assert candidates == {}


def test_road_and_curve_sentinels_are_hidden_but_real_road_limit_is_published(monkeypatch):
  serv, sm, pm, sent = make_serv(monkeypatch, AutoRoadSpeedLimitOffset=10, TurnSpeedControlMode=2)
  assert not list(publish(serv, sm, pm, sent).decelerationCandidates)
  external_event(serv)
  serv.xSpdDist = 0
  serv.nRoadLimitSpeed = 80
  result = publish(serv, sm, pm, sent)
  assert result.to_dict()['decelerationCandidates'] == [{'source': 'road', 'speedKph': 90., 'targetKph': 90., 'distanceM': -1.0}]


def test_curve_endpoints_are_scaled_with_their_caps_without_changing_winner(monkeypatch):
  serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=2, MapTurnSpeedFactor=80)
  serv.update_navi('', sm, pm, 100., [], [], 90., 'gpsLocationExternal', vturn_target=70., route_target=60.)
  result = sent['carrotMan'].carrotMan
  candidates = {candidate.source: candidate for candidate in result.decelerationCandidates}
  assert (result.desiredSource, result.desiredSpeed) == ('route', 72)
  assert (candidates['vturn'].speedKph, candidates['vturn'].targetKph) == (100., 70.)
  assert (candidates['route'].speedKph, candidates['route'].targetKph) == (72., 48.)


def man_method(name, namespace):
  # Exercise the production method without importing unrelated native servers.
  path = Path(carrot_serv.__file__).with_name('carrot_man.py')
  tree = ast.parse(path.read_text())
  klass = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'CarrotMan')
  method = next(node for node in klass.body if isinstance(node, ast.FunctionDef) and node.name == name)
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace[name]


@pytest.mark.parametrize('cap,raw_cap,final,expected', [(100, 100, 70, 70), (-100, 100, 70, 70),
                                                     (70, 100, 95, 0), (70, 70, 70, 70)])
def test_vision_endpoint_belongs_to_current_cap_not_held_other_geometry(cap, raw_cap, final, expected):
  method = man_method('vturn_speed', {'time': NS(monotonic=lambda: 1.),
                                    'curve_speed': lambda *_a, **_kw: NS(approach_kph=raw_cap, curve_kph=final)})
  man = NS(autoCurveSpeedFactor=1., carrot_serv=NS(autoCurveSpeedLowerLimit=20),
           vision_curve_speed=NS(update=lambda *_a, **_kw: cap), vturn_display_target=123.)
  class SM(dict):
    logMonoTime = {'modelV2': 1}
  cs = NS(vEgo=20, vCluRatio=1., aEgo=0.)
  assert method(man, cs, SM(modelV2=None)) == cap
  assert man.vturn_display_target == expected


def test_missing_model_clears_endpoint_without_changing_held_control_cap():
  method = man_method('carrot_curve_speed', {'time': NS(monotonic=lambda: 1.)})
  man = NS(carrot_curve_speed_params=lambda: None, vturn_display_target=80.,
           vision_curve_speed=NS(update=lambda *_a: 100.))
  sm = NS(alive={'carState': True, 'modelV2': False}, valid={'carState': True, 'modelV2': False})
  assert method(man, sm) == 100.
  assert man.vturn_display_target == 0.


@pytest.mark.parametrize('curvatures', [[0., 0., 1., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]])
def test_route_endpoint_tracks_selected_downstream_target_and_clears_offroad(curvatures):
  curves = iter(curvatures)
  line = NS(length=10 * (len(curvatures) + 7), interpolate=lambda d: NS(x=d, y=0))
  method = man_method('carrot_navi_route', {
    'np': np, 'SHAPELY_AVAILABLE': True, 'LineString': lambda _p: line,
    'get_path_after_distance': lambda *_a: ([(0, 0), (300, 0)], 0, (0, 0)),
    'gps_to_relative_xy': lambda *a: a[0], 'calculate_curvature': lambda *_a: next(curves),
    'V_CURVE_LOOKUP_BP': [0., 1.], 'V_CRUVE_LOOKUP_VALS': [100., 20.],
  })
  man = NS(params=NS(get_bool=lambda _k: True), navd_active=True, navi_points_active=True,
           navi_points=[], navi_points_start_index=0, active_carrot_last=2, sm={'carState': NS(vEgo=0.)},
           carrot_serv=NS(active_carrot=2, vpPosPointLon=0., vpPosPointLat=0., bearing=0.,
                          nRoadLimitSpeed=0, autoNaviSpeedDecelRate=1., autoNaviSpeedCtrlEnd=2.))
  _, _, cap = method(man)
  assert man.route_display_target == 20.
  assert cap >= man.route_display_target
  assert (cap == 20.) == (curvatures[0] == 1.)
  man.params.get_bool = lambda _k: False
  assert method(man) == ([], [], 300)
  assert man.route_display_target == 0.
