from dataclasses import replace
import sys
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.tests.test_navigation_source import services as services, packet
from openpilot.selfdrive.carrot.carrot_navi_control import parse_carrot_navi_control
from openpilot.selfdrive.navigation.helpers import navigation_input
from openpilot.selfdrive.navigation.provenance import RoutePublication, context_ready
from openpilot.selfdrive.navigation.tests.test_map_client import setup as setup, assert_zero


@pytest.mark.parametrize('field,value', [('instructionMonoTime', 0), ('instructionMonoTime', 7000000000),
                                       ('instructionMonoTime', 11000000000), ('distanceMonoTime', 0),
                                       ('distanceMonoTime', 9600000000), ('carrotRouteId', 124),
                                       ('navigationSource', 'navdy'), ('navigationSource', '')])
def test_heartbeat_does_not_renew_observation_or_change_route(setup, field, value):
  _, context, _, model, reader = setup
  assert reader.update(model)['nav_map'].any()
  context.recv_time = dict.fromkeys(context.recv_time, 10.)
  setattr(context['navInstructionCarrot'], field, value)
  assert_zero(reader.update(model))


def test_ego_projection_does_not_double_subtract_original_guidance_age(setup):
  _, context, _, model, reader = setup
  context['navInstructionCarrot'].instructionMonoTime = 9000000000
  context['navInstructionCarrot'].distanceMonoTime = 10000000000
  assert reader.update(model, 10.)['navigation'][0, -1] == pytest.approx(1.2)


POINTS = [{'latitude': 37., 'longitude': 127.}, {'latitude': 37.01, 'longitude': 127.01}]


@pytest.mark.parametrize('source,count,accepted', [('navdy', 5627, True), ('navdy', 8192, True),
                                                  ('navdy', 8193, False), ('network-v2', 4097, False)])
def test_route_publication_preserves_all_points_with_source_bound(source, count, accepted):
  points = [{'latitude': 37. + i * .00001, 'longitude': 127.} for i in range(count)]
  routes = RoutePublication()
  routes.offer(source, points, session='full-path')
  selected = routes.choose(source)
  assert len(selected.points) == (count if accepted else 0)
  if accepted:
    assert selected.points == tuple((p['latitude'], p['longitude']) for p in points)


def test_geometry_and_session_generations_are_stable_on_heartbeat(monkeypatch):
  clock = [10_000_000_000]
  monkeypatch.setattr('time.monotonic_ns', lambda: clock[0])
  routes = RoutePublication()
  routes.offer('network-v2', POINTS, session='first', observed=9_000_000_000)
  original = routes.choose('network-v2')
  clock[0] += 1_000_000_000
  routes.offer('network-v2', POINTS, session='first', observed=clock[0])
  assert routes.choose('network-v2') == original
  routes.offer('network-v2', POINTS, session='restarted', observed=clock[0])
  assert routes.choose('network-v2').generation != original.generation
  routes.choose('navdy')
  assert not routes.choose('network-v2').generation


def test_mapbox_resend_keeps_id_but_missing_provenance_cannot_enable_model():
  routes = RoutePublication()
  routes.offer('mapbox', POINTS, generation=123)
  assert routes.choose('mapbox').generation == 123
  routes.offer('mapbox', POINTS, generation=123)
  assert routes.choose('mapbox').generation == 123
  routes.offer('mapbox', POINTS)
  assert not routes.choose('mapbox').points


@pytest.fixture
def publisher(monkeypatch, services):
  clock = [10.]
  monkeypatch.setattr('time.monotonic', lambda: clock[0])
  monkeypatch.setattr('time.monotonic_ns', lambda: int(clock[0] * 1e9))
  def new_message(which, valid=False):
    event = log.Event.new_message()
    event.init(which)
    event.valid = valid
    return event
  monkeypatch.setitem(sys.modules, 'openpilot.cereal.messaging', NS(new_message=new_message))
  import openpilot.cereal
  monkeypatch.setattr(openpilot.cereal, 'messaging', sys.modules['openpilot.cereal.messaging'], raising=False)
  cls, values = services
  values['CarrotNaviSource'] = 0
  svc = cls()
  events = {}
  pm = NS(send=lambda topic, event: events.__setitem__(topic, event))
  sm = NS(updated={'navRouteNavd': False})
  return clock, svc, sm, pm, events, new_message


def control(*, session='first', stamp=9_000_000_000):
  meta = {'present': True, 'sequence': 1, 'receivedMonoTimeNanos': stamp}
  return parse_carrot_navi_control({'schemaVersion': 1, 'connected': True, 'sessionId': session,
      'route': {'meta': meta, 'polyline': POINTS},
      'guidanceCurrent': {'meta': meta, 'turnType': 12, 'distanceM': 120}})


def test_actual_external_producer_pairs_route_and_original_instruction(publisher):
  clock, svc, sm, pm, events, new_message = publisher
  svc.carrot_navi_active = True
  svc.carrot_navi_control = control()
  svc._apply_carrot_navi_guidance(svc.carrot_navi_control, force=True)
  route = svc._publish_model_route(sm, pm)
  inst = new_message('navInstructionCarrot', valid=True)
  svc._stamp_model_instruction(inst, route)
  assert inst.navInstructionCarrot.carrotRouteId == events['navRoute'].navRoute.carrotRouteId > 0
  assert inst.navInstructionCarrot.navigationSource == 'network-v2'
  assert inst.navInstructionCarrot.instructionMonoTime == 9_000_000_000
  clock[0] = 10.5
  svc._apply_carrot_navi_guidance(svc.carrot_navi_control)
  svc._stamp_model_instruction(inst, svc._publish_model_route(sm, pm))
  assert inst.navInstructionCarrot.instructionMonoTime == 9_000_000_000
  assert inst.navInstructionCarrot.distanceMonoTime == 9_000_000_000
  assert inst.navInstructionCarrot.allManeuvers[0].distance == 120.


def test_new_route_does_not_take_previous_guidance_or_other_source(publisher):
  _, svc, sm, pm, _, new_message = publisher
  svc.carrot_navi_active = True
  svc._apply_carrot_navi_guidance(control(stamp=9_000_000_000), force=True)
  svc.carrot_navi_control = control(stamp=9_500_000_000)
  route = svc._publish_model_route(sm, pm)
  inst = new_message('navInstructionCarrot', valid=True)
  svc._stamp_model_instruction(inst, route)
  assert not inst.navInstructionCarrot.carrotRouteId
  svc.navi_source = 1
  svc.carrot_navi_control = None
  svc.carrot_navi_active = False
  assert not svc._publish_model_route(sm, pm).generation


def test_navdy_without_whole_route_does_not_substitute_network_route(publisher):
  _, svc, sm, pm, _, _ = publisher
  svc.model_routes.offer('mapbox', POINTS, generation=123)
  svc.navi_source = 1
  navi = control()
  svc.carrot_navi_active = True
  svc.carrot_navi_control = replace(navi, route=replace(navi.route, polyline=()))
  selected = svc._publish_model_route(sm, pm)
  assert selected.source == 'navdy' and not selected.generation and not selected.points


def test_navdy_verified_geometry_and_native_maneuver_reach_model_inputs(publisher, monkeypatch):
  from openpilot.selfdrive.navdy.tests.test_model_navigation import tracker, packet
  clock, svc, sm, pm, events, new_message = publisher
  svc.navi_source = 1
  source = tracker()
  monkeypatch.setattr(svc.navdy_navigation, 'model_read', lambda: source.snapshot(clock[0]))
  route = svc._publish_model_route(sm, pm)
  inst = new_message('navInstructionCarrot', valid=False)
  svc._stamp_model_instruction(inst, route, 10.)
  assert inst.valid
  assert inst.navInstructionCarrot.carrotRouteId == events['navRoute'].navRoute.carrotRouteId > 0
  assert inst.navInstructionCarrot.navigationSource == 'navdy'
  assert inst.navInstructionCarrot.instructionMonoTime == 9_900_000_000
  assert navigation_input(inst.navInstructionCarrot.allManeuvers, 1.)[15] == 1.
  old_generation = route.generation
  clock[0] = 10.2
  source.accept(packet(sequence=2, route='rerouted'), 10.2, 10.2)
  route = svc._publish_model_route(sm, pm)
  assert route.generation != old_generation
  source.accept({**packet(sequence=3), 'active': False}, 10.3, 10.3)
  clock[0] = 10.3
  route = svc._publish_model_route(sm, pm)
  svc._stamp_model_instruction(inst, route)
  assert not route.generation and not inst.navInstructionCarrot.carrotRouteId


def test_navdy_native_observation_expires_without_renewal_by_route_publication(publisher, monkeypatch):
  from openpilot.selfdrive.navdy.tests.test_model_navigation import tracker
  clock, svc, sm, pm, _, new_message = publisher
  svc.navi_source = 1
  source = tracker()
  monkeypatch.setattr(svc.navdy_navigation, 'model_read', lambda: source.snapshot(clock[0]))
  route = svc._publish_model_route(sm, pm)
  clock[0] = 12.
  inst = new_message('navInstructionCarrot', valid=True)
  svc._stamp_model_instruction(inst, route)
  assert not inst.navInstructionCarrot.carrotRouteId
  assert not svc._publish_model_route(sm, pm).generation


def start_external(publisher, navi):
  _, svc, sm, pm, _, _ = publisher
  svc.carrot_navi_active = True
  svc.carrot_navi_control = navi
  svc._apply_carrot_navi_guidance(navi, force=True)
  return svc._publish_model_route(sm, pm)


def instruction_context(publisher, route, speed=0.):
  clock, svc, _, _, events, new_message = publisher
  inst = new_message('navInstructionCarrot', valid=True)
  inst.navInstructionCarrot.maneuverPrimaryText = 'display remains available'
  svc._stamp_model_instruction(inst, route, speed)
  class Context(dict):
    def update(self, _timeout):
      pass
  context = Context(navRoute=events['navRoute'].navRoute, navInstructionCarrot=inst.navInstructionCarrot)
  context.valid = dict.fromkeys(context, True)
  context.recv_time = dict.fromkeys(context, clock[0])
  return inst.navInstructionCarrot, context


def test_legacy_guidance_cannot_be_stamped_as_selected_v2(publisher):
  clock, svc, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  route = start_external(publisher, navi)
  svc.update(packet(nTBTTurnType=13, nTBTDist=40))
  svc._apply_carrot_navi_guidance(navi)  # The unchanged v2 heartbeat is not a new observation.
  instruction, context = instruction_context(publisher, route)
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])
  assert (svc.nTBTTurnType, svc.xDistToTurn) == (13, 40)
  assert instruction.maneuverPrimaryText == 'display remains available'


@pytest.mark.parametrize('source,session', [('network-v2', 'restarted'), ('network-v1', ''), ('navdy', 'first')])
def test_guidance_must_match_actual_source_and_session(publisher, source, session):
  clock, _, _, _, _, _ = publisher
  route = start_external(publisher, control())
  instruction, context = instruction_context(publisher, replace(route, source=source, session=session))
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])


@pytest.mark.parametrize('next_stamp', [0, 2_000_000_000, 8_000_000_000, 9_400_000_000, 11_000_000_000])
def test_next_guidance_cannot_borrow_current_freshness_or_new_route(publisher, next_stamp):
  clock, _, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  navi = replace(navi, current=replace(navi.current, distance_m=1),
                 next=replace(navi.current, turn_type=13, distance_m=100, received_mono_time_ns=next_stamp))
  route = start_external(publisher, navi)
  instruction, context = instruction_context(publisher, route, speed=10.)
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])


def test_fresh_current_remains_usable_when_next_is_stale(publisher):
  clock, _, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  navi = replace(navi, next=replace(navi.current, turn_type=13, distance_m=400, received_mono_time_ns=2_000_000_000))
  route = start_external(publisher, navi)
  instruction, context = instruction_context(publisher, route)
  assert context_ready(context, clock[0])
  assert len(instruction.allManeuvers) == 1
  assert navigation_input(instruction.allManeuvers, 0.)[15] == 1.


def test_next_keeps_its_own_observation_and_distance_clock(publisher):
  clock, svc, sm, pm, _, _ = publisher
  navi = control(stamp=9_000_000_000)
  navi = replace(navi, current=replace(navi.current, distance_m=1, received_mono_time_ns=9_800_000_000),
                 next=replace(navi.current, turn_type=13, distance_m=100, received_mono_time_ns=9_200_000_000))
  route = start_external(publisher, navi)
  for now in (10., 10.5):
    clock[0] = now
    svc._apply_carrot_navi_guidance(navi)
    instruction, context = instruction_context(publisher, svc._publish_model_route(sm, pm), speed=10.)
    assert context_ready(context, now)
    assert instruction.instructionMonoTime == instruction.distanceMonoTime == 9_200_000_000
    assert instruction.allManeuvers[0].distance == 100.
    output = navigation_input(instruction.allManeuvers, 10. * (now - instruction.distanceMonoTime / 1e9))
    assert output[10] == 1. and output[-1] == pytest.approx((100. - 10. * (now - 9.2)) / 100.)
  clock[0] = 11.2
  instruction, context = instruction_context(publisher, route, speed=10.)
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])


def test_legacy_next_does_not_survive_a_v2_current_only_update(publisher):
  clock, svc, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  start_external(publisher, navi)
  svc.update(packet(nTBTTurnType=12, nTBTDist=1, nTBTTurnTypeNext=13, nTBTDistNext=100))
  clock[0] = 10.2
  navi = replace(navi, current=replace(navi.current, sequence=2, distance_m=1, received_mono_time_ns=10_100_000_000))
  svc._apply_carrot_navi_guidance(navi)
  _, _, sm, pm, _, _ = publisher
  svc.carrot_navi_control = navi
  route = svc._publish_model_route(sm, pm)
  instruction, context = instruction_context(publisher, route, speed=20.)
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])
  assert svc.nTBTTurnTypeNext == 13  # Existing display/control state is not rewritten.


@pytest.mark.parametrize('kind', [-1, 0, 999])
def test_unknown_current_never_becomes_phantom_straight_or_skips_to_next(publisher, kind):
  clock, _, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  navi = replace(navi, current=replace(navi.current, turn_type=kind),
                 next=replace(navi.current, turn_type=13, distance_m=400))
  route = start_external(publisher, navi)
  instruction, context = instruction_context(publisher, route)
  assert not instruction.carrotRouteId and not context_ready(context, clock[0])


def test_model_stamp_is_cleared_when_reusing_an_instruction_after_expiry(publisher):
  clock, svc, _, _, _, new_message = publisher
  route = start_external(publisher, control(stamp=9_000_000_000))
  inst = new_message('navInstructionCarrot', valid=True)
  svc._stamp_model_instruction(inst, route)
  assert inst.navInstructionCarrot.carrotRouteId
  clock[0] = 11.
  svc._stamp_model_instruction(inst, route)
  assert not inst.navInstructionCarrot.carrotRouteId
  assert not inst.navInstructionCarrot.instructionMonoTime


@pytest.mark.parametrize('failure', ['legacy', 'stale_next', 'none'])
def test_update_navi_preserves_display_and_controls_while_gating_model(publisher, monkeypatch, failure):
  _, svc, _, pm, events, new_message = publisher
  navi = control(stamp=9_500_000_000)
  if failure == 'stale_next':
    navi = replace(navi, current=replace(navi.current, turn_type=201),
                   next=replace(navi.current, turn_type=13, distance_m=400, received_mono_time_ns=2_000_000_000))
  start_external(publisher, navi)
  if failure == 'legacy':
    svc.update(packet(nTBTTurnType=13, nTBTDist=40, nSdiType=-1))
  svc.active_count = 80
  svc.active_sdi_count = 200
  monkeypatch.setattr(svc, '_update_carrot_navi', lambda _sm: True)
  monkeypatch.setattr(svc, '_update_gps', lambda *_args: 0.)
  monkeypatch.setattr(svc, 'update_auto_turn', lambda *_args: (250., 'none', 250., 0.))
  monkeypatch.setitem(svc.update_navi.__globals__, 'messaging', NS(new_message=new_message))
  sm = NS(updated={'navRouteNavd': False}, alive={'carState': False, 'selfdriveState': False, 'navInstruction': False})
  display = (svc.nTBTTurnType, svc.nTBTDist, svc.xDistToTurn, svc.xTurnInfo)
  svc.update_navi('', sm, pm, 200., [], [], 200., 'gpsLocationExternal')
  instruction = events['navInstructionCarrot'].navInstructionCarrot
  assert (svc.nTBTTurnType, svc.nTBTDist, svc.xDistToTurn, svc.xTurnInfo) == display
  assert instruction.maneuverDistance == svc.nTBTDist
  assert events['carrotMan'].carrotMan.xTurnInfo == svc.xTurnInfo
  assert bool(instruction.carrotRouteId) == (failure == 'none')


@pytest.mark.parametrize('failure', ['legacy', 'stale_next'])
def test_rejected_producer_guidance_zeros_real_model_adapter(setup, publisher, monkeypatch, failure):
  from openpilot.selfdrive.navigation import map_client
  _, svc, _, _, _, _ = publisher
  navi = control(stamp=9_500_000_000)
  if failure == 'stale_next':
    navi = replace(navi, current=replace(navi.current, distance_m=1),
                   next=replace(navi.current, turn_type=13, distance_m=100, received_mono_time_ns=2_000_000_000))
  route = start_external(publisher, navi)
  if failure == 'legacy':
    svc.update(packet(nTBTTurnType=13, nTBTDist=40))
  _, context = instruction_context(publisher, route, speed=10.)
  monkeypatch.setattr(map_client, '_make_context', lambda: context)
  _, _, _, model, reader = setup
  assert_zero(reader.update(model, 10.))


def test_phone_gps_only_does_not_suppress_mapbox_or_renew_navigation(services):
  cls, _ = services
  svc = cls()
  svc.update({'latitude': 37., 'longitude': 127., 'heading': 0., 'accuracy': 5.})
  assert not svc._external_navigation_connected()
  assert svc._gps_sources().select(__import__('time').monotonic())[0] == 'phone'


@pytest.mark.parametrize('bad', ['oversize', 'invalid'])
def test_cereal_does_not_send_truncated_or_repaired_route_geometry(bad):
  from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload
  points = [{'lat': 37., 'lon': 127.}] * 4097 if bad == 'oversize' else [
    {'lat': 37., 'lon': 127.}, {'lat': float('nan'), 'lon': 127.}, {'lat': 37.01, 'lon': 127.01}]
  payload = build_carrot_navi_payload({'items': {'route': {'present': True, 'value': {'polyline': points}}}})
  assert payload['route']['polyline'] == []
