"""Real navd and cereal schemas; only platform IPC/Params and HTTP are doubled."""
from concurrent.futures import Future, ThreadPoolExecutor
import copy
import importlib.util
import json
import logging
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.navigation import helpers as position_helpers


class SM(dict):
  def update(self, _):
    pass


class ManualExecutor:
  def __init__(self, **kwargs):
    assert kwargs['max_workers'] == 1
    self.calls = []

  def submit(self, function, *args):
    future = Future()
    future.set_running_or_notify_cancel()
    self.calls.append((function, args, future))
    return future


@pytest.fixture
def engine(monkeypatch):
  values = {'CarrotNaviSource': 0, 'MapboxPublicKey': 'pk.test',
            'NavDestination': json.dumps({'latitude': 37.01, 'longitude': 127.01})}
  removed, published = [], []
  def remove(key):
    removed.append(key)
    values.pop(key, None)
  params = NS(get=values.get, get_int=lambda key: int(values.get(key, 0)), remove=remove)
  clock = [100.]
  clock_api = NS(monotonic=lambda: clock[0], monotonic_ns=lambda: int(clock[0] * 1e9))
  monkeypatch.setattr(position_helpers, 'time', clock_api)
  monkeypatch.delenv('MAPBOX_TOKEN', raising=False)

  def new_message(service, **kwargs):
    message = log.Event.new_message(logMonoTime=clock_api.monotonic_ns(), **kwargs)
    message.init(service)
    return message

  # Load the production modules under fixture-owned names; do not leave a fake
  # Params or messaging module cached for unrelated tests in this process.
  import openpilot.cereal as cereal
  for name, attrs in {
    'openpilot.common.params': {'Params': lambda: params},
    'openpilot.common.realtime': {'Ratekeeper': None},
    'openpilot.common.swaglog': {'cloudlog': logging.getLogger('test.navd')},
    'openpilot.cereal.messaging': {'new_message': new_message},
  }.items():
    module = ModuleType(name)
    module.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, module)
    if name == 'openpilot.cereal.messaging':
      monkeypatch.setattr(cereal, 'messaging', module, raising=False)
  navd_path = Path(__file__).parents[2] / 'navd'
  def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, navd_path / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
  route_helpers = load('_test_navd_helpers', 'helpers.py')
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.navd.helpers', route_helpers)
  navd = load('_test_navd', 'navd.py')
  monkeypatch.setattr(navd, 'ThreadPoolExecutor', ManualExecutor)
  monkeypatch.setattr(navd, 'time', clock_api)
  sm = SM(carrotMan=NS(activeCarrot=0, positionFixMonoTime=0, positionMonoTime=0,
                      xPosLat=37., xPosLon=127., xPosAngle=45., xPosSpeed=0.),
          gpsLocation=NS(latitude=37., longitude=127., bearingDeg=45., speed=0., hasFix=True),
          gpsLocationExternal=NS(hasFix=False),
          carState=NS(gearShifter='drive', standstill=True, vEgo=0.),
          managerState=NS(processes=[]))
  sm.valid = dict.fromkeys(sm, True)
  sm.alive = dict.fromkeys(sm, True)
  sm.updated = dict.fromkeys(sm, False)
  sm.recv_time = dict.fromkeys(sm, clock[0])
  sm.logMonoTime = dict.fromkeys(sm, int(clock[0] * 1e9))
  route_engine = navd.RouteEngine(sm, NS(send=lambda name, msg: published.append((name, msg))))
  route_engine.update_location()
  route_engine.sync_route_context()
  return NS(engine=route_engine, navd=navd, helpers=route_helpers, values=values, removed=removed,
            published=published, clock=clock, sm=sm)


def response():
  return {'routes': [{'legs': [{'annotation': {'maxspeed': [{'speed': 50, 'unit': 'km/h'}]}, 'steps': [
    {'distance': 100., 'duration': 12., 'geometry': {'coordinates': [[127., 37.], [127.01, 37.01]]},
     'bannerInstructions': [{'distanceAlongGeometry': 100., 'primary': {'type': 'turn', 'modifier': 'right', 'text': 'Right'}}]},
  ]}]}]}


def start(ctx):
  destination, _ = ctx.helpers.coordinate_from_param('NavDestination', ctx.engine.params)
  ctx.engine.calculate_route(destination)
  assert ctx.engine.pending is not None
  return ctx.engine.pending


def accept(ctx):
  start(ctx).set_result(response())
  ctx.engine.poll_route()
  assert ctx.engine.route_id > 0


def routes(ctx):
  return [msg for name, msg in ctx.published if name == 'navRouteNavd']


def instructions(ctx):
  return [msg for name, msg in ctx.published if name == 'navInstruction']


def test_pending_network_never_blocks_or_emits_old_instructions(engine):
  pending = start(engine)
  engine.engine.poll_route()
  engine.engine.send_instruction()
  assert not instructions(engine)[-1].valid
  pending.set_result(response())
  engine.engine.poll_route()
  assert engine.engine.step_idx == 0
  assert engine.engine.route_geometry[0][0].annotations['maxspeed'] == pytest.approx(50 / 3.6)


@pytest.mark.parametrize('change', ['destination', 'clear', 'key', 'no_key', 'waypoints', 'source', 'external', 'external_destination'])
def test_late_result_is_discarded_after_user_changes(engine, change):
  start(engine).set_result(response())
  if change == 'destination':
    engine.values['NavDestination'] = json.dumps({'latitude': 38., 'longitude': 128.})
  elif change == 'clear':
    engine.values['NavDestination'] = ''
  elif change in ('key', 'no_key'):
    engine.values['MapboxPublicKey'] = 'pk.new' if change == 'key' else ''
  elif change == 'waypoints':
    engine.values['NavDestinationWaypoints'] = '[[127,37]]'
  elif change == 'source':
    engine.values['CarrotNaviSource'] = 1
  elif change == 'external':
    engine.sm['carrotMan'].activeCarrot = 2
  else:
    engine.values['NavDestination'] = json.dumps({'latitude': 37.01, 'longitude': 127.01, 'place_name': 'External Navi'})
  engine.engine.poll_route()
  assert engine.engine.route is None and engine.engine.pending is None
  assert not any(msg.valid for msg in routes(engine))
  assert 'NavDestination' not in engine.removed


@pytest.mark.parametrize('change', ['source', 'key', 'destination', 'external', 'waypoints'])
def test_change_away_and_back_cannot_revive_inflight_generation(engine, change):
  pending = start(engine)
  original = engine.values.copy()
  if change == 'source':
    engine.values['CarrotNaviSource'] = 1
  elif change == 'key':
    engine.values['MapboxPublicKey'] = 'pk.new'
  elif change == 'destination':
    engine.values['NavDestination'] = ''
  elif change == 'external':
    engine.sm['carrotMan'].activeCarrot = 1
  else:
    engine.values['NavDestinationWaypoints'] = '[[127,37]]'
  engine.engine.sync_route_context()
  engine.values.clear()
  engine.values.update(original)
  engine.sm['carrotMan'].activeCarrot = 0
  engine.engine.sync_route_context()
  pending.set_result(response())
  engine.engine.poll_route()
  assert engine.engine.route is None and not routes(engine)


@pytest.mark.parametrize('active', [0, 1, 2])
@pytest.mark.parametrize('connected', [False, True])
def test_explicit_navdy_never_falls_back_to_mapbox(engine, active, connected):
  engine.values['CarrotNaviSource'] = 1
  engine.sm['carrotMan'].activeCarrot = active
  engine.sm.alive['carrotMan'] = connected
  engine.engine.update()
  engine.engine.send_route()
  assert not engine.engine.executor.calls
  assert not any(msg.valid for _, msg in engine.published)
  assert not engine.removed


@pytest.mark.parametrize('active,valid,alive,allowed', [(0, True, True, True), (1, True, True, False),
                                                       (2, True, True, False), (1, False, True, True), (1, True, False, True)])
def test_network_source_respects_active_external_owner(engine, active, valid, alive, allowed):
  engine.sm['carrotMan'].activeCarrot = active
  engine.sm.valid['carrotMan'], engine.sm.alive['carrotMan'] = valid, alive
  engine.engine.update()
  assert bool(engine.engine.executor.calls) is allowed


@pytest.mark.parametrize('source', [1, 2, -1])
def test_source_switch_clears_before_pending_poll_and_periodic_resend(engine, source):
  accept(engine)
  pending = start(engine)
  pending.set_result(response())
  engine.published.clear()
  engine.values['CarrotNaviSource'] = source
  engine.engine.resend_at = engine.clock[0] - 1.
  engine.engine.update()
  assert engine.engine.route_id == 0 and engine.engine.pending is None
  assert routes(engine) and instructions(engine)
  assert all(not msg.valid for _, msg in engine.published)
  assert all(not msg.navRouteNavd.coordinates for msg in routes(engine))
  assert not engine.removed


def test_external_destination_is_preserved_without_route_or_waypoint_deletion(engine):
  accept(engine)
  engine.values['NavDestination'] = json.dumps({'latitude': 37.01, 'longitude': 127.01, 'place_name': 'External Navi'})
  engine.values['NavDestinationWaypoints'] = '[[127,37]]'
  saved = engine.values.copy()
  engine.published.clear()
  engine.engine.update()
  assert engine.values == saved and not engine.removed
  assert all(not msg.valid for _, msg in engine.published)


def test_destination_change_clears_even_while_gps_invalid_and_request_running(engine):
  accept(engine)
  pending = start(engine)
  engine.values['NavDestination'] = json.dumps({'latitude': 38., 'longitude': 128.})
  engine.sm['gpsLocation'].hasFix = False
  engine.engine.update()
  assert engine.engine.route is None
  assert not instructions(engine)[-1].valid
  assert not routes(engine)[-1].navRouteNavd.coordinates
  assert engine.engine.pending is pending


def test_repeated_source_churn_never_queues_another_network_job(engine):
  pending = start(engine)
  for _ in range(50):
    engine.values['CarrotNaviSource'] = 1
    engine.engine.update()
    engine.values['CarrotNaviSource'] = 0
    engine.engine.update()
  assert len(engine.engine.executor.calls) == 1
  pending.set_result(response())
  engine.engine.update()
  assert engine.engine.route_id == 0
  assert len(engine.engine.executor.calls) == 2


def test_clear_cancels_queued_work_and_invalidates_both_messages(engine):
  accept(engine)
  engine.engine.pending = Future()
  engine.engine.clear_route()
  assert engine.engine.pending.cancelled()
  engine.engine.poll_route()
  assert engine.engine.pending is None
  route, instruction = routes(engine)[-1], instructions(engine)[-1]
  assert not route.valid and not route.navRouteNavd.coordinates and route.navRouteNavd.carrotRouteId == 0
  assert not instruction.valid and instruction.navInstruction.carrotRouteId == 0
  assert instruction.navInstruction.instructionMonoTime == instruction.navInstruction.distanceMonoTime == 0


def test_timed_out_request_cannot_publish_even_if_it_eventually_succeeds(engine):
  pending = start(engine)
  engine.clock[0] += engine.navd.ROUTE_REQUEST_TIMEOUT
  pending.set_result(response())
  engine.engine.poll_route()
  assert engine.engine.route_id == 0 and engine.engine.pending is None
  assert not routes(engine)


def test_worker_exception_is_contained_without_logging_token(engine, caplog):
  start(engine).set_exception(RuntimeError('https://example.test?access_token=do-not-log'))
  engine.engine.poll_route()
  assert engine.engine.pending is None and engine.engine.route is None
  assert 'do-not-log' not in caplog.text


def test_route_resends_preserve_generation_and_new_calculations_replace_it(engine):
  accept(engine)
  first_id = engine.engine.route_id
  assert first_id == int(engine.clock[0] * 1e9)
  engine.engine.send_instruction()
  engine.clock[0] += .1
  engine.engine.send_route()
  engine.engine.send_instruction()
  assert all(msg.navRouteNavd.carrotRouteId == first_id for msg in routes(engine))
  assert all(msg.navInstruction.carrotRouteId == first_id for msg in instructions(engine))
  assert all(msg.navRouteNavd.navigationSource == 'mapbox' for msg in routes(engine))
  for msg in instructions(engine):
    instruction = msg.navInstruction
    assert instruction.navigationSource == 'mapbox'
    assert instruction.instructionMonoTime == instruction.distanceMonoTime == msg.logMonoTime
    assert len(instruction.allManeuvers) == 1
    # Real Cap'n Proto serialization, not a permissive attribute stub.
    with log.Event.from_bytes(msg.to_bytes()) as read:
      assert read.navInstruction.carrotRouteId == first_id
  accept(engine)
  assert engine.engine.route_id > first_id
  engine.engine.send_instruction()
  assert instructions(engine)[-1].navInstruction.carrotRouteId == routes(engine)[-1].navRouteNavd.carrotRouteId


def test_guidance_and_distance_clock_are_calculation_time_not_route_time(engine):
  accept(engine)
  route_id = engine.engine.route_id
  engine.engine.send_instruction()
  original_distance = instructions(engine)[-1].navInstruction.maneuverDistance
  engine.clock[0] += 1.
  engine.sm.recv_time['gpsLocation'] = engine.clock[0]
  engine.sm['gpsLocation'].latitude += .0001
  engine.engine.send_instruction()
  instruction = instructions(engine)[-1].navInstruction
  assert instruction.carrotRouteId == route_id
  assert instruction.instructionMonoTime == instruction.distanceMonoTime == int(engine.clock[0] * 1e9)
  assert instruction.instructionMonoTime > route_id
  assert instruction.maneuverDistance < original_distance


def test_gps_expiry_never_restamps_old_instruction_as_fresh(engine):
  accept(engine)
  engine.clock[0] += 2.
  engine.engine.send_instruction()
  instruction = instructions(engine)[-1]
  assert not instruction.valid
  assert instruction.navInstruction.instructionMonoTime == instruction.navInstruction.distanceMonoTime == 0


def test_saved_gps_and_old_phone_coordinates_cannot_seed_startup_route(engine):
  engine.values['LastGPSPosition'] = json.dumps({'latitude': 37., 'longitude': 127.})
  engine.sm['gpsLocation'].hasFix = False
  engine.engine.update()
  assert not engine.engine.executor.calls and not instructions(engine)[-1].valid


@pytest.mark.parametrize('loss', ['drive', 'stale_car', 'poll_gap'])
def test_navd_parked_anchor_requires_continuous_fresh_park_evidence(engine, loss):
  engine.sm['carState'].gearShifter = 'park'
  engine.engine.update_location()
  engine.clock[0] += 1.
  engine.sm.recv_time['carState'] = engine.sm.recv_time['gpsLocation'] = engine.clock[0]
  engine.engine.update_location()
  accept(engine)
  anchor = engine.engine.position.anchor
  assert anchor is not None
  engine.sm['gpsLocation'].hasFix = False
  for _ in range(5):
    engine.clock[0] += 1.
    engine.sm.recv_time['carState'] = engine.clock[0]
    engine.engine.send_instruction()
    assert instructions(engine)[-1].valid
    assert engine.engine.position.anchor is anchor and engine.engine.position.held
  if loss == 'drive':
    engine.sm['carState'].gearShifter = 'drive'
  elif loss == 'stale_car':
    engine.sm.recv_time['carState'] -= .5
  else:
    engine.clock[0] += 1.6
    engine.sm.recv_time['carState'] = engine.clock[0]
  engine.engine.send_instruction()
  assert not instructions(engine)[-1].valid
  assert engine.engine.position.anchor is None


def test_accepted_waypoints_do_not_invalidate_the_just_published_generation(engine):
  engine.values['NavDestinationWaypoints'] = '[[127,37]]'
  accept(engine)
  route_id = engine.engine.route_id
  engine.engine.send_instruction()
  assert engine.removed == ['NavDestinationWaypoints']
  assert engine.engine.route_id == route_id
  assert instructions(engine)[-1].valid


@pytest.mark.parametrize('value', ['', 'bad json', '[]', '{"latitude":NaN,"longitude":127}'])
def test_invalid_saved_destination_is_rejected(engine, value):
  engine.values['NavDestination'] = value
  engine.engine.update()
  assert not engine.engine.executor.calls and not instructions(engine)[-1].valid


@pytest.mark.parametrize('value', ['bad json', 'null', '{}', '""', '[1]', '[[180.1,37]]', '[[127,86]]', '[[127,NaN]]',
                                  json.dumps([[127, 37]] * 24)])
def test_bad_or_excessive_waypoints_never_start_http(engine, value):
  engine.values['NavDestinationWaypoints'] = value
  engine.engine.recompute_route()
  assert not engine.engine.executor.calls


@pytest.mark.parametrize('invalid', ['empty', 'bad_shape', 'too_many_steps', 'too_many_points', 'empty_points',
                                   'nan_coord', 'bad_lat', 'negative_distance', 'nan_typical', 'bad_banner', 'nan_speed',
                                   'negative_speed', 'nan_banner_distance', 'bad_unused_banner'])
def test_invalid_response_never_publishes_route_generation(engine, invalid):
  payload = response()
  leg = payload['routes'][0]['legs'][0]
  step = leg['steps'][0]
  if invalid == 'empty':
    payload = {'routes': []}
  elif invalid == 'bad_shape':
    payload = ['not a route']
  elif invalid == 'too_many_steps':
    leg['steps'] *= 2049
  elif invalid == 'too_many_points':
    step['geometry']['coordinates'] = [[127., 37.]] * 50001
  elif invalid == 'empty_points':
    step['geometry']['coordinates'] = []
  elif invalid == 'nan_coord':
    step['geometry']['coordinates'][0][0] = float('nan')
  elif invalid == 'bad_lat':
    step['geometry']['coordinates'][0][1] = 86.
  elif invalid == 'negative_distance':
    step['distance'] = -1.
  elif invalid == 'nan_typical':
    step['duration_typical'] = float('nan')
  elif invalid in ('nan_speed', 'negative_speed'):
    leg['annotation']['maxspeed'][0]['speed'] = float('nan') if invalid == 'nan_speed' else -1.
  elif invalid == 'nan_banner_distance':
    step['bannerInstructions'][0]['distanceAlongGeometry'] = float('nan')
  elif invalid == 'bad_unused_banner':
    step['bannerInstructions'].append({'distanceAlongGeometry': -1., 'primary': None})
  else:
    step['bannerInstructions'] = [None]
  start(engine).set_result(payload)
  engine.engine.poll_route()
  assert engine.engine.route_id == 0 and not routes(engine)


def test_route_point_limit_keeps_endpoints_and_provenance(engine):
  payload = response()
  payload['routes'][0]['legs'][0]['steps'][0]['geometry']['coordinates'] = [[127., 37. + i * .000001] for i in range(5000)]
  start(engine).set_result(payload)
  engine.engine.poll_route()
  route = routes(engine)[-1].navRouteNavd
  assert len(route.coordinates) == engine.navd.NAV_ROUTE_MAX_POINTS
  assert route.coordinates[0].latitude == 37.
  assert route.coordinates[-1].latitude == pytest.approx(37.004999)
  assert route.carrotRouteId == engine.engine.route_id > 0


@pytest.mark.parametrize('kind', ['good', 'oversize', 'expired', 'invalid_json', 'http_error', 'redirect', 'exception'])
def test_http_is_bounded_and_does_not_log_credentials(engine, monkeypatch, caplog, kind):
  closed, calls = [], []
  class Response:
    status_code = 302 if kind == 'redirect' else 500 if kind == 'http_error' else 200
    def __enter__(self): return self
    def __exit__(self, *_): closed.append(True)
    def raise_for_status(self): pass
    def iter_content(self, size):
      assert size == 65536
      if kind == 'oversize':
        for _ in range(129):
          yield b'x' * size
      else:
        if kind == 'expired':
          engine.clock[0] += engine.navd.ROUTE_REQUEST_TIMEOUT
        yield b'invalid' if kind == 'invalid_json' else json.dumps(response()).encode()
  def get(url, **kwargs):
    calls.append((url, kwargs))
    if kind == 'exception':
      raise engine.navd.requests.RequestException('access_token=never-log-this')
    return Response()
  monkeypatch.setattr(engine.navd.requests, 'get', get)
  result = engine.navd.RouteEngine.request_route('https://example.test', {'access_token': 'never-log-this'})
  assert (result == response()) if kind == 'good' else result is None
  assert calls[0][1]['timeout'] == (3, 8)
  assert calls[0][1]['stream'] and not calls[0][1]['allow_redirects']
  assert closed == ([] if kind == 'exception' else [True])
  assert 'never-log-this' not in caplog.text


def test_real_worker_keeps_updates_responsive_and_discards_cancelled_result(engine, monkeypatch):
  entered, release = threading.Event(), threading.Event()
  def request(*_):
    entered.set()
    assert release.wait(3.)
    return copy.deepcopy(response())
  monkeypatch.setattr(engine.engine, 'request_route', request)
  executor = ThreadPoolExecutor(max_workers=1)
  engine.engine.executor = executor
  try:
    pending = start(engine)
    assert entered.wait(1.)
    started = time.monotonic()
    for _ in range(20):
      engine.engine.update()
    assert time.monotonic() - started < 1.
    assert not instructions(engine)[-1].valid
    engine.values['CarrotNaviSource'] = 1
    engine.engine.update()
    release.set()
    pending.result(timeout=1.)
    engine.engine.update()
    assert engine.engine.route_id == 0 and not any(msg.valid for msg in routes(engine))
  finally:
    release.set()
    executor.shutdown(wait=True)


def test_real_http_response_can_be_decoded_without_external_network(engine, monkeypatch):
  from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

  class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
      body = json.dumps(response()).encode()
      self.send_response(200)
      self.send_header('Content-Type', 'application/json')
      self.send_header('Content-Length', str(len(body)))
      self.end_headers()
      self.wfile.write(body)
    def log_message(self, *_): pass

  monkeypatch.setenv('NO_PROXY', '127.0.0.1')
  server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  try:
    url = f'http://127.0.0.1:{server.server_port}/directions'
    assert engine.navd.RouteEngine.request_route(url, {}) == response()
  finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2.)
    assert not thread.is_alive()
