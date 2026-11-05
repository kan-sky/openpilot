"""Exercise source selection without starting IPC, device services or CAN."""
import ast
import asyncio
import collections
import __future__
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock  # noqa: TID251 -- mocks only; tests use pytest

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from openpilot.common.constants import CV
from openpilot.selfdrive.carrot.carrot_navi_control import parse_carrot_navi_control
from openpilot.selfdrive.carrot.gps_sources import GPSSources
from openpilot.selfdrive.navigation.provenance import RoutePublication
from openpilot.selfdrive.navdy.navigation import FeedbackTracker, NavigationReader
from openpilot.selfdrive.navdy.tests.test_navigation import packet as feedback_packet

ROOT = Path(__file__).resolve().parents[3]


class FakeParams:
  def __init__(self, values=None):
    self.values = values if values is not None else {}

  def get(self, key):
    return self.values.get(key, "0")

  def get_int(self, key):
    return int(self.get(key))

  def get_float(self, key):
    return float(self.get(key))

  def get_bool(self, key):
    return self.get(key) in ("1", True)


def load_class(filename, name, scope):
  # Execute the real class, isolating only the native platform imports.
  tree = ast.parse((ROOT / "selfdrive/carrot" / filename).read_text())
  node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
  constants = [n for n in tree.body if isinstance(n, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id in ("NAVI_SOURCE_NETWORK", "NAVI_SOURCE_NAVDY", "nav_type_mapping") for t in n.targets)]
  code = compile(ast.Module(body=constants + [node], type_ignores=[]), filename, "exec",
                 flags=__future__.annotations.compiler_flag)
  exec(code, scope)
  return scope[name]


@pytest.fixture
def services(tmp_path):
  values = {}
  scope = {"Params": lambda *args: FakeParams(values), "collections": collections, "time": time, "PC": False,
           "NavigationReader": lambda: NavigationReader(tmp_path / 'navdy-feedback.json'),
           "CV": CV, "json": json, "math": math,
           "GPSSources": GPSSources, "RoutePublication": RoutePublication,
           "parse_carrot_navi_control": parse_carrot_navi_control}
  cls = load_class("carrot_serv.py", "CarrotServ", scope)
  return cls, values


def packet(**overrides):
  data = {"nRoadLimitSpeed": 60, "nSdiType": 1, "nSdiSpeedLimit": 50, "nSdiDist": 350,
          "nTBTTurnType": 12, "nTBTDist": 120, "szTBTMainText": "Turn left"}
  data.update(overrides)
  return data


@pytest.mark.parametrize("source", [0, 1])
def test_selected_source_updates_guidance(services, source):
  cls, values = services
  values["CarrotNaviSource"] = source
  svc = cls()
  svc._update_tbt = Mock()
  svc._update_sdi = Mock()
  for _ in range(6):
    svc.update(packet(), source=source)
  assert svc.nRoadLimitSpeed == 60
  assert svc.nSdiDist == 350 and svc.nTBTTurnType == 12
  assert svc.active_count == 80 and svc.active_sdi_count > 0
  assert svc._update_tbt.call_count == 6 and svc._update_sdi.call_count == 6


@pytest.mark.parametrize("source", [0, 1])
def test_unselected_input_cannot_modify_guidance_or_refresh_heartbeat(services, source):
  cls, values = services
  values["CarrotNaviSource"] = source
  svc = cls()
  svc.nRoadLimitSpeed = 40
  svc.active_count = 7
  svc.active_sdi_count = 4
  svc.update(packet(carrotIndex=1, goalPosX=127, goalPosY=37, source=source), source=1 - source)
  assert svc.nRoadLimitSpeed == 40 and svc.nSdiType == -1 and svc.nTBTTurnType == -1
  assert svc.goalPosX == 0
  assert svc.active_count == 7 and svc.active_sdi_count == 4


def test_default_network_and_restart_latched_selection(services):
  cls, values = services
  original = cls()
  assert original.accepts_navigation(0)
  values["CarrotNaviSource"] = 1
  original.update_params()
  assert original.accepts_navigation(0) and not original.accepts_navigation(1)
  replacement = cls()
  assert replacement.accepts_navigation(1) and not replacement.accepts_navigation(0)
  assert replacement.active_count == replacement.active_sdi_count == 0
  assert replacement.nTBTTurnType == -1 and replacement.xSpdLimit == 0


def test_network_time_gps_commands_continue_without_nav_heartbeat(services):
  cls, values = services
  values["CarrotNaviSource"] = 1
  svc = cls()
  svc.set_time = Mock()
  svc.update(packet(carrotIndex=60, epochTime=1800000000000, timezone="Asia/Seoul",
                    carrotCmd="DISPLAY", carrotArg="MAP", latitude=37.5, longitude=127.1,
                    heading=90, accuracy=5, gps_speed=12))
  svc.set_time.assert_called_once_with(1800000000000, "Asia/Seoul")
  assert svc.carrotCmd == "DISPLAY" and svc.phone_latitude == 37.5
  assert svc.nSdiType == -1 and svc.nTBTTurnType == -1 and svc.active_count == 0


def test_network_detection_cannot_supply_traffic_state_in_navdy_mode(services):
  cls, values = services
  values["CarrotNaviSource"] = 1
  svc = cls()
  svc.update({"carrotIndex": 3, "carrotCmd": "DETECT", "carrotArg": "red"})
  assert svc.carrotCmd == "" and svc.traffic_state == 0


def test_navdy_does_not_fall_back_to_kisa_or_navd(services):
  cls, values = services
  values["CarrotNaviSource"] = 1
  svc = cls()
  svc.update_kisa({"kisawazeroadspdlimit": 100})
  svc.update_nav_instruction(None)  # Must return without even consulting navd.
  assert svc.nRoadLimitSpeed == 30 and svc.active_kisa_count == 0
  assert svc.xTurnInfo == -1


def test_navdy_feedback_uses_carrot_control_mapping_and_clears_on_expiry(services):
  cls, values = services
  values["CarrotNaviSource"] = 1
  values["AutoNaviSpeedCtrlMode"] = 3
  values["AutoNaviSpeedSafetyFactor"] = 100
  svc = cls()
  svc.params_memory = Mock()
  tracker = FeedbackTracker()
  tracker.accept(feedback_packet(), 10, 10)
  svc.navdy_navigation = Mock()
  svc.navdy_navigation.read.return_value = tracker.snapshot(10)
  # No IPC object: selecting Navdy must not consult network carrotNavi at all.
  assert svc._update_carrot_navi(None)
  assert (svc.nRoadLimitSpeed, svc.xSpdLimit, svc.xSpdDist) == (60, 50, 350)
  assert (svc.nTBTTurnType, svc.xTurnInfo, svc.xDistToTurn) == (12, 1, 120)
  assert (svc.nGoPosDist, svc.nGoPosTime) == (1500, 240)
  assert json.loads(svc.params_memory.put_nonblocking.call_args.args[1])["lamp"] == "red"
  svc.navdy_navigation.read.return_value = tracker.snapshot(12)
  assert not svc._update_carrot_navi(None)
  assert svc.xSpdLimit == 0 and svc.xTurnInfo == -1
  assert svc.nGoPosDist == svc.nGoPosTime == 0
  assert not svc.carrot_navi_road_limit_valid
  svc.params_memory.remove.assert_called_with("TrafficLight")


def test_network_mode_does_not_read_navdy_cache(services):
  cls, _ = services
  svc = cls()
  svc.navdy_navigation = Mock()
  sm = Mock(alive={"carrotNavi": False}, valid={"carrotNavi": False})
  assert not svc._update_carrot_navi(sm)
  svc.navdy_navigation.read.assert_not_called()


@pytest.fixture
def manager(services):
  cls, values = services
  scope = {"Any": object, "NAVI_HTTP_PORT": 7713, "NAVI_SOURCE_NETWORK": 0, "NAVI_SOURCE_NAVDY": 1,
           "json": json, "web": web}
  manager_cls = load_class("carrot_man.py", "CarrotMan", scope)

  def make(source):
    values["CarrotNaviSource"] = source
    mgr = manager_cls.__new__(manager_cls)
    mgr.carrot_serv = cls()
    mgr.params = FakeParams(values)
    mgr._navi_event_lock = threading.Lock()
    mgr._last_navi_event = None
    mgr._last_navi_event_by_type = {}
    mgr._detect_navi_event_type = Mock(return_value="rgdata")
    mgr._get_timestamp_ms = Mock(return_value=1)
    mgr._store_navi_event = Mock()
    mgr._is_stale_rgdata = Mock(return_value=(False, 0))
    mgr._safe_dispatch_handler = Mock()
    mgr._write_navi_debug_param = Mock()
    mgr.broadcast_port = 7705
    return mgr

  return make


@pytest.mark.parametrize("field", ["rgdata", "route", "vrtx", "sinf", "ssinf", "complexCrossroad"])
def test_navdy_blocks_all_network_dispatch_types(manager, field):
  mgr = manager(1)
  assert mgr._dispatch_obj({field: packet()}) is False
  mgr._safe_dispatch_handler.assert_not_called()
  mgr._store_navi_event.assert_not_called()
  assert mgr.carrot_serv.active_count == 0


def test_network_dispatch_still_works(manager):
  mgr = manager(0)
  assert mgr._dispatch_obj({"rgdata": packet()}) is True
  mgr._safe_dispatch_handler.assert_called_once_with("rgdata", mgr.handle_carrot_state, packet())


def test_navdy_suppresses_legacy_route_socket_and_navd_routes(manager):
  mgr = manager(1)
  # No publisher/coordinates attributes: rejection must precede all processing.
  mgr.send_routes([object()], from_navd=True)
  mgr.send_routes = Mock()
  mgr.carrot_route()
  mgr.send_routes.assert_called_once_with([])


def test_http_does_not_claim_dropped_network_guidance_was_accepted(manager):
  async def run():
    mgr = manager(1)
    app = web.Application()
    app.router.add_post("/api/navi/{tmap_version}", mgr.carrot_http_post)
    app.router.add_get("/health", mgr.carrot_http_health)
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/navi/test", json={"rgdata": packet(), "source": "navdy"})
      assert response.status == 200
      assert (await response.json())["accepted"] is False
      assert mgr.carrot_serv.active_count == 0
      response = await client.get("/health")
      status = await response.json()
      assert status["navigationSource"] == 1
      assert status["navdyRelayImplemented"] is True
      assert status["navigationSourceReason"] == "navdy_disconnected"
      assert status["navigationSourceRestartRequired"] is False
      mgr.params.values["CarrotNaviSource"] = 0
      status = await (await client.get("/health")).json()
      assert status["navigationSource"] == 1 and status["configuredNavigationSource"] == 0
      assert status["navigationSourceRestartRequired"] is True
  asyncio.run(run())


def test_web_settings_registration_save_and_reload(monkeypatch):
  # Import the real handlers without starting unrelated native IPC features.
  package_name = "openpilot.selfdrive.carrot.server.features"
  package = ModuleType(package_name)
  package.__path__ = [str(ROOT / "selfdrive/carrot/server/features")]
  monkeypatch.setitem(sys.modules, package_name, package)
  from openpilot.selfdrive.carrot.server.features import params as params_api, settings as settings_api
  from openpilot.selfdrive.carrot.server.services import params as params_service, settings

  monkeypatch.setattr(params_service, "HAS_PARAMS", False)
  monkeypatch.setattr(settings_api, "HAS_PARAMS", False)
  monkeypatch.setattr(params_service, "_mem_store", {})
  monkeypatch.setitem(settings.settings_cache, "path", str(ROOT / "selfdrive/carrot_settings.json"))
  monkeypatch.setitem(settings.settings_cache, "data", None)

  async def run():
    app = web.Application()
    params_api.register(app)
    settings_api.register(app)
    async with TestClient(TestServer(app)) as client:
      payload = await (await client.get("/api/settings")).json()
      assert payload["ok"]
      defs = [p for items in payload["items_by_group"].values() for p in items if p["name"] == "CarrotNaviSource"]
      assert len(defs) == 1 and defs[0]["default"] == 0 and defs[0]["max"] == 1
      groups = [g for c in payload["categories"] for g in c["groups"]]
      sections = [s for g in groups for s in g["sections"] if "CarrotNaviSource" in s["items"]]
      assert len(sections) == 1
      for value in [0, 1, 0]:
        result = await client.post("/api/param_set", json={"name": "CarrotNaviSource", "value": value})
        assert result.status == 200
        saved = await (await client.get("/api/params_bulk?names=CarrotNaviSource")).json()
        assert int(saved["values"]["CarrotNaviSource"]) == value
      for value in [-1, 2, 0.5, "Navdy", "nan", None, True]:
        result = await client.post("/api/param_set", json={"name": "CarrotNaviSource", "value": value})
        assert result.status == 400
      assert params_service._mem_store["CarrotNaviSource"] == "0"
  asyncio.run(run())
