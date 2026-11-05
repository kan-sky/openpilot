import asyncio
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.common import fcw_warning_test as warning


@pytest.fixture
def feature(monkeypatch, tmp_path):
  root = Path(__file__).resolve().parents[4]
  package = ModuleType("openpilot.selfdrive.carrot.server.features")
  package.__path__ = [str(root / "selfdrive/carrot/server/features")]
  monkeypatch.setitem(sys.modules, package.__name__, package)
  from openpilot.selfdrive.carrot.server.features import params as api
  from openpilot.selfdrive.carrot.server.services import params, settings
  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  monkeypatch.setattr(settings, "settings_cache", {**settings.settings_cache,
                      "path": str(root / "selfdrive/carrot_settings.json"), "data": None})
  requests = []
  active = [False]
  def request(value):
    requests.append(value)
    active[0] = bool(value)
  monkeypatch.setattr(warning, "request_test", request)
  monkeypatch.setattr(warning, "current_status", lambda: {"active": active[0]})
  app = web.Application()
  api.register(app)
  return app, api, params, requests, active


def headers(client):
  return {"Origin": str(client.make_url("/")).rstrip("/"), "Sec-Fetch-Site": "same-origin"}


def test_live_ack_and_cancel_are_not_persistent_settings(feature):
  app, _, params, requests, active = feature
  async def run():
    async with TestClient(TestServer(app)) as client:
      for value in (1, 0):
        response = await client.post("/api/param_set", json={"name": warning.KEY, "value": value, "source": "web_ui"},
                                     headers=headers(client))
        assert response.status == 200
        assert (await response.json())["value"] == value
        assert params.get_param_values([warning.KEY]) == {warning.KEY: value}
      assert requests == [1, 0] and not params._mem_store
  asyncio.run(run())


@pytest.mark.parametrize("value", [True, False, 1.0, 0.1, 2, -1, None, [], {}, "01", "1.0", " 1"])
def test_invalid_values_never_request_warning(feature, value):
  app, _, _, requests, _ = feature
  async def run():
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/param_set", json={"name": warning.KEY, "value": value, "source": "web_ui"},
                                   headers=headers(client))
      assert response.status == 400
      assert requests == []
  asyncio.run(run())


@pytest.mark.parametrize("source", [None, "profile", "restore", "undo", "reset_defaults", "device"])
def test_replayed_settings_cannot_activate_warning(feature, source):
  app, _, _, requests, _ = feature
  async def run():
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/param_set", json={"name": warning.KEY, "value": 1, "source": source},
                                   headers=headers(client))
      assert response.status == 403
      assert requests == []
  asyncio.run(run())


@pytest.mark.parametrize("extra", [{"Origin": "https://example.org"}, {"Origin": ""},
                                  {"Sec-Fetch-Site": "cross-site"}, {"X-Forwarded-For": "127.0.0.1"}])
def test_cross_origin_and_proxy_rejected(feature, extra):
  app, _, _, requests, _ = feature
  async def run():
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/param_set", json={"name": warning.KEY, "value": 1, "source": "web_ui"},
                                   headers={**headers(client), **extra})
      assert response.status == 403
      assert requests == []
  asyncio.run(run())


def test_no_controller_ack_cancels_instead_of_reporting_success(feature, monkeypatch):
  app, _, _, requests, _ = feature
  monkeypatch.setattr(warning, "current_status", lambda: {"active": False})
  async def run():
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/param_set", json={"name": warning.KEY, "value": 1, "source": "web_ui"},
                                   headers=headers(client))
      assert response.status == 409
      assert requests == [1, 0]
  asyncio.run(run())


def test_unsafe_controller_error_reaches_ui_without_writes(feature, monkeypatch):
  app, _, _, requests, _ = feature
  def reject(value):
    raise ValueError("openpilot OFF required")
  monkeypatch.setattr(warning, "request_test", reject)
  async def run():
    async with TestClient(TestServer(app)) as client:
      response = await client.post("/api/param_set", json={"name": warning.KEY, "value": 1, "source": "web_ui"},
                                   headers=headers(client))
      assert response.status == 409
      assert "openpilot OFF" in (await response.json())["error"]
      assert not requests
  asyncio.run(run())


def test_catalog_and_backup_do_not_make_diagnostic_persistent(feature):
  _, _, params, _, _ = feature
  with pytest.raises(ValueError, match="internally"):
    params.set_param_value(warning.KEY, 1)
  assert warning.KEY not in params.filter_param_backup_values({warning.KEY: 1})
  catalog = json.loads((Path(__file__).resolve().parents[4] / "selfdrive/carrot_settings.json").read_text())
  setting = next(item for item in catalog["params"] if item["name"] == warning.KEY)
  assert setting["min"] == setting["default"] == 0 and setting["max"] == 1
  assert setting["control"] == "select"
  assert all(setting[field] for field in ("title", "descr", "etitle", "edescr", "ctitle", "cdescr"))
  sound = next(group for menu in catalog["menu"] for group in menu["groups"] if group["id"] == "SYS_SOUND")
  assert sound["params"][-1] == warning.KEY
