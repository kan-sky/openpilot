"""Exercise the real settings catalog/API without native Params or devices."""
import asyncio
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

ROOT = Path(__file__).resolve().parents[4]
SETTINGS_PATH = ROOT / "selfdrive/carrot_settings.json"
KEY = "NavdyConnectionMode"


def test_navdy_connection_mode_is_registered_as_persistent_integer():
  keys = (ROOT / "common/params_keys.h").read_text(encoding="utf-8")
  assert keys.count(f'"{KEY}"') == 1
  assert f'{{"{KEY}", {{PERSISTENT, INT, "0"}}}}' in keys


@pytest.fixture
def settings_api(monkeypatch, tmp_path):
  # Match test_navigation_source: skip only the eager native feature imports.
  package_name = "openpilot.selfdrive.carrot.server.features"
  package = ModuleType(package_name)
  package.__path__ = [str(ROOT / "selfdrive/carrot/server/features")]
  monkeypatch.setitem(sys.modules, package_name, package)
  from openpilot.selfdrive.carrot.server.features import params as params_api, settings as catalog_api
  from openpilot.selfdrive.carrot.server.services import param_changes, params, settings

  for module in (params, params_api, catalog_api):
    monkeypatch.setattr(module, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  monkeypatch.setattr(settings, "settings_cache", {**settings.settings_cache, "path": str(SETTINGS_PATH), "data": None})
  monkeypatch.setattr(catalog_api, "settings_cache", settings.settings_cache)
  monkeypatch.setattr(param_changes, "CARROT_PARAM_CHANGES_PATH", str(tmp_path / "changes.jsonl"))
  monkeypatch.setattr(param_changes, "_known_values", {})

  def make_app():
    app = web.Application()
    params_api.register(app)
    catalog_api.register(app)
    return app

  return make_app, params, settings, param_changes


def test_actual_catalog_exposes_localized_select_next_to_navigation_source(settings_api):
  make_app, params, _, _ = settings_api

  async def run():
    async with TestClient(TestServer(make_app())) as client:
      response = await client.get("/api/settings")
      assert response.status == 200
      payload = await response.json()
      assert payload["ok"]
      definitions = [item for items in payload["items_by_group"].values() for item in items]
      modes = [item for item in definitions if item["name"] == KEY]
      assert len(modes) == 1
      mode = modes[0]
      assert (mode["control"], mode["min"], mode["max"], mode["default"], mode["unit"]) == ("select", 0, 2, 0, 1)
      assert params.infer_type_from_setting(mode) == "int"
      assert mode["options"] == {
        "ko": ["\uae30\uc874 \uc124\uc815", "\uc720\uc120 USB", "\ubb34\uc120 Bluetooth"],
        "en": ["Existing configuration", "Wired USB", "Wireless Bluetooth"],
        "zh": ["\u73b0\u6709\u914d\u7f6e", "\u6709\u7ebf USB", "\u65e0\u7ebf Bluetooth"],
      }
      assert all(mode[field] for field in ("title", "descr", "etitle", "edescr", "ctitle", "cdescr"))
      for field in ("descr", "edescr", "cdescr"):
        assert all(text in mode[field] for text in ("0:", "1:", "2:", "USB", "Bluetooth", "Jetson", "5"))
      assert "briefly interrupt" in mode["edescr"] and "while parked" in mode["edescr"]
      assert "in-progress connection attempt may delay" in mode["edescr"]
      sections = [section for category in payload["categories"] for group in category["groups"]
                  for section in group["sections"] if KEY in section["items"]]
      assert len(sections) == 1
      items = sections[0]["items"]
      assert items[items.index("CarrotNaviSource") + 1] == KEY
      source = next(item for item in definitions if item["name"] == "CarrotNaviSource")
      assert (source["min"], source["max"], source["default"], source["unit"]) == (0, 1, 0, 1)
      assert "control" not in source
      assert all("Bluetooth" in source[field] for field in ("descr", "edescr", "cdescr"))
      assert "Restart comma" in source["edescr"]
      assert "Network guidance is excluded" in source["edescr"]
      assert "Music and time sync remain available" in source["edescr"]

      response = await client.get(f"/api/params_bulk?names={KEY}")
      assert response.status == 200
      assert (await response.json())["values"][KEY] == 0
      assert KEY not in params._mem_store

  asyncio.run(run())


@pytest.mark.parametrize("value", [0, 1, 2, "0", "1", "2"])
def test_connection_mode_save_and_reload_preserves_navigation_source(settings_api, value):
  make_app, params, settings, _ = settings_api
  params._mem_store.update({KEY: "1", "CarrotNaviSource": "1"})

  async def run():
    async with TestClient(TestServer(make_app())) as client:
      response = await client.post("/api/param_set", json={"name": KEY, "value": value})
      assert response.status == 200
      result = await response.json()
      assert result["ok"] and result["name"] == KEY
      assert type(result["value"]) is int and result["value"] == int(value)
      assert params._mem_store == {KEY: str(value), "CarrotNaviSource": "1"}

    # A new HTTP app and cold catalog must read the saved value, not a UI cache.
    settings.settings_cache["data"] = None
    async with TestClient(TestServer(make_app())) as client:
      response = await client.get(f"/api/params_bulk?names={KEY},CarrotNaviSource")
      assert response.status == 200
      result = await response.json()
      assert result["ok"]
      assert result["values"] == {KEY: str(value), "CarrotNaviSource": "1"}

  asyncio.run(run())


@pytest.mark.parametrize("value", [
  -1, 3, 100, 0.0, 1.0, 2.0, 0.5, 1.5, 2.5, True, False, None, [], {},
  "", "Navdy", "USB", "Bluetooth", "-1", "3", "2.0", "2.5", "2e0", "nan", "inf",
  "true", "false", "+1", "01", " 1", "1 ", "1\n", "\uff11",
])
def test_invalid_connection_mode_is_rejected_without_clamping_or_writes(settings_api, value):
  make_app, params, _, changes = settings_api
  original = {KEY: "1", "CarrotNaviSource": "1"}
  params._mem_store.update(original)

  async def run():
    async with TestClient(TestServer(make_app())) as client:
      response = await client.post("/api/param_set", json={"name": KEY, "value": value})
      assert response.status == 400, json.dumps(value)
      result = await response.json()
      assert result["ok"] is False and KEY in result["error"]
      assert params._mem_store == original
      assert changes.read_param_changes() == []
      response = await client.get(f"/api/params_bulk?names={KEY},CarrotNaviSource")
      assert (await response.json())["values"] == original

  asyncio.run(run())
