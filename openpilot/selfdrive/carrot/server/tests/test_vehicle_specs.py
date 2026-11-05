import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import types
from types import SimpleNamespace as NS

import pytest

from openpilot.common.vehicle_dynamics import SPEC_LIMITS
from openpilot.selfdrive.carrot.server.services import params
from openpilot.selfdrive.carrot.server.services.settings import (
  build_menu_categories, current_vehicle_spec_defaults, group_index, with_vehicle_spec_defaults,
)

ROOT = Path(__file__).resolve().parents[4]


def test_catalog_location_registration_and_limits():
  data = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text())
  _, by_name, _ = group_index(data)
  categories = build_menu_categories(data, by_name)
  driving = next(c for c in categories if c["id"] == "DRIVING")
  steering = next(g for g in driving["groups"] if g["id"] == "STEER")
  section = next(s for s in steering["sections"] if s["id"] == "STEER_VEHICLE_SPECS")
  assert section["items"] == list(SPEC_LIMITS)
  registry = (ROOT / "common/params_keys.h").read_text()
  for name, (low, high) in SPEC_LIMITS.items():
    p = by_name[name]
    assert (p["min"], p["max"], p["default"]) == (low, high, None)
    assert p["vehicle_default"]
    assert p["risk"] == "high"
    kind = "INT" if name == "VehicleMass" else "FLOAT"
    assert f'{{"{name}", {{PERSISTENT, {kind}, "0"}}}}' in registry
    assert all(p[field] for field in ("descr", "edescr", "cdescr"))


@pytest.mark.parametrize("name,value", [("VehicleMass", 2090), ("VehicleCenterToFrontRatio", 0.7), ("VehicleCenterToFrontRatio", 0.735)])
def test_write_read_roundtrip(monkeypatch, name, value):
  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  params.set_param_value(name, value)
  assert params.get_param_values([name])[name] == str(value)


@pytest.mark.parametrize("name,value", [
  ("VehicleMass", 499), ("VehicleMass", 5001), ("VehicleMass", "nan"),
  ("VehicleCenterToFrontRatio", 9), ("VehicleCenterToFrontRatio", 91),
  ("VehicleCenterToFrontRatio", 70.5), ("VehicleMass", True),
  ("VehicleMass", 0), ("VehicleCenterToFrontRatio", 0),
])
def test_write_rejects_invalid_values_before_storage(monkeypatch, name, value):
  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  with pytest.raises(ValueError):
    params.set_param_value(name, value)
  assert params._mem_store == {}


@pytest.fixture
def feature(monkeypatch):
  # Import only this HTTP feature, not native IPC/camera siblings.
  name = "openpilot.selfdrive.carrot.server.features"
  package = types.ModuleType(name)
  package.__path__ = [str(ROOT / "selfdrive/carrot/server/features")]
  monkeypatch.setitem(sys.modules, name, package)
  monkeypatch.setitem(sys.modules, name + ".system", NS(is_drive_engaged=lambda request: True))
  spec = importlib.util.spec_from_file_location(name + ".params", Path(package.__path__[0]) / "params.py")
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, spec.name, module)
  spec.loader.exec_module(module)
  return module


def test_http_rejects_before_rounding_or_clamping(monkeypatch, feature):
  writes = []
  monkeypatch.setattr(feature, "set_param_value", lambda *args: writes.append(args))

  class Request:
    async def json(self):
      return {"name": "VehicleCenterToFrontRatio", "value": 70.4}

  response = asyncio.run(feature.api_param_set(Request()))
  assert response.status == 400
  assert writes == []


def test_http_allows_live_write_and_returns_saved_value(monkeypatch, feature):
  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  monkeypatch.setattr(feature, "append_param_change", lambda *a, **kw: None)
  monkeypatch.setattr(feature, "get_settings_cached", lambda: (None, None, {"VehicleMass": {
    "min": 500, "max": 5000, "default": 2090,
  }}, None))

  class Request:
    async def json(self):
      return {"name": "VehicleMass", "value": 2090}

  response = asyncio.run(feature.api_param_set(Request()))
  assert response.status == 200
  assert json.loads(response.text)["value"] == 2090
  assert params._mem_store["VehicleMass"] == "2090"


def test_http_to_live_model_roundtrip_without_restart(monkeypatch, feature):
  from openpilot.common.tests.test_vehicle_dynamics import car_params, messages
  from openpilot.common.vehicle_dynamics import LiveVehicleDynamics, read_live_vehicle_dynamics
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.cereal import log

  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  monkeypatch.setattr(feature, "append_param_change", lambda *a, **kw: None)
  monkeypatch.setattr(feature, "get_settings_cached", lambda: (None, None, {}, None))
  cp = car_params()
  tuning = LiveVehicleDynamics(cp, NS(get=lambda key: params.get_param_value(key, 0)))
  vm = VehicleModel(cp)
  tuning.update(0)

  class Request:
    def __init__(self, name, value):
      self.body = {"name": name, "value": value}

    async def json(self):
      return self.body

  for name, value in (("VehicleMass", 2090), ("VehicleCenterToFrontRatio", 0.7)):
    assert asyncio.run(feature.api_param_set(Request(name, value))).status == 200
  for i in range(1, 401):
    updated = tuning.update(i * 0.05)
    sm = messages(cp, updated)
    with log.LiveParametersData.from_bytes(sm["liveParameters"].to_bytes()) as lp:
      sm["liveParameters"] = lp
      vm.update_dynamics(read_live_vehicle_dynamics(sm, cp))
  assert vm.m == 2226
  assert vm.aF == pytest.approx(3.07 * 0.7)
  assert cp.mass == 2091  # The startup/safety CarParams are not rewritten.


def test_restore_preview_rejects_fraction_before_integer_coercion(monkeypatch):
  monkeypatch.setattr(params, "HAS_PARAMS", True)
  monkeypatch.setattr(params, "Params", lambda: NS())
  monkeypatch.setattr(params, "ParamKeyType", NS(INT="int", BOOL="bool", FLOAT="float", BYTES="bytes", JSON="json"))
  monkeypatch.setattr(params, "_catalog_definitions", lambda: {})
  monkeypatch.setattr(params, "get_param_values", lambda *args: {})
  monkeypatch.setattr(params, "resolve_param_type", lambda *args: "int")
  monkeypatch.setattr(params, "resolve_param_type", lambda name, *args: "float" if name == "VehicleCenterToFrontRatio" else "int")
  result = params.preview_param_restore_values({"VehicleMass": 2090.1, "VehicleCenterToFrontRatio": 0.7})
  assert result["summary"]["invalid"] == 1
  assert result["summary"]["selected"] == 1


def test_installed_defaults_are_dynamic_and_do_not_mutate_catalog():
  from openpilot.common.tests.test_vehicle_dynamics import car_params
  data = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text())
  parts = (data, *group_index(data))
  for mass, ratio in ((2090, 0.7), (1955, 0.4)):
    cp = car_params(mass, ratio)
    defaults = current_vehicle_spec_defaults(NS(get=lambda key: cp.to_bytes()))
    assert defaults == {"VehicleMass": mass, "VehicleCenterToFrontRatio": ratio}
    adjusted = with_vehicle_spec_defaults(parts, defaults)
    assert adjusted[2]["VehicleMass"]["default"] == mass
    assert adjusted[2]["VehicleCenterToFrontRatio"]["default"] == ratio
  assert parts[2]["VehicleMass"]["default"] is None
  assert current_vehicle_spec_defaults(NS(get=lambda key: None)) == {}


@pytest.mark.parametrize("raw,expected", [(0, 0.7), (70, 0.7), (0.735, 0.735)])
def test_native_decimal_read_does_not_truncate(monkeypatch, raw, expected):
  from openpilot.common.tests.test_vehicle_dynamics import car_params
  cp = car_params(2090, 0.7)
  store = {"CarParamsPersistent": cp.to_bytes(), "VehicleCenterToFrontRatio": raw}
  assert params._read_param_value(NS(get=store.get), "VehicleCenterToFrontRatio", None) == expected


def test_http_decimal_roundtrip_with_native_float_metadata(monkeypatch, feature):
  monkeypatch.setattr(params, "HAS_PARAMS", False)
  monkeypatch.setattr(params, "_mem_store", {})
  monkeypatch.setattr(feature, "append_param_change", lambda *a, **kw: None)
  monkeypatch.setattr(feature, "get_settings_cached", lambda: (None, None, {"VehicleCenterToFrontRatio": {
    "min": 0.1, "max": 0.9, "default": 0.7,
  }}, None))

  class Request:
    async def json(self):
      return {"name": "VehicleCenterToFrontRatio", "value": 0.735}

  response = asyncio.run(feature.api_param_set(Request()))
  assert response.status == 200
  assert json.loads(response.text)["value"] == 0.735
  assert params._mem_store["VehicleCenterToFrontRatio"] == "0.735"
