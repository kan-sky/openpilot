import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, interfaces
from opendbc.car.gm import interface as gm_interface
from opendbc.car.gm.values import CAR


class MemoryParams:
  def __init__(self):
    self.values = {"NNFF": 0, "NNFFLite": 0}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, 0))

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def put_int(self, key, value):
    self.values[key] = value

  def put_nonblocking(self, key, value):
    self.values[key] = value


class IsolatedGMInterface(gm_interface.CarInterface):
  # No CAN parsing, transmitting or device access in the integration fixture.
  CarState = staticmethod(lambda cp: SimpleNamespace(get_can_parsers=lambda cp: {}))
  CarController = staticmethod(lambda dbc, cp: None)


class TestNNFFSubstitute(unittest.TestCase):
  def setUp(self):
    self.params = MemoryParams()
    self.donor = Path(interfaces.TORQUE_NN_MODEL_PATH) / "CHEVROLET_TRAILBLAZER.json"
    self.catalog = Path(interfaces.TORQUE_NN_MODEL_PATH)
    for module in (interfaces, gm_interface):
      self.enterContext(patch.object(module, "Params", return_value=self.params))
    self.enterContext(contextlib.redirect_stdout(io.StringIO()))

  def car_params(self, nnff):
    self.params.values["NNFF"] = int(nnff)
    return IsolatedGMInterface.get_params(CAR.CHEVROLET_TRAVERSE, gen_empty_fingerprint(), [], False, False, False)

  def select(self, car=CAR.CHEVROLET_TRAVERSE, firmware="", enabled=False):
    return interfaces.get_nn_model_path(car, firmware, allow_substitute=enabled)

  def test_substitute_is_opt_in(self):
    self.assertIsNone(self.select())
    self.assertEqual(self.select(enabled=True), str(self.donor))

  def test_firmware_does_not_change_explicit_donor(self):
    self.assertEqual(self.select(firmware="unknown EPS firmware", enabled=True), str(self.donor))

  def test_native_model_has_precedence(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      for name in ("CHEVROLET_TRAVERSE", "CHEVROLET_TRAILBLAZER"):
        (Path(directory) / f"{name}.json").touch()
      self.assertEqual(self.select(enabled=True), str(Path(directory) / "CHEVROLET_TRAVERSE.json"))

  def test_missing_donor_is_not_guessed(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      (Path(directory) / "CHEVROLET_TRAILBLAZER_CC.json").touch()
      self.assertIsNone(self.select(enabled=True))

  def test_empty_catalog_disables_nnff(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      self.assertIsNone(self.select(enabled=True))
      self.params.values["NNFF"] = 1
      cp = self.car_params(True)
      ci = IsolatedGMInterface(cp)
      self.assertFalse(ci.use_nnff)
      self.assertIsNone(ci.lat_torque_nn_model)
      self.assertIsNone(self.params.get("NNFFModelName"))

  def test_substitute_is_traverse_only(self):
    self.assertIsNone(self.select(car="UNSUPPORTED_CAR", enabled=True))
    self.assertIsNone(self.select(car="CHEVROLET_TRAVERSE_OTHER", enabled=True))

  def test_existing_catalog_matches_are_unchanged(self):
    models = sorted(self.catalog.glob("*.json"))
    self.assertGreater(len(models), 100)
    for model in models:
      with self.subTest(model=model.name):
        self.assertEqual(self.select(car=model.stem), self.select(car=model.stem, enabled=True))

  def test_model_loads_and_evaluates_full_inputs(self):
    model = interfaces.get_nn_model(CAR.CHEVROLET_TRAVERSE, "", allow_substitute=True)
    self.assertIsNotNone(model)
    self.assertEqual(model.input_size, 18)
    self.assertEqual(model.output_size, 1)
    for speed in (10.0, 20.0, 30.0):
      for accel in (-1.0, 0.0, 1.0):
        for roll in (-0.02, 0.0, 0.02):
          with self.subTest(speed=speed, accel=accel, roll=roll):
            inputs = [speed, accel, 0.0, roll] + [accel] * 7 + [roll] * 7
            self.assertTrue(math.isfinite(model.evaluate(inputs)))

  def test_enabled_interface_uses_donor(self):
    cp = self.car_params(True)
    ci = IsolatedGMInterface(cp)
    self.assertTrue(ci.use_nnff)
    self.assertFalse(ci.use_nnff_lite)
    self.assertIsNotNone(ci.lat_torque_nn_model)
    self.assertEqual(self.params.get("NNFFModelName"), "CHEVROLET TRAILBLAZER (substitute)")

  def test_disabled_interface_does_not_use_donor(self):
    cp = self.car_params(False)
    ci = IsolatedGMInterface(cp)
    self.assertFalse(ci.use_nnff)
    self.assertIsNone(ci.lat_torque_nn_model)
    self.assertIsNone(self.params.get("NNFFModelName"))

  def test_no_vehicle_or_safety_parameter_substitution(self):
    off = self.car_params(False)
    on = self.car_params(True)
    self.assertEqual(on.to_dict(), off.to_dict())
    self.assertEqual(on.carFingerprint, CAR.CHEVROLET_TRAVERSE)
    self.assertEqual(on.lateralTuning.which(), "torque")

  def test_comma_native_nnff_keeps_precedence(self):
    cp = self.car_params(True)
    with patch.object(IsolatedGMInterface, "check_comma_nn_ff_support", return_value=True):
      self.assertFalse(IsolatedGMInterface(cp).use_nnff)

  def test_lite_setting_is_not_changed(self):
    self.params.values["NNFFLite"] = 1
    off = IsolatedGMInterface(self.car_params(False))
    self.assertFalse(off.use_nnff)
    self.assertTrue(off.use_nnff_lite)
    on = IsolatedGMInterface(self.car_params(True))
    self.assertTrue(on.use_nnff)
    self.assertFalse(on.use_nnff_lite)
    self.assertEqual(self.params.get("NNFFLite"), 1)

  def test_real_torque_controller_selects_donor_only_when_enabled(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque

    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      for enabled in (False, True):
        with self.subTest(enabled=enabled):
          cp = self.car_params(enabled)
          ci = IsolatedGMInterface(cp)
          controller = latcontrol_torque.LatControlTorque(cp.as_reader(), ci)
          self.assertEqual(controller.use_nnff, enabled)
          self.assertEqual(controller.steer_max, 1.0)
          if enabled:
            self.assertTrue(math.isfinite(controller.torque_from_nn([20.0, 0.0, 0.0, 0.0] + [0.0] * 14)))

  def controller_step(self, active, model_available):
    from openpilot.selfdrive.controls.lib import latcontrol_torque
    from opendbc.car.vehicle_model import VehicleModel

    cp = self.car_params(True)
    ci = IsolatedGMInterface(cp)
    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      controller = latcontrol_torque.LatControlTorque(cp.as_reader(), ci)
      cs = SimpleNamespace(vEgo=20.0, aEgo=0.0, steeringAngleDeg=0.0,
                           steeringRateDeg=0.0, steeringPressed=False)
      cc = SimpleNamespace(angularVelocity=[0.0] * 3, orientationNED=[0.0] * 3)
      params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
      count = len(latcontrol_torque.ModelConstants.T_IDXS)
      model = SimpleNamespace(orientation=SimpleNamespace(x=[0.0] * count, y=[0.0] * count),
                              acceleration=SimpleNamespace(y=[0.4] * count)) if model_available else None
      with patch.object(controller, "torque_from_nn", wraps=controller.torque_from_nn) as evaluate:
        torque, angle, log = controller.update(active, cs, VehicleModel(cp), params, False, 0.001, cc, False, model)
        self.assertTrue(math.isfinite(torque) and math.isfinite(angle))
        self.assertLessEqual(abs(torque), 1.0)
        return torque, log.active, evaluate.call_count

  def test_active_controller_actually_evaluates_nnff(self):
    _, active, calls = self.controller_step(True, True)
    self.assertTrue(active)
    self.assertGreaterEqual(calls, 3)

  def test_inactive_controller_never_issues_torque(self):
    torque, active, calls = self.controller_step(False, True)
    self.assertEqual(torque, 0.0)
    self.assertFalse(active)
    self.assertEqual(calls, 0)

  def test_missing_model_keeps_existing_fallback(self):
    _, active, calls = self.controller_step(True, False)
    self.assertTrue(active)
    self.assertEqual(calls, 0)

  def test_web_toggle_contract(self):
    root = Path(__file__).resolve().parents[4]
    settings_file = root / "openpilot/selfdrive/carrot_settings.json"
    if not settings_file.exists():
      self.skipTest("Openpilot settings are outside this opendbc checkout")
    settings = json.loads(settings_file.read_text())
    entry = next(item for item in settings["params"] if item["name"] == "NNFF")
    self.assertEqual(entry["default"], 0)
    self.assertIn("트레일블레이저", entry["descr"])
    self.assertIn("experimental substitute", entry["edescr"])


if __name__ == "__main__":
  unittest.main()
