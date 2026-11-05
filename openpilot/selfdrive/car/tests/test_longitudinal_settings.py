"""Settings regressions without real Params, IPC, or vehicle CAN traffic."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.alternative_experience import get_alternative_experience

ROOT = Path(__file__).resolve().parents[3]


def method(path, cls, name):
  tree = ast.parse((ROOT / path).read_text())
  klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
  return next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == name)


def execute(nodes, scope):
  exec(compile(ast.Module(body=nodes, type_ignores=[]), '<settings-policy>', 'exec'), scope)


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get_float(self, key):
    return 50.0 if key == 'LongActuatorDelay' else 0.0

  def put_bool(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


def fresh_cp_cleanup(cp, params):
  init = method('selfdrive/selfdrived/selfdrived.py', 'SelfdriveD', '__init__')
  nodes = [n for n in init.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'not self.CP.alphaLongitudinalAvailable']
  assert len(nodes) == 1
  execute(nodes, {'self': NS(CP=cp, params=params)})


@pytest.mark.parametrize('mici', [False, True])
@pytest.mark.parametrize('available', [False, True, None])
@pytest.mark.parametrize('release', [False, True])
def test_settings_refresh_does_not_delete_opt_in_from_stale_cp(mici, available, release):
  path = f'selfdrive/ui/{"mici/" if mici else ""}layouts/settings/developer.py'
  tree = ast.parse((ROOT / path).read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and any(
    isinstance(m, ast.FunctionDef) and m.name == '_update_toggles' for m in n.body))
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_update_toggles')
  params = FakeParams(AlphaLongitudinalEnabled=True)
  item = Mock()
  self = NS(_params=params, _is_release=release, _refresh_toggles=[],
            **{name: item for name in ('_alpha_long_toggle', '_long_maneuver_toggle', '_joystick_toggle',
                                      '_adb_toggle', '_ssh_toggle', '_ui_debug_toggle')})
  ui = NS(CP=None if available is None else NS(alphaLongitudinalAvailable=available),
          params=params, is_release=release, update_params=lambda: None,
          has_longitudinal_control=False, is_offroad=lambda: True)
  execute([update], scope := {'ui_state': ui})
  for _ in range(3):
    scope['_update_toggles'](self)
  assert params.get_bool('AlphaLongitudinalEnabled') == (not release or available is None)
  assert ui.CP is None or ui.CP.alphaLongitudinalAvailable == available


@pytest.mark.parametrize('available', [False, True])
def test_fresh_unsupported_cp_still_clears_opt_in(available):
  params = FakeParams(AlphaLongitudinalEnabled=True)
  fresh_cp_cleanup(NS(alphaLongitudinalAvailable=available), params)
  assert params.get_bool('AlphaLongitudinalEnabled') == available


def safety_experience(disengage):
  init = method('selfdrive/car/card.py', 'Car', '__init__')
  nodes = [n for n in init.body if isinstance(n, ast.Assign) and
           any(ast.unparse(target) == 'self.CP.alternativeExperience' for target in n.targets)]
  assert len(nodes) == 1
  self = NS(CP=NS(alternativeExperience=0), params=FakeParams(DisengageOnAccelerator=disengage))
  execute(nodes, {'self': self, 'get_alternative_experience': get_alternative_experience})
  return self.CP.alternativeExperience


@pytest.mark.parametrize('disengage', [False, True])
def test_card_honors_accelerator_preference_without_other_safety_flags(disengage):
  assert safety_experience(disengage) == (0 if disengage else ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS)


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('dashcam_only', [False, True])
@pytest.mark.parametrize('has_controller', [False, True])
def test_master_toggle_preserves_no_output_for_unsupported_cars(enabled, dashcam_only, has_controller):
  init = method('selfdrive/car/card.py', 'Car', '__init__')
  start = next(i for i, n in enumerate(init.body) if isinstance(n, ast.Assign)
               and ast.unparse(n.targets[0]) == 'openpilot_enabled_toggle')
  nodes = init.body[start:start + 4]
  self = NS(CP=NS(dashcamOnly=dashcam_only, safetyConfigs=['gm']),
            CI=NS(CC=object() if has_controller else None), params=FakeParams(OpenpilotEnabledToggle=enabled))
  structs = NS(CarParams=NS(SafetyConfig=NS, SafetyModel=NS(noOutput='noOutput')))
  execute(nodes, {'self': self, 'structs': structs})
  assert self.CP.passive == (not enabled or dashcam_only or not has_controller)
  if self.CP.passive:
    assert self.CP.safetyConfigs[0].safetyModel == 'noOutput'
  else:
    assert self.CP.safetyConfigs == ['gm']


@pytest.mark.parametrize('startup_disengage', [False, True])
@pytest.mark.parametrize('current_disengage', [False, True])
@pytest.mark.parametrize('pedal', ['gas', 'brake', 'regen', 'none'])
def test_pedal_events_never_conflict_with_panda_safety(startup_disengage, current_disengage, pedal):
  update = method('selfdrive/selfdrived/selfdrived.py', 'SelfdriveD', 'update_events')
  gas = next(n for n in ast.walk(update) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == 'disengage_on_gas')
  condition = next(n for n in ast.walk(update) if isinstance(n, ast.If) and 'disengage_on_gas' in ast.unparse(n.test))
  events = Mock()
  self = NS(params=FakeParams(DisengageOnAccelerator=current_disengage),
            CP=NS(alternativeExperience=safety_experience(startup_disengage)),
            CS_prev=NS(gasPressed=False, brakePressed=False, regenBraking=False), events=events)
  cs = NS(gasPressed=pedal == 'gas', brakePressed=pedal == 'brake', regenBraking=pedal == 'regen', standstill=False)
  execute([gas, condition], {'self': self, 'CS': cs, 'EventName': NS(pedalPressed='pedalPressed'),
                            'ALTERNATIVE_EXPERIENCE': ALTERNATIVE_EXPERIENCE})
  expected = pedal in ('brake', 'regen') or (pedal == 'gas' and (startup_disengage or current_disengage))
  assert events.add.called == expected


@pytest.fixture
def gm_params(monkeypatch):
  # Native interfaces are exercised on comma; never instantiate the real Params store.
  pytest.importorskip('openpilot.common.params_pyx')
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.gm.interface import CarInterface
  params = FakeParams(AlphaLongitudinalEnabled=True)
  monkeypatch.setattr('opendbc.car.gm.interface.Params', lambda: params)
  monkeypatch.setattr('opendbc.car.interfaces.Params', lambda: params)

  def build(candidate, alpha):
    return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, True, False)

  return params, build


def test_traverse_longitudinal_opt_in_survives_three_drives(gm_params):
  from opendbc.car.gm.values import CAR, GMSafetyFlags
  params, build = gm_params
  for _ in range(3):
    cp = build(CAR.CHEVROLET_TRAVERSE, params.get_bool('AlphaLongitudinalEnabled'))
    assert cp.alphaLongitudinalAvailable and cp.openpilotLongitudinalControl
    assert not cp.pcmCruise and not cp.dashcamOnly
    assert cp.safetyConfigs[0].safetyParam == (GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG |
                                            GMSafetyFlags.HW_SDGM | GMSafetyFlags.MANUAL_AUTO_HOLD)
    fresh_cp_cleanup(cp, params)
    assert params.get_bool('AlphaLongitudinalEnabled')

  cp = build(CAR.CHEVROLET_TRAVERSE, False)
  assert cp.alphaLongitudinalAvailable and not cp.openpilotLongitudinalControl
  assert cp.pcmCruise
  assert cp.safetyConfigs[0].safetyParam == GMSafetyFlags.HW_CAM


@pytest.mark.parametrize('alpha', [False, True])
def test_other_unsupported_gm_cars_cannot_enable_long_from_stale_opt_in(gm_params, alpha):
  from opendbc.car.gm.values import CAR, SDGM_CAR, GMSafetyFlags
  _, build = gm_params
  for candidate in SDGM_CAR - {CAR.CHEVROLET_TRAVERSE}:
    cp = build(candidate, alpha)
    assert not cp.alphaLongitudinalAvailable
    assert not cp.openpilotLongitudinalControl
    assert cp.pcmCruise
    assert not cp.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_CAM_LONG


@pytest.mark.parametrize('alpha', [False, True])
def test_supported_camera_acc_keeps_existing_behavior(gm_params, alpha):
  from opendbc.car.gm.values import CAR, GMSafetyFlags
  _, build = gm_params
  cp = build(CAR.CHEVROLET_BOLT_EUV, alpha)
  assert cp.alphaLongitudinalAvailable
  assert cp.openpilotLongitudinalControl == alpha
  assert cp.pcmCruise != alpha
  assert bool(cp.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_CAM_LONG) == alpha


@pytest.mark.parametrize('alpha', [False, True])
def test_ascm_longitudinal_does_not_depend_on_alpha_opt_in(gm_params, alpha):
  from opendbc.car.gm.values import CAR, GMSafetyFlags
  _, build = gm_params
  cp = build(CAR.GMC_YUKON, alpha)
  assert cp.openpilotLongitudinalControl
  assert not cp.alphaLongitudinalAvailable
  assert not cp.pcmCruise
  assert not cp.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_CAM_LONG
