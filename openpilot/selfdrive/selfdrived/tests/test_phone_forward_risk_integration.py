"""Execute the actual DM/FCW/event sections without the host's native IPC/Params.

This covers schema serialization and warning policy, not a running selfdrived.
"""
import ast
import copy
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.phone_forward_risk import PhoneForwardRisk
from openpilot.selfdrive.controls.lib.tests.test_phone_forward_risk import Inputs
from openpilot.selfdrive.selfdrived.tests.test_alert_presentation import load_events


ROOT = Path(__file__).resolve().parents[1]
AlertLevel = log.DriverMonitoringState.AlertLevel
MonitoringPolicy = log.DriverMonitoringState.MonitoringPolicy


def actual_sections(module, replay=True):
  tree = ast.parse((ROOT / 'selfdrived.py').read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  methods = {n.name: copy.deepcopy(n) for n in cls.body if isinstance(n, ast.FunctionDef)}
  update = methods['update_events']
  # Include the full ordinary DM block and the additive call at its real location.
  start = next(i for i, n in enumerate(update.body) if isinstance(n, ast.Assign)
               and ast.unparse(n.targets[0]) == 'dm_disabled')
  stop = next(i for i, n in enumerate(update.body) if isinstance(n, ast.Expr)
              and ast.unparse(n.value).startswith('self._update_phone_forward_risk('))
  fcw = next(i for i, n in enumerate(update.body) if isinstance(n, ast.Assign)
             and ast.unparse(n.targets[0]) == 'stock_long_is_braking')
  update.body = update.body[start:stop + 1] + update.body[fcw:fcw + 4]
  writes = []
  ns = {'EventName': module.EventName, 'AlertLevel': AlertLevel, 'MonitoringPolicy': MonitoringPolicy,
            'REPLAY': replay, 'time': SimpleNamespace(monotonic=lambda: 10.0),
            'set_offroad_alert': lambda *args: writes.append(args)}
  body = [update, methods['_update_phone_forward_risk'], methods['update_dm_lockout']]
  exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(ROOT / 'selfdrived.py'), 'exec'), ns)
  return ns, writes


def controller(module, phone=True, replay=True):
  ns, offroad_writes = actual_sections(module, replay)
  sm = Inputs()
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = phone
  sm['longitudinalPlan'] = log.LongitudinalPlan.new_message()
  sm['modelV2'] = log.ModelDataV2.new_message()
  writes = []
  state = SimpleNamespace(sm=sm, phone_forward_risk=PhoneForwardRisk(), initialized=True,
                          CP=SimpleNamespace(notCar=False, passive=False, openpilotLongitudinalControl=True),
                          car_state_time_s=10.0, events=module.Events(), enabled=True,
                          dm_disabled_prev=False, dm_uncertain_alerted=False, dm_lockout_set=False,
                          params=SimpleNamespace(put_bool=lambda *args: writes.append(args)))
  state.update_dm_lockout = lambda: ns['update_dm_lockout'](state)
  state._update_phone_forward_risk = lambda cs: ns['_update_phone_forward_risk'](state, cs)
  state.update_events = lambda: ns['update_events'](state, sm.cs)
  return state, writes, offroad_writes


@pytest.mark.parametrize('replay', [False, True])
def test_integration_emits_real_warning_and_schema_roundtrip(replay):
  m = load_events()
  state, writes, _ = controller(m, replay=replay)
  state.update_events()
  assert state.events.names == [m.EventName.phoneLeadClosing]
  assert not writes
  event = state.events.to_msg()[0]
  with log.OnroadEvent.from_bytes(event.to_bytes()) as decoded:
    assert decoded.name == 'phoneLeadClosing'
    assert decoded.warning
    for kind in ('enable', 'noEntry', 'userDisable', 'softDisable', 'immediateDisable',
                 'preEnable', 'permanent', 'overrideLateral', 'overrideLongitudinal'):
      assert not getattr(decoded, kind)
  alerts = state.events.create_alerts([m.ET.WARNING])
  assert len(alerts) == 1
  alert = alerts[0]
  assert (alert.alert_text_1, alert.alert_text_2) == ('전방 주의', '전방 차량과 접근중')
  assert alert.duration == 200 and alert.audible_alert == m.AudibleAlert.prompt
  assert alert.priority == m.Priority.MID and alert.visual_alert == m.VisualAlert.none
  assert alert.alert_status == m.AlertStatus.userPrompt and alert.alert_size == m.AlertSize.mid
  assert set(m.EVENTS[m.EventName.phoneLeadClosing]) == {m.ET.WARNING}


@pytest.mark.parametrize('policy', ['vision', 'wheeltouch'])
@pytest.mark.parametrize('level', ['none', 'one', 'two', 'three'])
@pytest.mark.parametrize('lock', ['none', 'lockout', 'alwaysOnLockout'])
def test_ordinary_dm_alerts_and_lockout_are_unchanged(policy, level, lock):
  m = load_events()
  baseline, baseline_writes, baseline_offroad = controller(m, phone=False)
  warning, warning_writes, warning_offroad = controller(m)
  for state in (baseline, warning):
    dm = state.sm['driverMonitoringState']
    dm.activePolicy, dm.alertLevel = policy, level
    dm.visionPolicyState.uncertainOffroadAlertPercent = 100
    if lock != 'none':
      setattr(dm, lock, True)
    before = dm.to_dict()
    state.update_events()
    assert dm.to_dict() == before
  assert warning.events.names == sorted(baseline.events.names + [m.EventName.phoneLeadClosing])
  assert baseline_writes == warning_writes == ([('DriverTooDistracted', True)] if lock == 'lockout' else [])
  assert baseline_offroad == warning_offroad == [('Offroad_DriverMonitoringUncertain', True)]
  if lock != 'none':
    assert m.EventName.tooDistracted in warning.events.names
    assert warning.events.contains(m.ET.NO_ENTRY)
  if level != 'none':
    kind = 'driverDistracted' if policy == 'vision' else 'driverUnresponsive'
    assert getattr(m.EventName, kind + str(getattr(AlertLevel, level))) in warning.events.names


@pytest.mark.parametrize('invalid_service', ['driverMonitoringState', 'radarState'])
def test_invalid_input_suppresses_only_phone_warning(invalid_service):
  m = load_events()
  state, _, _ = controller(m)
  state.sm.valid[invalid_service] = False
  dm = state.sm['driverMonitoringState']
  dm.alertLevel, dm.lockout = 'three', True
  state.sm['longitudinalPlan'].fcw = True
  state.update_events()
  assert state.events.names == sorted([m.EventName.driverDistracted3, m.EventName.tooDistracted, m.EventName.fcw])


@pytest.mark.parametrize('gate', ['notCar', 'passive', 'uninitialized', 'dm2Disabled', 'cameraUnavailable'])
def test_integration_unavailable_context_never_adds_phone_warning(gate):
  m = load_events()
  state, _, _ = controller(m)
  if gate in ('notCar', 'passive'):
    setattr(state.CP, gate, True)
  elif gate == 'uninitialized':
    state.initialized = False
  else:
    setattr(state.sm['driverMonitoringState'], gate, True)
  state._update_phone_forward_risk(state.sm.cs)
  assert not state.events.names


def load_classes(filename, ns):
  tree = ast.parse((ROOT / filename).read_text())
  body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom, ast.With))]
  exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(ROOT / filename), 'exec'), ns)
  return SimpleNamespace(**ns)


def test_alert_lifetime_and_fcw_priority_are_unchanged():
  m = load_events()
  am_module = load_classes('alertmanager.py', {'copy': copy, 'defaultdict': defaultdict, 'dataclass': dataclass,
                                                  'Alert': m.Alert, 'EmptyAlert': m.EmptyAlert})
  state, _, _ = controller(m)
  state.update_events()
  am = am_module.AlertManager()
  am.add_many(100, state.events.create_alerts([m.ET.WARNING]))
  am.process_alerts(300, set())
  assert am.current_alert.alert_type == 'phoneLeadClosing/warning'
  am.process_alerts(301, set())
  assert am.current_alert is m.EmptyAlert
  state.events.add(m.EventName.fcw)
  am.add_many(400, state.events.create_alerts([m.ET.WARNING, m.ET.PERMANENT]))
  am.process_alerts(400, set())
  assert am.current_alert.alert_type == 'fcw/permanent'
  assert am.current_alert.audible_alert == m.AudibleAlert.stopStop
  assert am.current_alert.priority == m.Priority.HIGHEST


def test_warning_does_not_enable_disable_override_or_clear_lockout():
  m = load_events()
  machine_module = load_classes('state.py', {'log': log, 'Events': m.Events, 'ET': m.ET, 'DT_CTRL': .01})
  for initial in ('disabled', 'enabled', 'preEnabled', 'softDisabling', 'overriding'):
    for extras in ((), (m.EventName.buttonEnable,), (m.EventName.buttonEnable, m.EventName.tooDistracted),
                   (m.EventName.fcw,), (m.EventName.driverDistracted3,)):
      base, warned = machine_module.StateMachine(), machine_module.StateMachine()
      base.state = warned.state = getattr(log.SelfdriveState.OpenpilotState, initial)
      base.soft_disable_timer = warned.soft_disable_timer = 10
      events = m.Events()
      for event in extras:
        events.add(event)
      baseline_result = base.update(events)
      events.add(m.EventName.phoneLeadClosing)
      assert warned.update(events) == baseline_result
      assert vars(warned) == vars(base)
