import copy
import json
import math
import zlib
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.deceleration_display import deceleration_state
from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.deceleration import PHASE_IDS, SOURCE_IDS, badge_code
from openpilot.selfdrive.navdy.display import payload_from_messages
from openpilot.selfdrive.navdy.tests.test_bridge import Messages
from openpilot.selfdrive.navdy.tests.test_display import car, model, selfdrive
from openpilot.selfdrive.navdy.transport import encode_frame


def inputs(source='cam', cap=100, final=80, selected=120):
  return {'car_state': car(vCruise=selected, gasPressed=False, brakePressed=False),
          'selfdrive_state': selfdrive(), 'car_control': NS(latActive=True, longActive=True),
          'longitudinal_plan': NS(cruiseTarget=int(cap)),
          'carrot_man': NS(desiredSpeed=int(cap), desiredSource=source,
                           decelerationCandidates=[NS(source=source, speedKph=cap, targetKph=final, distanceM=100)]),
          'model': model(), 'onroad': True}


@pytest.mark.parametrize('source', list(SOURCE_IDS))
@pytest.mark.parametrize('cap,final,selected,phase', [(250, 80, 120, 'waiting'), (100, 80, 120, 'decelerating'),
                                                  (80, 80, 120, 'complete'), (250, 80, 70, 'complete'),
                                                  (100, 0, 120, 'decelerating')])
def test_every_current_reason_shares_c4_verdict(source, cap, final, selected, phase):
  data = inputs(source, cap, final, selected)
  shared = deceleration_state(data['car_state'], data['selfdrive_state'], data['car_control'],
                              data['longitudinal_plan'], data['carrot_man'])
  payload = payload_from_messages(**data)
  if source in ('vturn', 'route') and cap >= selected:
    assert shared is None
    assert payload['decel'] == 0
    return
  assert shared.phase == phase
  assert payload['decel'] == SOURCE_IDS[source] * 4 + PHASE_IDS[shared.phase]


@pytest.mark.parametrize('source', ['vturn', 'route', 'vturn:c', 'route:n'])
@pytest.mark.parametrize('cap,final,selected', [(175.442, 0, 100), (175.442, 175.442, 100),
                                              (120, 80, 120), (250, 80, 120), (100, 80, 70)])
def test_unapplied_curve_is_hidden_without_mutating_messages(source, cap, final, selected):
  data = inputs(source, cap, final, selected)
  before = copy.deepcopy(data)
  assert payload_from_messages(**data)['decel'] == 0
  assert data == before


@pytest.mark.parametrize('source', ['vturn', 'route'])
@pytest.mark.parametrize('fault', ['gas', 'brake', 'other_source', 'plan_mismatch', 'cap_mismatch'])
def test_curve_only_visible_while_its_limit_is_applied(source, fault):
  data = inputs(source)
  assert payload_from_messages(**data)['decel'] == SOURCE_IDS[source] * 4 + 2
  if fault in ('gas', 'brake'):
    setattr(data['car_state'], fault + 'Pressed', True)
  elif fault == 'other_source':
    data['carrot_man'].desiredSource = 'cam'
  elif fault == 'plan_mismatch':
    data['longitudinal_plan'].cruiseTarget = 90
  else:
    data['carrot_man'].decelerationCandidates[0].speedKph = 105
  assert payload_from_messages(**data)['decel'] == 0


@pytest.mark.parametrize('source', ['vturn', 'route'])
def test_hidden_curve_cannot_mask_other_pending_information(source):
  data = inputs(source, cap=100, final=50)
  data['longitudinal_plan'].cruiseTarget = 120  # Curve not yet applied by the planner.
  data['carrot_man'].decelerationCandidates.append(NS(source='cam', speedKph=250, targetKph=80, distanceM=300))
  assert payload_from_messages(**data)['decel'] == SOURCE_IDS['cam'] * 4 + 1


@pytest.mark.parametrize('source', ['vturn', 'route'])
def test_curve_application_completion_and_release(source):
  for cap, final, phase in [(160, 80, 0), (110, 80, 2), (90, 0, 2), (80, 80, 3), (160, 160, 0)]:
    assert payload_from_messages(**inputs(source, cap, final))['decel'] == (SOURCE_IDS[source] * 4 + phase if phase else 0)


def test_wire_ids_are_stable_and_exclude_unproduced_legacy_reasons():
  assert list(SOURCE_IDS.items()) == list(zip(
    ('cam', 'section', 'bump', 'police', 'waze', 'road', 'atc', 'atc2', 'hda', 'hda_section', 'hda_bump', 'school', 'vturn', 'route'),
    range(1, 15), strict=True))
  for source in ('model', 'turn', 'unknown', 'gas'):
    assert payload_from_messages(**inputs(source))['decel'] == 0


@pytest.mark.parametrize('source', ['cam:n', 'hda:v', 'vturn:c'])
def test_source_suffix_uses_same_code(source):
  base = source.split(':')[0]
  assert payload_from_messages(**inputs(source))['decel'] == SOURCE_IDS[base] * 4 + 2


@pytest.mark.parametrize('change', ['disabled', 'inactive', 'longInactive', 'offroad', 'reverse', 'badCan',
                                   'canTimeout', 'critical', 'smallAlert', 'fullAlert', 'missingCandidates', 'clear'])
def test_explicit_clear_gates(change):
  data = inputs()
  if change == 'disabled':
    data['selfdrive_state'].enabled = False
  elif change == 'inactive':
    data['selfdrive_state'].active = False
  elif change == 'longInactive':
    data['car_control'].longActive = False
  elif change == 'offroad':
    data['onroad'] = False
  elif change == 'reverse':
    data['car_state'].gearShifter = 'reverse'
  elif change == 'badCan':
    data['car_state'].canValid = False
  elif change == 'canTimeout':
    data['car_state'].canTimeout = True
  elif change == 'critical':
    data['selfdrive_state'].alertStatus = 'critical'
  elif change in ('smallAlert', 'fullAlert'):
    data['selfdrive_state'].alertSize = 'small' if change == 'smallAlert' else 'full'
  elif change == 'missingCandidates':
    del data['carrot_man'].decelerationCandidates
  else:
    data['carrot_man'].decelerationCandidates = []
  assert payload_from_messages(**data)['decel'] == 0


@pytest.mark.parametrize('missing', ['car_state', 'selfdrive_state', 'car_control', 'longitudinal_plan', 'carrot_man'])
def test_missing_messages_hide_without_changing_other_data(missing):
  data = inputs()
  data[missing] = None
  assert payload_from_messages(**data)['decel'] == 0


@pytest.mark.parametrize('field', ['speedKph', 'targetKph'])
@pytest.mark.parametrize('value', [None, 'bad', math.nan, math.inf, -1])
def test_invalid_metadata_fails_closed(field, value):
  data = inputs()
  setattr(data['carrot_man'].decelerationCandidates[0], field, value)
  assert payload_from_messages(**data)['decel'] == 0


@pytest.mark.parametrize('override', ['gasPressed', 'brakePressed'])
def test_driver_override_is_white_not_completed(override):
  data = inputs(cap=80)
  assert payload_from_messages(**data)['decel'] == 7
  setattr(data['car_state'], override, True)
  assert payload_from_messages(**data)['decel'] == 5


def test_selection_uses_control_set_speed_not_obd_or_cluster_display():
  data = inputs(cap=100, selected=120)
  data['car_state'].vEgo = 70 / 3.6
  assert payload_from_messages(**data)['decel'] == 6  # Slow actual vehicle is not completion.
  data['car_state'].vCruise = 80
  data['car_state'].vCruiseCluster = 82
  payload = payload_from_messages(**data)
  assert payload['setSpeedKph'] == 82  # Existing cluster calibration stays unchanged.
  assert payload['decel'] == 7         # Internal selected cap already satisfies final 80.


@pytest.mark.parametrize('source', list(SOURCE_IDS))
def test_new_field_only_and_bounded_wire_overhead(source):
  data = inputs(source)
  before = copy.deepcopy(data)
  payload = payload_from_messages(**data)
  without_metadata = dict(data, carrot_man=None)
  expected = payload_from_messages(**without_metadata)
  assert {k: v for k, v in payload.items() if k != 'decel'} == {k: v for k, v in expected.items() if k != 'decel'}
  assert data == before
  base = {k: v for k, v in payload.items() if k != 'decel'}
  raw, old = encode_frame(payload), encode_frame(base)
  assert 10 <= len(raw) - len(old) <= 11
  # Compression may shift matches beyond the new field; bound actual wire growth too.
  assert len(zlib.compress(raw)) - len(zlib.compress(old)) <= 32
  assert json.loads(raw)['decel'] == payload['decel']
  assert len(raw) < 32768
  assert bridge.DISPLAY_INTERVAL == 0.2  # No new packets or faster radio cadence.


@pytest.mark.parametrize('service', ['carState', 'selfdriveState', 'carControl', 'longitudinalPlan', 'carrotMan'])
@pytest.mark.parametrize('fault', ['receipt', 'publication', 'invalid', 'unseen', 'future'])
def test_bridge_freshness_clears_badge(service, fault, monkeypatch):
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: 100.)
  sm = Messages()
  data = inputs()
  sm.data.update(carState=data['car_state'], selfdriveState=data['selfdrive_state'], carControl=data['car_control'],
                 longitudinalPlan=data['longitudinal_plan'], carrotMan=data['carrot_man'])
  assert bridge.live_payload(sm, 1)['decel'] == 6
  if fault == 'receipt':
    sm.recv_time[service] = 99.5
  elif fault == 'publication':
    sm.logMonoTime[service] = 99_500_000_000
  elif fault == 'invalid':
    sm.valid[service] = False
  elif fault == 'unseen':
    sm.seen[service] = False
  else:
    sm.logMonoTime[service] = 101_000_000_000
  assert bridge.live_payload(sm, 2)['decel'] == 0


def test_bridge_missed_tick_is_not_a_badge_clear(monkeypatch):
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: 100.)
  sm = Messages()
  data = inputs()
  sm.data.update(carState=data['car_state'], selfdriveState=data['selfdrive_state'], carControl=data['car_control'],
                 longitudinalPlan=data['longitudinal_plan'], carrotMan=data['carrot_man'])
  sm.delay('carrotMan', 0.2)
  assert bridge.live_payload(sm, 1)['decel'] == 6


def test_absent_state_has_zero_code():
  assert badge_code(None, None, None, None, None) == 0
