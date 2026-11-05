from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.phone_forward_risk import PhoneForwardRisk, lead_closing_risk


def lead(**kwargs):
  return SimpleNamespace(**dict({'status': True, 'dRel': 30.0, 'vRel': 0.0, 'vLead': 20.0, 'fcw': False,
                                    'radar': True, 'radarTrackId': 4}, **kwargs))


@pytest.mark.parametrize('speed,values,previous,expected', [
  (24, {'dRel': 22, 'vRel': -7, 'vLead': 17}, None, True),
  (24, {'dRel': 35, 'vRel': 0, 'vLead': 0}, None, False),
  (24, {'dRel': 22.5, 'vRel': -2, 'vLead': 22}, 23.1, False),
  (24, {'dRel': 35, 'vRel': -6, 'vLead': 18}, 35.2, False),
  (24, {'dRel': 22.5, 'vRel': -.4, 'vLead': 23.6}, 23.1, False),
  (24, {'dRel': 42.8, 'vRel': -.4, 'vLead': 23.6}, 43.2, False),
  (18, {'dRel': 24, 'vRel': 0, 'vLead': 18, 'fcw': True}, 24, False),
  (22, {'dRel': 60, 'vRel': .2, 'vLead': 22.2}, None, False),
  (24, {'dRel': 16, 'vRel': -4}, 16.5, True),
  (24, {'dRel': 16, 'vRel': -4}, 16.49, False),
  (24, {'dRel': 16.01, 'vRel': -4}, 16.6, False),
  (24, {'dRel': 16, 'vRel': -3.99}, 17, False),
  (24, {'dRel': 30, 'vRel': -7.5}, 31, True),
  (24, {'dRel': 40, 'vRel': -10}, None, False),
  (24, {'dRel': 39.9, 'vRel': -10}, None, True),
  (5, {'dRel': 20, 'vRel': -5}, None, True),
  (4.99, {'dRel': 20, 'vRel': -5}, None, False),
  (24, {'dRel': 20.01, 'vRel': -5}, None, False),
  (24, {'dRel': 10, 'vRel': -4.99}, None, False),
  (24, {'dRel': 0, 'vRel': -5}, None, False),
  (24, {'dRel': -1, 'vRel': -5}, None, False),
  (24, {'status': False, 'dRel': 10, 'vRel': -5}, None, False),
])
def test_reference_conditions_and_boundaries(speed, values, previous, expected):
  # The first eight cases reproduce Wayon's b7660d1d tests; the rest cover edges.
  assert lead_closing_risk(speed, lead(**values), previous is not None, previous) is expected


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), None, 'invalid'])
@pytest.mark.parametrize('field', ['vEgo', 'dRel', 'vRel'])
def test_nonfinite_or_missing_inputs_fail_closed(bad, field):
  values = {'dRel': 10, 'vRel': -7}
  speed = 24
  if field == 'vEgo':
    speed = bad
  else:
    values[field] = bad
  assert not lead_closing_risk(speed, lead(**values), True, 30)
  assert not lead_closing_risk(24, SimpleNamespace(status=True))
  assert not lead_closing_risk(24, None)


@pytest.mark.parametrize('dt', [0, -.01, .151, 1, float('nan'), float('inf')])
def test_observed_history_requires_bounded_interval(dt):
  assert not lead_closing_risk(24, lead(dRel=16, vRel=-4), True, 16.5, dt=dt)
  assert lead_closing_risk(24, lead(dRel=16, vRel=-5), True, 16.5, dt=dt)


def test_observed_history_must_be_initialized_and_finite():
  for previous in (float('nan'), float('inf'), -float('inf')):
    assert not lead_closing_risk(24, lead(dRel=16, vRel=-4), True, previous)
  assert not lead_closing_risk(24, lead(dRel=16, vRel=-4), True, 16.5, False)


class Inputs(dict):
  """Real schema payloads with controllable publication clocks, without IPC."""
  def __init__(self):
    super().__init__(driverMonitoringState=log.DriverMonitoringState.new_message(activePolicy='vision'),
                     radarState=log.RadarState.new_message())
    self['driverMonitoringState'].visionPolicyState.distractedTypes.phone = True
    self['radarState'].leadOne = {'status': True, 'dRel': 22, 'vRel': -7, 'radar': True, 'radarTrackId': 4}
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)
    self.updated = dict.fromkeys(self, True)
    self.logMonoTime = dict.fromkeys(self, int(10e9))
    self.cs = car.CarState.new_message(vEgo=24, canValid=True)
    self.cs_time = 10.0

  def publish(self, now, services=('driverMonitoringState', 'radarState')):
    self.updated = dict.fromkeys(self, False)
    for service in services:
      self.updated[service] = True
      self.logMonoTime[service] = round(now * 1e9)
    self.cs_time = now

  def all_checks(self, services):
    return all(self.valid[s] and self.alive[s] for s in services)

  def tick(self, tracker, now=10.0, enabled=True):
    return tracker.update_live(now, self.cs, self, enabled, car_state_time_s=self.cs_time)


@pytest.mark.parametrize('service', ['driverMonitoringState', 'radarState'])
@pytest.mark.parametrize('fault', ['invalid', 'dead', 'old', 'future', 'zero', 'nan', 'inf'])
def test_no_warning_from_stale_or_invalid_messages(service, fault):
  sm, tracker = Inputs(), PhoneForwardRisk()
  if fault in ('invalid', 'dead'):
    (sm.valid if fault == 'invalid' else sm.alive)[service] = False
  else:
    sm.logMonoTime[service] = {'old': int(9.749e9), 'future': int(10.001e9), 'zero': 0,
                                   'nan': float('nan'), 'inf': float('inf')}[fault]
  assert not sm.tick(tracker)
  assert tracker.previous_lead is None


@pytest.mark.parametrize('field', ['canValid', 'speed', 'time_old', 'time_future', 'time_zero',
                                 'time_nan', 'disabled', 'camera', 'radar_error', 'not_enabled'])
def test_context_failures_do_not_warn(field):
  sm, tracker = Inputs(), PhoneForwardRisk()
  if field == 'canValid':
    sm.cs.canValid = False
  elif field == 'speed':
    sm.cs.vEgo = float('nan')
  elif field.startswith('time_'):
    sm.cs_time = {'time_old': 9.749, 'time_future': 10.001, 'time_zero': 0, 'time_nan': float('nan')}[field]
  elif field == 'disabled':
    sm['driverMonitoringState'].dm2Disabled = True
  elif field == 'camera':
    sm['driverMonitoringState'].cameraUnavailable = True
  elif field == 'radar_error':
    sm['radarState'].radarErrors.radarFault = True
  assert not sm.tick(tracker, enabled=field != 'not_enabled')


def test_only_phone_boolean_and_new_dm_can_start_warning():
  sm, tracker = Inputs(), PhoneForwardRisk()
  kinds = sm['driverMonitoringState'].visionPolicyState.distractedTypes
  kinds.phone, kinds.pose, kinds.eye, kinds.sleep = False, True, True, True
  assert not sm.tick(tracker)
  kinds.phone = True
  sm.publish(10.01, ('driverMonitoringState',))
  assert sm.tick(tracker, 10.01)
  for i in range(2, 20):
    sm.publish(10 + i / 100, ())
    assert not sm.tick(tracker, 10 + i / 100)


def test_risk_edge_and_four_second_cooldown_use_time_not_call_count():
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert sm.tick(tracker)
  for i in range(1, 1001):
    now = 10 + i / 100
    sm.publish(now, tuple(sm) if i % 5 == 0 else ())
    assert not sm.tick(tracker, now), 'A sustained risk must not retrigger at cooldown expiry'
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = False
  sm.publish(20.01, ('driverMonitoringState',))
  assert not sm.tick(tracker, 20.01)
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = True
  sm.publish(20.02, ('driverMonitoringState',))
  assert sm.tick(tracker, 20.02)
  for now, phone in ((20.03, False), (20.04, True), (24.01, False), (24.02, True)):
    sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = phone
    sm.publish(now)
    assert sm.tick(tracker, now) is (now == 24.02)


def test_suppressed_cooldown_edge_is_not_delayed_into_a_new_warning():
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert sm.tick(tracker)
  for i in range(1, 601):
    now = 10 + i / 100
    sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = i != 5
    sm.publish(now, tuple(sm) if i % 5 == 0 else ())
    assert not sm.tick(tracker, now)


def test_observed_drop_compares_distinct_radar_samples_and_holds_edge():
  sm, tracker = Inputs(), PhoneForwardRisk()
  lead_one = sm['radarState'].leadOne
  lead_one.dRel, lead_one.vRel = 16.5, -4
  assert not sm.tick(tracker)
  lead_one.dRel = 16
  sm.publish(10.05)
  assert sm.tick(tracker, 10.05)
  for i in range(6, 10):
    sm.publish(10 + i / 100, ('driverMonitoringState',))
    assert not sm.tick(tracker, 10 + i / 100)
    assert tracker.previous_risk
  sm.publish(10.10)
  assert not sm.tick(tracker, 10.10)
  assert not tracker.previous_risk


@pytest.mark.parametrize('discontinuity', ['track', 'source', 'status', 'radar_invalid', 'dm_invalid',
                                         'gap', 'dm_gap', 'clock_back', 'radar_back', 'dm_back', 'speed', 'nan'])
def test_discontinuity_cannot_supply_observed_distance_drop(discontinuity):
  sm, tracker = Inputs(), PhoneForwardRisk()
  target = sm['radarState'].leadOne
  target.dRel, target.vRel = 16.5, -4
  assert not sm.tick(tracker)
  target.dRel = 16
  now = 10.05
  if discontinuity == 'track':
    target.radarTrackId += 1
  elif discontinuity == 'source':
    target.radar = False
  elif discontinuity == 'gap':
    now = 10.151
  elif discontinuity == 'dm_gap':
    now = 10.3
  else:
    sm.publish(10.01)
    if discontinuity == 'status':
      target.status = False
    elif discontinuity in ('radar_invalid', 'dm_invalid'):
      sm.valid['radarState' if discontinuity == 'radar_invalid' else 'driverMonitoringState'] = False
    elif discontinuity == 'clock_back':
      assert not sm.tick(tracker, 9.99)
    elif discontinuity in ('radar_back', 'dm_back'):
      sm.logMonoTime['radarState' if discontinuity == 'radar_back' else 'driverMonitoringState'] = int(9.99e9)
    elif discontinuity == 'speed':
      sm.cs.vEgo = 0
    elif discontinuity == 'nan':
      target.dRel = float('nan')
    assert not sm.tick(tracker, 10.01)
    target.status, target.dRel, sm.cs.vEgo = True, 16, 24
    sm.valid = dict.fromkeys(sm, True)
  sm.publish(now)
  assert not sm.tick(tracker, now)


def test_reused_timestamp_cannot_rearm_or_change_distance_history():
  sm, tracker = Inputs(), PhoneForwardRisk()
  target = sm['radarState'].leadOne
  target.dRel, target.vRel = 16.5, -4
  assert not sm.tick(tracker)
  target.dRel = 16
  # Even an updated flag with a replayed publication time cannot create a drop.
  assert not sm.tick(tracker, 10.01)
  assert tracker.previous_lead[1] == 16.5
  sm.publish(10.05)
  assert sm.tick(tracker, 10.05)


def test_stale_gap_does_not_reuse_phone_or_distance_and_keeps_cooldown():
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert sm.tick(tracker)
  sm.publish(10.30, ('radarState',))
  assert not sm.tick(tracker, 10.30)
  assert tracker.previous_lead is None
  sm.publish(10.35)
  assert not sm.tick(tracker, 10.35)
  assert tracker.last_alert == 10
  sm.publish(14)
  assert sm.tick(tracker, 14)


def test_clock_rewind_does_not_erase_cooldown():
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert sm.tick(tracker)
  sm.publish(5)
  assert not sm.tick(tracker, 5)
  sm.publish(5.05)
  assert not sm.tick(tracker, 5.05)
  sm.publish(5.1)
  assert not sm.tick(tracker, 5.1)
  assert tracker.last_alert == 5


@pytest.mark.parametrize('now', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_clock_never_warns(now):
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert not sm.tick(tracker, now)
  assert tracker.previous_lead is None


@pytest.mark.parametrize('field,bad', [('dRel', float('nan')), ('vRel', -float('inf')), ('status', False)])
def test_invalid_lead_during_new_dm_does_not_use_cached_risk(field, bad):
  sm, tracker = Inputs(), PhoneForwardRisk()
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = False
  assert not sm.tick(tracker)
  setattr(sm['radarState'].leadOne, field, bad)
  sm['driverMonitoringState'].visionPolicyState.distractedTypes.phone = True
  sm.publish(10.01, ('driverMonitoringState',))
  assert not sm.tick(tracker, 10.01)
  assert not tracker.lead_risk


def test_input_validity_recovery_without_new_messages_cannot_warn():
  sm, tracker = Inputs(), PhoneForwardRisk()
  sm.valid['radarState'] = False
  assert not sm.tick(tracker)
  sm.valid['radarState'] = True
  assert not sm.tick(tracker, 10.01)
  sm.publish(10.05)
  assert sm.tick(tracker, 10.05)


@pytest.mark.parametrize('age,expected', [(.249, True), (.25, True), (.251, False)])
def test_message_freshness_boundary(age, expected):
  sm, tracker = Inputs(), PhoneForwardRisk()
  assert sm.tick(tracker, 10 + age) is expected
