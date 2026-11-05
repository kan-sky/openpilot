"""Bounded phone/lead advisory. Never changes DM, FCW or control permissions."""
import math


MIN_FORWARD_RISK_SPEED = 5.0
FAST_CLOSING_DISTANCE = 40.0
FAST_CLOSING_SPEED = 5.0
FAST_CLOSING_TTC = 4.0
FAST_CLOSING_DT = 0.05
OBSERVED_CLOSING_DISTANCE = 30.0
OBSERVED_CLOSING_MIN_DREL_DROP = 0.5
OBSERVED_CLOSING_MIN_REPORTED_SPEED = 4.0
PHONE_FORWARD_RISK_COOLDOWN = 4.0
MAX_INPUT_AGE = 0.25
MAX_HISTORY_GAP = 0.15


def _finite(value):
  try:
    value = float(value)
    return value if math.isfinite(value) else None
  except (TypeError, ValueError, OverflowError):
    return None


def lead_closing_risk(v_ego, lead, previous_lead_status=False, previous_lead_d_rel=0.0,
                      lead_history_initialized=True, dt=FAST_CLOSING_DT):
  """Wayon b7660d1d thresholds, with finite input and bounded history guards."""
  v_ego = _finite(v_ego)
  d_rel = _finite(getattr(lead, 'dRel', None))
  v_rel = _finite(getattr(lead, 'vRel', None))
  if (not getattr(lead, 'status', False) or v_ego is None or v_ego < MIN_FORWARD_RISK_SPEED
      or d_rel is None or d_rel <= 0.0 or v_rel is None):
    return False
  closing_speed = max(0.0, -v_rel)
  previous_d_rel, dt = _finite(previous_lead_d_rel), _finite(dt)
  observed_closing = (lead_history_initialized and previous_lead_status
                      and previous_d_rel is not None and dt is not None and 0.0 < dt <= MAX_HISTORY_GAP
                      and d_rel < OBSERVED_CLOSING_DISTANCE
                      and previous_d_rel - d_rel >= OBSERVED_CLOSING_MIN_DREL_DROP
                      and closing_speed >= OBSERVED_CLOSING_MIN_REPORTED_SPEED)
  fast_closing = d_rel < FAST_CLOSING_DISTANCE and closing_speed >= FAST_CLOSING_SPEED
  return (fast_closing or observed_closing) and d_rel / max(closing_speed, 0.1) <= FAST_CLOSING_TTC


class PhoneForwardRisk:
  def __init__(self):
    self.last_now = None
    self.last_times = (0.0, 0.0)
    self.last_alert = -math.inf
    self._clear_history()

  def _clear_history(self):
    # Keep the cooldown and consumed-message watermarks through invalid input.
    self.previous_lead = None
    self.lead_risk = False
    self.previous_risk = False

  def update_live(self, now, cs, sm, enabled, *, car_state_time_s):
    now = _finite(now)
    if now is None:
      self._clear_history()
      return False
    previous_now, self.last_now = self.last_now, now
    if previous_now is not None and now < previous_now:
      self._clear_history()
      if math.isfinite(self.last_alert):
        self.last_alert = now
      return False
    if previous_now is not None and now - previous_now > MAX_INPUT_AGE:
      self._clear_history()

    services = ('driverMonitoringState', 'radarState')
    timestamps = tuple(_finite(sm.logMonoTime[s]) for s in services)
    if any(t is None for t in timestamps):
      self._clear_history()
      return False
    times = tuple(t / 1e9 for t in timestamps)
    previous_times, self.last_times = self.last_times, times
    if any(t < old for t, old in zip(times, previous_times, strict=True)):
      self._clear_history()
      return False
    if any(old > 0 and t - old > MAX_INPUT_AGE for t, old in zip(times, previous_times, strict=True)):
      self._clear_history()
    fresh = all(sm.valid[s] and sm.alive[s] and t > 0 and 0 <= now - t <= MAX_INPUT_AGE
                for s, t in zip(services, times, strict=True))
    cs_time, v_ego = _finite(car_state_time_s), _finite(cs.vEgo)
    dm, radar = (sm[s] for s in services)
    if (not enabled or not fresh or not cs.canValid or cs_time is None or cs_time <= 0
        or not 0 <= now - cs_time <= MAX_INPUT_AGE or v_ego is None or v_ego < MIN_FORWARD_RISK_SPEED
        or dm.dm2Disabled or dm.cameraUnavailable or any(radar.radarErrors.to_dict().values())):
      self._clear_history()
      return False

    lead = radar.leadOne
    d_rel, v_rel = _finite(lead.dRel), _finite(lead.vRel)
    if not lead.status or d_rel is None or d_rel <= 0 or v_rel is None:
      self._clear_history()
      return False
    new_dm, new_radar = (sm.updated[s] and t > old for s, t, old in zip(services, times, previous_times, strict=True))
    if not new_dm and not new_radar:
      return False
    if new_radar:
      key = (lead.radar, lead.radarTrackId)
      previous = self.previous_lead
      continuous = (previous is not None and key == previous[0]
                    and 0 < times[1] - previous[2] <= MAX_HISTORY_GAP)
      if not continuous:
        self.previous_risk = False
      self.lead_risk = lead_closing_risk(v_ego, lead, previous_lead_status=continuous,
                                         previous_lead_d_rel=previous[1] if continuous else 0.0,
                                         lead_history_initialized=continuous,
                                         dt=times[1] - previous[2] if continuous else FAST_CLOSING_DT)
      self.previous_lead = (key, d_rel, times[1])

    # Repeated 100 Hz polls must not clear the edge from a slower radar sample.
    risk = bool(dm.visionPolicyState.distractedTypes.phone) and self.lead_risk
    warn = risk and not self.previous_risk and now - self.last_alert >= PHONE_FORWARD_RISK_COOLDOWN
    self.previous_risk = risk
    if warn:
      self.last_alert = now
    return warn
