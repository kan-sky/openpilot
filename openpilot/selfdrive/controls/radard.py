#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# Kans: sticky lead selection (ported from devel). A previously-selected
# track keeps being reported as the lead for up to STICKY_SELECTED_COUNT_MAX
# frames even when this frame's vision match fails, protected by
# track_discontinuous() resetting selected_count on any large dRel/yRel/vLead
# jump. dPath/in_lane_prob (below) were later ported back in from carrot-wip
# for the match tie-break and sticky drift guard - these come straight from
# modelV2 (laneLines/position), not from ajouatom's lane_planner2.py, which
# this fork is still deliberately moving away from (the actual cut-in-
# detection system stays excluded).
STICKY_SELECTED_COUNT_MAX = int(2.0 / DT_MDL)

# Kans (carrot-wip): lateral drift guard for a sticky-selected track - dPath
# here is against the EGO'S PLANNED PATH (md.position), not lane lines, and
# is unrelated to ajouatom's lane_planner2.py/cut-in system. If a sticky
# track wanders further than this off the ego path it's probably drifted
# onto an adjacent-lane/wrong object, so its sticky status gets dropped.
STICKY_MAX_DPATH = 0.8
STICKY_FAR_DREL = 60.0
STICKY_MAX_DPATH_FAR = 1.2
STICKY_PATH_Y_STD_GAIN = 0.5


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    self.dRel = 0.0
    self.yRel = 0.0
    self.vRel = 0.0
    self.vLead = v_lead

    # Kans: sticky-selection state (devel)
    self.selected_count = 0
    self.is_stopped_car_count = 0

    # Kans (carrot-wip): dPath/in_lane_prob (from md.laneLines, for the
    # match_vision_to_track tie-break) and sticky_dPath (from md.position,
    # the ego path, for the sticky drift guard). Neither is lane_planner2-
    # dependent - both come straight from modelV2.
    self.dPath = 0.0
    self.in_lane_prob = 1.0
    self.lane_half_width = 1.85
    self.sticky_dPath = 0.0
    self.sticky_path_y_std = 0.0

    # Kans: vlead_for_matching() noise-suppression state (devel)
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, radar_reaction_factor: float = 1.0,
             md=None):
    prev_dRel = self.dRel
    prev_yRel = self.yRel
    prev_vLead = self.vLead

    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # Kans: reset sticky-selection state on a large frame-to-frame jump, so a
    # track-ID reuse/glitch can't be mistaken for a continuously-tracked lead.
    track_discontinuous = self.cnt > 0 and (
      abs(self.dRel - prev_dRel) > 5.0 or
      abs(self.yRel - prev_yRel) > 2.0 or
      abs(self.vLead - prev_vLead) > 7.0
    )
    if track_discontinuous:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self._vLead_filt_init = False

    # Kans (carrot-wip): refresh dPath/in_lane_prob for matching, and drop
    # sticky status if a sticky track has drifted off the ego path.
    if md is not None:
      self.d_path(md)
      if self.selected_count > 0:
        self.sticky_dPath, self.sticky_path_y_std = self.path_d_path(md)
        if abs(self.sticky_dPath) > self.sticky_dpath_limit():
          self.selected_count = 0
          self.is_stopped_car_count = 0

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration. Kans (devel): RadarReactionFactor scales
    # both the threshold and the learned time constant - tz has no separate
    # aLead/jLead (only the Kalman-filtered aLeadK), so this uses aLeadK in
    # place of devel's aLead and drops the jLead check devel adds on top.
    a_lead_threshold = 0.5 * radar_reaction_factor
    if abs(self.aLeadK) < a_lead_threshold:
      self.aLeadTau.x = _LEAD_ACCEL_TAU * radar_reaction_factor
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def d_path(self, md):
    # Kans (carrot-wip): dPath/in_lane_prob against the lane lines model
    # gives us directly (md.laneLines), independent of lane_planner2.py.
    if len(md.laneLines) < 3 or len(md.laneLines[1].x) < 2:
      return
    lane_xs = md.laneLines[1].x
    left_ys = md.laneLines[1].y
    right_ys = md.laneLines[2].y

    left_lane_y = np.interp(self.dRel, lane_xs, left_ys)
    right_lane_y = np.interp(self.dRel, lane_xs, right_ys)
    center_y = (left_lane_y + right_lane_y) / 2.0
    lane_half_width = max(0.1, abs(right_lane_y - left_lane_y) / 2.0)
    dist_from_center = self.yRel + center_y

    self.dPath = float(dist_from_center)
    self.in_lane_prob = float(max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width)))
    self.lane_half_width = float(lane_half_width)

  def path_d_path(self, md) -> tuple[float, float]:
    # Kans (carrot-wip): dPath against the ego's own planned path
    # (md.position), used only for the sticky drift guard.
    if len(md.position.x) < 2:
      return self.dPath, 0.0
    path_y = float(np.interp(self.dRel, md.position.x, md.position.y))
    path_y_std = float(np.interp(self.dRel, md.position.x, md.position.yStd)) if len(md.position.yStd) else 0.0
    return float(self.yRel + path_y), path_y_std

  def sticky_dpath_limit(self) -> float:
    if self.dRel < STICKY_FAR_DREL:
      return STICKY_MAX_DPATH
    return float(np.clip(STICKY_MAX_DPATH + STICKY_PATH_Y_STD_GAIN * self.sticky_path_y_std,
                         STICKY_MAX_DPATH, STICKY_MAX_DPATH_FAR))

  def vlead_for_matching(self, dv_max: float = 4.0, alpha: float = 0.35) -> float:
    # Kans (devel): spike-clamp + IIR-smooth vLead for matching-score use only
    # (published vLead/vLeadK are untouched). If cnt < 2: raw vLead.
    v = float(self.vLead)

    if self.cnt < 2:
      return v

    if not self._vLead_filt_init:
      self._vLead_last = v
      self._vLead_filt = v
      self._vLead_filt_init = True
      return v

    v_last = self._vLead_last
    self._vLead_last = v

    v_clamped = float(np.clip(v, v_last - dv_max, v_last + dv_max))
    self._vLead_filt = alpha * v_clamped + (1.0 - alpha) * self._vLead_filt
    return float(self._vLead_filt)

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, lead_prob: float,
                          tracks: dict[int, Track], update_counters: bool = True):
  # Kans (devel): distance/velocity/lateral "sane" gates, a moving-bias
  # tolerance on vel_sane so a lead that's just started moving from a stop
  # isn't rejected by raw-vLead noise, a graduated lead_prob acceptance
  # floor for an already-selected track, and a dedicated "stopped-car-like"
  # match policy (case B) that needs ~1s of consistent evidence before
  # promoting a track that fails the strict velocity gate but passes
  # distance/wide-y. Case A also has a carrot-wip in-lane tie-break - see
  # STICKY_SELECTED_COUNT_MAX comment above for the dPath/in_lane_prob note.
  if not tracks:
    return None

  offset_vision_dist = float(lead.x[0] - RADAR_TO_CAMERA)

  max_vision_dist = max(offset_vision_dist * 1.25, 5.0)
  min_vision_dist = max(offset_vision_dist * 0.80, 1.0)
  max_vision_dist_wide = max(offset_vision_dist * 1.45, 5.0)
  min_vision_dist_wide = 1.5

  vel_tol = float(max(lead.v[0] * np.interp(lead_prob, [0.8, 0.98], [0.3, 0.5]), 5.0))
  vel_guard = max(vel_tol * 3.0, 20.0)

  def dist_sane(t: Track, wide: bool = False) -> bool:
    if wide:
      return min_vision_dist_wide < t.dRel < max_vision_dist_wide
    return min_vision_dist < t.dRel < max_vision_dist

  def y_sane(t: Track, wide: bool = False) -> bool:
    lim = 4.0 if wide else 2.0
    return abs(t.yRel + float(lead.y[0])) < lim

  def vel_sane(t: Track) -> bool:
    v_vis = float(lead.v[0])
    v_trk = float(t.vLead)
    dv = abs(v_trk - v_vis)
    if dv < vel_tol:
      return True
    # moving-bias: allow more mismatch once the track is actually moving,
    # within a guardrail, so a lead that's just resumed from a stop isn't
    # rejected by a noisy instantaneous vLead reading.
    moving = v_trk > 3.0
    if not moving:
      return False
    return dv <= vel_guard

  def score(t: Track) -> float:
    pd = laplacian_pdf(float(t.dRel), offset_vision_dist, float(lead.xStd[0]))
    py = laplacian_pdf(float(t.yRel), -float(lead.y[0]), float(lead.yStd[0]))
    pv = laplacian_pdf(t.vlead_for_matching(), float(lead.v[0]), float(lead.vStd[0]))
    return pd * py * pv

  first_track, second_track = None, None
  first_score, second_score = -1e18, -1e18
  for t in tracks.values():
    s = score(t)
    t.score = s
    if s > first_score:
      second_track, second_score = first_track, first_score
      first_track, first_score = t, s
    elif s > second_score:
      second_track, second_score = t, s

  best_track = None
  if first_track is not None and first_score >= 1e-4:
    # A) normal match. Kans (carrot-wip): if a closer, in-lane second_track
    # is also plausible, prefer it over first_track's raw score - this stops
    # frame-to-frame flip-flopping between two similarly-scored tracks (e.g.
    # a stopped lead vs. a track just behind it) from bouncing the selected
    # lead (and therefore the MPC's obstacle source) back and forth.
    select_second_track = (
      second_track is not None and dist_sane(first_track) and vel_sane(first_track) and
      vel_sane(second_track) and second_track.in_lane_prob > 0.3 and second_track.cnt > 5 and
      offset_vision_dist * 0.5 < second_track.dRel < first_track.dRel
    )
    if select_second_track:
      best_track = second_track
    elif dist_sane(first_track) and vel_sane(first_track) and y_sane(first_track):
      if lead_prob > 0.5:
        best_track = first_track
      elif lead_prob > 0.4 and first_track.selected_count > 0:
        best_track = first_track

    # B) stopped-car-like (only if not chosen yet)
    if best_track is None and dist_sane(first_track) and y_sane(first_track, wide=True):
      if (second_track is not None and second_score > 1e-5 and
          dist_sane(second_track) and y_sane(second_track) and vel_sane(second_track)):
        best_track = second_track
      elif first_track.selected_count > 0:
        best_track = first_track
      else:
        first_track.is_stopped_car_count += 2
        if first_track.is_stopped_car_count > int(1.0 / DT_MDL):
          best_track = first_track

  if update_counters:
    for t in tracks.values():
      if t is best_track:
        t.selected_count = min(t.selected_count + 1, STICKY_SELECTED_COUNT_MAX)
      elif best_track is not None:
        t.selected_count = 0
        t.is_stopped_car_count = max(0, t.is_stopped_car_count - 1)

  return best_track


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    # Kans (devel): RadarReactionFactor - scales the lead-acceleration
    # learning threshold/time-constant (see Track.update()). Default 0.2
    # matches devel's declared param default (20 -> *0.01).
    self.params = Params()
    self._param_frame = 0
    self.radar_reaction_factor = 0.2

    # Kans: debug - suspected cut-in-like deceleration investigation. Edge-triggered
    # on leadOne's selected *radar* track changing identity, so it prints once per
    # switch instead of every frame.
    self._debug_prev_lead_id: int | None = None

  def get_sticky_track(self, tracks: dict[int, Track]) -> Track | None:
    # Kans (devel): keep reporting a previously-selected track as the lead
    # even when this frame's vision match fails, as long as it's still being
    # measured and hasn't been reset by a track_discontinuous() jump.
    # Kans (carrot-wip): also drop sticky status here for a track that's
    # drifted off the ego path since its last update() (see sticky_dPath).
    sticky_tracks = []
    for t in tracks.values():
      if t.selected_count > 0 and abs(t.sticky_dPath) > t.sticky_dpath_limit():
        t.selected_count = 0
        t.is_stopped_car_count = 0
        continue
      if t.cnt > 2 and t.selected_count > 0 and 1.0 < t.dRel < 150.0:
        sticky_tracks.append(t)
    if not sticky_tracks:
      return None
    return max(sticky_tracks, key=lambda t: (t.selected_count, -t.dRel))

  def get_lead(self, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
               model_v_ego: float, lead_prob: float, low_speed_override: bool = True,
               sticky: bool = False) -> dict[str, Any]:
    # Determine leads, this is where the essential logic happens
    v_ego = self.v_ego
    ready = self.ready
    if len(tracks) > 0 and ready and lead_prob > .4:
      track = match_vision_to_track(v_ego, lead_msg, lead_prob, tracks, update_counters=sticky)
    else:
      track = None

    if track is None and sticky:
      track = self.get_sticky_track(tracks)
      if track is not None:
        track.selected_count = min(track.selected_count + 1, STICKY_SELECTED_COUNT_MAX)

    lead_dict = {'present': False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_prob)
    elif (track is None) and ready and (lead_prob > .5):
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
          lead_dict = closest_track.get_RadarState()

    return lead_dict

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    self._param_frame += 1
    if self._param_frame % 100 == 0:
      self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, self.radar_reaction_factor,
                              md=sm['modelV2'] if self.ready else None)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      self.radar_state.leadOne = self.get_lead(self.tracks, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x, low_speed_override=True, sticky=True)
      self.radar_state.leadTwo = self.get_lead(self.tracks, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x, low_speed_override=False, sticky=False)

      lead_one = self.radar_state.leadOne
      new_lead_id = lead_one.radarTrackId if (lead_one.present and lead_one.radar) else None
      if new_lead_id is not None and new_lead_id != self._debug_prev_lead_id:
        t = self.tracks.get(new_lead_id)
        print(f"[radard lead-switch] prevId={self._debug_prev_lead_id} -> newId={new_lead_id} "
              f"dRel={lead_one.dRel:.1f} yRel={lead_one.yRel:.1f} vLead={lead_one.vLead:.1f} "
              f"vRel={lead_one.vRel:.1f} vEgo={self.v_ego:.1f} "
              f"selectedCount={t.selected_count if t else -1} "
              f"isStoppedCarCount={t.is_stopped_car_count if t else -1}", flush=True)
      self._debug_prev_lead_id = new_lead_id

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
