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
from openpilot.selfdrive.controls.lib.cutin_helpers import (
  FRONT_CUTIN_MIN_CONFIRM_S,
  associate_cutin_tracks,
  combine_cutin_future_projection,
  cutin_confirmation_frames,
  cutin_entry_rejection_reason,
  cutin_min_track_age_frames,
  cutin_tuning_from_sensitivity,
  effective_cutin_inward_speed,
  is_cutin_track_discontinuous,
  is_fast_cutin_entry,
  is_front_radar_cutin_candidate,
  new_cutin_position_history,
  update_cutin_confirmation,
  update_lane_relative_motion,
)


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

# Kans (devel): EnableRadarTracks <= this forces vision-only mode (radar
# tracks cleared and ignored every frame). A real radar CAN fault
# (rr.errors.canError/radarFault) forces the same mode automatically,
# regardless of the param. devel's other EnableRadarTracks values (-1/1/2/3)
# select between SCC-radar/cut-in/corner-radar sources the Volt doesn't
# have, so tz only implements this one threshold.
VISION_ONLY_RADAR_TRACK_MODE = -2

# Kans (devel): front-radar cut-in detection constants. Corner-radar/SCC
# variants of these (CORNER_*, SIDE_CORNER_*) are dropped - see
# lib/cutin_helpers.py's module docstring for why.
CUTIN_STICKY_FRAMES = int(0.7 / DT_MDL)
CUTIN_OUTPUT_HOLD_FRAMES = max(1, int(round(0.5 / DT_MDL)))
CUTIN_OUTPUT_HOLD_DREL_M = 3.0
CUTIN_OUTPUT_HOLD_YREL_M = 1.0
CUTIN_OUTPUT_HOLD_VREL_MPS = 2.0
CUTIN_KEEP_FUTURE_IN_LANE_PROB = 0.12
CUTIN_KEEP_MAX_DPATH_FUTURE = 1.6
CUTIN_KEEP_MAX_MOVING_AWAY = 0.3
CUTIN_PROMOTE_DREL_MARGIN = 1.0
CUTIN_YAW_COMP_GAIN = 0.6
CUTIN_YAW_COMP_MAX_DREL = 50.0
CUTIN_YAW_COMP_MAX_YAW_RATE = 0.35
CUTIN_YAW_COMP_MAX_YVREL_CORRECTION = 1.5
CUTIN_YAW_COMP_MAX_VREL_CORRECTION = 0.6


def clamp(x: float, lo: float, hi: float) -> float:
  return float(np.clip(x, lo, hi))


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

    # Kans (devel): front-radar cut-in state. dRel_future/yRel_future are the
    # yaw-compensated position projected radar_lat_factor seconds ahead;
    # dPath_future/in_lane_prob_future are d_path() applied to that
    # projection. dPath_rate/dPath_inward_speed are the lane-relative-motion
    # estimate computed only while this track is an active cut-in candidate.
    self.cut_in_count = 0
    self.cutin_cnt = 0
    self.cut_in_start_abs_dpath = 0.0
    self.dRel_future = 0.0
    self.yRel_future = 0.0
    self.dPath_future = 0.0
    self.in_lane_prob_future = 0.0
    self.dPath_rate = 0.0
    self.dPath_inward_speed = 0.0
    self._cutin_position_history = new_cutin_position_history(DT_MDL)
    self.cutin_radar_inward_speed = 0.0

    # Kans: vlead_for_matching() noise-suppression state (devel)
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def inherit_cutin_state(self, source: 'Track') -> None:
    # Kans (devel): when associate_cutin_tracks() detects that this frame's
    # track at a new radar ID is really the same physical object as a track
    # from last frame (GM reassigns sequential IDs, so a momentary ID churn
    # would otherwise reset cut-in confirmation progress to zero), copy the
    # old track's cut-in state onto the new one.
    self.dRel = source.dRel
    self.yRel = source.yRel
    self.vRel = source.vRel
    self.vLead = source.vLead
    self.cnt = source.cnt
    self.cut_in_count = source.cut_in_count
    self.cutin_cnt = source.cutin_cnt
    self.cut_in_start_abs_dpath = source.cut_in_start_abs_dpath
    self._cutin_position_history.clear()
    self._cutin_position_history.extend(source._cutin_position_history)
    self.cutin_radar_inward_speed = source.cutin_radar_inward_speed

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, radar_reaction_factor: float = 1.0,
             md=None, radar_lat_factor: float = 0.0, yaw_rate: float = 0.0, is_cutin_track: bool = False,
             v_ego: float = 0.0):
    prev_dRel = self.dRel
    prev_yRel = self.yRel
    prev_vLead = self.vLead
    was_measured = self.cnt > 0

    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # Kans: reset sticky-selection state on a large frame-to-frame jump, so a
    # track-ID reuse/glitch can't be mistaken for a continuously-tracked lead.
    # Kans (devel): while this track is an active cut-in candidate, use the
    # tighter cutin-specific discontinuity thresholds instead - precision
    # matters more there since cut-in confirmation tracks lateral motion.
    track_discontinuous = (
      is_cutin_track_discontinuous(was_measured, prev_dRel, prev_yRel, prev_vLead, self.dRel, self.yRel, self.vLead)
      if is_cutin_track else
      was_measured and (
        abs(self.dRel - prev_dRel) > 5.0 or
        abs(self.yRel - prev_yRel) > 2.0 or
        abs(self.vLead - prev_vLead) > 7.0
      )
    )
    if track_discontinuous:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self.cut_in_count = 0
      self.cutin_cnt = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_position_history.clear()
      self._vLead_filt_init = False

    if is_cutin_track:
      self.cutin_cnt += 1
    else:
      self.cut_in_count = 0
      self.cutin_cnt = 0
      self.cut_in_start_abs_dpath = 0.0

    # Kans (devel): yaw-compensated future position, used by the lane-
    # relative-motion cut-in projection below.
    v_rel_future, yv_rel_future = self.yaw_compensated_velocities(yaw_rate)
    self.dRel_future = self.dRel + v_rel_future * radar_lat_factor
    self.yRel_future = self.yRel + yv_rel_future * radar_lat_factor

    # Kans (carrot-wip): refresh dPath/in_lane_prob for matching, and drop
    # sticky status if a sticky track has drifted off the ego path.
    if md is not None:
      self.d_path(md)

      # Kans (devel): while an active cut-in candidate, estimate lane-
      # relative rate of motion and project it forward to decide whether
      # this track is moving into our lane.
      if is_cutin_track and radar_lat_factor > 0.0:
        self.cutin_radar_inward_speed = max(0.0, -math.copysign(1.0, self.dPath) * yv_rel_future)
        self.dPath_rate, self.dPath_inward_speed = update_lane_relative_motion(
          self._cutin_position_history, self.dRel, self.yRel,
          md.laneLines[1].x, md.laneLines[1].y, md.laneLines[2].y,
          True, track_discontinuous, DT_MDL,
        )
        self.dPath_future, self.in_lane_prob_future = combine_cutin_future_projection(
          self.dPath, self.dPath_rate, radar_lat_factor, self.lane_half_width,
          self.dPath_future, self.in_lane_prob_future, self.cutin_radar_inward_speed,
        )
        self.dPath_inward_speed = effective_cutin_inward_speed(
          self.dRel, v_ego=v_ego, temporal_inward_speed=self.dPath_inward_speed,
          d_path=self.dPath, projected_d_path=self.dPath_future, horizon_s=radar_lat_factor,
        )
      else:
        self._cutin_position_history.clear()
        self.dPath_rate = 0.0
        self.dPath_inward_speed = 0.0
        self.cutin_radar_inward_speed = 0.0

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
    # Kans (devel): also computes the same against the yaw-compensated
    # future position (dRel_future/yRel_future) for cut-in projection.
    if len(md.laneLines) < 3 or len(md.laneLines[1].x) < 2:
      return
    lane_xs = md.laneLines[1].x
    left_ys = md.laneLines[1].y
    right_ys = md.laneLines[2].y

    def d_path_interp(d_rel, y_rel):
      left_lane_y = np.interp(d_rel, lane_xs, left_ys)
      right_lane_y = np.interp(d_rel, lane_xs, right_ys)
      center_y = (left_lane_y + right_lane_y) / 2.0
      lane_half_width = max(0.1, abs(right_lane_y - left_lane_y) / 2.0)
      dist_from_center = y_rel + center_y
      in_lane_prob = max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width))
      return dist_from_center, in_lane_prob, lane_half_width

    self.dPath, self.in_lane_prob, self.lane_half_width = d_path_interp(self.dRel, self.yRel)
    self.dPath_future, self.in_lane_prob_future, _ = d_path_interp(self.dRel_future, self.yRel_future)

  def yaw_compensated_velocities(self, yaw_rate: float) -> tuple[float, float]:
    # Kans (devel): a curved ego path creates apparent lateral velocity in
    # the ego frame (yaw_rate * dRel). Remove it before cut-in projection so
    # adjacent-lane objects on curves aren't classified as moving into our
    # lane. GM never reports a per-target yaw-relative velocity (yvRel is
    # always 0 from opendbc/car/gm/radar_interface.py), so 0.0 stands in for
    # devel's raw yvLead field here.
    yaw_rate = clamp(float(yaw_rate), -CUTIN_YAW_COMP_MAX_YAW_RATE, CUTIN_YAW_COMP_MAX_YAW_RATE)
    d_rel_for_comp = clamp(self.dRel, 0.0, CUTIN_YAW_COMP_MAX_DREL)
    yv_rel_corr = clamp(
      -yaw_rate * d_rel_for_comp * CUTIN_YAW_COMP_GAIN,
      -CUTIN_YAW_COMP_MAX_YVREL_CORRECTION, CUTIN_YAW_COMP_MAX_YVREL_CORRECTION,
    )
    v_rel_corr = clamp(
      yaw_rate * self.yRel * CUTIN_YAW_COMP_GAIN,
      -CUTIN_YAW_COMP_MAX_VREL_CORRECTION, CUTIN_YAW_COMP_MAX_VREL_CORRECTION,
    )
    return float(self.vRel + v_rel_corr), float(yv_rel_corr)

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
    self.enable_radar_tracks = 0

    # Kans (devel): front-radar cut-in detection, ported without the
    # corner-radar/SCC-fallback machinery devel also has (Volt has neither).
    # Gated on CarrotRadarMode (default off - opt-in) instead of devel's
    # car_brand=="hyundai" check; CarrotRadarCutInSensitivity (0-5, UI) gives
    # a live-adjustable dial instead of devel's fixed sensitivity=50.
    self.front_cutin_enabled = False
    self.lane_line_available = False
    self.radar_lat_factor = 0.0
    self.cutin_yaw_rate = 0.0
    self.cutin_yaw_rate_filter = FirstOrderFilter(0.0, 0.20, DT_MDL)
    self.cutin_sensitivity = 50.0
    self.cutin_tuning = cutin_tuning_from_sensitivity(self.cutin_sensitivity)
    self.cutin_confirm_frames = max(1, int(round(self.cutin_tuning["confirm_s"] / DT_MDL)))
    self.front_cutin_confirm_frames = max(self.cutin_confirm_frames, int(round(FRONT_CUTIN_MIN_CONFIRM_S / DT_MDL)))
    self.cutin_min_track_age = max(1, int(round(self.cutin_tuning["min_track_age_s"] / DT_MDL)))
    self.cutin_enter_min_x = self.cutin_tuning["enter_min_x"]
    self.cutin_enter_max_x = self.cutin_tuning["enter_max_x"]
    self.cutin_output_hold_count = 0
    self.cutin_output_hold_reference: tuple[float, float, float] | None = None

    # Kans: debug - suspected cut-in-like deceleration investigation. Edge-triggered
    # on leadOne's selected *radar* track changing identity, so it prints once per
    # switch instead of every frame.
    self._debug_prev_lead_id: int | None = None

    # Kans: debug - front-radar cut-in detection verification. Edge-triggered
    # on a track newly reaching confirmed status, so it prints once per
    # cut-in event instead of every frame it stays confirmed.
    self._debug_prev_cutin_ids: set[int] = set()

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

  # ---- front-radar cut-in detection (Kans, devel - corner/SCC pieces dropped) ----

  def _is_front_cutin_track(self, t: Track) -> bool:
    return is_front_radar_cutin_candidate(t.identifier, t.dRel, t.yRel)

  def _cutin_yaw_rate_from_state(self, sm: messaging.SubMaster) -> float:
    # Kans: devel prefers sm['livePose'].angularVelocityDevice.z when valid,
    # falling back to modelV2.orientationRate.z[0]. tz doesn't subscribe to
    # livePose here, so this always uses the modelV2 fallback - devel's own
    # fallback path, just without the optional upgrade.
    yaw_rate = 0.0
    if len(sm['modelV2'].orientationRate.z):
      yaw_rate = float(sm['modelV2'].orientationRate.z[0])
    yaw_rate = clamp(yaw_rate, -CUTIN_YAW_COMP_MAX_YAW_RATE, CUTIN_YAW_COMP_MAX_YAW_RATE)
    return float(self.cutin_yaw_rate_filter.update(yaw_rate))

  def _track_is_closer_than_lead_one(self, t: Track) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.present:
      return True
    return t.dRel + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

  def _cutin_is_closer_or_matches_lead_one(self, t: Track) -> bool:
    if self._track_is_closer_than_lead_one(t):
      return True
    lead_one = self.radar_state.leadOne
    return bool(lead_one.present and lead_one.radar and int(lead_one.radarTrackId) == t.identifier)

  def _is_cutin_enter_candidate(self, t: Track) -> bool:
    min_track_age = cutin_min_track_age_frames(self.cutin_min_track_age, t.dRel, t.dPath_inward_speed, self.v_ego)
    reason = cutin_entry_rejection_reason(
      enabled=self.front_cutin_enabled,
      lane_line_available=self.lane_line_available,
      is_cutin_candidate=self._is_front_cutin_track(t),
      closer_or_matching=self._cutin_is_closer_or_matches_lead_one(t),
      track_count=t.cutin_cnt,
      min_track_age=min_track_age,
      d_rel=t.dRel,
      v_lead=t.vLead,
      d_path=t.dPath,
      d_path_future=t.dPath_future,
      in_lane_prob=t.in_lane_prob,
      in_lane_prob_future=t.in_lane_prob_future,
      inward_speed=t.dPath_inward_speed,
      tuning=self.cutin_tuning,
      fast_lane_entry=is_fast_cutin_entry(
        t.dRel, self.v_ego, t.dPath, t.lane_half_width, t.dPath_inward_speed,
        t.cutin_radar_inward_speed, v_rel=t.vRel,
      ),
      radar_inward_speed=t.cutin_radar_inward_speed,
    )
    return reason is None

  def _is_cutin_keep_candidate(self, t: Track) -> bool:
    if not self.front_cutin_enabled or not self._is_front_cutin_track(t):
      return False
    if not self._cutin_is_closer_or_matches_lead_one(t):
      return False
    if not (0.8 < t.dRel < 55.0 and t.vLead > 2.0):
      return False
    moving_away = abs(t.dPath_future) - abs(t.dPath)
    if moving_away > CUTIN_KEEP_MAX_MOVING_AWAY:
      return False
    return t.in_lane_prob_future > CUTIN_KEEP_FUTURE_IN_LANE_PROB or abs(t.dPath_future) < CUTIN_KEEP_MAX_DPATH_FUTURE

  def _update_cutin_sticky(self, t: Track) -> bool:
    entering = self._is_cutin_enter_candidate(t)
    keeping = t.cut_in_count > 0 and self._is_cutin_keep_candidate(t)
    if keeping:
      entering = True
    confirm_frames = cutin_confirmation_frames(self.front_cutin_confirm_frames, t.dRel, t.dPath_inward_speed, self.v_ego)
    t.cut_in_count, t.cut_in_start_abs_dpath = update_cutin_confirmation(
      t.cut_in_count, t.cut_in_start_abs_dpath, t.dPath, t.dRel, entering, keeping,
      confirm_frames, CUTIN_STICKY_FRAMES, self.cutin_tuning["enter_min_progress"], self.v_ego,
    )
    return t.cut_in_count >= confirm_frames

  def _apply_cutin_output_hold(self, cutin_list: list[dict[str, Any]]) -> list[dict[str, Any]]:
    # Kans (devel): once a cut-in lead disappears (e.g. a brief miss), keep
    # publishing it for up to CUTIN_OUTPUT_HOLD_FRAMES more frames by
    # re-matching any still-active cut-in track near its last known position,
    # so a momentary detection gap doesn't yank leadTwo back and forth.
    if cutin_list:
      nearest = min(cutin_list, key=lambda lead: float(lead['dRel']))
      self.cutin_output_hold_reference = (float(nearest['dRel']), float(nearest['yRel']), float(nearest['vRel']))
      self.cutin_output_hold_count = CUTIN_OUTPUT_HOLD_FRAMES
      return cutin_list

    reference = self.cutin_output_hold_reference
    if self.cutin_output_hold_count <= 0 or reference is None:
      self.cutin_output_hold_reference = None
      return cutin_list

    d_rel, y_rel, v_rel = reference
    matches = [
      t for t in self.tracks.values()
      if self._is_front_cutin_track(t)
      and abs(t.dRel - d_rel) <= CUTIN_OUTPUT_HOLD_DREL_M
      and abs(t.yRel - y_rel) <= CUTIN_OUTPUT_HOLD_YREL_M
      and abs(t.vRel - v_rel) <= CUTIN_OUTPUT_HOLD_VREL_MPS
    ]
    if not matches:
      self.cutin_output_hold_count = 0
      self.cutin_output_hold_reference = None
      return cutin_list

    track = min(
      matches,
      key=lambda c: abs(c.dRel - d_rel) + abs(c.yRel - y_rel) + 0.5 * abs(c.vRel - v_rel),
    )
    lead = track.get_RadarState(0)
    lead['modelProb'] = 0.03
    self.cutin_output_hold_reference = (track.dRel, track.yRel, track.vRel)
    self.cutin_output_hold_count -= 1
    if self.cutin_output_hold_count == 0:
      self.cutin_output_hold_reference = None
    return [lead]

  def compute_cutin_list(self) -> list[dict[str, Any]]:
    # Kans: tz has no compute_leads()-style orchestrator (that's a much
    # bigger devel structure this fork doesn't have), so this is a new,
    # narrower entry point: advance every track's cut-in confirmation state
    # once per frame (this has to run every frame regardless of whether a
    # track ends up confirmed, since it's a stateful counter) and collect the
    # ones that are currently confirmed.
    if not self.front_cutin_enabled:
      self.cutin_output_hold_reference = None
      self.cutin_output_hold_count = 0
      self._debug_prev_cutin_ids = set()
      return []
    cutin_list = []
    confirmed_ids = set()
    for t in self.tracks.values():
      if self._update_cutin_sticky(t):
        lead = t.get_RadarState(0)
        lead['modelProb'] = 0.03
        cutin_list.append(lead)
        confirmed_ids.add(t.identifier)

    new_ids = confirmed_ids - self._debug_prev_cutin_ids
    for t in self.tracks.values():
      if t.identifier in new_ids:
        print(f"[radard cutin] confirmed trackId={t.identifier} dRel={t.dRel:.1f} yRel={t.yRel:.1f} "
              f"vLead={t.vLead:.1f} dPath={t.dPath:.2f} inwardSpeed={t.dPath_inward_speed:.2f} "
              f"vEgo={self.v_ego:.1f} sensitivity={self.cutin_sensitivity:.0f}", flush=True)
    self._debug_prev_cutin_ids = confirmed_ids

    return self._apply_cutin_output_hold(cutin_list)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    self._param_frame += 1
    if self._param_frame % 100 == 0:
      self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01
      self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
      self.front_cutin_enabled = self.params.get_int("CarrotRadarMode") > 0
      cutin_sensitivity_ui = self.params.get_int("CarrotRadarCutInSensitivity")
      self.cutin_sensitivity = float(np.interp(cutin_sensitivity_ui, [0, 1, 2, 3, 4, 5], [0, 15, 30, 50, 70, 90]))
      self.cutin_tuning = cutin_tuning_from_sensitivity(self.cutin_sensitivity)
      self.cutin_confirm_frames = max(1, int(round(self.cutin_tuning["confirm_s"] / DT_MDL)))
      self.front_cutin_confirm_frames = max(self.cutin_confirm_frames, int(round(FRONT_CUTIN_MIN_CONFIRM_S / DT_MDL)))
      self.cutin_min_track_age = max(1, int(round(self.cutin_tuning["min_track_age_s"] / DT_MDL)))
      self.cutin_enter_min_x = self.cutin_tuning["enter_min_x"]
      self.cutin_enter_max_x = self.cutin_tuning["enter_max_x"]

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    md = sm['modelV2']
    self.lane_line_available = len(md.laneLineProbs) > 2 and md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5
    self.radar_lat_factor = self.cutin_tuning["horizon_s"] if self.front_cutin_enabled else 0.0
    self.cutin_yaw_rate = self._cutin_yaw_rate_from_state(sm) if self.front_cutin_enabled else 0.0

    # Kans (devel): a real radar CAN fault, or EnableRadarTracks forced all
    # the way down, means we don't trust any radar output this frame - drop
    # every track and let get_lead() fall through to the vision-only path.
    radar_faulted = bool(rr.errors.canError or rr.errors.radarFault)
    vision_only_mode = self.enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE or radar_faulted

    if vision_only_mode:
      self.tracks.clear()
    else:
      ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

      # Kans (devel): snapshot which currently-tracked objects are active
      # cut-in candidates before this frame's pruning/creation, then
      # associate them with this frame's candidate points by position so a
      # radar-ID reassignment doesn't reset cut-in confirmation progress.
      previous_cutin_tracks: dict[int, Track] = {}
      cutin_associations: dict[int, int] = {}
      if self.front_cutin_enabled:
        previous_cutin_tracks = {
          tid: t for tid, t in self.tracks.items() if self._is_front_cutin_track(t)
        }
        previous_cutin_positions = {tid: (t.dRel, t.yRel, t.vRel) for tid, t in previous_cutin_tracks.items()}
        current_cutin_points = {
          tid: (float(rpt[0]), float(rpt[1]), float(rpt[2]))
          for tid, rpt in ar_pts.items() if is_front_radar_cutin_candidate(tid, rpt[0], rpt[1])
        }
        cutin_associations = associate_cutin_tracks(previous_cutin_positions, current_cutin_points)

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
          source_id = cutin_associations.get(ids)
          if source_id is not None and source_id != ids and source_id in previous_cutin_tracks:
            self.tracks[ids].inherit_cutin_state(previous_cutin_tracks[source_id])

        is_cutin_track = self.front_cutin_enabled and is_front_radar_cutin_candidate(ids, rpt[0], rpt[1])
        self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, self.radar_reaction_factor,
                                md=md if self.ready else None, radar_lat_factor=self.radar_lat_factor,
                                yaw_rate=self.cutin_yaw_rate if is_cutin_track else 0.0,
                                is_cutin_track=is_cutin_track, v_ego=self.v_ego)

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

      # Kans (devel): confirmed cut-in candidates are published in full via
      # leadsCutIn, and the nearest eligible one (not already leadOne) takes
      # priority over the plain vision-matched leadTwo above - this is how a
      # detected cut-in actually reaches longitudinal control.
      cutin_list = self.compute_cutin_list()
      self.radar_state.leadsCutIn = cutin_list
      if self.front_cutin_enabled and cutin_list:
        lead_one = self.radar_state.leadOne
        eligible = [
          c for c in cutin_list
          if self.cutin_enter_min_x < c['dRel'] < self.cutin_enter_max_x and c['vLead'] > 4.0
          and not (lead_one.present and lead_one.radar and int(lead_one.radarTrackId) == int(c['radarTrackId']))
        ]
        if eligible:
          self.radar_state.leadTwo = min(eligible, key=lambda c: c['dRel'])

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
