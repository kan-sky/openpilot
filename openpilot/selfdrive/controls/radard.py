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

# Kans: sticky lead selection(devel에서 이식). 이전에 선택됐던 트랙은 이번
# 프레임의 비전 매칭이 실패해도 STICKY_SELECTED_COUNT_MAX 프레임까지는 계속
# lead로 보고된다. 단 track_discontinuous()가 dRel/yRel/vLead에 큰 점프가
# 있으면 selected_count를 리셋해서 보호한다. dPath/in_lane_prob(아래)는
# 나중에 매칭 타이브레이크와 sticky drift 가드용으로 carrot-wip에서 다시
# 이식해온 것 - 이건 ajouatom의 lane_planner2.py가 아니라 modelV2
# (laneLines/position)에서 바로 가져온다. 이 포크는 lane_planner2.py로부터
# 계속 의도적으로 멀어지는 중이다(실제 컷인 감지 시스템은 여전히 제외됨).
STICKY_SELECTED_COUNT_MAX = int(2.0 / DT_MDL)

# Kans (carrot-wip): sticky-선택된 트랙의 횡방향 drift 가드 - 여기서 dPath는
# 차선이 아니라 자차의 계획 경로(md.position) 기준이고, ajouatom의
# lane_planner2.py/컷인 시스템과는 무관하다. sticky 트랙이 자차 경로에서
# 이만큼 벗어나면 인접 차선/엉뚱한 오브젝트로 drift된 걸로 보고 sticky
# 상태를 해제한다.
STICKY_MAX_DPATH = 0.8
STICKY_FAR_DREL = 60.0
STICKY_MAX_DPATH_FAR = 1.2
STICKY_PATH_Y_STD_GAIN = 0.5

# Kans (devel): EnableRadarTracks가 이 값 이하면 비전 전용 모드로 강제된다
# (레이더 트랙을 매 프레임 비우고 무시). 실제 레이더 CAN 오류
# (rr.errors.canError/radarFault)도 파라미터와 무관하게 자동으로 같은
# 모드를 강제한다. devel의 다른 EnableRadarTracks 값들(-1/1/2/3)은 볼트에는
# 없는 SCC레이더/컷인/코너레이더 소스를 선택하는 거라, tz는 이 임계값
# 하나만 구현한다.
VISION_ONLY_RADAR_TRACK_MODE = -2

# Kans (devel-0721, 사용자 선호에 따라 나중에 나온 접미사 없는 스냅샷보다
# 이쪽을 채택 - 그쪽은 carrot/ 통합 이전 버전이고 이 값들이 더 느슨하게
# 틀어져 있었다): 전방 레이더 컷인 감지 상수들.
# 이것들의 코너레이더/SCC 버전(CORNER_*, SIDE_CORNER_*)은 뺐다 - 이유는
# lib/cutin_helpers.py 모듈 docstring 참고.
CUTIN_STICKY_FRAMES = int(0.5 / DT_MDL)
CUTIN_OUTPUT_HOLD_FRAMES = max(1, int(round(0.25 / DT_MDL)))
CUTIN_OUTPUT_HOLD_DREL_M = 3.0
CUTIN_OUTPUT_HOLD_YREL_M = 1.0
CUTIN_OUTPUT_HOLD_VREL_MPS = 2.0
CUTIN_KEEP_FUTURE_IN_LANE_PROB = 0.12
CUTIN_KEEP_MAX_DPATH_FUTURE = 1.6
CUTIN_KEEP_MAX_MOVING_AWAY = 0.3
CUTIN_PROMOTE_DREL_MARGIN = 1.0
# Kans (devel-0721): 전방 컷인이 진입/발행될 수 있는 최대 거리 상한 - 0721은
# 이걸 전방 경로에도 적용한다(내가 처음 이식했던 버전은 코너레이더에만
# 썼었다).
VISION_CUTIN_WIDE_MAX_DREL = 45.0
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


# Kans (carrot-wip): 차선 안에 있는 트랙을 좌/우 side-lead 분류에서 제외하는
# 임계값(그건 옆이 아니라 전방에 속한다).
CENTER_LEAD_NEAR_DPATH_LIMIT = 1.2
CENTER_LEAD_FAR_DPATH_LIMIT = 0.9
CENTER_LEAD_FAR_DREL = 60.0
CENTER_LEAD_NEAR_IN_LANE_PROB = 0.3
CENTER_LEAD_FAR_IN_LANE_PROB = 0.45


def pick_side_lead(leads: list[dict[str, Any]]) -> dict[str, Any]:
  return min(
    (ld for ld in leads if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
    key=lambda d: d['dRel'],
    default={'present': False}
  )


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

    # Kans: sticky-selection 상태 (devel)
    self.selected_count = 0
    self.is_stopped_car_count = 0

    # Kans (carrot-wip): dPath/in_lane_prob(md.laneLines에서, match_vision_to_track
    # 타이브레이크용)와 sticky_dPath(md.position, 자차 경로에서, sticky drift
    # 가드용). 둘 다 lane_planner2에 의존하지 않고 - modelV2에서 바로 가져온다.
    self.dPath = 0.0
    self.in_lane_prob = 1.0
    self.lane_half_width = 1.85
    self.sticky_dPath = 0.0
    self.sticky_path_y_std = 0.0

    # Kans (devel): 전방 레이더 컷인 상태. dRel_future/yRel_future는
    # yaw-보정된 위치를 radar_lat_factor초 앞으로 투영한 값이고,
    # dPath_future/in_lane_prob_future는 그 투영값에 d_path()를 적용한 것.
    # dPath_rate/dPath_inward_speed는 이 트랙이 활성 컷인 후보인 동안에만
    # 계산되는 차선-상대-움직임 추정치.
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

    # Kans: vlead_for_matching()의 노이즈 억제 상태 (devel)
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def inherit_cutin_state(self, source: 'Track') -> None:
    # Kans (devel): associate_cutin_tracks()가 이번 프레임의 새 레이더ID
    # 트랙이 실은 지난 프레임 트랙과 같은 물리적 오브젝트라고 판단했을 때
    # (GM은 순차적으로 ID를 재할당해서, 순간적인 ID 변경이 있으면 컷인 확정
    # 진행도가 0으로 리셋돼버릴 것이다) 이전 트랙의 컷인 상태를 새 트랙에
    # 복사해준다.
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

    # Kans: 프레임 간 큰 점프가 있으면 sticky-selection 상태를 리셋해서,
    # 트랙ID 재사용/글리치가 계속 추적되던 lead로 오인되지 않게 한다.
    # Kans (devel): 이 트랙이 활성 컷인 후보인 동안은 더 엄격한 컷인 전용
    # discontinuity 임계값을 대신 쓴다 - 컷인 확정은 횡방향 움직임을
    # 추적하는 거라 정밀도가 더 중요하다.
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

    # Kans (devel): yaw-보정된 미래 위치. 아래의 차선-상대-움직임 컷인
    # 투영에 쓰인다.
    v_rel_future, yv_rel_future = self.yaw_compensated_velocities(yaw_rate)
    self.dRel_future = self.dRel + v_rel_future * radar_lat_factor
    self.yRel_future = self.yRel + yv_rel_future * radar_lat_factor

    # Kans (carrot-wip): 매칭용으로 dPath/in_lane_prob를 갱신하고, sticky
    # 트랙이 자차 경로에서 벗어났으면 sticky 상태를 해제한다.
    if md is not None:
      self.d_path(md)

      # Kans (devel): 활성 컷인 후보인 동안은 차선-상대 움직임 속도를
      # 추정하고 앞으로 투영해서 이 트랙이 우리 차선으로 들어오는 중인지
      # 판단한다.
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

    # Learn if constant acceleration. Kans (devel): RadarReactionFactor가
    # 임계값과 학습되는 시정수 둘 다 스케일한다 - tz는 별도의 aLead/jLead가
    # 없어서(칼만필터를 거친 aLeadK만 있음) devel의 aLead 자리에 aLeadK를
    # 쓰고, devel이 추가로 붙이는 jLead 체크는 뺐다.
    a_lead_threshold = 0.5 * radar_reaction_factor
    if abs(self.aLeadK) < a_lead_threshold:
      self.aLeadTau.x = _LEAD_ACCEL_TAU * radar_reaction_factor
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def d_path(self, md):
    # Kans (carrot-wip): 차선 모델(md.laneLines)에 대한 dPath/in_lane_prob를
    # 직접 구한다, lane_planner2.py와는 무관.
    # Kans (devel): 컷인 투영을 위해 yaw-보정된 미래 위치
    # (dRel_future/yRel_future)에 대해서도 똑같이 계산해둔다.
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
    # Kans (devel): 자차 경로가 휘어 있으면 자차 좌표계에서 겉보기 횡속도가
    # 생긴다(yaw_rate * dRel). 커브에서 옆차선 오브젝트가 우리 차선으로
    # 들어오는 걸로 오분류되지 않도록 컷인 투영 전에 이걸 제거한다. GM은
    # 타겟별 yaw-상대 속도를 절대 보고하지 않아서(opendbc/car/gm/
    # radar_interface.py에서 yvRel은 항상 0) 여기선 devel의 원본 yvLead
    # 필드 대신 0.0을 쓴다.
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
    # Kans (carrot-wip): 자차 자신의 계획 경로(md.position)에 대한 dPath -
    # sticky drift 가드에만 쓰인다.
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
    # Kans (devel): 매칭 점수용으로만 vLead에 spike-clamp + IIR 스무딩을
    # 적용한다(발행되는 vLead/vLeadK는 건드리지 않음). cnt < 2면 원본 vLead.
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
      "dPath": float(self.dPath),
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
  # Kans (devel): 거리/속도/횡방향 "정상범위" 게이트, 정지 상태에서 막
  # 출발한 lead가 원본-vLead 노이즈 때문에 거부되지 않도록 vel_sane에 두는
  # moving-bias 허용치, 이미 선택된 트랙에 대한 단계적 lead_prob 수용
  # 하한선, 그리고 엄격한 속도 게이트는 통과 못 해도 거리/넓은-y는 통과하는
  # 트랙을 승격시키기 전에 ~1초의 일관된 증거를 요구하는 전용 "정지차 같은"
  # 매칭 정책(케이스 B). 케이스 A에도 carrot-wip의 in-lane 타이브레이크가
  # 있다 - dPath/in_lane_prob 관련은 위쪽 STICKY_SELECTED_COUNT_MAX 주석
  # 참고.
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
    # A) normal match. Kans (carrot-wip): 더 가깝고 차선 안에 있는
    # second_track도 그럴듯하면, first_track의 원점수보다 그쪽을 우선한다 -
    # 이렇게 하면 점수가 비슷한 두 트랙(예: 정지한 lead vs 바로 뒤 트랙)
    # 사이에서 프레임마다 왔다갔다 하면서 선택된 lead(그리고 결국 MPC의
    # 장애물 소스)가 요동치는 걸 막는다.
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

    # Kans (devel): RadarReactionFactor - lead 가속도 학습 임계값/시정수를
    # 스케일한다(Track.update() 참고). 기본값 0.2는 devel이 선언한 파라미터
    # 기본값과 일치(20 -> *0.01).
    self.params = Params()
    self._param_frame = 0
    self.radar_reaction_factor = 0.2
    self.enable_radar_tracks = 0

    # Kans (devel): 전방 레이더 컷인 감지. devel에도 있는 코너레이더/SCC
    # 폴백 장치는 빼고 이식했다(볼트엔 둘 다 없음). devel의
    # car_brand=="hyundai" 체크 대신 CarrotRadarMode로 게이트(기본 꺼짐 -
    # 선택적 활성화)했고, devel의 고정 sensitivity=50 대신
    # CarrotRadarCutInSensitivity(0-5, UI)로 실시간 조절 다이얼을 준다.
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

    # Kans: 디버그용 - 컷인처럼 보이는 감속 현상 조사. leadOne으로 선택된
    # *레이더* 트랙의 정체가 바뀔 때만 엣지 트리거로 찍어서, 매 프레임이
    # 아니라 전환될 때 한 번만 출력된다.
    self._debug_prev_lead_id: int | None = None

    # Kans: 디버그용 - 정지 중/단일 lead와 가까울 때 leadTwo가 불안정해지는
    # 현상 의심(leadTwo는 leadOne과 달리 sticky 디바운스가 없어서, 비전의
    # 두번째-lead 매칭이 같은 차의 고스트/멀티패스 반사로 깜빡이며 long_mpc가
    # 붙잡는 대상이 바뀔 수 있다). leadTwo로 선택된 레이더 트랙의 정체가
    # 바뀔 때만 엣지 트리거.
    self._debug_prev_lead2_id: int | None = None

    # Kans: 디버그용 - 전방 레이더 컷인 감지 검증. 트랙이 새로 confirmed
    # 상태에 도달할 때만 엣지 트리거로 찍어서, confirmed로 유지되는 매
    # 프레임이 아니라 컷인 이벤트당 한 번만 출력된다.
    self._debug_prev_cutin_ids: set[int] = set()

  def get_sticky_track(self, tracks: dict[int, Track]) -> Track | None:
    # Kans (devel): 이번 프레임 비전 매칭이 실패해도, 계속 측정되고 있고
    # track_discontinuous() 점프로 리셋되지 않은 이상 이전에 선택됐던
    # 트랙을 계속 lead로 보고한다.
    # Kans (carrot-wip): 마지막 update() 이후 자차 경로에서 벗어난 트랙은
    # 여기서도 sticky 상태를 해제한다(sticky_dPath 참고).
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

  # ---- 전방 레이더 컷인 감지 (Kans, devel - 코너/SCC 부분은 뺌) ----

  def _is_front_cutin_track(self, t: Track) -> bool:
    return is_front_radar_cutin_candidate(t.identifier, t.dRel, t.yRel)

  def _cutin_yaw_rate_from_state(self, sm: messaging.SubMaster) -> float:
    # Kans: devel은 유효할 때 sm['livePose'].angularVelocityDevice.z를
    # 우선하고 modelV2.orientationRate.z[0]로 폴백한다. tz는 여기서
    # livePose를 구독하지 않아서 항상 modelV2 폴백을 쓴다 - devel 자신의
    # 폴백 경로이고, 그냥 선택적 업그레이드가 빠진 것뿐이다.
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
      max_d_rel=VISION_CUTIN_WIDE_MAX_DREL,
    )
    return reason is None

  def _is_cutin_keep_candidate(self, t: Track) -> bool:
    if not self.front_cutin_enabled or not self._is_front_cutin_track(t):
      return False
    if not self._cutin_is_closer_or_matches_lead_one(t):
      return False
    # Kans (devel-0721): 여기선 25.0 - 예전 접미사 없는 스냅샷의 55.0이
    # 아니다 - 0721은 확정된 전방 컷인을 25m까지만 유지한다.
    if not (0.8 < t.dRel < 25.0 and t.vLead > 2.0):
      return False
    moving_away = abs(t.dPath_future) - abs(t.dPath)
    if moving_away > CUTIN_KEEP_MAX_MOVING_AWAY:
      return False
    return t.in_lane_prob_future > CUTIN_KEEP_FUTURE_IN_LANE_PROB or abs(t.dPath_future) < CUTIN_KEEP_MAX_DPATH_FUTURE

  def _update_cutin_sticky(self, t: Track) -> bool:
    # Kans (devel-0721): 예전 접미사 없는 스냅샷은 전방 경로에서도 `keeping`
    # 혼자서 `entering`을 재무장할 수 있게 해서, 느슨한 keep-게이트만
    # 유지되면 확정된 트랙이 계속 확정 상태로 남았다. 0721은 그 지름길을
    # 사이드-코너 트랙(볼트엔 없음)에만 남겨뒀다 - 전방 트랙은 더 엄격한
    # enter-게이트를 계속 다시 통과해야 한다.
    entering = self._is_cutin_enter_candidate(t)
    keeping = t.cut_in_count > 0 and self._is_cutin_keep_candidate(t)
    confirm_frames = cutin_confirmation_frames(self.front_cutin_confirm_frames, t.dRel, t.dPath_inward_speed, self.v_ego)
    t.cut_in_count, t.cut_in_start_abs_dpath = update_cutin_confirmation(
      t.cut_in_count, t.cut_in_start_abs_dpath, t.dPath, t.dRel, entering, keeping,
      confirm_frames, CUTIN_STICKY_FRAMES, self.cutin_tuning["enter_min_progress"], self.v_ego,
    )
    return t.cut_in_count >= confirm_frames

  def _apply_cutin_output_hold(self, cutin_list: list[dict[str, Any]]) -> list[dict[str, Any]]:
    # Kans (devel): 컷인 lead가 사라지면(예: 순간적으로 놓침), 마지막으로
    # 알려진 위치 근처에서 여전히 활성 상태인 컷인 트랙을 재매칭해서
    # CUTIN_OUTPUT_HOLD_FRAMES 프레임만큼 더 발행을 유지한다 - 순간적인
    # 감지 공백 때문에 leadTwo가 왔다갔다 하지 않도록.
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
    # Kans: tz에는 compute_leads() 같은 오케스트레이터가 없어서(이 포크엔
    # 없는 훨씬 큰 devel 구조라서) 이건 새로 만든, 더 좁은 진입점이다: 매
    # 프레임 모든 트랙의 컷인 확정 상태를 한 번씩 진행시키고(상태를 갖는
    # 카운터라서 트랙이 결국 확정되든 안 되든 매 프레임 돌아가야 한다)
    # 현재 확정된 것들만 모은다.
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
        cloudlog.warning(f"[radard cutin] confirmed trackId={t.identifier} dRel={t.dRel:.1f} yRel={t.yRel:.1f} "
              f"vLead={t.vLead:.1f} dPath={t.dPath:.2f} inwardSpeed={t.dPath_inward_speed:.2f} "
              f"vEgo={self.v_ego:.1f} sensitivity={self.cutin_sensitivity:.0f}")
    self._debug_prev_cutin_ids = confirmed_ids

    return self._apply_cutin_output_hold(cutin_list)

  def _is_center_lead_candidate(self, t: Track) -> bool:
    # Kans (carrot-wip): 이 정도로 차선 안에 있는 트랙은 옆이 아니라 전방에 속한다.
    in_lane_min = CENTER_LEAD_NEAR_IN_LANE_PROB
    dpath_limit = CENTER_LEAD_NEAR_DPATH_LIMIT
    if t.dRel > CENTER_LEAD_FAR_DREL:
      in_lane_min = CENTER_LEAD_FAR_IN_LANE_PROB
      dpath_limit = CENTER_LEAD_FAR_DPATH_LIMIT
    return t.in_lane_prob > in_lane_min and abs(t.dPath) < dpath_limit

  def compute_side_leads(self) -> None:
    # Kans (carrot-wip): 전방 레이더 자체 트랙(yRel 부호)으로
    # leadLeft/leadRight/leadsLeft/leadsRight를 채운다. BSD와도, carrot-wip
    # 자체 버전이 있을 때 우선하는 코너레이더 하드웨어와도 무관 - GM엔 둘 다
    # 없어서 여기선 이게 유일하게 쓸 수 있는 side-lead 소스다.
    left_list: list[dict[str, Any]] = []
    right_list: list[dict[str, Any]] = []
    for t in self.tracks.values():
      if self._is_center_lead_candidate(t):
        continue
      ld = t.get_RadarState(0.0)
      if t.yRel > 0:
        left_list.append(ld)
      else:
        right_list.append(ld)

    self.radar_state.leadsLeft = left_list
    self.radar_state.leadsRight = right_list
    self.radar_state.leadLeft = pick_side_lead(left_list)
    self.radar_state.leadRight = pick_side_lead(right_list)

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

    # Kans (devel): 실제 레이더 CAN 오류가 났거나 EnableRadarTracks가
    # 완전히 낮춰져 있으면, 이번 프레임 레이더 출력은 아무것도 신뢰하지
    # 않는다는 뜻 - 트랙을 전부 버리고 get_lead()가 비전 전용 경로로
    # 넘어가게 둔다.
    radar_faulted = bool(rr.errors.canError or rr.errors.radarFault)
    vision_only_mode = self.enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE or radar_faulted

    if vision_only_mode:
      self.tracks.clear()
    else:
      ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

      # Kans (devel): 이번 프레임의 정리/생성 전에, 현재 추적 중인 오브젝트
      # 중 활성 컷인 후보였던 것들을 스냅샷으로 남겨두고, 위치를 기준으로
      # 이번 프레임의 후보 포인트들과 연결해서 레이더ID 재할당이 컷인
      # 확정 진행도를 리셋시키지 않게 한다.
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
    self.compute_side_leads()

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

      # Kans (devel): 확정된 컷인 후보들은 leadsCutIn을 통해 전부 발행되고,
      # 그중 자격 있고 가장 가까운 것(아직 leadOne이 아닌)이 위에서 순수
      # 비전 매칭된 leadTwo보다 우선순위를 갖는다 - 이렇게 감지된 컷인이
      # 실제로 롱컨(longitudinal control)까지 전달된다.
      cutin_list = self.compute_cutin_list()
      self.radar_state.leadsCutIn = cutin_list
      if self.front_cutin_enabled and cutin_list:
        lead_one = self.radar_state.leadOne
        # Kans (devel-0721): entry 게이트와 같은 VISION_CUTIN_WIDE_MAX_DREL
        # 상한을 적용해서, 애초에 진입할 수 없었던 트랙이 더 가까웠을 때
        # 확정됐다는 이유만으로 leadTwo로 발행되지 않게 한다.
        max_cutin_d_rel = min(self.cutin_enter_max_x, VISION_CUTIN_WIDE_MAX_DREL)
        eligible = [
          c for c in cutin_list
          if self.cutin_enter_min_x < c['dRel'] < max_cutin_d_rel and c['vLead'] > 4.0
          and not (lead_one.present and lead_one.radar and int(lead_one.radarTrackId) == int(c['radarTrackId']))
        ]
        if eligible:
          self.radar_state.leadTwo = min(eligible, key=lambda c: c['dRel'])

      lead_one = self.radar_state.leadOne
      new_lead_id = lead_one.radarTrackId if (lead_one.present and lead_one.radar) else None
      if new_lead_id is not None and new_lead_id != self._debug_prev_lead_id:
        t = self.tracks.get(new_lead_id)
        cloudlog.warning(f"[radard lead-switch] prevId={self._debug_prev_lead_id} -> newId={new_lead_id} "
              f"dRel={lead_one.dRel:.1f} yRel={lead_one.yRel:.1f} vLead={lead_one.vLead:.1f} "
              f"vRel={lead_one.vRel:.1f} vEgo={self.v_ego:.1f} "
              f"selectedCount={t.selected_count if t else -1} "
              f"isStoppedCarCount={t.is_stopped_car_count if t else -1}")
      self._debug_prev_lead_id = new_lead_id

      lead_two = self.radar_state.leadTwo
      new_lead2_id = lead_two.radarTrackId if (lead_two.present and lead_two.radar) else None
      if new_lead2_id is not None and new_lead2_id != self._debug_prev_lead2_id:
        cloudlog.warning(f"[radard lead2-switch] prevId={self._debug_prev_lead2_id} -> newId={new_lead2_id} "
              f"dRel={lead_two.dRel:.1f} yRel={lead_two.yRel:.1f} vLead={lead_two.vLead:.1f} "
              f"vEgo={self.v_ego:.1f} leadOneId={new_lead_id} "
              f"leadOneDRel={lead_one.dRel:.1f}")
      self._debug_prev_lead2_id = new_lead2_id

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
