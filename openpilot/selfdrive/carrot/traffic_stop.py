from __future__ import annotations

from collections import deque
from statistics import median

import numpy as np


TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0

# Kans (carrot-wip-0913): 정지 직전 모델 lead를 "진짜 서 있는 차"로 확정하기 위한
# 상수들. MODEL_LEAD_STOP_OFFSET_M은 모델이 보통 대기 중인 차 뒤 약 2m 지점에
# e2e 정지 지점을 찍는다는 경험칙에서 나온 값 - 확정되면 trafficStopDistanceAdjust
# 대신 이 오프셋을 신호정지 장애물 보정값으로 쓴다.
MODEL_LEAD_STOP_OFFSET_M = 2.0
MODEL_LEAD_STOP_CONFIRM_FRAMES = 5
MODEL_LEAD_STOP_PROBABILITY_MIN = 0.90
MODEL_LEAD_STOP_DISTANCE_MIN_M = 4.0
MODEL_LEAD_STOP_DISTANCE_MAX_M = 80.0
MODEL_LEAD_STOP_GAP_MIN_M = 0.0
MODEL_LEAD_STOP_GAP_MAX_M = 3.0
MODEL_LEAD_STOP_SPEED_MAX_MPS = 2.0
MODEL_LEAD_STOP_X_STD_MAX_M = 5.0
MODEL_LEAD_STOP_Y_STD_MAX_M = 0.75
MODEL_LEAD_STOP_V_STD_MAX_MPS = 1.5


class TrafficStopModelLeadMatcher:
  """Kans (carrot-wip-0913): e2e 정지 지점이 정지해 있는 모델 lead에 실제로 붙어
  있는지 확인한다.

  주행 모델은 대기 중인 차량 뒤 약 2m 지점에 자차의 정지 끝점을 찍는 게
  보통이다. 그 관계가 여러 프레임 동안 안정적으로 유지되면, 그 차량의 대략적인
  위치를 MPC 장애물로 노출해서(평소 설정된 추종거리는 그대로 유지) 모델
  자신의 정지선 추정치(stop_model_x_rl)가 그날그날 크게 틀어져도 실제 차
  위치 기준으로 보정되게 한다. 어디까지나 e2e 장애물 보정일 뿐이고, 레이더
  lead를 발행하거나 승격시키는 일은 절대 없다.
  """

  def __init__(self, confirm_frames: int = MODEL_LEAD_STOP_CONFIRM_FRAMES):
    self._confirm_frames = max(1, int(confirm_frames))
    self._lead_distances = deque(maxlen=self._confirm_frames)
    self._lead_velocities = deque(maxlen=self._confirm_frames)
    self._match_count = 0
    self._confirmed = False

  def _clear_pending(self) -> None:
    self._lead_distances.clear()
    self._lead_velocities.clear()
    self._match_count = 0

  def reset(self) -> None:
    self._clear_pending()
    self._confirmed = False

  def update(self, *, stop_active: bool, allow_confirmation: bool, active_lead: bool,
             stop_distance: float, lead_probability: float, lead_distance: float,
             lead_velocity: float, lead_x_std: float, lead_y_std: float,
             lead_v_std: float) -> float:
    if not stop_active or active_lead:
      self.reset()
      return 0.0

    if self._confirmed:
      return MODEL_LEAD_STOP_OFFSET_M

    values = (
      stop_distance, lead_probability, lead_distance, lead_velocity,
      lead_x_std, lead_y_std, lead_v_std,
    )
    if not allow_confirmation or not all(np.isfinite(value) for value in values):
      self._clear_pending()
      return 0.0

    self._lead_distances.append(float(lead_distance))
    self._lead_velocities.append(float(lead_velocity))
    filtered_distance = float(median(self._lead_distances))
    filtered_velocity = float(median(self._lead_velocities))
    endpoint_gap = filtered_distance - float(stop_distance)

    valid = (
      float(lead_probability) >= MODEL_LEAD_STOP_PROBABILITY_MIN
      and MODEL_LEAD_STOP_DISTANCE_MIN_M <= filtered_distance <= MODEL_LEAD_STOP_DISTANCE_MAX_M
      and MODEL_LEAD_STOP_GAP_MIN_M <= endpoint_gap <= MODEL_LEAD_STOP_GAP_MAX_M
      and abs(filtered_velocity) <= MODEL_LEAD_STOP_SPEED_MAX_MPS
      and 0.0 <= float(lead_x_std) <= MODEL_LEAD_STOP_X_STD_MAX_M
      and 0.0 <= float(lead_y_std) <= MODEL_LEAD_STOP_Y_STD_MAX_M
      and 0.0 <= float(lead_v_std) <= MODEL_LEAD_STOP_V_STD_MAX_MPS
    )
    self._match_count = self._match_count + 1 if valid else 0
    if not valid:
      self._lead_distances.clear()
      self._lead_velocities.clear()
    elif self._match_count >= self._confirm_frames:
      self._confirmed = True

    return MODEL_LEAD_STOP_OFFSET_M if self._confirmed else 0.0


def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  # 신호정지 새로 진입하는 것만 조향각으로 막는다(이미 진입한 정지는 유지).
  return abs(float(steering_angle_deg)) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_distance_adjust(configured_adjust: float, v_ego: float,
                                     model_lead_offset: float) -> float:
  # Kans (carrot-wip-0913): 신호(정지선)용 보정값과 대기차량용 보정값이
  # 섞이지 않도록 분리한다. TrafficStopModelLeadMatcher가 실제 대기 차량을
  # 확정했으면(model_lead_offset>0) 그 값을 최우선으로 쓰고, 아니면 기존처럼
  # 사용자 설정값(정지 접근 중) 또는 -2.0(완전 정지 후, 살짝 당겨서 유지)을 쓴다.
  model_lead_offset = float(model_lead_offset)
  if np.isfinite(model_lead_offset) and model_lead_offset > 0.0:
    return model_lead_offset
  return float(configured_adjust) if float(v_ego) > 0.1 else -2.0


def get_traffic_stop_obstacle_distance(stop_distance: float, cruise_obstacle_distance: float,
                                       distance_adjust: float, release_distance: float = 50.0) -> float:
  # 신호정지 앞에서 예전부터 쓰던 cruise-거리 마스크를 부드럽게 풀어준다.
  signal_obstacle = max(0.0, float(stop_distance) + float(distance_adjust))
  cruise_obstacle = max(0.0, float(cruise_obstacle_distance))
  release_distance = max(0.0, float(release_distance))

  # 과거엔 50m~cruise 안전거리 사이의 신호 장애물을 cruise 장애물로 통째로
  # 대체했다가 50m 지점에서 한번에 노출시켰다. 그 첫 접촉 시점의 보호는
  # 유지하되, 50m 경계 이전부터 제동이 미리 걸리도록 실제 신호 장애물을
  # 점진적으로 드러낸다.
  if release_distance < signal_obstacle < cruise_obstacle:
    release = float(np.interp(signal_obstacle, [release_distance, cruise_obstacle], [1.0, 0.0]))
    return cruise_obstacle + release * (signal_obstacle - cruise_obstacle)
  return signal_obstacle
