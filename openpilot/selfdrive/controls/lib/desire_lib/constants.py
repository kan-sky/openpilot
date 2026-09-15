from openpilot.cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.0

# Kans: xDistToTurn이 이 거리 이내면, auto_lane_change_trigger 입장에서는
# 라이브 edge_available 값과 무관하게 "탈 곳이 있다"로 간주한다. 야간 비전
# 노이즈가 하필 last-lane이 무장되는 바로 그 프레임에 edge_available을
# 떨어뜨리면 그 fork의 자동 트리거가 영구히 막혀버리는 문제 때문. ATC가
# fork/진출을 준비하기 시작하는 지점이 대략 115m라서 그보다 넉넉하게 잡았다
# - 부족하면 315/415 등으로 더 키워볼 수 있다. `_is_last_lane`의 무장 조건
# 자체는 이 값과 무관하게 그대로 엄격하다; 실제 실행도 여전히 그 프레임의
# lane_available_trigger(옆차선이 지금 실시간으로 벌어지고 있다는 신호)가
# 있어야만 일어난다.
EDGE_AVAILABLE_MEMORY_DIST = 215.0

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}
