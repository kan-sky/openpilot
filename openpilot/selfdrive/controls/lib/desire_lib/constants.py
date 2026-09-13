from openpilot.cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.0

# Kans: xDistToTurn 기준으로 얼마나 먼 거리까지, 순간적인 edge_available 소실을
# auto_lane_change_trigger 입장에서 "있었다"고 계속 기억해줄지. 야간 비전
# 노이즈가 하필 last-lane이 무장되는 바로 그 프레임에 edge_available을 떨어뜨리면
# 그 fork의 자동 트리거가 영구히 막혀버린다. ATC가 fork/진출을 준비하기
# 시작하는 지점이 대략 115m라서, 접근 구간 전체를 커버하면서도 내비가 이미
# 1~2km 전부터 알려주는 다음(무관한) 이벤트까지 새지 않도록 +100m 여유를 뒀다.
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
