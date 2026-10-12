from opendbc.car import DT_CTRL
from opendbc.car.gm.values import GMFlags

# Kans: GM 오토홀드 모듈
# - update_auto_hold(): interface.apply()에서 carcontroller 전에 호출 (운전자 브레이크 정지 → 홀드 진입/해제)
# - GMAutoHold.update(): carcontroller 25Hz 블록에서 호출, 홀드 브레이크 명령 여부 반환


def update_auto_hold(CS, CP):
  CS.autoHoldActivated = False
  CS.out.autoHoldActivated = False

  if not CS.autoHold:
    CS.autoHoldActive = False
    return

  # 가스페달, 리제패들 해제
  if CS.out.gasPressed or CS.out.regenBraking:
    CS.autoHoldActive = False
    return

  # Kans: 이 포크의 AutoResume는 가속페달 없이 RES_ACCEL 버튼만으로 재출발시키므로,
  # gasPressed가 한 번도 안 걸려서 위 조건으로는 절대 안 풀림 - 차가 실제로
  # vEgoStarting을 넘어 움직이기 시작하면 페달 여부와 무관하게 해제.
  # (이게 안 풀리고 sticky로 남아있으면 auto_resume.py의 manual_auto_hold가
  # 다음 정지에서 크루즈가 잠깐이라도 꺼지는 순간 엉뚱하게 True가 되어
  # resume_frame/resume_fault_guard를 리셋시켜버릴 수 있음)
  if CS.out.vEgo > CP.vEgoStarting:
    CS.autoHoldActive = False
    return

  # 이미 AutoHold 중이면 유지
  if CS.autoHoldActive:
    CS.autoHoldActivated = True
    CS.out.autoHoldActivated = True
    return

  # 브레이크 압력 기준
  if CP.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value:
    brake_hold_pressed = CS.out.brakePressed and CS.out.brake >= 0.04
  else:
    brake_hold_pressed = CS.out.brakePressed and CS.out.brake >= 8

  # 운전자 브레이크로 충분히 정지했을 때만 AutoHold 진입
  if CS.out.vEgo < 0.05 and brake_hold_pressed:
    CS.autoHoldActive = True
    CS.autoHoldActivated = True
    CS.out.autoHoldActivated = True
    return


class GMAutoHold:
  def __init__(self):
    # AutoHold post-cruise guard timestamp
    self.autoHold_cruise_cancel_frame = -1

  def update(self, CC, CS, frame):
    """반환: auto_hold_cmd (True면 carcontroller가 홀드 브레이크 명령 송신) - 196과 동일"""
    # Kans: 크루즈 해제 직후 AutoHold 브레이크 명령 지연(1.5s)
    if CC.enabled or CS.out.cruiseState.enabled:
      self.autoHold_cruise_cancel_frame = frame

    auto_hold_cruise_cancel_delay = (
      self.autoHold_cruise_cancel_frame >= 0 and
      (frame - self.autoHold_cruise_cancel_frame) * DT_CTRL < 1.5
    )
    # Kans: AutoHold 조건
    auto_hold_cmd = (
      not CC.longActive and
      not CC.enabled and
      not CS.out.cruiseState.enabled and
      not auto_hold_cruise_cancel_delay and
      CS.autoHold and
      CS.autoHoldActive and
      not CS.out.gasPressed and
      CS.out.gearShifter in ['drive', 'low'] and
      CS.out.vEgo < 0.05 and
      not CS.out.regenBraking
    )
    return auto_hold_cmd


def is_auto_hold(CC, CS):
  # 오토홀드: 운전자 브레이크로 정지해 크루즈가 꺼진 뒤, 브레이크에서 발을 떼도 차가 정지 유지되는 상태
  return (
    CS.autoHold and
    (CS.autoHoldActive or CS.autoHoldActivated or CS.out.autoHoldActivated) and
    not CS.out.cruiseState.enabled and
    not CC.enabled
  )
