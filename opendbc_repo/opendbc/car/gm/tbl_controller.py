"""쉐보레 트레일블레이져(2021-22) 카메라 롱컨(longitudinal) 전용 헬퍼.

트레일블레이져는 순정 카메라가 자기 자신의 ACC 상태(0x2CB/0x370)를
openpilot의 대체 명령과 함께 계속 보내고, 0x2CB의 체크섬이 다른 모든 GM
차량과 다르며, 5km/h 이하에서 인게이지하면 폴트가 납니다. 이 모듈이 그런
차종 전용 로직을 전부 격리해서, gmcan.py는 모든 GM 차량이 공유하는 범용
CAN 메시지 빌더로 남고, carstate.py/carcontroller.py는 작고 테스트하기
쉬운 인터페이스만 호출하면 됩니다.
"""

import copy

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.gm.values import CAR

NetworkLocation = structs.CarParams.NetworkLocation
ButtonType = structs.CarState.ButtonEvent.Type


def is_trailblazer_camera_longitudinal(CP):
  return (CP.openpilotLongitudinalControl and
          CP.carFingerprint == CAR.CHEVROLET_TRAILBLAZER and
          CP.networkLocation == NetworkLocation.fwdCamera)


def get_longitudinal_sync_messages(CP):
  if is_trailblazer_camera_longitudinal(CP):
    # 이 메시지들은 동기화 기준용이지, 차량 전체의 CAN 유효성 조건이 아님.
    # 카메라가 늦게 뜬다고 차량 전체를 무효화하면 안 됨.
    return [("ASCMGasRegenCmd", float('nan')), ("ASCMActiveCruiseControlStatus", float('nan'))]
  return []


def update_camera_longitudinal_state(CS, CP, cam_cp):
  """트레일블레이져 순정 0x2CB/0x370 카메라 롱컨 상태를 추적. CS를 직접 수정함."""
  CS.cam_ascm_2cb_counter_updated = False
  CS.cam_stock_long_cancel = False
  if not is_trailblazer_camera_longitudinal(CP):
    return

  # 순정 0x2CB 카운터는 콜드스타트 중 일부 구간에서 ASCM_2CD보다 한 사이클
  # 늦게 따라오다가, ACC가 활성화된 후에는 맞춰짐. 그래서 ASCM_2CD 대신
  # 0x2CB 자체를 따라감.
  counters = cam_cp.vl_all["ASCMGasRegenCmd"]["RollingCounter"]
  active_states = cam_cp.vl_all["ASCMGasRegenCmd"]["GasRegenCmdActive"]
  CS.cam_ascm_2cb_counter_updated = len(counters) > 0 and len(active_states) > 0
  if CS.cam_ascm_2cb_counter_updated:
    previous_stock_long_active = CS.cam_stock_long_active
    CS.cam_ascm_2cb_counter = int(counters[-1])
    CS.cam_ascm_2cb_counter_ts_nanos = cam_cp.ts_nanos["ASCMGasRegenCmd"]["RollingCounter"]
    CS.cam_stock_long_active = bool(active_states[-1])
    CS.cam_stock_long_cancel = previous_stock_long_active is True and not CS.cam_stock_long_active

    # 엣지 트리거(매 프레임 아님) - 카메라가 자기 ACC 권한을 뺏거나
    # 되찾는 게 이 포팅 전체가 걸려있는 핵심 이벤트라서 로그로 남김.
    if CS.cam_stock_long_active != previous_stock_long_active:
      carlog.warning(f"[tbl camera-long] stock_long_active {previous_stock_long_active}->{CS.cam_stock_long_active} "
                      f"cancelSynthesized={CS.cam_stock_long_cancel} counter={CS.cam_ascm_2cb_counter} "
                      f"vEgo={CS.out.vEgo:.2f}")

  # 카메라의 완전한 ACC 상태를 대체용 0x370의 템플릿으로 유지함. 이
  # 트레일블레이져 세대는 ACCCruiseState와 두 고정 비트가 순정값(2, 0, 0)을
  # 그대로 유지하길 기대함.
  if len(cam_cp.vl_all["ASCMActiveCruiseControlStatus"]["ACCCruiseState"]) > 0:
    CS.cam_acc_status = copy.copy(cam_cp.vl["ASCMActiveCruiseControlStatus"])


def create_stock_long_cancel_button_events() -> list:
  """순정 ACC 권한 상실에 대응하는 원자적(atomic) 합성 캔슬 클릭을 생성."""
  # VCruiseCarrot은 버튼 지속시간을 press/release 엣지 매칭으로 추적함.
  # press만 있는 합성 캔슬은 트래커를 영구히 붙잡아서 결국 롱프레스로
  # 해석되고, 이후 SET/RES 버튼 처리를 막아버림. 그래서 두 엣지를 한
  # carState 메시지에 같이 담아서 따로 유실되지 않게 함.
  return [
    structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel),
    structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel),
  ]


def get_longitudinal_command_timing(CP, CS, frame):
  if is_trailblazer_camera_longitudinal(CP):
    # 순정 명령 카운터가 콜드스타트 중엔 ASCM_2CD보다 한 사이클 늦다가
    # ACC 활성화 후엔 맞춰짐. 추측하지 않고 실제 순정 0x2CB 명령을
    # 따라가면 두 구간 다 처리됨.
    stock_references_ready = (
      CS.cam_ascm_2cb_counter_ts_nanos != 0 and
      CS.cam_stock_long_active is not None and
      CS.cam_acc_status is not None
    )
    if not stock_references_ready:
      return False, 0
    return CS.cam_ascm_2cb_counter_updated, CS.cam_ascm_2cb_counter
  return frame % 4 == 0, (frame // 4) % 4


def apply_driver_gas_override(car_fingerprint, gas_pressed, inactive_regen, apply_gas, apply_brake,
                              at_full_stop, near_stop):
  # 2021-22 트레일블레이져는 controls가 longActive를 해제하기 전에 가속페달을
  # 감지할 수 있음. 그 전환 구간에서 완전한 비활성 명령 세트를 보내서,
  # Panda가 카운터가 안 맞는 0x2CB/0x315 쌍을 드롭하지 않게 함.
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER and gas_pressed:
    return inactive_regen, 0, False, False
  return apply_gas, apply_brake, at_full_stop, near_stop


def apply_stock_longitudinal_gate(car_fingerprint, stock_long_active, inactive_regen, apply_gas, apply_brake,
                                  at_full_stop, near_stop, acc_engaged):
  # 트레일블레이져의 카메라는 ECM 크루즈 상태가 바뀌기 전에 롱컨 권한을
  # 뺏을 수 있음. 같은 순정 명령 사이클에서 액추에이션을 멈춰서, EBCM이
  # 카메라가 idle이 된 이후에 활성 상태인 대체 명령을 절대 못 보게 함.
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER and stock_long_active is not True:
    return inactive_regen, 0, False, False, False
  return apply_gas, apply_brake, at_full_stop, near_stop, acc_engaged


def get_acc_dashboard_enabled(car_fingerprint, enabled, in_drive, stock_long_active, stock_acc_status):
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER:
    return (enabled and in_drive and stock_long_active is True and stock_acc_status is not None and
            bool(stock_acc_status["ACCCmdActive"]))
  return enabled


def compute_gas_regen_checksum(car_fingerprint, dat, idx):
  """0x2CB (ASCMGasRegenCmd) 체크섬. GasRegenCmdActive 최상위 비트는 제외."""
  if car_fingerprint == CAR.CHEVROLET_TRAILBLAZER:
    # 실차에서 캡처한 2021-22 트레일블레이져 프레임은 바이트 간 캐리/보로우가
    # 있는 24비트 뺄셈 한 번을 씀. 다른 GM 차종은 순정 프레임이 같은 방식을
    # 요구한다는 게 확인되기 전까지는 기존 바이트 단위 체크섬을 유지함.
    return (0x1000000 - int.from_bytes(dat[1:4], "big") - idx) & 0xFFFFFF
  return (((0xff - dat[1]) & 0xff) << 16) | \
         (((0xff - dat[2]) & 0xff) << 8) | \
         ((0x100 - dat[3] - idx) & 0xff)
