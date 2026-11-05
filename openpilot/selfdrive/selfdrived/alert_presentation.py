"""Mici wording and typography, independent of event selection and alert policy.

Presentation data comes from Wayon b7660d1d403938c464d4f14f842449e3f0e64ebd.
Only existing Carrot alerts are decorated; the data cannot create events.
"""
import json
import math
from functools import lru_cache
from pathlib import Path
from string import Formatter


OVERRIDES_PATH = Path(__file__).resolve().parents[1] / "ui/mici/mici_event_alert_overrides.json"
EVENT_TYPE_JSON_NAME = {
  "enable": "ENABLE", "preEnable": "PRE_ENABLE",
  "overrideLateral": "OVERRIDE_LATERAL", "overrideLongitudinal": "OVERRIDE_LONGITUDINAL",
  "noEntry": "NO_ENTRY", "warning": "WARNING", "userDisable": "USER_DISABLE",
  "softDisable": "SOFT_DISABLE", "immediateDisable": "IMMEDIATE_DISABLE", "permanent": "PERMANENT",
}

# The verified reference and current Carrot schema both use numbered DM events.
# Accept legacy warning/preDriver*/promptDriver* editor keys as data, never as
# enums or event types in EVENTS. Exact current-schema entries take precedence.
LEGACY_DM_KEYS = {
  ("driverDistracted1", "permanent"): ("preDriverDistracted", "WARNING"),
  ("driverDistracted2", "permanent"): ("promptDriverDistracted", "WARNING"),
  ("driverDistracted3", "permanent"): ("driverDistracted", "WARNING"),
  ("driverUnresponsive1", "permanent"): ("preDriverUnresponsive", "WARNING"),
  ("driverUnresponsive2", "permanent"): ("promptDriverUnresponsive", "WARNING"),
  ("driverUnresponsive3", "permanent"): ("driverUnresponsive", "WARNING"),
}


@lru_cache(maxsize=1)
def load_overrides() -> dict[str, dict]:
  # A broken optional presentation asset must not prevent selfdrived startup.
  try:
    document = json.loads(OVERRIDES_PATH.read_text(encoding="utf-8"))
    configs = document.get("events", {})
    if document.get("schema") != 1 or not isinstance(configs, dict):
      return {}
    return {key: value for key, value in configs.items() if isinstance(value, dict)}
  except (OSError, ValueError, AttributeError):
    return {}


def get_override(event_name: str, event_type: str) -> dict:
  configs = load_overrides()
  kind = EVENT_TYPE_JSON_NAME.get(event_type)
  if kind is None:
    return {}
  key = f"EVENTS.{event_name}.{kind}"
  config = configs.get(key)
  if config is None and (event_name, event_type) in LEGACY_DM_KEYS:
    name, kind = LEGACY_DM_KEYS[event_name, event_type]
    config = configs.get(f"EVENTS.{name}.{kind}")
  return config if config and config.get("source") == "EVENTS" else {}


def translate_text(text: str) -> str:
  if text.startswith("Driving Personality: "):
    return "주행 성향: " + text.removeprefix("Driving Personality: ")
  return MICI_ALERT_TEXT_TRANSLATIONS.get(text, text)


def format_text(template, fallback: str, values: dict) -> str:
  if not isinstance(template, str):
    return fallback
  try:
    return template.format_map(values)
  except (KeyError, ValueError, IndexError, AttributeError, TypeError):
    return fallback


@lru_cache(maxsize=256)
def format_fields(event_name: str, event_type: str) -> frozenset[str]:
  config = get_override(event_name, event_type)
  try:
    return frozenset(field for key in ("title", "subtitle") if isinstance(config.get(key), str)
                     for _, field, _, _ in Formatter().parse(config[key]) if field)
  except ValueError:
    return frozenset()


def format_values(callback_args, *, display_speed, min_speed_filter, feedback_duration, sample_buffer, sample_rate,
                  fields: frozenset[str]) -> dict:
  if not fields or len(callback_args) < 4:
    return {}
  CP, _CS, sm, metric = callback_args[:4]
  # Resolve each field independently: a missing diagnostic must not suppress
  # other fields, invent a value, or leave a raw template on the display.
  readers = {
    "minSteerSpeed": lambda: display_speed(CP.minSteerSpeed, metric),
    "minEnableSpeed": lambda: display_speed(CP.minEnableSpeed, metric),
    "MIN_SPEED_FILTER": lambda: display_speed(min_speed_filter, metric),
    "seconds": lambda: str(round(feedback_duration - (sm["audioFeedback"].blockNum + 1) * sample_buffer / sample_rate)),
    "calPerc": lambda: f"{sm['liveCalibration'].calPerc:.0f}",
    "yaw": lambda: f"{math.degrees(sm['liveCalibration'].rpyCalib[2]):.1f}",
    "pitch": lambda: f"{math.degrees(sm['liveCalibration'].rpyCalib[1]):.1f}",
    "bad camera list": lambda: ", ".join(s.replace("State", "") for s in
      ("roadCameraState", "driverCameraState", "wideRoadCameraState") if s in sm.data and not sm.all_checks([s])),
    "service names": lambda: ", ".join([s for s in sm.data if not sm.all_checks([s])][:3]),
    "accel": lambda: str(round(sm["carControl"].actuators.accel / 4. * 100.)),
    "steer": lambda: str(round(sm["carControl"].actuators.torque * 100.)),
    "memoryUsagePercent": lambda: str(sm["deviceState"].memoryUsagePercent),
    "temperature": lambda: "{:.0f}".format(max(max(sm['deviceState'].cpuTempC, default=0.),
                                             max(sm['deviceState'].gpuTempC, default=0.), sm['deviceState'].memoryTempC)),
    "frameDropPerc": lambda: f"{sm['modelV2'].frameDropPerc:.1f}",
    "offset": lambda: f"{sm['liveParameters'].angleOffsetDeg:.1f}",
  }
  values = {}
  for key, read in readers.items():
    if key not in fields:
      continue
    try:
      values[key] = read()
    except (KeyError, IndexError, AttributeError, TypeError, ValueError, OverflowError):
      pass
  return values


def apply_text(alert, event_name: str, event_type: str, values: dict, *, critical=False, angle_offset_invalid=True):
  """Mutate only the two text fields of an already copied, selected alert."""
  title, subtitle = translate_text(alert.alert_text_1), translate_text(alert.alert_text_2)
  config = get_override(event_name, event_type)
  # This callback also diagnoses steer ratio and tire stiffness. The reference's
  # unconditional angle-offset wording must not misidentify those other faults.
  if event_name == "paramsdTemporaryError" and event_type == "noEntry" and not angle_offset_invalid:
    config = {}
  if event_name == "canError" and event_type == "permanent":
    if "CameraSCC" in title:
      config = {}  # Carrot's actionable, bus-specific diagnosis
    elif subtitle:
      config = {k: v for k, v in config.items() if k != "subtitle"}
  title_override = config.get("title")
  if critical and event_type == "softDisable":
    title_override = None  # retain immediate-takeover wording on escalation
  alert.alert_text_1 = format_text(title_override, title, values)
  alert.alert_text_2 = format_text(config.get("subtitle"), subtitle, values)
  return alert


def font_sizes(event_name: str, event_type: str, title: str, subtitle: str, has_icon: bool) -> tuple[int, int]:
  config = get_override(event_name, event_type)

  def font_px(key):
    value = config.get(key)
    return value if type(value) is int and 0 < value <= 200 else None

  title_size = font_px("titleFontPx")
  if title_size is None:
    title_size = (82 if len(title) <= 12 else 70 if len(title) <= 16 else 54) - (10 if has_icon else 0)
  # The reference's >24 branch is unreachable after >18; retain its actual
  # 36px behavior for long subtitles, then fit to the real viewport in the UI.
  subtitle_size = font_px("subtitleFontPx") or (36 if len(subtitle) > 18 else 40)
  return title_size, subtitle_size


MICI_ALERT_TEXT_TRANSLATIONS = {
  # Text-only counterparts of the reference's Mici EVENTS.update block.
  "Pay Attention": "운전에 집중하세요",
  "Press Resume": "Resume 버튼을 누르세요",
  "Steer Left": "왼쪽 조향",
  "Steer Right": "오른쪽 조향",
  "Confirm Lane Change": "차선 변경 확인",
  "Car in Blindspot": "사각지대 차량",
  "take control": "직접 운전하세요",
  "turn exceeds limit": "조향 한계 초과",
  "Calibration Incomplete": "캘리브레이션이 완료되지 않았습니다",
  "Calibrating": "캘리브레이션 중",
  "Reverse": "후진",
  "System Initializing": "시스템 시작 중",
  "openpilot Unavailable": "오픈파일럿 사용 불가",
  "Be ready to take over at any time": "언제든지 운전대를 잡을 준비를 하세요",
  "WARNING: This branch is not tested": "경고: 테스트되지 않은 브랜치입니다",
  "Dashcam mode": "블랙박스 모드",
  "Dashcam Mode": "블랙박스 모드",
  "Dashcam mode for unsupported car": "지원되지 않는 차량: 블랙박스 모드",
  "Security Key Not Available": "보안 키를 사용할 수 없습니다",
  "BRAKE!": "브레이크!",
  "Emergency Braking: Risk of Collision": "긴급 제동: 충돌 위험",
  "Stock LKAS: Lane Departure Detected": "순정 LKAS: 차선 이탈 감지됨",
  "DISENGAGE IMMEDIATELY": "즉시 제어하세요",
  "Driver Distracted": "운전자 부주의 감지됨",
  "Touch Steering Wheel: No Face Detected": "스티어링 휠을 잡아주세요: 운전자 감지 안됨",
  "Touch Steering Wheel": "스티어링 휠을 잡아주세요",
  "Driver Unresponsive": "운전자 응답 없음",
  "Cancel Pressed": "취소 버튼 눌림",
  "Press Resume to Exit Brake Hold": "브레이크 홀드를 해제하려면 Resume 버튼을 누르세요",
  "Parking Brake Engaged": "주차 브레이크 체결됨",
  "Pedal Pressed": "페달 눌림",
  "Steering Pressed": "스티어링 조작됨",
  "Enable Adaptive Cruise to Engage": "활성화하려면 어댑티브 크루즈를 켜세요",
  "Press Set to Engage": "활성화하려면 SET 버튼을 누르세요",
  "Adaptive Cruise Disabled": "어댑티브 크루즈 꺼짐",
  "TAKE CONTROL IMMEDIATELY": "즉시 제어하세요",
  "Vehicle Steering Time Limit": "차량 조향 시간 제한",
  "Sensor Data Invalid": "센서 데이터 이상",
  "Possible Hardware Issue": "하드웨어 문제가 있을 수 있습니다",
  "openpilot will disengage": "오픈파일럿이 곧 비활성화됩니다",
  "Gear not D": "기어가 D가 아닙니다",
  "Seatbelt Unlatched": "안전벨트가 체결되지 않았습니다",
  "System Lagging": "시스템 지연",
  "Selfdrive Process Lagging: Reboot Your Device": "selfdrive 지연: 기기를 재부팅하세요",
  "Process Not Running": "외부 프로그램 비정상",
  "Radar Error: Restart the Car": "레이더 오류: 차량을 재시동하세요",
  "Radar Temporarily Unavailable": "레이더 일시 사용 불가",
  "Posenet Speed Invalid": "포즈넷 속도 이상",
  "USB Error: Reboot Your Device": "USB 오류: 기기를 재부팅하세요",
  "Camera Malfunction": "카메라 이상",
  "Calibration Invalid": "캘리브레이션 오류",
  "Steering misalignment detected": "조향 정렬 이상 감지됨",
  "Steer ratio mismatch": "조향비 불일치",
  "Abnormal tire stiffness": "타이어 강성 이상",
  "paramsd Temporary Error": "차량 파라미터 일시 오류",
  "System Overheated": "기기 온도 높음",
  "Low Memory": "메모리 부족",
  "High CPU Usage": "CPU 사용량 높음",
  "Driving Model Lagging": "주행 모델 지연",
  "Joystick Mode": "조이스틱 모드",
  "Longitudinal Maneuver Mode": "종방향 테스트 모드",
  "Lateral Maneuver Mode": "횡방향 테스트 모드",
  "Ensure road ahead is clear": "전방 도로가 안전한지 확인하세요",
  "Speed Too High": "속도가 너무 높습니다",
  "Model uncertain at this speed": "현재 속도에서 모델 예측이 불안정합니다",
  "Slow down to engage": "속도를 낮추면 활성화할 수 있습니다",
  "Vehicle Sensors Invalid": "차량 센서 이상",
  "Vehicle Sensors Calibrating": "차량 센서 캘리브레이션 중",
  "Drive to Calibrate": "캘리브레이션을 위해 주행하세요",
  "Bookmark Saved": "북마크 저장됨",
  "Recording Audio Feedback": "음성 피드백 녹음 중",
  "Harness Relay Malfunction": "하네스 릴레이 이상",
  "Check Hardware": "하드웨어 점검 필요",
  "openpilot Canceled": "오픈파일럿 취소됨",
  "Speed too low": "속도가 너무 낮습니다",
}
