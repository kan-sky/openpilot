#!/usr/bin/env python3
from __future__ import annotations

import math
from collections import deque
from datetime import UTC, datetime
from types import SimpleNamespace


HARD_ACCEL_MPS2 = 2.0
HARD_BRAKE_MPS2 = -2.5
LOW_SPEED_MPS = 8.33
STOPPED_MPS = 0.15
MOVING_MPS = 0.55
TIMELINE_INTERVAL_S = 5.0
BLACKBOX_WINDOW_S = 30.0
BLACKBOX_EVENT_LIMIT = 24
TIMELINE_LIMIT = 1200
INPUT_MAX_AGE_S = 0.75  # Includes the 2 Hz longitudinalPlan/DM qlog cadence.
ROUTE_LOCATION_MAX_GAP_S = 2.0
REPORT_SERVICES = ("carState", "carControl", "selfdriveState", "driverMonitoringState", "radarState",
                   "longitudinalPlan", "longitudinalPlanSP")
UNEXPLAINED_BRAKE_COMMAND_MPS2 = -0.75
UNEXPLAINED_ACCEL_COMMAND_MPS2 = 0.75
AUTO_HOLD_CREEP_SPEED_MPS = 0.18
PROCESS_COMMUNICATION_MARKERS = (
  "commissue", "processnotrunning", "processcommunication", "controlsmismatch",
  "communication error", "process error", "프로세스간 통신", "차량과의 통신",
)


def _enum_name(value):
  name = getattr(value, "name", None)
  return str(name if name is not None else value).rsplit(".", 1)[-1]


def _safe_float(value, default=0.0):
  try:
    result = float(value)
    return result if math.isfinite(result) else default
  except (TypeError, ValueError):
    return default


def _iso_at(route_start, first_mono, mono):
  if route_start is None or route_start.tzinfo is None or first_mono is None:
    return None
  timestamp = route_start.timestamp() + max(0.0, mono - first_mono)
  return datetime.fromtimestamp(timestamp, UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _percent(numerator, denominator):
  return round(100.0 * numerator / denominator, 1) if denominator > 0 else None


def _clamp(value, lower, upper):
  return max(lower, min(upper, value))


class DriveReportAccumulator:
  """Builds a compact post-drive report while a route log is read once."""

  def __init__(self, route_start=None, *, route_first_mono=None):
    if route_first_mono is not None and (not math.isfinite(route_first_mono) or route_first_mono < 0):
      raise ValueError("invalid route origin")
    self.route_start = route_start
    self.first_mono = route_first_mono
    self.service_times = {}
    self.service_counts = dict.fromkeys(REPORT_SERVICES, 0)
    self.rejected_samples = 0
    self.timeline_truncated = False
    self.last_mono = None
    self.last_car_mono = None
    self.last_control_mono = None
    self.last_dm_mono = None
    self.last_speed = 0.0
    self.last_accel = None
    self.last_jerk = 0.0
    self.last_event_mono = {}
    self.suspect_started_mono = {}
    self.recent_car = deque(maxlen=700)

    self.op_active = False
    self.op_enabled = False
    self.latest_car = None
    self.latest_lead = None
    self.latest_control_accel = None
    self.latest_plan_accel = None
    self.latest_plan_source = "unknown"
    self.latest_plan_should_stop = None
    self.latest_plan_allow_throttle = None
    self.latest_plan_allow_brake = None
    self.latest_plan_processing_delay = None
    self.latest_plan_solver_time = None
    self.latest_plan_mono = None
    self.latest_radar_mono = None
    self.latest_sp_source = None
    self.latest_speed_limit_active = None
    self.latest_curve_control_active = None
    self.long_active = False
    self.lead_present = False
    self.driver_distracted = False
    self.face_detected = False
    self.steering_pressed = False

    self.car_samples = 0
    self.control_samples = 0
    self.dm_samples = 0
    self.radar_samples = 0
    self.op_active_s = 0.0
    self.manual_s = 0.0
    self.unknown_automation_s = 0.0
    self.car_observed_s = 0.0
    self.dm_vision_available = False
    self.long_active_s = 0.0
    self.throttle_command_s = 0.0
    self.brake_command_s = 0.0
    self.distracted_s = 0.0
    self.face_detected_s = 0.0
    self.dm_observed_s = 0.0
    self.steering_override_s = 0.0

    self.stop_count = 0
    self.harsh_stop_count = 0
    self.hard_accel_count = 0
    self.hard_brake_count = 0
    self.low_speed_oscillation_count = 0
    self.steering_intervention_count = 0
    self.disengagement_count = 0
    self.attention_alert_count = 0
    self.system_alert_count = 0
    self.lead_acquisition_count = 0
    self.fcw_count = 0

    self.peak_accel = 0.0
    self.peak_decel = 0.0
    self.peak_jerk = 0.0
    self.peak_command_accel = 0.0
    self.peak_command_decel = 0.0
    self.low_speed_accel_sign = 0
    self.last_low_speed_switch_mono = -1e9
    self.last_timeline_mono = -1e9
    self.last_alert_type = ""
    self.last_attention_level = "none"

    self.timeline = []
    self.moments = []
    self.blackbox_events = []

  def update(self, msg):
    which = msg.which()
    if which not in REPORT_SERVICES:
      return
    mono_ns = _safe_float(msg.logMonoTime, None)
    if (not getattr(msg, "valid", True) or mono_ns is None or mono_ns < 0
        or mono_ns / 1e9 <= self.service_times.get(which, -1.0)):
      self.rejected_samples += 1
      self.suspect_started_mono.clear()
      return
    mono = mono_ns / 1e9
    if self.first_mono is not None and mono < self.first_mono:
      self.rejected_samples += 1
      return
    if self.first_mono is None:
      self.first_mono = mono
    self.last_mono = max(self.last_mono if self.last_mono is not None else mono, mono)
    self.service_times[which] = mono
    self.service_counts[which] += 1

    if which == "carState":
      self._update_car_state(msg.carState, mono)
    elif which == "carControl":
      self._update_car_control(msg.carControl, mono)
    elif which == "selfdriveState":
      self._update_selfdrive_state(msg.selfdriveState, mono)
    elif which == "driverMonitoringState":
      self._update_driver_monitoring(msg.driverMonitoringState, mono)
    elif which == "radarState":
      self._update_radar(msg.radarState, mono)
    elif which == "longitudinalPlan":
      self._update_longitudinal_plan(msg.longitudinalPlan, mono)
    elif which == "longitudinalPlanSP":
      self._update_longitudinal_plan_sp(msg.longitudinalPlanSP, mono)

  def _fresh(self, service, mono):
    previous = self.service_times.get(service)
    return previous is not None and 0.0 <= mono - previous <= INPUT_MAX_AGE_S

  def _offset(self, mono):
    return max(0.0, mono - (self.first_mono if self.first_mono is not None else mono))

  def _event_allowed(self, event_type, mono, cooldown_s):
    previous = self.last_event_mono.get(event_type, -1e9)
    if mono - previous < cooldown_s:
      return False
    self.last_event_mono[event_type] = mono
    return True

  def _add_moment(self, mono, event_type, severity, title, detail, **extra):
    if len(self.moments) >= 120:
      return
    moment = {
      "time": _iso_at(self.route_start, self.first_mono, mono),
      "offsetS": round(self._offset(mono), 2),
      "type": event_type,
      "severity": severity,
      "title": title,
      "detail": detail,
    }
    moment.update(extra)
    self.moments.append(moment)

  def _sustained_ready(self, event_type, condition, mono, required_s, cooldown_s):
    if not condition:
      self.suspect_started_mono.pop(event_type, None)
      return False
    started = self.suspect_started_mono.setdefault(event_type, mono)
    if mono - started < required_s:
      return False
    self.suspect_started_mono.pop(event_type, None)
    return self._event_allowed(f"blackbox:{event_type}", mono, cooldown_s)

  def _add_blackbox_event(self, mono, event_type, severity, title, detail, metrics, causes):
    if len(self.blackbox_events) >= BLACKBOX_EVENT_LIMIT:
      return
    offset_s = round(self._offset(mono), 2)
    event_id = f"{event_type}-{int(round(offset_s * 1000)):09d}"
    event = {
      "id": event_id,
      "time": _iso_at(self.route_start, self.first_mono, mono),
      "offsetS": offset_s,
      "type": event_type,
      "severity": severity,
      "title": title,
      "detail": detail,
      "captureWindow": {
        "beforeS": BLACKBOX_WINDOW_S,
        "afterS": BLACKBOX_WINDOW_S,
        "startOffsetS": round(max(0.0, offset_s - BLACKBOX_WINDOW_S), 2),
        "endOffsetS": round(offset_s + BLACKBOX_WINDOW_S, 2),
      },
      "metrics": metrics,
      "causeCandidates": causes,
      "causeStatus": "hypothesis_not_diagnosis",
      "package": {"status": "pending"},
    }
    self.blackbox_events.append(event)
    self._add_moment(
      mono, event_type, severity, title, detail,
      blackboxId=event_id,
      speedKph=metrics.get("speedKph"),
    )

  @staticmethod
  def _cause(code, label, confidence, evidence):
    return {
      "code": code,
      "label": label,
      "confidence": confidence,
      "evidence": [str(item) for item in evidence if item],
    }

  def _blackbox_metrics(self, speed, accel, car_state, hold_active=False):
    lead = self.latest_lead
    radar_age = None if self.latest_radar_mono is None or self.last_mono is None else max(0.0, self.last_mono - self.latest_radar_mono)
    plan_age = None if self.latest_plan_mono is None or self.last_mono is None else max(0.0, self.last_mono - self.latest_plan_mono)
    return {
      "speedKph": round(speed * 3.6, 1),
      "actualAccelerationMps2": round(accel, 2),
      "commandedAccelerationMps2": round(self.latest_control_accel, 2) if self.latest_control_accel is not None else None,
      "plannedAccelerationMps2": round(self.latest_plan_accel, 2) if self.latest_plan_accel is not None else None,
      "planSource": self.latest_plan_source,
      "sunnypilotPlanSource": self.latest_sp_source,
      "planShouldStop": self.latest_plan_should_stop,
      "planAllowsThrottle": self.latest_plan_allow_throttle,
      "planAllowsBrake": self.latest_plan_allow_brake,
      "leadDistanceM": round(lead.dRel, 1) if lead is not None else None,
      "leadRelativeSpeedMps": round(lead.vRel, 2) if lead is not None else None,
      "radarAgeS": round(radar_age, 3) if radar_age is not None else None,
      "planAgeS": round(plan_age, 3) if plan_age is not None else None,
      "gasPressed": bool(getattr(car_state, "gasPressed", False)),
      "brakePressed": bool(getattr(car_state, "brakePressed", False)),
      "standstill": bool(getattr(car_state, "standstill", False)),
      "cruiseStandstill": bool(getattr(getattr(car_state, "cruiseState", None), "standstill", False)),
      "brakeHoldActive": bool(getattr(car_state, "brakeHoldActive", False)),
      "holdActive": bool(hold_active),
      "openpilotActive": self.op_active,
      "longActive": self.long_active,
      "inputFreshness": {service: self._fresh(service, self.last_mono) for service in REPORT_SERVICES},
    }

  def _detect_blackbox_events(self, car_state, mono, speed, accel):
    gas_pressed = bool(getattr(car_state, "gasPressed", False))
    brake_pressed = bool(getattr(car_state, "brakePressed", False))
    lead = self.latest_lead
    lead_distance = _safe_float(getattr(lead, "dRel", 1e6), 1e6) if lead is not None else 1e6
    lead_vrel = _safe_float(getattr(lead, "vRel", 0.0)) if lead is not None else 0.0
    control_known = (self._fresh("carControl", mono) and self._fresh("selfdriveState", mono)
                     and self.latest_control_accel is not None)
    plan_known = self._fresh("longitudinalPlan", mono) and self.latest_plan_accel is not None
    radar_known = self._fresh("radarState", mono)
    lead_explains_brake = lead is not None and (lead_distance < 55.0 or lead_vrel < -1.0)
    # Carrot has no longitudinalPlanSP. Absence cannot exclude curve/speed-limit braking.
    assists_known = (self._fresh("longitudinalPlanSP", mono) and self.latest_sp_source is not None
                     and self.latest_speed_limit_active is not None and self.latest_curve_control_active is not None)
    assisted_speed_control = self.latest_speed_limit_active or self.latest_curve_control_active or self.latest_sp_source != "cruise"

    unexpected_brake = (
      control_known and plan_known and radar_known and assists_known and
      self.op_active and self.long_active and speed > 2.0 and not brake_pressed and
      self.latest_control_accel <= UNEXPLAINED_BRAKE_COMMAND_MPS2 and accel < -0.25 and
      not lead_explains_brake and self.latest_plan_should_stop is False and not assisted_speed_control
    )
    if self._sustained_ready("unexplained_braking", unexpected_brake, mono, 0.65, 12.0):
      metrics = self._blackbox_metrics(speed, accel, car_state)
      causes = [
        self._cause(
          "cruise_target_or_controller_change", "크루즈 목표 또는 제어기 출력 변화", "medium",
          [f"명령 {self.latest_control_accel:.2f} m/s²", f"계획 소스 {self.latest_plan_source}"],
        ),
      ]
      if (metrics["planAgeS"] or 0.0) > 0.3 or (metrics["radarAgeS"] or 0.0) > 0.3:
        causes.insert(0, self._cause(
          "stale_planning_input", "레이더 또는 계획 데이터 지연", "high",
          [f"레이더 지연 {metrics['radarAgeS']}초", f"계획 지연 {metrics['planAgeS']}초"],
        ))
      self._add_blackbox_event(
        mono, "unexplained_braking", "high", "원인 불명 제동 후보",
        "선행차·정지·커브·속도제한으로 설명되지 않는 제동 명령이 지속됐습니다.", metrics, causes,
      )

    command_conflicts_with_plan = plan_known and (self.latest_plan_accel < -0.15 or self.latest_plan_should_stop)
    closing_lead = radar_known and lead is not None and lead_distance < 35.0 and lead_vrel < -0.7
    unexpected_accel = (
      control_known and self.op_active and self.long_active and speed > 0.5 and not gas_pressed and
      self.latest_control_accel >= UNEXPLAINED_ACCEL_COMMAND_MPS2 and
      (command_conflicts_with_plan or closing_lead)
    )
    if self._sustained_ready("unexplained_acceleration", unexpected_accel, mono, 0.5, 12.0):
      metrics = self._blackbox_metrics(speed, accel, car_state)
      causes = []
      if command_conflicts_with_plan:
        causes.append(self._cause(
          "plan_control_mismatch", "롱컨 계획과 액추에이터 명령 불일치", "high",
          [f"계획 {self.latest_plan_accel:.2f} m/s²", f"명령 {self.latest_control_accel:.2f} m/s²"],
        ))
      if closing_lead:
        causes.append(self._cause(
          "late_lead_response", "가까워지는 선행차 반응 지연", "high",
          [f"거리 {lead_distance:.1f} m", f"상대속도 {lead_vrel:.2f} m/s"],
        ))
      self._add_blackbox_event(
        mono, "unexplained_acceleration", "critical", "의도와 다른 가속 후보",
        "정지 계획 또는 가까워지는 선행차와 반대되는 가속 명령이 지속됐습니다.", metrics, causes,
      )

    vehicle_standstill = bool(
      getattr(car_state, "standstill", False) or
      getattr(getattr(car_state, "cruiseState", None), "standstill", False)
    )
    hold_active = bool(getattr(car_state, "brakeHoldActive", False) or (control_known and self.long_active and vehicle_standstill))
    hold_creep = control_known and hold_active and not gas_pressed and speed > AUTO_HOLD_CREEP_SPEED_MPS
    if self._sustained_ready("auto_hold_creep", hold_creep, mono, 0.4, 8.0):
      metrics = self._blackbox_metrics(speed, accel, car_state, hold_active=True)
      causes = [self._cause(
        "insufficient_hold_force", "오토홀드 제동력 부족 또는 해제", "high",
        [f"홀드 중 속도 {speed * 3.6:.1f} km/h", f"명령 {self.latest_control_accel:.2f} m/s²"],
      )]
      if vehicle_standstill:
        causes.append(self._cause(
          "standstill_speed_mismatch", "정차 상태와 실제 이동 불일치", "high",
          ["standstill=true인 동안 차량 속도가 검출됨"],
        ))
      self._add_blackbox_event(
        mono, "auto_hold_creep", "critical", "오토홀드 밀림 감지",
        "오토홀드 상태에서 운전자 가속 입력 없이 차량 이동이 검출됐습니다.", metrics, causes,
      )

  def _update_car_state(self, car_state, mono):
    if any(_safe_float(getattr(car_state, key, None), None) is None for key in ("vEgo", "aEgo")):
      self.rejected_samples += 1
      self.suspect_started_mono.clear()
      self.last_car_mono = None
      return
    raw_speed = max(0.0, _safe_float(getattr(car_state, "vEgo", 0.0)))
    speed = raw_speed
    cluster_speed = _safe_float(getattr(car_state, "vEgoCluster", 0.0))
    if cluster_speed > 0.0:
      speed = max(raw_speed, cluster_speed)
    accel = _safe_float(getattr(car_state, "aEgo", 0.0))
    dt = mono - self.last_car_mono if self.last_car_mono is not None else 0.0
    valid_dt = 0.0 < dt <= 0.5
    if not valid_dt:
      self.suspect_started_mono.clear()
      self.recent_car.clear()
      self.last_jerk = 0.0

    if valid_dt:
      self.car_observed_s += dt
      if not self._fresh("selfdriveState", mono):
        self.unknown_automation_s += dt
      elif self.op_active:
        self.op_active_s += dt
      else:
        self.manual_s += dt
      if self._fresh("selfdriveState", mono) and self.op_active and bool(getattr(car_state, "steeringPressed", False)):
        self.steering_override_s += dt

    if valid_dt and self.last_accel is not None and dt >= 0.02:
      jerk = _clamp((accel - self.last_accel) / dt, -20.0, 20.0)
      self.last_jerk = 0.72 * self.last_jerk + 0.28 * jerk
    self.peak_accel = max(self.peak_accel, accel)
    self.peak_decel = min(self.peak_decel, accel)
    self.peak_jerk = max(self.peak_jerk, abs(self.last_jerk))

    if accel >= HARD_ACCEL_MPS2 and self._event_allowed("hard_accel", mono, 4.0):
      self.hard_accel_count += 1
      self._add_moment(mono, "hard_accel", "medium", "강한 가속", f"가속도 {accel:.1f} m/s²", speedKph=round(speed * 3.6, 1))
    if accel <= HARD_BRAKE_MPS2 and self._event_allowed("hard_brake", mono, 4.0):
      self.hard_brake_count += 1
      self._add_moment(mono, "hard_brake", "high", "강한 제동", f"감속도 {accel:.1f} m/s²", speedKph=round(speed * 3.6, 1))

    accel_sign = 1 if accel > 0.35 else -1 if accel < -0.35 else 0
    if 0.7 < speed < LOW_SPEED_MPS and accel_sign and self.low_speed_accel_sign and accel_sign != self.low_speed_accel_sign:
      if mono - self.last_low_speed_switch_mono >= 1.0:
        self.low_speed_oscillation_count += 1
        self.last_low_speed_switch_mono = mono
    if accel_sign:
      self.low_speed_accel_sign = accel_sign
    if speed >= LOW_SPEED_MPS:
      self.low_speed_accel_sign = 0

    self.recent_car.append((mono, speed, accel, self.last_jerk))
    while self.recent_car and mono - self.recent_car[0][0] > 6.0:
      self.recent_car.popleft()

    if valid_dt and self.last_speed >= MOVING_MPS and speed <= STOPPED_MPS:
      self._record_stop(mono)

    steering_now = bool(getattr(car_state, "steeringPressed", False))
    if self._fresh("selfdriveState", mono) and self.op_active and steering_now and not self.steering_pressed and speed > 1.4:
      self.steering_intervention_count += 1
      self._add_moment(mono, "steering_intervention", "medium", "운전자 조향 개입",
                       "오픈파일럿 주행 중 운전자가 조향에 개입했습니다.", speedKph=round(speed * 3.6, 1))
    self.steering_pressed = steering_now

    # Retain only scalars, not Cap'n Proto readers backed by an entire log buffer.
    self.latest_car = SimpleNamespace(**{key: getattr(car_state, key, None) for key in (
      "vEgo", "aEgo", "brakePressed", "gasPressed", "steeringPressed", "standstill", "brakeHoldActive",
    )})
    # Incident triggers use the vehicle estimate rather than the display speed.
    # vEgoCluster can remain non-zero briefly after a real stop.
    self._detect_blackbox_events(car_state, mono, raw_speed, accel)
    self.car_samples += 1
    self.last_speed = speed
    self.last_accel = accel
    self.last_car_mono = mono

    if mono - self.last_timeline_mono >= TIMELINE_INTERVAL_S:
      lead = self.latest_lead
      if len(self.timeline) >= TIMELINE_LIMIT:
        self.timeline_truncated = True
        return
      self.timeline.append({
        "time": _iso_at(self.route_start, self.first_mono, mono),
        "offsetS": round(self._offset(mono), 1),
        "speedKph": round(speed * 3.6, 1),
        "accelMps2": round(accel, 2),
        "opActive": self.op_active if self._fresh("selfdriveState", mono) else None,
        "leadDistanceM": round(_safe_float(getattr(lead, "dRel", None)), 1) if lead is not None else None,
        "leadRelativeSpeedMps": round(_safe_float(getattr(lead, "vRel", None)), 2) if lead is not None else None,
        "distracted": self.driver_distracted if self.dm_vision_available and self._fresh("driverMonitoringState", mono) else None,
      })
      self.last_timeline_mono = mono

  def _record_stop(self, mono):
    self.stop_count += 1
    low_speed = [sample for sample in self.recent_car if sample[1] <= 2.0]
    peak_decel = min((sample[2] for sample in low_speed), default=0.0)
    peak_jerk = max((abs(sample[3]) for sample in low_speed), default=0.0)
    harsh = peak_decel < -1.35 or peak_jerk > 4.5
    if harsh:
      self.harsh_stop_count += 1
      self._add_moment(
        mono, "harsh_stop", "high", "정지 직전 충격",
        f"저속 최대 감속 {peak_decel:.1f} m/s² · jerk {peak_jerk:.1f} m/s³",
        speedKph=0.0,
      )

  def _update_car_control(self, car_control, mono):
    dt = mono - self.last_control_mono if self.last_control_mono is not None else 0.0
    if not 0.0 < dt <= 0.5:
      dt = 0.0
    actuators = getattr(car_control, "actuators", None)
    accel = _safe_float(getattr(actuators, "accel", None), None)
    self.long_active = bool(getattr(car_control, "longActive", False))
    self.latest_control_accel = accel
    if accel is None:
      self.last_control_mono = None
      return
    if self.long_active:
      self.long_active_s += dt
      if accel > 0.08:
        self.throttle_command_s += dt
      elif accel < -0.08:
        self.brake_command_s += dt
    self.peak_command_accel = max(self.peak_command_accel, accel)
    self.peak_command_decel = min(self.peak_command_decel, accel)
    self.control_samples += 1
    self.last_control_mono = mono

  def _update_selfdrive_state(self, state, mono):
    active = bool(getattr(state, "active", False))
    enabled = bool(getattr(state, "enabled", False))
    if self.op_active and not active:
      self.disengagement_count += 1
      car = self.latest_car
      brake = bool(getattr(car, "brakePressed", False)) if car is not None else False
      gas = bool(getattr(car, "gasPressed", False)) if car is not None else False
      steering = bool(getattr(car, "steeringPressed", False)) if car is not None else False
      reason = "브레이크" if brake else "가속 페달" if gas else "조향" if steering else "시스템 또는 버튼"
      self._add_moment(mono, "disengagement", "medium", "오픈파일럿 해제", f"해제 추정 원인: {reason}")

    alert = getattr(state, "alertType", "")
    alert_type = str(alert or "")
    alert_status = _enum_name(getattr(state, "alertStatus", "normal")).lower()
    if alert_type and alert_type != self.last_alert_type and alert_status not in ("normal", "none"):
      self.system_alert_count += 1
      text1 = str(getattr(state, "alertText1", "") or "")
      text2 = str(getattr(state, "alertText2", "") or "")
      detail = " · ".join(value for value in (text1, text2) if value) or alert_type
      severity = "high" if alert_status in ("critical", "userprompt") else "medium"
      alert_search = f"{alert_type} {detail}".lower().replace("_", "")
      if any(marker in alert_search for marker in PROCESS_COMMUNICATION_MARKERS):
        metrics = self._blackbox_metrics(
          max(0.0, _safe_float(getattr(self.latest_car, "vEgo", 0.0))) if self.latest_car is not None else 0.0,
          _safe_float(getattr(self.latest_car, "aEgo", 0.0)) if self.latest_car is not None else 0.0,
          self.latest_car or object(),
        )
        metrics.update({
          "alertType": alert_type,
          "alertStatus": alert_status,
          "alertText": detail[:180],
          "plannerProcessingDelayS": self.latest_plan_processing_delay,
          "plannerSolverTimeS": self.latest_plan_solver_time,
        })
        causes = [self._cause(
          "message_or_process_timeout", "메시지 지연 또는 프로세스 응답 중단", "high",
          [alert_type, detail[:120]],
        )]
        if self.latest_plan_processing_delay is not None and self.latest_plan_processing_delay > 0.25:
          causes.append(self._cause(
            "planner_delay", "롱컨 계획 처리 지연", "medium",
            [f"처리 지연 {self.latest_plan_processing_delay:.3f}초"],
          ))
        self._add_blackbox_event(
          mono, "process_communication_error", severity, "프로세스간 통신 오류",
          detail[:180], metrics, causes,
        )
      else:
        self._add_moment(mono, "system_alert", severity, "주행 시스템 알림", detail[:180])
    self.last_alert_type = alert_type
    self.op_active = active
    self.op_enabled = enabled

  def _update_driver_monitoring(self, dm, mono):
    dt = mono - self.last_dm_mono if self.last_dm_mono is not None else 0.0
    vision = getattr(dm, "visionPolicyState", None)
    available = (vision is not None and _enum_name(getattr(dm, "activePolicy", "vision")) == "vision"
                 and not getattr(dm, "cameraUnavailable", False) and not getattr(dm, "dm2Disabled", False))
    if available and self.dm_vision_available and 0.0 < dt <= 0.5:
      self.dm_observed_s += dt
      if self.driver_distracted:
        self.distracted_s += dt
      if self.face_detected:
        self.face_detected_s += dt

    self.dm_vision_available = available
    distracted = bool(getattr(vision, "isDistracted", False)) if vision is not None else False
    face_detected = bool(getattr(vision, "faceDetected", False)) if vision is not None else False
    level = _enum_name(getattr(dm, "alertLevel", "none")).lower()
    if level not in ("none", "0") and level != self.last_attention_level:
      self.attention_alert_count += 1
      severity = "high" if level in ("three", "3") else "medium"
      self._add_moment(mono, "attention", severity, "운전자 주의 경고", "운전자 모니터링 경고가 발생했습니다.")
    self.last_attention_level = level
    self.driver_distracted = distracted
    self.face_detected = face_detected
    self.dm_samples += 1
    self.last_dm_mono = mono

  def _update_radar(self, radar_state, mono):
    lead = getattr(radar_state, "leadOne", None)
    if lead is None or (getattr(lead, "status", False) and any(
        _safe_float(getattr(lead, key, None), None) is None for key in ("dRel", "vRel"))):
      self.service_times.pop("radarState", None)
      self.latest_lead = None
      return
    present = bool(getattr(lead, "status", False)) if lead is not None else False
    if present and not self.lead_present:
      self.lead_acquisition_count += 1
      distance = _safe_float(getattr(lead, "dRel", 0.0))
      relative_speed = _safe_float(getattr(lead, "vRel", 0.0))
      if distance < 35.0 and relative_speed < -2.0 and self._event_allowed("close_lead_acquired", mono, 5.0):
        self._add_moment(
          mono, "close_lead_acquired", "high", "가까운 선행차 진입",
          f"거리 {distance:.1f} m · 상대속도 {relative_speed * 3.6:.0f} km/h",
          leadDistanceM=round(distance, 1), relativeSpeedMps=round(relative_speed, 2),
        )
    self.lead_present = present
    self.latest_lead = SimpleNamespace(dRel=float(lead.dRel), vRel=float(lead.vRel)) if present else None
    self.latest_radar_mono = mono
    self.radar_samples += 1

  def _update_longitudinal_plan(self, plan, mono):
    self.latest_plan_accel = _safe_float(getattr(plan, "aTarget", None), None)
    self.latest_plan_source = _enum_name(getattr(plan, "longitudinalPlanSource", "unknown"))
    self.latest_plan_should_stop = getattr(plan, "shouldStop", None)
    self.latest_plan_allow_throttle = getattr(plan, "allowThrottle", None)
    self.latest_plan_allow_brake = getattr(plan, "allowBrake", None)
    processing_delay = _safe_float(getattr(plan, "processingDelay", None), -1.0)
    solver_time = _safe_float(getattr(plan, "solverExecutionTime", None), -1.0)
    self.latest_plan_processing_delay = processing_delay if processing_delay >= 0.0 else None
    self.latest_plan_solver_time = solver_time if solver_time >= 0.0 else None
    self.latest_plan_mono = mono
    if bool(getattr(plan, "fcw", False)) and self._event_allowed("fcw", mono, 8.0):
      self.fcw_count += 1
      self._add_moment(mono, "fcw", "critical", "전방 충돌 경고", "롱컨 전방 충돌 경고가 발생했습니다.")

  def _update_longitudinal_plan_sp(self, plan, mono):
    source = getattr(plan, "longitudinalPlanSource", None)
    self.latest_sp_source = _enum_name(source) if source is not None else None
    speed_limit = getattr(plan, "speedLimit", None)
    assist = getattr(speed_limit, "assist", None) if speed_limit is not None else None
    self.latest_speed_limit_active = getattr(assist, "active", None)
    smart_cruise = getattr(plan, "smartCruiseControl", None)
    vision = getattr(smart_cruise, "vision", None) if smart_cruise is not None else None
    map_control = getattr(smart_cruise, "map", None) if smart_cruise is not None else None
    vision_active = getattr(vision, "active", None)
    map_active = getattr(map_control, "active", None)
    self.latest_curve_control_active = (bool(vision_active or map_active)
                                        if vision_active is not None and map_active is not None else None)

  def _attach_route_context(self, route):
    timed_points = []
    for point in route:
      try:
        when = datetime.fromisoformat(str(point.get("time", "")).replace("Z", "+00:00"))
        latitude = _safe_float(point.get("latitude"), None)
        longitude = _safe_float(point.get("longitude"), None)
        if (when.tzinfo is None or latitude is None or longitude is None
            or isinstance(point.get("latitude"), bool) or isinstance(point.get("longitude"), bool)
            or not -90 <= latitude <= 90 or not -180 <= longitude <= 180):
          continue
        timestamp = when.timestamp()
        if math.isfinite(timestamp):
          timed_points.append((timestamp, latitude, longitude))
      except (TypeError, ValueError):
        continue

    for moment in [*self.moments, *self.blackbox_events]:
      # finalize may be called again with a different route; never retain an old fix.
      moment.pop("latitude", None)
      moment.pop("longitude", None)
      moment["locationContext"] = {"status": "unavailable", "maxGapS": ROUTE_LOCATION_MAX_GAP_S}
      if not moment.get("time") or not timed_points:
        continue
      try:
        timestamp = datetime.fromisoformat(moment["time"].replace("Z", "+00:00")).timestamp()
      except ValueError:
        continue
      point_time, latitude, longitude = min(timed_points, key=lambda item: abs(item[0] - timestamp))
      delta = point_time - timestamp
      if abs(delta) > ROUTE_LOCATION_MAX_GAP_S:
        continue
      moment["latitude"] = latitude
      moment["longitude"] = longitude
      moment["locationContext"].update(status="approximate", source="nearest_route_point", timeDeltaS=round(delta, 3))

  def finalize(self, route):
    self._attach_route_context(route)
    duration_s = max(0.0, (self.last_mono or 0.0) - (self.first_mono or 0.0))
    observed_drive_s = self.op_active_s + self.manual_s
    op_usage = _percent(self.op_active_s, observed_drive_s)
    face_percent = _percent(self.face_detected_s, self.dm_observed_s)
    distracted_percent = _percent(self.distracted_s, self.dm_observed_s)

    comfort_penalty = (
      self.harsh_stop_count * 10.0 + self.hard_brake_count * 6.0 +
      self.hard_accel_count * 4.0 + min(20.0, self.low_speed_oscillation_count * 1.5) +
      max(0.0, self.peak_jerk - 2.5) * 1.2
    )
    comfort_score = round(_clamp(100.0 - comfort_penalty, 0.0, 100.0)) if self.car_observed_s > 0 else None
    attention_score = None
    if self.dm_observed_s >= 10.0:
      attention_score = round(_clamp(100.0 - (distracted_percent or 0.0) * 1.2 - self.attention_alert_count * 8.0, 0.0, 100.0))
    stability_score = (round(_clamp(100.0 - self.system_alert_count * 8.0 - self.disengagement_count * 2.0, 0.0, 100.0))
                       if self.service_counts["selfdriveState"] > 1 else None)

    important = sorted(
      self.moments,
      key=lambda item: ({"critical": 3, "high": 2, "medium": 1}.get(item["severity"], 0), item["offsetS"]),
      reverse=True,
    )[:12]
    confidence = "high" if self.car_samples >= 500 and duration_s >= 60 else "medium" if self.car_samples >= 100 else "low"
    for event in self.blackbox_events:
      event["captureWindow"]["endOffsetS"] = round(min(duration_s, event["captureWindow"]["endOffsetS"]), 2)

    return {
      "schemaVersion": "wayon-drive-report-v2",
      "generatedAt": datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z"),
      "dataQuality": {
        "confidence": confidence,
        "durationS": round(duration_s, 1),
        "carStateSamples": self.car_samples,
        "controlSamples": self.control_samples,
        "driverMonitoringSamples": self.dm_samples,
        "radarSamples": self.radar_samples,
        "routeFirstMonoS": self.first_mono,
        "serviceSamples": dict(self.service_counts),
        "missingServices": [service for service, count in self.service_counts.items() if count == 0],
        "rejectedSamples": self.rejected_samples,
        "timelineTruncated": self.timeline_truncated,
        "scoresAreHeuristic": True,
        "timestampAvailable": self.route_start is not None and self.route_start.tzinfo is not None,
        "routeLocationMaxGapS": ROUTE_LOCATION_MAX_GAP_S,
        "unexplainedBrakingAssessment": "available" if self.service_counts["longitudinalPlanSP"] else "unknown_missing_assist_context",
      },
      "scores": {
        "comfort": comfort_score,
        "attention": attention_score,
        "systemStability": stability_score,
      },
      "automation": {
        "opActiveS": round(self.op_active_s, 1),
        "manualS": round(self.manual_s, 1),
        "unknownS": round(self.unknown_automation_s, 1),
        "opUsagePercent": op_usage,
        "disengagementCount": self.disengagement_count,
        "steeringInterventionCount": self.steering_intervention_count,
      },
      "comfort": {
        "stopCount": self.stop_count,
        "harshStopCount": self.harsh_stop_count,
        "hardAccelerationCount": self.hard_accel_count,
        "hardBrakingCount": self.hard_brake_count,
        "lowSpeedOscillationCount": self.low_speed_oscillation_count,
        "peakAccelerationMps2": round(self.peak_accel, 2),
        "peakDecelerationMps2": round(self.peak_decel, 2),
        "peakJerkMps3": round(self.peak_jerk, 2),
      },
      "longitudinal": {
        "longActiveS": round(self.long_active_s, 1),
        "throttleCommandS": round(self.throttle_command_s, 1),
        "brakeCommandS": round(self.brake_command_s, 1),
        "peakCommandAccelerationMps2": round(self.peak_command_accel, 2),
        "peakCommandDecelerationMps2": round(self.peak_command_decel, 2),
        "leadAcquisitionCount": self.lead_acquisition_count,
        "fcwCount": self.fcw_count,
      },
      "attention": {
        "observedS": round(self.dm_observed_s, 1),
        "faceDetectedPercent": face_percent,
        "distractedS": round(self.distracted_s, 1),
        "distractedPercent": distracted_percent,
        "alertCount": self.attention_alert_count,
      },
      "system": {
        "alertCount": self.system_alert_count,
      },
      "blackbox": {
        "schemaVersion": "wayon-drive-blackbox-v1",
        "eventCount": len(self.blackbox_events),
        "windowBeforeS": BLACKBOX_WINDOW_S,
        "windowAfterS": BLACKBOX_WINDOW_S,
        "events": self.blackbox_events,
      },
      "moments": important,
      "timeline": list(self.timeline),
    }


def build_route_analysis(route_group, route, route_start=None, *, route_first_mono=None, allowed=None, limits=None):
  """Optional bounded offroad qlog pass; never uploads or modifies the legacy report.

  The uploader may instead feed DriveReportAccumulator inside its existing bounded
  qlog loop, then attach finalize(route) as payload['analysis']. Pass the same
  route_start/route_first_mono used for GPS timestamp reconstruction. Missing UTC
  stays null. WorkAborted must prevent marking the route as successfully reported.
  """
  from openpilot.system.wayon_event_blackbox import WorkBudget, WorkAborted, iter_route_log, offroad_allowed, route_log_file, route_segments

  budget = WorkBudget(offroad_allowed if allowed is None else allowed, limits)
  budget.check()
  segments = route_segments(route_group, budget)
  if segments and segments[0][0] != 0 and route_first_mono is None:
    raise WorkAborted("route origin unavailable; pass route_first_mono")
  accumulator = DriveReportAccumulator(route_start, route_first_mono=route_first_mono)
  missing = []
  for segment, directory, _ in segments:
    log_file = route_log_file(directory, qlog_only=True, budget=budget)
    if log_file is None:
      missing.append(segment)
      if segment == 0 and route_first_mono is None:
        raise WorkAborted("route origin qlog unavailable")
      continue
    for msg in iter_route_log(log_file, budget=budget):
      accumulator.update(msg)
  budget.check()
  result = accumulator.finalize(route)
  result["dataQuality"].update(source="sampled_qlog", missingSegments=missing,
                               allListedSegmentsRead=bool(segments) and not missing)
  return result
