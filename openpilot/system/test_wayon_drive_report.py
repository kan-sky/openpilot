from types import SimpleNamespace
from datetime import UTC, datetime, timedelta

import pytest

from openpilot.system.wayon_drive_report import DriveReportAccumulator, ROUTE_LOCATION_MAX_GAP_S, TIMELINE_LIMIT, build_route_analysis


def message(which, mono_s, payload):
  return SimpleNamespace(which=lambda: which, logMonoTime=int(mono_s * 1e9), **{which: payload})


def car_state(speed, accel=0.0, steering=False, brake=False, gas=False):
  return SimpleNamespace(vEgo=speed, vEgoCluster=0.0, aEgo=accel, steeringPressed=steering,
                         brakePressed=brake, gasPressed=gas, standstill=False,
                         brakeHoldActive=False, cruiseState=SimpleNamespace(standstill=False))


def car_control(accel, long_active=True):
  return SimpleNamespace(actuators=SimpleNamespace(accel=accel), longActive=long_active)


def longitudinal_plan(accel=0.0, source="cruise", should_stop=False):
  return SimpleNamespace(
    aTarget=accel,
    longitudinalPlanSource=source,
    shouldStop=should_stop,
    allowThrottle=True,
    allowBrake=True,
    processingDelay=0.02,
    solverExecutionTime=0.01,
    fcw=False,
  )


def assist_plan():
  return SimpleNamespace(longitudinalPlanSource="cruise", speedLimit=SimpleNamespace(assist=SimpleNamespace(active=False)),
                         smartCruiseControl=SimpleNamespace(vision=SimpleNamespace(active=False), map=SimpleNamespace(active=False)))


def assisted_inputs(report, mono, *, command=-1.1, plan=-0.2, sp=True, lead=None):
  report.update(message("selfdriveState", mono, SimpleNamespace(active=True, enabled=True, alertType="", alertStatus="normal")))
  report.update(message("longitudinalPlan", mono, longitudinal_plan(accel=plan)))
  report.update(message("carControl", mono, car_control(command)))
  report.update(message("radarState", mono, SimpleNamespace(leadOne=lead or SimpleNamespace(status=False))))
  if sp:
    report.update(message("longitudinalPlanSP", mono, assist_plan()))


def test_drive_report_counts_op_usage_and_harsh_stop():
  report = DriveReportAccumulator()
  report.update(message("selfdriveState", 0.0, SimpleNamespace(
    active=True, enabled=True, alertType="", alertStatus="normal")))
  report.update(message("carState", 0.0, car_state(4.0)))
  report.update(message("carState", 0.1, car_state(2.0, -1.0)))
  report.update(message("carState", 0.2, car_state(0.7, -1.7)))
  report.update(message("carState", 0.3, car_state(0.1, -1.5)))

  result = report.finalize([])
  assert result["automation"]["opUsagePercent"] == 100.0
  assert result["comfort"]["stopCount"] == 1
  assert result["comfort"]["harshStopCount"] == 1
  assert any(moment["type"] == "harsh_stop" for moment in result["moments"])


def test_drive_report_tracks_distraction_and_close_lead():
  report = DriveReportAccumulator()
  vision = SimpleNamespace(isDistracted=True, faceDetected=True)
  report.update(message("driverMonitoringState", 0.0, SimpleNamespace(
    visionPolicyState=vision, alertLevel="one")))
  report.update(message("driverMonitoringState", 1.0, SimpleNamespace(
    visionPolicyState=vision, alertLevel="one")))
  lead = SimpleNamespace(status=True, dRel=18.0, vRel=-4.0)
  report.update(message("radarState", 1.1, SimpleNamespace(leadOne=lead)))

  result = report.finalize([])
  assert result["attention"]["alertCount"] == 1
  assert result["longitudinal"]["leadAcquisitionCount"] == 1
  assert any(moment["type"] == "close_lead_acquired" for moment in result["moments"])


def test_drive_report_deduplicates_sustained_hard_braking():
  report = DriveReportAccumulator()
  for index in range(30):
    report.update(message("carState", index * 0.1, car_state(15.0, -3.0)))
  result = report.finalize([])
  assert result["comfort"]["hardBrakingCount"] == 1


def test_blackbox_detects_unexplained_braking_without_a_lead():
  report = DriveReportAccumulator()
  for index in range(10):
    assisted_inputs(report, 100.0 + index * 0.1)
    report.update(message("carState", 100.0 + index * 0.1, car_state(12.0, -0.8)))

  events = report.finalize([])["blackbox"]["events"]
  assert [event["type"] for event in events] == ["unexplained_braking"]
  assert events[0]["captureWindow"]["beforeS"] == 30.0
  assert events[0]["metrics"]["commandedAccelerationMps2"] == -1.1


def test_blackbox_does_not_call_explained_lead_braking_unexplained():
  report = DriveReportAccumulator()
  report.update(message("selfdriveState", 100.0, SimpleNamespace(
    active=True, enabled=True, alertType="", alertStatus="normal")))
  report.update(message("radarState", 100.0, SimpleNamespace(
    leadOne=SimpleNamespace(status=True, dRel=22.0, vRel=-2.0))))
  report.update(message("longitudinalPlan", 100.0, longitudinal_plan(accel=-1.0, source="lead0")))
  report.update(message("carControl", 100.0, car_control(-1.1)))
  for index in range(10):
    report.update(message("carState", 100.0 + index * 0.1, car_state(12.0, -0.8)))

  events = report.finalize([])["blackbox"]["events"]
  assert not any(event["type"] == "unexplained_braking" for event in events)


def test_blackbox_detects_auto_hold_creep_with_standstill_still_true():
  report = DriveReportAccumulator()
  report.update(message("selfdriveState", 100.0, SimpleNamespace(
    active=True, enabled=True, alertType="", alertStatus="normal")))
  report.update(message("carControl", 100.0, car_control(-0.4)))
  for index in range(8):
    state = car_state(0.32, 0.02)
    state.standstill = True
    state.brakeHoldActive = True
    state.cruiseState.standstill = True
    report.update(message("carState", 100.0 + index * 0.1, state))

  events = report.finalize([])["blackbox"]["events"]
  creep = next(event for event in events if event["type"] == "auto_hold_creep")
  assert creep["metrics"]["standstill"] is True
  assert any(cause["code"] == "standstill_speed_mismatch" for cause in creep["causeCandidates"])


def test_blackbox_ignores_stale_cluster_speed_during_real_stop():
  report = DriveReportAccumulator()
  report.update(message("selfdriveState", 100.0, SimpleNamespace(
    active=True, enabled=True, alertType="", alertStatus="normal")))
  report.update(message("carControl", 100.0, car_control(-0.4)))
  for index in range(8):
    state = car_state(0.0, 0.0)
    state.vEgoCluster = 0.5
    state.standstill = True
    state.brakeHoldActive = True
    state.cruiseState.standstill = True
    report.update(message("carState", 100.0 + index * 0.1, state))

  events = report.finalize([])["blackbox"]["events"]
  assert not any(event["type"] == "auto_hold_creep" for event in events)


def test_blackbox_records_process_communication_alert_once():
  report = DriveReportAccumulator()
  alert = SimpleNamespace(
    active=True,
    enabled=True,
    alertType="commIssue/warning",
    alertStatus="userPrompt",
    alertText1="프로세스간 통신 오류",
    alertText2="장치를 재시작하세요",
  )
  report.update(message("selfdriveState", 100.0, alert))
  report.update(message("selfdriveState", 100.1, alert))

  events = report.finalize([])["blackbox"]["events"]
  assert [event["type"] for event in events] == ["process_communication_error"]
  assert events[0]["metrics"]["alertType"] == "commIssue/warning"


def test_empty_report_is_not_a_perfect_drive():
  result = DriveReportAccumulator().finalize([])
  assert result["scores"] == {"comfort": None, "attention": None, "systemStability": None}
  assert result["automation"]["opUsagePercent"] is None
  assert result["dataQuality"]["unexplainedBrakingAssessment"] == "unknown_missing_assist_context"


def test_carrot_without_sp_keeps_other_analysis_but_cannot_exclude_speed_assists():
  report = DriveReportAccumulator()
  for index in range(20):
    mono = index * 0.1
    assisted_inputs(report, mono, sp=False)
    report.update(message("carState", mono, car_state(12.0, -3.0)))
  result = report.finalize([])
  assert result["comfort"]["hardBrakingCount"] == 1
  assert result["automation"]["opUsagePercent"] == 100
  assert "longitudinalPlanSP" in result["dataQuality"]["missingServices"]
  assert not any(e["type"] == "unexplained_braking" for e in result["blackbox"]["events"])


def test_carrot_acceleration_plan_conflict_does_not_require_sp():
  report = DriveReportAccumulator()
  for index in range(10):
    mono = index * 0.1
    assisted_inputs(report, mono, command=1.0, plan=-0.5, sp=False)
    report.update(message("carState", mono, car_state(12.0, 0.5)))
  events = report.finalize([])["blackbox"]["events"]
  assert [e["type"] for e in events] == ["unexplained_acceleration"]
  assert events[0]["metrics"]["sunnypilotPlanSource"] is None
  assert events[0]["causeStatus"] == "hypothesis_not_diagnosis"


def test_stale_inputs_cannot_create_a_new_event_or_claim_manual_driving():
  report = DriveReportAccumulator()
  assisted_inputs(report, 0.0, command=1.0, plan=-0.5)
  for index in range(30):
    report.update(message("carState", 10 + index * 0.1, car_state(12.0, 0.5)))
  result = report.finalize([])
  assert not result["blackbox"]["events"]
  assert result["automation"]["manualS"] == 0
  assert result["automation"]["unknownS"] > 2
  assert result["automation"]["opUsagePercent"] is None


def test_gap_resets_sustained_candidate_and_stop_history():
  report = DriveReportAccumulator()
  for mono in (0.0, 0.1, 0.2, 5.0, 5.1, 5.2):
    assisted_inputs(report, mono)
    report.update(message("carState", mono, car_state(12.0, -0.8)))
  assert not report.finalize([])["blackbox"]["events"]
  report.update(message("carState", 20, car_state(0.0)))
  assert report.finalize([])["comfort"]["stopCount"] == 0


def test_zero_origin_and_explicit_utc_origin_are_preserved():
  origin = datetime(2026, 10, 28, tzinfo=UTC)
  report = DriveReportAccumulator(origin, route_first_mono=0.0)
  report.update(message("carState", 1.5, car_state(12.0, -3.0)))
  moment = report.finalize([])["moments"][0]
  assert moment["offsetS"] == 1.5
  assert moment["time"] == "2026-10-28T00:00:01.500Z"


def test_missing_utc_does_not_invent_a_timestamp():
  report = DriveReportAccumulator()
  report.update(message("carState", 2.0, car_state(12.0, -3.0)))
  result = report.finalize([])
  assert result["moments"][0]["time"] is None
  assert result["dataQuality"]["timestampAvailable"] is False


@pytest.mark.parametrize("gap,located", [(-120, False), (-2.001, False), (-2, True), (0, True), (2, True), (2.001, False), (120, False)])
def test_route_location_requires_a_nearby_fix_for_moments_and_blackbox(gap, located):
  origin = datetime(2026, 10, 28, tzinfo=UTC)
  report = DriveReportAccumulator(origin, route_first_mono=100)
  report.update(message("carState", 100, car_state(10, -3)))
  report.update(message("selfdriveState", 100, SimpleNamespace(active=True, enabled=True, alertType="commIssue/warning",
                                                              alertStatus="userPrompt")))
  result = report.finalize([{"time": (origin + timedelta(seconds=gap)).isoformat(), "latitude": 37.0, "longitude": 127.0}])
  assert result["dataQuality"]["routeLocationMaxGapS"] == ROUTE_LOCATION_MAX_GAP_S
  assert result["moments"] and result["blackbox"]["events"]
  for moment in [*result["moments"], *result["blackbox"]["events"]]:
    assert ("latitude" in moment) == located
    assert ("longitude" in moment) == located
    assert moment["locationContext"]["status"] == ("approximate" if located else "unavailable")
    if located:
      assert moment["latitude"] == 37.0 and moment["longitude"] == 127.0
      assert moment["locationContext"]["timeDeltaS"] == gap


@pytest.mark.parametrize("point", [
  {"latitude": None, "longitude": 127}, {"latitude": float("nan"), "longitude": 127},
  {"latitude": 91, "longitude": 127}, {"latitude": 37, "longitude": 181},
  {"latitude": True, "longitude": 127}, {"latitude": 37, "longitude": "invalid"},
  {"latitude": 37, "longitude": 127, "time": "2026-10-28T00:00:00"},
])
def test_invalid_route_fix_does_not_invent_a_location(point):
  origin = datetime(2026, 10, 28, tzinfo=UTC)
  report = DriveReportAccumulator(origin, route_first_mono=0)
  report.update(message("carState", 0, car_state(10, -3)))
  moment = report.finalize([{"time": origin.isoformat(), **point}])["moments"][0]
  assert "latitude" not in moment and "longitude" not in moment


def test_finalize_does_not_retain_location_from_an_old_route():
  origin = datetime(2026, 10, 28, tzinfo=UTC)
  report = DriveReportAccumulator(origin, route_first_mono=0)
  report.update(message("carState", 0, car_state(10, -3)))
  assert report.finalize([{"time": origin.isoformat(), "latitude": 37, "longitude": 127}])["moments"][0]["latitude"] == 37
  assert "latitude" not in report.finalize([])["moments"][0]


@pytest.mark.parametrize("policy, disabled, camera_unavailable", [("wheeltouch", False, False), ("vision", True, False), ("vision", False, True)])
def test_no_vision_observation_is_not_a_perfect_attention_score(policy, disabled, camera_unavailable):
  report = DriveReportAccumulator()
  for index in range(30):
    report.update(message("driverMonitoringState", index * 0.5, SimpleNamespace(
      activePolicy=policy, dm2Disabled=disabled, cameraUnavailable=camera_unavailable,
      visionPolicyState=SimpleNamespace(faceDetected=False, isDistracted=False), alertLevel="none")))
  result = report.finalize([])
  assert result["scores"]["attention"] is None
  assert result["attention"]["observedS"] == 0


def test_observed_attention_keeps_original_scoring():
  report = DriveReportAccumulator()
  for index in range(30):
    report.update(message("driverMonitoringState", index * 0.5, SimpleNamespace(
      activePolicy="vision", visionPolicyState=SimpleNamespace(faceDetected=True, isDistracted=True), alertLevel="none")))
  result = report.finalize([])
  assert result["attention"]["distractedPercent"] == 100
  assert result["scores"]["attention"] == 0


def test_invalid_duplicate_reversed_and_nonfinite_samples_are_rejected():
  report = DriveReportAccumulator()
  report.update(message("carState", 1, car_state(10)))
  report.update(message("carState", 1, car_state(10, -3)))
  report.update(message("carState", 0, car_state(10, -3)))
  invalid = message("carState", 2, car_state(10, -3))
  invalid.valid = False
  report.update(invalid)
  report.update(message("carState", 3, car_state(float("nan"), -3)))
  result = report.finalize([])
  assert result["dataQuality"]["rejectedSamples"] == 4
  assert result["comfort"]["hardBrakingCount"] == 0


def test_timeline_storage_is_bounded_during_accumulation():
  report = DriveReportAccumulator()
  for index in range(TIMELINE_LIMIT + 20):
    report.update(message("carState", index * 5.0, car_state(10)))
  assert len(report.timeline) == TIMELINE_LIMIT
  assert report.finalize([])["dataQuality"]["timelineTruncated"] is True


def test_actual_carrot_schema_without_sp_and_bounded_qlog(tmp_path):
  from openpilot.cereal import log
  from openpilot.system.wayon_event_blackbox import WorkLimits, WorkAborted
  assert "longitudinalPlanSP" not in log.Event.schema.fields
  directory = tmp_path / "route--0"
  directory.mkdir()
  events = []
  for index in range(20):
    mono = int((100 + index * 0.1) * 1e9)
    for which, payload in (
      ("selfdriveState", {"active": True, "enabled": True}),
      ("carControl", {"longActive": True, "actuators": {"accel": 1.0}}),
      ("longitudinalPlan", {"aTarget": -0.5, "shouldStop": False}),
      ("carState", {"vEgo": 10.0, "aEgo": 0.5}),
    ):
      events.append(log.Event.new_message(logMonoTime=mono, valid=True, **{which: payload}).to_bytes())
  qlog = directory / "qlog"
  qlog.write_bytes(b"".join(events))
  group = {"route_name": "route", "segments": [(0, directory, qlog)]}
  result = build_route_analysis(group, [], route_first_mono=100, allowed=lambda: True)
  assert result["schemaVersion"] == "wayon-drive-report-v2"
  assert result["dataQuality"]["source"] == "sampled_qlog"
  assert result["blackbox"]["events"][0]["type"] == "unexplained_acceleration"
  with pytest.raises(WorkAborted):
    build_route_analysis(group, [], allowed=lambda: False)
  with pytest.raises(WorkAborted):
    build_route_analysis(group, [], allowed=lambda: True, limits=WorkLimits(max_messages=2))
