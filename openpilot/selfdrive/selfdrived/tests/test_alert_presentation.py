"""Dependency-isolated execution of the real event/presentation/label code.

Run with unittest on hosts without native cereal/raylib. Enum IDs are read from
the checked-in schema; this does not replace a native Cap'n Proto/UI test.
"""
import ast
import bisect
import copy
import importlib.util
import math
import os
import re
import unittest
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.selfdrived import alert_presentation as presentation

ROOT = Path(__file__).resolve().parents[3]
EVENTS_PATH = ROOT / "selfdrive/selfdrived/events.py"


class Messages(dict):
  @property
  def data(self):
    return self

  def all_checks(self, services):
    return not any(s in ("roadCameraState", "modelV2") for s in services)


def callback_args(soft_time=100, metric=True):
  sm = Messages(
    liveCalibration=SimpleNamespace(calStatus=0, calPerc=87, rpyCalib=[0, .02, -.03]),
    liveParameters=SimpleNamespace(angleOffsetValid=False, angleOffsetDeg=2.5, steerRatioValid=True,
                                   steerRatio=15.0, stiffnessFactorValid=True, stiffnessFactor=1.0),
    deviceState=SimpleNamespace(memoryUsagePercent=93, cpuTempC=[76], gpuTempC=[72], memoryTempC=65,
                               freeSpacePercent=12, cpuUsagePercent=[22, 45]),
    carControl=SimpleNamespace(actuators=SimpleNamespace(accel=-2., torque=.25)),
    modelV2=SimpleNamespace(frameDropPerc=12.3, velocity=SimpleNamespace(x=[10.])),
    audioFeedback=SimpleNamespace(blockNum=1),
    managerState=SimpleNamespace(processes=[SimpleNamespace(name="testd", running=False, shouldBeRunning=True)]),
    alertDebug=SimpleNamespace(alertText1="Active maneuver", alertText2="Keep clear"),
    roadCameraState=SimpleNamespace(), driverCameraState=SimpleNamespace(), wideRoadCameraState=SimpleNamespace(),
  )
  return [SimpleNamespace(minSteerSpeed=10., minEnableSpeed=5., brand="mock", flags=0),
          SimpleNamespace(vEgo=10.), sm, metric, soft_time, "standard"]


def load_events(device="mici"):
  tree = ast.parse(EVENTS_PATH.read_text())
  schema = (ROOT / "cereal/log.capnp").read_text().split("struct OnroadEvent", 1)[1].split("enum EventName", 1)[1].split("}", 1)[0]
  enumerants = {name: int(value) for name, value in re.findall(r"^\s+(\w+)\s+@(\d+);", schema, re.M)}
  assert len(enumerants) == len(set(enumerants.values()))
  enum = SimpleNamespace(**enumerants, schema=SimpleNamespace(enumerants=enumerants))
  ns = {
    "__name__": "isolated_events", "bisect": bisect, "copy": copy, "math": math, "os": os, "IntEnum": IntEnum,
    "EventName": enum, "DT_CTRL": .01, "MIN_SPEED_FILTER": 15 / 3.6, "FEEDBACK_MAX_DURATION": 60,
    "SAMPLE_BUFFER": 48000, "SAMPLE_RATE": 48000,
    "HARDWARE": SimpleNamespace(get_device_type=lambda: device),
    "tr": lambda text: text, "tr_noop": lambda text: text, "trn": lambda a, b, count: a if count == 1 else b,
    "get_short_branch": lambda: "test", "Params": lambda: {"NNFFModelName": "TEST CAR"},
    "CV": SimpleNamespace(MS_TO_KPH=3.6, MS_TO_MPH=2.236936), "alert_presentation": presentation,
    "log": SimpleNamespace(LiveCalibrationData=SimpleNamespace(Status=SimpleNamespace(recalibrating=1))),
  }
  for name, members in {
    "AlertSize": ["none", "small", "mid", "full"], "AlertStatus": ["normal", "userPrompt", "critical"],
    "VisualAlert": sorted({n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute) and
                           isinstance(n.value, ast.Name) and n.value.id == "VisualAlert"}),
    "AudibleAlert": sorted({n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute) and
                            isinstance(n.value, ast.Name) and n.value.id == "AudibleAlert"}),
  }.items():
    ns[name] = SimpleNamespace(**{name: i for i, name in enumerate(members)})
  if importlib.util.find_spec("capnp") is not None:
    from openpilot.cereal import log, car
    assert enumerants == log.OnroadEvent.EventName.schema.enumerants
    ns.update(log=log, car=car, EventName=log.OnroadEvent.EventName,
              AlertSize=log.SelfdriveState.AlertSize, AlertStatus=log.SelfdriveState.AlertStatus,
              VisualAlert=car.CarControl.HUDControl.VisualAlert, AudibleAlert=car.CarControl.HUDControl.AudibleAlert)
  excluded = {"AlertSize", "AlertStatus", "VisualAlert", "AudibleAlert", "EventName", "AlertCallbackType"}
  body = [ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
  for node in tree.body:
    if isinstance(node, (ast.Import, ast.ImportFrom)):
      continue
    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in excluded for t in node.targets):
      continue
    body.append(node)
  exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(EVENTS_PATH), "exec"), ns)
  return SimpleNamespace(**ns)


def supported_configs(module):
  kinds = presentation.EVENT_TYPE_JSON_NAME
  supported = {f"EVENTS.{module.EVENT_NAME[event]}.{kinds[kind]}"
               for event, alerts in module.EVENTS.items() for kind in alerts}
  return {key: config for key, config in presentation.load_overrides().items() if key in supported}


class TestAlertPresentation(unittest.TestCase):
  def setUp(self):
    self.module = load_events()

  def test_every_alert_keeps_policy_and_source_objects(self):
    m = self.module
    snapshots = {id(alert): vars(alert).copy() for alerts in m.EVENTS.values() for alert in alerts.values()
                 if isinstance(alert, m.Alert)}
    for soft_time in (0, 49, 50, 100):
      for metric in (True, False):
        for event, kinds in m.EVENTS.items():
          with self.subTest(event=m.EVENT_NAME[event], soft_time=soft_time, metric=metric):
            ev = m.Events()
            ev.add(event)
            ev.event_counters[event] = 10000
            ev._steer_warning_fault_frames = m.TEMP_STEER_WARNING_DELAY_FRAMES
            args = callback_args(soft_time, metric)
            with patch.object(presentation, "apply_text", side_effect=lambda alert, *args, **kwargs: alert):
              baseline = ev.create_alerts(list(kinds), args)
            decorated = ev.create_alerts(list(kinds), args)
            self.assertEqual(len(baseline), len(decorated))
            for old, new in zip(baseline, decorated, strict=True):
              metadata = lambda a: {k: v for k, v in vars(a).items() if k not in ("alert_text_1", "alert_text_2")}
              self.assertEqual(metadata(old), metadata(new))
        for alerts in m.EVENTS.values():
          for alert in alerts.values():
            if isinstance(alert, m.Alert):
              self.assertEqual(vars(alert), snapshots[id(alert)])

  def test_non_mici_never_applies_overrides(self):
    m = load_events("tici")
    ev = m.Events()
    ev.add(m.EventName.lowMemory)
    with patch.object(presentation, "apply_text", side_effect=AssertionError("non-Mici presentation")):
      alerts = ev.create_alerts([m.ET.PERMANENT], callback_args())
    self.assertEqual(alerts[0].alert_text_1, "Low Memory")

  def test_traffic_stopping_is_silent_visible_and_warning_only(self):
    for device in ("mici", "tici"):
      with self.subTest(device=device):
        m = load_events(device)
        ev = m.Events()
        ev.add(m.EventName.trafficStopping)
        self.assertEqual(set(m.EVENTS[m.EventName.trafficStopping]), {m.ET.WARNING})
        self.assertEqual(ev.create_alerts([m.ET.PERMANENT], callback_args()), [])
        alert, = ev.create_alerts([m.ET.WARNING], callback_args())
        self.assertEqual((alert.alert_text_1, alert.alert_text_2), ("정지합니다", "전방 신호 감지됨."))
        self.assertEqual(alert.alert_type, "trafficStopping/warning")
        self.assertEqual(alert.alert_status, m.AlertStatus.normal)
        self.assertEqual(alert.alert_size, m.AlertSize.small)
        self.assertEqual(alert.priority, m.Priority.LOW)
        self.assertEqual(alert.visual_alert, m.VisualAlert.none)
        self.assertEqual(alert.audible_alert, m.AudibleAlert.none)
        self.assertEqual(alert.duration, int(2. / m.DT_CTRL))
        self.assertEqual(alert.creation_delay, 0.)

  def test_traffic_stop_notice_does_not_replace_safety_or_departure_sounds(self):
    m = self.module
    stopping = m.EVENTS[m.EventName.trafficStopping][m.ET.WARNING]
    for event in (m.EventName.fcw, m.EventName.aeb, m.EventName.stockAeb):
      alert = m.EVENTS[event][m.ET.PERMANENT]
      self.assertGreater(alert.priority, stopping.priority)
      self.assertEqual(alert.audible_alert, m.AudibleAlert.stopStop)
    for event, sound in ((m.EventName.trafficSignGreen, m.AudibleAlert.trafficSignGreen),
                         (m.EventName.trafficSignChanged, m.AudibleAlert.trafficSignChanged),
                         (m.EventName.stopStop, m.AudibleAlert.stopStop)):
      self.assertEqual(m.EVENTS[event][m.ET.WARNING].audible_alert, sound)

  def test_cutin_notice_has_text_and_original_sound_without_control_events(self):
    for device in ("mici", "tici"):
      with self.subTest(device=device):
        m = load_events(device)
        ev = m.Events()
        ev.add(m.EventName.radarCutin)
        self.assertEqual(set(m.EVENTS[m.EventName.radarCutin]), {m.ET.WARNING})
        alert, = ev.create_alerts([m.ET.WARNING], callback_args())
        self.assertEqual((alert.alert_text_1, alert.alert_text_2), ("끼어드는 차량 감지", "전방 차량을 확인하세요"))
        self.assertEqual(alert.alert_type, "radarCutin/warning")
        self.assertEqual(alert.alert_status, m.AlertStatus.normal)
        self.assertEqual(alert.alert_size, m.AlertSize.small)
        self.assertEqual(alert.visual_alert, m.VisualAlert.none)
        self.assertEqual(alert.audible_alert, m.AudibleAlert.radarCutin)
        self.assertEqual(alert.duration, int(2. / m.DT_CTRL))
        self.assertEqual(alert.creation_delay, 0.)
        self.assertEqual(ev.create_alerts([m.ET.NO_ENTRY, m.ET.SOFT_DISABLE, m.ET.IMMEDIATE_DISABLE], callback_args()), [])
        for event in (m.EventName.fcw, m.EventName.aeb, m.EventName.stockAeb,
                      m.EventName.driverDistracted1, m.EventName.driverDistracted2, m.EventName.driverDistracted3,
                      m.EventName.driverUnresponsive1, m.EventName.driverUnresponsive2, m.EventName.driverUnresponsive3):
          self.assertLess(alert.priority, m.EVENTS[event][m.ET.PERMANENT].priority)

  def test_steering_delay_and_critical_bypass_unchanged(self):
    m = self.module
    ev = m.Events()
    for _ in range(m.TEMP_STEER_WARNING_DELAY_FRAMES):
      ev.add(m.EventName.steerTempUnavailable)
      self.assertEqual(ev.create_alerts([m.ET.SOFT_DISABLE], callback_args()), [])
      self.assertEqual(len(ev.create_alerts([m.ET.NO_ENTRY], callback_args())), 1)
      critical, = ev.create_alerts([m.ET.SOFT_DISABLE], callback_args(0))
      self.assertEqual(critical.alert_status, m.AlertStatus.critical)
      self.assertEqual(critical.alert_text_1, "즉시 제어하세요")
      ev.clear()
    ev.add(m.EventName.steerTempUnavailable)
    self.assertEqual(len(ev.create_alerts([m.ET.SOFT_DISABLE], callback_args())), 1)

  def test_explicit_dm_schema_mapping_and_legacy_keys(self):
    for (name, kind), (old_name, old_kind) in presentation.LEGACY_DM_KEYS.items():
      event = self.module.EventName.schema.enumerants[name]
      self.assertIn(kind, self.module.EVENTS[event])
      config = {"source": "EVENTS", "title": "legacy"}
      with patch.object(presentation, "load_overrides", return_value={f"EVENTS.{old_name}.{old_kind}": config}):
        self.assertEqual(presentation.get_override(name, kind), config)
    self.assertEqual(presentation.get_override("driverDistracted2", "permanent")["titleFontPx"], 60)

  def test_asset_counts_and_unsupported_entries_are_not_events(self):
    self.assertEqual(len(presentation.load_overrides()), 111)
    supported = supported_configs(self.module)
    unsupported = set(presentation.load_overrides()) - set(supported)
    self.assertTrue(all(not key.startswith("STARPILOT_EVENTS.") for key in supported))
    self.assertIn("EVENTS.laneChangeUnavailable.WARNING", unsupported)
    self.assertNotIn("laneChangeUnavailable", self.module.EventName.schema.enumerants)

  def test_dynamic_fields_and_missing_values(self):
    args = callback_args()
    fields = frozenset(field for key in supported_configs(self.module) for field in
                       presentation.format_fields(key.split('.')[1], next(k for k, v in presentation.EVENT_TYPE_JSON_NAME.items()
                                                                          if v == key.split('.')[2])))
    values = presentation.format_values(args, display_speed=self.module.get_display_speed, min_speed_filter=15 / 3.6,
                                       feedback_duration=60, sample_buffer=48000, sample_rate=48000, fields=fields)
    self.assertEqual(values["minSteerSpeed"], "36 km/h")
    self.assertEqual(values["memoryUsagePercent"], "93")
    self.assertEqual(values["pitch"], "1.1")
    self.assertEqual(values["yaw"], "-1.7")
    self.assertEqual(values["seconds"], "58")
    self.assertEqual(values["accel"], "-50")
    self.assertEqual(values["steer"], "25")
    self.assertEqual(values["bad camera list"], "roadCamera")
    for config in supported_configs(self.module).values():
      for key in ("title", "subtitle"):
        self.assertNotIn("{", presentation.format_text(config[key], "original", values))
    self.assertEqual(presentation.format_text("{missing}", "original", {}), "original")
    self.assertEqual(presentation.format_text("{broken", "original", {}), "original")
    self.assertEqual(presentation.format_text(None, "original", {}), "original")
    self.assertEqual(presentation.format_values([], display_speed=None, min_speed_filter=0, feedback_duration=0,
                                               sample_buffer=0, sample_rate=0, fields=fields), {})

  def test_parameter_diagnostics_keep_non_angle_faults(self):
    m = self.module
    ev = m.Events()
    ev.add(m.EventName.paramsdTemporaryError)
    args = callback_args()
    args[2]['liveParameters'].angleOffsetValid = True
    args[2]['liveParameters'].steerRatioValid = False
    alert, = ev.create_alerts([m.ET.NO_ENTRY], args)
    self.assertIn("Ratio: 15.0", alert.alert_text_1 + alert.alert_text_2)
    self.assertNotIn("Offset", alert.alert_text_1 + alert.alert_text_2)

  def test_can_parser_diagnostics_are_not_erased(self):
    alert = SimpleNamespace(alert_text_1="CAN Error: Check Connections!!", alert_text_2="missing message 123")
    presentation.apply_text(alert, "canError", "permanent", {})
    self.assertEqual(alert.alert_text_2, "missing message 123")
    alert = SimpleNamespace(alert_text_1="CAN Error: Enable CameraSCC", alert_text_2="SCC detected on camera bus")
    presentation.apply_text(alert, "canError", "permanent", {})
    self.assertEqual(alert.alert_text_1, "CAN Error: Enable CameraSCC")

  def test_malformed_asset_is_optional_and_loaded_once(self):
    presentation.load_overrides.cache_clear()
    with patch.object(Path, "read_text", return_value="[]") as read:
      self.assertEqual(presentation.load_overrides(), {})
      self.assertEqual(presentation.load_overrides(), {})
      self.assertEqual(read.call_count, 1)
    presentation.load_overrides.cache_clear()

  def test_configured_font_sizes_are_preserved_before_fitting(self):
    self.assertEqual(presentation.font_sizes("steerSaturated", "warning", "title", "subtitle", False), (66, 47))
    self.assertEqual(presentation.font_sizes("preLaneChangeLeft", "warning", "title", "subtitle", True)[0], 62)
    self.assertEqual(presentation.font_sizes("unsupported", "warning", "short", "subtitle", False), (82, 40))

  @unittest.skipUnless(importlib.util.find_spec("capnp"), "native pycapnp not available")
  def test_native_event_serialization_keeps_flags(self):
    m = self.module
    ev = m.Events()
    for event in m.EVENTS:
      ev.add(event)
    for msg in ev.to_msg():
      kinds = m.EVENTS[msg.name.raw]
      for kind in presentation.EVENT_TYPE_JSON_NAME:
        self.assertEqual(getattr(msg, kind), kind in kinds)

  @unittest.skipUnless(importlib.util.find_spec("capnp"), "native pycapnp not available")
  def test_native_alert_roundtrip(self):
    m = self.module
    for event, kinds in m.EVENTS.items():
      ev = m.Events()
      ev.add(event)
      ev.event_counters[event] = 10000
      ev._steer_warning_fault_frames = m.TEMP_STEER_WARNING_DELAY_FRAMES
      for alert in ev.create_alerts(list(kinds), callback_args()):
        msg = m.log.SelfdriveState.new_message(
          alertText1=alert.alert_text_1, alertText2=alert.alert_text_2, alertType=alert.alert_type,
          alertStatus=alert.alert_status, alertSize=alert.alert_size, alertSound=alert.audible_alert,
          alertHudVisual=alert.visual_alert)
        with m.log.SelfdriveState.from_bytes(msg.to_bytes()) as restored:
          self.assertEqual(restored.alertText1, alert.alert_text_1)
          self.assertEqual(restored.alertText2, alert.alert_text_2)
          self.assertEqual(restored.alertType, alert.alert_type)
          self.assertEqual(restored.alertStatus.raw, alert.alert_status)
          self.assertEqual(restored.alertSize.raw, alert.alert_size)
          self.assertEqual(restored.alertSound.raw, alert.audible_alert)
          self.assertEqual(restored.alertHudVisual.raw, alert.visual_alert)


if __name__ == "__main__":
  unittest.main()
