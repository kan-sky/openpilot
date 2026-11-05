import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.xiaoge.xiaoge_vision import (
  XIAOGE_LANE_TIMEOUT_NS, XiaogeVisionResult, apply_xiaoge_vision_result,
)
from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy import bluetooth as bt
from openpilot.selfdrive.navdy import bluetooth_pipeline as pipe
from openpilot.selfdrive.navdy.display import navdy_lead_geometry, navdy_model_geometry
from openpilot.selfdrive.navdy.tests.test_bluetooth_pipeline import Wire
from openpilot.selfdrive.navdy.tests.test_bridge import Messages
from openpilot.selfdrive.navdy.tests.test_display import car, lead, model, payload, selfdrive
from openpilot.selfdrive.navdy.transport import encode_frame


STYLE_KEYS = ("navLaneLeftType", "navLaneRightType")


def pre_style_payload(*, model, **inputs):
  # Historical-commit replay is checked separately, without a Git dependency here.
  result = payload(**inputs)
  if result["active"]:
    geometry = navdy_model_geometry(model)
    result.update(geometry)
    if geometry:
      result.update(navdy_lead_geometry(model, inputs.get("radar_state"), inputs.get("longitudinal_plan")))
  return result


def styles(result):
  return tuple(result[key] for key in STYLE_KEYS)


@pytest.mark.parametrize("color", [0, 10, 20, 30])
@pytest.mark.parametrize("marking", range(9))
def test_documented_vehicle_codes_match_c4_without_center_prefix(color, marking):
  code = color + marking
  result = payload(car_state=car(leftLaneLine=code, rightLaneLine=code), model=model())
  expected = "dashed" if marking == 0 else "solid"
  assert styles(result) == (expected, expected)
  assert all(type(result[key]) is str for key in STYLE_KEYS)


@pytest.mark.parametrize("bad", [
  -1, -2, -32768, 9, 19, 29, 39, 40, 99, 32767, 32768, 10**100,
  None, True, False, 0.0, 1.0, 20.0, 20.5, math.nan, math.inf, -math.inf,
  "0", "20", "solid", b"0", [], {}, object(),
])
def test_invalid_or_reserved_code_is_unknown_without_changing_other_side(bad):
  result = payload(car_state=car(leftLaneLine=bad, rightLaneLine=21), model=model())
  assert styles(result) == ("unknown", "solid")
  encode_frame(result)


@pytest.mark.parametrize("probability,expected", [
  (0, "unknown"), (0.1, "unknown"), (math.nextafter(0.3, 0.0), "unknown"),
  (0.3, "unknown"), (math.nextafter(0.3, 1.0), "dashed"), (0.301, "dashed"),
  (1, "dashed"), (1.0, "dashed"), (math.nextafter(1.0, 2.0), "unknown"),
  (-0.1, "unknown"), (2, "unknown"), (10**100, "unknown"),
  (math.nan, "unknown"), (math.inf, "unknown"), (-math.inf, "unknown"),
  (None, "unknown"), (True, "unknown"), (False, "unknown"),
  ("0.9", "unknown"), (b"0.9", "unknown"), ([], "unknown"), ({}, "unknown"),
])
def test_raw_probability_is_validated_before_rounding_or_clamping(probability, expected):
  m = model()
  m.laneLineProbs = [1.0, probability, 0.9, 1.0]
  result = payload(car_state=car(leftLaneLine=20, rightLaneLine=11), model=m)
  assert styles(result) == (expected, "solid")
  assert result["navLaneLeftProb"] == navdy_model_geometry(m)["navLaneLeftProb"]
  encode_frame(result)


@pytest.mark.parametrize("probs,expected", [
  (None, ("unknown", "unknown")), ([], ("unknown", "unknown")),
  ([1.0], ("unknown", "unknown")), ([1.0, 0.9], ("dashed", "unknown")),
  ([1.0, 0.3, 0.9], ("unknown", "solid")), ([0.0, 0.9, 0.9, 0.0], ("dashed", "solid")),
])
def test_only_corresponding_inner_lane_probability_grants_style(probs, expected):
  m = model()
  m.laneLineProbs = probs
  result = payload(car_state=car(leftLaneLine=10, rightLaneLine=31), model=m)
  assert styles(result) == expected


def test_missing_fields_send_explicit_unknown_for_existing_renderer_fallback():
  m = model()
  assert styles(payload(car_state=car(), model=m)) == ("unknown", "unknown")
  assert styles(payload(car_state=car(leftLaneLine=0), model=m)) == ("dashed", "unknown")
  assert styles(payload(car_state=car(rightLaneLine=1), model=m)) == ("unknown", "solid")
  del m.laneLineProbs
  assert styles(payload(car_state=car(leftLaneLine=0, rightLaneLine=1), model=m)) == ("unknown", "unknown")


@pytest.mark.parametrize("overrides", [
  {"onroad": False}, {"car_state": None}, {"selfdrive_state": None},
  {"car_state": car(canValid=False)}, {"car_state": car(canTimeout=True)},
  {"car_state": car(vEgo=math.nan)}, {"car_state": car(gearShifter="reverse")},
  {"selfdrive_state": selfdrive(enabled=False, active=True)},
  {"selfdrive_state": selfdrive(active=False)},
])
def test_inactive_invalid_or_absent_critical_input_omits_styles(overrides):
  inputs = {"car_state": car(leftLaneLine=0, rightLaneLine=1), "model": model()} | overrides
  result = payload(**inputs)
  assert not result["active"]
  assert not any(key in result for key in STYLE_KEYS)
  assert "navLaneLeft" not in result and "navLaneRight" not in result


@pytest.mark.parametrize("broken", ["absent", "empty", "path", "left", "right", "lanes"])
def test_active_without_valid_geometry_omits_styles(broken):
  m = model()
  if broken == "absent":
    m = None
  elif broken == "empty":
    m = NS()
  elif broken == "lanes":
    m.laneLines = m.laneLines[:2]
  else:
    line = m.position if broken == "path" else m.laneLines[1 if broken == "left" else 2]
    line.y = [math.nan] * len(line.x)
  result = payload(car_state=car(leftLaneLine=0, rightLaneLine=1), model=m)
  assert result["active"]
  assert not any(key in result for key in STYLE_KEYS)
  assert "navLaneLeft" not in result and "navLaneRight" not in result


def test_style_changes_do_not_retain_previous_marking():
  cs, m = car(leftLaneLine=0, rightLaneLine=1), model()
  assert styles(payload(car_state=cs, model=m)) == ("dashed", "solid")
  cs.leftLaneLine, cs.rightLaneLine = 21, -1
  assert styles(payload(car_state=cs, model=m)) == ("solid", "unknown")
  m.laneLineProbs[1] = 0.3
  assert styles(payload(car_state=cs, model=m)) == ("unknown", "unknown")


@pytest.mark.parametrize("service", ["carState", "selfdriveState", "modelV2"])
@pytest.mark.parametrize("clock", ["recv_time", "logMonoTime"])
@pytest.mark.parametrize("age", [0.499, 0.5, 0.501])
def test_caller_freshness_deadline_also_gates_styles(monkeypatch, service, clock, age):
  monkeypatch.setattr(bridge.time, "monotonic", lambda: 100.0)
  sm = Messages()
  sm.data["carState"] = car(leftLaneLine=20, rightLaneLine=14)
  assert styles(bridge.live_payload(sm, 1)) == ("dashed", "solid")
  getattr(sm, clock)[service] = 100.0 - age if clock == "recv_time" else round((100.0 - age) * 1e9)
  result = bridge.live_payload(sm, 2)
  if age < bridge.DISPLAY_MAX_AGE:
    assert styles(result) == ("dashed", "solid")
  else:
    assert not any(key in result for key in STYLE_KEYS)
    if service == "modelV2":
      assert result["active"] and result["navLaneLeft"] == result["navLaneRight"] == []
    else:
      assert not result["active"]


@pytest.mark.parametrize("service", ["carState", "selfdriveState", "modelV2"])
@pytest.mark.parametrize("flag", ["seen", "valid"])
def test_caller_invalid_or_unseen_service_omits_styles(monkeypatch, service, flag):
  monkeypatch.setattr(bridge.time, "monotonic", lambda: 100.0)
  sm = Messages()
  sm.data["carState"] = car(leftLaneLine=0, rightLaneLine=1)
  getattr(sm, flag)[service] = False
  assert not any(key in bridge.live_payload(sm, 1) for key in STYLE_KEYS)


@pytest.mark.parametrize("age,expected", [
  (0, ("dashed", "solid")), (XIAOGE_LANE_TIMEOUT_NS, ("dashed", "solid")),
  (XIAOGE_LANE_TIMEOUT_NS + 1, ("solid", "solid")),
])
def test_actual_vision_merge_preserves_color_and_owns_its_timeout(age, expected):
  cs = car(leftLaneLine=24, rightLaneLine=14)
  vision = XiaogeVisionResult(0, 1, True, 1, False, False, False, 0)
  apply_xiaoge_vision_result(cs, vision, 1 + age)
  assert styles(payload(car_state=cs, model=model())) == expected
  assert (cs.leftLaneLine // 10, cs.rightLaneLine // 10) == (2, 1)


def test_real_capnp_fields_round_trip_into_string_styles():
  from opendbc.car import structs
  from openpilot.cereal import log

  cs = structs.CarState.new_message(canValid=True, canTimeout=False, vEgo=20.0,
                                    gearShifter="drive", leftLaneLine=24, rightLaneLine=10)
  m = model(samples=2)
  md = log.ModelDataV2.new_message(position=vars(m.position), laneLines=[vars(line) for line in m.laneLines],
                                  laneLineProbs=m.laneLineProbs)
  with structs.CarState.from_bytes(cs.to_bytes()) as car_reader, log.ModelDataV2.from_bytes(md.to_bytes()) as model_reader:
    assert styles(payload(car_state=car_reader, model=model_reader)) == ("solid", "dashed")


@pytest.mark.parametrize("stock,expected", [
  ((1, 0), ("solid", "dashed")), ((0, 1), ("dashed", "solid")),
  ((-1, 1), ("unknown", "solid")), ((-1, -1), ("unknown", "unknown")),
])
def test_stock_lanes_override_onnx_on_existing_wire_fields_only(stock, expected):
  cs = car(leftLaneLine=0, rightLaneLine=1)
  m = model()
  previous = payload(car_state=cs, model=m)
  cs.stockLaneMarkings = NS(supported=True, left=stock[0], right=stock[1])
  result = payload(car_state=cs, model=m)
  assert styles(result) == expected
  assert {k: v for k, v in result.items() if k not in STYLE_KEYS} == {k: v for k, v in previous.items() if k not in STYLE_KEYS}
  encode_frame(result)


@pytest.mark.parametrize("codes", [(0, 1), (20, 24), (-1, -1), (True, "0")])
def test_all_other_payload_values_order_and_wire_bytes_are_unchanged(codes):
  m = model()
  m.laneLineProbs[1] = 0.301
  inputs = {
    "car_state": car(leftLaneLine=codes[0], rightLaneLine=codes[1]),
    "selfdrive_state": selfdrive(alertText1="Original \uacbd\uace0", alertText2="Keep this text"),
    "controls_state": NS(activeLaneLine=True), "car_control": NS(latActive=True, longActive=True),
    "car_output": NS(actuatorsOutput=NS(accel=-0.8)), "radar_state": NS(leadOne=lead()),
    "longitudinal_plan": NS(longitudinalPlanSource="lead0"), "seq": 42,
  }
  original = copy.deepcopy((inputs, m))
  expected = pre_style_payload(model=m, **inputs)
  result = payload(model=m, **inputs)
  without_styles = {key: value for key, value in result.items() if key not in STYLE_KEYS}
  assert set(result) - set(expected) == set(STYLE_KEYS)
  assert encode_frame(without_styles) == encode_frame(expected)
  assert (inputs, m) == original


@pytest.mark.parametrize("left", [-1, 0, 1])
@pytest.mark.parametrize("right", [-1, 0, 1])
@pytest.mark.parametrize("samples", [2, 33])
def test_bluetooth_v2_compressed_round_trip_and_bounded_overhead(monkeypatch, left, right, samples):
  now_ns = 10_000_000_000
  monkeypatch.setattr(pipe.time, "monotonic_ns", lambda: now_ns)
  inputs = {"car_state": car(leftLaneLine=left, rightLaneLine=right), "selfdrive_state": selfdrive(),
            "model": model(samples=samples), "onroad": True}
  current = payload(**inputs)
  previous = pre_style_payload(**inputs)
  assert encode_frame({key: value for key, value in current.items() if key not in STYLE_KEYS}) == encode_frame(previous)
  headers, frames = [], []
  for value in (previous, current):
    wire = Wire()
    pipeline = pipe.SnapshotPipeline(wire, now_ns, reader_factory=lambda conn: pipe.ResponseReader(conn, threaded=False))
    try:
      assert pipeline.update(value, captured_ns=now_ns) == []
      frame = bytes(wire.output)
      head = bt.REQUEST.unpack(frame[:bt.REQUEST.size])
      assert head[:4] == (b"NVQ2", 0, 0, 0)
      assert head[7] == 1 and len(frame) == bt.REQUEST.size + head[5]
      raw = bt.unpack_payload(frame[bt.REQUEST.size:], head[4], head[7], head[6])
      assert raw == encode_frame(value)[:-1]
      assert json.loads(raw) == value
      headers.append(head)
      frames.append(frame)
    finally:
      pipeline.close()
  assert 53 <= headers[1][4] - headers[0][4] <= 57
  assert len(frames[1]) - len(frames[0]) <= 40
