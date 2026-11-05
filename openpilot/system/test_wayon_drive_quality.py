import unittest

from openpilot.system.wayon_drive_quality import (
  StopQualityTracker,
  cutin_risk_stage,
  evaluate_route_report,
  resolve_operating_state,
)


class TestWayonDriveQuality(unittest.TestCase):
  def test_cutin_stages(self):
    self.assertEqual(cutin_risk_stage(0.1)["name"], "clear")
    self.assertEqual(cutin_risk_stage(0.4)["name"], "prepare")
    self.assertEqual(cutin_risk_stage(0.6, 6.0, -4.0)["name"], "brake")

  def test_operating_state_priority(self):
    self.assertEqual(resolve_operating_state(connected=False, onroad=True), "disconnected")
    self.assertEqual(resolve_operating_state(connected=True, onroad=True, gear="reverse"), "reverse")
    self.assertEqual(resolve_operating_state(connected=True, onroad=True, overspeed=True), "overspeed")
    self.assertEqual(resolve_operating_state(connected=True, onroad=False, door_open=True), "offroad_door")

  def test_stop_quality(self):
    tracker = StopQualityTracker()
    speeds = [4.0, 3.2, 2.3, 1.4, 0.7, 0.2, 0.05, 0.0, 0.0, 0.0]
    for index, speed in enumerate(speeds):
      tracker.update(index * 0.1, speed, -0.7, speed == 0.0)
    summary = tracker.summary()
    self.assertEqual(summary["count"], 1)
    self.assertGreaterEqual(summary["score"], 70)

  def test_regression_failure(self):
    result = evaluate_route_report({"stopQuality": {"count": 2, "score": 55}})
    self.assertFalse(result["passed"])


if __name__ == "__main__":
  unittest.main()
