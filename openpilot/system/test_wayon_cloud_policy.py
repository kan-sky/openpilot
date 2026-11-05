import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("cloud_policy", Path(__file__).with_name("wayon_cloud_policy.py"))
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class CloudPolicyTest(unittest.TestCase):
  def test_backoff_is_bounded_and_resets(self):
    retry = policy.UploadBackoff(jitter=lambda: 0)
    self.assertEqual([retry.failure_delay() for _ in range(6)], [30, 60, 120, 240, 300, 300])
    retry.success()
    self.assertEqual(retry.failure_delay(), 30)

  def test_independent_channels_and_jitter(self):
    telemetry = policy.UploadBackoff(jitter=lambda: 1)
    routes = policy.UploadBackoff(jitter=lambda: 0)
    self.assertEqual(telemetry.failure_delay(), 36)
    self.assertEqual(routes.failure_delay(), 30)
    self.assertLessEqual(max(telemetry.failure_delay() for _ in range(100)), 300)

  def test_malformed_config_cannot_crash_or_spin_uploader(self):
    for value in (None, {}, "bad", "nan", float("inf")):
      self.assertEqual(policy.bounded_interval(value, 30, 5, 3600), 30)
    self.assertEqual(policy.bounded_interval(-1, 30, 5, 3600), 5)
    self.assertEqual(policy.bounded_interval(1e9, 30, 5, 3600), 3600)


if __name__ == "__main__":
  unittest.main()
