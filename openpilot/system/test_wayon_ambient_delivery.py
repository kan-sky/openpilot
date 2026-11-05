import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("delivery", Path(__file__).with_name("wayon_ambient_delivery.py"))
delivery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(delivery)


class TestAmbientDelivery(unittest.TestCase):
  def test_startup_then_bounded_poll_without_relay(self):
    with tempfile.TemporaryDirectory() as directory:
      scheduler = delivery.AmbientCommandDelivery(Path(directory) / "wake")
      calls = []
      for second in range(86400):
        scheduler.run(second, lambda: calls.append(second))
      self.assertEqual(calls, list(range(0, 86400, 60)))

  def test_periodic_poll_failure_retains_backoff_and_notification(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "wake"
      scheduler = delivery.AmbientCommandDelivery(path)
      scheduler.run(0, lambda: None)
      def fail():
        raise OSError("offline")
      for now, retry in ((60, 65), (65, 75), (75, 95), (95, 135), (135, 195), (195, 255)):
        with self.assertRaises(OSError):
          scheduler.run(now, fail)
        delivery.notify_ambient_command(path)
        self.assertFalse(scheduler.run(retry - .1, fail))
        self.assertEqual(scheduler.next_retry, retry)
      self.assertTrue(scheduler.run(255, lambda: None))
      self.assertEqual(scheduler.failures, 0)
      self.assertFalse(scheduler.run(314.9, fail))
      self.assertTrue(scheduler.run(315, lambda: None))

  def test_unreadable_notification_file_does_not_disable_fallback(self):
    with tempfile.TemporaryDirectory() as directory:
      scheduler = delivery.AmbientCommandDelivery(Path(directory))
      self.assertTrue(scheduler.run(0, lambda: None))
      self.assertFalse(scheduler.run(59, lambda: self.fail("early poll")))
      self.assertTrue(scheduler.run(60, lambda: None))

  def test_notifications_coalesce_and_request_race_is_retained(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "wake"
      scheduler = delivery.AmbientCommandDelivery(path)
      scheduler.run(0, lambda: None)
      delivery.notify_ambient_command(path)
      delivery.notify_ambient_command(path)
      self.assertTrue(scheduler.run(1, lambda: delivery.notify_ambient_command(path)))
      self.assertTrue(scheduler.run(2, lambda: None))
      self.assertFalse(scheduler.run(3, lambda: self.fail("idle request")))

  def test_failure_retries_without_losing_notification(self):
    with tempfile.TemporaryDirectory() as directory:
      scheduler = delivery.AmbientCommandDelivery(Path(directory) / "wake")
      def fail():
        raise OSError("offline")
      with self.assertRaises(OSError):
        scheduler.run(0, fail)
      self.assertFalse(scheduler.run(4, fail))
      self.assertTrue(scheduler.run(5, lambda: None))
      self.assertFalse(scheduler.run(6, fail))


if __name__ == "__main__":
  unittest.main()
