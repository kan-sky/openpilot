import gzip
import importlib.util
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location("bluetooth_probe", Path(__file__).with_name("bluetooth_probe.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BluetoothProbeTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.root = Path(self.tmp.name)

  def write(self, path, text=""):
    target = self.root / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text)

  def config(self, text):
    target = self.root / "proc/config.gz"
    target.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(target, "wt") as stream:
      stream.write(text)

  def ready_files(self):
    self.config("\n".join(f"{key}=y" for key in module.CONFIG_KEYS))
    for path in ("dev/ttyHS0", "dev/ttyHS1", "dev/btpower", "usr/bin/bluetoothctl",
                 "usr/bin/btattach", "usr/lib/systemd/system/carrot-bluetooth-radio.service"):
      self.write(path)

  def test_disabled_kernel_is_not_a_radio_toggle(self):
    self.config("# CONFIG_BT is not set\n")
    result = module.probe(self.root)
    self.assertEqual(result["kernel_config"]["CONFIG_BT"], "n")
    self.assertIn("CONFIG_BT", result["missing_or_unknown_prerequisites"])
    self.assertEqual(result["status"], "missing_prerequisites")

  def test_missing_config_is_unknown_not_disabled(self):
    result = module.probe(self.root)
    self.assertIsNone(result["kernel_config"]["CONFIG_BT"])
    self.assertEqual(result["kernel_config_error"], "FileNotFoundError")

  def test_corrupt_config_is_reported(self):
    self.write("proc/config.gz", "not gzip")
    self.assertEqual(module.probe(self.root)["kernel_config_error"], "BadGzipFile")

  def test_gps_uart_alone_does_not_count_as_bluetooth(self):
    self.write("dev/ttyHS0")
    result = module.probe(self.root)
    self.assertTrue(result["checks"]["gps_uart_present"])
    self.assertFalse(result["checks"]["bluetooth_uart_present"])

  def test_installed_os_does_not_mean_radio_or_link_work(self):
    self.ready_files()
    result = module.probe(self.root)
    self.assertEqual(result["missing_or_unknown_prerequisites"], [])
    self.assertEqual(result["status"], "radio_not_initialized")
    self.assertFalse(result["checks"]["radio_opted_in"])
    self.assertFalse(result["navdy_link_verified"])

  def test_controller_presence_does_not_prove_concurrency(self):
    self.ready_files()
    self.write("sys/class/bluetooth/hci0/name", "test")
    self.write("data/bluetooth/ENABLED")
    result = module.probe(self.root)
    self.assertEqual(result["status"], "controller_present_unverified")
    self.assertFalse(result["phone_ambient_concurrency_verified"])

  def test_module_config_is_not_claimed_loaded(self):
    self.ready_files()
    self.config("\n".join(f"{key}=m" for key in module.CONFIG_KEYS))
    self.assertEqual(module.probe(self.root)["status"], "radio_not_initialized")

  def test_probe_does_not_write_or_expose_pairing_keys(self):
    self.ready_files()
    self.write("data/bluetooth/private/info", "secret-pairing-key")
    self.write("data/params/d/IsOnroad", "0")
    before = {str(p): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
    result = module.probe(self.root)
    after = {str(p): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
    self.assertEqual(before, after)
    self.assertNotIn("secret-pairing-key", str(result))
    self.assertEqual(result["observed_params"]["IsOnroad"], "0")


if __name__ == "__main__":
  unittest.main()
