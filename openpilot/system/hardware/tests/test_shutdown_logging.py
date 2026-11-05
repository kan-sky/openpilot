import ast
import logging
from pathlib import Path
from types import SimpleNamespace
import unittest

from openpilot.common.logging_extra import SwagFormatter, SwagLogger


def shutdown_block():
  source = Path(__file__).parents[1] / "hardwared.py"
  tree = ast.parse(source.read_text())
  blocks = [node for node in ast.walk(tree)
            if isinstance(node, ast.If) and isinstance(node.test, ast.Name) and node.test.id == "power_reason"]
  assert len(blocks) == 1
  return compile(ast.Module(body=blocks, type_ignores=[]), str(source), "exec")


class RecordingHandler(logging.Handler):
  def __init__(self, logger):
    super().__init__()
    self.records = []
    self.formatter = SwagFormatter(logger)

  def emit(self, record):
    self.records.append(self.formatter.format_dict(record))


def check_shutdown_request_logging(reason, voltage):
  logger = SwagLogger()
  logger.setLevel(logging.DEBUG)
  logger.propagate = False
  handler = RecordingHandler(logger)
  logger.addHandler(handler)
  requests = {}
  namespace = {
    "cloudlog": logger, "power_reason": reason, "power_shutdown_pending": False,
    "off_ts": 25., "voltage": voltage, "power_monitor": SimpleNamespace(car_voltage_mV=11250.),
    "params": SimpleNamespace(put=requests.__setitem__, remove=lambda key: requests.pop(key, None)),
    "REQUEST_KEY": "AutomaticShutdownRequest", "time": SimpleNamespace(monotonic=lambda: 100.),
  }
  block = shutdown_block()
  exec(block, namespace)
  assert requests["AutomaticShutdownRequest"] == {"reason": reason, "monotonicTime": 100.}
  assert namespace["power_shutdown_pending"] is True
  assert len(handler.records) == 1
  assert handler.records[0]["level"] == "WARNING"
  assert handler.records[0]["msg"] == {
    "event": "automatic shutdown requested", "reason": reason, "offroad_since": 25.,
    "voltage": voltage, "filtered_voltage": 11250.,
  }

  exec(block, namespace)
  assert len(handler.records) == 1
  namespace["power_reason"] = None
  exec(block, namespace)
  assert requests == {}
  assert namespace["power_shutdown_pending"] is False
  assert handler.records[-1]["msg"] == "automatic shutdown request cancelled"


class TestShutdownLogging(unittest.TestCase):
  def test_request_refresh_and_cancellation(self):
    for reason in ("low_voltage", "offroad_timeout"):
      for voltage in (None, 11200):
        with self.subTest(reason=reason, voltage=voltage):
          check_shutdown_request_logging(reason, voltage)
