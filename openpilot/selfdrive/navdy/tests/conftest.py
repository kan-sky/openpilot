import pytest
from types import SimpleNamespace

from openpilot.selfdrive.navdy import bridge, journal


@pytest.fixture(autouse=True)
def isolated_journal(monkeypatch, tmp_path):
  # Bridge tests must never write production logs, including when run as root.
  monkeypatch.setattr(journal, 'DIRECTORY', tmp_path.resolve() / 'diagnostics')
  monkeypatch.setattr(bridge, 'read_transport_mode', lambda: 0)
  monkeypatch.setattr(bridge, 'UsbFaultRecorder', lambda *a, **kw: SimpleNamespace(capture=lambda _: None, close=lambda: None))
