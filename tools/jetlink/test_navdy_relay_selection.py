from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from openpilot.selfdrive.navdy import bridge
from tools.jetlink import manage_jetson


@pytest.mark.parametrize('mode,expected', [
  (None, '/opt/carrot-jetlink/venv/bin/python -u /opt/carrot-jetlink/current/openpilot/selfdrive/navdy/transport.py --stdio'),
  ('protected-tools-v1', 'sudo -n /usr/bin/python3 -u /run/carrot-data/hl-navdy/navdy_management.py relay'),
])
def test_relay_uses_only_selected_fixed_endpoint(monkeypatch, tmp_path, mode, expected):
  monkeypatch.setattr(manage_jetson, 'paired_config', lambda state: {'navdy_relay': mode})
  monkeypatch.setattr(manage_jetson, 'ssh_options', lambda state: ['-F', '/dev/null'])
  process = SimpleNamespace(stdin=SimpleNamespace(fileno=lambda: 98), stdout=SimpleNamespace(fileno=lambda: 99))
  popen = Mock(return_value=process)
  monkeypatch.setattr(bridge.subprocess, 'Popen', popen)
  monkeypatch.setattr(bridge.os, 'set_blocking', lambda *args: None)
  bridge.SshTransport('192.168.60.12', tmp_path)
  assert popen.call_args.args[0] == ['ssh', '-F', '/dev/null', '-T', 'jetlink@192.168.60.12', expected]


def test_mobile_candidates_remain_opt_in(monkeypatch, tmp_path):
  from tools.jetlink import usb_internet
  monkeypatch.setattr(manage_jetson, 'usb_candidate', lambda: [])
  monkeypatch.setattr(usb_internet, 'mobile_candidates', lambda: ['192.168.60.12'])
  config = {'host': '192.168.0.21'}
  assert manage_jetson.candidates(config, tmp_path) == ['192.168.0.21']
  (tmp_path / 'usb-internet.enabled').touch()
  assert manage_jetson.candidates(config, tmp_path) == ['192.168.60.12', '192.168.0.21']
