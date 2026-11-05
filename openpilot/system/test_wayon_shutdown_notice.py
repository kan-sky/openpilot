import subprocess
import json
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.system.wayon_shutdown_notice import NOTICE_BUDGET_S, _https_post, _NoRedirect, notice_payload, notify_automatic_shutdown, send_notice


@pytest.mark.parametrize("result,status", [(0, "accepted"), (1, "unavailable"), ("timeout", "timeout"), ("error", "unavailable")])
def test_bounded_child_and_secret_free_arguments(result, status, capsys):
  runner = Mock(return_value=Mock(returncode=result))
  if result == "timeout":
    runner.side_effect = subprocess.TimeoutExpired("isolated", NOTICE_BUDGET_S)
  elif result == "error":
    runner.side_effect = OSError("failed")
  with patch("openpilot.system.wayon_shutdown_notice.notice_payload", return_value={"reason": "low_voltage"}), \
       patch("openpilot.system.wayon_shutdown_notice.subprocess.run", runner):
    notify_automatic_shutdown("low_voltage", Mock(), Mock())
  args, kwargs = runner.call_args
  assert args[0] == [sys.executable, "-m", "openpilot.system.wayon_shutdown_notice"]
  assert kwargs["timeout"] == 1.0
  assert kwargs["input"] == '{"reason": "low_voltage"}'
  assert kwargs["stdout"] == kwargs["stderr"] == subprocess.DEVNULL
  assert status in capsys.readouterr().out


def test_real_hung_child_is_killed_and_reaped():
  # Exercise the actual subprocess deadline, without networking or a poweroff.
  started = time.monotonic()
  with pytest.raises(subprocess.TimeoutExpired):
    subprocess.run([sys.executable, "-c", "import time; time.sleep(30)"], timeout=NOTICE_BUDGET_S)
  assert time.monotonic() - started < 2.0


@pytest.mark.parametrize("reason", ["low_voltage", "offroad_timeout"])
def test_sender_requires_cloud_acceptance(reason):
  config = {"endpoint": "https://example.test", "token": "wayon_synthetic", "device_id": "test"}
  payload = {"eventType": "automatic_shutdown", "reason": reason}
  post = Mock(return_value=Mock(status_code=202, json=lambda: {"accepted": True}))
  with patch("openpilot.system.wayon_identity.ensure_wayon_identity", return_value=config):
    assert send_notice(payload, post=post)
    post.return_value.status_code = 200
    assert not send_notice(payload, post=post)
  args, kwargs = post.call_args
  assert args == ("https://example.test/api/vehicle-event",)
  assert kwargs["allow_redirects"] is False
  assert kwargs["timeout"] == (0.35, 0.6)
  assert kwargs["json"] == payload


@pytest.mark.parametrize("config", [None, {"endpoint": "http://example.test", "token": "wayon_test"},
                                   {"endpoint": "https://user:secret@example.test", "token": "wayon_test"},
                                   {"endpoint": "https://example.test?token=secret", "token": "wayon_test"}])
def test_disabled_or_invalid_config_never_posts(config):
  post = Mock()
  with patch("openpilot.system.wayon_identity.ensure_wayon_identity", return_value=config):
    assert not send_notice({"eventType": "automatic_shutdown", "reason": "low_voltage"}, post=post)
  post.assert_not_called()


@pytest.mark.parametrize("payload", [None, {}, {"eventType": "automatic_shutdown", "reason": "manual"}])
def test_invalid_payload_never_posts(payload):
  post = Mock()
  assert not send_notice(payload, post=post)
  post.assert_not_called()


def test_native_http_adapter_posts_json_without_redirects():
  response = Mock(status=202, read=Mock(return_value=b'{"accepted":true}'))
  context = Mock(__enter__=Mock(return_value=response), __exit__=Mock(return_value=False))
  opener = Mock(open=Mock(return_value=context))
  with patch("openpilot.system.wayon_shutdown_notice.build_opener", return_value=opener) as builder:
    result = _https_post("https://example.test/api/vehicle-event", json={"test": True},
                         headers={"Authorization": "Bearer synthetic"}, timeout=(0.35, 0.6), allow_redirects=False)
  assert result.status_code == 202 and result.json()["accepted"] is True
  req = opener.open.call_args.args[0]
  assert req.get_method() == "POST" and json.loads(req.data) == {"test": True}
  assert req.get_header("Content-type") == "application/json"
  assert isinstance(builder.call_args.args[0], _NoRedirect)
  assert _NoRedirect().redirect_request(None, None, None, None, None, None) is None
  response.read.assert_called_once_with(4096)


@pytest.mark.parametrize("fresh,voltage,expected", [(True,11200,11.2),(False,11200,None),(True,0,None)])
def test_payload_uses_only_fresh_voltage(fresh, voltage, expected):
  sm=Mock()
  sm.__getitem__=Mock(return_value=NS(voltage=voltage))
  params=Mock(get_int=Mock(return_value=4320))
  with patch("openpilot.system.hardware.automatic_shutdown.message_is_fresh", return_value=fresh):
    value=notice_payload("low_voltage",sm,params)
  assert value["voltageV"] == expected and value["offroadLimitMinutes"] == 4320
  assert value["id"].startswith("shutdown-") and len(value["id"]) == 41
  assert value["eventType"] == "automatic_shutdown" and "test" not in value
