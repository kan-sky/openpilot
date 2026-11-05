import pytest

from openpilot.selfdrive.carrot.deceleration_source import external_navigation_connected


@pytest.mark.parametrize(("remote", "connected", "expected"), [
  (None, False, False),
  ("", False, False),
  ("  ", False, False),
  ("192.0.2.1", False, True),
  ("", True, True),
])
def test_external_navigation_legacy_call_is_unchanged(remote, connected, expected):
  assert external_navigation_connected(remote, connected) is expected


@pytest.mark.parametrize("remote", ["", "192.0.2.1"])
@pytest.mark.parametrize("connected", [False, True])
@pytest.mark.parametrize("active", [False, True])
def test_navdy_selection_requires_its_own_fresh_navigation(remote, connected, active):
  assert external_navigation_connected(
    remote, connected, external_navigation_source="navdy", external_navigation_active=active,
  ) is active


@pytest.mark.parametrize(("remote", "connected", "active", "expected"), [
  ("", False, False, False),
  ("", False, True, True),
  ("192.0.2.1", False, False, True),
  ("", True, False, True),
])
def test_network_selection_preserves_existing_fallbacks(remote, connected, active, expected):
  assert external_navigation_connected(
    remote, connected, external_navigation_source="network", external_navigation_active=active,
  ) is expected
