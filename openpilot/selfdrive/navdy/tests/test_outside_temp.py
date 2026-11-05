from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy.tests.test_display import car, payload
from openpilot.selfdrive.navdy.transport import encode_frame


def with_oat(valid=True, celsius=14.5, **kwargs):
  return car(outsideAirTemp=NS(valid=valid, celsius=celsius), **kwargs)


def test_valid_can_temperature_is_added_without_touching_other_fields():
  base = payload()
  result = payload(car_state=with_oat())
  assert result["outsideTempC"] == 14.5
  assert {k: v for k, v in result.items() if k != "outsideTempC"} == base
  encode_frame(result)


@pytest.mark.parametrize("state", [
  car(),
  with_oat(valid=False),
  with_oat(celsius=float("nan")),
  with_oat(celsius=-41.0),
  with_oat(canValid=False),
  with_oat(canTimeout=True),
])
def test_missing_or_invalid_temperature_leaves_key_absent(state):
  assert "outsideTempC" not in payload(car_state=state)


def test_temperature_is_sent_offroad_and_while_disengaged():
  assert payload(car_state=with_oat(), onroad=False)["outsideTempC"] == 14.5
  assert payload(car_state=with_oat(celsius=-3.0), selfdrive_state=None, onroad=False)["outsideTempC"] == -3.0
