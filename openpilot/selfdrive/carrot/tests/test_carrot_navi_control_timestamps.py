import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.carrot.carrot_navi_control import NaviSpeedControl, parse_carrot_navi_control


@pytest.mark.parametrize('received,expected', [
  (None, 0), (0, 0), (-1, 0), ('invalid', 0), (float('nan'), 0),
  (float('inf'), 0), (123_456_789_012, 123_456_789_012),
])
def test_speed_preserves_original_receipt_timestamp_without_changing_control(received, expected):
  meta = {'present': True, 'sequence': 3, 'sourceTimestampMillis': 999_999_999}
  if received is not None:
    meta['receivedMonoTimeNanos'] = received
  data = {
    'schemaVersion': 1, 'connected': True,
    'speed': {
      'meta': meta, 'roadLimitValid': True, 'roadLimitKph': 80,
      'sdiPresent': True, 'sdiType': 1, 'sdiDistanceM': 420, 'sdiSpeedLimitKph': 60,
    },
  }
  control = parse_carrot_navi_control(data)
  assert control.speed.received_mono_time_ns == expected
  assert control.speed.road_limit_kph == 80
  assert (control.speed.sdi_type, control.speed.sdi_distance_m, control.speed.sdi_speed_limit_kph) == (1, 420, 60)


def test_speed_receipt_timestamp_survives_cereal_roundtrip():
  message = custom.CarrotNaviState.new_message(schemaVersion=1, connected=True, speed={
    'meta': {'present': True, 'sequence': 7, 'receivedMonoTimeNanos': 123_456_789_012},
    'roadLimitValid': True, 'roadLimitKph': 80,
  })
  with custom.CarrotNaviState.from_bytes(message.to_bytes()) as decoded:
    speed = parse_carrot_navi_control(decoded).speed
  assert (speed.received_mono_time_ns, speed.sequence, speed.road_limit_kph) == (123_456_789_012, 7, 80)


def test_speed_receipt_timestamp_defaults_to_unknown_for_existing_callers():
  assert NaviSpeedControl(True, 1, 80).received_mono_time_ns == 0
