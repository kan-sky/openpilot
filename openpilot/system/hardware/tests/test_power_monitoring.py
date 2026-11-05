import pytest

from openpilot.common.params import Params
from openpilot.system.hardware.power_monitoring import PowerMonitoring, CAR_BATTERY_CAPACITY_uWh, \
                                                CAR_CHARGING_RATE_W, VBATT_PAUSE_CHARGING, LOW_VOLTAGE_CONFIRMATION_S

# Create fake time
ssb = 0.
def mock_time_monotonic():
  global ssb
  ssb += 1.
  return ssb

TEST_DURATION_S = 50
GOOD_VOLTAGE = 12 * 1e3
VOLTAGE_BELOW_PAUSE_CHARGING = (VBATT_PAUSE_CHARGING - 1) * 1e3

def pm_patch(mocker, name, value, constant=False):
  if constant:
    mocker.patch(f"openpilot.system.hardware.power_monitoring.{name}", value)
  else:
    mocker.patch(f"openpilot.system.hardware.power_monitoring.{name}", return_value=value)


def confirmed_voltage(pm, mocker, voltage, start=0.):
  clock = mocker.patch("time.monotonic", return_value=start)
  pm.car_voltage_mV = voltage
  for tick in range(int(LOW_VOLTAGE_CONFIRMATION_S * 2) + 1):
    clock.return_value = start + tick * 0.5
    pm.calculate(voltage, False, True)
  return clock


@pytest.fixture(autouse=True)
def mock_time(mocker):
  global ssb
  ssb = 0.
  mocker.patch("time.monotonic", mock_time_monotonic)


class TestPowerMonitoring:
  def setup_method(self):
    self.params = Params()

  # Test to see that it doesn't do anything when pandaState is None
  def test_panda_state_present(self):
    pm = PowerMonitoring()
    for _ in range(10):
      pm.calculate(None, False, True)
    assert pm.get_power_used() == 0
    assert pm.get_car_battery_capacity() == (CAR_BATTERY_CAPACITY_uWh / 10)

  # Test to see that it doesn't integrate offroad when ignition is True
  def test_offroad_ignition(self):
    pm = PowerMonitoring()
    for _ in range(10):
      pm.calculate(GOOD_VOLTAGE, True, True)
    assert pm.get_power_used() == 0

  # Test to see that it integrates with discharging battery
  def test_offroad_integration_discharging(self, mocker):
    POWER_DRAW = 4
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    pm = PowerMonitoring()
    for _ in range(TEST_DURATION_S + 1):
      pm.calculate(GOOD_VOLTAGE, False, True)
    expected_power_usage = ((TEST_DURATION_S/3600) * POWER_DRAW * 1e6)
    assert abs(pm.get_power_used() - expected_power_usage) < 10

  # Test to check positive integration of car_battery_capacity
  def test_car_battery_integration_onroad(self, mocker):
    POWER_DRAW = 4
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = 0
    for _ in range(TEST_DURATION_S + 1):
      pm.calculate(GOOD_VOLTAGE, True, True)
    expected_capacity = ((TEST_DURATION_S/3600) * CAR_CHARGING_RATE_W * 1e6)
    assert abs(pm.get_car_battery_capacity() - expected_capacity) < 10

  # Test to check positive integration upper limit
  def test_car_battery_integration_upper_limit(self, mocker):
    POWER_DRAW = 4
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = CAR_BATTERY_CAPACITY_uWh - 1000
    for _ in range(TEST_DURATION_S + 1):
      pm.calculate(GOOD_VOLTAGE, True, True)
    estimated_capacity = CAR_BATTERY_CAPACITY_uWh + (CAR_CHARGING_RATE_W / 3600 * 1e6)
    assert abs(pm.get_car_battery_capacity() - estimated_capacity) < 10

  # Test to check negative integration of car_battery_capacity
  def test_car_battery_integration_offroad(self, mocker):
    POWER_DRAW = 4
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = CAR_BATTERY_CAPACITY_uWh
    for _ in range(TEST_DURATION_S + 1):
      pm.calculate(GOOD_VOLTAGE, False, True)
    expected_capacity = CAR_BATTERY_CAPACITY_uWh - ((TEST_DURATION_S/3600) * POWER_DRAW * 1e6)
    assert abs(pm.get_car_battery_capacity() - expected_capacity) < 10

  # Test to check negative integration lower limit
  def test_car_battery_integration_lower_limit(self, mocker):
    POWER_DRAW = 4
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = 1000
    for _ in range(TEST_DURATION_S + 1):
      pm.calculate(GOOD_VOLTAGE, False, True)
    estimated_capacity = 0 - ((1/3600) * POWER_DRAW * 1e6)
    assert abs(pm.get_car_battery_capacity() - estimated_capacity) < 10

  @pytest.mark.parametrize("minutes", [0, 60, 4320])
  @pytest.mark.parametrize("offset, expected", [(0, False), (1, True)])
  @pytest.mark.parametrize("in_car", [False, True])
  def test_max_time_offroad(self, mocker, minutes, offset, expected, in_car):
    self.params.put_int("MaxTimeOffroadMin", minutes)
    pm = PowerMonitoring()
    start_time = 10.
    mocker.patch("time.monotonic", return_value=start_time + minutes * 60 + offset)
    assert pm.should_shutdown(False, in_car, start_time, False) == expected

  def test_car_voltage(self, mocker):
    POWER_DRAW = 0 # To stop shutting down for other reasons
    TEST_TIME = 350
    pm_patch(mocker, "HARDWARE.get_current_power_draw", POWER_DRAW)
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = CAR_BATTERY_CAPACITY_uWh
    ignition = False
    start_time = 0.
    clock = mocker.patch("time.monotonic", return_value=start_time)
    first_low = None
    for i in range(TEST_TIME):
      clock.return_value = i * 0.5
      pm.calculate(VOLTAGE_BELOW_PAUSE_CHARGING, ignition, True)
      if pm.car_voltage_mV < VBATT_PAUSE_CHARGING * 1e3 and first_low is None:
        first_low = clock.return_value
      expected = first_low is not None and clock.return_value - first_low >= LOW_VOLTAGE_CONFIRMATION_S
      assert pm.should_shutdown(ignition, True, start_time, True) == expected
    assert pm.should_shutdown(ignition, True, start_time, True)

  @pytest.mark.parametrize("voltage, expected", [(11299, True), (11300, False), (11301, False), (11700, False)])
  def test_voltage_threshold(self, mocker, voltage, expected):
    assert VBATT_PAUSE_CHARGING == 11.3
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    confirmed_voltage(pm, mocker, voltage)
    assert pm.should_shutdown(False, True, 0., False) == expected

  @pytest.mark.parametrize("elapsed", [0, 1, 59, 60, 299, 300, 3599])
  def test_only_ten_second_confirmation_not_old_startup_delays(self, mocker, elapsed):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    mocker.patch("time.monotonic", return_value=elapsed)
    pm = PowerMonitoring()
    pm.car_voltage_mV = 11200
    pm.calculate(11200, False, True)
    assert not pm.should_shutdown(False, True, 0., False)
    confirmed_voltage(pm, mocker, 11200, start=elapsed)
    assert pm.should_shutdown(False, True, 0., False)

  @pytest.mark.parametrize("voltage", [0, 5000, 9000, 11200])
  def test_bench_voltage_never_triggers_car_battery_shutdown(self, voltage):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    pm.car_voltage_mV = voltage
    assert not pm.should_shutdown(False, False, 0., False)

  @pytest.mark.parametrize("voltage", [0, 5000, 9000])
  def test_bench_voltage_does_not_pollute_car_battery_filter(self, voltage):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    capacity = pm.get_car_battery_capacity()
    for _ in range(600):
      pm.calculate(voltage, False, False)
      assert not pm.should_shutdown(False, False, 0., False)
    assert pm.car_voltage_mV == GOOD_VOLTAGE
    assert pm.last_measurement_time is None
    assert pm.get_power_used() == 0
    assert pm.get_car_battery_capacity() == capacity

    # Connecting a healthy car must not inherit the bench supply's low voltage.
    pm.calculate(GOOD_VOLTAGE, False, True)
    assert not pm.should_shutdown(False, True, 0., False)

  def test_bench_to_low_car_battery_still_shuts_down(self):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    for _ in range(600):
      pm.calculate(5000, False, False)
    for _ in range(350):
      pm.calculate(VOLTAGE_BELOW_PAUSE_CHARGING, False, True)
    assert pm.should_shutdown(False, True, 0., False)

  def test_disconnect_stops_battery_integration_and_voltage_shutdown(self, mocker):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm_patch(mocker, "HARDWARE.get_current_power_draw", 4)
    pm = PowerMonitoring()
    for _ in range(350):
      pm.calculate(VOLTAGE_BELOW_PAUSE_CHARGING, False, True)
    assert pm.should_shutdown(False, True, 0., False)
    capacity = pm.get_car_battery_capacity()
    for _ in range(100):
      pm.calculate(5000, False, False)
      assert not pm.should_shutdown(False, False, 0., False)
    assert pm.last_measurement_time is None
    assert pm.get_power_used() == 0
    assert pm.get_car_battery_capacity() == capacity

  @pytest.mark.parametrize("capacity", [0, -1])
  def test_empty_estimated_capacity_does_not_shutdown(self, capacity):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    pm.car_battery_capacity_uWh = capacity
    assert not pm.should_shutdown(False, True, 0., True)

  @pytest.mark.parametrize("key", ["DisablePowerDown", "ForcePowerDown"])
  def test_legacy_override_does_not_change_policy(self, mocker, key):
    self.params.put_bool(key, True)
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    assert not pm.should_shutdown(False, True, 0., True)
    confirmed_voltage(pm, mocker, 11200)
    assert pm.should_shutdown(False, True, 0., True)
    assert not pm.should_shutdown(True, True, 0., True)

  @pytest.mark.parametrize("voltage", [10000, 12000])
  @pytest.mark.parametrize("elapsed", [1, 4320 * 60 + 1])
  def test_ignition_always_blocks_shutdown(self, mocker, voltage, elapsed):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    self.params.put_bool("ForcePowerDown", True)
    mocker.patch("time.monotonic", return_value=elapsed)
    pm = PowerMonitoring()
    pm.car_voltage_mV = voltage
    assert not pm.should_shutdown(True, True, 0., True)

  def test_unknown_offroad_timestamp(self):
    pm = PowerMonitoring()
    pm.car_voltage_mV = 10000
    assert not pm.should_shutdown(False, True, None, False)

  def test_timeout_setting_updates_without_restart(self, mocker):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    mocker.patch("time.monotonic", return_value=61.)
    pm = PowerMonitoring()
    assert not pm.should_shutdown(False, True, 0., True)
    self.params.put_int("MaxTimeOffroadMin", 1)
    assert pm.should_shutdown(False, True, 0., True)

  @pytest.mark.parametrize("change", ["ignition", "raw_recovery", "missing_voltage", "harness", "zero_voltage"])
  def test_low_voltage_confirmation_resets(self, mocker, change):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    clock = confirmed_voltage(pm, mocker, 11200)
    assert pm.shutdown_reason(False, True, 0.) == "low_voltage"
    clock.return_value += 0.5
    pm.calculate({"raw_recovery": 12000, "missing_voltage": None, "zero_voltage": 0}.get(change, 11200),
                 change == "ignition", change != "harness")
    assert pm.low_voltage_since is None
    assert pm.shutdown_reason(False, True, 0.) is None
    clock.return_value += 0.5
    pm.calculate(11200, False, True)
    assert pm.shutdown_reason(False, True, 0.) is None

  def test_short_crank_dip_never_requests_shutdown(self, mocker):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    pm.car_voltage_mV = 11250
    clock = mocker.patch("time.monotonic", return_value=0.)
    for tick in range(8):
      clock.return_value = tick * 0.5
      pm.calculate(9000, False, True)
      assert not pm.should_shutdown(False, True, 0., False)
    clock.return_value += 0.5
    pm.calculate(14000, True, True)
    assert not pm.should_shutdown(True, True, 0., False)
    assert pm.low_voltage_since is None

  @pytest.mark.parametrize("gap", [1.501, 5., -1.])
  def test_missing_or_reversed_measurement_time_restarts_confirmation(self, mocker, gap):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    clock = confirmed_voltage(pm, mocker, 11200)
    clock.return_value += gap
    assert not pm.should_shutdown(False, True, 0., False)
    # Avoid the independent energy integrator's deliberate negative-time error.
    pm.last_measurement_time = None
    pm.calculate(11200, False, True)
    assert not pm.should_shutdown(False, True, 0., False)

  def test_repeated_decision_without_new_samples_cannot_confirm_low_voltage(self, mocker):
    self.params.put_int("MaxTimeOffroadMin", 4320)
    pm = PowerMonitoring()
    clock = mocker.patch("time.monotonic", return_value=0.)
    pm.car_voltage_mV = 11200
    pm.calculate(11200, False, True)
    for elapsed in (1., 10., 100.):
      clock.return_value = elapsed
      assert not pm.should_shutdown(False, True, 0., False)
