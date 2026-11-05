from types import SimpleNamespace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.car.cruise import ButtonType, VCruiseCarrot, is_hold_interlock_active
from openpilot.selfdrive.carrot.cruise_gap import cruise_gap_levels
from openpilot.selfdrive.carrot.bluetooth.model import BLUETOOTH_CANCEL


def make_cruise_helper(button_kph, cruise_button_mode, carrot_cruise_active, cruise_enabled,
                       cruise_speed_initialized=True, cruise_speed_at_brake=0):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False, openpilotLongitudinalControl=True)
  helper.bluetooth_commands = SimpleNamespace(read=lambda **kwargs: None)
  helper._prepare_buttons = lambda CS, v_cruise_kph, remote=None: (button_kph, ButtonType.accelCruise, False)
  helper._carrot_command = lambda v_cruise_kph, button_type, long_pressed: (v_cruise_kph, button_type, long_pressed)
  helper._update_cruise_state = lambda CS, CC, v_cruise_kph: v_cruise_kph
  helper._add_log = lambda log: None

  helper._paddle_decel_active = False
  helper._activate_cruise = 0
  helper._cancel_timer = 0
  helper.autoCruiseControl = 1
  helper.button_cnt = 0
  helper._resume_button_active = False
  helper.autoCruiseControl_cancel_timer = 0
  helper._cruise_cancel_state = False
  helper._lat_enabled = False
  helper._pause_auto_speed_up = True
  helper._soft_hold_active = 0
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_ready = False
  helper._v_cruise_kph_at_brake = cruise_speed_at_brake
  helper._cruise_speed_initialized = cruise_speed_initialized
  helper._cruise_button_mode = cruise_button_mode
  helper._cruise_speed_unit = 10
  helper._cruise_speed_min = 5
  helper._cruise_speed_max = 161
  helper.navigation_set_speed = False
  helper.disengage_on_accelerator = False
  helper._paddle_mode = 0
  helper.v_ego_kph_set = 70
  helper.carrot_cruise_active = carrot_cruise_active

  CS = car.CarState(cruiseState={"standstill": False})
  CC = car.CarControl(enabled=cruise_enabled)
  return helper, CS, CC


@pytest.mark.parametrize("cruise_button_mode, button_kph", [(0, 81), (2, 81)])
def test_accel_exits_carrot_cruise_without_increasing_speed(cruise_button_mode, button_kph):
  helper, CS, CC = make_cruise_helper(
    button_kph,
    cruise_button_mode,
    carrot_cruise_active=True,
    cruise_enabled=True,
    cruise_speed_initialized=False,
    cruise_speed_at_brake=90,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert not helper.carrot_cruise_active
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_button_mode, expected_speed", [(0, 81), (2, 90)])
@pytest.mark.parametrize("helper_state", ["resume_speed", "uninitialized"])
def test_accel_always_increases_speed_while_cruise_is_on(cruise_button_mode, expected_speed, helper_state):
  helper, CS, CC = make_cruise_helper(
    81,
    cruise_button_mode,
    carrot_cruise_active=False,
    cruise_enabled=True,
    cruise_speed_initialized=helper_state != "uninitialized",
    cruise_speed_at_brake=95 if helper_state == "resume_speed" else 0,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == expected_speed
  assert helper._cruise_speed_initialized
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_speed_at_brake, expected_speed", [(75, 80), (90, 90)])
def test_accel_restores_at_least_brake_speed_while_cruise_is_off(cruise_speed_at_brake, expected_speed):
  helper, CS, CC = make_cruise_helper(
    81,
    cruise_button_mode=0,
    carrot_cruise_active=False,
    cruise_enabled=False,
    cruise_speed_at_brake=cruise_speed_at_brake,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == expected_speed
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_button_mode", [0, 2])
def test_accel_keeps_initialized_speed_without_brake_snapshot_while_cruise_is_off(cruise_button_mode):
  helper, CS, CC = make_cruise_helper(81, cruise_button_mode, carrot_cruise_active=False, cruise_enabled=False)

  assert helper._update_cruise_buttons(CS, CC, 80) == 80


@pytest.mark.parametrize(('action', 'expected'), [('accelCruise', 81), ('decelCruise', 79)])
def test_bluetooth_uses_cruise_button_steps(action, expected):
  helper, CS, CC = make_cruise_helper(81, 0, False, True)
  helper._prepare_buttons = VCruiseCarrot._prepare_buttons.__get__(helper)
  helper.button_cnt = 0
  helper.button_long_time = 50
  helper.long_pressed = False
  helper._cruise_speed_unit_basic = 1
  helper._cruise_button_long_delay = 50
  helper.is_metric = True
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: action if allowed else None)
  CS.canValid = True
  CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  CS.cruiseSpeedBigStep = True  # A physical stalk's big-step bit must not affect HID.
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper.button_cnt == 0
  assert len(CS.buttonEvents) == 0  # No fake CAN/CarState events.


@pytest.mark.parametrize('block', ['can', 'available', 'gear', 'button', 'held'])
def test_bluetooth_does_not_override_vehicle_button_or_gate(block):
  helper, CS, CC = make_cruise_helper(81, 0, False, True)
  helper.button_cnt = int(block == 'held')
  CS.canValid = block != 'can'
  CS.cruiseState.available = block != 'available'
  CS.gearShifter = 'park' if block == 'gear' else 'drive'
  if block == 'button':
    CS.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  allowed_values = []
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: allowed_values.append(allowed))
  helper._update_cruise_buttons(CS, CC, 80)
  assert allowed_values == [False]


def test_bluetooth_paddle_decel_uses_ready_path_independent_of_physical_paddle_mode():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, 0, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'paddleDecel' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  calls = []
  helper._cruise_control = lambda *args: calls.append(args[:2])
  helper._update_cruise_buttons(CS, CC, 80)
  assert calls == [(-2, -1)]
  assert helper._paddle_decel_active
  assert helper._paddle_mode == 0


def test_bluetooth_gap_cycles_even_with_pcm_gap():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, ButtonType.gapAdjustCruise, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'gapAdjustCruise' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  CS.pcmCruiseGap = 4
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=True)
  values = {'LongitudinalPersonalityMax': 4, 'CruiseGapLevels': 4, 'LongitudinalPersonality': 3}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  helper._update_cruise_buttons(CS, CC, 80)
  assert values['LongitudinalPersonality'] != 3


@pytest.mark.parametrize('initial', [False, True])
def test_bluetooth_carrot_cruise_enters_existing_mode_without_toggling_or_speed_change(initial):
  helper, CS, CC = make_cruise_helper(80, 0, initial, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, 0, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'carrotCruise' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  calls = []
  helper._cruise_control = lambda *args: calls.append(args)
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert helper.carrot_cruise_active
  assert not helper._paddle_decel_active
  assert calls == []


def make_remote_helper(action, enabled=False):
  helper, CS, CC = make_cruise_helper(80, 0, False, enabled)
  helper._prepare_buttons = VCruiseCarrot._prepare_buttons.__get__(helper)
  helper.button_long_time = helper._cruise_button_long_delay = 50
  helper.long_pressed = False
  helper._cruise_speed_unit_basic = 1
  helper.is_metric = True
  helper._cancel_button_mode = 0
  helper._lfa_button_mode = 0
  helper.useLaneLineSpeed = 50
  helper.useLaneLineSpeedApply = 0
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: action if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  return helper, CS, CC


@pytest.mark.parametrize('mode', [0, 1, 2, 3])
@pytest.mark.parametrize('held_frames', [3, 55, 130])
@pytest.mark.parametrize('enable_frame', [1, 2, None])
def test_gm_resume_press_does_not_also_increase_speed(mode, held_frames, enable_frame):
  from opendbc.car.gm.carstate import CarState

  helper, CS, CC = make_remote_helper(None)
  helper._cruise_button_mode = mode
  helper._cruise_speed_unit_basic = 5
  helper._cruise_speed_table = [80, 90, 100]
  gm = SimpleNamespace(CP=SimpleNamespace(pcmCruise=False))
  speed = 80

  for frame in range(held_frames + 1):
    CS.buttonEvents = ([{'type': 'accelCruise', 'pressed': True}] if frame == 0 else
                       [{'type': 'accelCruise', 'pressed': False}] if frame == held_frames else [])
    CS.buttonEnable = CarState.update_button_enable(gm, CS.buttonEvents)
    CC.enabled = enable_frame is not None and frame >= enable_frame
    speed = helper._update_cruise_buttons(CS, CC, speed)
    assert speed == 80, (frame, CS.buttonEnable, CC.enabled)

  # A separate press after engagement still increases the speed normally.
  CC.enabled = True
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': True}]
  CS.buttonEnable = CarState.update_button_enable(gm, CS.buttonEvents)
  assert helper._update_cruise_buttons(CS, CC, speed) == 80
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': False}]
  CS.buttonEnable = CarState.update_button_enable(gm, CS.buttonEvents)
  assert helper._update_cruise_buttons(CS, CC, speed) == (85 if mode == 0 else 90)


@pytest.mark.parametrize('initialized,saved,expected', [(True, 0, 80), (True, 95, 95), (False, 0, 70)])
def test_gm_resume_selects_speed_before_enabled_feedback(monkeypatch, initialized, saved, expected):
  from opendbc.car.gm.carstate import CarState
  from openpilot.selfdrive.car import cruise
  from openpilot.selfdrive.car.tests.test_toyota_pcm_mode0 import MemoryParams, MemoryReader, SubMaster

  monkeypatch.setattr(cruise, 'Params', MemoryParams)
  monkeypatch.setattr(cruise, 'CommandReader', MemoryReader)
  CP = car.CarParams(brand='gm', carFingerprint='CHEVROLET_TRAVERSE',
                    pcmCruise=False, openpilotLongitudinalControl=True)
  helper = VCruiseCarrot(CP)
  helper.cruise_state_available_last = True
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 80
  helper._cruise_speed_initialized = initialized
  helper._v_cruise_kph_at_brake = saved
  helper._gas_pressed_count = helper._brake_pressed_count = -2
  CS = car.CarState(canValid=True, gearShifter='drive', vEgo=70 / 3.6, vEgoCluster=70 / 3.6,
                    cruiseState={'available': True})
  sm = SubMaster(CS)
  sm['carControl'].enabled = False
  gm = SimpleNamespace(CP=CP)

  for frame in range(56):
    CS.buttonEvents = ([{'type': 'accelCruise', 'pressed': True}] if frame == 0 else
                       [{'type': 'accelCruise', 'pressed': False}] if frame == 55 else [])
    CS.buttonEnable = CarState.update_button_enable(gm, CS.buttonEvents)
    sm['carControl'].enabled = frame > 0
    helper.update_v_cruise(CS, sm, True)
    assert helper.v_cruise_kph == expected, frame
    assert helper.v_cruise_cluster_kph == expected
    assert helper._v_cruise_kph_at_brake == 0
    assert helper._cruise_speed_initialized


def test_gm_resume_gesture_does_not_reengage_after_intervening_disengagement():
  helper, CS, CC = make_remote_helper(None)
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': True}]
  CS.buttonEnable = True
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  CS.buttonEvents = []
  CS.buttonEnable = False
  CC.enabled = True
  assert helper._update_cruise_buttons(CS, CC, 80) == 80

  CC.enabled = False
  helper._cruise_cancel_state = True
  helper.autoCruiseControl_cancel_timer = 100
  calls = []
  helper._cruise_control = lambda *args, **kwargs: calls.append(args)
  for _ in range(120):
    assert helper._update_cruise_buttons(CS, CC, 80) == 80
    assert helper._cruise_cancel_state
    assert helper.autoCruiseControl_cancel_timer == 100
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': False}]
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert helper._cruise_cancel_state
  assert helper.autoCruiseControl_cancel_timer == 100
  assert calls == []

  # After releasing the resume gesture, an engaged long RES retains its +10 step.
  CC.enabled = True
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': True}]
  speed = helper._update_cruise_buttons(CS, CC, 80)
  CS.buttonEvents = []
  for _ in range(51):
    speed = helper._update_cruise_buttons(CS, CC, speed)
  assert speed == 90
  CS.buttonEvents = [{'type': 'accelCruise', 'pressed': False}]
  assert helper._update_cruise_buttons(CS, CC, speed) == 90


@pytest.mark.parametrize('physical', ['setCruise', 'resumeCruise', 'accelCruise', 'decelCruise'])
@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('mode', [0, 2])
@pytest.mark.parametrize('long_press', [False, True])
def test_vw_set_resume_are_distinct_from_speed_adjustment(physical, enabled, mode, long_press):
  from opendbc.car.volkswagen.carstate import CarState
  from opendbc.car.volkswagen.values import CAR

  candidate = CAR.VOLKSWAGEN_ID4_MK1
  vw = CarState(car.CarParams(carFingerprint=candidate, flags=int(candidate.config.flags), transmissionType='direct'))
  buttons = vw.CCP.BUTTONS
  cp = SimpleNamespace(vl={})
  for button in buttons:
    cp.vl.setdefault(button.can_addr, {})[button.can_msg] = 0
  source = next(button for button in buttons if button.event_type == getattr(ButtonType, physical))
  helper, CS, CC = make_remote_helper(None, enabled=enabled)
  helper._cruise_button_mode = mode
  helper._v_cruise_kph_at_brake = 95
  helper._cruise_speed_initialized = False
  helper._cruise_cancel_state = True
  helper.autoCruiseControl_cancel_timer = 100
  speed = 80

  for pressed in [True] * (52 if long_press else 3) + [False]:
    cp.vl[source.can_addr][source.can_msg] = source.values[0] if pressed else 0
    CS.buttonEvents = vw.create_button_events(cp, buttons)
    speed = helper._update_cruise_buttons(CS, CC, speed)

  increase = physical in ('resumeCruise', 'accelCruise')
  if physical == 'setCruise':
    expected = 70
  elif physical == 'resumeCruise':
    expected = 95
  elif long_press:
    expected = 90 if increase else 70
  elif not enabled:
    expected = 95 if increase else 70
  elif mode == 0:
    expected = 81 if increase else 79
  else:
    expected = 90 if increase else 70
  assert speed == expected
  assert helper._v_cruise_kph_at_brake == 0
  assert not helper._cruise_cancel_state
  assert helper.autoCruiseControl_cancel_timer == 0
  assert helper.button_cnt == 0
  if not long_press or physical in ('setCruise', 'resumeCruise'):
    assert helper._lat_enabled
    assert helper._cruise_speed_initialized
  assert vw.update_button_enable(CS.buttonEvents) is (physical in ('setCruise', 'resumeCruise'))

  # The real release must not add another short-press speed step after a hold.
  CS.buttonEvents = vw.create_button_events(cp, buttons)
  assert helper._update_cruise_buttons(CS, CC, speed) == speed
  assert not vw.update_button_enable(CS.buttonEvents)


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('initialized,saved,expected', [(True, 0, 80), (True, 60, 60), (True, 95, 95), (False, 0, 70)])
def test_dedicated_resume_restores_saved_speed_without_increment(enabled, initialized, saved, expected):
  helper, CS, CC = make_remote_helper(None, enabled=enabled)
  helper._cruise_speed_initialized = initialized
  helper._v_cruise_kph_at_brake = saved
  CS.buttonEvents = [{'type': 'resumeCruise', 'pressed': False}]
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper._cruise_speed_initialized
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize('physical', ['setCruise', 'resumeCruise'])
def test_dedicated_set_resume_hold_is_one_action_on_release(physical):
  helper, CS, CC = make_remote_helper(None, enabled=True)
  helper._v_cruise_kph_at_brake = 95
  CS.cruiseSpeedBigStep = True
  CS.buttonEvents = [{'type': physical, 'pressed': True}]
  speed = helper._update_cruise_buttons(CS, CC, 80)
  assert speed == 80
  CS.buttonEvents = []
  for _ in range(200):
    assert helper._update_cruise_buttons(CS, CC, speed) == 80
  CS.buttonEvents = [{'type': physical, 'pressed': False}]
  assert helper._update_cruise_buttons(CS, CC, speed) == (70 if physical == 'setCruise' else 95)
  assert helper.button_cnt == 0


@pytest.mark.parametrize('physical', ['setCruise', 'resumeCruise'])
def test_dedicated_buttons_leave_hold_ready_and_carrot_modes(physical):
  helper, CS, CC = make_remote_helper(None, enabled=True)
  helper._soft_hold_active = 2
  helper._cruise_ready = helper._paddle_decel_active = helper.carrot_cruise_active = True
  CS.buttonEvents = [{'type': physical, 'pressed': False}]
  assert helper._update_cruise_buttons(CS, CC, 80) == (70 if physical == 'setCruise' else 80)
  assert helper._soft_hold_active == 0
  assert not helper._cruise_ready
  assert not helper._paddle_decel_active
  assert not helper.carrot_cruise_active
  assert helper._lat_enabled


@pytest.mark.parametrize('physical,expected', [('setCruise', 70), ('resumeCruise', 95)])
def test_explicit_set_resume_speed_wins_over_same_frame_automatic_speed(physical, expected):
  helper, CS, CC = make_remote_helper(None, enabled=False)
  helper._v_cruise_kph_at_brake = 95
  CS.buttonEvents = [{'type': physical, 'pressed': False}]

  def automatic_update(CS, CC, speed):
    helper._activate_cruise = -1
    return 40

  helper._update_cruise_state = automatic_update
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper._activate_cruise == -1


@pytest.mark.parametrize('action', ['accelCruise', 'decelCruise', 'accelCruiseLong', 'decelCruiseLong'])
@pytest.mark.parametrize('auto_cruise', [0, 1])
def test_remote_cruise_buttons_request_engagement_independently_of_auto_cruise(action, auto_cruise):
  helper, CS, CC = make_remote_helper(action)
  helper.autoCruiseControl = auto_cruise
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == 1
  assert helper._lat_enabled
  assert helper.button_cnt == 0
  assert not CS.buttonEvents


@pytest.mark.parametrize('block', ['brake', 'gas', 'hold', 'steering', 'gear', 'can', 'unavailable', 'physical_cancel', 'negative_request'])
@pytest.mark.parametrize('action', ['accelCruise', 'decelCruiseLong'])
def test_remote_engagement_preserves_interlocks_and_physical_priority(block, action):
  helper, CS, CC = make_remote_helper(action)
  CS.brakePressed = block == 'brake'
  CS.gasPressed = block == 'gas'
  helper._hold_interlock_active = block == 'hold'
  helper._steering_interlock_active = block == 'steering'
  if block == 'gear':
    CS.gearShifter = 'park'
  if block == 'can':
    CS.canValid = False
  if block == 'unavailable':
    CS.cruiseState.available = helper._cruise_available = False
  if block == 'physical_cancel':
    CS.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  if block == 'negative_request':
    helper._activate_cruise = -1
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise <= 0


@pytest.mark.parametrize('block', [None, 'doorOpen', 'seatbeltUnlatched', 'accFaulted'])
def test_remote_engagement_reaches_existing_state_machine_without_bypassing_no_entry(block):
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents
  from openpilot.selfdrive.selfdrived.state import StateMachine
  helper, CS, CC = make_remote_helper('accelCruise')
  helper.autoCruiseControl = 0
  if block:
    setattr(CS, block, True)
  helper._update_cruise_buttons(CS, CC, 80)
  CS.activateCruise = helper._activate_cruise  # card.py publishes this existing field.
  car_events = CarSpecificEvents(SimpleNamespace(pcmCruise=False, openpilotLongitudinalControl=True))
  events = car_events.create_common_events(CS, car.CarState(), pcm_enable=False)
  enabled, _ = StateMachine().update(events)
  assert enabled is (block is None)


@pytest.mark.parametrize('action,expected', [('accelCruiseLong', 90), ('decelCruiseLong', 80)])
def test_remote_long_speed_action_is_one_native_step_without_held_state(action, expected):
  helper, CS, CC = make_remote_helper(action, enabled=True)
  assert helper._update_cruise_buttons(CS, CC, 83) == expected
  assert helper.button_cnt == 0
  assert not helper.long_pressed
  assert helper._activate_cruise == 0
  helper.bluetooth_commands = SimpleNamespace(read=lambda **_: None)
  assert helper._update_cruise_buttons(CS, CC, expected) == expected


@pytest.mark.parametrize('action', ['accelCruise', 'decelCruise', 'accelCruiseLong', 'decelCruiseLong'])
def test_remote_repeat_cannot_reengage_disabled_cruise(action):
  helper, CS, CC = make_remote_helper(action, enabled=False)
  helper.bluetooth_commands.is_repeat = True
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert helper._activate_cruise == 0
  assert not helper._lat_enabled


def test_remote_gap_long_cycles_driving_mode_without_changing_gap():
  helper, CS, CC = make_remote_helper('gapAdjustCruiseLong', enabled=True)
  values = {'MyDrivingMode': 4, 'LongitudinalPersonality': 2}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert values == {'MyDrivingMode': 1, 'LongitudinalPersonality': 2}


def test_remote_lfa_short_and_long_follow_existing_distinct_handlers():
  helper, CS, CC = make_remote_helper('lfaButton', enabled=True)
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._lat_enabled
  helper.bluetooth_commands = SimpleNamespace(read=lambda **_: 'lfaButtonLong')
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper.useLaneLineSpeedApply == 50
  assert helper._lat_enabled


@pytest.mark.parametrize('action,lat_enabled', [('cancel', True), ('cancelLong', False)])
def test_remote_cancel_actions_disengage_even_with_auto_cruise_disabled(action, lat_enabled):
  helper, CS, CC = make_remote_helper(action, enabled=True)
  helper._lat_enabled = True
  helper.autoCruiseControl = 0
  helper.autoCruiseControl_cancel_timer = 2000
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == BLUETOOTH_CANCEL
  assert helper._lat_enabled is lat_enabled
  assert helper._cruise_cancel_state


def test_remote_set_keeps_soft_hold_cancel_behavior():
  helper, CS, CC = make_remote_helper('decelCruise')
  helper._soft_hold_active = 2
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == -1


@pytest.mark.parametrize('pcm', [False, True])
def test_remote_cancel_reaches_existing_disengagement_state_machine(pcm):
  from openpilot.cereal import log
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents
  from openpilot.selfdrive.selfdrived.state import StateMachine
  helper, CS, CC = make_remote_helper('cancelLong', enabled=True)
  helper._update_cruise_buttons(CS, CC, 80)
  CS.activateCruise = helper._activate_cruise
  CS.cruiseState.enabled = pcm
  previous = car.CarState(cruiseState={'enabled': pcm})
  car_events = CarSpecificEvents(SimpleNamespace(pcmCruise=pcm, openpilotLongitudinalControl=not pcm))
  events = car_events.create_common_events(CS, previous, pcm_enable=pcm)
  state = StateMachine()
  state.state = log.SelfdriveState.OpenpilotState.enabled
  enabled, _ = state.update(events)
  assert not enabled


def test_auto_hold_blocks_automatic_cruise_activation():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = True
  helper._steering_interlock_active = False
  helper._activate_cruise = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (test)")

  assert helper._activate_cruise == 0


def make_pedal_speed_helper(auto_cruise, speed=50, enabled=False):
  helper, CS, CC = make_cruise_helper(80, 0, False, enabled)
  helper._update_cruise_state = VCruiseCarrot._update_cruise_state.__get__(helper)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.params = SimpleNamespace(get_bool=lambda key: False)
  helper.autoCruiseControl = auto_cruise
  helper.autoGasTokSpeed = 0
  helper.autoGasCancelSpeed = 30
  helper.autoGasSyncSpeed = 1
  helper.disengage_on_accelerator = False
  helper.enabled_last = enabled
  helper.v_cruise_kph = 80
  helper.v_ego_kph_set = speed
  helper._gas_pressed_count = -10
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper.speedcam_gas_tok_consumed = False
  helper._brake_pressed_count = -10
  helper._soft_hold_count = 0
  helper.soft_hold_on_cancel = False
  helper.xState = 0
  helper.aTarget = 0
  helper.d_rel = helper.v_rel = helper.v_lead_kph = 0
  helper.desiredSpeed = 250
  helper.cruiseOnDist = 0
  helper.nRoadLimitSpeed = helper.nRoadLimitSpeed_last = 80
  helper.applyModelSpeed = helper.autoRoadSpeedAdjust = 0
  helper.autoSpeedUptoRoadSpeedLimit = 0.2
  helper.autoRoadSpeedLimitOffset = -1
  CS.vEgo = CS.vEgoCluster = speed / 3.6
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  return helper, CS, CC


@pytest.mark.parametrize('auto_cruise', [0, 1, 2, 3])
@pytest.mark.parametrize('scenario,speed,enabled,auto_speed,activation', [
  ('brake_release', 50, False, 50, 1),
  ('brake_signal', 20, False, 20, 1),
  ('gas_release', 50, False, 50, 1),
  ('gas_signal', 50, False, 50, -1),
  ('gas_tap_off', 90, False, 90, 1),
])
def test_pedal_release_speed_changes_require_auto_cruise(auto_cruise, scenario, speed, enabled, auto_speed, activation):
  helper, CS, CC = make_pedal_speed_helper(auto_cruise, speed, enabled)
  if scenario.startswith('brake'):
    helper._brake_pressed_count = -1
  else:
    helper._gas_pressed_count = -1
  helper._gas_tok = scenario.startswith('gas_tap')
  if scenario.endswith('signal'):
    helper.xState = 3
  if scenario == 'brake_signal':
    helper.autoGasTokSpeed = 30

  assert helper._update_cruise_state(CS, CC, 80) == (auto_speed if auto_cruise else 80)
  assert helper._activate_cruise == (activation if auto_cruise else 0)


@pytest.mark.parametrize('auto_cruise', [0, 1, 2, 3])
@pytest.mark.parametrize('set_speed,step,expected', [(80, 10, 90), (85, 10, 90), (90, 10, 100), (80, 5, 85)])
def test_engaged_gas_tap_changes_speed_without_automatic_activation(auto_cruise, set_speed, step, expected):
  helper, CS, CC = make_pedal_speed_helper(auto_cruise, 50, True)
  helper._cruise_speed_unit = step
  CS.gasPressed, CS.gas = True, 0.1
  for _ in range(20):
    helper._prepare_brake_gas(CS, CC)
    assert helper._update_cruise_state(CS, CC, set_speed) == set_speed

  CS.gasPressed, CS.gas = False, 0
  helper._prepare_brake_gas(CS, CC)
  assert helper._gas_tok
  assert helper._update_cruise_state(CS, CC, set_speed) == expected
  assert helper._activate_cruise == 0

  for _ in range(3):
    helper._prepare_brake_gas(CS, CC)
    assert not helper._gas_tok
    assert helper._update_cruise_state(CS, CC, expected) == expected
    assert helper._activate_cruise == 0


@pytest.mark.parametrize('pressed_frames', [0, 1, 20, 39, 40, 41, 100])
def test_engaged_gas_tap_keeps_existing_duration_boundary(pressed_frames):
  helper, CS, CC = make_pedal_speed_helper(0, 50, True)
  CS.gasPressed, CS.gas = True, 0.1
  for _ in range(pressed_frames):
    helper._prepare_brake_gas(CS, CC)
    assert helper._update_cruise_state(CS, CC, 80) == 80
  CS.gasPressed, CS.gas = False, 0
  helper._prepare_brake_gas(CS, CC)
  is_tap = 0 < pressed_frames < helper._gas_tok_timer
  assert helper._gas_tok == is_tap
  assert helper._update_cruise_state(CS, CC, 80) == (90 if is_tap else 80)
  assert helper._activate_cruise == 0


@pytest.mark.parametrize('auto_cruise', [0, 1, 2, 3])
@pytest.mark.parametrize('block', ['camera_skip', 'disengage_on_accelerator', 'below_start_speed'])
def test_engaged_gas_tap_preserves_speed_change_gates(auto_cruise, block):
  helper, CS, CC = make_pedal_speed_helper(auto_cruise, 50, True)
  helper.speedcam_gas_tok_consumed = block == 'camera_skip'
  helper.disengage_on_accelerator = block == 'disengage_on_accelerator'
  helper.autoGasTokSpeed = 60 if block == 'below_start_speed' else 0
  helper._gas_pressed_count = 20
  helper._prepare_brake_gas(CS, CC)
  assert helper._update_cruise_state(CS, CC, 80) == 80
  assert helper._activate_cruise == 0


def test_gas_tap_does_not_reactivate_or_change_speed_after_disengagement():
  helper, CS, CC = make_pedal_speed_helper(0, 90, True)
  helper._gas_pressed_count = 20
  CC.enabled = helper.enabled_last = False
  helper._prepare_brake_gas(CS, CC)
  assert helper._gas_tok
  assert helper._update_cruise_state(CS, CC, 80) == 80
  assert helper._activate_cruise == 0


@pytest.mark.parametrize('button,expected', [(ButtonType.accelCruise, 80), (ButtonType.decelCruise, 50)])
def test_manual_braking_keeps_set_and_resume_speed_until_explicit_button(button, expected):
  helper, CS, CC = make_pedal_speed_helper(0, 80, True)
  CS.brakePressed = True
  helper._prepare_brake_gas(CS, CC)
  assert helper._v_cruise_kph_at_brake == 80
  CC.enabled = helper.enabled_last = False
  CS.vEgo = CS.vEgoCluster = 50 / 3.6
  helper.v_ego_kph_set = 50
  assert helper._update_cruise_state(CS, CC, 80) == 80

  CS.brakePressed = False
  helper._prepare_brake_gas(CS, CC)
  assert helper._brake_pressed_count == -1
  assert helper._update_cruise_state(CS, CC, 80) == 80
  assert helper._v_cruise_kph_at_brake == 80
  assert helper._activate_cruise == 0

  helper._prepare_brake_gas(CS, CC)
  helper._prepare_buttons = lambda *args: (80, button, False)
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize('sync,speed,expected', [(0, 90, 80), (1, 90, 90), (1, 50, 80)])
def test_manual_gas_sync_setting_is_independent_of_auto_cruise(sync, speed, expected):
  helper, CS, CC = make_pedal_speed_helper(0, speed, True)
  helper.autoGasSyncSpeed = sync
  helper._gas_pressed_count = helper._gas_tok_timer + 1
  CS.gasPressed = True
  assert helper._update_cruise_state(CS, CC, 80) == expected


@pytest.mark.parametrize('xstate,lead', [(3, 0), (0, 10)])
def test_paddle_decel_speed_selection_is_unchanged(xstate, lead):
  helper, CS, CC = make_pedal_speed_helper(0, 50, True)
  helper._paddle_decel_active = True
  helper.xState, helper.d_rel = xstate, lead
  assert helper._update_cruise_state(CS, CC, 80) == 50
  assert not helper._paddle_decel_active


@pytest.mark.parametrize('button', [ButtonType.decelCruise, ButtonType.setCruise])
@pytest.mark.parametrize('metric', [False, True])
@pytest.mark.parametrize('limit,current,expected', [
  (80, 50, 90), (80, 90, 90), (80, 100, 100),
  (30, 20, 35), (30, 35, 35), (30, 40, 40),
  (50, 0, 60), (160, 140, 161),
  (0, 50, 50), (-1, 50, 50), (200, 50, 50),
])
def test_set_engagement_uses_fresh_road_limit_in_kph(button, metric, limit, current, expected):
  helper, CS, CC = make_remote_helper(None)
  helper.navigation_set_speed = True
  CS.buttonEnable = True
  helper.is_metric = metric
  helper.v_ego_kph_set = current
  helper._set_speed_road_limit = limit
  helper._prepare_buttons = lambda *args: (80, button, False)
  assert helper._update_cruise_buttons(CS, CC, 80) == expected


@pytest.mark.parametrize('button,expected', [(ButtonType.decelCruise, 79), (ButtonType.setCruise, 50),
                                         (ButtonType.accelCruise, 81), (ButtonType.resumeCruise, 80)])
def test_navigation_does_not_change_engaged_buttons(button, expected):
  helper, CS, CC = make_remote_helper(None, enabled=True)
  helper.navigation_set_speed = True
  CS.buttonEnable = True
  helper._set_speed_road_limit = 80
  helper.v_ego_kph_set = 50
  helper._prepare_buttons = lambda *args: (79 if button == ButtonType.decelCruise else 81, button, False)
  assert helper._update_cruise_buttons(CS, CC, 80) == expected


@pytest.mark.parametrize('button', [ButtonType.accelCruise, ButtonType.resumeCruise])
def test_resume_uses_saved_speed_instead_of_road_limit(button):
  helper, CS, CC = make_remote_helper(None)
  helper.navigation_set_speed = True
  CS.buttonEnable = True
  helper._set_speed_road_limit = 80
  helper._v_cruise_kph_at_brake = 70
  helper._prepare_buttons = lambda *args: (51, button, False)
  assert helper._update_cruise_buttons(CS, CC, 50) == 70


@pytest.mark.parametrize('block', ['pcm', 'lateral_only', 'can', 'unavailable', 'park', 'brake', 'gas', 'hold', 'parking_brake'])
def test_navigation_set_speed_does_not_bypass_engagement_inputs(block):
  helper, CS, _ = make_remote_helper(None)
  helper.navigation_set_speed = True
  helper.disengage_on_accelerator = True
  helper._set_speed_road_limit = 80
  helper.CP.pcmCruise = block == 'pcm'
  helper.CP.openpilotLongitudinalControl = block != 'lateral_only'
  CS.canValid = block != 'can'
  CS.cruiseState.available = block != 'unavailable'
  CS.gearShifter = 'park' if block == 'park' else 'drive'
  CS.brakePressed = block == 'brake'
  CS.gasPressed = block == 'gas'
  CS.brakeHoldActive = block == 'hold'
  CS.parkingBrake = block == 'parking_brake'
  assert helper._navigation_set_speed(CS) is None


def test_navigation_set_owns_engagement_frame_without_changing_no_navigation_behavior():
  helper, CS, CC = make_remote_helper(None)
  helper.navigation_set_speed = True
  CS.buttonEnable = True
  helper.v_ego_kph_set = 50
  helper._prepare_buttons = lambda *args: (79, ButtonType.decelCruise, False)
  calls = []
  helper._update_cruise_state = lambda *args: calls.append(args[2]) or 20
  helper._set_speed_road_limit = 80
  assert helper._update_cruise_buttons(CS, CC, 80) == 90
  assert calls == [90]
  helper._set_speed_road_limit = 0
  assert helper._update_cruise_buttons(CS, CC, 80) == 20


@pytest.mark.parametrize('brand,button,expected', [
  ('gm', ButtonType.decelCruise, 90),
  ('volkswagen', ButtonType.setCruise, 90),
  ('volkswagen', ButtonType.decelCruise, 50),
])
@pytest.mark.parametrize('option', [False, True])
def test_navigation_set_requires_actual_vehicle_enable_button(brand, button, expected, option):
  from opendbc.car.gm.carstate import CarState as GmCarState
  from opendbc.car.volkswagen.carstate import CarState as VwCarState
  cls = GmCarState if brand == 'gm' else VwCarState
  vehicle = cls.__new__(cls)
  vehicle.CP = SimpleNamespace(pcmCruise=False)
  helper, CS, CC = make_remote_helper(None)
  helper.navigation_set_speed = option
  helper._set_speed_road_limit = 80
  helper.v_ego_kph_set = 50
  speed = 80
  for pressed in (True, False):
    CS.buttonEvents = [{'type': button, 'pressed': pressed}]
    CS.buttonEnable = vehicle.update_button_enable(CS.buttonEvents)
    speed = helper._update_cruise_buttons(CS, CC, speed)
  assert speed == (expected if option else 50)
  assert helper._navigation_set_applied == (option and expected == 90)


@pytest.mark.parametrize('disengage_on_gas,expected', [(False, 90), (True, 50)])
def test_physical_navigation_set_keeps_existing_gas_override_policy(disengage_on_gas, expected):
  helper, CS, CC = make_remote_helper(None)
  helper.navigation_set_speed = True
  helper.disengage_on_accelerator = disengage_on_gas
  helper._set_speed_road_limit = 80
  helper.v_ego_kph_set = 50
  CS.buttonEnable = CS.gasPressed = True
  helper._prepare_buttons = lambda *args: (79, ButtonType.decelCruise, False)
  assert helper._update_cruise_buttons(CS, CC, 80) == expected


@pytest.mark.parametrize('gas,expected,activation', [(False, 90, 1), (True, 50, 0)])
def test_bluetooth_navigation_set_preserves_remote_gas_gate(gas, expected, activation):
  helper, CS, CC = make_remote_helper('decelCruise')
  helper._cruise_control = VCruiseCarrot._cruise_control.__get__(helper)
  helper.navigation_set_speed = True
  helper._set_speed_road_limit = 80
  helper.v_ego_kph_set = 50
  CS.gasPressed = gas
  assert not CS.buttonEnable
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper._activate_cruise == activation


def test_navigation_set_toggle_refreshes_live_without_changing_existing_speed():
  helper, CS, CC = make_remote_helper(None)
  values = {'NavigationSetSpeed': 0}
  helper.params = SimpleNamespace(get_bool=lambda key: bool(values.get(key, 0)),
                                  get_int=lambda key: int(values.get(key, 0)),
                                  get_float=lambda key: float(values.get(key, 0)))
  helper._set_speed_road_limit = helper.nRoadLimitSpeed = 80
  helper.v_ego_kph_set = 50
  helper.v_cruise_kph = 75
  CS.buttonEnable = True
  helper._prepare_buttons = lambda *args: (74, ButtonType.decelCruise, False)
  for frame, enabled, expected in [(0, False, 50), (10, True, 90), (20, False, 50)]:
    values['NavigationSetSpeed'] = int(enabled)
    helper.frame = frame
    helper.update_params(True)
    assert helper.navigation_set_speed is enabled
    assert helper.v_cruise_kph == 75
    assert helper._update_cruise_buttons(CS, CC, 75) == expected


def test_navigation_set_is_off_when_setting_is_missing():
  helper, CS, _ = make_remote_helper(None)
  helper._set_speed_road_limit = 80
  del helper.navigation_set_speed
  assert helper._navigation_set_speed(CS) is None


@pytest.mark.parametrize('first_available', [False, True])
def test_navigation_set_full_update_loop_and_live_toggle(monkeypatch, first_available):
  import openpilot.selfdrive.car.cruise as cruise
  from openpilot.selfdrive.carrot.server.services import params as web_params
  from openpilot.selfdrive.car.tests.test_toyota_pcm_mode0 import MemoryParams, MemoryReader, SubMaster

  monkeypatch.setattr(cruise, 'Params', MemoryParams)
  monkeypatch.setattr(cruise, 'CommandReader', MemoryReader)
  helper = VCruiseCarrot(car.CarParams(brand='gm', carFingerprint='CHEVROLET_TRAVERSE',
                                      pcmCruise=False, openpilotLongitudinalControl=True))
  helper.params.put_bool = helper.params.values.__setitem__
  monkeypatch.setattr(web_params, 'HAS_PARAMS', True)
  monkeypatch.setattr(web_params, 'Params', lambda: helper.params)
  monkeypatch.setattr(web_params, 'ParamKeyType', None)
  definition = {'min': 0, 'max': 1, 'default': 0, 'control': 'toggle'}
  helper.cruise_state_available_last = True
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 75
  helper._gas_pressed_count = helper._brake_pressed_count = -2
  CS = car.CarState(canValid=True, gearShifter='drive', vEgo=50 / 3.6, vEgoCluster=50 / 3.6,
                    cruiseState={'available': True})
  sm = SubMaster(CS)
  sm['carControl'].enabled = False
  sm['carrotMan'].externalNavigationActive = sm['carrotMan'].roadLimitSpeedValid = True
  assert not helper.navigation_set_speed

  def tick():
    helper.update_v_cruise(CS, sm, True)
    return helper.v_cruise_kph

  web_params.set_param_value('NavigationSetSpeed', 1, definition)
  for _ in range(10):
    assert tick() == 75  # Enabling the option alone does not retarget cruise.
  assert helper.navigation_set_speed
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': True}]
  tick()
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': False}]
  CS.buttonEnable = True
  helper.cruise_state_available_last = not first_available
  assert tick() == 90

  CS.buttonEvents = []
  CS.buttonEnable = False
  sm['carControl'].enabled = True
  web_params.set_param_value('NavigationSetSpeed', 0, definition)
  for _ in range(10):
    assert tick() == 90  # Disabling also leaves the active set speed untouched.
  assert not helper.navigation_set_speed
  sm['carControl'].enabled = False
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': True}]
  tick()
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': False}]
  CS.buttonEnable = True
  assert tick() == 50


def test_large_steering_angle_blocks_automatic_cruise_activation():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = True
  helper._activate_cruise = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (speed)")

  assert helper._activate_cruise == 0


@pytest.mark.parametrize(("cruise_available", "expected_activate"), [(False, 0), (True, 1)])
def test_cruise_availability_gates_automatic_activation(cruise_available, expected_activate):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = cruise_available
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = False
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_active = 0
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (test)")

  assert helper._activate_cruise == expected_activate


def test_soft_hold_does_not_arm_when_cruise_is_unavailable():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = False
  helper._cruise_cancel_state = False
  helper.autoCruiseControl_cancel_timer = 0

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=False),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == 0
  assert helper._soft_hold_active == 0


@pytest.mark.parametrize(("cancel_timer", "expected_count", "expected_active"), [
  (1, 0, 0),
  (0, 61, 1),
])
def test_post_shift_cancel_timer_gates_soft_hold(cancel_timer, expected_count, expected_active):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = False
  helper._cruise_cancel_state = False
  helper.autoCruiseControl_cancel_timer = cancel_timer

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=True),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == expected_count
  assert helper._soft_hold_active == expected_active


@pytest.mark.parametrize(("soft_hold_on_cancel", "expected_count", "expected_active"), [
  (False, 0, 0),
  (True, 61, 1),
])
def test_cancel_state_soft_hold_policy(soft_hold_on_cancel, expected_count, expected_active):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = soft_hold_on_cancel
  helper._cruise_cancel_state = True
  helper.autoCruiseControl_cancel_timer = 0

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=True),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == expected_count
  assert helper._soft_hold_active == expected_active


def test_soft_hold_on_cancel_keeps_cancel_state_while_engaging():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = True
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_active = 1
  helper.soft_hold_on_cancel = True
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper._add_log = lambda log: None

  helper._engage_soft_hold()

  assert helper._soft_hold_active == 2
  assert helper._cruise_cancel_state
  assert helper._activate_cruise == 1


@pytest.mark.parametrize(("cancel_state", "expected_activate"), [
  (False, 0),
  (True, -1),
])
def test_gas_releases_cancel_soft_hold_to_cruise_off(cancel_state, expected_activate):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = cancel_state
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_count = 0
  helper._soft_hold_active = 2
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper.disengage_on_accelerator = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper.carrot_cruise_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = -1
  helper._add_log = lambda log: None

  CS = SimpleNamespace(gasPressed=True, gas=0.2, brakePressed=False)
  helper._prepare_brake_gas(CS, car.CarControl(enabled=True))

  assert helper._soft_hold_active == 0
  assert helper._cruise_cancel_state is cancel_state
  assert helper._activate_cruise == expected_activate


@pytest.mark.parametrize(("brake_hold_active", "parking_brake", "active"), [
  (False, False, False),
  (True, False, True),
  (False, True, True),
  (True, True, True),
])
def test_cruise_hold_interlock_sources(brake_hold_active, parking_brake, active):
  CS = car.CarState(brakeHoldActive=brake_hold_active, parkingBrake=parking_brake)

  assert is_hold_interlock_active(CS) is active


@pytest.mark.parametrize("maximum,requested,expected", [
  (4, 4, 4), (3, 4, 3), (4, 2, 2), (3, 2, 2), (4, 3, 3),
  (3, 0, 3), (4, 0, 4), (0, 4, 3), (4, 99, 4), (3, 1, 2),
])
def test_gap_cycle_limits(maximum, requested, expected):
  assert cruise_gap_levels(requested, maximum) == expected


@pytest.mark.parametrize("maximum,requested,pcm_gap,expected", [
  (4, 2, 0, [1, 0, 1, 0]),
  (4, 2, 4, [1, 0, 1, 0]),
  (4, 3, 4, [2, 1, 0, 2]),
  (3, 2, 3, [1, 0, 1, 0]),
  (4, 4, 0, [3, 2, 1, 0]),
  (3, 4, 0, [2, 1, 0, 2]),
])
def test_gap_button_cycles_selected_levels(maximum, requested, pcm_gap, expected):
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, False)
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=True)
  values = {"LongitudinalPersonalityMax": maximum, "CruiseGapLevels": requested, "LongitudinalPersonality": 0}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  CS.pcmCruiseGap = pcm_gap
  for personality in expected:
    assert helper._update_cruise_buttons(CS, CC, 80) == 80
    assert values["LongitudinalPersonality"] == personality
  assert values["CruiseGapLevels"] == requested


@pytest.mark.parametrize("openpilot_long,requested,pcm_gap,current,expected", [
  (True, 2, 4, 3, 1),  # Reducing from TF4 enters TF2 on the next press.
  (True, 4, 3, 0, 2),  # Default preserves the vehicle-reported gap.
  (False, 2, 4, 0, 3),  # Stock ACC owns its gap cycle.
  (False, 2, 0, 0, 3),
  (True, 4, 9, 0, 3),  # Invalid OEM values cannot create an invalid enum.
])
def test_gap_button_reduction_and_oem_gap(openpilot_long, requested, pcm_gap, current, expected):
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, False)
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=openpilot_long)
  values = {"LongitudinalPersonalityMax": 4, "CruiseGapLevels": requested, "LongitudinalPersonality": current}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  CS.pcmCruiseGap = pcm_gap
  helper._update_cruise_buttons(CS, CC, 80)
  assert values["LongitudinalPersonality"] == expected


def test_gap_long_press_still_changes_driving_mode():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, True)
  values = {"MyDrivingMode": 4, "LongitudinalPersonality": 1}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  helper._update_cruise_buttons(CS, CC, 80)
  assert values == {"MyDrivingMode": 1, "LongitudinalPersonality": 1}
