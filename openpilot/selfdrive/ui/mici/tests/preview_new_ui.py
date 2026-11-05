"""Offline native Wayon HUD preview. Synthetic inputs, no device or publishers."""
import argparse
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS

import pyray as rl


def main():
  cases = ('set-speed', 'steady', 'disengaged', 'bsd', 'critical', 'parking', 'hold', 'three-digits', 'lateral-only',
           'cam-active', 'cam-dim', 'cam-idle', 'route-active', 'route-idle',
           'turn-active', 'section-active', 'cap-max', 'cap-warning', 'cap-timer',
           'bsm-left', 'bsm-right', 'bsm-both', 'bsm-max', 'bsm-critical', 'bsm-timer',
           'bsm-fade-in', 'bsm-fade-out', 'bsm-cleared', 'bsm-stale',
           'dm-face', 'dm-yaw', 'dm-warning', 'dm-phone', 'dm-stale')
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--case', choices=cases, action='append', help='Render only these cases (repeatable).')
  parser.add_argument('--wayon-dm-reference', type=Path, help='Compare using an original Wayon driver_state.py (DM cases only).')
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)

  class SM(dict):
    valid = dict.fromkeys(('carState', 'selfdriveState', 'carControl', 'carrotMan', 'longitudinalPlan',
                          'driverMonitoringState', 'driverStateV2'), True)
    alive = valid.copy()
    recv_frame = dict.fromkeys(valid, 101)
    seen = valid.copy()
    recv_time = dict.fromkeys(valid, time.monotonic())
    logMonoTime = dict.fromkeys(valid, int(time.monotonic() * 1e9))
    def all_checks(self, services):
      return all(self.valid[s] and self.alive[s] for s in services)
  from openpilot.cereal import log
  dm = log.DriverMonitoringState.new_message(activePolicy='vision')
  dm.visionPolicyState.faceDetected = True
  dm.visionPolicyState.awarenessPercent = 100
  car = NS(leftBlindspot=False, rightBlindspot=False, steeringAngleDeg=4, vCruise=100, gasPressed=False, brakePressed=False)
  sm = SM(carState=car, carControl=NS(latActive=False, longActive=True), selfdriveState=NS(enabled=True, alertSize=log.SelfdriveState.AlertSize.none),
          controlsState=NS(lateralControlState=NS(which=lambda: 'torqueState')), carOutput=NS(actuatorsOutput=NS(torque=-.45)),
          carrotMan=NS(desiredSpeed=250, desiredSource='none'), longitudinalPlan=NS(cruiseTarget=100),
          driverMonitoringState=dm, driverStateV2=log.DriverStateV2.new_message(frameId=100))
  ui = NS(started=True, started_frame=100, status='engaged', is_metric=True, sm=sm, new_ui=True, share_data=False,
          always_on_dm=False, is_onroad=lambda: True)
  # Inject only UI inputs; keep real widgets, fonts, textures, filters and GPU draw calls.
  state_module = ModuleType('openpilot.selfdrive.ui.ui_state')
  state_module.ui_state = ui
  state_module.UIStatus = NS(DISENGAGED='disengaged', ENGAGED='engaged')
  sys.modules[state_module.__name__] = state_module
  from openpilot.system.ui.lib.application import FontWeight, gui_app
  from openpilot.selfdrive.ui.mici.onroad.wayon_hud import WayonHud
  from openpilot.selfdrive.ui.mici.onroad.status_timer_renderer import StatusTimerRenderer
  from openpilot.selfdrive.ui.mici.onroad.status_timers import TimerDisplay
  from openpilot.selfdrive.ui.mici.onroad.vision_renderer import VisionRenderer
  from openpilot.selfdrive.ui.mici.onroad.wayon_driver_preview import WayonDriverPreview

  reference = None
  if args.wayon_dm_reference:
    assert args.case and all(c.startswith('dm-') and c != 'dm-stale' for c in args.case)
    spec = importlib.util.spec_from_file_location('_wayon_original_dm', args.wayon_dm_reference)
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)

  gui_app.init_window('C4 new ui native preview', font_weights=[FontWeight.DISPLAY, FontWeight.SEMI_BOLD,
                                                            FontWeight.KOREAN, FontWeight.UNIFONT])
  real_clock = rl.get_time
  preview_time = [10.]
  rl.get_time = lambda: preview_time[0]
  try:
    renderer = WayonHud()
    timer = StatusTimerRenderer()
    vision = VisionRenderer()
    hud = NS(_can_draw_top_icons=True, _status_timer_visible=False, _show_wheel_critical=False,
             is_cruise_set=True, _engaged=True, set_speed=100, _set_speed_changed_time=rl.get_time(),
             _font_display=gui_app.font(FontWeight.DISPLAY), _font_semi_bold=gui_app.font(FontWeight.SEMI_BOLD),
             _turn_intent=NS(render=lambda _: None))
    rect = rl.Rectangle(0, 0, 536, 240)
    selected_cases = args.case or cases
    for case in selected_cases:
      driver = (reference.DriverStateRenderer() if reference else WayonDriverPreview()) if case.startswith('dm-') else None
      dm.visionPolicyState.distractedTypes.phone = case == 'dm-phone'
      dm.visionPolicyState.awarenessPercent = 90 if case == 'dm-warning' else 100
      dm.visionPolicyState.pose.yaw = .25 if case == 'dm-yaw' else 0
      preview_time[0] = 10.
      renderer.reset()
      sm.alive['carState'] = True
      ui.status = 'disengaged' if case in ('disengaged', 'lateral-only') else 'engaged'
      hud._engaged = case not in ('disengaged', 'lateral-only')
      sm['carControl'].latActive = case == 'lateral-only' or case.endswith('active') or case.startswith('bsm-')
      sm['selfdriveState'].enabled = hud._engaged
      sm['carControl'].longActive = hud._engaged and case != 'cam-idle'
      hud._status_timer_visible = case in ('parking', 'hold', 'cap-timer', 'bsm-timer')
      hud._show_wheel_critical = case in ('critical', 'bsm-critical')
      hud._can_draw_top_icons = case not in ('critical', 'cap-warning', 'bsm-critical')
      hud.set_speed = 254 if case == 'three-digits' else 100
      hud._set_speed_changed_time = rl.get_time() if case in ('set-speed', 'three-digits', 'cap-max', 'bsm-max') else -10
      car.leftBlindspot = case == 'bsd' or case.startswith('bsm-') and case != 'bsm-right'
      car.rightBlindspot = case == 'bsd' or case.startswith('bsm-') and case != 'bsm-left'
      cap = case.startswith(('cam-', 'cap-', 'bsm-')) or case in ('turn-active', 'section-active')
      sm['carrotMan'].desiredSpeed = 60 if cap else 250
      sm['carrotMan'].desiredSource = 'atc' if case == 'turn-active' else 'section' if case == 'section-active' else 'cam'
      sm['longitudinalPlan'].cruiseTarget = 60 if cap else 100
      car.vCruise = 35 if case == 'route-idle' else 100
      if case.startswith('route-'):
        sm['carrotMan'].desiredSource = 'route'
        sm['carrotMan'].desiredSpeed = 180 if case == 'route-idle' else 60
        sm['longitudinalPlan'].cruiseTarget = 35 if case == 'route-idle' else 60
      frames = 1 if case == 'bsm-fade-in' else 41 if case in ('bsm-fade-out', 'bsm-stale') else 60 if case == 'bsm-cleared' else 40
      for frame in range(frames):
        if frame >= 40:
          if case in ('bsm-fade-out', 'bsm-cleared'):
            car.leftBlindspot = car.rightBlindspot = False
          elif case == 'bsm-stale':
            sm.alive['carState'] = False
        sm.recv_time['carrotMan'] = time.monotonic()
        sm.logMonoTime = dict.fromkeys(sm.logMonoTime, 0 if case == 'dm-stale' else int(time.monotonic() * 1e9))
        sm['driverStateV2'].frameId += 1
        preview_time[0] = 10.7 if case == 'cam-dim' and frame > 0 else 10.
        rl.begin_drawing()
        rl.clear_background(rl.Color(22, 24, 26, 255))
        renderer.draw(rect, hud)
        vision.render(rect)
        if driver is not None:
          if reference:
            driver.set_should_draw(True)
            driver.set_position(16, 10)
            driver.render()
          else:
            driver.draw_onroad(rect, False)
        if hud._status_timer_visible:
          timer.draw(rect, TimerDisplay('parking' if case in ('cap-timer', 'bsm-timer') else case, 83), 1.)
        if frame == frames - 1:
          # Read the frame just drawn, not the old back buffer after swapping.
          rl.rl_draw_render_batch_active()
          image = rl.load_image_from_screen()
          try:
            assert image.width == 536 and image.height == 240
            assert rl.export_image(image, str(args.output / f'{case}.png'))
          finally:
            rl.unload_image(image)
        rl.end_drawing()
    print(f'PASS: {len(selected_cases)} native 536x240 HUD frames; isolated synthetic inputs, not a vehicle capture')
  finally:
    rl.get_time = real_clock
    gui_app.close()


if __name__ == '__main__':
  main()
