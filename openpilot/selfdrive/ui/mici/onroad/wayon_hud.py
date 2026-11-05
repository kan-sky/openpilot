"""Wayon C4 presentation; all vehicle and alert decisions stay in their owners."""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.mici.onroad.deceleration_badge import DecelerationBadge, selected_deceleration
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached


class WayonHud:
  def __init__(self):
    self._torque_bar = TorqueBar()
    self._deceleration_badge = DecelerationBadge()
    self._blind_spots = BlindSpotIndicators()
    self._wheel = gui_app.texture("icons_mici/wheel.png", 50, 50)
    self._critical_wheel = gui_app.texture("icons_mici/wheel_critical.png", 50, 50)
    self._exclamation = gui_app.texture("icons_mici/exclamation_point.png", 9, 44)
    self._speed_alpha = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._wheel_alpha = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self.reset()

  def reset(self):
    self._deceleration_badge.reset()
    self._blind_spots.reset()
    self._speed_alpha.x = self._wheel_alpha.x = self._wheel_y.x = 0
    self._torque_bar._torque_filter.x = self._torque_bar._torque_line_alpha_filter.x = 0
    self._torque_bar.lateral_active = False
    self._started_frame = ui_state.started_frame

  def drawing_top_icons(self) -> bool:
    return self._speed_alpha.x > 1e-2

  def draw(self, rect: rl.Rectangle, hud):
    if self._started_frame != ui_state.started_frame:
      self.reset()
    sm = ui_state.sm
    fresh = ui_state.started and all(
      sm.valid[s] and sm.alive[s] and sm.recv_frame[s] >= ui_state.started_frame
      for s in ("carState", "selfdriveState")
    )
    # BSM only needs fresh carState; loss of the model/control stream must not hide it.
    self._blind_spots.update()
    bsm_visible = self._blind_spots.detected

    # Carrot can steer with ACC disabled; preserve its active-steering indication.
    lateral_active = fresh and sm.valid['carControl'] and sm.alive['carControl'] and \
      sm.recv_frame['carControl'] >= ui_state.started_frame and sm['carControl'].latActive
    self._torque_bar.lateral_active = lateral_active
    bottom_clear = fresh and not hud._status_timer_visible and not hud._show_wheel_critical
    if bottom_clear:
      self._torque_bar.render(rect)
    else:
      self._torque_bar._torque_filter.x = self._torque_bar._torque_line_alpha_filter.x = 0

    # Warnings/timers take this space immediately, not after a fade-out.
    clear = hud._can_draw_top_icons and not hud._status_timer_visible
    if fresh and clear and not self._blind_spots.left_visible:
      show_speed = hud.is_cruise_set and hud._engaged and 0 <= rl.get_time() - hud._set_speed_changed_time < 2.5
      self._speed_alpha.update(show_speed)
    else:
      self._speed_alpha.x = 0
    if self.drawing_top_icons():
      self._draw_set_speed(rect, hud)

    deceleration = selected_deceleration(sm, ui_state.started_frame) if fresh and clear and not hud._show_wheel_critical else None
    self._deceleration_badge.draw(rect, hud._font_semi_bold, deceleration, rl.get_time())

    car_state = sm["carState"]
    critical = hud._show_wheel_critical
    if not critical and not hud._status_timer_visible:
      self._blind_spots.render(rect)
    # The original Wayon icons replace the ordinary wheel; critical warnings win.
    show_wheel = not bsm_visible and (lateral_active or (fresh and ui_state.status != UIStatus.DISENGAGED))
    self._wheel_alpha.update(255 if critical else 230 if show_wheel else 0)
    self._wheel_y.update(0 if critical or show_wheel else 25)
    if not fresh and not critical:
      return
    wheel = self._critical_wheel if critical else self._wheel
    cx, cy = rect.x + 46, rect.y + rect.height - 39 + self._wheel_y.x
    if bottom_clear and not bsm_visible:
      hud._turn_intent.render(rl.Rectangle(cx - 50, cy - 50, 100, 100))
    color = rl.Color(0, 255, 0, int(self._wheel_alpha.x)) if lateral_active and not critical else \
      rl.Color(255, 255, 255, int(self._wheel_alpha.x))
    rl.draw_texture_pro(wheel, rl.Rectangle(0, 0, wheel.width, wheel.height),
                        rl.Rectangle(cx, cy, wheel.width, wheel.height), rl.Vector2(25, 25),
                        -car_state.steeringAngleDeg if fresh else 0,
                        color)
    if critical:
      rl.draw_texture_ex(self._exclamation, rl.Vector2(cx + 30.5, cy - 22), 0, 1, rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle, hud):
    alpha = self._speed_alpha.x
    speed = hud.set_speed * (1 if ui_state.is_metric else 0.621371)
    text = str(round(speed))
    size = 112
    measured = measure_text_cached(hud._font_display, text, size)
    size *= min(1, 190 / max(1, measured.x))
    rl.draw_circle_gradient(rl.Vector2(rect.x + 81, rect.y + 81), 81, rl.Color(0, 0, 0, int(127 * alpha)), rl.BLANK)
    color = rl.Color(255, 255, 255, int(230 * alpha))
    rl.draw_text_ex(hud._font_display, text, rl.Vector2(rect.x + 17, rect.y - 4), size, 0, color)
    rl.draw_text_ex(hud._font_semi_bold, tr("MAX"), rl.Vector2(rect.x + 25, rect.y + 109), 36, 0, color)
