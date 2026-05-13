import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget
# Kans
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False
    self._engageable: bool = False

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    #self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    # Kans:
    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/wheel.png', icon_size, icon_size)
    self._carrot_wheel: rl.Texture = gui_app.texture('icons/carrot.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed or not self._engageable else 255

    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)
    #rl.draw_texture(texture, center_x - texture.width // 2, center_y - texture.height // 2, self._white_color)
    # Kans: wheel rotate
    texture_wheel = self._txt_exp if self._held_or_actual_mode() else self._txt_wheel
    carrot_wheel = self._carrot_wheel
    # Always visible (no hide). We keep filters but drive them to stable values.
    self._wheel_alpha_filter.update(255 * 0.95)
    self._wheel_y_filter.update(0)
    # Color: green if lat_active else white
    if ui_state.lat_active:
      wheel_color = rl.Color(0, 255, 0, int(self._wheel_alpha_filter.x))     # green
    else:
      wheel_color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x)) # white

    src_rect = rl.Rectangle(0, 0, texture_wheel.width, texture_wheel.height)
    dest_rect = rl.Rectangle(center_x, center_y, texture_wheel.width, texture_wheel.height)
    origin = (texture_wheel.width / 2, texture_wheel.height / 2)

    rl.draw_texture_pro(texture_wheel, src_rect, dest_rect, origin, -ui_state.angleSteers, wheel_color)
    rl.draw_texture_pro(carrot_wheel, src_rect, dest_rect, origin, -ui_state.angleSteers, rl.Color(255, 255, 255, 250))

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._experimental_mode

  def _is_toggle_allowed(self):
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    # Mirror exp mode toggle using persistent car params
    return ui_state.has_longitudinal_control
