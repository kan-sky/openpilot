"""Wayon/sunnypilot BSM presentation with current-session input validation.

Adapted from sunnypilot, Copyright (c) 2021-, Haibin Wen,
sunnypilot, and a number of other contributors.
See docs/third_party/sunnypilot-bsm-license.md for the upstream license.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app


class BlindSpotIndicators:
  def __init__(self):
    self._left = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 108, 128)
    self._right = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 108, 128, flip_x=True)
    self._left_alpha = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)
    self._right_alpha = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)
    self.reset()

  def reset(self):
    self._left_alpha.x = self._right_alpha.x = 0
    self._started_frame = ui_state.started_frame

  def update(self):
    if self._started_frame != ui_state.started_frame:
      self.reset()
    sm = ui_state.sm
    if not (ui_state.started and sm.valid['carState'] and sm.alive['carState'] and
            sm.recv_frame['carState'] > 0 and sm.recv_frame['carState'] >= ui_state.started_frame):
      self.reset()
      return
    self._left_alpha.update(float(sm['carState'].leftBlindspot))
    self._right_alpha.update(float(sm['carState'].rightBlindspot))

  @property
  def left_visible(self) -> bool:
    return self._left_alpha.x > 0.01

  @property
  def detected(self) -> bool:
    return self.left_visible or self._right_alpha.x > 0.01

  def render(self, rect: rl.Rectangle):
    for texture, alpha, x in ((self._left, self._left_alpha.x, rect.x + 20),
                              (self._right, self._right_alpha.x, rect.x + rect.width - 20 - self._right.width)):
      if alpha > 0.01:
        rl.draw_texture_ex(texture, rl.Vector2(int(x), int(rect.y + 100)), 0., 1., rl.Color(255, 255, 255, int(255 * alpha)))
