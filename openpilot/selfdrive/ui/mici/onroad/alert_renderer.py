import math
import time
from enum import StrEnum
from typing import NamedTuple
import pyray as rl
import random
import string
from dataclasses import dataclass
from openpilot.cereal import messaging, log, car
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.filter_simple import BounceFilter, FirstOrderFilter
from openpilot.system.hardware import TICI
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.selfdrive.selfdrived.alert_presentation import font_sizes
from openpilot.selfdrive.ui.mici.onroad.status_timers import StatusTimers, TimerDisplay
from openpilot.selfdrive.ui.mici.onroad.status_timer_renderer import StatusTimerRenderer

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus

ALERT_MARGIN = 18

ALERT_FONT_SMALL = 66 - 50
ALERT_FONT_BIG = 88 - 40

SELFDRIVE_STATE_TIMEOUT = 5  # Seconds
SELFDRIVE_UNRESPONSIVE_TIMEOUT = 10  # Seconds

# Constants
ALERT_COLORS = {
  AlertStatus.normal: rl.Color(0, 0, 0, 255),
  AlertStatus.userPrompt: rl.Color(255, 115, 0, 255),
  AlertStatus.critical: rl.Color(255, 0, 21, 255),
}

TURN_SIGNAL_BLINK_PERIOD = 1 / (80 / 60)  # Mazda heartbeat turn signal BPM

DEBUG = False


class IconSide(StrEnum):
  left = 'left'
  right = 'right'


class IconLayout(NamedTuple):
  texture: rl.Texture
  side: IconSide
  margin_x: int
  margin_y: int


class AlertLayout(NamedTuple):
  text_rect: rl.Rectangle
  icon: IconLayout | None


@dataclass
class Alert:
  text1: str = ""
  text2: str = ""
  size: int = 0
  status: int = 0
  visual_alert: int = car.CarControl.HUDControl.VisualAlert.none
  alert_type: str = ""


# Pre-defined alert instances
ALERT_STARTUP_PENDING = Alert(
  text1="openpilot Unavailable",
  text2="Waiting to start",
  size=AlertSize.mid,
  status=AlertStatus.normal,
)

ALERT_CRITICAL_TIMEOUT = Alert(
  text1="TAKE CONTROL IMMEDIATELY",
  text2="System Unresponsive",
  size=AlertSize.full,
  status=AlertStatus.critical,
)

ALERT_CRITICAL_REBOOT = Alert(
  text1="System Unresponsive",
  text2="Reboot Device",
  size=AlertSize.full,
  status=AlertStatus.critical,
)


class AlertRenderer(Widget):
  def __init__(self):
    super().__init__()

    self._alert_text1_label = UnifiedLabel(text="", font_size=ALERT_FONT_BIG, font_weight=FontWeight.DISPLAY, elide=False)
    self._alert_text2_label = UnifiedLabel(text="", font_size=ALERT_FONT_SMALL, font_weight=FontWeight.ROMAN, elide=False)
    self._text_layout_key = None
    self._text_layout_heights = (0.0, 0.0)

    self._prev_alert: Alert | None = None
    self._text_gen_time = 0
    self._alert_text2_gen = ''

    # animation filters
    # TODO: use 0.1 but with proper alert height calculation
    self._alert_y_filter = BounceFilter(0, 0.1, 1 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)

    self._turn_signal_timer = 0.0
    self._turn_signal_alpha_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)
    self._last_icon_side: IconSide | None = None

    self._status_timers = StatusTimers()
    self._status_timer_renderer = StatusTimerRenderer()
    self._last_started_frame = -1
    self.timer_visible = False
    self._pending_presentation = None
    ui_state.add_offroad_transition_callback(self._reset_timers)

    self._load_icons()

  def _reset_timers(self):
    self._status_timers.reset()
    self._prev_alert = None
    self._alpha_filter.x = 0.
    self.timer_visible = False
    self._pending_presentation = None

  def _presentation(self):
    if self._last_started_frame != ui_state.started_frame or not ui_state.started:
      self._last_started_frame = ui_state.started_frame
      self._reset_timers()
    alert = self.get_alert(ui_state.sm)
    timer = self._timer_display(alert)
    self.timer_visible = timer is not None
    if self.timer_visible:
      self._prev_alert = None
    return alert, timer

  def _timer_display(self, alert: Alert | None) -> TimerDisplay | None:
    sm = ui_state.sm
    now = time.monotonic()

    def fresh_messages(names, max_age):
      return all(sm.valid[name] and sm.alive[name] and sm.recv_frame[name] >= ui_state.started_frame and
                 0 <= now - sm.recv_time[name] <= max_age for name in names)

    fresh = fresh_messages(('carState', 'selfdriveState'), 1.)
    cs = sm['carState']
    ss = sm['selfdriveState']
    cc = sm['carControl']
    gear = car.CarState.GearShifter
    # GM can clear the stock standstill flag. Read the current commanded stop
    # for display only; never change that flag or the vehicle's steering limit.
    commanded_hold = (
      fresh_messages(('carState', 'selfdriveState', 'carControl'), .25) and
      cs.standstill and abs(cs.vEgo) <= .1 and not cs.gasPressed and not cs.brakePressed and
      cs.gearShifter in (gear.drive, gear.sport, gear.low, gear.eco, gear.manumatic) and
      ss.enabled and ss.active and cc.enabled and cc.longActive and
      cc.actuators.longControlState == car.CarControl.Actuators.LongControlState.stopping and
      math.isfinite(cc.actuators.accel) and cc.actuators.accel < 0.
    )
    return self._status_timers.update(
      now=now, started=ui_state.started, session=ui_state.started_frame, fresh=fresh,
      standstill=cs.standstill, parking_brake=cs.parkingBrake,
      hold_active=cs.brakeHoldActive or cs.softHoldActive == 2 or cs.cruiseState.standstill or commanded_hold,
      hold_without_alert=cs.brakeHoldActive or cs.softHoldActive == 2 or commanded_hold,
      alert_type=alert.alert_type if alert is not None else None,
      critical=alert is not None and alert.status == AlertStatus.critical,
    )

  def _load_icons(self):
    self._txt_turn_signal_left = gui_app.texture('icons_mici/onroad/turn_signal_left.png', 104, 96)
    self._txt_turn_signal_right = gui_app.texture('icons_mici/onroad/turn_signal_left.png', 104, 96, flip_x=True)
    self._txt_blind_spot_left = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 134, 150)
    self._txt_blind_spot_right = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 134, 150, flip_x=True)

  def get_alert(self, sm: messaging.SubMaster) -> Alert | None:
    """Generate the current alert based on selfdrive state."""
    ss = sm['selfdriveState']

    # Check if selfdriveState messages have stopped arriving
    if not sm.updated['selfdriveState']:
      recv_frame = sm.recv_frame['selfdriveState']
      time_since_onroad = time.monotonic() - ui_state.started_time

      # 1. Never received selfdriveState since going onroad
      waiting_for_startup = recv_frame < ui_state.started_frame
      if waiting_for_startup and time_since_onroad > 5:
        return ALERT_STARTUP_PENDING

      # 2. Lost communication with selfdriveState after receiving it
      if TICI and not waiting_for_startup:
        ss_missing = time.monotonic() - sm.recv_time['selfdriveState']
        if ss_missing > SELFDRIVE_STATE_TIMEOUT:
          if ss.enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < SELFDRIVE_UNRESPONSIVE_TIMEOUT:
            return ALERT_CRITICAL_TIMEOUT
          return ALERT_CRITICAL_REBOOT

    # No alert if size is none
    if ss.alertSize == 0:
      return None

    # Return current alert
    ret = Alert(text1=ss.alertText1, text2=ss.alertText2, size=ss.alertSize.raw, status=ss.alertStatus.raw,
                visual_alert=ss.alertHudVisual, alert_type=ss.alertType)
    self._prev_alert = ret
    return ret

  def will_render(self) -> tuple[Alert | None, bool]:
    self._pending_presentation = self._presentation()
    alert, _ = self._pending_presentation
    return alert or self._prev_alert, alert is None

  def _icon_helper(self, alert: Alert) -> AlertLayout:
    icon_side = None
    txt_icon = None
    icon_margin_x = 20
    icon_margin_y = 18

    # alert_type format is "EventName/eventType" (e.g., "preLaneChangeLeft/warning")
    event_name = alert.alert_type.split('/')[0] if alert.alert_type else ''

    if event_name == 'preLaneChangeLeft':
      icon_side = IconSide.left
      txt_icon = self._txt_turn_signal_left
      icon_margin_x = 2
      icon_margin_y = 5

    elif event_name == 'preLaneChangeRight':
      icon_side = IconSide.right
      txt_icon = self._txt_turn_signal_right
      icon_margin_x = 2
      icon_margin_y = 5

    elif event_name == 'laneChange':
      CS = ui_state.sm['carState']
      if CS.leftBlinker:
        icon_side = IconSide.left
      elif CS.rightBlinker:
        icon_side = IconSide.right
      else:
        icon_side = self._last_icon_side
      txt_icon = self._txt_turn_signal_left if self._last_icon_side == 'left' else self._txt_turn_signal_right
      icon_margin_x = 2
      icon_margin_y = 5

    elif event_name == 'laneChangeBlocked':
      CS = ui_state.sm['carState']
      if CS.leftBlinker:
        icon_side = IconSide.left
      elif CS.rightBlinker:
        icon_side = IconSide.right
      else:
        icon_side = self._last_icon_side
      txt_icon = self._txt_blind_spot_left if icon_side == 'left' else self._txt_blind_spot_right
      icon_margin_x = 8
      icon_margin_y = 0

    else:
      self._turn_signal_timer = 0.0

    self._last_icon_side = icon_side

    # create text rect based on icon presence
    text_x = self._rect.x + ALERT_MARGIN
    text_width = self._rect.width - 2 * ALERT_MARGIN
    if icon_side == 'left':
      text_x = self._rect.x + txt_icon.width + icon_margin_x + ALERT_MARGIN
      text_width = self._rect.width - txt_icon.width - icon_margin_x - 2 * ALERT_MARGIN
    elif icon_side == 'right':
      text_x = self._rect.x + ALERT_MARGIN
      text_width = self._rect.width - txt_icon.width - icon_margin_x - 2 * ALERT_MARGIN

    text_rect = rl.Rectangle(
      text_x,
      self._alert_y_filter.x,
      text_width,
      self._rect.height,
    )
    icon_layout = IconLayout(txt_icon, icon_side, icon_margin_x, icon_margin_y) if txt_icon is not None and icon_side is not None else None
    return AlertLayout(text_rect, icon_layout)

  def _render(self, rect: rl.Rectangle) -> bool:
    if not ui_state.started:
      self._reset_timers()
      return False
    alert, timer = self._pending_presentation if self._pending_presentation is not None else self._presentation()
    self._pending_presentation = None
    has_indicator = alert is not None or timer is not None

    # Animate fade and slide in/out
    self._alert_y_filter.update(self._rect.y if has_indicator else self._rect.y - 50)
    self._alpha_filter.update(1 if has_indicator else 0)

    if timer is not None:
      self._prev_alert = None
      self._status_timer_renderer.draw(rect, timer, self._alpha_filter.x, self._alert_y_filter.x - rect.y)
      return True

    if alert is None:
      # If still animating out, keep the previous alert
      if self._alpha_filter.x > 0.01 and self._prev_alert is not None:
        alert = self._prev_alert
      else:
        self._prev_alert = None
        return False

    self._draw_background(alert)

    alert_layout = self._icon_helper(alert)
    self._draw_text(alert, alert_layout)
    self._draw_icons(alert_layout)

    return True

  def _draw_icons(self, alert_layout: AlertLayout) -> None:
    if alert_layout.icon is None:
      return

    if time.monotonic() - self._turn_signal_timer > TURN_SIGNAL_BLINK_PERIOD:
      self._turn_signal_timer = time.monotonic()
      self._turn_signal_alpha_filter.x = 255 * 2
    else:
      self._turn_signal_alpha_filter.update(255 * 0.2)

    if alert_layout.icon.side == 'left':
      pos_x = int(self._rect.x + alert_layout.icon.margin_x)
    else:
      pos_x = int(self._rect.x + self._rect.width - alert_layout.icon.margin_x - alert_layout.icon.texture.width)

    if alert_layout.icon.texture not in (self._txt_turn_signal_left, self._txt_turn_signal_right):
      icon_alpha = 255
    else:
      icon_alpha = int(min(self._turn_signal_alpha_filter.x, 255))

    rl.draw_texture(alert_layout.icon.texture, pos_x, int(self._rect.y + alert_layout.icon.margin_y),
                    rl.Color(255, 255, 255, int(icon_alpha * self._alpha_filter.x)))

  def _draw_background(self, alert: Alert) -> None:
    # draw top gradient for alert text at top
    color = ALERT_COLORS.get(alert.status, ALERT_COLORS[AlertStatus.normal])
    color = rl.Color(color.r, color.g, color.b, int(255 * 0.90 * self._alpha_filter.x))
    translucent_color = rl.Color(color.r, color.g, color.b, int(0 * self._alpha_filter.x))

    small_alert_height = round(self._rect.height * 0.583) # 140px at mici height
    medium_alert_height = round(self._rect.height * 0.833) # 200px at mici height

    # alert_type format is "EventName/eventType" (e.g., "preLaneChangeLeft/warning")
    event_name = alert.alert_type.split('/')[0] if alert.alert_type else ''

    if event_name == 'preLaneChangeLeft':
      bg_height = small_alert_height
    elif event_name == 'preLaneChangeRight':
      bg_height = small_alert_height
    elif event_name == 'laneChange':
      bg_height = small_alert_height
    elif event_name == 'laneChangeBlocked':
      bg_height = medium_alert_height
    else:
      bg_height = int(self._rect.height)

    solid_height = round(bg_height * 0.2)
    rl.draw_rectangle(int(self._rect.x), int(self._rect.y), int(self._rect.width), solid_height, color)
    rl.draw_rectangle_gradient_v(int(self._rect.x), int(self._rect.y + solid_height), int(self._rect.width),
                                 int(bg_height - solid_height),
                                 color, translucent_color)

  def _draw_text(self, alert: Alert, alert_layout: AlertLayout) -> None:
    icon_side = alert_layout.icon.side if alert_layout.icon is not None else None
    event_name, _, event_type = alert.alert_type.partition('/')
    alert_text2 = alert.text2

    # randomize chars and length for testing
    if DEBUG:
      if time.monotonic() - self._text_gen_time > 0.5:
        self._alert_text2_gen = ''.join(random.choices(string.ascii_lowercase + ' ', k=random.randint(0, 40)))
        self._text_gen_time = time.monotonic()
      alert_text2 = self._alert_text2_gen or alert_text2

    title_size, subtitle_size = font_sizes(event_name, event_type, alert.text1, alert_text2, icon_side is not None)
    width = max(1, int(alert_layout.text_rect.width))
    height = max(1, int(alert_layout.text_rect.height) - 16)
    title_height, subtitle_height = self._fit_text(alert.text1, alert_text2, title_size, subtitle_size, width, height)
    alignment = TextAlignment.LEFT if icon_side != 'left' else TextAlignment.RIGHT
    top = alert_layout.text_rect.y + 4
    for label, y, content_height, alpha in (
      (self._alert_text1_label, top, title_height, 0.9),
      (self._alert_text2_label, top + title_height + 4, subtitle_height, 0.65),
    ):
      if content_height:
        label.set_alignment(alignment)
        label.set_text_color(rl.Color(255, 255, 255, int(255 * alpha * self._alpha_filter.x)))
        label.render(rl.Rectangle(alert_layout.text_rect.x, y, width, content_height))

  def _fit_text(self, title: str, subtitle: str, title_size: int, subtitle_size: int,
                width: int, height: int) -> tuple[float, float]:
    key = (title, subtitle, title_size, subtitle_size, width, height)
    if key == self._text_layout_key:
      return self._text_layout_heights
    self._alert_text1_label.set_text(title)
    self._alert_text2_label.set_text(subtitle)

    def measure(percent):
      self._alert_text1_label.set_font_size(max(1, title_size * percent // 100))
      self._alert_text2_label.set_font_size(max(1, subtitle_size * percent // 100))
      return (self._alert_text1_label.get_content_height(width) if title else 0.0,
              self._alert_text2_label.get_content_height(width) if subtitle else 0.0)

    title_height, subtitle_height = measure(100)
    gap = 4 if subtitle else 0
    if title_height + subtitle_height + gap > height:
      # Bounded search avoids generating dozens of cached wraps for a changing
      # diagnostic, while retaining the largest fitting reference font sizes.
      low, high, best = 1, 99, 1
      while low <= high:
        percent = (low + high) // 2
        h1, h2 = measure(percent)
        if h1 + h2 + gap <= height:
          best, low = percent, percent + 1
        else:
          high = percent - 1
      title_height, subtitle_height = measure(best)
    self._text_layout_key = key
    self._text_layout_heights = (title_height, subtitle_height)
    return self._text_layout_heights
