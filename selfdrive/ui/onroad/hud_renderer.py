import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
# Carrot
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log
from openpilot.common.params import Params
from datetime import datetime
EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'
# Kans
SET_SPEED_PERSISTENCE = 2.5  # seconds

# Carrot
@dataclass(frozen=True)
class SetSpeedOverrideState:
  active: bool
  speed_kph: float
  label: str
  speed_color_mode: int # 0: white, 1: green, 2: orange
  force_persist: bool

# Carrot
class SetSpeedOverride:
  def compute(self, sm, set_speed_kph: float) -> SetSpeedOverrideState:
    # eco (highest)
    cruise_target = None
    try:
      cruise_target = float(sm['longitudinalPlan'].cruiseTarget)
    except Exception:
      cruise_target = None

    if cruise_target is not None and cruise_target > (set_speed_kph + 0.5):
      return SetSpeedOverrideState(
        active=True,
        speed_kph=cruise_target,
        label="eco",
        speed_color_mode=1,
        force_persist=True,   # eco 조건 유지되는 동안 계속 표시
      )

    # apply_speed (desiredSpeed/source)
    desired_speed = None
    desired_source = ""
    try:
      desired_speed = float(sm['carrotMan'].desiredSpeed)
      desired_source = str(sm['carrotMan'].desiredSource or "")
    except Exception:
      desired_speed = None
      desired_source = ""

    if desired_speed is not None and 0 < desired_speed < 200 and desired_speed < set_speed_kph:
      label = desired_source.strip() or "apply"
      label = label[:8]  # 너무 길면 UI 깨짐 방지 (원하면 길이 조절)
      return SetSpeedOverrideState(
        active=True,
        speed_kph=desired_speed,
        label=label,
        speed_color_mode=2,
        force_persist=True,   # 조건 유지되는 동안 계속 표시
      )

    # default
    return SetSpeedOverrideState(
      active=False,
      speed_kph=set_speed_kph,
      label=tr("MAX"),
      speed_color_mode=0,
      force_persist=False,
    )
@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  # Kans
  GREEN = rl.Color(0, 200, 0, 100)
  BLUE = rl.Color(0, 140, 255, 120)
  OCHRE = rl.Color(204, 153, 0, 128)
  ORANGE = rl.Color(255, 187, 0, 128)


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()

# Carrot
class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, 0.0, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])

    if left or right:
      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out only, keep last direction
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    # carrot
    self._debug_speed_panel = False
    self._set_speed_changed_time: float = 0
    self._engaged: bool = False
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False

    self._can_draw_top_icons = True
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    # Carrot
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    self._debug_traffic_light = False
    self._set_speed_override = SetSpeedOverride()
    self._txt_wheel: rl.Texture = gui_app.texture('icons/wheel.png', 100, 100) # 이미지 사이즈용으로 사용
    self._txt_speed_bg: rl.Texture = gui_app.texture('images/speed_bg.png', 307, 115)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def _draw_text_with_outline(self, text, pos, font_size, text_color, outline_color=rl.BLACK, thickness=1):
    x, y = pos.x, pos.y
    for dx in range(-thickness, thickness + 1):
      for dy in range(-thickness, thickness + 1):
        if dx == 0 and dy == 0:
          continue
        rl.draw_text_ex(self._font_display, text, rl.Vector2(x + dx, y + dy), font_size, 0, outline_color)

    # main text
    rl.draw_text_ex(self._font_display, text, rl.Vector2(x, y), font_size, 0, text_color)

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    # carrot
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    self._update_state()

    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)
    #self._draw_current_speed(rect)
    self._draw_speed_limit_sign(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))
    self._draw_steering_wheel(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  # Carrot
  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel

    # Always visible (no hide). We keep filters but drive them to stable values.
    self._wheel_alpha_filter.update(255 * 0.95)
    self._wheel_y_filter.update(0)

    # 기본 좌표(Top_left)
    margin_x = 20
    margin_y = 20
    pos_x = int(rect.x + margin_x + wheel_txt.width / 2)
    pos_y = int(rect.y + margin_y + wheel_txt.height / 2 + self._wheel_y_filter.x)

    self._draw_steering_wheel_icon(wheel_txt, pos_x, pos_y)
    self._draw_wheel_side_info(wheel_txt, pos_x, pos_y)


  def _draw_steering_wheel_icon(self, wheel_txt, pos_x: int, pos_y: int) -> None:
    rotation = 0.0

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    if ui_state.lat_active:
      wheel_color = rl.Color(0, 255, 0, int(self._wheel_alpha_filter.x))
    else:
      wheel_color = rl.Color(160, 160, 160, int(self._wheel_alpha_filter.x))

    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, wheel_color)

  def _get_cpu_temp_text(self) -> str:
    try:
      ds = ui_state.sm['deviceState']
      cpu_temps = getattr(ds, 'cpuTempC', None)

      if cpu_temps is not None and len(cpu_temps) > 0:
        valid_temps = [float(t) for t in cpu_temps]
        if len(valid_temps) > 0:
          cpu_temp = sum(valid_temps) / float(len(valid_temps))
          return f"CPU: {cpu_temp:.0f}"
    except Exception:
      pass

    return "CPU: --"


  def _draw_wheel_side_info(self, wheel_txt, pos_x: int, pos_y: int) -> None:
    now = datetime.now()

    try:
      show_date_time = int(ui_state.show_date_time)
    except Exception:
      show_date_time = 1

    try:
      show_debug_ui = int(ui_state.show_debug_ui)
    except Exception:
      show_debug_ui = 0

    time_font = int(wheel_txt.height * 0.8)
    small_dt_font = max(25, int(time_font * 0.62))   # date+time 2줄용
    side_font = max(25, int(time_font * 0.33))

    time_x = pos_x + wheel_txt.width / 2 + 15

    # Date / Time
    # show_date_time: 0=hide, 1=date+time, 2=time only, 3=date only
    time_block_right = time_x

    if show_date_time != 0:
      time_text = now.strftime("%H:%M:%S")
      date_text = now.strftime("%y-%m-%d")

      if show_date_time == 1:
        # two lines: both use smaller font
        dt_font = small_dt_font

        date_size = measure_text_cached(self._font_medium, date_text, dt_font)
        time_size = measure_text_cached(self._font_semi_bold, time_text, dt_font)

        line_gap = max(2, int(dt_font * 0.10))
        total_h = date_size.y + line_gap + time_size.y
        base_y = pos_y - total_h / 2

        date_y = base_y
        time_y = date_y + date_size.y + line_gap

        block_w = max(date_size.x, time_size.x)
        date_x = time_x + (block_w - date_size.x) / 2
        draw_time_x = time_x + (block_w - time_size.x) / 2

        self._draw_text_with_outline(
          date_text,
          rl.Vector2(date_x, date_y),
          dt_font,
          rl.Color(255, 255, 255, 220),
          rl.BLACK,
          thickness=1
        )

        self._draw_text_with_outline(
          time_text,
          rl.Vector2(draw_time_x, time_y),
          dt_font,
          rl.Color(255, 255, 255, 230),
          rl.BLACK,
          thickness=1
        )

        time_block_right = time_x + block_w

      elif show_date_time == 2:
        # time only: large font
        text_font = time_font
        time_size = measure_text_cached(self._font_semi_bold, time_text, text_font)
        time_y = pos_y - time_size.y / 2

        self._draw_text_with_outline(
          time_text,
          rl.Vector2(time_x, time_y),
          text_font,
          rl.Color(255, 255, 255, 230),
          rl.BLACK,
          thickness=1
        )

        time_block_right = time_x + time_size.x

      elif show_date_time == 3:
        # date only: also large font
        text_font = time_font
        date_size = measure_text_cached(self._font_medium, date_text, text_font)
        date_y = pos_y - date_size.y / 2

        self._draw_text_with_outline(
          date_text,
          rl.Vector2(time_x, date_y),
          text_font,
          rl.Color(255, 255, 255, 220),
          rl.BLACK,
          thickness=1
        )

        time_block_right = time_x + date_size.x

    # Traffic Light (always higher priority than debug UI)
    traffic_x = int(time_block_right + 12)
    traffic_y = int(pos_y)

    if self._draw_traffic_light_info(traffic_x, traffic_y):
      return

    # Debug UI
    if show_debug_ui == 0:
      return

    info_x = time_block_right + 25

    cpu_text = self._get_cpu_temp_text()

    try:
      steer_ratio = float(ui_state.sm['liveParameters'].steerRatio)
      sr_text = f"SR: {steer_ratio:.1f}"
    except Exception:
      sr_text = "SR: --.-"

    try:
      road_name = ui_state.sm['carrotMan'].szPosRoadName
      if not road_name:
        road_name = ""
    except Exception:
      road_name = ""

    cpu_size = measure_text_cached(self._font_medium, cpu_text, side_font)
    sr_size = measure_text_cached(self._font_medium, sr_text, side_font)
    road_size = measure_text_cached(self._font_medium, road_name, side_font) if road_name else rl.Vector2(0, 0)

    line_gap = max(4, int(side_font * 0.15))

    total_h = cpu_size.y + line_gap + sr_size.y
    if road_name:
      total_h += line_gap + road_size.y

    base_y = pos_y - total_h / 2

    cpu_y = base_y
    sr_y = cpu_y + cpu_size.y + line_gap
    road_y = sr_y + sr_size.y + line_gap

    self._draw_text_with_outline(
      cpu_text,
      rl.Vector2(info_x, cpu_y),
      side_font,
      rl.Color(255, 255, 255, 210),
      rl.BLACK,
      thickness=1
    )

    self._draw_text_with_outline(
      sr_text,
      rl.Vector2(info_x, sr_y),
      side_font,
      rl.Color(255, 255, 255, 210),
      rl.BLACK,
      thickness=1
    )

    if road_name:
      self._draw_text_with_outline(
        road_name,
        rl.Vector2(info_x, road_y),
        side_font,
        rl.Color(255, 255, 255, 210),
        rl.BLACK,
        thickness=1
      )


  def _get_gear_text(self) -> str:
    sm = ui_state.sm

    try:
      car_state = sm["carState"]
      gear = car_state.gearShifter
    except Exception:
      return "R"

    # cereal enum → 문자열 변환
    try:
      gear_name = str(gear).split('.')[-1]
    except Exception:
      gear_name = str(gear)

    # DRIVE 처리
    if "DRIVE" in gear_name.upper():
      try:
        step = int(car_state.gearStep)
        if step > 0:
          return str(step)
        else:
          return "D"
      except Exception:
        return "D"

    if "PARK" in gear_name.upper():
      return "P"

    if "REVERSE" in gear_name.upper():
      return "R"

    if "NEUTRAL" in gear_name.upper():
      return "N"

    if "SPORT" in gear_name.upper():
      return "S"

    if "LOW" in gear_name.upper():
      return "L"

    if "BRAKE" in gear_name.upper():
      return "B"

    if "ECO" in gear_name.upper():
      return "E"

    if "UNKNOWN" in gear_name.upper():
      return "U"

    return "M"

  def _get_cruise_gap(self) -> int:
    try:
      personality = Params().get_int("LongitudinalPersonality")
      gap = int(personality) + 1
    except Exception:
      gap = 8

    return gap

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    ov = self._set_speed_override.compute(ui_state.sm, float(self.set_speed))

    # ----- panel placement (bottom-left) -----
    bg = self._txt_speed_bg
    panel_w = bg.width
    panel_h = bg.height

    margin_x = 10
    margin_y = 10
    panel_x = int(rect.x + margin_x)
    panel_y = int(rect.y + rect.height - panel_h - margin_y)

    # draw background
    rl.draw_texture(bg, panel_x, panel_y, rl.WHITE)

    # current speed(big, left)
    if self._debug_speed_panel:
      cur_speed_int = 123
    else:
      cur_speed_int = int(round(self.speed))

    cur_text = str(cur_speed_int)

    cur_font = 80
    cur_size = measure_text_cached(self._font_display, cur_text, cur_font)
    cur_x = panel_x + 18

    cur_y = int(panel_y + panel_h * 0.48 - cur_size.y * 0.5) - 2

    self._draw_text_with_outline(cur_text, rl.Vector2(cur_x, cur_y), cur_font, rl.WHITE, rl.BLACK, thickness=2)
    
    mode_text, mode_color = self._get_driving_mode_text_and_color()
    if self._debug_speed_panel:
      mode_text = "safe"
      mode_color = rl.Color(0, 255, 0, 230)

    if mode_text:
      mode_font = 30
      mode_size = measure_text_cached(self._font_semi_bold, mode_text, mode_font)

      mode_x = panel_x + 10
      mode_y = int(panel_y + panel_h * 0.05 - mode_size.y * 0.5 - 15)

      self._draw_text_with_outline(mode_text, rl.Vector2(mode_x, mode_y), mode_font, mode_color, rl.BLACK, thickness=1)
  
    # set speed (center, smaller)
    show_set = self._engaged and self.is_cruise_set
    if True:
      if show_set:
        set_speed = self.set_speed
        if not ui_state.is_metric:
          set_speed *= KM_TO_MILE
        set_text = str(int(round(set_speed)))
      else:
        set_text = "--"

      set_color = rl.Color(0, 255, 0, 230)

      if self._debug_speed_panel:
        set_text = str(123)

      set_font = 40
      set_size = measure_text_cached(self._font_display, set_text, set_font)
      set_x = int(panel_x + panel_w * 0.76 - set_size.x * 0.5)
      set_y = int(panel_y + panel_h * 0.33 - set_size.y * 0.5)
      self._draw_text_with_outline(set_text, rl.Vector2(set_x, set_y), set_font, set_color, rl.WHITE, thickness=1)
      if ov.active:
        set_speed = ov.speed_kph
        if not ui_state.is_metric:
          set_speed *= KM_TO_MILE
        set_text = str(int(round(set_speed)))
        set_label_text = ov.label

        if ov.speed_color_mode == 1:      # eco
          set_color = rl.Color(0, 255, 0, 230)
        elif ov.speed_color_mode == 2:    # apply
          set_color = rl.Color(255, 165, 0, 230)
        else:
          set_color = rl.Color(0, 255, 0, 230)   # your sample is green

        if self._debug_speed_panel:
          set_text = str(111)
          set_color = rl.Color(255, 165, 0, 230)
          set_label_text = "vturn"

        set_font = 40
        set_size = measure_text_cached(self._font_display, set_text, set_font)
        set_x = int(panel_x + panel_w * 0.90 - set_size.x * 0.5 + 50)
        set_y = int(panel_y + panel_h * 0.25 - set_size.y * 0.5)
        self._draw_text_with_outline(set_text, rl.Vector2(set_x, set_y), set_font, set_color, rl.WHITE, thickness=1)
        set_font = 30
        set_size = measure_text_cached(self._font_display, set_label_text, set_font)
        set_x = int(panel_x + panel_w * 0.90 - set_size.x * 0.5 + 50)
        set_y = int(panel_y + panel_h * 0.10 - set_size.y * 0.5 - 20)
        self._draw_text_with_outline(set_label_text, rl.Vector2(set_x, set_y), set_font, set_color, rl.BLACK, thickness=1)

    # cruise gap (small circle + number, bottom-mid-right)
    gap = self._get_cruise_gap()
    gap_center_x = int(panel_x + panel_w * 0.90)
    gap_center_y = int(panel_y + panel_h * 0.82)

    gap_text = str(gap)
    gap_font = 28
    gap_size = measure_text_cached(self._font_semi_bold, gap_text, gap_font)
    self._draw_text_with_outline(gap_text, rl.Vector2(gap_center_x - gap_size.x * 0.5, gap_center_y - gap_size.y * 0.5), gap_font, rl.WHITE, rl.BLACK, thickness=1)
    
    # Carrot: active carrot
    sm = ui_state.sm
    active_carrot = sm['carrotMan'].activeCarrot

    if active_carrot >= 2:
      x = int(panel_x + panel_w * 0.60)
      y = int(panel_y + panel_h * 0.82)
      self._draw_text_with_outline("NAV", rl.Vector2(x, y), 26, rl.GREEN, rl.BLACK, thickness=1)

    # gear (right side box with letter)
    gear = self._get_gear_text()
    box_w = 44
    box_h = 54
    box_x = int(panel_x + panel_w - box_w - 14 + 70)
    box_y = int(panel_y + panel_h * 0.50)

    # Fill (dark) + border (green)
    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_w, box_h), 0.2, 8, rl.Color(0, 0, 0, 120))
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(box_x, box_y, box_w, box_h), 0.2, 8, 3, rl.Color(0, 255, 0, 230))

    gear_font = 44
    gear_size = measure_text_cached(self._font_display, gear, gear_font)
    rl.draw_text_ex(
      self._font_display,
      gear,
      rl.Vector2(box_x + (box_w - gear_size.x) * 0.5, box_y + (box_h - gear_size.y) * 0.5),
      gear_font,
      0,
      rl.WHITE,
    )

    if self._debug_speed_panel:
      active_lane_line = True
    else:
      active_lane_line = bool(ui_state.sm['controlsState'].activeLaneLine)      

    line1 = "lane"
    line2 = "mode" if active_lane_line else "less"

    lane_font = 26  # 원하면 22~30 사이로 조절
    lane_color = rl.Color(255, 255, 255, 220)  # 흰색

    lane_x = box_x + box_w + 80
    lane_y1 = box_y + 2
    lane_y2 = box_y + 2 + lane_font + 2

    # 오른쪽 정렬(gear box 옆에 딱 붙게)
    s1 = measure_text_cached(self._font_semi_bold, line1, lane_font)
    s2 = measure_text_cached(self._font_semi_bold, line2, lane_font)

    self._draw_text_with_outline(line1, rl.Vector2(lane_x - s1.x, lane_y1), lane_font, lane_color, rl.BLACK, thickness=1)
    self._draw_text_with_outline(line2, rl.Vector2(lane_x - s2.x, lane_y2), lane_font, lane_color, rl.BLACK, thickness=1)

  def _get_driving_mode_text_and_color(self) -> tuple[str, rl.Color]:
    # Carrot
    carState = ui_state.sm["carState"]
    if carState.brakeHoldActive:
      return tr("brake hold"), rl.Color(255, 0, 0, 230)
    elif carState.softHoldActive:
      return tr("soft hold"), rl.Color(255, 165, 0, 230)
    elif carState.carrotCruise:
      return tr("carrot"), rl.Color(0, 255, 0, 230)
    
    try:
      mode_val = int(ui_state.sm["longitudinalPlan"].myDrivingMode)
    except Exception:
      return "", rl.Color(255, 255, 255, 200)

    if mode_val == 1:   # eco
      return tr("eco"), rl.Color(0, 255, 0, 200)
    if mode_val == 2:   # safe
      return tr("safe"), rl.Color(255, 165, 0, 200)
    if mode_val == 3:   # normal
      return tr("norm"), rl.Color(255, 255, 255, 200)
    if mode_val == 4:   # high
      return tr("high"), rl.Color(255, 0, 0, 200)

    return "", rl.Color(255, 255, 255, 200)


  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, rect.y + 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, rect.y + 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

  def _draw_speed_limit_sign(self, rect: rl.Rectangle) -> None:
    carrot_man = self.sm['carrotMan']
    active_carrot = carrot_man.activeCarrot
    limit_speed = carrot_man.xSpdLimit
    limit_dist = carrot_man.xSpdDist

    s_center_x = rect.x + UI_CONFIG.border_size + 110
    s_center_y = rect.y + 700
    d_center_y = s_center_y - 160

    diameters = (220, 180, 202)
    rects = {
      "inner": rl.Rectangle(s_center_x - diameters[1]//2, s_center_y - diameters[1]//2, diameters[1], diameters[1]),
      "main":  rl.Rectangle(s_center_x - diameters[0]//2, s_center_y - diameters[0]//2, diameters[0], diameters[0]),
      "outer": rl.Rectangle(s_center_x - diameters[2]//2, s_center_y - diameters[2]//2, diameters[2], diameters[2]),
      "limit_dist":  rl.Rectangle(s_center_x - 110, d_center_y - 35, 220, 70),
    }

    sl_opacity = 1
    limit_spd, limit_dist = ui_state.limitSpeed, ui_state.limitDist

    if limit_spd <= 21 and limit_dist == 0:
      return
    alpha = lambda v: int(255 / sl_opacity * v)
    visual_offset = 1.2

    if active_carrot >= 2:
      cx, cy = int(rects["inner"].x + rects["inner"].width / 2), int(rects["inner"].y + rects["inner"].height / 2)
      rl.draw_circle(cx, cy, int(diameters[0] / 2), rl.RED)
      rl.draw_circle(cx, cy, int(diameters[1] / 2), rl.WHITE)
      text = str(int(limit_spd))
      font_size = 110 if limit_spd < 100 else 90
      text_size = rl.measure_text_ex(self._font_bold, text, font_size, 0)
      text_x = rects["inner"].x + (rects["inner"].width - text_size.x*visual_offset) / 2
      text_y = rects["inner"].y + (rects["inner"].height - text_size.y*visual_offset) / 2
      rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), font_size, 0, rl.BLACK)

    if limit_dist == 0:
      return

    opacity = max(0, min(255, int(((600 - limit_dist) * 0.425) / sl_opacity))) if limit_dist <= 600 else 0
    rl.draw_rectangle_rounded(rects["limit_dist"], 0.35, 32, rl.Color(255, 0, 0, opacity))
    rl.draw_rectangle_rounded_lines_ex(rects["limit_dist"], 0.35, 32, 6, COLORS.WHITE_TRANSLUCENT)

    if ui_state.is_metric:
      dist_text = (
        f"{limit_dist:.0f}m" if limit_dist < 1000 else
        f"{limit_dist/1000:.2f}km" if limit_dist < 10000 else
        f"{limit_dist/1000:.1f}km"
      )
    else:
      dist_ft = limit_dist * 3.28084
      dist_text = f"{dist_ft:.0f}ft" if dist_ft < 1000 else f"{limit_dist * 0.000621:.2f}mi"

    font_size = 55
    text_size = rl.measure_text_ex(self._font_bold, dist_text, font_size, 0)
    text_x = rects["limit_dist"].x + (rects["limit_dist"].width - text_size.x*visual_offset) / 2
    text_y = rects["limit_dist"].y + (rects["limit_dist"].height - text_size.y*visual_offset) / 2
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x, text_y), font_size, 0, rl.WHITE)

  def _get_traffic_light_info(self):
    # debug demo
    if self._debug_traffic_light:
      demo_list = [
        {"lamp": "red", "remain": 13, "ts": time.monotonic()},
        {"lamp": "green", "remain": 8, "ts": time.monotonic()},
        {"lamp": "left", "remain": 7, "ts": time.monotonic()},
        {"lamp": "right", "remain": 5, "ts": time.monotonic()},
        {"lamp": "uturn", "remain": 4, "ts": time.monotonic()},
      ]
      idx = int(time.monotonic() // 2) % len(demo_list)
      return demo_list[idx]

    try:
      raw = ui_state.params_memory.get("TrafficLight", encoding="utf-8")
      if not raw:
        return None

      d = json.loads(raw)
      lamp = str(d.get("lamp", "")).strip()
      remain = int(d.get("remain", 0))
      ts = float(d.get("ts", 0.0))

      if lamp not in ("red", "green", "left", "right", "uturn"):
        return None

      if remain <= 0:
        return None

      # 1초마다 들어온다고 했으니, 2.5초 정도 지나면 stale로 보고 숨김
      if ts > 0.0 and (time.monotonic() - ts) > 2.5:
        return None

      return {
        "lamp": lamp,
        "remain": remain,
      }
    except Exception:
      return None

  def _draw_traffic_light_lamp(self, lamp: str, cx: int, cy: int, size: int) -> None:
    if lamp == "red":
      rl.draw_circle(cx, cy, size, rl.Color(255, 70, 70, 245))
      rl.draw_circle_lines(cx, cy, size, rl.Color(255, 255, 255, 220))
      return

    if lamp == "green":
      rl.draw_circle(cx, cy, size, rl.Color(0, 220, 80, 245))
      rl.draw_circle_lines(cx, cy, size, rl.Color(255, 255, 255, 220))
      return

    if lamp == "left":
      txt = "<-"
      color = rl.Color(0, 255, 100, 240)
    elif lamp == "right":
      txt = "->"
      color = rl.Color(0, 255, 100, 240)
    elif lamp == "uturn":
      txt = "U"
      color = rl.Color(255, 220, 80, 240)
    else:
      return

    font_size = int(size * 2.0)
    text_size = measure_text_cached(self._font_display, txt, font_size)
    self._draw_text_with_outline(
      txt,
      rl.Vector2(cx - text_size.x * 0.5, cy - text_size.y * 0.5),
      font_size,
      color,
      rl.BLACK,
      thickness=1
    )
  
  def _draw_traffic_light_info(self, pos_x: int, pos_y: int) -> bool:
    info = self._get_traffic_light_info()
    if not info:
      return False

    lamp = info["lamp"]
    remain = str(info["remain"])

    lamp_size = 24
    remain_font = 28
    gap = 5

    remain_size = measure_text_cached(self._font_semi_bold, remain, remain_font)

    lamp_cx = pos_x + lamp_size
    lamp_cy = int(pos_y)

    self._draw_traffic_light_lamp(lamp, lamp_cx, lamp_cy, lamp_size)

    text_x = lamp_cx + lamp_size + gap
    text_y = int(pos_y - remain_size.y / 2)

    self._draw_text_with_outline(
      remain,
      rl.Vector2(text_x, text_y),
      remain_font,
      rl.Color(255, 255, 255, 235),
      rl.BLACK,
      thickness=1
    )

    return True

