import pyray as rl
from enum import IntEnum
import time
import cereal.messaging as messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.layouts.sidebar import Sidebar, SIDEBAR_WIDTH
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout, PanelType
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.selfdrive.ui.layouts.onboarding import OnboardingWindow
from openpilot.selfdrive.ui.body.layouts.onroad import BodyLayout

# Carrot
from openpilot.selfdrive.ui.onroad.debug_plot import DebugPlot
ONROAD_DELAY = 2.5

class MainState(IntEnum):
  HOME = 0
  SETTINGS = 1
  ONROAD = 2


class MainLayout(Widget):
  def __init__(self):
    super().__init__()

    self._pm = messaging.PubMaster(['bookmarkButton'])

    self._sidebar = Sidebar()
    self._current_mode = MainState.HOME

    # Initialize layouts
    self._home_layout = HomeLayout()
    self._home_body_layout = BodyLayout()

    # Custom onroad widgets
    self._onroad_layout = AugmentedRoadView()
    self._debug_plot = DebugPlot()

    self._layouts = {
      MainState.HOME: HomeLayout(),
      MainState.SETTINGS: SettingsLayout(),
      MainState.ONROAD: self._onroad_layout,
    }

    self._prev_onroad = False
    self._prev_standstill = False
    self._onroad_time_delay: float | None = None

    self._sidebar_rect = rl.Rectangle(0, 0, 0, 0)
    self._content_rect = rl.Rectangle(0, 0, 0, 0)

    self._onboarding_window = OnboardingWindow()
    self._setup_callbacks()
    gui_app.push_widget(self)

    # Start onboarding if terms or training not completed, make sure to push after self
    if not self._onboarding_window.completed:
      gui_app.push_widget(self._onboarding_window)
    
  def _render(self, _):
    self._handle_transitions()
    self._update_layout_rects()
    self._render_main_content()


  def _setup_callbacks(self):
    self._sidebar.set_callbacks(on_settings=self._on_settings_clicked,
      on_flag=self._on_bookmark_clicked,
      open_settings=lambda: self.open_settings(PanelType.TOGGLES))
    self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self.open_settings(PanelType.FIREHOSE))
    self._layouts[MainState.HOME].set_settings_callback(lambda: self.open_settings(PanelType.TOGGLES))

    # Kans: self._layouts[MainState.SETTINGS].set_callbacks(on_close=self._set_mode_for_state)
    self._layouts[MainState.SETTINGS].set_callbacks(on_close=self._on_settings_closed)

    # Kans: onroad tap -> toggle sidebar
    for layout in (self._onroad_layout, self._home_body_layout):
      layout.set_click_callback(self._on_onroad_clicked)
    device.add_interactive_timeout_callback(self._on_interactive_timeout)
    ui_state.add_on_body_changed_callbacks(self._on_body_changed)

  def _update_layout_rects(self):
    self._sidebar_rect = rl.Rectangle(self._rect.x, self._rect.y, SIDEBAR_WIDTH, self._rect.height)
    x_offset = SIDEBAR_WIDTH if self._sidebar.is_visible else 0
    self._content_rect = rl.Rectangle(self._rect.x + x_offset, self._rect.y, self._rect.width - x_offset, self._rect.height)

  # Kans:
  def _handle_transitions(self):
    # Don't transition while onboarding is on top
    if gui_app.get_active_widget() == self._onboarding_window:
      return

    # Detect onroad edge
    if ui_state.started != self._prev_onroad:
      self._prev_onroad = ui_state.started

      # Kans: self._set_mode_for_state()
      if ui_state.started:
        self._onroad_time_delay = rl.get_time()
      else:
        # Offroad: go HOME and show sidebar
        self._onroad_time_delay = None
        self._sidebar.set_visible(True)
        self._set_current_layout(MainState.HOME)
        self._prev_standstill = False
        return

    # After delay, force ONROAD unless user is in settings
    if self._onroad_time_delay is not None and (rl.get_time() - self._onroad_time_delay) >= ONROAD_DELAY:
      if self._current_mode != MainState.SETTINGS:
        self._set_current_layout(MainState.ONROAD)
        self._sidebar.set_visible(False)
      self._onroad_time_delay = None

    # Handle standstill -> moving transition
    try:
      standstill = bool(ui_state.sm["carState"].standstill)
    except Exception:
      standstill = False

    if ui_state.started:
      if (not standstill) and self._prev_standstill:
        if self._current_mode != MainState.SETTINGS:
          self._sidebar.set_visible(False)
          self._set_current_layout(MainState.ONROAD)
      self._prev_standstill = standstill
    else:
      self._prev_standstill = False

  def _set_current_layout(self, layout: MainState):
    if layout != self._current_mode:
      self._layouts[self._current_mode].hide_event()
      self._current_mode = layout
      self._layouts[self._current_mode].show_event()

  def open_settings(self, panel_type: PanelType):
    self._layouts[MainState.SETTINGS].set_current_panel(panel_type)
    self._set_current_layout(MainState.SETTINGS)
    self._sidebar.set_visible(False)

  def _on_settings_clicked(self):
    self.open_settings(PanelType.DEVICE)

  # Kans:
  def _on_settings_closed(self, *_args, **_kwargs):
    # When settings closes, return to ONROAD if started else HOME
    if ui_state.started:
      self._sidebar.set_visible(False)
      self._set_current_layout(MainState.ONROAD)
    else:
      self._sidebar.set_visible(True)
      self._set_current_layout(MainState.HOME)

  def _on_bookmark_clicked(self):
    user_bookmark = messaging.new_message('bookmarkButton')
    user_bookmark.valid = True
    self._pm.send('bookmarkButton', user_bookmark)

  def _on_onroad_clicked(self):
    self._sidebar.set_visible(not self._sidebar.is_visible)

  def _on_body_changed(self):
    self._layouts[MainState.HOME] = self._home_body_layout if ui_state.is_body else self._home_layout
    # maintain Home in body Mode
    if ui_state.is_body:
      self._onroad_time_delay = None
      self._set_current_layout(MainState.HOME)
      self._sidebar.set_visible(not ui_state.ignition)
      return

    # C3X/Kans: maintain existing started/onroad delay 
    if not ui_state.started:
      self._onroad_time_delay = None
      self._sidebar.set_visible(True)
      self._set_current_layout(MainState.HOME)

  # Kans
  def _on_interactive_timeout(self):
    if gui_app.get_active_widget() == self._onboarding_window:
      return

    if ui_state.started:
      # Don't pop to onroad if at standstill
      try:
        if not ui_state.sm["carState"].standstill:
          self._sidebar.set_visible(False)
          self._set_current_layout(MainState.ONROAD)
      except Exception:
        self._sidebar.set_visible(False)
        self._set_current_layout(MainState.ONROAD)
    else:
      self._sidebar.set_visible(True)
      self._set_current_layout(MainState.HOME)

  def _render_main_content(self):
    # Render sidebar first
    if self._sidebar.is_visible:
      self._sidebar.render(self._sidebar_rect)

    content_rect = self._content_rect if self._sidebar.is_visible else self._rect

    # ONROAD is special: render augmented road view + optional debug overlay
    if self._current_mode == MainState.ONROAD:
      self._onroad_layout.render(content_rect)

      show_plot_mode = int(ui_state.params.get_int("ShowPlotMode"))
      if show_plot_mode > 0:
        self._debug_plot.render(content_rect)
    else:
      self._layouts[self._current_mode].render(content_rect)
