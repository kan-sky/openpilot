import time
import numpy as np
import raylib_nb as rl
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

OpState = log.SelfdriveState.OpenpilotState
CALIBRATED = log.ExtrinsicsCalibration.Status.calibrated
NARROW_ROAD_CAM = VisionStreamType.VISION_STREAM_NARROW_ROAD
WIDE_CAM = VisionStreamType.VISION_STREAM_WIDE_ROAD
DEFAULT_DEVICE_CAMERA = DEVICE_CAMERAS["tici", "ar0231"]

BORDER_COLORS = {
  UIStatus.DISENGAGED: rl.Color(0x12, 0x28, 0x39, 0xFF),  # Blue for disengaged state
  UIStatus.OVERRIDE: rl.Color(0x89, 0x92, 0x8D, 0xFF),  # Gray for override state
  UIStatus.ENGAGED: rl.Color(0x16, 0x7F, 0x40, 0xFF),  # Green for engaged state
}

WIDE_CAM_MAX_SPEED = 5.0  # m/s (11 mph)
ROAD_CAM_MIN_SPEED = 10.0  # m/s (22 mph)
INF_POINT = np.array([1000.0, 0.0, 0.0])
BORDER_FONT_SIZE = 38
TPMS_FONT_SIZE = 40
TPMS_NORMAL_COLOR = rl.Color(255, 255, 255, 220)
TPMS_LOW_COLOR = rl.Color(255, 90, 90, 220)


class AugmentedRoadView(CameraView):
  def __init__(self, stream_type: VisionStreamType = VisionStreamType.VISION_STREAM_NARROW_ROAD):
    super().__init__("camerad", stream_type)
    self._set_placeholder_color(BORDER_COLORS[UIStatus.DISENGAGED])

    self.device_camera: DeviceCameraConfig | None = None
    self.view_from_calib = view_frame_from_device_frame.copy()
    self.view_from_wide_calib = view_frame_from_device_frame.copy()

    self._matrix_cache_key = (0, 0.0, 0.0, stream_type)
    self._cached_matrix: np.ndarray | None = None
    self._content_rect = rl.Rectangle()

    self.model_renderer = ModelRenderer()
    self._hud_renderer = HudRenderer()
    self.alert_renderer = AlertRenderer()
    self.driver_state_renderer = DriverStateRenderer()

    # 테두리 정보 표시용
    self._border_font = gui_app.font(FontWeight.MEDIUM)
    self._tpms_font = gui_app.font(FontWeight.DISPLAY)
    self._params_memory = Params("/dev/shm/params")
    self._border_params_next_t = 0.0
    self._git_branch = ""
    self._network_address = ""
    self._lane_width_filter = FirstOrderFilter(3.2, 3.0, DT_MDL)

  def _render(self, rect):
    # Only render when system is started to avoid invalid data access
    if not ui_state.started:
      return

    self._switch_stream_if_needed(ui_state.sm)

    # Update calibration before rendering
    self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      rect.x + UI_BORDER_SIZE,
      rect.y + UI_BORDER_SIZE,
      rect.width - 2 * UI_BORDER_SIZE,
      rect.height - 2 * UI_BORDER_SIZE,
    )

    # Enable scissor mode to clip all rendering within content rectangle boundaries
    # This creates a rendering viewport that prevents graphics from drawing outside the border
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    # Render the base camera view
    super()._render(self._content_rect)

    # Draw all UI overlays
    self.model_renderer.render(self._content_rect)
    self._hud_renderer.render(self._content_rect)
    self.alert_renderer.render(self._content_rect)
    self.driver_state_renderer.render(self._content_rect)

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds
    self._draw_tpms(self._content_rect)

    # End clipping region
    rl.end_scissor_mode()

    # Draw colored border based on driving state
    self._draw_border(rect)
    self._draw_border_info(rect)

  def _handle_mouse_press(self, _):
    if self._click_callback is not None:
      self._click_callback()

  def _handle_mouse_release(self, _):
    # We only call click callback on press
    pass

  def _draw_border(self, rect: rl.Rectangle):
    rl.draw_rectangle_lines_ex(rect, UI_BORDER_SIZE, rl.BLACK)
    border_roundness = 0.12
    border_color = BORDER_COLORS.get(ui_state.status, BORDER_COLORS[UIStatus.DISENGAGED])
    border_rect = rl.Rectangle(rect.x + UI_BORDER_SIZE, rect.y + UI_BORDER_SIZE,
                               rect.width - 2 * UI_BORDER_SIZE, rect.height - 2 * UI_BORDER_SIZE)
    rl.draw_rectangle_rounded_lines_ex(border_rect, border_roundness, 10, UI_BORDER_SIZE, border_color)

  def _refresh_border_params(self):
    # 파라미터 파일 읽기는 2초에 한 번만
    now = time.monotonic()
    if now < self._border_params_next_t:
      return
    self._border_params_next_t = now + 2.0
    try:
      self._git_branch = ui_state.params.get("GitBranch") or ""
      self._network_address = self._params_memory.get("NetworkAddress") or ""
    except Exception:
      pass

  def _border_text(self, text: str, x: float, y: float, align: str, color: rl.Color = rl.WHITE) -> float:
    if not text:
      return 0.0
    font = self._border_font if text.isascii() else gui_app.fallback_font()
    size = measure_text_cached(font, text, BORDER_FONT_SIZE)
    if align == "center":
      x -= size.x / 2
    elif align == "right":
      x -= size.x
    rl.draw_text_ex(font, text, rl.Vector2(x + 2, y + 2), BORDER_FONT_SIZE, 0, rl.Color(0, 0, 0, 200))
    rl.draw_text_ex(font, text, rl.Vector2(x, y), BORDER_FONT_SIZE, 0, color)
    return size.x

  @staticmethod
  def _get_tpms_color(tpms: float) -> rl.Color:
    if tpms < 5 or tpms > 60:
      return TPMS_NORMAL_COLOR
    if tpms < 31:
      return TPMS_LOW_COLOR
    return TPMS_NORMAL_COLOR

  @staticmethod
  def _get_tpms_text(tpms: float) -> str:
    if tpms < 5 or tpms > 60:
      return '  -'
    return f'{round(tpms):.0f}'

  def _draw_tpms(self, rect: rl.Rectangle):
    # 카메라 화면 오른쪽 위에 네 바퀴 공기압(psi)을 차 모양 배치로 표시
    try:
      tpms = ui_state.sm['carState'].tpms
      fl = float(tpms.fl)
      fr = float(tpms.fr)
      rl_v = float(tpms.rl)
      rr = float(tpms.rr)
    except Exception:
      return

    bx = rect.x + rect.width - 125
    by = rect.y + 130
    dw = 80
    self._draw_tpms_value(fl, bx - dw, by - 55)
    self._draw_tpms_value(fr, bx + dw, by - 55)
    self._draw_tpms_value(rl_v, bx - dw, by + 70)
    self._draw_tpms_value(rr, bx + dw, by + 70)

  def _draw_tpms_value(self, value: float, cx: float, bottom_y: float):
    # 가운데 정렬, 글자 아래쪽을 bottom_y 에 맞추고 그림자를 깐다
    text = self._get_tpms_text(value)
    size = measure_text_cached(self._tpms_font, text, TPMS_FONT_SIZE)
    x = cx - size.x / 2
    y = bottom_y - size.y
    rl.draw_text_ex(self._tpms_font, text, rl.Vector2(x + 4, y + 4), TPMS_FONT_SIZE, 0, rl.Color(0, 0, 0, 160))
    rl.draw_text_ex(self._tpms_font, text, rl.Vector2(x, y), TPMS_FONT_SIZE, 0, self._get_tpms_color(value))

  def _draw_record_indicator(self, x: float, y: float):
    # 화면 녹화 중이면 깜빡이는 빨간 점과 경과 시간 표시
    if not gui_app.is_recording():
      return
    elapsed = int(gui_app.recording_elapsed())
    radius = BORDER_FONT_SIZE * 0.3
    cy = y + BORDER_FONT_SIZE / 2
    if elapsed % 2 == 0:
      rl.draw_circle(int(x + radius), int(cy), radius, rl.Color(255, 40, 40, 255))
    self._border_text(f"REC {elapsed // 60:02d}:{elapsed % 60:02d}", x + radius * 2 + 10, y, "left", rl.Color(255, 90, 90, 255))

  def _draw_border_info(self, rect: rl.Rectangle):
    # 테두리 위아래에 차량/보정/장치 상태를 표시
    sm = ui_state.sm
    self._refresh_border_params()

    top_left = top_center = top_right = bottom_center = device_text = ""
    try:
      cp = sm["carParams"]
      top_left = str(cp.carFingerprint) + (" - OP Long" if cp.openpilotLongitudinalControl else "")
      top_center = str(sm["carState"].logCarrot)

      parts = []
      if sm.valid["lateralDelay"] and sm.recv_frame["lateralDelay"] > 0:
        ld = sm["lateralDelay"]
        parts.append(f"LD[{ld.calPerc}%,{ld.lateralDelay:.2f}]")
      if sm.valid["lateralTorqueParameters"] and sm.recv_frame["lateralTorqueParameters"] > 0:
        lt = sm["lateralTorqueParameters"]
        parts.append(f"LT[{lt.calPerc}%,{'ON' if lt.valid else 'OFF'}]({lt.latAccelFactorFiltered:.2f}/{lt.frictionCoefficientFiltered:.2f})")
      if sm.recv_frame["vehicleParameters"] > 0:
        parts.append(f"SR({sm['vehicleParameters'].steerRatio:.1f})")
      top_right = ", ".join(parts)

      ds = sm["deviceState"]
      cpu_usage = list(ds.cpuUsagePercent)
      cpu_temp = list(ds.cpuTempC)
      cpu_text = f"{sum(cpu_usage) / len(cpu_usage):.0f}%" if cpu_usage else "--"
      if cpu_temp:
        cpu_text += f",{max(cpu_temp):.0f}°C"
      device_text = f"MEM:{ds.memoryUsagePercent}% DISK:{100 - ds.freeSpacePercent:.0f}% CPU:{cpu_text}"

      # 차선폭: 왼쪽 | 내 차선 | 오른쪽
      md = sm["modelV2"]
      if sm.updated["modelV2"] and len(md.laneLines) >= 4 and len(md.laneLineProbs) >= 4:
        if md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5:
          self._lane_width_filter.update(abs(md.laneLines[2].y[0] - md.laneLines[1].y[0]))
      bottom_center = f"{md.meta.laneWidthLeft:.1f}m | {self._lane_width_filter.x:.1f}m | {md.meta.laneWidthRight:.1f}m"
    except Exception:
      pass

    # 글자가 테두리보다 커서 위는 테두리 상단, 아래는 테두리 하단에 맞춘다
    margin = 40.0
    line_h = BORDER_FONT_SIZE + 4
    top_y = rect.y + 2
    bottom_y = rect.y + rect.height - BORDER_FONT_SIZE - 2
    self._border_text(top_left, rect.x + margin, top_y, "left")
    self._border_text(top_center, rect.x + rect.width / 2, top_y, "center")
    self._border_text(top_right, rect.x + rect.width - margin, top_y, "right")
    self._border_text(device_text, rect.x + rect.width - margin, top_y + line_h, "right")
    self._draw_record_indicator(rect.x + margin, top_y + line_h)
    self._border_text(self._git_branch, rect.x + margin, bottom_y, "left")
    self._border_text(bottom_center, rect.x + rect.width / 2, bottom_y, "center")
    self._border_text(self._network_address, rect.x + rect.width - margin, bottom_y, "right")
  def _switch_stream_if_needed(self, sm):
    if sm['carParams'].openpilotLongitudinalControl and WIDE_CAM in self.available_streams:
      v_ego = sm['carState'].vEgo
      if v_ego < WIDE_CAM_MAX_SPEED:
        target = WIDE_CAM
      elif v_ego > ROAD_CAM_MIN_SPEED:
        target = NARROW_ROAD_CAM
      else:
        # Hysteresis zone - keep current stream
        target = self.stream_type
    else:
      target = NARROW_ROAD_CAM

    if self.stream_type != target:
      self.switch_stream(target)

  def _update_calibration(self):
    # Update device camera if not already set
    sm = ui_state.sm
    if not self.device_camera and sm.seen['narrowRoadCameraState'] and sm.seen['deviceState']:
      self.device_camera = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]

    # Check if camera calibration data is available and valid
    if not (sm.updated["extrinsicsCalibration"] and sm.valid['extrinsicsCalibration']):
      return

    calib = sm['extrinsicsCalibration']
    if len(calib.rpyCalib) != 3 or calib.calStatus != CALIBRATED:
      return

    # Update view_from_calib matrix
    device_from_calib = rot_from_euler(calib.rpyCalib)
    self.view_from_calib = view_frame_from_device_frame @ device_from_calib

    # Update wide calibration if available
    if hasattr(calib, 'wideFromDeviceEuler') and len(calib.wideFromDeviceEuler) == 3:
      wide_from_device = rot_from_euler(calib.wideFromDeviceEuler)
      self.view_from_wide_calib = view_frame_from_device_frame @ wide_from_device @ device_from_calib

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    # Check if we can use cached matrix
    cache_key = (
      ui_state.sm.recv_frame['extrinsicsCalibration'],
      self._content_rect.width,
      self._content_rect.height,
      self.stream_type
    )
    if cache_key == self._matrix_cache_key and self._cached_matrix is not None:
      return self._cached_matrix

    # Get camera configuration
    device_camera = self.device_camera or DEFAULT_DEVICE_CAMERA
    is_wide_camera = self.stream_type == WIDE_CAM
    intrinsic = device_camera.wide_road.intrinsics if is_wide_camera else device_camera.narrow_road.intrinsics
    calibration = self.view_from_wide_calib if is_wide_camera else self.view_from_calib
    zoom = 2.0 if is_wide_camera else 1.1

    # Calculate transforms for vanishing point
    calib_transform = intrinsic @ calibration
    kep = calib_transform @ INF_POINT

    # Calculate center points and dimensions
    x, y = self._content_rect.x, self._content_rect.y
    w, h = self._content_rect.width, self._content_rect.height
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]

    # Ensure zoom views the whole area
    zoom = max(zoom, w / (2 * cx), h / (2 * cy))

    # Calculate max allowed offsets with margins
    margin = 5
    max_x_offset = max(0.0, cx * zoom - w / 2 - margin)
    max_y_offset = max(0.0, cy * zoom - h / 2 - margin)

    # Calculate and clamp offsets to prevent out-of-bounds issues
    try:
      if abs(kep[2]) > 1e-6:
        x_offset = np.clip((kep[0] / kep[2] - cx) * zoom, -max_x_offset, max_x_offset)
        y_offset = np.clip((kep[1] / kep[2] - cy) * zoom, -max_y_offset, max_y_offset)
      else:
        x_offset, y_offset = 0, 0
    except (ZeroDivisionError, OverflowError):
      x_offset, y_offset = 0, 0

    # Cache the computed transformation matrix to avoid recalculations
    self._matrix_cache_key = cache_key
    self._cached_matrix = np.array([
      [zoom * 2 * cx / w, 0, -x_offset / w * 2],
      [0, zoom * 2 * cy / h, -y_offset / h * 2],
      [0, 0, 1.0]
    ])

    video_transform = np.array([
      [zoom, 0.0, (w / 2 + x - x_offset) - (cx * zoom)],
      [0.0, zoom, (h / 2 + y - y_offset) - (cy * zoom)],
      [0.0, 0.0, 1.0]
    ])
    self.model_renderer.set_transform(video_transform @ calib_transform)

    return self._cached_matrix


if __name__ == "__main__":
  gui_app.init_window("OnRoad Camera View")
  road_camera_view = AugmentedRoadView(NARROW_ROAD_CAM)
  gui_app.push_widget(road_camera_view)
  print("***press space to switch camera view***")
  try:
    for _ in gui_app.render():
      ui_state.update()
      if rl.is_key_released(rl.KeyboardKey.KEY_SPACE):
        if WIDE_CAM in road_camera_view.available_streams:
          stream = NARROW_ROAD_CAM if road_camera_view.stream_type == WIDE_CAM else WIDE_CAM
          road_camera_view.switch_stream(stream)
  finally:
    road_camera_view.close()
