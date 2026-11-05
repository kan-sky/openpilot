"""Wayon DM artwork for NewUi; freshness guards affect presentation only."""
import math
import time

from openpilot.selfdrive.ui.dm_preview import PREVIEW_MAX_AGE, sample_fresh
from openpilot.selfdrive.ui.mici.onroad.driver_state import AlertSize, DriverStateRenderer
from openpilot.selfdrive.ui.ui_state import ui_state


class WayonDriverPreview(DriverStateRenderer):
  def __init__(self):
    super().__init__()
    self.set_enabled(False)
    self._closed = False
    self._started_frame = ui_state.started_frame
    self._model_frame = None
    self._model_progress = 0.0
    self._last_draw = 0.0

  def _reset_pose(self):
    self._is_active = self._phone_detected = self._looking_center = False
    self._phone_icon_filter.x = self._looking_center_filter.x = 0.0
    for f in (self._pitch_filter, self._yaw_filter, self._rotation_filter):
      f.x, f.initialized = 0.0, False

  def reset(self):
    self._reset_pose()
    self._fade_filter.x = 0.0
    self._color_fade_filter.x = 1.0
    self._model_frame = None
    self._model_progress = self._last_draw = 0.0
    self.set_should_draw(False)

  def close(self):
    self._closed = True
    self.reset()

  def draw_onroad(self, parent, alert_visible: bool) -> bool:
    sm = ui_state.sm
    if self._started_frame != ui_state.started_frame:
      self.reset()
      self._started_frame = ui_state.started_frame
    visible = (not self._closed and ui_state.started and
               (sm['selfdriveState'].enabled or ui_state.always_on_dm) and not alert_visible and
               sm['selfdriveState'].alertSize == AlertSize.none and not sm['driverMonitoringState'].dm2Disabled)
    if not visible:
      self.reset()
      return False
    now = time.monotonic()
    if now - self._last_draw >= PREVIEW_MAX_AGE:
      self._reset_pose()
    self._last_draw = now
    self.set_should_draw(True)
    self.set_position(parent.x + 16, parent.y + 10)
    self.render()
    return True

  def _update_state(self):
    now, sm = time.monotonic(), ui_state.sm
    fresh = all(sm.valid[s] and sm.alive[s] and sm.recv_frame[s] > ui_state.started_frame and
                sample_fresh(now, sm.logMonoTime[s] / 1e9) for s in ('driverMonitoringState', 'driverStateV2'))
    model = sm['driverStateV2']
    if fresh and model.frameId != self._model_frame:
      self._model_frame, self._model_progress = model.frameId, now
    fresh = fresh and now - self._model_progress < PREVIEW_MAX_AGE
    dm = sm['driverMonitoringState']
    vision = dm.visionPolicyState
    if (not fresh or dm.cameraUnavailable or not vision.faceDetected or
        not all(math.isfinite(v) for v in (vision.pose.pitch, vision.pose.yaw))):
      # No stale green cone, phone or retained pose after data/face loss.
      self._reset_pose()
      self._fade_filter.update(0.35)
      return
    super()._update_state()
    self._awareness_unfull |= bool(vision.isDistracted or dm.lockout or dm.alwaysOnLockout)
