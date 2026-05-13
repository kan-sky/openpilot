import os
import math
import importlib # Kans

from cereal import messaging, log
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.onroad.driver_camera_dialog import DriverCameraDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.onboarding import TrainingGuide
from openpilot.selfdrive.ui.widgets.pairing_dialog import PairingDialog
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import multilang, tr, tr_noop
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.html_render import HtmlModal
from openpilot.system.ui.widgets.list_view import text_item, button_item, dual_button_item, single_button_item, triple_button_item # Kans
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller

# Description constants
DESCRIPTIONS = {
  'pair_device': tr_noop("Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."),
  'driver_camera': tr_noop("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"),
  'reset_calibration': tr_noop("openpilot requires the device to be mounted within 4° left or right and within 5° up or 9° down."),
  'review_guide': tr_noop("Review the rules, features, and limitations of openpilot"),
}

# Kans: CarSelector - Car List Widget (NO MultiOptionDialog)
class CarListDialog(Widget):
  def __init__(self, params: Params, title: str, car_names: list[str], cur: str | None):
    super().__init__()
    self._params = params
    self._title = title
    self._car_names = car_names
    self._cur = cur

    # 상단 안내(선택된 차량 표시)
    header = text_item(lambda: tr("Select Your Car"), lambda: (self._cur or tr("None")))

    def make_car_button(name: str):
      # 현재 선택 강조를 description으로 표시(색상제어 없이 텍스트만)
      desc = (lambda n=name: tr("Currently selected") if (self._cur == n) else "")
      return button_item(lambda n=name: n, lambda: tr("SELECT"), description=desc,
                         callback=lambda n=name: self._select_car(n))

    items = [header] + [make_car_button(n) for n in self._car_names]
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def show_event(self):
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)

  def _select_car(self, name: str):
    try:
      self._params.put("CarSelected3", name)
    except Exception:
      cloudlog.exception("failed to save CarSelected3")
      gui_app.push_widget(alert_dialog(tr("Failed to save selection")))
      return
    # 닫기
    gui_app.pop_widget()
    gui_app.pop_widget()

# Kans: CarSelector - Brand List Widget (NO MultiOptionDialog, NO pkgutil)
class BrandListDialog(Widget):
  def __init__(self, params: Params, brands: list[str]):
    super().__init__()
    self._params = params
    self._brands = brands

    header = text_item(lambda: tr("Select Brand"), lambda: tr("Choose a vehicle brand"))

    def make_brand_button(brand: str):
      return button_item(lambda b=brand: b, lambda: tr("OPEN"),
                         callback=lambda b=brand: self._open_brand(b))

    items = [header] + [make_brand_button(b) for b in self._brands]
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def show_event(self):
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)

  def _open_brand(self, brand: str):
    # brand: "GM" / "HYUNDAI" / "TOYOTA" ...
    try:
      car_names = DeviceLayout._get_car_doc_names_by_brand_static(brand)
      if not car_names:
        gui_app.push_widget(alert_dialog(tr("No car list found.")))
        return

      cur = self._params.get("CarSelected3")
      if isinstance(cur, (bytes, bytearray)):
        cur = cur.decode(errors="ignore")
      cur = cur or None

      gui_app.push_widget(CarListDialog(self._params, brand, car_names, cur))
    except Exception:
      cloudlog.exception("failed to open car list (brand)")
      gui_app.push_widget(alert_dialog(tr("Car selector error")))


class DeviceLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    self._select_language_dialog: MultiOptionDialog | None = None
    self._fcc_dialog: HtmlModal | None = None
    self._training_guide: TrainingGuide | None = None

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

    ui_state.add_offroad_transition_callback(self._offroad_transition)

  def _initialize_items(self):
    self._pair_device_btn = button_item(lambda: tr("Pair Device"), lambda: tr("PAIR"), lambda: tr(DESCRIPTIONS['pair_device']),
                                        callback=lambda: gui_app.push_widget(PairingDialog()))
    self._pair_device_btn.set_visible(lambda: not ui_state.prime_state.is_paired())

    # Kans: CarSelector (Dual Button)
    self._select_car_btn = dual_button_item(lambda: tr("Select Car"), lambda: self._get_selected_car_button_text(),
      left_callback=self._open_brand_list, right_callback=self._delete_selected_car_prompt,
      description=lambda: self._get_selected_car_label())
    self._reset_calib_btn = button_item(lambda: tr("Reset Calibration"), lambda: tr("RESET"), lambda: tr(DESCRIPTIONS['reset_calibration']),
                                        callback=self._reset_calibration_prompt)
    self._reset_calib_btn.set_description_opened_callback(self._update_calib_description)

    # Kans: triple btn for Pwr Off
    self._power_off_btn = triple_button_item(lambda: tr("Reset Calibration"), lambda: tr("Reboot"), lambda: tr("Power Off"),
                            left_callback=self._reset_calibration_prompt, mid_callback=self._reboot_prompt, right_callback=self._power_off_prompt)
    items = [
      text_item(lambda: tr("Dongle ID"), self._params.get("DongleId") or (lambda: tr("N/A"))),
      text_item(lambda: tr("Serial"), self._params.get("HardwareSerial") or (lambda: tr("N/A"))),

      # Kans: CarSelector (Dual Button row)
      self._select_car_btn,
      self._pair_device_btn,
      button_item(lambda: tr("Driver Camera"), lambda: tr("PREVIEW"), lambda: tr(DESCRIPTIONS['driver_camera']),
                  callback=lambda: gui_app.push_widget(DriverCameraDialog()), enabled=ui_state.is_offroad),
      self._power_off_btn,
      self._reset_calib_btn,
      button_item(lambda: tr("Review Training Guide"), lambda: tr("REVIEW"), lambda: tr(DESCRIPTIONS['review_guide']),
                  self._on_review_training_guide, enabled=ui_state.is_offroad),
      button_item(lambda: tr("Regulatory"), lambda: tr("VIEW"), callback=self._on_regulatory, enabled=ui_state.is_offroad),
      button_item(lambda: tr("Change Language"), lambda: tr("CHANGE"), callback=self._show_language_dialog),
    ]
    return items

  def _offroad_transition(self):
    self._power_off_btn.action_item.right_button.set_visible(True) # Kans: visible pwr Btn

  def show_event(self):
    super().show_event()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)

  def _show_language_dialog(self):
    def handle_language_selection(result: DialogResult):
      if result == DialogResult.CONFIRM and self._select_language_dialog:
        selected_language = multilang.languages[self._select_language_dialog.selection]
        multilang.change_language(selected_language)
        self._update_calib_description()
      self._select_language_dialog = None

    self._select_language_dialog = MultiOptionDialog(tr("Select a language"), multilang.languages, multilang.codes[multilang.language],
                                                     option_font_weight=FontWeight.UNIFONT, callback=handle_language_selection)
    gui_app.push_widget(self._select_language_dialog)

  def _reset_calibration_prompt(self):
    if ui_state.engaged:
      gui_app.push_widget(alert_dialog(tr("Disengage to Reset Calibration")))
      return

    def reset_calibration(result: DialogResult):
      # Check engaged again in case it changed while the dialog was open
      if ui_state.engaged or result != DialogResult.CONFIRM:
        return

      self._params.remove("CalibrationParams")
      self._params.remove("LiveTorqueParameters")
      self._params.remove("LiveParameters")
      self._params.remove("LiveParametersV2")
      self._params.remove("LiveDelay")
      self._params.put_bool("OnroadCycleRequested", True)
      self._update_calib_description()

    dialog = ConfirmDialog(tr("Are you sure you want to reset calibration?"), tr("Reset"), callback=reset_calibration)
    gui_app.push_widget(dialog)

  def _update_calib_description(self):
    desc = tr(DESCRIPTIONS['reset_calibration'])

    calib_bytes = self._params.get("CalibrationParams")
    if calib_bytes:
      try:
        calib = messaging.log_from_bytes(calib_bytes, log.Event).liveCalibration

        if calib.calStatus != log.LiveCalibrationData.Status.uncalibrated:
          pitch = math.degrees(calib.rpyCalib[1])
          yaw = math.degrees(calib.rpyCalib[2])
          desc += tr(" Your device is pointed {:.1f}° {} and {:.1f}° {}.").format(abs(pitch), tr("down") if pitch > 0 else tr("up"),
                                                                                  abs(yaw), tr("left") if yaw > 0 else tr("right"))
      except Exception:
        cloudlog.exception("invalid CalibrationParams")

    lag_perc = 0
    lag_bytes = self._params.get("LiveDelay")
    if lag_bytes:
      try:
        lag_perc = messaging.log_from_bytes(lag_bytes, log.Event).liveDelay.calPerc
      except Exception:
        cloudlog.exception("invalid LiveDelay")
    if lag_perc < 100:
      desc += tr("<br><br>Steering lag calibration is {}% complete.").format(lag_perc)
    else:
      desc += tr("<br><br>Steering lag calibration is complete.")

    torque_bytes = self._params.get("LiveTorqueParameters")
    if torque_bytes:
      try:
        torque = messaging.log_from_bytes(torque_bytes, log.Event).liveTorqueParameters
        # don't add for non-torque cars
        if torque.useParams:
          torque_perc = torque.calPerc
          if torque_perc < 100:
            desc += tr(" Steering torque response calibration is {}% complete.").format(torque_perc)
          else:
            desc += tr(" Steering torque response calibration is complete.")
      except Exception:
        cloudlog.exception("invalid LiveTorqueParameters")

    desc += "<br><br>"
    desc += tr("openpilot is continuously calibrating, resetting is rarely required. " +
               "Resetting calibration will restart openpilot if the car is powered on.")

    self._reset_calib_btn.set_description(desc)

  def _reboot_prompt(self):
    if ui_state.engaged:
      gui_app.push_widget(alert_dialog(tr("Disengage to Reboot")))
      return

    def perform_reboot(result: DialogResult):
      if not ui_state.engaged and result == DialogResult.CONFIRM:
        self._params.put_bool_nonblocking("DoReboot", True)

    dialog = ConfirmDialog(tr("Are you sure you want to reboot?"), tr("Reboot"), callback=perform_reboot)
    gui_app.push_widget(dialog)

  def _power_off_prompt(self):
    if ui_state.engaged:
      gui_app.push_widget(alert_dialog(tr("Disengage to Power Off")))
      return

    def perform_power_off(result: DialogResult):
      if not ui_state.engaged and result == DialogResult.CONFIRM:
        self._params.put_bool_nonblocking("DoShutdown", True)

    dialog = ConfirmDialog(tr("Are you sure you want to power off?"), tr("Power Off"), callback=perform_power_off)
    gui_app.push_widget(dialog)

  def _on_regulatory(self):
    if not self._fcc_dialog:
      self._fcc_dialog = HtmlModal(os.path.join(BASEDIR, "selfdrive/assets/offroad/fcc.html"))
    gui_app.push_widget(self._fcc_dialog)

  def _on_review_training_guide(self):
    if not self._training_guide:
      self._training_guide = TrainingGuide()
    gui_app.push_widget(self._training_guide)

  # Kans: CarSelector
  def _get_selected_car_button_text(self) -> str:
    cur = self._params.get("CarSelected3")
    if isinstance(cur, (bytes, bytearray)):
      cur = cur.decode(errors="ignore")
    cur = (cur or "").strip()
    if not cur:
      return tr("Delete Car")
    max_len = 18
    return cur if len(cur) <= max_len else (cur[:max_len - 1] + "…")

  def _get_selected_car_label(self) -> str:
    cur = self._params.get("CarSelected3")
    if isinstance(cur, (bytes, bytearray)):
      cur = cur.decode(errors="ignore")
    cur = (cur or "").strip()
    return (tr("Selected: ") + cur) if cur else tr("Selected: None")

  def _delete_selected_car_prompt(self):
    cur = self._params.get("CarSelected3")
    if isinstance(cur, (bytes, bytearray)):
      cur = cur.decode(errors="ignore")
    if not cur:
      gui_app.push_widget(alert_dialog(tr("No selection to delete.")))
      return

    def do_delete(res: DialogResult):
      if res == DialogResult.CONFIRM:
        self._params.remove("CarSelected3")

    gui_app.push_widget(ConfirmDialog(tr("Delete current selection?") + f"\n{cur}", tr("Delete"), callback=do_delete))

  def _open_brand_list(self):
    try:
      brands = self._discover_brands()  # ["GM", "HYUNDAI", "TOYOTA", ...]
      if not brands:
        gui_app.push_widget(alert_dialog(tr("No brands found.")))
        return
      gui_app.push_widget(BrandListDialog(self._params, brands))
    except Exception:
      cloudlog.exception("failed to open brand list")
      gui_app.push_widget(alert_dialog(tr("Car selector error")))

  def _discover_brands(self) -> list[str]:
    """
    opendbc/car/<brand>/values.py 를 직접 스캔해서 CAR가 있는 브랜드만 반환
    returns: ["GM","HYUNDAI","TOYOTA", ...]
    """
    brands: list[str] = []
    try:
      import opendbc.car as car_pkg
      car_dir = os.path.dirname(car_pkg.__file__)  # .../opendbc/car

      for name in os.listdir(car_dir):
        if name.startswith("_"):
          continue
        path = os.path.join(car_dir, name)
        if not os.path.isdir(path):
          continue
        values_py = os.path.join(path, "values.py")
        if not os.path.isfile(values_py):
          continue

        try:
          mod = importlib.import_module(f"opendbc.car.{name}.values")
          if hasattr(mod, "CAR"):
            brands.append(name.upper())
        except Exception:
          continue

    except Exception:
      cloudlog.exception("brand discovery failed")

    # fallback
    if not brands:
      brands = ["GM", "HYUNDAI"]

    return sorted(set(brands))

  @staticmethod
  def _get_car_doc_names_by_brand_static(brand: str) -> list[str]:
    """
    brand: "GM" / "HYUNDAI" / "TOYOTA" ...
    """
    cars: list[str] = []
    try:
      mod = importlib.import_module(f"opendbc.car.{brand.lower()}.values")
      CAR = getattr(mod, "CAR", None)
      if CAR is None:
        return []

      for platform in CAR:
        try:
          for doc in platform.config.car_docs:
            name = getattr(doc, "name", None)
            if name:
              cars.append(name)
        except Exception:
          continue
    except Exception:
      cloudlog.exception(f"failed to load {brand} car docs")

    return sorted(set(cars))