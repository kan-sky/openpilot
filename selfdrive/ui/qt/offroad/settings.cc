#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <string>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

TogglesPanel::TogglesPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  QList<ParamControl*> toggles;

  toggles.append(new ParamControl("OpenpilotEnabledToggle",
                                  "openpilot 사용",
                                  "Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off.",
                                  "../assets/offroad/icon_openpilot.png",
                                  this));

  toggles.append(new ParamControl("MetricResetSwitch",
                                  "화면 정보표시(tap me; reset)",
                                  "차량이 켜져 있을 때 현재 속도를 눌러 편집 모드를 잠금해제한 다음 더 눌러 오른쪽에 표시된 메트릭 수를 순환하십시오. 표시되는 정보를 변경하려면 각 메트릭을 누릅니다. 이 토글을 사용하여 다음 번 차량 시동 시 주행 거리, EV 소비 및 효율 트립 및 5mi/8km 메트릭을 0으로 재설정합니다.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));
  toggles.append(new ParamControl("DisableDisengageOnGas",
                                  "가스페달 인게이지",
                                  "Keep steering when gas (accelerator) is pressed",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("MADSEnabled",
                                  "[GM] MADS(tap me)",
                                  "MADS = 보조 안전운전Modified Assistive Driving Safety. 3개의 독립적으로 전환 가능한 상시 ON 기능: 자동 조향, 리드 브레이크 및 원 페달 모드. 이 토글을 사용하려면 어느 것이라도 사용하면 됨. LKAS 버튼으로, 인게이지 들어가기 전, 심지어 브레이크를 밟은 경우에도 자동 조향을 전환합니다. 메인 크루즈 버튼으로 모든 OP 컨트롤을 차단하십시오. MADS가 실행 중일 때 최대 속도는 MADS 아이콘으로 대체됩니다.",
                                  "../assets/offroad/icon_car_MADS.png",
                                  this));
  toggles.append(new ParamControl("MADSLeadBraking",
                                   "MADS 리드브레이크(tap me)",
                                   "MADS 아이콘이 표시중일 때, ACC Distance 버튼을 눌러 전환합니다. 오픈파일럿이 부드럽게 리드카 뒤에 멈출 것이다. 가스/브레이크를 가볍게 터치해도 리드 브레이크가 오버라이드됩니다. 이 기능을 활성화하면 MADS 아이콘 주위에 흰색 원이 추가로 나타납니다.",
                                   "../assets/offroad/icon_car_MADS.png",
                                   this));
  toggles.append(new ParamControl("MADSOnePedalMode",
                                  "MADS One-pedal mode (tap me)",
                                  "리젠 패들을 두 번 누르거나 MADS 아이콘을 눌러 전환합니다. 활성화되면 MADS 아이콘이 색상이 지정된 원 페달 아이콘으로 바뀌고, L 모드에서 가스 페달을 밟지 않을 때 오파가 가벼운 제동을 걸어 정지시킵니다. 원 페달 한 번에 스톱시키기: 리젠 패들로 8Km 미만을 유지하면 원페달 모드가 일시적으로 중지되며, 주행을 재개할 때 꺼집니다.",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("MADSPauseBlinkerSteering",
                                  "MADS 스티어링 일시 중지",
                                  "MADS가 활성화되어 있을 때 깜빡이를 켜고 시속 70키로 이하로 감속하면 급격한 회전을 쉽게 수행하기 위해 스티어링이 일시 중지됩니다(오파 의도와 반대일 수 있음). 가속을 재개하거나 다시 70키로를 초과하면 스티어링이 일시 중지가 해제됩니다.",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));

  toggles.append(new ParamControl("OPParamsLiveTuneEnabled",
                                  "SSH를 통한 사용자자 맞춤조정",
                                  "SSH를 통해 장치에 연결하고 '/data/openpilot/' 디렉토리에서 './opparams.py'를 실행하면 많은 기능을 원하는 대로 미세 조정할 수 있습니다. 조정 가능한 대부분의 파라미터는 라이브 튜닝이 가능하며, 이 토글이 활성화된 경우, 주행 중에 수정할 수 있으며  즉시 적용됩니다. 이 토글이 활성화되지 않은 경우, 차량 또는 OpenPilot 재시작이 필요한 \"시작\" 매개 변수가 된다. 이 토글의 변경 사항은 다음에 자동차를 시동할 때 적용되며 'opparam'을 다시 시작해야 합니다.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));

  toggles.append(new ParamControl("OPParamsLateralOverride",
                                  "횡가속 사용자맞춤",
                                  "This enables the lateral tune specified in opParams.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));

  toggles.append(new ParamControl("OPParamsLongitudinalOverride",
                                  "롱컨 사용자맞춤",
                                  "This enables the longitudinal tune specified in opParams.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));
  
  toggles.append(new ParamControl("LongRangeLeadsEnabled",
                                  "앞차 인식거리 확장(alpha)",
                                  "앞차 인식거리를 순정보다 40% 정도 확장함과 동시에 레이더에 의한 인식율도 10% 확장시킴.",
                                  "../assets/offroad/icon_plus.png",
                                  this));

  toggles.append(new ParamControl("ExtendedRadar",
                                  "레이더 확장(alpha:벌트엔 사용하지 말것.)",
                                  "모든 차량 트레킹이 가능해짐; 선두차에 대한 브레이킹, 확장된 앞차인식, 차선위치 자동조정, 마주오는 차선과 진행하는 차선 그리기 등을 사용하기 위해서 필요함.",
                                  "../assets/offroad/icon_plus.png",
                                  this));
  toggles.append(new ParamControl("TurnVisionControl",
                                  "비전카메라 기반 커브 컨트롤",
                                  "Use vision path predictions to estimate the appropiate speed to drive through turns ahead.",
                                  "../assets/offroad/icon_slow_curves_vision.png",
                                  this));
  toggles.append(new ParamControl("TurnSpeedControl",
                                  "지도기반 커브 컨트롤(권장되지 않음)",
                                  "Use curvature info from map data to define speed limits to take turns ahead",
                                  "../assets/offroad/icon_slow_curves_map.png",
                                  this));
  toggles.append(new ParamControl("SpeedLimitControl",
                                  "속도제한 컨트롤(권장되지 않음)",
                                  "Use speed limit signs information from map data and car interface to automatically adapt cruise speed to road limits.",
                                  "../assets/offroad/icon_speed_limit_sign.png",
                                  this));
  toggles.append(new ParamControl("EUSpeedLimitStyle",
                                  "유럽형 속도제한 표시(권장되지 않음)",
                                  "If enabled, show EU style circular sign. If disabled, show US/Canada style rectangular sign.",
                                  "../assets/offroad/icon_speed_limit_sign.png",
                                  this));
  toggles.append(new ParamControl("SpeedLimitPercOffset",
                                  "속도제한 오프셑(권장되지 않음)",
                                  "Set speed limit slightly higher than actual speed limit for a more natural drive.",
                                  "../assets/offroad/icon_speed_limit_percent.png",
                                  this));
  toggles.append(new ParamControl("ReverseSpeedAdjust",
                                  "크루즈 버튼 리버스",
                                  "짧게/길게 누를시 5/1 단위로 속도 변경.",
                                  "../assets/offroad/icon_stock_adjust_speed.png",
                                  this));
  toggles.append(new ParamControl("CruiseSpeedOffset",
                                  "크루즈 속도 오프셑(+3mph, 권장되지 않음)",
                                  "When adjusting, cruise speed will be {8, 13, 18, 23, 28} mph.",
                                  "../assets/offroad/icon_speed_offset.png",
                                  this));
  toggles.append(new ParamControl("LanePositionEnabled",
                                  "차선위치 조정",
                                  "화면상 나타나는 화살표를 터치하여 임시로 주행위치 조정.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AutoAutoLanePosition",
                                  "차선위지 자동조정",
                                  "10mph 이상의 속도로 주행할 때 자동으로 차선 위치를 설정하고 차선을 확보하며 교통이 혼잡할 때 자동으로 차선을 설정합니다. 자동 차선 위치는 가능한 경우/안전할 때 인접 차량으로부터 더 멀리 떨어져 있게 해줍니다.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AccelModeButton",
                                  "가속프로파일",
                                  "노말(스톡), 에코, 스포츠모드 가속 프로파일.",
                                  "../assets/offroad/icon_rocket.png",
                                  this));
  toggles.append(new ParamControl("DynamicFollowToggle",
                                  "자동추종 모드",
                                  "Automatically (and imperceptibly) switch between close/medium/far follow profiles based on speed and traffic.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("NudgelessLaneChange",
                                  "핸들넛지 없이 자동 차선변경(1초후) ⚠️",
                                  "Perform lane change without requiring nudge from driver",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));
  toggles.append(new ParamControl("GMAutoHold",
                                  "[GM]오토홀드",
                                  "Holds brakes automatically after coming to a complete stop, even when OP is disengaged.",
                                  "../assets/offroad/icon_gm_autohold.png",
                                  this));
  toggles.append(new ParamControl("Coasting",
                                  "[GM]타력주행(tap me)",
                                  "OP will allow the car to coast above the set speed rather than use engine/regen/friction braking. If you also have the \"Brake indicator\" toggle enabled, you can toggle coasting while driving by tapping the brake indicator, but you can only disable coasting while driving if you're traveling below your set speed (or at any speed if you have the \"Engine/regen braking\" toggle enabled).",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("CoastingBrakeOverSpeed",
                                  "[GM]타력주행시: 세팅속도보다 15% 가속",
                                  "타력주행시, 세팅속도의 15%까지 가속해도 크루즈 감속 하지 않음.",
                                  "../assets/offroad/icon_speed_offset.png",
                                  this));
  toggles.append(new ParamControl("CoastingDL",
                                  "[Volt]D/L기어로 타력주행",
                                  "벌트에서 D기어 타력주행, L기어 쿠루즈속도 주행",
                                  "../assets/offroad/icon_gear_shifter.png",
                                  this));
  toggles.append(new ParamControl("RegenBraking",
                                  "[GM]리젠 브레킹",
                                  "Disable friction braking when OP is slowing to maintain cruise/speed limit; still brake for following/curves",
                                  "../assets/img_brake.png",
                                  this));
  toggles.append(new ParamControl("BrakeIndicator",
                                  "[GM]Power/Brake 막대표시",
                                  "Brake indicator at bottom-right when driving or power meter to right. Tap indicator or meter to change. Circle at indicator center grows and turns red to indicate the level of braking. Pulses immediately after starting car to let you know it's on.",
                                  "../assets/offroad/icon_brake_disc.png",
                                  this));
  toggles.append(new ParamControl("CustomSounds",
                                  "대체 사운드 사용",
                                  "Uses alternative set of sound effects.",
                                  "../assets/offroad/icon_custom_sounds.png",
                                  this));
  toggles.append(new ParamControl("SilentEngageDisengage",
                                  "디스/인게이지 사운드 무음",
                                  "Mute engage and disengage sounds.",
                                  "../assets/offroad/icon_mute.png",
                                  this));
  toggles.append(new ParamControl("IgnoreMissingNVME",
                                  "NVME 드라이브 오류무시",
                                  "Prevent an error about missing NVME drive from showing on 32GB C3's. (restart device for change to take effect)",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("GrayPandaSupport",
                                  "그레이 판다 지원",
                                  "Necessary to run on gray panda",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("FPVolt",
                                  "Volt핑거프린트 강제",
                                  "Forces Volt fingerprint",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("LowOverheadMode",
                                  "디바이스 파워 감소하기",
                                  "Decreases device power, CPU, and storage use for running better on older hardware by: 1) defaulting to medium brightness (tap DM icon to change), 2) disable onroad logging (loggerd and proclogd). Your device will not keep or upload logs with this enabled!",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("WeatherDisplayEnabled",
                                  "기상정보 표시",
                                  "간단한 레이아웃과 전체 레이아웃을 전환하려면 날씨 정보를 누릅니다. 원하는 경우 /data/OpenWeatherMap_apiKey.txt에 자신의 OpenWeatherMap.org API 키를 입력합니다.",
                                  "../assets/weather/10n.png",
                                  this));
  toggles.append(new ParamControl("WeatherSafetyEnabled",
                                  "기상 안전장치",
                                  "OpenPilot은 자동으로 더 부드러운 가속과 곡선 제동, 더 먼 거리의 교통 흐름을 따르거나 사용 중 제한 속도 오프셋을 낮추어줍니다. 원하는 경우 /data/OpenWeatherMap_apiKey.txt 파일에 여러분 자신의 OpenWeatherMap.org API 키를 입력합십시오.",
                                  "../assets/weather/10n.png",
                                  this));
  toggles.append(new ParamControl("ColorPath",
                                  "경로색상",
                                  "횡가속도 보정량에 따른 경로색상",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AlternateColors",
                                  "대체 컬러사용",
                                  "Use alternate color set.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AdjacentPaths",
                                  "반대/진행차선경로 표시(권장되지않음)",
                                  "레이더확장기능 토글On 필요, 반대차선은 Red, 진행차선은 Green.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("PrintLeadInfo",
                                  "리드카 정보표시",
                                  "리드카 표시기 옆에 리드차량 도달시간 및 거리, 절대 및 상대속도 표시",
                                  "../assets/offroad/icon_metric.png",
                                  this));
  toggles.append(new ParamControl("PrintAdjacentLeadSpeeds",
                                  "모든차량 감지 표시(권장되지 않음)",
                                  "[Requires extended radar toggle] Print speeds of all cars being tracked by radar and/or vision. Tap at screen bottom in the middle of the path to toggle display modes between printing inside the indicator or along the bottom of the screen, out from the center to the left/right in order of distance, so the numbers closest to the center are for the more immediate cars. Cars are also indicated onscreen as oncoming (red) or ongoing (green).",
                                  "../assets/offroad/icon_metric.png",
                                  this));

  toggles.append(new ParamControl("EnableTorqueControl",
                                  "토크조향 컨트롤",
                                  "(Restart car to take effect) Use the newer torque-based steering control that steers by achieving a target amount of lateral acceleration rather than achieving a target steering angle. Torque tune is only available in the Volt.",
                                  "../assets/offroad/icon_openpilot.png",
                                  this));


  toggles.append(new ParamControl("IsLdwEnabled",
                                  "차선이탈 경고",
                                  "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31mph (50kph).",
                                  "../assets/offroad/icon_warning.png",
                                  this));
  toggles.append(new ParamControl("IsRHD",
                                  "우측 운전석 사용",
                                  "Allow openpilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat.",
                                  "../assets/offroad/icon_openpilot_mirrored.png",
                                  this));
  toggles.append(new ParamControl("IsMetric",
                                  "미터법 사용",
                                  "Display speed in km/h instead of mp/h.",
                                  "../assets/offroad/icon_metric.png",
                                  this));

  toggles.append(new ParamControl("UploadRaw",
                                  "로그기록 업로드",
                                  "Upload full logs and full resolution video by default while on WiFi. If not enabled, individual logs can be marked for upload at my.comma.ai/useradmin.",
                                  "../assets/offroad/icon_network.png",
                                  this));

  toggles.append(new ParamControl("DisableOnroadUploads",
                                  "업로드 금지",
                                  "Completely disable uploads when onroad. Necessary to avoid high data use when connected to wifi hotspot.",
                                  "../assets/offroad/icon_network.png",
                                  this));

  ParamControl *record_toggle = new ParamControl("RecordFront",
                                                 "녹화및 업로드",
                                                 "Upload data from the driver facing camera and help improve the driver monitoring algorithm.",
                                                 "../assets/offroad/icon_monitoring.png",
                                                 this);
  toggles.append(record_toggle);
  toggles.append(new ParamControl("EndToEndToggle",
                                  "차선사용 무시(Alpha)",
                                  "In this mode openpilot will ignore lanelines and just drive how it thinks a human would.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  
  toggles.append(new ParamControl("HandsOnWheelMonitoring",
                                  "핸들 손올림 감지",
                                  "Monitor and alert when driver is not keeping the hands on the steering wheel.",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));
  toggles.append(new ParamControl("ShowDebugUI",
                                  "debug UI표시",
                                  "Show UI elements that aid debugging.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));

#ifdef ENABLE_MAPS
  toggles.append(new ParamControl("NavSettingTime24h",
                                  "24시간제 사용",
                                  "Use 24h format instead of am/pm",
                                  "../assets/offroad/icon_metric.png",
                                  this));
#endif

  bool record_lock = Params().getBool("RecordFrontLock");
  record_toggle->setEnabled(!record_lock);

  for(ParamControl *toggle : toggles) {
    if(main_layout->count() != 0) {
      main_layout->addWidget(horizontal_line());
    }
    main_layout->addWidget(toggle);
  }
}

DevicePanel::DevicePanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  Params params = Params();
  main_layout->addWidget(new LabelControl("Dongle ID", getDongleId().value_or("N/A")));
  main_layout->addWidget(horizontal_line());

  QString serial = QString::fromStdString(params.get("HardwareSerial", false));
  main_layout->addWidget(new LabelControl("Serial", serial));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton("Soft restart");
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  main_layout->addWidget(horizontal_line());
  main_layout->addLayout(reset_layout);

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton("Reset Calibration");
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration and live params?", this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  main_layout->addLayout(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl("Driver Camera", "PREVIEW",
                                        "Preview the driver facing camera to help optimize device mounting position for best driver monitoring experience. (vehicle must be off)");
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });

  QString resetCalibDesc = "openpilot requires the device to be mounted within 4° left or right and within 5° up or down. openpilot is continuously calibrating, resetting is rarely required.";
  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", resetCalibDesc);
  connect(resetCalibBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      Params().remove("CalibrationParams");
    }
  });
  connect(resetCalibBtn, &ButtonControl::showDescription, [=]() {
    QString desc = resetCalibDesc;
    std::string calib_bytes = Params().get("CalibrationParams");
    if (!calib_bytes.empty()) {
      try {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
        auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
        if (calib.getCalStatus() != 0) {
          double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
          double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
          desc += QString(" Your device is pointed %1° %2 and %3° %4.")
                                .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "up" : "down",
                                     QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "right" : "left");
        }
      } catch (kj::Exception) {
        qInfo() << "invalid CalibrationParams";
      }
    }
    resetCalibBtn->setDescription(desc);
  });

  ButtonControl *retrainingBtn = nullptr;
  if (!params.getBool("Passive")) {
    retrainingBtn = new ButtonControl("Review Training Guide", "REVIEW", "Review the rules, features, and limitations of openpilot");
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm("Are you sure you want to review the training guide?", this)) {
        Params().remove("CompletedTrainingVersion");
        emit reviewTrainingGuide();
      }
    });
  }

  ButtonControl *regulatoryBtn = nullptr;
  if (Hardware::TICI()) {
    regulatoryBtn = new ButtonControl("Regulatory", "VIEW", "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file(ASSET_PATH.toStdString() + "/offroad/fcc.html");
      RichTextDialog::alert(QString::fromStdString(txt), this);
    });
  }

  for (auto btn : {dcamBtn, resetCalibBtn, retrainingBtn, regulatoryBtn}) {
    if (btn) {
      main_layout->addWidget(horizontal_line());
      connect(parent, SIGNAL(offroadTransition(bool)), btn, SLOT(setEnabled(bool)));
      main_layout->addWidget(btn);
    }
  }

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton("Reboot");
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reboot?", this)) {
      Hardware::reboot();
    }
  });

  QPushButton *poweroff_btn = new QPushButton("Power Off");
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to power off?", this)) {
      Hardware::poweroff();
    }
  });

  setStyleSheet(R"(
    QPushButton {
      height: 120px;
      border-radius: 15px;
    }
    #reboot_btn { background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  main_layout->addLayout(power_layout);
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : QWidget(parent) {
  gitBranchLbl = new LabelControl("Git Branch");
  gitCommitLbl = new LabelControl("Git Commit");
  osVersionLbl = new LabelControl("OS Version");
  versionLbl = new LabelControl("Version", "", QString::fromStdString(params.get("ReleaseNotes")).trimmed());
  lastUpdateLbl = new LabelControl("Last Update Check", "", "The last time openpilot successfully checked for an update. The updater only runs while the car is off.");
  updateBtn = new ButtonControl("Check for Update", "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    if (params.getBool("IsOffroad")) {
      fs_watch->addPath(QString::fromStdString(params.getParamPath("LastUpdateTime")));
      fs_watch->addPath(QString::fromStdString(params.getParamPath("UpdateFailedCount")));
      updateBtn->setText("CHECKING");
      updateBtn->setEnabled(false);
    }
    std::system("pkill -1 -f selfdrive.updated");
  });

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QWidget *widgets[] = {versionLbl, lastUpdateLbl, updateBtn, gitBranchLbl, gitCommitLbl, osVersionLbl};
  for (int i = 0; i < std::size(widgets); ++i) {
    main_layout->addWidget(widgets[i]);
    main_layout->addWidget(horizontal_line());
  }

  auto uninstallBtn = new ButtonControl("Uninstall " + getBrand(), "UNINSTALL");
  connect(uninstallBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to uninstall?", this)) {
      Params().putBool("DoUninstall", true);
    }
  });
  connect(parent, SIGNAL(offroadTransition(bool)), uninstallBtn, SLOT(setEnabled(bool)));
  main_layout->addWidget(uninstallBtn);

  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    int update_failed_count = params.get<int>("UpdateFailedCount").value_or(0);
    if (path.contains("UpdateFailedCount") && update_failed_count > 0) {
      lastUpdateLbl->setText("failed to fetch update");
      updateBtn->setText("CHECK");
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  auto tm = params.get("LastUpdateTime");
  if (!tm.empty()) {
    lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
  }

  versionLbl->setText(getBrandVersion());
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText("CHECK");
  updateBtn->setEnabled(true);
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

QWidget * network_panel(QWidget * parent) {
#ifdef QCOM
  QWidget *w = new QWidget(parent);
  QVBoxLayout *layout = new QVBoxLayout(w);
  layout->setSpacing(30);

  // wifi + tethering buttons
  auto wifiBtn = new ButtonControl("WiFi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  layout->addWidget(wifiBtn);
  layout->addWidget(horizontal_line());

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  layout->addWidget(tetheringBtn);
  layout->addWidget(horizontal_line());

  // SSH key management
  layout->addWidget(new SshToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new SshControl());

  layout->addStretch(1);
#else
  Networking *w = new Networking(parent);
#endif
  return w;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("×");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  QList<QPair<QString, QWidget *>> panels = {
    {"Device", device},
    {"Network", network_panel(this)},
    {"Toggles", new TogglesPanel(this)},
    {"Software", new SoftwarePanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({"Navigation", map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup();
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != "Network" ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}
