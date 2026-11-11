#pragma once

#include "opendbc/safety/declarations.h"

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
#define GM_COMMON_RX_CHECKS \
    {.msg = {{0x184, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0x34A, 0, 5, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0x1E1, 0, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0xF1, 0, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0x1C4, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0xC9, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define GM_ACC_RX_CHECKS \
    {.msg = {{0xBE, 0, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},    /* Volt, Silverado, Acadia Denali */ \
             {0xBE, 0, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},    /* Bolt EUV */ \
             {0xBE, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},  /* Escalade */ \

enum {
  GM_BTN_UNPRESS = 1,
  GM_BTN_RESUME = 2,
  GM_BTN_SET = 3,
  GM_BTN_MAIN = 5,
  GM_BTN_CANCEL = 6,
};

static void gm_rx_hook(const CANPacket_t *msg) {
  const int GM_STANDSTILL_THRSLD = 10;  // 0.311kph
  const int GM_GAS_INTERCEPTOR_THRESHOLD = 550;

  if (msg_matches(msg, 0x184U, 0U)) {
    int torque_driver_new = ((msg->data[6] & 0x7U) << 8) | msg->data[7];
    torque_driver_new = to_signed(torque_driver_new, 11);
    // update array of samples
    update_sample(&safety.torque_driver, torque_driver_new);
  }

  // sample rear wheel speeds
  if (msg_matches(msg, 0x34AU, 0U)) {
    int left_rear_speed = (msg->data[0] << 8) | msg->data[1];
    int right_rear_speed = (msg->data[2] << 8) | msg->data[3];
    safety.vehicle_moving = (left_rear_speed > GM_STANDSTILL_THRSLD) || (right_rear_speed > GM_STANDSTILL_THRSLD);
  }

  // ACC steering wheel buttons (GM_CAM is tied to the PCM)
  if (msg_matches(msg, 0x1E1U, 0U) && !safety_setup.gm_pcm_cruise) {
    int button = (msg->data[5] & 0x70U) >> 4;

    // enter controls on falling edge of set or rising edge of resume (avoids fault)
    bool set = (button != GM_BTN_SET) && (safety.cruise_button_prev == GM_BTN_SET);
    bool res = (button == GM_BTN_RESUME) && (safety.cruise_button_prev != GM_BTN_RESUME);
    if (set || res) {
      safety.controls_allowed = true;
    }

    // exit controls on cancel press
    if (button == GM_BTN_CANCEL) {
      safety.controls_allowed = false;
    }

    safety.cruise_button_prev = button;
  }

  // Reference for brake pressed signals:
  // https://github.com/commaai/openpilot/blob/master/selfdrive/car/gm/carstate.py
  // Prefer 0xC9 (ECMEngineStatus) when gm_force_brake_c9 is set, otherwise keep legacy behavior.
  // This allows SDGM/Traverse variants without 0xBE (ECMAcceleratorPos) to report brake correctly.
  // Kans: EV(Volt 등)는 0xBE(ECMAcceleratorPos)의 threshold=8 근처 노이즈로 brake_pressed가
  // 튀는 문제가 실제 로그로 확인돼서, EBCM 브레이크 페달 센서(0xF1)로 전환 (opgm/opendbc 참고)
  if (msg_matches(msg, 0xC9U, 0U) && safety_setup.gm_force_brake_c9) {
    safety.brake_pressed = GET_BIT(msg, 40U);
  } else if (msg_matches(msg, 0xF1U, 0U) && (safety_setup.gm_hw == GM_ASCM) && safety_setup.gm_ev) {
    safety.brake_pressed = msg->data[1] >= 6U;
  } else if (msg_matches(msg, 0xBEU, 0U) && ((safety_setup.gm_hw == GM_ASCM) || (safety_setup.gm_hw == GM_SDGM)) && !safety_setup.gm_ev) {
    safety.brake_pressed = msg->data[1] >= 8U;
  } else if (msg_matches(msg, 0xC9U, 0U) && (safety_setup.gm_hw == GM_CAM)) {
    safety.brake_pressed = GET_BIT(msg, 40U);
  }

  if (msg_matches(msg, 0x1C4U, 0U)) {
    if (!safety_setup.enable_gas_interceptor) {
      safety.gas_pressed = msg->data[5] != 0U;
    }

    // enter controls on rising edge of ACC, exit controls when ACC off
    if (safety_setup.gm_pcm_cruise && safety_setup.gm_has_acc) {
      bool cruise_engaged = (msg->data[1] >> 5) != 0U;
      pcm_cruise_check(cruise_engaged);
    }
  }

  if (msg_matches(msg, 0xBDU, 0U)) {
    safety.regen_braking = (msg->data[0] >> 4) != 0U;
  }

    // Pedal Interceptor
  if (msg_matches(msg, 0x201U, 0U) && safety_setup.enable_gas_interceptor) {
    // Pedal Interceptor: average between 2 tracks
    int track1 = ((msg->data[0] << 8) + msg->data[1]);
    int track2 = ((msg->data[2] << 8) + msg->data[3]);
    int gas_interceptor = (track1 + track2) / 2;
    safety.gas_pressed = gas_interceptor > GM_GAS_INTERCEPTOR_THRESHOLD;
  }

  // ACC engaged status from camera/SDGM
  if (msg_matches(msg, 0x370U, 2U)) {
    bool cruise_engaged = GET_BIT(msg, 23U);  // ACCCmdActive
    if (safety_setup.gm_pcm_cruise) {
      pcm_cruise_check(cruise_engaged);
    } else {
      safety.cruise_engaged_prev = cruise_engaged;
    }
  }

  // AOL
  if (msg->addr == 0xC9U) {
    safety.acc_main_on = GET_BIT(msg, 29U);
  }
}

static bool gm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GM_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 20,
    .max_rate_down = 25,
    .driver_torque_allowance = 65,
    .driver_torque_multiplier = 4,
    .max_rt_delta = 128,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // BRAKE: safety check
  if (msg->addr == 0x315U) {
    int brake = ((msg->data[0] & 0xFU) << 8) + msg->data[1];
    brake = (0x1000 - brake) & 0xFFF;
    if (longitudinal_brake_checks(brake, *safety_setup.gm_long_limits)) {
      tx = false;
    }
  }

  // LKA STEER: safety check
  if (msg->addr == 0x180U) {
    int desired_torque = ((msg->data[0] & 0x7U) << 8) + msg->data[1];
    desired_torque = to_signed(desired_torque, 11);

    bool steer_req = GET_BIT(msg, 3U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, GM_STEERING_LIMITS)) {
      // tx = false;
    }
  }

  // GAS/REGEN: safety check
  if (msg->addr == 0x2CBU) {
    bool apply = GET_BIT(msg, 0U);
    if (apply && !safety.gm_prev_apply && !safety.controls_allowed) {
      if (safety.acc_main_on) {
        safety.controls_allowed = true;
      }        
    }
    safety.gm_prev_apply = apply;

    // convert float CAN signal to an int for gas checks: 22534 / 0.125 = 180272
    int gas_regen = (((msg->data[1] & 0x7U) << 16) | (msg->data[2] << 8) | msg->data[3]) - 180272U;

    bool violation = false;
    // Allow apply bit in pre-enabled and overriding states
    violation |= !safety.controls_allowed && apply;
    violation |= longitudinal_gas_checks(gas_regen, *safety_setup.gm_long_limits);

    if (violation) {
      tx = false;
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation with stock longitudinal
  if ((msg->addr == 0x1E1U) && (safety_setup.gm_pcm_cruise || safety_setup.gm_pedal_long)) {
    int button = (msg->data[5] >> 4) & 0x7U;

    bool allowed_btn = (button == GM_BTN_CANCEL) && safety.cruise_engaged_prev;
    // For CC_LONG or PCM cruise vehicles, allow SET/RESUME when cruise is engaged
    if (safety_setup.gm_pcm_cruise) {
      allowed_btn |= safety.cruise_engaged_prev && ((button == GM_BTN_SET) || (button == GM_BTN_RESUME) || (button == GM_BTN_UNPRESS));
    }

    if (!allowed_btn) {
      tx = false;
    }
  }

  // GAS: safety check (interceptor)
  if (msg->addr == 0x200U) {
    if (longitudinal_interceptor_checks(msg)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config gm_init(uint16_t param) {
  const uint16_t GM_PARAM_HW_CAM = 1;
  const uint16_t GM_PARAM_HW_CAM_LONG = 2;
  const uint16_t GM_PARAM_HW_ASCM_LONG = 4;
  const uint16_t GM_PARAM_NO_ACC = 8;
  const uint16_t GM_PARAM_PEDAL_LONG = 16;
  const uint16_t GM_PARAM_PEDAL_INTERCEPTOR = 32;  // TODO: this can be inferred
  const uint16_t GM_PARAM_EV = 64;
  const uint16_t GM_PARAM_HW_SDGM = 128;
  const uint16_t GM_PARAM_ASCM_INT = 256;
  const uint16_t GM_PARAM_FORCE_BRAKE_C9 = 512;

  // common safety checks assume unscaled integer values
  static const int GM_GAS_TO_CAN = 8;  // 1 / 0.125

  static const LongitudinalLimits GM_ASCM_LONG_LIMITS = {
    .max_gas = 1018 * GM_GAS_TO_CAN,
    .min_gas = -650 * GM_GAS_TO_CAN,
    .inactive_gas = -650 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

  static const CanMsg GM_ASCM_TX_MSGS[] = {{0x180, 0, 4, .check_relay = true}, {0x409, 0, 7, .check_relay = false},
                                           {0x40A, 0, 7, .check_relay = false}, {0x2CB, 0, 8, .check_relay = true},
                                           {0x370, 0, 6, .check_relay = false}, {0x200, 0, 6, .check_relay = false},
                                           {0x1E1, 0, 7, .check_relay = false}, {0xBD, 0, 7, .check_relay = false},  // pt bus
                                           {0xA1, 1, 7, .check_relay = false}, {0x306, 1, 8, .check_relay = false},
                                           {0x308, 1, 7, .check_relay = false}, {0x310, 1, 2, .check_relay = false},  // obs bus
                                           {0x315, 2, 5, .check_relay = false}};  // ch bus


  static const LongitudinalLimits GM_CAM_LONG_LIMITS = {
    .max_gas = 1346 * GM_GAS_TO_CAN,
    .min_gas = -540 * GM_GAS_TO_CAN,
    .inactive_gas = -500 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

#ifdef ALLOW_DEBUG
  // block PSCMStatus (0x184); forwarded through openpilot to hide an alert from the camera
  //
  // EBCMFrictionBrakeCmd (0x315) belongs on bus 0 (pt bus), not bus 2, for
  // camera-long GM cars - moving it to bus 2 here made panda silently drop
  // every brake command on a real 2021-22 Trailblazer (fwdCamera, non-SDGM),
  // which the car then reported as a permanent FrictionBrakeUnavailable
  // cruise fault from boot. A known-working reference build (pre-dating that
  // move) sends 0x315 on bus 0 and drives the same vehicle/harness fine.
  static const CanMsg GM_CAM_LONG_TX_MSGS[] = {{0x180, 0, 4, .check_relay = true}, {0x2CB, 0, 8, .check_relay = true},
                                               {0x370, 0, 6, .check_relay = true}, {0x200, 0, 6, .check_relay = true},
                                               {0x1E1, 0, 7, .check_relay = true}, {0x315, 0, 5, .check_relay = true},  // pt bus
                                               {0x184, 2, 8, .check_relay = true}};  // camera bus
#endif


  static const RxCheck gm_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    GM_ACC_RX_CHECKS
  };

  static const RxCheck gm_ev_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    GM_ACC_RX_CHECKS
    {.msg = {{0xBD, 0, 7, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

   static const RxCheck gm_ascm_int_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    GM_ACC_RX_CHECKS
  };

  static const RxCheck gm_no_acc_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0x3D1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // Non-ACC PCM
  };

  static const RxCheck gm_no_acc_ev_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xBD, 0, 7, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x3D1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // Non-ACC PCM
  };

  static const RxCheck gm_pedal_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xBD, 0, 7, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x3D1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // Non-ACC PCM
    {.msg = {{0x201, 0, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // pedal
  };

  static const CanMsg GM_CAM_TX_MSGS[] = {{0x180, 0, 4, .check_relay = true}, {0x200, 0, 6, .check_relay = false},  // pt bus
                                          {0x1E1, 2, 7, .check_relay = false}, {0x184, 2, 8, .check_relay = true}};  // camera bus

  //gm_hw = GET_FLAG(param, GM_PARAM_HW_CAM) ? GM_CAM : GM_ASCM;
  const bool gm_ascm_int = GET_FLAG(param, GM_PARAM_ASCM_INT);  
  if (GET_FLAG(param, GM_PARAM_HW_CAM)) {
    safety_setup.gm_hw = GM_CAM;
  } else if (GET_FLAG(param, GM_PARAM_HW_SDGM)) {
    safety_setup.gm_hw = GM_SDGM;
  } else {
    safety_setup.gm_hw = GM_ASCM;
  }

  safety_setup.gm_force_ascm = GET_FLAG(param, GM_PARAM_HW_ASCM_LONG);

  if ((safety_setup.gm_hw == GM_ASCM) || safety_setup.gm_force_ascm || gm_ascm_int) {
    safety_setup.gm_long_limits = &GM_ASCM_LONG_LIMITS;
  } else if ((safety_setup.gm_hw == GM_CAM) || (safety_setup.gm_hw == GM_SDGM)) {
    safety_setup.gm_long_limits = &GM_CAM_LONG_LIMITS;
  } else {
  }

  safety_setup.gm_pedal_long = GET_FLAG(param, GM_PARAM_PEDAL_LONG);
  safety_setup.gm_cam_long = GET_FLAG(param, GM_PARAM_HW_CAM_LONG);
  safety_setup.gm_pcm_cruise = (((safety_setup.gm_hw == GM_CAM) || (safety_setup.gm_hw == GM_SDGM)) && !safety_setup.gm_cam_long && !safety_setup.gm_force_ascm && !safety_setup.gm_pedal_long);
  safety_setup.gm_has_acc = !GET_FLAG(param, GM_PARAM_NO_ACC);
  safety_setup.enable_gas_interceptor = GET_FLAG(param, GM_PARAM_PEDAL_INTERCEPTOR);
  safety_setup.gm_force_brake_c9 = GET_FLAG(param, GM_PARAM_FORCE_BRAKE_C9);

  safety_config ret;
  if (safety_setup.gm_hw == GM_CAM) {
    if (safety_setup.gm_cam_long) {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_LONG_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_TX_MSGS);
    }
  } else {
    ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_ASCM_TX_MSGS);
  }

  safety_setup.gm_ev = GET_FLAG(param, GM_PARAM_EV);
  if (safety_setup.gm_hw != GM_SDGM) {
    if (safety_setup.enable_gas_interceptor) {
      SET_RX_CHECKS(gm_pedal_rx_checks, ret);
    } else if (!safety_setup.gm_has_acc && safety_setup.gm_ev) {
      SET_RX_CHECKS(gm_no_acc_ev_rx_checks, ret);
    } else if (!safety_setup.gm_has_acc && !safety_setup.gm_ev) {
      SET_RX_CHECKS(gm_no_acc_rx_checks, ret);
    } else if (safety_setup.gm_ev) {
      SET_RX_CHECKS(gm_ev_rx_checks, ret);
    } else if (gm_ascm_int) {
      SET_RX_CHECKS(gm_ascm_int_rx_checks, ret);
    } else {}
  }

  // ASCM does not forward any messages
  if (safety_setup.gm_hw == GM_ASCM) {
    ret.disable_forwarding = true;
  }
  return ret;
}

const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
};
