#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"


static safety_config volkswagen_mlb_init(uint16_t param) {
  // Transmit of LS_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {{MSG_HCA_01, 0, 8, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                        {MSG_LS_01, 0, 4, .check_relay = false}, {MSG_LS_01, 2, 4, .check_relay = false}};

  // OP 纵向模式：代发 ACC_02/ACC_05/ACC_04（Macan 纵向适配，sp-longtest-0807 成果）
  static const CanMsg VOLKSWAGEN_MLB_LONG_TX_MSGS[] = {{MSG_HCA_01, 0, 8, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                       {MSG_ACC_02, 0, 8, .check_relay = true}, {MSG_ACC_05, 0, 8, .check_relay = true},
                                                       {MSG_ACC_04, 0, 8, .check_relay = true},
                                                       // LS_01 按键帧：起步跟停（MacanStartStop）代发 RESUME + 物理按键转发（relay 断开后 bus0->bus2 不转发）
                                                       {MSG_LS_01, 0, 4, .check_relay = false}, {MSG_LS_01, 2, 4, .check_relay = false}};

  static RxCheck volkswagen_mlb_rx_checks[] = {
    // TODO: implement checksum validation
    {.msg = {{MSG_ESP_03, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_TSK_04, 1, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LS_01, 0, 4, 10U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif
  volkswagen_common_init();

  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
}

static void volkswagen_mlb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Check all wheel speeds for any movement
    // Signals: ESP_03.ESP_[VL|VR|HL|HR]_Radgeschw
    if (msg->addr == MSG_ESP_03) {
      uint32_t speed = 0;
      speed += ((msg->data[3] & 0xFU) << 8) | msg->data[2];   // FL
      speed += (msg->data[4] << 4) | (msg->data[3] >> 4);     // FR
      speed += ((msg->data[6] & 0xFU) << 8) | msg->data[5];   // RL
      speed += (msg->data[7] << 4) | (msg->data[6] >> 4);     // RR
      vehicle_moving = speed > 0U;
    }

    // Update driver input torque
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    if (msg->addr == MSG_LS_01) {
      if (volkswagen_longitudinal) {
        // When using openpilot longitudinal, read ACC main switch from LS_01 (bus 0)
        // instead of relying on ACC_05 from the stock radar (bus 2)
        // Signal: LS_01.LS_Hauptschalter
        acc_main_on = GET_BIT(msg, 12U);
        if (!acc_main_on) {
          controls_allowed = false;
        }

        // Enter controls on falling edge of Set or Resume with main switch on
        // Signal: LS_01.LS_Tip_Setzen
        // Signal: LS_01.LS_Tip_Wiederaufnahme
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      // Always exit controls on rising edge of Cancel
      // Signal: LS_01.LS_Abbrechen
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
      acc_main_on = GET_BIT(msg, 12U);  // LS_Hauptschalter
    }

    // Signal: Motor_03.MO_Fahrpedalrohwert_01
    // Signal: Motor_03.MO_Fahrer_bremst
    if (msg->addr == MSG_MOTOR_03) {
      gas_pressed = msg->data[6] != 0U;
      volkswagen_brake_pedal_switch = GET_BIT(msg, 35U);
    }

    if (msg->addr == MSG_ESP_05) {
      volkswagen_brake_pressure_detected = GET_BIT(msg, 26U);
    }

    brake_pressed = volkswagen_brake_pedal_switch || volkswagen_brake_pressure_detected;

  }

  if (msg->bus == 1U) {
    if (msg->addr == MSG_TSK_04) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Signal: TSK_04.TSK_Status_GRA_ACC_02
      int acc_status = (msg->data[7] & 0xC0U) >> 6;
      bool cruise_engaged = (acc_status == 1) || (acc_status == 2);

      // 2026-08-16 Macan(MLB) 适配：LONG 模式（OP 纵向）跳过 pcm_cruise_check——
      // controls_allowed 由 LS_01 按键（SET/Resume 下降沿+主开关）管理。
      // 无条件执行会在原厂巡航未激活（停车等红灯 TSK_04∉1/2）时撤 controls_allowed，
      // 而 OP 已 enabled → selfdrived mismatch_counter 200 帧 → controlsMismatch 报警。
      // 与 volkswagen_mqb.h / volkswagen_meb.h 处理一致。
      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }
    }
  }
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
    .max_torque = 300,             // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 169,           // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 112.5 ; 112.5 * 1.5 for safety pad = 168.75
    .max_rate_up = 9,              // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,           // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_allowance = 60,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for HCA_01 Heading Control Assist torque
  if (msg->addr == MSG_HCA_01) {
    int desired_torque = volkswagen_mlb_mqb_steering_control_torque(msg);

    int steer_status = msg->data[4] & 0xFU;
    bool steer_req = (steer_status == 5) || (steer_status == 7);

    if (steer_torque_cmd_checks(desired_torque, steer_req, VOLKSWAGEN_MLB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_LS_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if (GET_BIT(msg, 16U) || GET_BIT(msg, 19U)) {
      tx = false;
    }
  }

  return tx;
}

// TODO: rename these functions to MXB or something
const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
