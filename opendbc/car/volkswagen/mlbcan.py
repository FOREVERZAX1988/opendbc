from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  return mqb_create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, set_increase=False, set_decrease=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
    "LS_Tip_Setzen": set_increase,    # SET: forward push on stalk (with LS_Tip_Hoch for speed increase)
    "LS_Tip_Hoch": set_increase,      # UP tip: speed increase (+1 mph per short press)
    "LS_Tip_Runter": set_decrease,    # DOWN tip: speed decrease (-1 mph per short press)
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, tsk_acc_status):
  # 1. 最高优先级：故障状态
  if acc_faulted or tsk_acc_status == 3:
    acc_control = 6
  # 2. 完全听 TSK_04 的：TSK说激活就激活，TSK说介入就介入
  elif tsk_acc_status == 1:
    acc_control = 3
  elif tsk_acc_status == 2:
    acc_control = 4
  # 3. 待机状态：TSK=0 且 总开关打开
  elif main_switch_on:
    acc_control = 2
  # 4. 完全关闭：总开关关闭
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, tsk_acc_status):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active, tsk_acc_status)

# 新增CS入参，用于读取真实车速
def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, gear_ratio=0, CS=None):
  commands = []

  # ACC_05: additive torque control with physics-based gain
  #
  # acc_moment = cruise_torque(v) + accel * torque_gain(gear_ratio)
  #
  # cruise_torque: quadratic fit to real steady-state engine torque (MO_Mom_Ist)
  #   from drive data. Matches the torque needed to maintain speed on flat ground.
  #   Values: 46 Nm at standstill, ~68 at 30 kph, ~100 at 60 kph, ~189 at 120 kph.
  #   Replaces the old linear (2.5*v+141) which was 40-100 Nm too high at mid-speeds.
  #
  # torque_gain: converts planner acceleration (m/s²) to engine torque (Nm) using
  #   real-time effective gear ratio from Getriebe_03 GE_Uefkt:
  #     torque_gain = mass * wheel_radius / gear_ratio
  #   In low gears (ratio ~17), small engine torque → big wheel force, so gain is low (~43 Nm/m/s²).
  #   In high gears (ratio ~2.4), gain is high (~306 Nm/m/s²). This makes the planner's accel
  #   request map ~1:1 to actual acceleration regardless of gear.
  #
  # Max engine-braking decel (acc_moment=0) is cruise_torque/torque_gain:
  #   ~0.4 m/s² at mid-speeds, ~0.6 m/s² at highway. Beyond that, hydraulic brakes take over.
  MASS = 2000.0   # 2023 Macan S ~1955 kg curb + driver
  WHEEL_R = 0.36  # 255/55R18 effective rolling radius
  # 新增在开头先定义车速兼容逻辑，后面统一用 current_v_ego，确保在CS缺失或vEgo不可用时不会出问题
  current_v_ego = CS.out.vEgo if (CS is not None and hasattr(CS, 'out') and hasattr(CS.out, 'vEgo')) else v_ego

  if acc_enabled:
    braking = accel < -0.4 or stopping or (v_ego < 2.0 and accel <= 0)
  else:
    braking = False

  if acc_enabled and not braking:
    cruise_torque = 0.064 * v_ego * v_ego + 2.16 * v_ego + 46.0

    if gear_ratio > 1.0:
      torque_gain = MASS * WHEEL_R / gear_ratio
    else:
      # No valid gear ratio (standstill/neutral or startup). Use a conservative
      # gain matching ~1st gear (ratio ~17) so launches are gentle. As soon as
      # the PDK engages a gear, real gear_ratio takes over.
      torque_gain = MASS * WHEEL_R / 17.0

    acc_moment = int(max(0, min(500, cruise_torque + accel * torque_gain)))
  else:
    acc_moment = 0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    # Stock ACC_Verz_anf range during braking: -2.015 to 0
    # Panda safety allows -3.5; DBC allows -7.22. Use panda limit for max braking.
    "ACC_Verz_anf": -2.0 if (acc_enabled and ((current_v_ego < 0.1) or esp_hold)) else (max(accel, -2.0) if braking else (0 if acc_enabled else 0.0)),
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0,
    # ACC_ax_Getriebe: tells PDK what acceleration to expect (gear selection hint).
    # DBC: [-2.016, +10.248]. Values below -2.016 WRAP to ~+10 (unsigned overflow).
    # Accel > 0.25: hint positive, capped at 1.3 (prevents high-RPM downshifts)
    # Cruise/mild decel: 0 (no gear hunting)
    # Engine braking (accel < -0.25): mild negative hint, capped at -0.5
    # Hydraulic braking: speed-dependent negative, clamped to DBC min -2.016
    "ACC_ax_Getriebe": ((min(accel, 1.3) if accel > 0.25 else
                          (max(accel, -0.5) if accel < -0.25 else 0)) if not braking else
                         max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    "ACC_Beeinflussung_ESP": 1 if (stopping or esp_hold or (braking and accel < -1.0)) else 0,  # ESP for stopping, hold, or hard braking (>1 m/s²)
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Anhalten": stopping,
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
    # 【新增 核心修正】匹配原厂逻辑，ACC激活时置1
    "ACC_KD_Fehler": 1,
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4):
  # Stock radar's lead_object is accurate when working, but gets suppressed to 0 during
  # irreversible fault (status 7) even though ACC_Abstandsindex still tracks distance.
  # Fallback: if lead_object=0 but valid distance exists, the radar is faulted -- use distance.
  lead_obj = lead_object if lead_object else (1 if 0 < lead_distance < 1000 else 0)
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": zeitluecke,  # Mirror stock radar's ZL from ext bus (responds to DIST button)
    "ACC_Display_Prio": 2 if lead_obj else 3,
    "ACC_Abstandsindex": lead_distance,
    "ACC_Relevantes_Objekt": lead_obj,
  }

  return packer.make_can_msg("ACC_02", bus, values)

# 【新增】生成 ACC04 报文：复用原厂雷达信号，确保状态正常
def create_acc04_control(packer, bus, original_values):
  # 1. 复制原厂所有信号
  values = original_values.copy()
  if values["ACC_Charisma_Status"] == 2:
      values["ACC_Charisma_Status"] = 1
  return packer.make_can_msg("ACC_04", bus, values)

def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x111: 0x10, # TSK_05
    0x30C: 0x0F, # ACC_02
    0x324: 0x27, # ACC_04
    0x10B: 0xA,  # LS_01
    0x10D: 0x0C, # ACC_05
    0x10F: 0x0E, # ACC_0x10F
    0x311: 0x12, # ACC_0x311
    0x397: 0x94, # LDW_02
    0x10C: 0x0D, # TSK_02
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return volkswagen_mqb_meb_checksum(address, sig, d)
