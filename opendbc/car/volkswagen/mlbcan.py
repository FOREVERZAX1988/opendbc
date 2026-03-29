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


def acc_control_value(main_switch_on, acc_faulted, long_active, gas_pressed=False):
  # 最高优先级：故障 → 固定6
  if acc_faulted:
    acc_control = 6
  # 第二优先级：ACC已激活 → 才会判断油门，输出3/4
  elif long_active:
    acc_control = 4 if gas_pressed else 3
  # 第三优先级：主开关打开但未激活 → 固定2，不管踩不踩油门
  elif main_switch_on:
    acc_control = 2
  # 最低优先级：主开关完全关闭 → 固定0，不管任何操作
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, gas_pressed=False):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active, gas_pressed)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, gear_ratio=0, gas_pressed=False, resume=False, stock_acc05_values=None):
  commands = []

  # ACC_05: additive torque control with physics-based gain
  #
  # acc_moment = cruise_torque(v) + accel * torque_gain(gear_ratio)
  #
  # cruise_torque: quadratic fit to real steady-state engine torque (MO_Mom_Ist)
  #   from drive data. Matches the torque needed to maintain speed on flat ground.
  #   Values: 46 Nm at standstill, ~68 at 30 kph, ~100 at 60 kph, ~189 at 120 kph.
  #   Replaces the old linear (2.5*v+141) which was 40-100 Nm too high at mid-speeds.
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

  # 精简你写好的正确逻辑，不重复计算
  freigabe_moment = 1 if (acc_control in (3, 4) and not (acc_control == 3 and accel < 0)) else 0
  freigabe_verzanf = 1 if (acc_control == 3 and freigabe_moment == 0) else 0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    #3. ACC_Freigabe_Momentenanforderung：ACC加速请求使能, 1=允许ACC加速请求（油门控制），0=禁止ACC加速请求（仅制动控制）。根据实际情况设置：当ACC需要加速时启用（如巡航或加速状态且不制动），否则禁用以确保安全。
    "ACC_Freigabe_Momentenanf": freigabe_moment,
    #4. ACC_Freigabe_Verzanf：ACC减速请求使能,
    "ACC_Freigabe_Verzanf": freigabe_verzanf,
    #5.P档请求状态,查看routes基本都是0. Not used by ACC logic, but required for checksum. Always 0 in stock ACC.
    "ACC_Getriebestellung_P": 0,
    #6.起步动态限制:❌ 扭矩无限制:0✅️限制扭矩:1（1为猜测值，目前实际行车恒定为0）
    "ACC_limitierte_Anfahrdyn": 0,
    #7. ACC_Momentenanforderung：ACC加速请求，范围:代码定义[0~500]；实际观察：[0~199]（对应0~3.5 m/s²左右加速），ACC关闭时保持0
    "ACC_Momentenanforderung": acc_moment,
    #8.允许调节偏差，查看routes基本都是0，没见过其他值.
    "ACC_zul_Regelabw": 0,
    #9. ACC_Verz_anf减速度请求，范围:代码定义[-3.5[兜底]~ 3.01[关闭] ]；实际观察：[-2.00,1.72]；(关闭状态基本都是保持0，自动刹停保持-2，acc激活+驾驶员踩松踩油门才短暂为+)
    # Stock ACC_Verz_anf range during braking: -2.015 to 0
    # Panda safety allows -3.5; DBC allows -7.22. Use panda limit for max braking.
    "ACC_Verz_anf": -2.0 if (acc_control == 3 and v_ego < 0.1) else (max(accel, -2.0) if (acc_control == 3 and braking) else 0.0),
    #10. ACC_Loeseanforderung：制动保持解除请求（完全复刻原厂）
    "ACC_Loeseanforderung": 1 if (acc_enabled and v_ego < 0.1 and (gas_pressed or resume)) else 0,
    #11. ACC_StartStopp_Info：ACC启动/停止信息.跟随acc_enabled变化，ACC关闭时保持0，开启时保持1（acc_enabled跟openpilot纵向激活状态同步）
    "ACC_StartStopp_Info": acc_enabled,
    #12.制动系统预充油状态：0=无预充，1=预充，2=预充完成（从数据观察到的状态，实际行车中基本保持在0）
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    #13. ACC_ax_Getriebe: 变速箱加速请求（与加速请求配合使用，提供换挡提示以优化加速响应）
    # ACC_ax_Getriebe: tells PDK what acceleration to expect (gear selection hint).
    # DBC: [-2.016, +10.248]. Values below -2.016 WRAP to ~+10 (unsigned overflow).
    # Accel > 0.25: hint positive, capped at 1.3 (prevents high-RPM downshifts)
    # Cruise/mild decel: 0 (no gear hunting)
    # Engine braking (accel < -0.25): mild negative hint, capped at -0.5
    # Hydraulic braking: speed-dependent negative, clamped to DBC min -2.016
    #"ACC_ax_Getriebe": ((min(accel, 1.3) if accel > 0.25 else
    #                      (max(accel, -0.5) if accel < -0.25 else 0)) if not braking else
    #                     max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_ax_Getriebe": stock_acc05_values.get("ACC_ax_Getriebe", 0) if (acc_enabled and stock_acc05_values and acc_control != 6) else ((min(accel, 1.3) if accel > 0.25 else (max(accel, -0.5) if accel < -0.25 else 0)) if not braking else max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    #14. ACC_Status_ACC: 0=off, 1=standby, 2=ready, 3=active, 4=override (gas), 5=override (brake), 6=fault, 7=initializing
    "ACC_Status_ACC": acc_control,
    #15. ACC_Betaetigung_EPB: ESP保持状态，0=不保持，1=保持。根据实际情况设置：当ACC需要制动但不直接控制制动力时（如紧急制动或低速停止），设置为1以请求ESP保持车辆静止。
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
    #16. ACC_Beeinflussung_ESP: ESP干预请求，1=请求ESP介入（如制动保持、紧急制动辅助等），0=不请求ESP介入。根据实际情况设置：当ACC需要制动但不直接控制制动力时（如紧急制动或低速停止），请求ESP介入以确保安全停车。
    "ACC_Beeinflussung_ESP": 1 if (stopping or esp_hold or (braking and accel < -1.0)) else 0,  # ESP for stopping, hold, or hard braking (>1 m/s²)
    #17. ACC_Anhalten: 停车状态，1=停车，0=未停车
    "ACC_Anhalten": stopping,
    #18. ACC_KD_Fehler: 巡航控制错误？但是观察原厂acc使用信号，无论正常还是ACC故障状态，基本都是保持1，没见过其他值。为了安全起见，复刻原厂行为，始终保持1。
    "ACC_KD_Fehler": 1,
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_control, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4):
  # Stock radar's lead_object is accurate when working, but gets suppressed to 0 during
  # irreversible fault (status 7) even though ACC_Abstandsindex still tracks distance.
  # Fallback: if lead_object=0 but valid distance exists, the radar is faulted -- use distance.
  lead_obj = lead_object if lead_object else (1 if 0 < lead_distance < 1000 else 0)
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Status_Prim_Anz": 1 if (acc_control == 3) else 0, # 4 仅当纯acc控制时才为1，任何介入或关闭acc功能均为0.
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
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
  # 2. 强制 ACC_Charisma_Status 为 1（正常），避免雷达故障状态
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
