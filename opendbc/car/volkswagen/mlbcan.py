from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# 柔和加速（原厂ACC风格）：k_accel 上限 0.55、scale 封顶 1.8、扭矩上升斜坡 8Nm/帧(≈400Nm/s)
#
# 2026-08-03 原厂 ACC_05 实测校准（route 00000004--915ebf086f，59段全扫描）：
#   - ACC_limitierte_Anfahrdyn / ACC_Loeseanforderung：原厂全程=0（此前的假设性补发已移除）
#   - 力矩基线按原厂拟合：0km/h 起步=27Nm、20km/h=48-52Nm、100km/h+=180-198Nm
#     （旧公式 2.5*v+141 低速段偏高约3倍，是起步发冲的根本原因）
#   - 减速 ACC_Verz_anf 斜坡渐进：原厂每帧约-0.025（50Hz≈1.25m/s²/s），最深-2.215
_ACC_MOMENT_RAMP = 8.0
_ACC_SCALE_MAX = 1.8
_last_acc_moment = 0.0
_last_accel_cmd = 0.0  # 减速请求斜坡状态（原厂 verz 渐进式加深）

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
  if acc_faulted:
    acc_control = 6
  elif long_active:
    # 激活中踩油门 → OVERRIDE(4)（原厂行为，00000004--seg7 实锤：激活中踩油门 st 3→4，
    # 力矩照发巡航值；松油门自动回 3）。若激活中仍发 3(active)，ECU 检测到
    # 「ACC激活+油门踏板」矛盾会写 DTC 锁死（00000015--seg23 踩油门0.8s accFaulted）。
    acc_control = 4 if gas_pressed else 3
  elif main_switch_on:
    # 待机：踩不踩油门都保持 2（原厂行为，00000004--seg1 实锤：待机踩油门 st 全程=2）。
    # 注意：不能在这里发 4——ECU 会把「2→4 未经过3」视为异常状态跳变。
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, gas_pressed=False):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active, gas_pressed)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, engine_torque=0):
  commands = []

  # ACC_05: multiplicative torque control
  #
  # Torque baseline calibrated to STOCK ACC observations (route 00000004--915ebf086f):
  #   standstill: ~27 Nm | 20 km/h: 48-52 Nm | 100 km/h+: 180-198 Nm
  #   fit: full_cruise = 6.3 * v_ego + 15 (v_ego in m/s)
  #   low-speed ramp: 27 Nm at standstill -> full baseline at ~20 km/h (v_ego/5.56)
  #
  # Scaling strategy (unchanged):
  #   accel > 0:  scale up   (more torque, car accelerates)
  #   accel = 0:  scale = 1  (cruise torque, hold speed)
  #   accel < 0:  scale down (less torque, engine braking)
  #   accel = -0.4: scale = 0 (max engine braking, transition to hydraulic)
  # Asymmetric k: k_accel ramps from 0.30 (launch, stock-ish 1.5 m/s² -> ~40 Nm on 27 Nm
  # base) to 0.55 at ~25 kph; scale capped at 1.8. k_decel=2.5 reaches 0 torque at -0.40.
  # Rise limited by _ACC_MOMENT_RAMP (8 Nm/frame ≈ 400 Nm/s) for stock-like smooth launch.

  # OVERRIDE(4)：驾驶员踩油门时 ACC 保持 armed，力矩照发巡航值（原厂 st=4 力矩≈st=3），
  # 仅状态字从 3 切 4；松油门后状态回 3 自动恢复。
  if acc_enabled:
    braking = accel < -0.4 or stopping or (v_ego < 2.0 and accel <= 0)
  else:
    braking = False

  if acc_enabled and not braking:
    full_cruise = 6.3 * v_ego + 15.0
    low_speed_ramp = min(1.0, v_ego / 5.56)   # 0 -> 20 km/h linear to full baseline
    cruise_torque = 27.0 + (full_cruise - 27.0) * low_speed_ramp
    if accel >= 0:
      # 原厂实测拟合（00000015--994ca60130--23 radar）：v≈1.6m/s 加速 ax=1.22 → Mom=145Nm（巡航基线≈27Nm）
      # 旧乘性模型（k_accel=0.30、scale≤1.8）在低速段最多输出 27*1.8=48Nm，仅够维持 6km/h 怠速蠕行
      # → 用户症状"激活成功但车不加速"的直接根因（同窗口原厂请求 145Nm，差 5 倍）。
      # 修复：正加速度改用加性映射 Mom = 巡航基线 + accel*85 Nm/(m/s²)（原厂斜率≈97，留12%余量防过冲）；
      # 8Nm/帧上升斜坡（_ACC_MOMENT_RAMP）继续保证起步柔和。
      acc_moment = int(min(500, cruise_torque + accel * 85.0))
    else:
      scale = max(0.0, 1.0 + accel * 2.5)
      acc_moment = int(min(500, cruise_torque * scale))
    # 上升斜坡：激活/加速时扭矩渐进（原厂ACC柔和感）
    global _last_acc_moment
    if acc_moment > _last_acc_moment:
      acc_moment = min(acc_moment, int(_last_acc_moment + _ACC_MOMENT_RAMP))
    _last_acc_moment = float(acc_moment)
  else:
    acc_moment = 0
    _last_acc_moment = 0.0

  # 减速请求斜坡（原厂实测）：braking 时 ACC_Verz_anf 每帧加深≤0.025（50Hz≈1.25m/s²/s），
  # 最深-2.215；请求变浅/恢复立即响应。原厂最深-2.215，上限收紧到-2.2（贴近原厂）。
  if braking:
    target_verz = max(accel, -2.2)  # 原厂实测最深-2.215（route 00000004）
    global _last_accel_cmd
    if target_verz < _last_accel_cmd:
      verz = max(target_verz, _last_accel_cmd - 0.025)
    else:
      verz = target_verz
    _last_accel_cmd = verz
  else:
    verz = 0 if acc_enabled else 3.01
    _last_accel_cmd = 0.0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Verz_anf": verz,
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0.0,  # 原厂实测：激活巡航时=0（00000005--4 route 130帧 status=3），保持原厂一致
    # 原厂59段全扫描（00000004--915ebf086f）：ACC_limitierte_Anfahrdyn 全程=0，
    # 柔和起步靠力矩渐进（_ACC_MOMENT_RAMP）+低基线（27Nm），无需限动力信号
    "ACC_limitierte_Anfahrdyn": 0,
    # 原厂实测全程=0；刹放平顺由力矩斜坡与 verz 管理
    "ACC_Loeseanforderung": 0,
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
    # KD_Fehler (63|1 = byte7 bit7): 原厂实测（route 00000004 全59段）恒 1 = 正常。
    # DBC 命名误导——它是 ACC 健康位而非故障位。OP 此前漏设 → packer 恒发 0，
    # ECU 在激活+驾驶员介入(st=4)时判定"ACC 自报故障却仍在请求" → 锁死 ACC/PAS
    # （route 0000002e seg5 463.3s accFaulted 实锤）。与原厂逐字节对齐：恒 1。
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


def create_acc_04_control(packer, bus, lead_speed_kph=327.36, acc_control=2):
  # OP 代发 ACC_04（原厂雷达状态文本，~16Hz）。屏蔽 bus2->bus0 转发后，
  # 由 OP 在 bus0 保持 ACC_04 周期在线，避免网关/仪表对 ACC_04 超时监测报 ACC 故障。
  # 内容对齐原厂规则（route 00000004--915ebf086f 实测，原厂ACC纵向+OP横向）：
  #   - ACC_Texte_Zusatzanz 随 ACC_Status_ACC 变化：off=1 / 待机=2 / 激活=8 / 超驰=3
  #   - Charisma 恒定 (FahrPr=2, Status=1, Umschaltung=0)
  #   - ACC_Geschw_Zielfahrzeug：无目标=满量程 327.36（0x3FF），有目标=真实前车速度
  texte_zusatz = {0: 1, 2: 2, 3: 8, 4: 3}.get(acc_control, 0)
  values = {
    "ACC_Texte_Zusatzanz": texte_zusatz,
    "ACC_Status_Zusatzanz": 0,
    "ACC_Texte": 0,
    "ACC_Texte_braking_guard": 0,
    "ACC_Warnhinweis": 0,
    "ACC_Geschw_Zielfahrzeug": lead_speed_kph,
    "ACC_Charisma_FahrPr": 2,
    "ACC_Charisma_Status": 1,
    "ACC_Charisma_Umschaltung": 0,
  }
  return packer.make_can_msg("ACC_04", bus, values)

def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x10E: 0x0F, # TSK_04
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
