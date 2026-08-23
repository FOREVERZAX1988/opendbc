from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# 柔和加速（原厂ACC风格）：k_accel 上限 0.55、scale 封顶 1.8、扭矩上升斜坡 8Nm/帧(≈400Nm/s)
#
# 2026-08-03 原厂 ACC_05 实测校准（route 00000004--915ebf086f，59段全扫描）：
#   - ACC_limitierte_Anfahrdyn / ACC_Loeseanforderung：00000004 扫描为0，但 00000049（官方master）
#     实测踩油门起步时 Loeseanforderung=1（约0.7s）——起步确认信号，已按官方行为恢复代发
#   - 力矩基线按原厂拟合：0km/h 起步=27Nm、20km/h=48-52Nm、100km/h+=180-198Nm
#     （旧公式 2.5*v+141 低速段偏高约3倍，是起步发冲的根本原因）
#   - 减速 ACC_Verz_anf 斜坡渐进：缓刹约-0.025/帧（00000004），紧急加深实测 0.06-0.07/帧
#     （0000004c seg22/23/25）——OP 取 0.07/帧（目标值仍由 accel 决定，缓刹不受影响）
_ACC_MOMENT_RAMP = 8.0
_ACC_MOMENT_RAMP_DOWN = 3.0  # 撤力跟随斜坡（50Hz≈150Nm/s）：00000042 seg7 原厂 mom 96→0 约0.68s(≈2.8/帧)，
                             # OP 用 3/帧 斜坡下降避免 110→0 瞬间跳变与原厂残余力矩方向相反 → st=6
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


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, engine_torque=0, stock_esp=False, stock_follow=False, gas_override=False, stock_fv=False, stock_mom=0.0, slope_pct=0.0, slope_comp=False, slope_comp_unlimited=False, sng_resume_req=False):
  global _last_acc_moment
  commands = []

  # Macan 坡度补偿（2026-08-19 重新实施）：slope_pct 由 carcontroller 经参数传入（非 CS 变量）。
  # accel_eff = accel + g*sin(atan(slope_pct/100))——上坡(正)增加力矩，下坡(负)触发/加深 verz（刹一脚）。
  # 受 MacanSlopeComp 开关控制；原厂限制：slope_comp_unlimited=False=min(stock_mom)（选项1），
  # True=min(max(stock_mom,200))（选项2 放开小坡空间）。
  if slope_comp:
    import math
    accel_eff = accel + 9.81 * math.sin(math.atan(slope_pct / 100.0))
  else:
    accel_eff = accel


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
    # 00000039 seg5 实锤：原厂 verz=-0.40 被旧阈值 -0.4（严格小于）吞掉 → OP 发 verz=0
    # → 雷达自检"减速请求未执行" → st6。放宽到 -0.05：任何轻微减速都走 braking 发 verz。
    # 00000042 seg3/seg6 实锤：油门超驰（gas_override）时不得因 v_ego<2 走 braking——
    # 原厂跟停中踩油门会切 st=4 并发力矩（mom 70->140/FM=1/FV=0），若 OP 因低速条件
    # 走 braking（FV=1/verz=0）则与原厂方向矛盾 → TSK_04 2->0 退出 → 松油门不加速。
    braking = (accel_eff < -0.05 and not gas_override) or stopping or (not gas_override and v_ego < 2.0 and accel_eff <= 0)
  else:
    braking = False

  # 撤力跟随优先（00000041 修复）：原厂撤力/减速（stock_follow=True）时，OP 代发帧必须完全
  # 镜像原厂——力矩归零、力矩通道关闭（FM=0）。即使 accel=0（非 braking 分支），
  # 也绝不允许发出巡航基线力矩（6.3*v+15），否则与原厂 mom=0 方向矛盾 → 雷达 st6 →
  # TSK_04 1→0 退出 → controlsMismatch。verz/FV 由 braking 逻辑自然跟随：
  #   纯撤力（原厂 FV=0, verz≈0）→ accel=0 → 非 braking → verz=0, FV=0, FM=0, mom=0（滑行）
  #   跟减速（原厂 FV=1, verz<0）→ accel=stock_verz → braking → verz=stock_verz, FV=1, FM=0
  # 00000042 seg7 修复：mom 斜坡下降（3/帧≈150Nm/s）跟随原厂缓撤力曲线（96→0 约0.68s≈2.8/帧），
  # 而非瞬间 110→0 跳变——否则与原厂 FM=1 残余力矩方向相反 → 雷达 st6 → TSK_04 退出。
  # 独立于正常力矩计算（不经过 +8 上升斜坡再 -3 的净+5 问题），纯斜坡下降；
  # 撤力结束（stock_follow=False）后从当前 _last 平滑恢复发力矩。
  if stock_follow:
    acc_moment = max(0, int(_last_acc_moment - _ACC_MOMENT_RAMP_DOWN))
    _last_acc_moment = float(acc_moment)
  elif acc_enabled and not braking:
    full_cruise = 6.3 * v_ego + 15.0
    low_speed_ramp = min(1.0, v_ego / 5.56)   # 0 -> 20 km/h linear to full baseline
    cruise_torque = 27.0 + (full_cruise - 27.0) * low_speed_ramp
    if accel_eff >= 0:
      # 原厂实测拟合（00000015--994ca60130--23 radar）：v≈1.6m/s 加速 ax=1.22 → Mom=145Nm（巡航基线≈27Nm）
      # 旧乘性模型（k_accel=0.30、scale≤1.8）在低速段最多输出 27*1.8=48Nm，仅够维持 6km/h 怠速蠕行
      # → 用户症状"激活成功但车不加速"的直接根因（同窗口原厂请求 145Nm，差 5 倍）。
      # 修复：正加速度改用加性映射 Mom = 巡航基线 + accel*85 Nm/(m/s²)（原厂斜率≈97，留12%余量防过冲）；
      # 8Nm/帧上升斜坡（_ACC_MOMENT_RAMP）继续保证起步柔和。
      acc_moment = int(min(500, cruise_torque + accel_eff * 85.0))
    else:
      scale = max(0.0, 1.0 + accel_eff * 2.5)
      acc_moment = int(min(500, cruise_torque * scale))
    # 上升斜坡：激活/加速时扭矩渐进（原厂ACC柔和感）
    if acc_moment > _last_acc_moment:
      acc_moment = min(acc_moment, int(_last_acc_moment + _ACC_MOMENT_RAMP))
    # 方案A：OP力矩≤原厂雷达力矩（2026-08-17 cut-in实证）
    # 原厂收油预备（Mom 88→81 持续下降）时 OP 必须跟随，不能反向加速（Mom 100-164）——
    # 水平限制强制 OP ≤ 原厂；v_ego≤11km/h 豁免起步（SnG 正常，原厂 mom=0/爬升期不被锁死）；
    # 正常加速 OP 本来就≤原厂（实测 72.9 vs 87.9），限制不生效；上升斜坡保证平滑爬升。
    if stock_mom > 0 and v_ego > 3.0:
      if slope_comp and slope_comp_unlimited:
        acc_moment = min(acc_moment, int(max(stock_mom, 200.0)))   # 选项2：放开给小坡空间
      else:
        acc_moment = min(acc_moment, int(stock_mom))                # 选项1：原厂限制
    _last_acc_moment = float(acc_moment)
  else:
    acc_moment = 0
    _last_acc_moment = 0.0

  # 减速请求斜坡（原厂实测）：braking 时 ACC_Verz_anf 每帧加深≤0.025（50Hz≈1.25m/s²/s），
  # 最深-2.215；请求变浅/恢复立即响应。原厂最深-2.215，上限收紧到-2.2（贴近原厂）。
  if braking:
    # 停车保持（stopping）时镜像原厂保持力度 -2.0（0000004c seg1-5 坡道后溜实锤：
    # OP 保持 verz=-0.55 vs 原厂 -2.0，坡上保持力不足→后溜）。刹停过程仍用 accel 目标。
    target_verz = -2.0 if stopping else max(accel_eff, -2.2)  # 原厂实测最深-2.215
    global _last_accel_cmd
    if target_verz < _last_accel_cmd:
      verz = max(target_verz, _last_accel_cmd - 0.07)  # 原厂实测 0.06-0.07/帧（0000004c seg22/23/25 紧急加深）
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
    "ACC_Freigabe_Verzanf": 1 if (braking or (acc_enabled and stock_fv and not gas_override)) else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking and not stock_follow) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0.0,  # 原厂实测：激活巡航时=0（00000005--4 route 130帧 status=3），保持原厂一致
    # 原厂59段全扫描（00000004--915ebf086f）：ACC_limitierte_Anfahrdyn 全程=0，
    # 柔和起步靠力矩渐进（_ACC_MOMENT_RAMP）+低基线（27Nm），无需限动力信号
    "ACC_limitierte_Anfahrdyn": 0,
    # ACC_Loeseanforderung（解除请求）= 起步确认信号。
    # 00000049（官方 master）实测：踩油门起步时 OP+原厂均置 1（约0.7s，车动后回 0）——
    # 即"驾驶员起步确认"。SnG 自动起步时由 sng_resume_req 模拟该语义（不踩油门）。
    # 2026-08-23 修复（00000054 seg18 st=6 实锤）：loes=1 不能与停车保持帧（verz=-2/anh=1）
    # 同时出现——"请求解除但保持刹车"自相矛盾 → 原厂自检失败 st=6。仅在 OP 激活
    # （acc_enabled）且非停车保持（not stopping）时发；OP 待机时绝不发（st=7 也涉及）。
    "ACC_Loeseanforderung": 1 if (acc_enabled and not stopping and (gas_override or sng_resume_req)) else 0,
    # ACC_ax_Getriebe: tells PDK what acceleration to expect (gear selection hint).
    # DBC: [-2.016, +10.248]. Values below -2.016 WRAP to ~+10 (unsigned overflow).
    # Accel > 0.25: hint positive, capped at 1.3 (prevents high-RPM downshifts)
    # Cruise/mild decel: 0 (no gear hunting)
    # Engine braking (accel < -0.25): mild negative hint, capped at -0.5
    # Hydraulic braking: speed-dependent negative, clamped to DBC min -2.016
    # 00000039 seg7 实锤：原厂跟停保持（vEgo=0, anh=1）axG=+1.63（1挡怠速拖滞蠕行），
    # 旧逻辑 braking 时发 max(accel, -0.6...)≈-0.60 负值 → 与原厂方向矛盾 → 雷达 st6。
    # 修复：stopping（跟停/停车保持）时 axG 按原厂 +1.63 正蠕行；普通 braking 保持负值。
    "ACC_ax_Getriebe": (1.63 if stopping else
                         ((min(accel, 1.3) if accel > 0.25 else
                           (max(accel, -0.5) if accel < -0.25 else 0)) if not braking else
                          max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6)))) if acc_enabled else 0,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    # 00000039 seg7 实锤：原厂跟停全程 ESP=0（ESP_VerzTSK=0，靠1挡怠速拖滞），
    # 旧逻辑 stopping 时发 ESP=1 → 雷达自检异常 → st6。仅 esp_hold 或硬刹车(<-1.0) 才请求 ESP。
    # 0000003f 实锤：原厂跟停全程 ESP=0（怠速拖滞），旧条件 braking and accel<-1.0
    # 在仲裁把 accel 压到 stock_verz(-2) 时误触发 → OP ESP=1 与原厂矛盾。
    # 仲裁已保证 accel 最负 -1.0（原厂不减速时），accel<-1 只可能来自跟随原厂 verz，
    # 此时 ESP 应以原厂为准（透传 stock_esp）；esp_hold（原厂 ESP hold 确认）保留。
    "ACC_Beeinflussung_ESP": 1 if (esp_hold or stock_esp) else 0,
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


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4, stock_prim_anz=0, stock_status_anzeige=None, stock_texte_prim=0, stock_display_prio=None):
  # Stock radar's lead_object is accurate when working, but gets suppressed to 0 during
  # irreversible fault (status 7) even though ACC_Abstandsindex still tracks distance.
  # Fallback: if lead_object=0 but valid distance exists, the radar is faulted -- use distance.
  lead_obj = lead_object if lead_object else (1 if 0 < lead_distance < 1000 else 0)
  values = {
    # ACC_Status_Anzeige：2026-08-22 改透传原厂（原厂激活=3/故障=6；旧逻辑 acc_hud_status
    # 重算，踩油门=4≠原厂3 → 状态矛盾 → st=6，00000053 seg6/7 实锤）。None 时回退旧逻辑。
    "ACC_Status_Anzeige": acc_hud_status if stock_status_anzeige is None else stock_status_anzeige,
    # ACC_Status_Prim_Anz：原厂（00000004 实测）anz=3 激活时 primAnz=1（跟车 HUD 主状态），
    # anz=2 待机/anz=4 超驰时 primAnz=0。2026-08-22 改透传原厂值（carstate.stock_prim_anz）：
    # 旧逻辑 1 if acc_hud_status==3 else 0 在踩油门时 acc_hud_status=4(超驰)→发0，而原厂
    # st=3(激活)发1 → bus2/bus128 状态矛盾 → 原厂检出自检失败 st=6（00000053 seg6/7 实锤）。
    "ACC_Status_Prim_Anz": stock_prim_anz,
    # ACC_Display_Prio：原厂（00000004 全段 44|2 实测）只出现 2/3（36001 帧无 0/1）：
    #   prio=3 常态（93.5%，有/无目标均多）、prio=2 偶发（6.5%，集中待机有目标窗口）。
    # 保持上游 opendbc 标准行为 2/3（有目标→2，无目标→3），勿改 0/1。
    # ACC_Display_Prio：2026-08-22 改透传原厂（原厂按 ab 判定 2/3；OP 按视觉 lead_obj 会相反）。
    # None 时回退 OP 逻辑（其他平台不传此参数不受影响）。
    "ACC_Display_Prio": (2 if lead_obj else 3) if stock_display_prio is None else stock_display_prio,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": zeitluecke,  # Mirror stock radar's ZL from ext bus (responds to DIST button)
    "ACC_Abstandsindex": lead_distance,
    "ACC_Relevantes_Objekt": lead_obj,
    # ACC_Texte_Primaeranz：2026-08-22 改透传原厂（原厂故障时=1 显示故障文本，OP 默认0 丢失→状态矛盾）
    "ACC_Texte_Primaeranz": stock_texte_prim,
  }

  return packer.make_can_msg("ACC_02", bus, values)


def create_acc_04_control(packer, bus, lead_speed_kph=327.36, acc_control=2, stock_texte_zusatz=None, stock_charisma_status=None):
  # OP 代发 ACC_04（原厂雷达状态文本，~16Hz）。屏蔽 bus2->bus0 转发后，
  # 由 OP 在 bus0 保持 ACC_04 周期在线，避免网关/仪表对 ACC_04 超时监测报 ACC 故障。
  # 内容对齐原厂规则（route 00000004--915ebf086f 实测，原厂ACC纵向+OP横向）：
  #   - ACC_Texte_Zusatzanz 随 ACC_Status_ACC 变化：off=1 / 待机=2 / 激活=8 / 超驰=3
  #   - Charisma 恒定 (FahrPr=2, Status=1, Umschaltung=0)
  #   - ACC_Geschw_Zielfahrzeug：无目标=满量程 327.36（0x3FF），有目标=真实前车速度
  # ACC_Texte_Zusatzanz / ACC_Charisma_Status：2026-08-22 改透传原厂（00000053 seg6 实锤：
  # 踩油门 acc_control=4→texte=3 而原厂 st=3→texte=8；原厂故障 texte=0/Charisma=2 而 OP
  # 正常模板=2/1 → 显示矛盾）。None 时回退旧映射（其他平台不传不受影响）。
  texte_zusatz = {0: 1, 2: 2, 3: 8, 4: 3}.get(acc_control, 0) if stock_texte_zusatz is None else stock_texte_zusatz
  values = {
    "ACC_Texte_Zusatzanz": texte_zusatz,
    "ACC_Status_Zusatzanz": 0,
    "ACC_Texte": 0,
    "ACC_Texte_braking_guard": 0,
    "ACC_Warnhinweis": 0,
    "ACC_Geschw_Zielfahrzeug": lead_speed_kph,
    "ACC_Charisma_FahrPr": 2,
    "ACC_Charisma_Status": 1 if stock_charisma_status is None else stock_charisma_status,
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
