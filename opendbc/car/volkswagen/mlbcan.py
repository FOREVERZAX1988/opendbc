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
_last_ax_ge = 0.0  # ACC_ax_Getriebe 缓爬状态（2026-08-23 拟合原厂，rate 0.005/帧）
# 超驰斜坡状态（2026-08-24 对齐原厂）：减速中驾驶员踩油门切超驰时，原厂发"加速声明"
# 斜坡（verz 爬正 +1.285/180ms 再归0，00000002 31次减速→超驰事件每次必发，2043 窗口
# 序列 0.025→0.205→0.385→...→1.285→0）。受 MacanSlopeComp 开关控制：关=直接归0（现状）。
_prev_braking = False       # 上一帧 braking 状态（减速→超驰上升沿检测）
_ovr_slope_active = False   # 超驰斜坡进行中
_ovr_slope_step = 0         # 剩余步数（9→1：0.025→1.285→归0，9帧=180ms）

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


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, set_increase=False, set_decrease=False,
                               distance_increase=False, distance_decrease=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  # 距离键（DIST +/-）：值1=拉近-1格、值2=拉远+1格（20|2）。开机档位同步
  # （MacanStartupGapSync）与物理按键转发共用此函数，0=无按键。
  zeitluecke_val = 2 if distance_increase else (1 if distance_decrease else 0)

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
    "LS_Tip_Setzen": set_increase,    # SET: forward push on stalk (with LS_Tip_Hoch for speed increase)
    "LS_Tip_Hoch": set_increase,      # UP tip: speed increase (+1 mph per short press)
    "LS_Tip_Runter": set_decrease,    # DOWN tip: speed decrease (-1 mph per short press)
    "LS_Verstellung_Zeitluecke": zeitluecke_val,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, gas_pressed=False, stock_st=None):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    # 激活中踩油门 → OVERRIDE(4)（原厂行为，00000004--seg7 实锤：激活中踩油门 st 3→4，
    # 力矩照发巡航值；松油门自动回 3）。若激活中仍发 3(active)，ECU 检测到
    # 「ACC激活+油门踏板」矛盾会写 DTC 锁死（00000015--seg23 踩油门0.8s accFaulted）。
    # 2026-09-01 st 镜像（方案B）：激活域内跟随原厂 ACC_05（原厂3→3、4→4）——OP 永不
    # 自己判定 st=4（消除 00000056「OP早切4vs原厂未确认」+ 65#3「原厂已退OP仍3」窗口）。
    # stock_st 不在 3/4（原厂已退出，carcontroller 已降级 long_active）兜底旧逻辑。
    # 2026-09-01 st=6 跟随退出（回放实锤 65-seg5/11/15）：原厂 st6 期间 OP 若仍发 3/4，
    # 会保留「原厂已故障/退出 vs OP 仍激活」矛盾窗口（原厂 st6 后实测 100% 回 st=2 待机，
    # 从未回 3）。故 stock_st==6 → 立即发 2 跟随，消除窗口。
    acc_control = stock_st if stock_st in (3, 4) else (2 if stock_st == 6 else (4 if gas_pressed else 3))
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


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, engine_torque=0, stock_esp=False, stock_follow=False, gas_override=False, stock_fv=False, stock_mom=0.0, slope_pct=0.0, slope_comp=False, slope_comp_unlimited=False, sng_resume_req=False, stock_verz=0.0, verz_follow=False, axg_comp=False, stock_axg=0.0, stock_fm=False, stock_anhalten=False):
  global _last_acc_moment
  global _last_ax_ge
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

  # 超驰斜坡触发（2026-08-24 对齐原厂）：上一帧 braking（减速中）→ 当前帧被 gas_override
  # 抑制（踩油门切超驰）→ 原厂发 verz 爬正斜坡（加速声明仪式，31次减速→超驰实证每次必发，
  # 峰值+1.285/180ms）。MacanSlopeComp 开关控制：开=发斜坡；关=直接归0（现状，31次实证
  # 原厂容忍）。斜坡期间若 braking 重新成立（踩刹车/松油门后目标减速），下一帧走 braking
  # 分支自然退出斜坡。
  global _prev_braking, _ovr_slope_active, _ovr_slope_step
  if slope_comp and not braking and _prev_braking and gas_override and not _ovr_slope_active:
    _ovr_slope_active = True
    _ovr_slope_step = 9
  _prev_braking = braking

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
    # 方案B：SnG 起步窗口 mom 下限=起步基线 65（2026-08-24，治 st6#2 "车没动"）。
    # 00000056 实测（330 vs 938 对照）：330s 成功案例 OP mom=23-35（比原厂 87-184 低
    # 4-5 倍！）但 vEgo 动了（0→0.3→1.0）→ 原厂放行；938s 失败 OP mom=40 且 vEgo 全程
    # 0.0（车纹丝不动）→ 原厂判"起步请求未响应"→ st6。**判据是"车动不动"不是 mom 差距**。
    # 修复：起步窗口 mom 下限=65（ECU 发力矩阈值 60 之上、原厂起步基线 57-87 下沿）——
    # 响应快（车能克服静止摩擦立即动，原厂放行）+ 柔和（比原厂 87-100+ 低 25%，爬升靠
    # +8/帧斜坡慢慢加，用户需求"稳稳的慢慢起步，安全第一"；后续成熟可逐级上调 75/85）。
    # 注：v_ego<=3.0 与方案A（v_ego>3 上限）代码层面完全互斥（sng_resume_req 窗口
    # 0.6s 内 v_ego 物理上也到不了 3.0）。
    if sng_resume_req and stock_mom > 0 and v_ego <= 3.0:
      acc_moment = max(acc_moment, 65)
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
      delta = _last_accel_cmd - target_verz
      if delta > 0.15:
        # 急刹：对齐原厂瞬间跳深（00000002 全量统计：原厂急刹 1.49/2.0 一帧到位，
        # 无 0.07 中间阶梯；旧代码 0.07/帧追深把急刹拉成 ~210ms 斜坡 → 刹车响应慢）。
        verz = target_verz
      else:
        verz = max(target_verz, _last_accel_cmd - 0.07)  # 缓刹渐进（对齐原厂平滑微调）
    else:
      verz = target_verz
    _last_accel_cmd = verz
  else:
    # 待机/关闭/故障：verz=0.0（对齐原厂）。00000002(67段)+00000056 全 route 实测：原厂
    # st=0/2/6 时 verz 全 0.0，原厂 verz 全域 [-2.0,+1.58] 从无 3.01。旧代码 3.01（raw2046
    # 饱和值）是自创占位，ECU 可能做值域检查判异常，且与原厂"待机=中性0"语义偏离——已修正。
    # 2026-08-24 坡度补偿扩展：MacanSlopeComp 开启且激活中上坡（slope_pct>0）时，
    # verz 发正值 = "加速声明"（对齐原厂）。00000002 拟合：verz_positive ≈ 4.2*sinθ
    # （坡度6%→+0.26、8%→+0.35、10%→+0.42，R²~0.95），上限 1.0 防过大。
    # 原厂语义（00000002 全 route 实证）：verz 是双向加速度请求（负=减速/0=巡航/正=加速
    # 声明+上坡补偿），正值期间 mom 同步发力矩（巡航）或 mom=0 交棒（超驰过渡）。开关关
    # =完全现状（verz=0）；mom 的 accel_eff 坡度补偿保留（上坡加力矩）。
    if _ovr_slope_active:
      # 超驰斜坡序列（对齐原厂 2043 窗口）：0.025 → 每帧+0.18 → 峰值1.285 → 归0
      if _ovr_slope_step >= 2:
        verz = round(0.025 + 0.18 * (9 - _ovr_slope_step), 3)
      else:
        verz = 0.0  # step=1：归0收尾（mom 接管发力）
      _ovr_slope_step -= 1
      if _ovr_slope_step <= 0:
        _ovr_slope_active = False
    elif acc_enabled and slope_comp and slope_pct > 0:
      verz = round(min(4.2 * math.sin(math.atan(slope_pct / 100.0)), 1.0), 3)
    else:
      verz = 0.0
    _last_accel_cmd = 0.0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=0, all others=0
  # ACC_ax_Getriebe（变速箱预期加速度提示）：2026-08-23 拟合 + 2026-08-24 修正。
  # 原厂实测（route 00000002 67段全量）：
  #  - 停车保持：axG=0.0（保持帧 5040 个中 93% 为 0；旧 0.55 拟合来自 00000055 seg05
  #    OP 代发场景，非纯原厂行为——0057 段7 原厂 src2 保持期 +1.992 同样是
  #    OP 代发场景下原厂计算值，纯原厂保持期为 0.0）
  #  - 起步/加速：随 mom 缓爬（axG≈0.01*mom：00000002 loes 起步 0.144→1.248 与
  #    mom 49→114 同步爬升）
  #  - 巡航：0.0（原厂 94% 时间），待机：0.0
  #  - 减速：负值透传（保留旧逻辑，速度相关 clamp 到 -2.016）
  # 注：旧注释"00000039 seg7 实锤原厂 axG=+1.63"被 00000002（67段完整原厂）推翻——
  # 那次大概率看的是 OP 自己代发的帧。
  # 注：停车保持 axG=0.0 对齐原厂（2026-08-25 终审：00000002 纯原厂 35599 静止帧
  # 98.3% axG 严格=0，非零仅刹停衰减尾巴/起步瞬间/保持微调；旧 0.55 来自
  # 00000055 seg05 OP 代发场景误拟合）。
  if acc_enabled:
    if stopping:
      ax_target = 0.0
    elif braking:
      ax_target = max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))
    elif accel > 0.05 or sng_resume_req:
      # 真正加速请求才提示变速箱（原厂巡航 axG=0，仅加速/起步爬升）。
      # 2026-08-24 修复（0057段7/0058段2/938 st6 实锤）：SnG 起步窗口即使 accel 低
      # （前车慢起步 <0.05m/s²）也强制 axG=0.01*mom 提示变速箱接合——原厂自动起步
      # axG 持续爬升（00000002 实测 0.144→1.248 与 mom 同步），旧代码 accel≤0.05 走
      # else → axG 掉 0 → 变速箱脱开 → 扭矩不传递 → 车不动（转速恒定 800）→ 原厂 st6。
      # axG 跟随 mom（0.01*mom）：前车起步又停 → stock_mom 掉 0 → mom 掉 → axG 自动
      # 归 0，不会空转提示加速（碰撞防护仍在 planner/MPC 层 + braking 分支优先）。
      ax_target = min(0.01 * acc_moment, 1.3)
    elif acc_control == 4 and not gas_override and v_ego > 5.0:
      # 超驰滑行（st=4 且司机松油门且车速>18km/h）：发负值提示变速箱降挡。
      # 原厂 4 个 route 确诊（00000002/04/05/49）：st=4 时 axG 负值占比 71-85%，
      # 渐进值 -0.024~-1.08（0.024 步进加深），语义="正在减速/滑行，准备降挡"。
      # 踩油门超驰（gas_override=True）仍走上方加速分支（加速意图，axG 正）。
      ax_target = -0.3  # 渐进由 _last_ax_ge 缓爬状态机自动完成
    else:
      ax_target = 0.0  # 巡航/匀速：不干扰变速箱
  else:
    ax_target = 0.0
  if _last_ax_ge < ax_target:
    _last_ax_ge = min(_last_ax_ge + 0.005, ax_target)
  else:
    _last_ax_ge = max(_last_ax_ge - 0.005, ax_target)
  ax_ge = round(_last_ax_ge, 3)
  # 2026-08-29 修复（65-5 实锤）：SnG 起步窗口 axG 缓爬 0.005/帧太慢——st6 前只爬到
  # 0.05（无效区，原厂起步即 0.144+）→ 变速箱不接合 → 车不动 → st6。MacanAxGComp
  # 开关开启时，sng_resume_req 起步窗口 axG 直接给有效下限（参考原厂 0.144），跳过无效区。
  # 只动 axG（变速箱预告），不碰 mom——实际加速体感不变。
  if axg_comp and sng_resume_req and ax_ge < 0.15:
    ax_ge = 0.15
    _last_ax_ge = 0.15
  # verz 跟随原厂（2026-08-29，65-11 实锤）：gas_override 时 braking 被抑制 verz=0，
  # 原厂却 verz<0 → 矛盾。跟随：verz 取原厂值（上限 -2.2 对齐原厂最深）。
  if verz_follow and stock_verz < -0.05 and verz > stock_verz:
    verz = max(stock_verz, -2.2)
  # 超驰透传（2026-08-29 4e/4f 38窗口3299帧离线验证）：超驰时OP axG矛盾62%、verz矛盾0.6%，
  # 透传原厂后0矛盾。驾驶员踩油门超驰=归还控制权，verz/axG跟随原厂值，消除
  # "OP帧(CAN0)≠雷达请求(CAN2)"执行反馈矛盾（st6根因）。verz钳制对齐原厂全域[-2.2,1.0]。
  if gas_override:
    verz = max(min(stock_verz, 1.0), -2.2)
    ax_ge = stock_axg
    # 2026-09-01 mom 透传（方案B）：介入期力矩=原厂值（轻踩撤力 mom=0→OP 发 0；深踩
    # 配合加速 mom>0→OP 发原厂值），杜绝「axG/verz 已透传、mom 仍 OP 巡航基线」混合矛盾。
    # _last 同步保证松油门后从原厂值平滑恢复（bump-less）。
    acc_moment = max(0, int(round(stock_mom)))
    _last_acc_moment = float(acc_moment)
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Verz_anf": verz,
    "ACC_Freigabe_Verzanf": (1 if stock_fv else 0) if gas_override else (1 if (braking or verz < -0.05 or (acc_enabled and stock_fv)) else 0),
    "ACC_Freigabe_Momentenanf": (1 if stock_fm else 0) if gas_override else (1 if (acc_enabled and not braking and not stock_follow) else 0),
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
    # 2026-08-23 事件化（对齐原厂 route 00000002）：原厂 loes=1 期间 verz 恒=0/mom>0，
    # 车动前回 0（500-620ms）。verz>=0 兜底：loes=1 绝不与刹车请求共存（loes+braking 也是矛盾帧）。
    # gas_override 独立条件移除——踩油门由 carcontroller 跟随原厂 loes 控制（不跟油门持续）。
    "ACC_Loeseanforderung": 1 if (acc_enabled and not stopping and verz >= 0.0 and sng_resume_req) else 0,
    "ACC_ax_Getriebe": ax_ge,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    # 00000039 seg7 实锤：原厂跟停全程 ESP=0（ESP_VerzTSK=0，靠1挡怠速拖滞），
    # 旧逻辑 stopping 时发 ESP=1 → 雷达自检异常 → st6。仅 esp_hold 或硬刹车(<-1.0) 才请求 ESP。
    # 0000003f 实锤：原厂跟停全程 ESP=0（怠速拖滞），旧条件 braking and accel<-1.0
    # 在仲裁把 accel 压到 stock_verz(-2) 时误触发 → OP ESP=1 与原厂矛盾。
    # 仲裁已保证 accel 最负 -1.0（原厂不减速时），accel<-1 只可能来自跟随原厂 verz，
    # 此时 ESP 应以原厂为准（透传 stock_esp）；esp_hold（原厂 ESP hold 确认）保留。
    "ACC_Beeinflussung_ESP": 1 if (esp_hold or stock_esp) else 0,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Anhalten": stock_anhalten if gas_override else stopping,
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
    # KD_Fehler (63|1 = byte7 bit7): 原厂实测（route 00000004 全59段）恒 1 = 正常。
    # DBC 命名误导——它是 ACC 健康位而非故障位。OP 此前漏设 → packer 恒发 0，
    # ECU 在激活+驾驶员介入(st=4)时判定"ACC 自报故障却仍在请求" → 锁死 ACC/PAS
    # （route 0000002e seg5 463.3s accFaulted 实锤）。与原厂逐字节对齐：恒 1。
    "ACC_KD_Fehler": 1,
  }
  # 2026-08-29 修复（65-11 实锤）：gas_override 时 braking=False → verz=0，但原厂仍发
  # verz<0（前车近要减速）→ OP verz=0 与原厂 -1.17 矛盾帧 → st6。MacanVerzFollow 开启时
  # 跟随原厂减速请求（ECU 层仲裁驾驶员油门 vs 减速请求——与原厂一致即不矛盾）。
  # 注意：跟随 verz 时 FV（减速许可）也必须=1（见 ACC_Freigabe_Verzanf 条件 verz<0）。
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4, stock_prim_anz=0, stock_status_anzeige=None, stock_texte_prim=0, stock_display_prio=None, stock_wunschgeschw=None):
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
    # ACC_Wunschgeschw_02：2026-08-25 改透传原厂（stock_wunschgeschw）。原厂行为
    # （00000002 纯原厂全量）：st=0 才清空为 327；st=2 待机 77% 帧保留上次设定值显示；
    # st=3/4 恒显示当前设定。旧逻辑 OP 自算 setSpeed → 待机期残留 OP 值最长 60s、
    # 激活期与原厂差 -3.8km/h（0058段10 实证）。None 回退旧逻辑（其他平台不受影响）。
    "ACC_Wunschgeschw_02": (set_speed if set_speed < 250 else 327.36) if stock_wunschgeschw is None else stock_wunschgeschw,
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
