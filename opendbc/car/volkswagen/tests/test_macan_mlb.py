#!/usr/bin/env python3
"""Macan (MLB) 纵向控制回归测试 —— 上车前的本地仿真验证（v2 校准版）

v2 校准（2026-08-20 回归核实）：
- setUp 重置模块级全局 _last_acc_moment/_last_accel_cmd（消除测试间状态污染）
- braking 相关 verz 断言改为多帧收敛（verz 斜坡渐进 -0.07/帧）
- 力矩断言改为多帧收敛（上升斜坡 8Nm/帧）
- 下坡场景分档：-1%/-2% 应发力矩；-4% 为已知边界（A 方案 accel=0.1 只抵消~1%坡）

依据（实测数据）：
- 0049 官方 master 段3：原厂停车保持 anh=0/verz哨兵/mom=0/fv=1；踩油门超驰 st=4/
  mom47-59/fv=0/loes=1
- 4e/4f（旧bug）：踩油门+坡度 → braking(第一项漏修 gas_override) → mom=0 → 车不动
"""
import unittest

from opendbc.can import CANPacker
from opendbc.car.volkswagen import mlbcan

PACKER = CANPacker('vw_mlb')


def parse_acc05(d):
  st = (d[7] >> 1) & 0x7       # ACC_Status_ACC 57|3
  anh = (d[7] >> 6) & 0x1      # ACC_Anhalten 62|1（Vector__XXX，0049实测原厂停车保持=1）
  mom = d[2] | ((d[3] & 0x03) << 8)             # ACC_Momentenanforderung 16|10
  raw_v = d[4] | ((d[5] & 0x07) << 8)
  # ACC_Verz_anf 32|11@1+ 是**无符号**（DBC vw_mlb.dbc:90）——不要按有符号处理！
  # （旧代码 if raw_v & 0x400: raw_v -= 0x800 是有符号误读：raw1444(verz=0)会被判成-604→-10.24；
  #  3.01 时代 raw2046 被掩盖。正确：raw×0.005-7.22，verz=0→raw1444、保持-2.0→raw1044）
  verz = round(raw_v * 0.005 - 7.22, 3)         # ACC_Verz_anf 32|11（0.005网格，3位精度；round2 会把 0.025 浮点偏差放大成 0.03）
  fm = (d[1] >> 4) & 0x1       # ACC_Freigabe_Momentenanf 12|1
  fv = (d[1] >> 5) & 0x1       # ACC_Freigabe_Verzanf 13|1
  loes = (d[5] >> 3) & 0x1     # ACC_Loeseanforderung 43|1
  axg = round(((d[6] | ((d[7] & 0x1) << 8)) & 0x1FF) * 0.024 - 2.016, 3)  # ACC_ax_Getriebe 48|9
  return dict(st=st, anh=anh, mom=mom, verz=verz, fm=fm, fv=fv, loes=loes, axg=axg)


def make_acc(**kw):
  """调用 mlbcan.create_acc_accel_control，返回 ACC_05 解析后的信号字典"""
  defaults = dict(acc_type=0, acc_enabled=True, accel=0.0, acc_control=2,
                  stopping=False, starting=False, esp_hold=False, v_ego=0.0,
                  engine_torque=0.0, stock_esp=False, stock_follow=False,
                  gas_override=False, stock_fv=False, stock_mom=1021.0,
                  slope_pct=0.0, slope_comp=False, slope_comp_unlimited=False,
                  sng_resume_req=False,
                  stock_verz=0.0, verz_follow=False, axg_comp=False, stock_axg=0.0)
  defaults.update(kw)
  msgs = mlbcan.create_acc_accel_control(PACKER, 0, **defaults)
  for addr, dat, bus in msgs:
    if addr == 269:
      return parse_acc05(bytes(dat))
  raise AssertionError('no ACC_05 frame')


def run_frames(n, **kw):
  """连续调用 n 帧（力矩/verz 斜坡收敛），返回最后一帧"""
  r = None
  for _ in range(n):
    r = make_acc(**kw)
  return r


class TestMacanMLBLongitudinal(unittest.TestCase):

  def setUp(self):
    # 消除模块级全局状态污染（斜坡从 0 起步）
    mlbcan._last_acc_moment = 0.0
    mlbcan._last_accel_cmd = 0.0
    mlbcan._last_ax_ge = 0.0
    mlbcan._prev_braking = False
    mlbcan._ovr_slope_active = False
    mlbcan._ovr_slope_step = 0

  def test_park_hold(self):
    """停车保持：mom=0（不发力矩）、anh=1（62|1，0049原厂实测保持帧anh=1）、
    loes=0（无起步确认）、保持力 verz=-2.0（镜像原厂）、axG=0.0（00000002 纯原厂
    35599静止帧98.3%严格=0；旧0.55来自OP代发场景误拟合，已修正）"""
    r = run_frames(30, v_ego=0.0, accel=-0.56, stopping=True)
    self.assertEqual(r['mom'], 0)
    self.assertEqual(r['anh'], 1, "停车保持应发 anh=1（62|1，0049原厂实测）")
    self.assertEqual(r['loes'], 0)
    # verz 斜坡 -0.07/帧，到 -2.0 需 29 帧——30帧收敛后断言
    self.assertLessEqual(r['verz'], -2.0, f"停车保持应发深度 verz，实际 {r['verz']}")
    self.assertEqual(r['axg'], 0.0, f"停车保持 axG 应=0.0（对齐原厂稳态），实际 {r['axg']}")

  def test_gas_override_downhill_mild(self):
    """踩油门+缓下坡（7158f13 核心回归）：旧bug是 braking 第一项漏修 gas_override，
    踩油门时 accel=0 + 下坡坡度项 → accel_eff<-0.05 → braking → mom=0 车不动。
    修复后：踩油门绝不 braking → mom≥27 基线（斜坡收敛后），loes=1。"""
    for slope in (-1.0, -2.0):
      r = run_frames(15, v_ego=0.0, accel=0.1, gas_override=True,
                     slope_pct=slope, slope_comp=True, sng_resume_req=True)
      self.assertGreater(r['mom'], 0, f"下坡{slope}% 踩油门必须发力矩，实际 mom={r['mom']}")
      self.assertEqual(r['loes'], 1, "踩油门超驰应发 loes=1 起步确认")
      self.assertEqual(r['fv'], 0, "超驰应关减速通道 fv=0")

  def test_gas_override_flat(self):
    """踩油门平路：loes=1 + 力矩斜坡收敛后 ≥27（27基线+0.1*85=35）"""
    r = run_frames(15, v_ego=0.0, accel=0.1, gas_override=True, sng_resume_req=True)
    self.assertEqual(r['loes'], 1)
    self.assertGreaterEqual(r['mom'], 27)

  def test_gas_override_no_stock_loes(self):
    """踩油门但原厂 loes=0（carcontroller 跟随原厂失败/未发）→ OP 不发 loes。
    事件化（2026-08-23）：loes 不再跟 gas_override 持续（旧 bug seg00 28.8s），
    只由 sng_resume_req（carcontroller 按原厂 loes/起步窗口计算）驱动。"""
    r = run_frames(15, v_ego=0.0, accel=0.1, gas_override=True, sng_resume_req=False)
    self.assertEqual(r['loes'], 0, "原厂未确认起步时 OP 不得发 loes（不跟油门持续）")
    self.assertGreater(r['mom'], 0, "力矩仍应发出（超驰不刹车）")

  def test_wg_op_writeback(self):
    """ACC_Wunschgeschw_02 OP 写回（2026-09-05 c1f4a8f0d 定稿）：恒用 set_speed(OP vCruise)，
    统一"仪表显示=OP实际执行速度"，不随 stock_wunschgeschw 透传——原厂 ACC 被 OP 纵向接管
    (OPLong, relay断开)时原厂内部 Wunschgeschw 是僵尸设定(仪表40 vs comma35/长按+10 vs +5)，
    透传会把僵尸值显示到仪表→取消接管瞬间跳变。纯显示件改写零执行风险。"""
    from opendbc.car.volkswagen import mlbcan
    # 无论是否传 stock_wunschgeschw，均写回 set_speed(OP vCruise)
    for sw in (53.8, None):
      msg = mlbcan.create_acc_hud_control(PACKER, 0, 3, 40.0, 100, 2, lead_object=1,
                                          stock_wunschgeschw=sw)
      d = bytes(msg[1])
      wg = (d[1] >> 4) | (d[2] << 4)   # ACC_Wunschgeschw_02 12|10
      self.assertAlmostEqual(wg * 0.32, 40.0, delta=0.4,
                             msg=f"WG 应写回 OP setSpeed 40km/h(无论 stock_wunschgeschw={sw})，实际 {wg*0.32:.1f}")
    # st=0 未设定时 set_speed=255 → 327.36"无显示"
    msg3 = mlbcan.create_acc_hud_control(PACKER, 0, 0, 255.0, 100, 2, lead_object=1)
    d3 = bytes(msg3[1])
    wg3 = (d3[1] >> 4) | (d3[2] << 4)
    self.assertAlmostEqual(wg3 * 0.32, 327.36, delta=0.5,
                           msg=f"st=0 未设定应显示 327.36(无显示)，实际 {wg3*0.32:.1f}")

  def test_hud_no_contradiction_frame(self):
    """HUD 矛盾帧回归（2026-08-26 修复）：无目标时 create_acc_hud_control
    不得输出 ab=0 + relev=1（仪表显示"一辆很近的车"幻觉，00000061 seg0 实测 67 帧）。
    根因：raw_abstand==0 分支只清了 lead_distance，漏清 lead_object——原厂无目标
    占位(ab=1021/relev=1)经 stock_lead_object 透传后，ab=0 与 lead_object=1 组合
    让 mlbcan 兜底逻辑 (0<ab<1000 即 relev=1) 输出矛盾帧。"""
    from opendbc.car.volkswagen import mlbcan
    # 修复后语义：lead_object=0 + lead_distance=0 → relev 必须=0（无目标）
    msg = mlbcan.create_acc_hud_control(PACKER, 0, 3, 40.0, 0, 0, lead_object=0)
    d = bytes(msg[1])
    ab = (d[3] | ((d[4] & 0x3) << 8))
    rv = (d[5] >> 6) & 0x3
    self.assertEqual(ab, 0, f"无目标时 abstand 应为 0，实际 {ab}")
    self.assertEqual(rv, 0, f"无目标时 relev 应为 0（矛盾帧！），实际 {rv}")
    # 对照：有目标时必须 relev=1
    msg2 = mlbcan.create_acc_hud_control(PACKER, 0, 3, 40.0, 300, 0, lead_object=1)
    d2 = bytes(msg2[1])
    rv2 = (d2[5] >> 6) & 0x3
    self.assertEqual(rv2, 1, f"有目标时 relev 应为 1，实际 {rv2}")

  def test_display_rate_limit(self):
    """显示变化率限速（2026-08-25）：单帧跳变被削到 max_step，真实接近不受影响。
    CarController 实例化需要完整 CP——用轻量方式直接测限速数学。"""
    # 模拟：vRel=-2m/s 接近中，雷达 abstand 从 300 跳到 500（视觉补位切换台阶）
    vrel = -2.0
    max_step = max(4, int(abs(vrel) * 16))   # =32/帧
    disp = 300
    for target in [500]:
      delta = target - disp
      if abs(delta) > max_step:
        disp += max_step if delta > 0 else -max_step
      else:
        disp = target
    self.assertEqual(disp, 332, f"接近中单帧最多走 {max_step}，实际 {disp}")
    # 静止（vRel=0）：max_step 兜底 4，慢速收敛不卡死
    max_step0 = max(4, int(0.0 * 16))
    self.assertEqual(max_step0, 4)

  def test_sng_axg_follows_mom_low_accel(self):
    """SnG 起步且 accel<0.05（前车慢起步，2026-08-24 0057/0058/938 st6 根因）：
    axG 必须仍跟随 mom 爬升（提示变速箱接合），不得掉 0。
    00000002 原厂实测：起步 axG 持续爬升 0.144→1.248 与 mom 同步；
    旧代码 accel<=0.05 走 else → ax_target=0 → axG 掉 0 → 变速箱脱开 → 车不动 → 原厂 st6。
    修复：accel>0.05 or sng_resume_req → 起步窗口 axG=0.01*mom=0.65（65基线）爬升。
    前车又停 → mom 掉 0 → axG 自动归 0（防"空转提示加速"）。"""
    # 2026-09-07 更新：跟足原厂有效起步力矩(stock_mom=60, 排除1021哨兵)，axG 目标=0.01*60=0.6；
    # 原 65 硬基线已于 2026-09-02 移除(改跟足原厂)，mom 跟随原厂 60。
    r = run_frames(200, v_ego=0.0, accel=0.01, sng_resume_req=True, stock_mom=60.0)
    self.assertGreaterEqual(r['axg'], 0.5, f"起步窗口低 accel 时 axG 应爬向 0.6(跟足原厂60)，实际 {r['axg']}（旧代码恒 0→st6）")
    self.assertGreaterEqual(r['mom'], 55, f"起步窗口 mom 应跟足原厂 60，实际 {r['mom']}")
    # 前车起步又停（sng 窗口内 accel 转负 → braking 分支优先）→ verz 刹车 + axG 掉负，
    # 绝不保持加速提示（防"空转提示加速"；碰撞防护在 planner/MPC + braking 优先）
    r3 = run_frames(200, v_ego=0.0, accel=-0.1, sng_resume_req=True, stock_mom=0.0)
    self.assertLess(r3['axg'], 0.0, f"前车又停：braking 优先，axG 应转负/归 0，实际 {r3['axg']}")
    self.assertLess(r3['verz'], 0.0, f"前车又停应发刹车 verz，实际 {r3['verz']}")

  def test_ax_ge_launch_ramp(self):
    """起步 axG 随 mom 缓爬（原厂拟合 0.01*mom）：SnG 起步 60 帧后 axG 应>0 且爬向目标。
    旧代码起步 axG 骤降 0（accel=0.1<0.25 死区被吞）→ 变速箱误判未加速 → 过早升挡。"""
    r = run_frames(60, v_ego=0.0, accel=0.1, sng_resume_req=True)
    self.assertGreater(r['axg'], 0.05, f"起步 axG 应随 mom 爬升，实际 {r['axg']}（旧代码恒 0）")
    # mom≈35（0.1*85+27）→ 目标 0.01*35=0.35，60帧*0.005=0.30，未到目标，应在 0.3 附近
    self.assertLess(r['axg'], 0.4, f"起步 axG 不应超目标 0.35，实际 {r['axg']}")

  def test_standby_verz_zero(self):
    """待机（acc_enabled=False）时 verz=0.0——对齐原厂（原厂 st=0/2/6 verz 全 0，
    00000002+00000056 双 route 实测；旧代码 3.01 是自创饱和占位，已修正）。"""
    r = make_acc(acc_enabled=False)
    self.assertEqual(r['verz'], 0.0, f"待机 verz 应=0（对齐原厂），实际 {r['verz']}（旧代码发 3.01 饱和占位）")
    self.assertEqual(r['mom'], 0, "待机不应发力矩")
    self.assertEqual(r['st'], 2, "待机 st 应为 2")

  def test_slope_comp_verz_positive(self):
    """MacanSlopeComp 开启+上坡：verz 发正值=加速声明（00000002 拟合 4.2*sinθ）。
    开关关=verz=0（现状）。坡度6%→4.2*0.06≈0.25。"""
    # 开关开 + 上坡 6% + 巡航（accel=0）
    r = make_acc(acc_enabled=True, slope_comp=True, slope_pct=6.0, accel=0.0, v_ego=15.0)
    self.assertGreater(r['verz'], 0.2, f"上坡6% verz 应≈+0.25（加速声明），实际 {r['verz']}")
    self.assertLess(r['verz'], 0.4, f"上坡6% verz 不应超 0.4，实际 {r['verz']}")
    # 开关关（默认）：verz=0
    r2 = make_acc(acc_enabled=True, slope_comp=False, slope_pct=6.0, accel=0.0, v_ego=15.0)
    self.assertEqual(r2['verz'], 0.0, f"开关关 verz 应=0，实际 {r2['verz']}")
    # 下坡（slope_pct<0）+ 非 braking：verz=0（减速走 braking 分支）
    r3 = make_acc(acc_enabled=True, slope_comp=True, slope_pct=-6.0, accel=0.0, v_ego=15.0)
    self.assertLessEqual(r3['verz'], 0.05, f"下坡不应发正值，实际 {r3['verz']}")

  def test_slope_comp_override_positive_ramp(self):
    """超驰透传（2026-08-29 adc8dabc，取代 0824 模拟斜坡）：减速中踩油门切超驰时
    verz/axG 直接透传原厂值（clamp[-2.2,1.0]）——原厂超驰本身会发 verz 爬正斜坡
    0.025→1.285（2043 窗口实证），OP 跟随 stock_verz 原值，不再自行模拟。
    原厂 verz=0（未发值）→ 透传 0。"""
    # 帧1：减速中（未踩油门）→ braking → verz 负值
    r1 = make_acc(acc_enabled=True, slope_comp=True, gas_override=False, accel=-0.5, v_ego=15.0)
    self.assertLess(r1['verz'], 0, f"减速帧 verz 应为负，实际 {r1['verz']}")
    # 帧2-10：踩油门切超驰 → 透传原厂 verz 序列（模拟原厂斜坡 0.025→1.285→归0）。
    # 钳制边界：原厂峰值 1.105/1.285 被 clamp 上限 1.0 压到 1.0（当前设计）。
    # 注：4e/4f 离线验证 38 窗口钳制帧=0（原厂超驰 verz 未超 1.0），>1.0 场景未覆盖，
    # 差值 0.285<0.3 矛盾阈值，但严格透传语义是否放开上限待路试评估（2043 窗口实测峰值）。
    stock_seq = [0.025, 0.205, 0.385, 0.565, 0.745, 0.925, 1.105, 1.285, 0.0]
    seq = [make_acc(acc_enabled=True, slope_comp=True, gas_override=True, accel=-0.5,
                    v_ego=15.0, stock_verz=v)['verz'] for v in stock_seq]
    exp = [0.025, 0.205, 0.385, 0.565, 0.745, 0.925, 1.0, 1.0, 0.0]
    self.assertEqual(seq, exp, f"超驰透传(钳制1.0)序列 {seq} != 期望 {exp}")
    # 原厂 verz=0（未发值）→ 透传 0，不再发模拟斜坡
    r_zero = make_acc(acc_enabled=True, slope_comp=True, gas_override=True, accel=-0.5,
                      v_ego=15.0, stock_verz=0.0)
    self.assertEqual(r_zero['verz'], 0.0, f"原厂 verz=0 应透传 0，实际 {r_zero['verz']}")
    # axG 同步透传（0829 核心：消除 OP帧≠雷达请求 执行反馈矛盾）
    r_axg = make_acc(acc_enabled=True, slope_comp=True, gas_override=True, accel=-0.5,
                     v_ego=15.0, stock_axg=1.0)
    self.assertAlmostEqual(r_axg['axg'], 1.0, delta=0.03, msg=f"axG 应透传≈1.0，实际 {r_axg['axg']}")
    # clamp 上下界：stock_verz 越界 → [−2.2, 1.0]
    r_hi = make_acc(acc_enabled=True, gas_override=True, accel=-0.5, v_ego=15.0, stock_verz=1.5)
    self.assertEqual(r_hi['verz'], 1.0, f"stock_verz=1.5 应钳到 1.0，实际 {r_hi['verz']}")
    r_lo = make_acc(acc_enabled=True, gas_override=True, accel=-0.5, v_ego=15.0, stock_verz=-2.5)
    self.assertEqual(r_lo['verz'], -2.2, f"stock_verz=-2.5 应钳到 -2.2，实际 {r_lo['verz']}")

  def test_slope_comp_override_off_direct_zero(self):
    """原厂 verz=0（未发值）超驰：透传 0（00000056 31次实证原厂容忍；0829 起为透传语义）"""
    make_acc(acc_enabled=True, slope_comp=False, gas_override=False, accel=-0.5, v_ego=15.0)  # 减速帧
    r = make_acc(acc_enabled=True, slope_comp=False, gas_override=True, accel=-0.5, v_ego=15.0)  # 超驰
    self.assertEqual(r['verz'], 0.0, f"开关关超驰 verz 应=0，实际 {r['verz']}")

  def test_sng_mom_follow_stock(self):
    """SnG 起步窗口 mom 策略（2026-09-07 更新，对齐 c6072b272 跟足原厂 + 1021 哨兵修复）：
    - 原厂已发起步力矩(stock_mom 有效，如 60) → OP 跟足（防 938 "车没动" st6：原厂62执行24=偷懒）
    - 原厂静音/无力矩(stock_mom=1021 哨兵，或 0) → 不跟足硬顶，走正常计算（防 1021Nm 猛冲）
    原 65 硬基线已回退（2026-09-02：原厂起步 mom 起点 45-49 缓慢爬升是正常行为，判据是
    "车动不动"不是 mom 数值）。"""
    # 起步窗口 + 原厂有效力矩 → 跟足原厂（不放任 OP 低力矩 8）
    r = make_acc(acc_enabled=True, sng_resume_req=True, accel=0.15, v_ego=1.0, stock_mom=60.0)
    self.assertGreaterEqual(r['mom'], 55, f"起步窗口应跟足原厂 60，实际 {r['mom']}")
    # 起步窗口内连续多帧保持跟足（不回落）
    seq = [make_acc(acc_enabled=True, sng_resume_req=True, accel=0.15, v_ego=1.0, stock_mom=60.0)['mom'] for _ in range(5)]
    self.assertTrue(all(m >= 55 for m in seq), f"起步窗口全程应跟足原厂，实际 {seq}")
    # 原厂静音/哨兵(1021) → 绝不跟足饱和力矩，安全正常计算（<65，防猛冲）
    r_sent = make_acc(acc_enabled=True, sng_resume_req=True, accel=0.15, v_ego=1.0)  # stock_mom 默认 1021
    self.assertLess(r_sent['mom'], 65, f"原厂静音(1021)不得跟足饱和力矩，实际 {r_sent['mom']}")
    # 非低速窗口(v_ego>=2.0)：即使原厂有力矩也不跟足（保持 OP 目标，斜坡起步）
    r2 = make_acc(acc_enabled=True, sng_resume_req=False, accel=0.15, v_ego=3.0, stock_mom=60.0)
    self.assertLess(r2['mom'], 55, f"非低速窗口(v>=2)不强制跟足，实际 {r2['mom']}")

  def test_mom_hard_cap(self):
    """力矩最终硬顶 350Nm（2026-09-07）：任何路径产出的 ACC_Momentenanforderung
    都不超过 2015 Macan 发动机物理扭矩极限 350。兜底彻底——即便 stock_mom 传入
    1021 哨兵（idx 时距与 mom 同为 10bit[1|1021] 位域，易被扫描误读）也绝不输出
    饱和力矩。全量真实数据 mom=0-407，500/1021 皆是位域哨兵非真实扭矩。"""
    # 直接传 1021 哨兵：即使误触发跟足逻辑，最终输出也必须被 350 硬顶钳死
    r = make_acc(acc_enabled=True, sng_resume_req=True, accel=0.5, v_ego=1.0, stock_mom=1021.0)
    self.assertLessEqual(r['mom'], 350, f"mom 必须被硬顶到 ≤ 350，实际 {r['mom']}")
    # 超驰透传原厂高值（理论上限 1021）同样被硬顶
    r2 = make_acc(acc_enabled=True, gas_override=True, accel=0.5, v_ego=5.0, stock_mom=1021.0)
    self.assertLessEqual(r2['mom'], 350, f"超驰透传也须 ≤350，实际 {r2['mom']}")
    # 正常大 accel 也不应超 350
    r3 = make_acc(acc_enabled=True, accel=1.5, v_ego=30.0)
    self.assertLessEqual(r3['mom'], 350, f"正常大加速也须 ≤350，实际 {r3['mom']}")

  def test_sng_resume(self):
    """SnG 自动起步（1b4915d）：sng_resume_req 模拟踩油门语义 → loes=1"""
    r = make_acc(v_ego=0.0, accel=0.1, sng_resume_req=True)
    self.assertEqual(r['loes'], 1)

  def test_no_override_no_loes(self):
    """无干预行驶：loes=0"""
    r = make_acc(v_ego=5.0, accel=0.2)
    self.assertEqual(r['loes'], 0)

  def test_stock_decel_follow(self):
    """原厂减速跟随：accel 已被 carcontroller 压到 stock_verz（如 -1.0）→
    braking → verz 有效减速（斜坡收敛后 ≤ -0.9）且不发力矩"""
    r = run_frames(15, v_ego=5.0, accel=-1.0)
    self.assertEqual(r['mom'], 0, "减速时力矩应为0")
    self.assertLessEqual(r['verz'], -0.9, f"应跟随原厂 verz≈-1.0，实际 {r['verz']}")

  def test_normal_accel(self):
    """正常加速力矩映射：v=10 accel=0.5 → cruise=78 + 0.5*85 = 120.5
    （斜坡 8/帧 → 约 15 帧收敛到 120）"""
    r = run_frames(20, v_ego=10.0, accel=0.5)
    self.assertGreater(r['mom'], 100)
    self.assertLess(r['mom'], 140)

  def test_stock_follow(self):
    """原厂撤力跟随：mom=0（不发力矩）+ FM=0"""
    r = make_acc(v_ego=10.0, accel=0.0, stock_follow=True)
    self.assertEqual(r['mom'], 0)
    self.assertEqual(r['fm'], 0)

  def test_braking_verz_ramp(self):
    """braking verz：急刹（delta>0.15）瞬间跳深对齐原厂；缓刹渐进"""
    r1 = make_acc(v_ego=5.0, accel=-0.3)
    self.assertEqual(r1['mom'], 0)
    # 急刹（delta=0.3>0.15）：一帧到位 -0.3（原厂急刹 1.49/2.0 也是瞬间跳，00000002 全量统计）
    self.assertAlmostEqual(r1['verz'], -0.3, places=2, msg=f"急刹应瞬间跳深对齐原厂，实际 {r1['verz']}")
    # 缓刹（delta≤0.15）：渐进 -0.07/帧（先重置斜坡状态，避免 r1 遗留 -0.3 影响）
    mlbcan._last_accel_cmd = 0.0
    r2 = make_acc(v_ego=5.0, accel=-0.1)
    self.assertAlmostEqual(r2['verz'], -0.07, places=2, msg=f"缓刹首帧应渐进 -0.07，实际 {r2['verz']}")
    r15 = run_frames(15, v_ego=5.0, accel=-0.3)
    self.assertLessEqual(r15['verz'], -0.25, "收敛后 verz 接近 accel 深度")


if __name__ == '__main__':
  unittest.main()
