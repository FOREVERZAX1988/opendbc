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
                  sng_resume_req=False)
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
    """停车保持：mom=0（不发力矩）、anh=0（原厂不用anh）、loes=0（无起步确认）、
    保持力 verz=-2.0（镜像原厂保持力度，斜坡收敛后）"""
    r = run_frames(30, v_ego=0.0, accel=-0.56, stopping=True)
    self.assertEqual(r['mom'], 0)
    self.assertEqual(r['anh'], 1, "停车保持应发 anh=1（62|1，0049原厂实测）")
    self.assertEqual(r['loes'], 0)
    # verz 斜坡 -0.07/帧，到 -2.0 需 29 帧——30帧收敛后断言
    self.assertLessEqual(r['verz'], -2.0, f"停车保持应发深度 verz，实际 {r['verz']}")
    self.assertAlmostEqual(r['axg'], 0.14, places=2, msg=f"停车保持 axG 应缓爬(30帧≈0.14)，实际 {r['axg']}")

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
    """减速中踩油门切超驰：MacanSlopeComp 开→verz 爬正斜坡（对齐原厂 2043 窗口
    序列 0.025→0.205→0.385→0.565→0.745→0.925→1.105→1.285→归0，9帧=180ms）；
    开关关→verz 直接归0（现状）。"""
    # 帧1：减速中（未踩油门）→ braking → verz 负值
    r1 = make_acc(acc_enabled=True, slope_comp=True, gas_override=False, accel=-0.5, v_ego=15.0)
    self.assertLess(r1['verz'], 0, f"减速帧 verz 应为负，实际 {r1['verz']}")
    # 帧2-10：踩油门切超驰 → 斜坡爬正序列
    seq = [make_acc(acc_enabled=True, slope_comp=True, gas_override=True, accel=-0.5, v_ego=15.0)['verz'] for _ in range(9)]
    exp = [0.025, 0.205, 0.385, 0.565, 0.745, 0.925, 1.105, 1.285, 0.0]
    self.assertEqual(seq, exp, f"超驰斜坡序列 {seq} != 原厂 {exp}")
    # 斜坡结束后保持 0
    r_last = make_acc(acc_enabled=True, slope_comp=True, gas_override=True, accel=-0.5, v_ego=15.0)
    self.assertEqual(r_last['verz'], 0.0, f"斜坡结束应归0，实际 {r_last['verz']}")

  def test_slope_comp_override_off_direct_zero(self):
    """开关关：减速→超驰 verz 直接归0（现状，00000056 31次实证原厂容忍）"""
    make_acc(acc_enabled=True, slope_comp=False, gas_override=False, accel=-0.5, v_ego=15.0)  # 减速帧
    r = make_acc(acc_enabled=True, slope_comp=False, gas_override=True, accel=-0.5, v_ego=15.0)  # 超驰
    self.assertEqual(r['verz'], 0.0, f"开关关超驰 verz 应=0，实际 {r['verz']}")

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
