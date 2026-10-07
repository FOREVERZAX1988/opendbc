"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Macan（VW MLB）Abstandsindex <-> 距离 的【唯一】标定源（单表 B1）。

【为什么要有这个文件 —— 2026-10-09 复盘】
修复前同一份代码里存在两套互不相同的 idx→米 公式，同一个 idx 给出两个距离：
  - 融合/仪表（radard A2 / radar_interface A3 / carcontroller 仪表）
        时距表 B1  t = 0.008969*idx + 0.332 ,  d = t * max(v, 5)
  - SnG 起步门（opendbc/sunnypilot/car/volkswagen/stop_and_go.py）
        线性近似  d = idx * 0.0424     （丢掉 +0.332 截距）
例：idx=188 -> 融合 10.09 m，SnG 门 7.97 m，差 2.1 m。
“同一个量两个数字”本身就是 bug（门与执行层对同一辆车距判断不一致），
现全部收敛到本模块：任何一处要改标定，只改这里。
"""

MACAN_B1_T_A = 0.008969
MACAN_B1_T_B = 0.332

# idx 是“时距”口径，低速（v->0）时用 max(v, 5) 兜底，避免 t*v 退化到 0
MACAN_IDX_EQUIV_V = 5.0

# Abstandsindex 有效域：0 = 无目标；1021 = 原厂“无目标/饱和”哨兵
MACAN_IDX_MIN = 1.0
MACAN_IDX_MAX = 1020.0


def t_from_idx(idx: float) -> float:
  """Abstandsindex -> 时距 t(s)（B1 直线）。"""
  return MACAN_B1_T_A * idx + MACAN_B1_T_B


def idx_to_drel(idx: float, v_ego: float) -> float:
  """Abstandsindex -> 本车到前车距离(m)。低速用等效 t*max(v, 5)。"""
  return t_from_idx(idx) * (v_ego if v_ego > MACAN_IDX_EQUIV_V else MACAN_IDX_EQUIV_V)


def drel_to_idx(drel: float, v_ego: float) -> float:
  """距离(m) -> Abstandsindex（B1 反解，钳到有效域 1..1020）。返回 float，调用方按需取整。"""
  t = drel / (v_ego if v_ego > MACAN_IDX_EQUIV_V else MACAN_IDX_EQUIV_V)
  v = (t - MACAN_B1_T_B) / MACAN_B1_T_A
  return float(min(max(v, MACAN_IDX_MIN), MACAN_IDX_MAX))


def idx_is_valid(idx: float) -> bool:
  """0（无目标）与 >=1021（哨兵）都视为无有效目标。"""
  return MACAN_IDX_MIN <= idx <= MACAN_IDX_MAX
