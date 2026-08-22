"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.volkswagen.values_ext import VolkswagenFlagsSP

# OP 判定可起步的加速度阈值（m/s²）。停车保持态时 OP 的 aTarget≈0 或负（前车未动时
# accel 恒 -0.55），前车起步/绿灯时视觉模型输出正加速度请求（0000004c 全29段实测
# 2026-08-17：无 gas 长停车 planner accel 峰值仅 0.00-0.26，旧阈值 0.3 永远达不到
# ——SnG 12次停车0次自动起步的根因）。0.15 + 连续帧确认可覆盖实测无 gas 峰值
# （seg11=0.16 / seg14=0.17 / seg24=0.23），同时滤除单帧模型抖动（停车保持态
# accel≈0 或负，误触发面很小）。
_RESUME_ACCEL_THRESHOLD = 0.15
# 起步意图确认帧数：连续 N 帧 accel>阈值才触发（5 帧 ≈ 50ms @100Hz 控制帧率）
_RESUME_CONFIRM_FRAMES = 5
# 起步确认阈值：vEgo 超过该值视为车已动起来，重置防抖状态
_RESUME_VEGO_RESET = 0.5
# 停车保持态下连续发送 RESUME 的最大帧数（0.2s @100Hz 控制帧率）
_RESUME_MAX_FRAMES = 20


class SnGCarController:
  """Macan (MLB) 起步跟停：

  原厂 ACC 停车进入保持态（ACC_Anhalten=1）后需要起步信号（轻踩油门/SET/RESUME）
  才恢复（00000047 全route实测：9次停车7次需踩油门）。本模块在开关开启时，
  由 OP 视觉模型判定可起步（planner 输出正加速度）后，代发 LS_01 RESUME 按键帧
  到 CAN.ext（bus2，雷达侧）——原厂收到 RESUME 后自动解除停车保持态并起步，
  OP 仲裁跟随原厂（00000047 seg29 1765s 实测：gas=1 瞬间原厂 anh 1→0 放行）。

  与 Subaru SnG 的区别：Subaru 是 OP 全控（代发 throttle/brake_pedal），
  Macan 是原厂 ACC 仲裁（代发按键信号让原厂自己解除），更安全、与"原厂意图
  仲裁优先"设计一致。
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    # 平台过滤：仅 Macan(MLB) 生效——其他 VW 平台即使误开开关也不触发（安全兜底）
    self._platform_ok = (CP.brand == "volkswagen" and CP.carFingerprint == "PORSCHE_MACAN_MK1")
    # 初始 enabled：flags（card 启动时按 MacanStartStop 设置）。若 Params 可达则
    # 以 Params 为准并定时刷新（update_stop_and_go 内每 100 帧）——中途开/关开关
    # 无需重启 car 进程即生效（0000004f 实测根因：CP_SP.flags 开机后固定，
    # 中途开开关 enabled 仍 False，SnG 永不触发）。
    self.enabled = self._platform_ok and bool(CP_SP.flags & VolkswagenFlagsSP.STOP_AND_GO)
    # 起步安全距离开关（MacanStartStopDistance）：开=需雷达/视觉确认前车距离才自动起步（安全）；
    # 关=V1 纯意图起步（拥堵防加塞）。默认开（与历史 v2 行为一致）。仅 SnG 开启时生效。
    self._distance_enabled = True
    self._mp = None
    try:
      from openpilot.common.params import Params
      self._mp = Params()
      self.enabled = self._platform_ok and self._mp.get_bool("MacanStartStop")
      self._distance_enabled = self._mp.get_bool("MacanStartStopDistance")
    except Exception:
      pass  # opendbc 测试环境无 openpilot 包：保持 flags 判断

    self._last_refresh_frame = -100  # 首次调用立即刷新
    self.last_standstill_frame = 0
    self.resume_frames_sent = 0
    self.confirm_frames = 0
    self.prev_close_distance = 0.0

  def update_stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int,
                            a_target: float | None = None) -> bool:
    """返回 True 表示本帧应代发 RESUME 按键帧。"""

    # 每 100 帧（1s）刷新开关状态：中途开/关 MacanStartStop 立即生效，无需重启
    if self._mp is not None and frame - self._last_refresh_frame >= 100:
      self._last_refresh_frame = frame
      try:
        self.enabled = self._platform_ok and self._mp.get_bool("MacanStartStop")
        self._distance_enabled = self._mp.get_bool("MacanStartStopDistance")
      except Exception:
        pass

    if not self.enabled:
      return False

    if not CC.enabled:
      return False

    # 驾驶员干预时绝不代发（踩油门/刹车归驾驶员控制）
    if CS.out.gasPressed or CS.out.brakePressed:
      self.resume_frames_sent = 0
      self.confirm_frames = 0
      return False

    # 车未静止（行驶中）不触发
    if not CS.out.standstill:
      self.resume_frames_sent = 0
      self.confirm_frames = 0
      return False

    # 挡位限制：不在前进挡（D/S/M）不代发 RESUME——点火静止时车辆在 P/N 挡，
    # SnG 的 aTarget 判定来自未稳定的 planner（冷启动噪声），误代发 RESUME 会让
    # 原厂 ACC 提前激活 → 与 OP 状态冲突 → controlsMismatch（00000050 实锤：
    # LS_01 帧 ~7.0s 与 mismatch 7.1s 时间重叠）。仅前进挡才允许起步跟停。
    if CS.out.gearShifter not in (structs.CarState.GearShifter.drive,
                                  structs.CarState.GearShifter.sport,
                                  structs.CarState.GearShifter.manumatic):
      self.resume_frames_sent = 0
      self.confirm_frames = 0
      return False

    # 原厂ACC激活确认（2026-08-22）：仅原厂ACC处于active控制（bus2 ACC_05 st=3，
    # carstate.acc05_stock_status）才代发 RESUME。刚上车/ACC未激活（st=2待机）时若OP代发
    # RESUME，原厂ACC状态不匹配→报错（控制项不匹配；之前挡位限制只堵了P挡点火，D挡未激活
    # 仍会误发）。停车保持时原厂st=3（ACC_AKTIV_regelt），正常SnG不受影响；st≠3不代发。
    if getattr(CS, 'acc05_stock_status', 0) != 3:
      self.resume_frames_sent = 0
      self.confirm_frames = 0
      return False

    # 起步目标确认（由 MacanStartStopDistance 开关控制）：
    # - 开（默认）：需 原厂雷达有距离(ab>0) 或 视觉确认前车>5m 才起步——防静止误起步
    # - 关：V1 纯意图起步（仅 aTarget>0.15+5帧确认，无距离条件）——拥堵路段保持紧凑
    #   跟车、防加塞（用户2026-08-22需求）。大前提（挡位/无油门刹车/st==3）仍须满足。
    # 说明：v3"必须雷达ab>0"曾收紧此条件，但 00000053 seg7 实测 OP 未代发 RESUME 仍
    # st=6，证明 st=6 主因是 ACC_02 Prim_Anz 不一致，与起步距离条件无关——故改开关可调。
    if self._distance_enabled:
      radar_dist = getattr(CS, 'stock_lead_distance', 0)
      vis_dist = getattr(CS, 'op_lead_dRel', 0.0)
      if not (radar_dist > 0 or vis_dist > 5.0):
        self.resume_frames_sent = 0
        self.confirm_frames = 0
        return False

    # OP 判定可起步：优先用 planner 原始 aTarget（经 CC_SP.params 传入）而非
    # CC.actuators.accel——LoC 在停车保持态（原厂 cruise_standstill=True）
    # 卡在 stopping 状态，输出恒 ≤0（0000004d 实测 aTarget 0.21-0.45 但
    # accel=0 → 5 次长停全未自动起步）。aTarget 是真实起步意图，不受
    # LoC 状态机压制。0.15 + 5帧确认避免红灯/前车未动误触发。
    target = a_target if a_target is not None else CC.actuators.accel
    if target <= _RESUME_ACCEL_THRESHOLD:
      self.confirm_frames = 0
      self.resume_frames_sent = 0
      return False

    self.confirm_frames += 1
    if self.confirm_frames < _RESUME_CONFIRM_FRAMES:
      return False

    # 防抖：连续发送有上限（RESUME 是瞬时按键，过长可能被原厂当长按）
    if self.resume_frames_sent >= _RESUME_MAX_FRAMES:
      return False

    self.resume_frames_sent += 1
    return True

  def create_stop_and_go(self, CCS, packer, bus, CC: structs.CarControl, CS: CarStateBase, frame: int,
                         resume_ready: bool | None = None) -> list[CanData]:
    """resume_ready: carcontroller 已在帧首调用 update_stop_and_go 时传入结果，
    避免重复调用导致 resume_frames_sent 双倍计数。"""
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update_stop_and_go(CC, CS, frame) if resume_ready is None else resume_ready
    if send_resume:
      can_sends.append(CCS.create_acc_buttons_control(packer, bus, CS.gra_stock_values, resume=True))

    return can_sends
