"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.volkswagen.values_ext import VolkswagenFlagsSP

# OP 判定可起步的加速度阈值（m/s²）。停车保持态时 OP 的 aTarget 通常≈0，
# 前车起步/绿灯时视觉模型输出正加速度请求（00000047 seg29 实测：起步瞬间 aTarget 从
# +0.01 → +0.60）。accel>0.3 表示 OP 明确要正加速——触发代发 RESUME 解除原厂停车保持。
_RESUME_ACCEL_THRESHOLD = 0.3
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
    self.enabled = bool(CP_SP.flags & VolkswagenFlagsSP.STOP_AND_GO)

    self.last_standstill_frame = 0
    self.resume_frames_sent = 0
    self.prev_close_distance = 0.0

  def update_stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> bool:
    """返回 True 表示本帧应代发 RESUME 按键帧。"""

    if not self.enabled:
      return False

    if not CC.enabled:
      return False

    # 驾驶员干预时绝不代发（踩油门/刹车归驾驶员控制）
    if CS.out.gasPressed or CS.out.brakePressed:
      self.resume_frames_sent = 0
      return False

    # 车未静止（行驶中）不触发
    if not CS.out.standstill:
      self.resume_frames_sent = 0
      return False

    # OP 判定可起步：planner 请求正加速度（视觉模型看到前车起步/绿灯）
    # 注意：停车保持态时 OP 的 aTarget≈0（00000047 seg29 实测），只有模型
    # 明确放行起步时才 >0.3——避免在红灯/前车未动时误触发。
    if CC.actuators.accel <= _RESUME_ACCEL_THRESHOLD:
      self.resume_frames_sent = 0
      return False

    # 防抖：连续发送有上限（RESUME 是瞬时按键，过长可能被原厂当长按）
    if self.resume_frames_sent >= _RESUME_MAX_FRAMES:
      return False

    self.resume_frames_sent += 1
    return True

  def create_stop_and_go(self, CCS, packer, bus, CC: structs.CarControl, CS: CarStateBase, frame: int) -> list[CanData]:
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update_stop_and_go(CC, CS, frame)
    if send_resume:
      can_sends.append(CCS.create_acc_buttons_control(packer, bus, CS.gra_stock_values, resume=True))

    return can_sends
