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
# RESUME 脉冲整形（2026-08-31，方案A）：OP 代发模拟"人按一下 RESUME"的干净单脉冲。
# 人为按键实测（0002 seg6 899.8s / seg14 1311.9s）：LS_Tip_Wiederaufnahme=1 持续 160-180ms。
# 旧实现按 aTarget 条件逐帧发送：前车走走停停时视觉 aTarget 抖动（0.15 附近反复）→
# confirm/resume 计数被反复清零 → LS_01 断续成簇（90-100ms×N，63/65 实测多脉冲）→
# 原厂 ACC 上升沿检测把每个 0→1 都当一次 RESUME → 起步状态机反复触发（SnG st6 嫌疑根因）。
# 修复：5帧确认后锁定发送一个 180ms 连续脉冲（无视 aTarget 抖动），结束进 3s 冷却
# （vEgo>0.5 车动立即解除）——一次起步意图 = 一次干净脉冲。
_RESUME_PULSE_FRAMES = 18       # 脉冲总长度：180ms @100Hz 控制帧率（对齐人为按键 160-180ms）
_RESUME_COOLDOWN_FRAMES = 300   # 冷却：3s 内不重发（人不会 3 秒内按两次 RESUME）


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
    # 起步安全距离（MacanStartStopDistance，INT 米；0=Off/V1纯意图起步，3~10=需前车距离>阈值）：
    # 开（>0）时：需 原厂雷达距离(ab) 或 视觉前车距离 换算后 > 阈值才起步（防误起步）。
    # 关（0）：V1 纯意图起步（仅 aTarget>0.15+5帧确认）——拥堵防加塞。仅 SnG 开启时生效。
    self._distance_m = 5  # 默认 5 米（与历史 v2 视觉>5m 行为一致）
    self._mp = None
    try:
      from openpilot.common.params import Params
      self._mp = Params()
      self.enabled = self._platform_ok and self._mp.get_bool("MacanStartStop")
      self._distance_m = int(self._mp.get("MacanStartStopDistance") or 5)
    except Exception:
      pass  # opendbc 测试环境无 openpilot 包：保持 flags 判断

    self._last_refresh_frame = -100  # 首次调用立即刷新
    self.last_standstill_frame = 0
    self.resume_frames_sent = 0
    self.confirm_frames = 0
    self._pulse_frames_left = 0     # 脉冲锁定剩余帧（>0=发送中，无视 aTarget 抖动）
    self._cooldown_frames_left = 0  # 冷却剩余帧（防连续短脉冲成簇）
    self.prev_close_distance = 0.0

  def update_stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int,
                            a_target: float | None = None) -> bool:
    """返回 True 表示本帧应代发 RESUME 按键帧。"""

    # 每 100 帧（1s）刷新开关状态：中途开/关 MacanStartStop 立即生效，无需重启
    if self._mp is not None and frame - self._last_refresh_frame >= 100:
      self._last_refresh_frame = frame
      try:
        self.enabled = self._platform_ok and self._mp.get_bool("MacanStartStop")
        self._distance_m = int(self._mp.get("MacanStartStopDistance") or 5)
      except Exception:
        pass

    if not self.enabled:
      return False

    if not CC.enabled:
      self._pulse_frames_left = 0
      self._cooldown_frames_left = 0
      return False

    # 驾驶员干预时绝不代发（踩油门/刹车归驾驶员控制）
    if CS.out.gasPressed or CS.out.brakePressed:
      self.resume_frames_sent = 0
      self.confirm_frames = 0
      self._pulse_frames_left = 0
      self._cooldown_frames_left = 0
      return False

    # ---- RESUME 脉冲锁定（方案A 2026-08-31）：触发后无视 aTarget 抖动，发满一个
    # 180ms 单脉冲（对齐人为按键）。一次起步意图 = 一次干净脉冲，杜绝"多短脉冲
    # 成簇 → 原厂 ACC 上升沿检测当成多次 RESUME"（63/65 实测根因）。----
    if self._pulse_frames_left > 0:
      self._pulse_frames_left -= 1
      self.resume_frames_sent += 1
      if CS.out.vEgo > _RESUME_VEGO_RESET:
        # 车已动（vEgo>0.5）= 原厂放行起步成功 → 脉冲提前结束，剩余帧不再发
        self._pulse_frames_left = 0
        return False
      if self._pulse_frames_left == 0:
        # 脉冲自然结束但车没动：进冷却，等原厂响应/超时，防止立即重触发
        self._cooldown_frames_left = _RESUME_COOLDOWN_FRAMES
      return True

    # 冷却期：不重发；车已动（vEgo>0.5）立即解除冷却，允许下一次 SnG
    if self._cooldown_frames_left > 0:
      self._cooldown_frames_left -= 1
      if CS.out.vEgo > _RESUME_VEGO_RESET:
        self._cooldown_frames_left = 0
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

    # 起步目标确认（MacanStartStopDistance 可调车距，米）：
    # - 0=Off：V1 纯意图起步（仅 aTarget>0.15+5帧确认，无距离条件）——拥堵防加塞
    # - 3~10米：需 原厂雷达距离(ab) 或 视觉前车距离 换算后 > 阈值才起步——防误起步
    # 大前提（挡位/无油门刹车/st==3）仍须满足。说明：v3"必须雷达ab>0"曾收紧此条件，但
    # 00000053 seg7 实测 OP 未代发 RESUME 仍 st=6（主因是 ACC_02 Prim_Anz 不一致），
    # 与起步距离条件无关——故改为可调车距（用户2026-08-22需求：tizi 0/3-10米、mici 3/5/10）。
    if self._distance_m > 0:
      radar_dist = getattr(CS, 'stock_lead_distance', 0)  # 原厂 ACC_Abstandsindex（0-1021 索引）
      vis_dist = getattr(CS, 'op_lead_dRel', 0.0)          # 视觉前车距离（米）
      # ab→米换算：实车标定 ab250≈10.6m → 0.0424 m/ab（00000004/0052 路试配对数据）
      radar_ok = radar_dist * 0.0424 > self._distance_m
      vis_ok = vis_dist > self._distance_m
      if not (radar_ok or vis_ok):
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

    # 确认满5帧 → 启动 180ms 锁定脉冲（本帧已发出第1帧；后续帧由脉冲锁定分支接管）
    self._pulse_frames_left = _RESUME_PULSE_FRAMES - 1
    self.confirm_frames = 0
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


class StartupGapSyncCarController:
  """Macan (MLB) 开机距离档位同步：

  原厂 ACC ECU 每次点火把内部距离档重置为 3 格，而 OP 的驾驶风格带记忆
  （LongitudinalPersonality：从容=4格/标准=3格/激进=1格）。OP 代发 ACC_02 时
  仪表显示由 CS.stock_zeitluecke 驱动（已与记忆同步），但原厂 ACC 内部仍是 3 格
  ——OP 退出/降级时原厂将以 3 格（标准）接管，与用户记忆不符。

  本模块在停车+ACC 待机（主开关 ON 未激活）时，代发 LS_01 距离键脉冲到
  CAN.ext（bus2 雷达侧），让原厂内部档位与记忆对齐。仅在停车时发送
  （行驶/激活中禁止——会改变实际跟车距离）。每次 onroad 会话只同步一次。
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self._platform_ok = (CP.brand == "volkswagen" and CP.carFingerprint == "PORSCHE_MACAN_MK1")
    self.enabled = False
    self._mp = None
    try:
      from openpilot.common.params import Params
      self._mp = Params()
      self.enabled = self._platform_ok and self._mp.get_bool("MacanStartupGapSync")
    except Exception:
      pass  # opendbc 测试环境无 openpilot 包：保持关闭

    # 同步状态机（实例随 onroad 会话重建，天然每次点火重置）
    self._synced = False       # 本次会话已完成同步
    self._pending = 0          # 剩余待发脉冲数
    self._send_increase = False  # True=拉远(+1格)/False=拉近(-1格)
    self._last_send_frame = -30

  def create_startup_gap_sync(self, CCS, packer, bus, CS: CarStateBase, frame: int) -> list[CanData]:
    """返回应代发的 LS_01 距离键帧（可能为空）。仅停车+待机时发送。"""
    can_sends = []
    if not self.enabled or self._synced:
      return can_sends

    # 安全硬条件：停车 + ACC 待机（主开关 ON 且未激活）。
    # 点火初期/行驶中/ACC 激活中一律不触发（行驶中改档会改变跟车距离）。
    if not CS.out.standstill or not CS.out.cruiseState.available or CS.out.cruiseState.enabled:
      return can_sends

    # 首次满足条件：计算目标档位（与 selfdrived._zeitluecke / carstate.stock_zeitluecke 同源）
    if self._pending == 0:
      personality = 1
      if self._mp is not None:
        try:
          personality = self._mp.get("LongitudinalPersonality", return_default=True)
        except Exception:
          pass
      target = {2: 4, 1: 3, 0: 1}.get(personality, 3)
      delta = target - 3  # 原厂点火默认 3 格
      if delta == 0:
        self._synced = True  # 记忆即默认，无需同步
        return can_sends
      self._pending = abs(delta)
      self._send_increase = delta > 0

    # 脉冲间隔 ~300ms（控制帧率 100Hz → 30 帧）。距离键是瞬时按键，
    # 间隔发送模拟用户多次按键（3→1 需按 2 次拉近；3→4 按 1 次拉远）。
    if frame - self._last_send_frame < 30:
      return can_sends

    can_sends.append(CCS.create_acc_buttons_control(packer, bus, CS.gra_stock_values,
                                                    distance_increase=self._send_increase,
                                                    distance_decrease=not self._send_increase))
    self._pending -= 1
    self._last_send_frame = frame
    if self._pending <= 0:
      self._synced = True
    return can_sends


class VcruiseSyncCarController:
  """Macan (MLB) 巡航速度自动同步（MacanVcruiseSync，开关开=OP主动同步）：

  背景：Macan 为 openpilotLongitudinalControl（OP 自维护 vCruise）+ 原厂 ACC 雷达并行。
  原厂 ACC 内部有一套自己的巡航设定（ACC_02.Wunschgeschw_02），OP 建立起自己的
  vCruise 与之独立，两者可漂移（超驰+SET 锚定、长按步长不一致是主要来源）。

  本模块在 OP 巡航激活 + 原厂 ACC 激活(st=3/4)时，若 |OP_vCruise - 原厂Wunsch| > 1 km/h，
  代发 LS_01 按键脉冲（SET+/SET-）到 CAN.ext(bus2 雷达侧)，让原厂内部设定向 OP 逼近，
  直到两者在 ±1 km/h 内一致。读回依据=carstate 的 stock_wunschgeschw（ACC_02 原厂值）。

  防死锁（2026-09-07 设计）：按键窗口按 20→50→80ms 递增，每个窗口层连续 3 次
  判定"原厂 Wunsch 未向正确方向变化"即升一档；80ms 仍失败则放弃本次 + 冷却 5s，
  防止"原厂不认 20ms 单帧按键 → 发了也没用而无限循环"。
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self._platform_ok = (CP.brand == "volkswagen" and CP.carFingerprint == "PORSCHE_MACAN_MK1")
    self.enabled = False
    self._mp = None
    try:
      from openpilot.common.params import Params
      self._mp = Params()
      self.enabled = self._platform_ok and self._mp.get_bool("MacanVcruiseSync")
    except Exception:
      pass  # opendbc 测试环境无 openpilot 包：保持关闭

    # 按键窗口（帧数）：控制帧率 ~100Hz(10ms) → 20ms≈2帧 / 50ms≈5帧 / 80ms≈8帧
    # 用帧数表示"按住时长"（一次完整按键 = 连续 N 帧置位 + 至少 1 帧释放）
    self.windows_frames = [2, 5, 8]      # 20/50/80ms 三档
    self.window_idx = 0                   # 当前档位
    self.fail_count = 0                   # 当前档位连续失败次数
    self.hold_remaining = 0               # 当前按键还剩余置位帧数
    self.need_increase = 0                # 本批脉冲方向：1=需+ / -1=需-
    self.stock_at_burst = None            # 本批按键起点的原厂 Wunsch（回读判据）
    self.cooldown_until = 0               # 放弃后冷却到该帧（防反复）
    self.gave_up = False

  def create_vcruise_sync(self, CCS, packer, bus, CS: CarStateBase, frame: int) -> list[CanData]:
    """返回应代发的 LS_01 按键帧（可能为空）。仅 OP 与原厂 ACC 速度设定不一致时发送。"""
    can_sends = []
    if not self.enabled:
      return can_sends

    # 冷却结束后允许重试（重置放弃标志）
    if self.gave_up:
      if frame < self.cooldown_until:
        return can_sends
      self.gave_up = False
      self.window_idx = 0
      self.fail_count = 0

    # 只在 OP 纵向激活 + 原厂 ACC 激活（st=3/4）时同步。非激活/停车/待机不做——
    # 速度设定同步仅在原厂接管速度时才有意义。
    stock_status = getattr(CS, 'acc05_stock_status', 3)
    if stock_status not in (3, 4) or not CS.out.cruiseState.enabled:
      return can_sends

    op_cruise = float(getattr(CS, 'vCruise', 0.0))       # OP 巡航（kph）
    stock = float(getattr(CS, 'stock_wunschgeschw', 0.0))  # 原厂 Wunsch（kph）
    if op_cruise <= 0 or stock <= 0:
      return can_sends

    delta = op_cruise - stock

    # 正在持续一段按键置位：继续按住（不新发起、不判回读）
    if self.hold_remaining > 0:
      self.hold_remaining -= 1
      can_sends.append(CCS.create_acc_buttons_control(
        packer, bus, CS.gra_stock_values,
        set_increase=(self.need_increase > 0),
        set_decrease=(self.need_increase < 0)))
      return can_sends

    # 上一批按键已释放：回读判据
    if self.stock_at_burst is not None:
      moved = (stock - self.stock_at_burst) * (1 if self.need_increase > 0 else -1)
      if moved > 0:
        # 原厂 Wunsch 已向 OP 方向逼近 → 成功，保持最快档
        self.window_idx = 0
        self.fail_count = 0
      else:
        # 本次按键没生效 → 计数；连续 3 次升档，80ms 仍失败则放弃+冷却
        self.fail_count += 1
        if self.fail_count >= 3:
          if self.window_idx < len(self.windows_frames) - 1:
            self.window_idx += 1
            self.fail_count = 0
          else:
            self.gave_up = True
            self.cooldown_until = frame + 500  # 停 5s 防反复
            self.stock_at_burst = None
            return can_sends
      self.stock_at_burst = None

    # 重新计算（可能已被上批修正）：在 ±1 kph 内 = 同步完成
    delta = op_cruise - float(getattr(CS, 'stock_wunschgeschw', 0.0))
    if abs(delta) <= 1.0:
      return can_sends

    # 发一批新按键：指向 OP 方向
    self.need_increase = 1 if delta > 0 else -1
    self.hold_remaining = self.windows_frames[self.window_idx]
    self.stock_at_burst = float(getattr(CS, 'stock_wunschgeschw', 0.0))
    self.hold_remaining -= 1
    can_sends.append(CCS.create_acc_buttons_control(
      packer, bus, CS.gra_stock_values,
      set_increase=(self.need_increase > 0),
      set_decrease=(self.need_increase < 0)))
    return can_sends
