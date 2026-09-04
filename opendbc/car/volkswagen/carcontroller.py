import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from opendbc.sunnypilot.car.volkswagen.stop_and_go import SnGCarController, StartupGapSyncCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


# 全量标定 v4（2026-09-02, 223段/100055样本, 留出验证中位误差9.74%）
# bus2(原厂ACC) Abstandsindex × 视觉dRel 配对; T单调化用于 t→idx 反向插值
_MACAN_ABSTAND_IDX = [27, 32, 37, 42, 47, 52, 57, 62, 67, 72, 77, 82, 87, 92, 97, 102, 107, 112, 117, 122, 127, 132, 137, 142, 147, 152, 157, 162, 167, 172, 177, 182, 187, 192, 197, 202, 207, 212, 217, 222, 227, 232, 237, 242, 247, 252, 257, 262, 267, 272, 277, 282, 287, 292, 297, 302, 307, 312, 317, 322, 327, 332, 337, 342, 347, 352, 357, 362, 367, 372, 377, 382, 387, 392, 397, 402, 407, 412, 417, 422, 427, 432, 437, 442, 447, 452, 457, 462, 467, 472, 477, 482, 487, 492, 497, 502, 507, 512, 517, 522, 527, 532, 537, 542, 547, 552, 557, 562, 567, 572, 577, 582, 587, 592, 597, 602, 607, 612, 617, 622, 627, 632, 637, 642, 647, 652, 657, 662, 667, 672, 677, 682, 687, 692, 697, 702, 707, 712, 717, 722, 727, 732, 737, 742, 747, 752, 757, 762, 767, 772, 777, 780, 1021]
_MACAN_ABSTAND_T_MONO = [np.float64(0.81), np.float64(0.81), np.float64(0.81), np.float64(0.81), np.float64(0.81), np.float64(0.81), np.float64(0.81), np.float64(0.822), np.float64(0.837), np.float64(0.872), np.float64(0.942), np.float64(0.942), np.float64(0.942), np.float64(0.978), np.float64(1.025), np.float64(1.14), np.float64(1.168), np.float64(1.185), np.float64(1.203), np.float64(1.275), np.float64(1.288), np.float64(1.364), np.float64(1.422), np.float64(1.439), np.float64(1.459), np.float64(1.505), np.float64(1.571), np.float64(1.616), np.float64(1.682), np.float64(1.708), np.float64(1.766), np.float64(1.801), np.float64(1.828), np.float64(1.869), np.float64(1.933), np.float64(2.0), np.float64(2.028), np.float64(2.091), np.float64(2.12), np.float64(2.17), np.float64(2.246), np.float64(2.246), np.float64(2.351), np.float64(2.353), np.float64(2.394), np.float64(2.435), np.float64(2.467), np.float64(2.497), np.float64(2.553), np.float64(2.635), np.float64(2.653), np.float64(2.711), np.float64(2.774), np.float64(2.823), np.float64(2.923), np.float64(2.968), np.float64(3.031), np.float64(3.075), np.float64(3.121), np.float64(3.128), np.float64(3.197), np.float64(3.263), np.float64(3.263), np.float64(3.263), np.float64(3.299), np.float64(3.398), np.float64(3.472), np.float64(3.472), np.float64(3.472), np.float64(3.531), np.float64(3.619), np.float64(3.628), np.float64(3.684), np.float64(3.826), np.float64(3.826), np.float64(3.826), np.float64(3.884), np.float64(3.925), np.float64(3.955), np.float64(3.955), np.float64(4.074), np.float64(4.074), np.float64(4.074), np.float64(4.074), np.float64(4.074), np.float64(4.154), np.float64(4.193), np.float64(4.216), np.float64(4.406), np.float64(4.453), np.float64(4.453), np.float64(4.453), np.float64(4.562), np.float64(4.562), np.float64(4.562), np.float64(4.587), np.float64(4.71), np.float64(4.802), np.float64(4.802), np.float64(4.802), np.float64(4.835), np.float64(4.974), np.float64(4.974), np.float64(4.974), np.float64(4.974), np.float64(5.048), np.float64(5.048), np.float64(5.052), np.float64(5.098), np.float64(5.098), np.float64(5.237), np.float64(5.364), np.float64(5.606), np.float64(5.606), np.float64(5.606), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.782), np.float64(5.787), np.float64(5.909), np.float64(5.909), np.float64(6.202), np.float64(6.331), np.float64(6.331), np.float64(6.331), np.float64(6.331), np.float64(6.331), np.float64(6.331), np.float64(6.331), np.float64(6.458), np.float64(6.458), np.float64(6.458), np.float64(6.59), np.float64(6.59), np.float64(6.59), np.float64(6.59), np.float64(6.59), np.float64(6.59), np.float64(6.94), np.float64(6.94), np.float64(6.94), np.float64(6.94), np.float64(6.94), np.float64(7.149), np.float64(7.149), np.float64(7.149), np.float64(7.149)]

class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ
    # Macan 坡度补偿/转向系数开关（重启生效；opendbc 测试环境无 openpilot 包时安全降级 False）
    try:
      from openpilot.common.params import Params
      self._mp = Params()
      self.slope_comp = self._mp.get_bool("MacanSlopeComp")
      self.slope_comp_unlimited = self._mp.get_bool("MacanSlopeCompUnlimited")
      # 2026-08-29: 65-5/65-11 实锤修复（本轮默认开，开一版试一版）
      # Params key 未注册 → 默认开启；注册后按注册值（后续做成正式开关）
      try:
        self.macan_axg_comp = self._mp.get_bool("MacanAxGComp")       # SnG起步axG跳过无效区(0.15)
      except Exception:
        self.macan_axg_comp = True
      try:
        self.macan_verz_follow = self._mp.get_bool("MacanVerzFollow") # 踩油门跟随原厂减速verz
      except Exception:
        self.macan_verz_follow = True
    except Exception:
      self.slope_comp = False
      self.slope_comp_unlimited = False

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.accel_last = 0.
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.gra_acc_counter_last = None
    # 跟停保持（00000039 seg7 实锤）：OP 的 stopping 状态在停稳后偶发掉 0 导致
    # ACC_Anhalten 抖动，原厂 anh 全程保持。进入 stopping 且 vEgo≈0 后保持 anh，
    # 直到起步（vEgo>0.5）或驾驶员刹车才释放。
    self.stopping_hold = False
    # 车距显示 hold（00000041 问题3）：视觉补位 leadOne.present 断续（0.5-1s）导致仪表
    # 前车图标闪烁。一旦视觉补位生效，保持显示 2s（HUD 10Hz ≈ 20 帧），视觉目标短暂
    # 丢失时图标不闪；连续丢失超过 2s 才清除（对齐原厂"main 开+有障碍物即显示"行为）。
    self.lead_hold_expire = 0       # 单调时钟纳秒
    self.lead_hold_distance = 0
    # 仪表距离显示平滑（2026-08-25）：迟滞防源切换抖动 + 变化率限速防跳变。
    # 依据（0058 前5段 357k 帧）：融合后帧间 |Δabstand| p90=801cm vs 视觉 p90=398cm
    # ——A2 硬切换台阶直接透传到仪表；仅动显示路径，控制路径（MPC 卡尔曼）不受影响。
    self.disp_abstand = None        # 当前显示的 abstand（原厂单位 1-1021）
    self.disp_src_radar = False     # 当前显示源是否雷达（迟滞状态）
    # SnG loes（起步确认）窗口保持（2026-08-21）：RESUME 代发上升沿起 loes=1 保持 0.6s，
    # 不依赖 standstill（车动即停会截断确认窗口）。00000049 原厂踩油门实测 400-520ms，
    # 0051 段19 SnG 仅 160ms → 原厂起步确认不足 → 撤力退出（cruiseMismatch）。
    self.gap_sync = StartupGapSyncCarController(CP, CP_SP)  # 开机距离档同步
    self.sng_loes_until = 0          # 单调时钟纳秒
    self.sng_loes_start = 0          # loes 窗口起点（事件化回收基准）
    self.sng_resume_ready_last = False
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0

    # EPS timer reset workaround for MLB platforms (Porsche Macan, Audi, etc.)
    # MQB racks reset the timer after a single frame of HCA disabled.
    # MLB racks need > 1 second to reset; we try to reset when
    # engaged for a long time and torque output is currently low.
    self.eps_timer_workaround = bool(CP.flags & VolkswagenFlags.MLB)
    self.hca_frame_timer_resetting = 0
    self.hca_frame_low_torque = 0

  @staticmethod
  def op_lead_to_index(drel, vego):
    """OP 前车距离 → 原厂 ACC_Abstandsindex（1-1021）。全量标定 v4（2026-09-02，
    223段/100055样本，留出验证 9.74%）：bus2(原厂)idx × 视觉dRel 配对，反向插值
    用单调化时距表 _MACAN_ABSTAND_T_MONO。低速（vEgo<5m/s）等效距离兜底。"""
    t = drel / vego if vego > 5.0 else drel / 5.0
    return int(np.interp(t, _MACAN_ABSTAND_T_MONO, _MACAN_ABSTAND_IDX))

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    # SnG 起步判定提前到帧首（供 ACC 段释放 stopping_hold）：OP 停车保持态持续代发
    # ACC_05 anh=1，会覆盖用户/代发的 RESUME 按键（0000004c+用户实测 2026-08-17：
    # 停车按 SET/RESUME 无效，只有轻踩油门 gasPressed 才释放 anh）。SnG 判定
    # 可起步时同步释放 stopping_hold，代发 RESUME 才能生效。
    # SnG 判定输入用 planner 原始 aTarget（controlsd_ext 经 CC_SP.params 传入）
    # ——LoC 在停车保持态压 accel≤0，CC.actuators.accel 看不到正信号（0000004d 实测）。
    a_target = None
    self.slope_pct = 0.0
    self.slope_oem_filtered = 0.0  # 原厂坡度低通滤波（v2 双源）
    for _p in CC_SP.params:
      _k = _p.get("key")
      try:
        _v = _p.get("value")
        if _k == "aTarget":
          a_target = float(_v.decode() if isinstance(_v, bytes) else _v)
        elif _k == "slopePct":
          self.slope_pct = float(_v.decode() if isinstance(_v, bytes) else _v)
      except (ValueError, TypeError, AttributeError):
        if _k == "aTarget":
          a_target = None
    sng_resume_ready = self.update_stop_and_go(CC, CS, self.frame, a_target=a_target)
    # loes 窗口延长：RESUME 代发上升沿起保持 0.6s（对齐原厂踩油门起步确认窗口+余量），
    # 车动（standstill→False）不再截断 loes——原厂 ACC 无油门起步完全依赖该信号确认。
    if sng_resume_ready and not self.sng_resume_ready_last:
      self.sng_loes_start = now_nanos
      self.sng_loes_until = now_nanos + 600_000_000
    self.sng_resume_ready_last = sng_resume_ready

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continuously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # compensate the gap between measured and current curvature
          apply_curvature = actuators.curvature + (CS.curvature_meas - CC.currentCurvature)
          apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                   CC.latActive, self.CCP.STEER_STEP)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                     [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

        else:
          if self.steering_power_last > 0:  # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = float(np.clip(CS.curvature_meas, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX))
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.  # inactive curvature
            steering_power = 0

        can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.
        # MLB racks need > 1 second to reset; try to reset if engaged for a long
        # time and torque output is currently low. Resets are aborted early if
        # torque demand rises. Long resets, completed or not, need apply_torque
        # reset to 0 on exit due to rate limit safety.

        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1, -1)[apply_torque < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_torque) > 0

          # EPS timer reset workaround for MLB platforms
          if self.eps_timer_workaround and self.hca_frame_timer_running >= self.CCP.STEER_TIME_BM / DT_CTRL:
            if abs(apply_torque) <= self.CCP.STEER_LOW_TORQUE:
              self.hca_frame_low_torque += self.CCP.STEER_STEP
              if self.hca_frame_low_torque >= self.CCP.STEER_TIME_LOW_TORQUE / DT_CTRL:
                hca_enabled = False
            else:
              self.hca_frame_low_torque = 0
              if self.hca_frame_timer_resetting > 0:
                apply_torque = 0
        else:
          self.hca_frame_low_torque = 0
          hca_enabled = False
          apply_torque = 0

        if hca_enabled:
          output_torque = apply_torque
          self.hca_frame_timer_resetting = 0
        else:
          output_torque = 0
          self.hca_frame_timer_resetting += self.CCP.STEER_STEP
          if self.hca_frame_timer_resetting >= self.CCP.STEER_TIME_RESET / DT_CTRL or not self.eps_timer_workaround:
            self.hca_frame_timer_running = 0
            apply_torque = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, output_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))
    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel hands-on message to keep ACC resume active and control Emergency Assist
      # MEB Emergency Assist brake jerks after 30s of continued hands-off time.
      # We send the stock wheeltouch message to start the stock DM timer when openpilot latches the critical driver monitoring alert
      if self.frame % self.CCP.KLR_01_STEP == 0:
        lat_active = CC.latActive and not CC.driverMonitoringEscalation
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, lat_active, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, lat_active, CS.klr_stock_values))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      # 原厂超驰踏板阈值（MLB Macan）：踏板位置>5% 才确认超驰。原厂 st=4 切换实测阈值 5.6-7.6%
      # （00000002 1781.279/2043.582），快踩/慢踩都等踏板爬过阈值；OP 用 gasPressed(>0) 在 1.2%
      # 就切 -> 「OP已超驰 vs 原厂未确认」矛盾窗口 -> st=6（00000056 682.109 实锤）。
      # 非 MLB 平台（无 pedal_value）回退 gasPressed 布尔，行为不变。
      pedal_value = getattr(CS, 'pedal_value', None)
      gas_override_stock = (pedal_value > 5.0) if pedal_value is not None else CS.out.gasPressed
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        stopping = actuators.longControlState == LongCtrlState.stopping

        if self.CP.flags & VolkswagenFlags.MEB:
          # only send ACC_HMS_RELEASE when in cruise standstill and want to resume
          starting = actuators.longControlState == LongCtrlState.pid and CS.esp_hold_confirmation
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)

          long_override = CC.cruiseControl.override or CS.out.gasPressed
          self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
          long_override_begin = long_override and self.long_override_counter < 5

          self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
          long_disabling = not CC.enabled and self.long_disabled_counter < 5

          acc_control = mebcan.get_acc_control(CS.out, CC, long_override)
          acc_hold_type = mebcan.get_acc_hold_type(CS.out, CC, starting, stopping,
                                                   CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling)
          can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CCP, CS.acc_type, CC.enabled,
                                                           accel, acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                           CS.out.vEgoRaw * CV.MS_TO_KPH, long_override, CS.travel_assist_available))
          self.accel_last = accel

        else:
          # 刹车优先：踩下刹车立即切 standby 并清空力矩，消除「刹车+ACC激活」矛盾窗口
          # （ECU 检测到刹车踏板=1 且 ACC Status=3 同时出现会写 DTC 锁死 ACC）
          brake_override = CS.out.brakePressed  # master-c3 CarState 无 brake 力度字段，仅 brakePressed 开关量
          # 油门超驰：long_active 保持（不排除 gas）——acc_control_value 在 long_active 分支内
          # 处理 gas（4 if gas else 3）。若在此排除 gasPressed，long_active 变 False 会导致
          # acc_control_value 掉到 main_switch_on→2（待机），踩油门发 st=2 而非 st=4，
          # ECU 看到「激活(3)→待机(2)跳变+油门」会锁死 ACC。原厂行为：激活中踩油门 st 3→4。
          long_active = CC.longActive and not brake_override
          # 原厂状态同步（00000033 根因修复）：OP 激活期间若原厂 ACC_05 已退出（st∉(3,4)），
          # 强制 acc_control 回待机(2)——消除「OP st=3 + 原厂已撤力矩退出」矛盾窗口，
          # 防止 ECU 检测到状态矛盾写 DTC 锁死 ACC/PAS（需两次点火清除）。
          # 正常激活时原厂 src=2 st=3（实测一致），仅在原厂雷达撤力/退出时触发。
          if long_active and getattr(CS, 'acc05_stock_status', 3) not in (3, 4):
            long_active = False
          gas_override = gas_override_stock and CS.out.cruiseState.available and not brake_override
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, gas_override_stock, getattr(CS, 'acc05_stock_status', 3))
          # OVERRIDE(4) 时保持巡航力矩（原厂 st=4 力矩≈st=3，仅状态字切 4）；accel=0 → 巡航基线。
          # 力矩许可只跟 long_active（激活态），不跟 gas_override（00000004 seg2 实锤：待机+踩油门
          # 原厂 ACC_05 全程 st=2/fm=0/mom=0，ACC 不发力矩，动力完全由驾驶员油门主导；
          # 00000041 seg2@175.41s 旧代码 torque_active=long_active or gas_override → 待机踩油门
          # 时 OP 代发 fm=1/mom=27（巡航基线）与原厂零力矩矛盾，状态机待机却请求力矩）。
          # 激活中踩油门 long_active 保持 True（gas 不改 long_active），力矩照发 → st=4 超驰不受影响。
          torque_active = long_active
          # 油门超驰：gas 时 accel 强制 0 → 力矩=巡航基线，驾驶员主导加速；松油门立即恢复 planner 控制
          accel = float(np.clip(0.0 if gas_override_stock else actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if torque_active else 0)
          # 原厂意图仲裁（00000037/00000038 根因修复）：OP 代发请求不能与原厂雷达矛盾。
          # 原厂在请求减速（verz<0）或停车（anh=1）时，OP 跟随原厂意图（最多同深度减速），
          # 绝不允许比原厂激进（正加速）——否则雷达自检失败报 st=6 → ECU 写 DTC 锁死 ACC。
          # 跟停后起步需等原厂先释放（anh=0 且 verz≥0），起步略延迟是安全代价。
          stock_verz = getattr(CS, 'acc05_stock_verz', 0.0)
          stock_anhalten = getattr(CS, 'acc05_stock_anhalten', False)
          stock_fv = getattr(CS, 'acc05_stock_fv', False)  # 原厂减速通道许可（00000041 seg7 实锤）
          stock_mom = getattr(CS, 'acc05_stock_mom', 1021.0)
          if torque_active and (stock_anhalten or stock_fv or stock_verz < -0.05):
            # 原厂意图跟随（00000037/38/41 根因修复）：原厂请求减速（verz<0）或开减速通道
            # （FV=1，00000041 seg7 原厂 verz 只缓降到 -0.06，旧阈值 -0.15 拦不住）时，
            # OP 最多同深度减速，绝不比原厂激进。
            # 00000044 实锤修正（2026-08-13）：原厂 verz 为正（如 +1.48）是"允许/请求加速"
            # （跟停起步原厂先释放 anh、verz 转正），旧代码 verz>=0 时压 -1.0 会反向压制原厂
            # 加速意图 → 绿灯不自动起步。现在：anh=1（原厂停车请求）压 -1.0；verz<0 跟随同深度；
            # verz>=0（原厂允许加速）不压，让 OP 正 accel 通过（跟随后续由 mpc/planner 接管）。
            if stock_anhalten:
              accel = min(accel, -1.0)
            elif stock_verz < 0:
              # min 在负值域取**更深**的一方 = OP 刹车深度永远取最深（安全设计）：
              #   原厂没识别静止目标（verz=0）→ 本分支不执行 → OP 自主深刹 ✓
              #   原厂浅刹 -0.36 / OP 想深刹 -0.5 → min(-0.5,-0.36)=-0.5 → OP 保持深刹 ✓
              #   OP 浅 / 原厂深 → OP 被压到原厂深度 ✓（刹车从不比原厂浅）
              # 绝不可改 max——原厂未识别静止车辆时 OP 会失去刹车能力（致命事故 bug）。
              # 848 案例 OP -0.42 vs 原厂 -0.36（OP 深 0.06）是预期行为（取更深），非 bug。
              accel = min(accel, stock_verz)
          # 撤力跟随（00000041 seg5/seg7 实锤补丁）：原厂力矩归零（mom<60）或切减速通道
          # （fv=1）时，OP 必须让代发帧完全镜像原厂（mom=0、FM=0、FV/verz 跟随原厂）。
          # 旧仲裁只压 accel≤0，但 mlbcan 在 accel=0 时仍发巡航基线力矩（6.3*v+15≈80），
          # 与原厂 mom=0 方向矛盾 → 雷达 st6 → TSK_04 1→0 退出 → controlsMismatch。
          # 撤力跟随条件收紧（00000042 seg3/seg6 实锤补丁）：仅原厂激活(st=3)且未跟停(anh=0)时
          # 才跟随撤力。跟停中 mom=0/FV=1 是停车保持（非撤力），踩油门后原厂切 st=4 超驰
          # 发力矩（mom70-140/FM1）更不应跟随——否则 OP 撤力 vs 原厂发力矩方向矛盾 →
          # TSK_04 2->0 退出 → 松油门后纵向已退出不加速。
          # seg5/seg7 撤力场景（st=3/anh=0）不受影响；跟停正常帧靠 stopping 逻辑输出
          # （00000042 seg3@224s 实证 OP=RADAR 一致，不依赖 stock_follow）。
          stock_status = getattr(CS, 'acc05_stock_status', 3)
          stock_follow = torque_active and stock_status == 3 and not stock_anhalten and (stock_mom < 60 or stock_fv)
          if stock_follow:
            accel = min(accel, 0.0)
          stopping = actuators.longControlState == LongCtrlState.stopping and not brake_override
          # 油门超驰（0000003f 红灯起步实锤）：驾驶员踩油门时强制释放停车请求，
          # 否则模型 shouldStop 卡1 → LCS 卡 stopping → Anh=1 → 踩油门也被按死。
          # 对齐原厂 st=4 超驰语义：油门接管时 ACC 不请求保持停车。
          if CS.out.gasPressed:
            stopping = False
            if not stock_anhalten:
              accel = max(accel, 0.1)  # 油门超驰给正力矩（0049原厂超驰 mom47-59 实测；0.1→mom≈35）
          # 跟停保持：进入 stopping 且 vEgo≈0 后保持 anh=1，防止停稳后 stopping 偶发掉 0
          # （00000039 seg7: 512.9-513.2 OP anh 掉 0 → 原厂雷达判定矛盾 st6）。
          # 起步（vEgo>0.5）或踩刹车时释放。
          # 用户按 RESUME/SET 也想手动起步——原厂保持态（anh=1）下按键被 ACC_05 覆盖
          # （0000004d 用户实测：停车按 SET/RESUME 无效，只有踩油门才起步）。
          # 检测到按键时释放 stopping_hold，原厂才能响应 RESUME 放行。
          # gra_stock_values 是原始 LS_01 报文（不经 OP 消费），MLB 按键在此。
          resume_btn = bool(CS.gra_stock_values.get("LS_Tip_Wiederaufnahme", 0)) or \
                       bool(CS.gra_stock_values.get("LS_Tip_Setzen", 0))
          # 跟停保持：进入 stopping 或（纵向激活+停稳+无油门/刹车+无起步意图）时保持 anh=1。
          # 显式条件不依赖 LoC 的 longControlState（0000004f 实锤：LoC 不卡 stopping 时
          # 旧逻辑 anh=0 → 原厂收不到保持请求 → 停车3秒后进 OVERRIDE(4) 只认踩油门
          # → SET/RESUME/SnG 全部无效。保持 anh=1 让原厂维持可恢复保持态）。
          # 2026-08-20 位定义实锤：ACC_Anhalten 真实位=62|1（Vector__XXX 原厂节点），
          # 0049 段3 实测原厂停车保持(st=3)/超驰(st=4)均发 anh=1——本逻辑与原厂一致
          # （1b4915d 以'原厂 anh=0'回退 d2c241a，其依据为 56|1 错位解析，已修正）。
          if stopping or (torque_active and CS.out.standstill and not CS.out.gasPressed and not brake_override and accel <= 0.05):
            self.stopping_hold = True
          elif self.stopping_hold and (brake_override or CS.out.gasPressed or CS.out.vEgo > 0.5 or sng_resume_ready or accel > 0.05 or resume_btn or not getattr(CS, 'acc05_stock_anhalten', False)):
            # 2026-08-23 st=6 修复：原厂已不请求停车保持（acc05_stock_anhalten=False，如
            # 前车远去/起步原厂发布 loes=1/mom=58 时）OP 也同步释放 stopping_hold——
            # 避免"原厂已起步、OP 还发刹车帧（verz=-2/anh=1）"方向矛盾 → st=6
            # （00000054 seg18 实锤）。只对齐停车保持时机，OP 的 accel/verz/mom 仍由
            # OP 视觉/planner 决定（安全，不透传原厂加速意图）。
            self.stopping_hold = False
          stopping = stopping or self.stopping_hold
          # SnG 自动起步/手动按键起步：强制解除停车保持请求+请求正力矩。
          # LoC 卡 stopping（should_stop 未及时变 False）时 stopping 仍 True → anh 还 1 →
          # 原厂等不到释放 → 锁死（0000004d 未起步区间实测：aTarget 正 0.21-0.45 但 accel≤0.135）。
          # 仅停车中+原厂已放行（未请求保持）时生效——原厂 stock_anhalten=True 时跟随原厂（安全）。
          # 2026-09-03 seg18实锤修复（起步不抢跑）：原厂还在刹(stock_verz<-0.05=它判定前车未动/
          # 保持刹未释放)时 SnG 不自动起步——上方仲裁已把 accel 压到刹，此处 max(accel,0.1) 放开
          # 会覆盖仲裁=抢跑 → 原厂方向矛盾 st6（00000070 seg18: 原厂 vz-2.0 保持中 OP 发 mom42）。
          # 等原厂先释放(verz 回正)再起步；原厂一直不切→保持→走超驰/按键安全路径(st3→st2 正常
          # 退出，永不 st6)。seg7 对照：驾驶员踩油门起步=原厂让位 st3→st2 安全。
          # 2026-09-04 B2实锤修复（雷达锁目标才自动起步）：SnG 自动起步加 bus2 原厂
          # ACC_Relevantes_Objekt==1 闸门——原厂雷达没锁到跟车目标（relobj=0，如红灯前
          # 静止车队首次接触、雷区）时 OP 不抢跑，等原厂先锁定（00000070 seg6/18 st6
          # 全 18 帧实测 relobj=0：OP la/en=True lc=pid 强驱起步而原厂 st=2 未授权力矩
          # → 越权 st6）。仅锁自动起步路径；驾驶员手动 resume/踩油门=原厂让位安全路径
          # （seg7 对照 st3→st2 永不 st6），不受此闸门限制。
          if (sng_resume_ready and getattr(CS, 'stock_lead_object', 0) == 1 or resume_btn) and CS.out.standstill and not stock_anhalten and stock_verz > -0.05:
            stopping = False
            self.stopping_hold = False
            accel = max(accel, 0.1)
          # vEgoStopping 字段在 car.capnp 与 volkswagenMqbEvo@29 ordinal 冲突被 capnp 静默忽略 → 运行时缺失。
          # getattr 兜底 2.0 m/s（≈7.2km/h 低速起步阈值，VW ACC 起步语义），避免激活后 AttributeError 崩溃。
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < getattr(self.CP, 'vEgoStopping', 2.0)) and not brake_override
          # ---- 坡度补偿 v2：原厂 ESP 纵向加速度主源 + IMU 复核（双源交叉校验）----
          slope_imu = self.slope_pct
          esp_laengs = getattr(CS, 'esp_laengsbeschl', 0.0)
          if self.slope_comp:
            # 原厂坡度：ESP 传感器总加速度 - 运动加速度 = 重力分量（车体坐标系，无需 IMU 标定）
            slope_oem = (esp_laengs - CS.out.aEgo) / 9.81 * 100.0
            # 低通滤波（ESP 100Hz，滤掉 aEgo 微分噪声）
            self.slope_oem_filtered = 0.8 * self.slope_oem_filtered + 0.2 * slope_oem
            if abs(self.slope_oem_filtered - slope_imu) < 3.0:
              slope_used = self.slope_oem_filtered  # 原厂主源（车体传感器更准）
            else:
              # 双源不一致 → 降级：取绝对值较小者（保守，防传感器故障误补偿）
              slope_used = slope_imu if abs(slope_imu) < abs(self.slope_oem_filtered) else self.slope_oem_filtered
          else:
            slope_used = slope_imu  # 开关关：保持 v1 行为（mlbcan 端 slope_comp=False 时不补偿）
          # ---- loes 事件化（2026-08-23 对齐原厂）：loes=1 只在"解除保持→起步力矩建立"窗口
          # 持续 500-620ms（原厂 route 00000002 seg14/15 实证），车动前即回 0，loes 期间 verz 恒=0。
          # SnG 起步：sng_resume_ready 上升沿起窗口（600ms 兜底），提前回收 = 车动(vEgo>0.5)
          #   或原厂 loes 已回 0 且已发≥300ms（跟原厂节奏，不再靠固定窗口）。
          # 踩油门：跟随原厂 loes（原厂发 OP 发、原厂回 OP 回）——不再跟油门持续（旧 bug seg00 28.8s）。
          stock_loes = getattr(CS, 'acc05_stock_loes', False)
          if CS.out.gasPressed:
            loes_active = stock_loes
          else:
            loes_early_release = CS.out.vEgo > 0.5 or (not stock_loes and now_nanos - self.sng_loes_start > 300_000_000)
            loes_active = sng_resume_ready or (now_nanos < self.sng_loes_until and not loes_early_release)
            # 2026-09-02 修复 loes 断续（seg27 实锤：planner accel 震荡→stopping 每帧翻转→
            # verz=-2.0/0 交替→loes=1 只在 verz>=0 帧输出→5% 占空比断续）。
            # loes 窗口期（600ms）强制 stopping=False——原厂 loes 期间 anh=0/verz 恒=0，
            # 窗口内持续输出 loes=1，不再被停车保持帧打断。
            if loes_active and now_nanos < self.sng_loes_until:
              stopping = False
          can_sends.extend(self.CCS.create_acc_accel_control(
self.packer_pt, self.CAN.pt, CS.acc_type, torque_active, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation, v_ego=CS.out.vEgo,
                                                             engine_torque=getattr(CS, 'engine_torque_output', 0),
                                                             stock_esp=getattr(CS, 'acc05_stock_esp', False),
                                                             stock_follow=stock_follow,
                                                             gas_override=CS.out.gasPressed,
                                                             stock_fv=stock_fv,
                                                             stock_mom=stock_mom,
                                                             stock_verz=getattr(CS, 'acc05_stock_verz', 0.0),
                                                             verz_follow=self.macan_verz_follow,
                                                             axg_comp=self.macan_axg_comp,
                                                             stock_axg=getattr(CS, 'acc05_stock_axg', 0.0),
                                                             stock_fm=getattr(CS, 'acc05_stock_fm', False),
                                                             stock_anhalten=getattr(CS, 'acc05_stock_anhalten', False),
                                                             slope_pct=slope_used,
                                                             slope_comp=self.slope_comp,
                                                             slope_comp_unlimited=self.slope_comp_unlimited,
                                                             sng_resume_req=loes_active))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = self.frame - self.distance_bar_frame < 400
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
          lead_distance = 8
        acc_hud_status = mebcan.get_acc_hud_status(CS.out, CC, CC.cruiseControl.override or CS.out.gasPressed)
        can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                       hud_control.leadVisible, hud_control.leadDistanceBars + 1, show_distance_bars,
                                                       CS.esp_hold_confirmation, lead_distance, 0, fcw_alert))

      else:
        lead_distance = getattr(CS, 'stock_lead_distance', 0)
        lead_object = getattr(CS, 'stock_lead_object', 0)
        op_drel = getattr(CS, 'op_lead_dRel', 0.0)
        # 仪表盘车距显示（用户设计意图）：雷达距离有效(>0且非错误值) -> 透传原厂雷达；
        # 雷达无信号(0或错误值) -> 用视觉换算补位，让仪表显示视觉识别的前车。
        # 之前"lead_object==0 就走视觉换算"的 bug：原厂雷达有效(316)时也切去视觉(243)，
        # 导致 OP 代发背离原厂（00000052 seg9、00000051 9退出点）。现改为"雷达有效才透传，
        # 雷达无效才视觉补位"。
        # ---- 显示源选择 + 迟滞（2026-08-25）：进视觉补位阈值 30%、退出阈值 20%，
        # 防两源在边界来回切（0058 实测 28.2% 帧 Abstandsindex 大差异即此抖动）。
        radar_valid = 0 < lead_distance < 1021
        vis_raw = self.op_lead_to_index(op_drel, CS.out.vEgo) if op_drel > 0 else 0
        if radar_valid and not self.disp_src_radar:
          # 视觉源中：仅当视觉也明显偏离（>30%）才切回雷达（迟滞进入条件）
          if vis_raw == 0 or abs(lead_distance - vis_raw) > 0.30 * max(vis_raw, 1):
            self.disp_src_radar = True
        elif not radar_valid:
          self.disp_src_radar = False
        use_radar = radar_valid and self.disp_src_radar
        if use_radar:
          raw_abstand = lead_distance
        elif op_drel > 0:
          raw_abstand = vis_raw
        elif now_nanos < self.lead_hold_expire and self.lead_hold_distance > 0:
          raw_abstand = self.lead_hold_distance   # 保持窗（2s）：两源都无目标
        else:
          raw_abstand = 0

        if use_radar:
          lead_object = max(lead_object, 1)
          self.lead_hold_expire = now_nanos + 2_000_000_000
          self.lead_hold_distance = lead_distance
        elif op_drel > 0:
          lead_object = 1
          self.lead_hold_expire = now_nanos + 2_000_000_000
          self.lead_hold_distance = lead_distance
        elif now_nanos < self.lead_hold_expire and self.lead_hold_distance > 0:
          lead_object = 1

        # ---- 显示变化率限速（每帧最大变化 = 相对速度 × 步长，物理上限；
        # 瞬态跳变被削平，真实接近/远离不受影响）。ACC_HUD_STEP=6 → 16Hz。
        # 2026-08-26 修复：无目标时 raw_abstand=0 必须透传 0（无前车语义）——
        # 此前 max(1,...) 钳位把 0 变成 1（最近档），叠加 mlbcan 的
        # "0<ab<1000 即 Relevantes_Objekt=1" 兜底 → 无车时仪表恒显示
        # "一辆车在最近位置"。仅在有目标时做平滑限速与 1..1021 clamp。
        max_step = max(4, int(abs(getattr(CS, 'op_lead_vRel', 0.0)) * 16 * (self.CCP.ACC_HUD_STEP / self.CCP.ACC_CONTROL_STEP)))
        if raw_abstand == 0:
          self.disp_abstand = None   # 复位平滑状态，下次出现目标从头开始
          lead_distance = 0          # 透传"无前车"
          lead_object = 0            # 2026-08-26 修复：无有效距离必须同步清目标标志，
                                     # 否则原厂无目标占位(ab=1021/relev=1)透传导致 ab=0/relev=1 矛盾帧
        elif self.disp_abstand is None:
          self.disp_abstand = raw_abstand
          lead_distance = max(1, min(1021, int(raw_abstand)))
        else:
          delta = raw_abstand - self.disp_abstand
          if abs(delta) > max_step:
            delta = max_step if delta > 0 else -max_step
            self.disp_abstand += delta
          else:
            self.disp_abstand = raw_abstand
          lead_distance = max(1, min(1021, int(self.disp_abstand)))
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, gas_override_stock)
        # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
        # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars, lead_object,
                                                         zeitluecke=getattr(CS, 'stock_zeitluecke', 4),
                                                         stock_prim_anz=getattr(CS, 'stock_prim_anz', 0),
                                                         stock_status_anzeige=getattr(CS, 'stock_status_anzeige', None),
                                                         stock_texte_prim=getattr(CS, 'stock_texte_prim', 0),
                                                         stock_display_prio=getattr(CS, 'stock_display_prio', None),
                                                         stock_wunschgeschw=getattr(CS, 'stock_wunschgeschw', None)))
        # OP 代发 ACC_04（原厂雷达状态文本，16Hz）：屏蔽 bus2->bus0 转发后由 OP 保持总线活跃，
        # 内容为原厂正常模板（无故障文本），避免网关/仪表对 ACC_04 超时监测报 ACC 故障
        lead_speed_kph = getattr(CS, 'stock_lead_speed_kph', 327.36)
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, gas_override_stock, getattr(CS, 'acc05_stock_status', 3))
        can_sends.append(self.CCS.create_acc_04_control(self.packer_pt, self.CAN.pt, lead_speed_kph, acc_control,
                                                         stock_texte_zusatz=getattr(CS, 'stock_acc04_texte_zusatz', None),
                                                         stock_charisma_status=getattr(CS, 'stock_acc04_charisma_status', None)))

    # **** Stock ACC Button Controls **************************************** #

    # 物理按键转发（relay 断开后 bus0->bus2 不转发，必须由 OP 代发才能到达原厂 ACC）：
    # pcmCruise 模式下 CC.cruiseControl 恒 False，改为直接读取原厂 LS_01 按键位——
    # 用户按 SET/RESUME/± 时 COUNTER 变化 → 原样代发到 bus2（原厂 ACC 侧），
    # 恢复"原厂 ACC 模式下按 SET/RESUME 继续起步"的原厂行为（00000047 路试反馈）。
    # 平台过滤：仅 MLB（Macan 等）启用物理按键转发——MQB/PQ/MEB 的
    # create_acc_buttons_control 不支持 set_increase/set_decrease（接口签名不同），
    # 且这些平台走原厂按键转发路径（bus0->bus2），无需 OP 代发。
    gra_send_ready = (self.CP.flags & VolkswagenFlags.MLB) and self.CP.pcmCruise \
                     and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=bool(CS.gra_stock_values.get("LS_Abbrechen", 0)),
                                                           resume=bool(CS.gra_stock_values.get("LS_Tip_Wiederaufnahme", 0)),
                                                           set_increase=bool(CS.gra_stock_values.get("LS_Tip_Setzen", 0)),
                                                           set_decrease=bool(CS.gra_stock_values.get("LS_Tip_Runter", 0))))

    # **** Macan 起步跟停（MacanStartStop）：原厂停车保持态时由视觉模型判定起步，
    # OP 代发 LS_01 RESUME 按键帧解除原厂 anh 保持（00000047 根因修复）********* #
    can_sends.extend(self.create_stop_and_go(self.CCS, self.packer_pt, self.CAN.ext, CC, CS, self.frame,
                                             resume_ready=sng_resume_ready))

    # 开机距离档位同步：停车+待机时代发 DIST +/- 脉冲，让原厂 ACC 内部档位
    # 与 OP 记忆对齐（MacanStartupGapSync，默认关；仅 MLB 生效，行驶/激活中禁止）
    can_sends.extend(self.gap_sync.create_startup_gap_sync(self.CCS, self.packer_pt, self.CAN.ext, CS, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
