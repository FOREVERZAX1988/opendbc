import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

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
    self.hca_mitigation = HCAMitigation(self.CCP)

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

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
        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

        apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
        hca_enabled = apply_torque != 0
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

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
          gas_override = CS.out.gasPressed and CS.out.cruiseState.available and not brake_override
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, CS.out.gasPressed)
          # OVERRIDE(4) 时保持巡航力矩（原厂 st=4 力矩≈st=3，仅状态字切 4）；accel=0 → 巡航基线
          torque_active = long_active or gas_override
          # 油门超驰：gas 时 accel 强制 0 → 力矩=巡航基线，驾驶员主导加速；松油门立即恢复 planner 控制
          accel = float(np.clip(0.0 if CS.out.gasPressed else actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if torque_active else 0)
          # 原厂意图仲裁（00000037/00000038 根因修复）：OP 代发请求不能与原厂雷达矛盾。
          # 原厂在请求减速（verz<0）或停车（anh=1）时，OP 跟随原厂意图（最多同深度减速），
          # 绝不允许比原厂激进（正加速）——否则雷达自检失败报 st=6 → ECU 写 DTC 锁死 ACC。
          # 跟停后起步需等原厂先释放（anh=0 且 verz≥0），起步略延迟是安全代价。
          stock_verz = getattr(CS, 'acc05_stock_verz', 0.0)
          stock_anhalten = getattr(CS, 'acc05_stock_anhalten', False)
          if torque_active and (stock_anhalten or stock_verz < -0.15):
            accel = min(accel, stock_verz if stock_verz < 0 else -1.0)
          # 撤力仲裁（00000038 实锤补丁）：原厂力矩请求持续归零（mom<60）表示雷达在撤动力、
          # 认为不应加速（目标接近/限速/弯道）。此时 OP 若仍猛加速（00000038: 92→152）会触发
          # 雷达自检 st=6 → DTC 锁死。OP 跟随：原厂撤力时禁止正加速，最多滑行（accel<=0）。
          # 反向（原厂保持、OP 减速）已实测安全（7 窗口 0 锁死），故此处只限制「比原厂激进」。
          stock_mom = getattr(CS, 'acc05_stock_mom', 1021.0)
          if torque_active and stock_mom < 60:
            accel = min(accel, 0.0)
          stopping = actuators.longControlState == LongCtrlState.stopping and not brake_override
          # 油门超驰（0000003f 红灯起步实锤）：驾驶员踩油门时强制释放停车请求，
          # 否则模型 shouldStop 卡1 → LCS 卡 stopping → Anh=1 → 踩油门也被按死。
          # 对齐原厂 st=4 超驰语义：油门接管时 ACC 不请求保持停车。
          if CS.out.gasPressed:
            stopping = False
          # 跟停保持：进入 stopping 且 vEgo≈0 后保持 anh=1，防止停稳后 stopping 偶发掉 0
          # （00000039 seg7: 512.9-513.2 OP anh 掉 0 → 原厂雷达判定矛盾 st6）。
          # 起步（vEgo>0.5）或踩刹车时释放。
          if stopping:
            self.stopping_hold = True
          elif self.stopping_hold and (brake_override or CS.out.gasPressed or CS.out.vEgo > 0.5):
            self.stopping_hold = False
          stopping = stopping or self.stopping_hold
          # vEgoStopping 字段在 car.capnp 与 volkswagenMqbEvo@29 ordinal 冲突被 capnp 静默忽略 → 运行时缺失。
          # getattr 兜底 2.0 m/s（≈7.2km/h 低速起步阈值，VW ACC 起步语义），避免激活后 AttributeError 崩溃。
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < getattr(self.CP, 'vEgoStopping', 2.0)) and not brake_override
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, torque_active, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation, v_ego=CS.out.vEgo,
                                                             engine_torque=getattr(CS, 'engine_torque_output', 0)))

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
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, CS.out.gasPressed)
        # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
        # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars, lead_object,
                                                         zeitluecke=getattr(CS, 'stock_zeitluecke', 4)))
        # OP 代发 ACC_04（原厂雷达状态文本，16Hz）：屏蔽 bus2->bus0 转发后由 OP 保持总线活跃，
        # 内容为原厂正常模板（无故障文本），避免网关/仪表对 ACC_04 超时监测报 ACC 故障
        lead_speed_kph = getattr(CS, 'stock_lead_speed_kph', 327.36)
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active, CS.out.gasPressed)
        can_sends.append(self.CCS.create_acc_04_control(self.packer_pt, self.CAN.pt, lead_speed_kph, acc_control))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
