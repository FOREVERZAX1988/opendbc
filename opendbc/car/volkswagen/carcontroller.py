import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from opendbc.sunnypilot.car.volkswagen.icbm import IntelligentCruiseButtonManagementInterface

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


# 1. 恢复多继承 (从代码1恢复)
class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    # 2. 显式调用父类初始化 (从代码1恢复)
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)

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
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)

    # 3. 恢复 EPS Timer Workaround 变量初始化 (从代码1恢复)
    self.eps_timer_workaround = bool(CP.flags & VolkswagenFlags.MLB)
    self.hca_frame_timer_resetting = 0
    self.hca_frame_low_torque = 0
    self.hca_frame_timer_running = 0
    # 注意：hca_frame_same_torque 不再需要，因为逻辑已封装进 HCAMitigation 类

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    output_torque = 0

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

        # 4. 运行计时器 (从代码1恢复)
        self.hca_frame_timer_running += self.CCP.STEER_STEP

        # 5. 使用上游新的 HCAMitigation 类 (保留代码2的优化)
        apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)

        hca_enabled = abs(apply_torque) > 0

        # 6. 恢复 MLB EPS Timer Reset Workaround 核心逻辑 (从代码1恢复)
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

      # 7. 恢复 output_torque 赋值逻辑 (修复代码2的Bug，从代码1恢复)
      if hca_enabled:
        output_torque = apply_torque
        self.hca_frame_timer_resetting = 0
      else:
        output_torque = 0
        self.hca_frame_timer_resetting += self.CCP.STEER_STEP
        if self.hca_frame_timer_resetting >= self.CCP.STEER_TIME_RESET / DT_CTRL or not self.eps_timer_workaround:
          self.hca_frame_timer_running = 0
          apply_torque = 0

      # 8. 恢复软禁用警报 (从代码1恢复)
      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL

      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, output_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #
    # (修正为 gear_ratio，与 mlbcan.py 匹配)

    if self.CP.openpilotLongitudinalControl:

      # 【参考 ACC05】控制 ACC04 发送频率为 25Hz
      if self.frame % 4 == 0:  # 100Hz / 4 = 25Hz
        if hasattr(CS, 'acc04_stock_values') and CS.acc04_stock_values:
          can_sends.append(self.CCS.create_acc04_control(self.packer_pt, self.CAN.pt, CS.acc04_stock_values))

      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, CS.out.gasPressed)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation, v_ego=CS.out.vEgo,
                                                           gear_ratio=getattr(CS, 'gear_ratio', 0.0),
                                                           gas_pressed=CS.out.gasPressed, resume=CC.cruiseControl.resume, stock_acc05_values=getattr(CS, 'stock_acc05_values', None)))


    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = getattr(CS, 'stock_lead_distance', 0)
      lead_object = getattr(CS, 'stock_lead_object', 0)
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, CS.out.gasPressed)
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_control, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars, lead_object,
                                                       zeitluecke=getattr(CS, 'stock_zeitluecke', 4)))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    # **** Intelligent Cruise Button Management ******************************** #
    # (保持不变，依赖于 __init__ 中正确初始化了接口)

    if self.CP.flags & VolkswagenFlags.MLB:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.CAN.ext,
                                                                        self.frame, self.last_button_frame))

    new_actuators = actuators.as_builder()
    new_actuators.torque = output_torque / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends