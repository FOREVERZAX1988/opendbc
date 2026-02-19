"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import mlbcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, packer, bus, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      set_increase = self.ICBM.sendButton == SendButtonState.increase
      set_decrease = self.ICBM.sendButton == SendButtonState.decrease

      # Send at ~5Hz to inject speed button presses into the stock ACC.
      # LS_01 from the real stalk runs at ~37Hz; injecting at ~5Hz provides
      # enough button-pressed frames for the ACC ECU to register the action
      # (stock ACC needs ~120ms to register a short press).
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          can_sends.append(mlbcan.create_acc_buttons_control(
            packer, bus, CS.gra_stock_values,
            set_increase=set_increase,
            set_decrease=set_decrease,
          ))
          self.last_button_frame = self.frame

    return can_sends
