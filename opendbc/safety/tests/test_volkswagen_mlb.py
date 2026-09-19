#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg

MSG_ACC_02 = 0x30C
MSG_ACC_05 = 0x10D
MSG_ACC_04 = 0x324

MSG_LS_01 = 0x10B       # TX by OP, ACC control buttons for cancel/resume
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts


class TestVolkswagenMlbSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02)}

  MAX_RATE_UP = 9
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 169

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 3

  # Wheel speeds _esp_03_msg
  def _speed_msg(self, speed):
    values = {"ESP_%s_Radgeschw" % s: speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESP_03", 0, values)

  # Driver brake pressure over threshold
  def _esp_05_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Brake pedal switch
  def _motor_03_msg(self, brake_signal=False, gas_signal=0):
    values = {
      "MO_Fahrer_bremst": brake_signal,
      "MO_Fahrpedalrohwert_01": gas_signal,
    }
    return self.packer.make_can_msg_safety("Motor_03", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_03_msg(brake_signal=brake)

  def _user_gas_msg(self, gas):
    return self._motor_03_msg(gas_signal=gas)

  # ACC engagement status
  def _tsk_status_msg(self, enable):
    values = {"TSK_Status_GRA_ACC_02": 1 if enable else 0}
    return self.packer.make_can_msg_safety("TSK_04", 1, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"HCA_01_LM_Offset": abs(torque),
              "HCA_01_LM_OffSign": torque < 0,
              "HCA_01_Sendestatus": steer_req,
              "HCA_01_Status_HCA": 7 if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_01", 0, values)

  # Cruise control buttons
  def _ls_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"LS_Abbrechen": cancel, "LS_Tip_Setzen": _set, "LS_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("LS_01", bus, values)

  # Verify brake_pressed is true if either the switch or pressure threshold signals are true
  def test_redundant_brake_signals(self):
    test_combinations = [(True, True, True), (True, True, False), (True, False, True), (False, False, False)]
    for brake_pressed, motor_03_signal, esp_05_signal in test_combinations:
      self._rx(self._motor_03_msg(brake_signal=False))
      self._rx(self._esp_05_msg(False))
      self.assertFalse(self.safety.get_brake_pressed_prev())
      self._rx(self._motor_03_msg(brake_signal=motor_03_signal))
      self._rx(self._esp_05_msg(esp_05_signal))
      self.assertEqual(brake_pressed, self.safety.get_brake_pressed_prev(),
                       f"expected {brake_pressed=} with {motor_03_signal=} and {esp_05_signal=}")

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenMlbStockSafety(TestVolkswagenMlbSafetyBase):
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mlb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ls_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._ls_01_msg(resume=1)))
    self.assertFalse(self._tx(self._ls_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ls_01_msg(resume=1)))

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False))
    self.safety.set_controls_allowed(1)
    self._rx(self._ls_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")


class TestVolkswagenMlbLongSafety(TestVolkswagenMlbSafetyBase):
  # Regression guard for the panda tx whitelist with openpilot longitudinal enabled.
  # volkswagen_common_init() zeroes volkswagen_longitudinal, so the flag must be read AFTER it
  # (as volkswagen_mqb.h / volkswagen_pq.h already do). With the reversed order panda installed
  # VOLKSWAGEN_MLB_STOCK_TX_MSGS while openpilot believed it had longitudinal control, so every
  # ACC_02/ACC_05/ACC_04 frame was rejected on the bus and the car silently fell back to the
  # stock radar ACC (route 0000007b: src=192 rejected frames every 20ms).
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_ACC_02, 0], [MSG_ACC_05, 0], [MSG_ACC_04, 0],
             [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LS_01], 2: [MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_05, MSG_ACC_04]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_05, MSG_ACC_04), 2: (MSG_LS_01,)}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mlb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, VolkswagenSafetyFlags.LONG_CONTROL.value)
    self.safety.init_tests()

  # stock cruise engagement is bypassed under openpilot longitudinal control (LS_01 buttons instead)
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_long_control_flag_installs_long_tx_msgs(self):
    for addr in (MSG_ACC_02, MSG_ACC_05, MSG_ACC_04):
      self.assertTrue(self._tx(make_msg(0, addr, 8)),
                      f"0x{addr:x} rejected: the long control flag was cleared by volkswagen_common_init()")

  def test_long_control_flag_is_required_for_acc_tx(self):
    # without the flag panda must install the stock tx list and reject the ACC frames
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, 0)
    self.safety.init_tests()
    for addr in (MSG_ACC_02, MSG_ACC_05, MSG_ACC_04):
      self.assertFalse(self._tx(make_msg(0, addr, 8)), f"0x{addr:x} allowed without the long control flag")


if __name__ == "__main__":
  unittest.main()
