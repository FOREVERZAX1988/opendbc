#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

MSG_LS_01 = 0x10B       # TX by OP, ACC control buttons for cancel/resume
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts
MSG_ACC_02 = 0x30C      # TX by OP, ACC HUD data to the instrument cluster
MSG_ACC_05 = 0x10D      # RX from radar / TX by OP, ACC status
MSG_ACC_04 = 0x324      # RX from radar, ACC status texts (HUD/Cluster)
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
  def _ls_01_msg(self, cancel=0, resume=0, _set=0, main_on=0, bus=2):
    values = {"LS_Abbrechen": cancel, "LS_Tip_Setzen": _set, "LS_Tip_Wiederaufnahme": resume,
              "LS_Hauptschalter": main_on}
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
class TestVolkswagenMlbStockSafety(TestVolkswagenMlbSafetyBase):
  DBC = "vw_mlb"
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenMlb

  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

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
  """OP longitudinal (Macan): LS_01 owns engagement, the stock ACC status is bypassed."""
  DBC = "vw_mlb"
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenMlb
  SAFETY_PARAM = VolkswagenSafetyFlags.LONG_CONTROL

  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_ACC_02, 0], [MSG_ACC_05, 0], [MSG_ACC_04, 0],
             [MSG_LS_01, 0], [MSG_LS_01, 2]]
  # Every frame OP authors with check_relay=true trips the relay check when seen on the wire,
  # and is also blocked from being forwarded onto the bus OP transmits it on.
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_05, MSG_ACC_04), 2: (MSG_LS_01,)}
  # bus 0 -> 2: LS_01 is authored by OP on bus 2 (physical stalk copy), so the gateway copy must
  # not race it there - that double source is what used to raise ACC/PAS st=6 on the Macan.
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LS_01], 2: [MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_05, MSG_ACC_04]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  # The stock cruise state must not move controls_allowed under OP longitudinal
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_engage(self):
    for button, kwargs in (("SET", {"_set": 1}), ("RESUME", {"resume": 1})):
      self.safety.set_controls_allowed(0)
      # main switch on, no button edge -> no engagement
      self._rx(self._ls_01_msg(main_on=1, bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), "engaged without a button edge")

      self._rx(self._ls_01_msg(main_on=1, bus=0, **kwargs))
      self.assertTrue(self.safety.get_controls_allowed(), f"{button} did not engage")

      # releasing the button keeps controls
      self._rx(self._ls_01_msg(main_on=1, bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"{button} dropped controls on release")

      # cancel disengages
      self._rx(self._ls_01_msg(cancel=1, main_on=1, bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), "cancel did not disengage")

      # and the same button re-engages after a full release
      self._rx(self._ls_01_msg(main_on=1, bus=0))
      self._rx(self._ls_01_msg(main_on=1, bus=0, **kwargs))
      self.assertTrue(self.safety.get_controls_allowed(), f"{button} did not re-engage")

  def test_no_engage_with_main_switch_off(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._ls_01_msg(main_on=0, bus=0))
    self._rx(self._ls_01_msg(_set=1, main_on=0, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "engaged with the ACC main switch off")

  def test_main_switch_off_drops_controls(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._ls_01_msg(main_on=1, bus=0))
    self._rx(self._ls_01_msg(_set=1, main_on=1, bus=0))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._ls_01_msg(main_on=0, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "main switch off did not drop controls")

  def test_stock_acc_status_ignored(self):
    # TSK_04 carries the stock ACC state; it must not touch controls in this mode
    self.safety.set_controls_allowed(0)
    self._rx(self._ls_01_msg(main_on=1, bus=0))
    self._rx(self._ls_01_msg(resume=1, main_on=1, bus=0))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._tsk_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed(), "stock ACC disengage dropped OP longitudinal")


if __name__ == "__main__":
  unittest.main()
