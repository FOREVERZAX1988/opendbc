import unittest

from opendbc.car import create_button_events, structs
from opendbc.car.toyota.carstate import (TOYOTA_VIRTUAL_CRUISE_LONG_PRESS, VIRTUAL_CRUISE_BUTTONS,
                                         get_virtual_cruise_button)
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from opendbc.sunnypilot.car.interfaces import _initialize_toyota
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, VIRTUAL_CRUISE_SPEED_CAR

ButtonType = structs.CarState.ButtonEvent.Type


def build_params(*, platform=CAR.TOYOTA_COROLLA_TSS2, available=True, openpilot_long=True):
  CP = structs.CarParams(
    brand="toyota",
    carFingerprint=str(platform),
    openpilotLongitudinalControl=openpilot_long,
    alphaLongitudinalAvailable=True,
    safetyConfigs=[structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.toyota)],
  )
  flags = ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE if available else 0
  return CP, structs.CarParamsSP(flags=flags, pcmCruiseSpeed=True)


class TestVirtualCruiseButtonAPI(unittest.TestCase):
  def test_long_press_threshold_matches_openpilot(self):
    # Must stay in sync with openpilot/selfdrive/car/cruise.py, otherwise the
    # software-PCM long-press cadence and the button timers disagree.
    self.assertEqual(TOYOTA_VIRTUAL_CRUISE_LONG_PRESS, 65)

  def test_button_mapping(self):
    self.assertEqual(set(VIRTUAL_CRUISE_BUTTONS), {1, 2})
    self.assertEqual(VIRTUAL_CRUISE_BUTTONS[1], ButtonType.accelCruise)
    self.assertEqual(VIRTUAL_CRUISE_BUTTONS[2], ButtonType.decelCruise)

  def test_raw_pair_decoding(self):
    for cruise_res, cruise_set, expected in ((0, 0, 0), (1, 0, 1), (0, 1, 2), (1, 1, 0)):
      with self.subTest(res=cruise_res, set=cruise_set):
        self.assertEqual(get_virtual_cruise_button(cruise_res, cruise_set), expected)

  def test_button_edges(self):
    cases = (
      (0, 1, ((ButtonType.accelCruise, True),)),
      (1, 0, ((ButtonType.accelCruise, False),)),
      (1, 2, ((ButtonType.accelCruise, False), (ButtonType.decelCruise, True))),
    )
    for previous, current, expected in cases:
      with self.subTest(previous=previous, current=current):
        events = create_button_events(current, previous, VIRTUAL_CRUISE_BUTTONS)
        self.assertEqual(tuple((event.type, event.pressed) for event in events), expected)


class TestVirtualCruiseCapability(unittest.TestCase):
  def test_platform_scope_and_flag_bit(self):
    self.assertEqual(VIRTUAL_CRUISE_SPEED_CAR, {CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_PRIUS_TSS2})
    # 256 is SP_BRAKE_ONSET on this branch, so the upstream value would alias.
    self.assertEqual(int(ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE), 2048)
    self.assertEqual(ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE & ToyotaFlagsSP.SP_BRAKE_ONSET, 0)

  def test_capability_is_platform_scoped(self):
    for candidate, expected in (
      (CAR.TOYOTA_COROLLA_TSS2, True),
      (CAR.TOYOTA_PRIUS_TSS2, True),
      (CAR.TOYOTA_RAV4_TSS2, False),
    ):
      with self.subTest(candidate=str(candidate)):
        CP, CP_SP = build_params(platform=candidate, available=False)
        CarInterface._get_params_sp(CP, CP_SP, candidate, {0: {}}, [], False, False, False)
        self.assertEqual(bool(CP_SP.flags & ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE), expected)

  def test_ownership_is_opt_in(self):
    cases = (
      (True, True, True, False),
      (False, True, True, True),
      (True, False, True, True),
      (True, True, False, True),
    )
    for enabled, available, openpilot_long, expected in cases:
      with self.subTest(enabled=enabled, available=available, openpilot_long=openpilot_long):
        CP, CP_SP = build_params(available=available, openpilot_long=openpilot_long)
        _initialize_toyota(CP, CP_SP, {"ToyotaVirtualCruiseSpeed": int(enabled)})
        self.assertEqual(CP_SP.pcmCruiseSpeed, expected)

  def test_stock_longitudinal_wins(self):
    CP, CP_SP = build_params()
    _initialize_toyota(CP, CP_SP, {
      "ToyotaEnforceStockLongitudinal": 1,
      "ToyotaVirtualCruiseSpeed": 1,
    })
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP_SP.pcmCruiseSpeed)

  def test_missing_param_is_safe(self):
    # The key may not exist yet on older installs; must not raise or silently enable.
    CP, CP_SP = build_params()
    _initialize_toyota(CP, CP_SP, {})
    self.assertTrue(CP_SP.pcmCruiseSpeed)


if __name__ == "__main__":
  unittest.main()
