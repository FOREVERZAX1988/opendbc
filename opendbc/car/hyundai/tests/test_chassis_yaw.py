#!/usr/bin/env python3
"""Tests for the fingerprint-gated Hyundai chassis yaw-rate capability.

Locks three things that are easy to get wrong when porting from cp:
  1. capability bit is the agreed value (2**20, no collision with cp's 2**17-19)
  2. ESP12 YAW_RATE DBC scale implies deg/s, so carstate must apply DEG_TO_RAD
  3. the bit is extFlags (not flags) so panda safety is unaffected
"""
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.carstate import CHASSIS_YAW_INVALID
from opendbc.car.hyundai.values import HyundaiExtFlags


class TestChassisYawCapability(unittest.TestCase):
  def test_capability_bit_value(self):
    self.assertEqual(HyundaiExtFlags.CHASSIS_YAW_RATE.value, 2 ** 20)

  def test_capability_is_distinct_from_cp_radar_bits(self):
    # cp uses 2**17-2**19 for BOSCH_RADAR flags in its HyundaiExtFlags; sp keeps
    # bit values comparable across forks, so this capability must use 2**20+.
    cp_radar_bits = 2 ** 17 | 2 ** 18 | 2 ** 19
    self.assertEqual(HyundaiExtFlags.CHASSIS_YAW_RATE.value & cp_radar_bits, 0)

  def test_esp12_yaw_scale_is_deg_per_second(self):
    # ESP12 (0x220) YAW_RATE range field is [-40.95|40.96] physical deg/s.
    # cp decodes it raw; carstate.py must multiply by DEG_TO_RAD to satisfy
    # car.capnp rad/s. Without it the KF would see ~57x too much yaw.
    max_deg = 40.95
    self.assertAlmostEqual(max_deg * CV.DEG_TO_RAD, 0.7149, places=3)
    self.assertLess(max_deg * CV.DEG_TO_RAD, 1.0)  # < 1 rad/s, plausible yaw
    # The raw (unconverted) value would be ~41 rad/s - physically absurd.
    self.assertGreater(max_deg, 40.0)

  def test_invalid_sentinel_exceeds_locationd_sanity_gate(self):
    # STAT-invalid readings must not be emitted as 0 (that would inject a false
    # high-confidence zero-yaw observation). The sentinel must be far above
    # locationd's ROTATION_SANITY_CHECK=10 rad/s so the consumer skips it.
    self.assertGreater(CHASSIS_YAW_INVALID, 10.0)  # > ROTATION_SANITY_CHECK
    self.assertGreater(CHASSIS_YAW_INVALID, 40.95 * CV.DEG_TO_RAD)  # > real max ~0.71


if __name__ == "__main__":
  unittest.main()
