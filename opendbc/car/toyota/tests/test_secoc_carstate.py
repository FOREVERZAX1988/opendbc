"""SecOC TSS2 carstate through the real CarInterface parsers.

Rewritten from StarPilot's panda-layer SecOC AOL tests (test_toyota_secoc_aol.py) for
this fork's architecture: gear/main/door/belt gating lives in the car state and MADS
layers, not the panda safety layer, so the intent of the source tests (gear sourced
from the checksummed 0x127 GEAR_PACKET_HYBRID, stale frames must not mislead it, real
captured Sienna frames parse into sane physical state) is verified here instead.

The real CAN byte fixtures are captured frames from a Sienna 4th gen and carried over
verbatim from the StarPilot tests:
- 0x1D3 PCM_CRUISE_2: MAIN on, SET_SPEED 29 km/h (checksummed, matches toyota_checksum)
- 0x620 BODY_CONTROL_STATE: doors closed, driver belt latched, parking brake set
  (raw capture; checksum is a panda-side concern, the parser reads the signals)
- 0x262 EPS_STATUS: IPAS_STATE 0, LTA_STATE 1, LKA_STATE 1 (checksummed, matches
  toyota_checksum)

Parser note: EPS_STATUS is not in get_can_parsers' explicit list — it is auto-
registered on first read inside carstate.update(), so its values only land on the
second update. GEAR_PACKET/GEAR_PACKET_HYBRID are explicitly registered (NaN rate,
alive-ignored) and land on the first update.
"""
import unittest

from opendbc.car import gen_empty_fingerprint, Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota.carstate import CarState, TEMP_STEER_FAULTS, PERM_STEER_FAULTS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.toyota.values import CAR, ToyotaFlags

GearShifter = structs.CarState.GearShifter

REAL_1D3 = bytes.fromhex("00a81d39f80000d2")
REAL_620 = bytes.fromhex("000000008000001a")
REAL_262 = bytes.fromhex("0008000208ffff7c")

P, R, N, D, B = 0, 1, 2, 3, 4


def checksum(address: int, data: bytes) -> bytes:
  """The StarPilot fixture checksum — identical algorithm to toyota_checksum."""
  data = bytearray(data)
  data[-1] = ((address & 255) + (address >> 8) + len(data) + sum(data[:-1])) & 255
  return bytes(data)


def hybrid_gear(gear: int) -> bytes:
  """A 0x127 GEAR_PACKET_HYBRID frame carrying `gear` (GEAR@bit47, 4 bits), checksummed."""
  return checksum(0x127, bytes(5) + bytes([gear << 4]) + bytes(2))


class SecocCarstateHarness:

  def __init__(self, candidate):
    self.CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [],
                                      alpha_long=False, is_release=False, docs=False)
    self.CP_SP = CarInterface.get_params_sp(self.CP, candidate, gen_empty_fingerprint(), [],
                                            alpha_long=False, is_release_sp=False, docs=False)
    self.CI = CarInterface(self.CP, self.CP_SP)

  def feed(self, i, *frames):
    """One CarInterface.update with (addr, data, bus) triples; 10 ms cadence."""
    return self.CI.update([(i * 10_000_000, [CanData(a, d, b) for a, d, b in frames])])

  def physical(self, i, gear=D, *, real=False, repeats=2):
    """The StarPilot `physical()` frame set: main, EPS, then gear.

    Two identical updates by default so the auto-registered EPS_STATUS lands.
    """
    if real:
      frames = [(0x1D3, REAL_1D3, 0), (0x620, REAL_620, 0), (0x262, REAL_262, 0)]
    else:
      main = checksum(0x1D3, bytes([0, 128]) + bytes(6))
      eps = checksum(0x262, bytes(3) + bytes([1 << 1]) + bytes(4))
      frames = [(0x1D3, main, 0), (0x262, eps, 0)]
    if gear is not None:
      frames.append((0x127, hybrid_gear(gear), 0))
    for j in range(i, i + repeats):
      self.feed(j, *frames)

  @property
  def out(self):
    return self.CI.CS.out


class TestRealSiennaFrames(unittest.TestCase):
  """The captured frames parse into the physical state they were captured in."""

  def setUp(self):
    self.h = SecocCarstateHarness(CAR.TOYOTA_SIENNA_4TH_GEN)
    self.assertTrue(self.h.CP.flags & ToyotaFlags.SECOC)

  def test_real_frames_parse_into_sane_state(self):
    self.h.physical(0, gear=D, real=True, repeats=5)
    self.assertEqual(self.h.out.gearShifter, GearShifter.drive)
    self.assertTrue(self.h.out.cruiseState.available)   # MAIN on in REAL_1D3
    self.assertFalse(self.h.out.doorOpen)               # all doors closed in REAL_620
    self.assertFalse(self.h.out.seatbeltUnlatched)      # driver belt latched in REAL_620
    self.assertTrue(self.h.out.parkingBrake)            # set in REAL_620 (stationary capture)
    self.assertFalse(self.h.out.steerFaultTemporary)    # LKA/LTA_STATE 1 in REAL_262
    self.assertFalse(self.h.out.steerFaultPermanent)

  def test_main_off_hides_cruise(self):
    main_off = checksum(0x1D3, bytes(8))
    for i in range(5):
      self.h.feed(i, (0x1D3, main_off, 0), (0x127, hybrid_gear(D), 0))
    self.assertFalse(self.h.out.cruiseState.available)


class TestChecksumFixtures(unittest.TestCase):
  """The checksummed fixtures verify the toyota_checksum algorithm against real frames."""

  def test_1d3_and_262_fixtures_are_checksummed(self):
    self.assertEqual(checksum(0x1D3, REAL_1D3), REAL_1D3)
    self.assertEqual(checksum(0x262, REAL_262), REAL_262)
    self.assertEqual(toyota_checksum(0x1D3, None, bytearray(REAL_1D3)), REAL_1D3[-1])
    self.assertEqual(toyota_checksum(0x262, None, bytearray(REAL_262)), REAL_262[-1])

  def test_620_is_a_raw_capture(self):
    # StarPilot injects BODY_CONTROL_STATE without recomputing the checksum (their rx()
    # defaults crc=False for it); the byte is not the toyota_checksum of the data.
    self.assertNotEqual(checksum(0x620, REAL_620), REAL_620)

  def test_corrupt_checksum_recompute_restores(self):
    # A single flipped bit in the checksum byte breaks the relation; recomputing restores it.
    broken = bytearray(REAL_262)
    broken[-1] ^= 1
    self.assertNotEqual(checksum(0x262, bytes(broken[:-1]) + bytes([broken[-1]])), bytes(broken))
    self.assertEqual(checksum(0x262, bytes(broken[:-1]) + bytes([broken[-1]])), REAL_262)


class TestHybridGearParsing(unittest.TestCase):
  """GEAR (P 0, R 1, N 2, D 3, B 4) rides in 0x127 on SECOC cars, ~60 Hz."""

  def setUp(self):
    self.h = SecocCarstateHarness(CAR.TOYOTA_SIENNA_4TH_GEN)

  def test_every_gear_value_maps(self):
    expected = {
      P: GearShifter.park,
      R: GearShifter.reverse,
      N: GearShifter.neutral,
      D: GearShifter.drive,
      B: GearShifter.brake,
    }
    for gear, shifter in expected.items():
      self.h.physical(0, gear=gear)
      self.assertEqual(self.h.out.gearShifter, shifter, gear)

  def test_hybrid_frame_is_the_source_on_secoc(self):
    # A stale 0x3BC GEAR_PACKET must not override the hybrid gear on a SECOC car.
    self.h.physical(0, gear=D)
    stale_3bc = checksum(0x3BC, bytes([0, 16]) + bytes(6))  # GEAR@bit13 = 16 -> "R" per the 0x3BC VAL_ table
    self.h.feed(1, (0x3BC, stale_3bc, 0), (0x127, hybrid_gear(D), 0))
    self.assertEqual(self.h.out.gearShifter, GearShifter.drive)


class TestGearSourceSelection(unittest.TestCase):
  """Non-SECOC cars read the gear from 0x3BC; both gear parsers are always registered."""

  def test_non_secoc_sienna_reads_gear_packet(self):
    h = SecocCarstateHarness(CAR.TOYOTA_SIENNA)
    self.assertFalse(h.CP.flags & ToyotaFlags.SECOC)
    # GEAR@bit13 = 32 -> "P" per the 0x3BC VAL_ table (0 "D" 1 "S" 8 "N" 16 "R" 32 "P")
    p = checksum(0x3BC, bytes([0, 32]) + bytes(6))
    for i in range(5):
      h.feed(i, (0x3BC, p, 0))
    self.assertEqual(h.out.gearShifter, GearShifter.park)

  def test_both_gear_parsers_registered_on_secoc(self):
    h = SecocCarstateHarness(CAR.TOYOTA_SIENNA_4TH_GEN)
    # Both are NaN-rate alive-ignored parsers, but on a SECOC car the carstate only
    # reads GEAR_PACKET_HYBRID — the 0x3BC values never reach gearShifter.
    pt_parsers = CarState.get_can_parsers(h.CP, h.CP_SP)
    self.assertIn("GEAR_PACKET_HYBRID", pt_parsers[Bus.pt].vl)
    self.assertIn("GEAR_PACKET", pt_parsers[Bus.pt].vl)


class TestEpsFaultStates(unittest.TestCase):
  """LKA/LTA_STATE drive steerFaultTemporary/Permanent exactly as StarPilot's eps gate did."""

  def setUp(self):
    self.h = SecocCarstateHarness(CAR.TOYOTA_SIENNA_4TH_GEN)

  def eps(self, state):
    frame = checksum(0x262, bytes(3) + bytes([(state << 1) & 0xFF]) + bytes(4))
    for j in range(5):
      self.h.feed(j, (0x262, frame, 0))

  def test_healthy_eps_has_no_fault(self):
    self.h.physical(0, gear=D, repeats=5)
    self.assertFalse(self.h.out.steerFaultTemporary)
    self.assertFalse(self.h.out.steerFaultPermanent)

  def test_temporary_fault_states(self):
    for state in (9, 11, 21, 25):
      self.setUp()
      self.eps(state)
      self.assertTrue(self.h.out.steerFaultTemporary, state)
      self.assertFalse(self.h.out.steerFaultPermanent, state)

  def test_permanent_fault_states(self):
    for state in (3, 17):
      self.setUp()
      self.eps(state)
      self.assertTrue(self.h.out.steerFaultPermanent, state)
      self.assertFalse(self.h.out.steerFaultTemporary, state)

  def test_fault_constants_cover_starpilot_rejected_states(self):
    # StarPilot's AOL gate rejected eps in (0, 3, 9, 17, 21, 25); this fork maps
    # 0/9/11/21/25 to temporary faults and 3/17 to permanent ones.
    for state in (0, 9, 11, 21, 25):
      self.assertIn(state, TEMP_STEER_FAULTS, state)
    for state in (3, 17):
      self.assertIn(state, PERM_STEER_FAULTS, state)
    self.assertNotIn(1, TEMP_STEER_FAULTS)
    self.assertNotIn(1, PERM_STEER_FAULTS)


if __name__ == "__main__":
  unittest.main()
