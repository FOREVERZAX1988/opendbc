import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from openpilot.common.params import Params
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_group3 import Group3Object, Group3TrackIds

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
RADAR_REQUIRED_MSG_COUNT = 32
RADAR_MSG_COUNT4 = 8

# CAN-FD radar groups. Which one a car broadcasts is detected at fingerprint time
# (HyundaiExtFlags) because it varies by platform and ECU part number.
RADAR_START_ADDR_CANFD1 = 0x210  # group 1, two objects per message
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5  # group 2, one object per message
RADAR_MSG_COUNT2 = 32
RADAR_START_ADDR_CANFD3 = 0x400  # group 3, one object per message
RADAR_MSG_COUNT3 = 30
RADAR_GROUP3_DBC = "hyundai_canfd_radar_generated"

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count, required_msg_count):
  """CAN parser for the radar track messages.

  CAN-FD cars read the grouped DBC (``hyundai_canfd_radar_generated``), which carries the
  group-1/2/3 layouts; the Mando path reads the car's own radar DBC. Ported from cp.
  """
  if not radar_tracks:
    return None

  if CP.flags & HyundaiFlags.CANFD:
    CAN = CanBus(CP)
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
    return CANParser(RADAR_GROUP3_DBC, messages, CAN.ACAN)

  # Legacy Mando radars expose either 32 or 64 consecutive slots. Keep the first 32
  # mandatory for timing/CAN validity and accept the upper bank when present, so a
  # 32-slot radar stays compatible.
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None
  messages = [(f"RADAR_TRACK_{addr:x}", 20 if index < required_msg_count else math.nan)
              for index, addr in enumerate(range(msg_start_addr, msg_start_addr + msg_count))]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()

    self.canfd = bool(CP.flags & HyundaiFlags.CANFD)
    self.radar_group1 = False
    self.radar_group3 = False
    self.radar_group4 = not self.canfd and bool(CP.extFlags & HyundaiExtFlags.RADAR_GROUP4.value)

    # Which CAN-FD radar group this car broadcasts. Detected at fingerprint time because
    # it varies by platform and ECU part number; the decoding differs per group, so this
    # has to be settled before any parser is built. Legacy CAN always uses the Mando
    # layout. Ported from cp.
    if self.canfd:
      if CP.extFlags & HyundaiExtFlags.RADAR_GROUP1.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD1
        self.radar_msg_count = RADAR_MSG_COUNT1
        self.radar_group1 = True
      elif CP.extFlags & HyundaiExtFlags.RADAR_GROUP3.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD3
        self.radar_msg_count = RADAR_MSG_COUNT3
        self.radar_group3 = True
      else:
        # Group 2 is the default: it is what a CAN-FD car with no recognised group
        # broadcasts, and decoding it the Mando way is the previous behaviour.
        self.radar_start_addr = RADAR_START_ADDR_CANFD2
        self.radar_msg_count = RADAR_MSG_COUNT2
    else:
      self.radar_start_addr = RADAR_START_ADDR
      self.radar_msg_count = RADAR_MSG_COUNT4 if self.radar_group4 else RADAR_MSG_COUNT

    self.radar_required_msg_count = self.radar_msg_count
    if not self.canfd and not self.radar_group4:
      self.radar_required_msg_count = RADAR_REQUIRED_MSG_COUNT

    self.trigger_msg = self.radar_start_addr + self.radar_required_msg_count - 1

    self.radar_off_can = CP.radarUnavailable
    # EnableRadarTracks gates whether raw tracks are decoded at all. Read through Params
    # rather than CP so it can be toggled without a re-fingerprint; >= 1 matches cp.
    self.radar_tracks = Params().get_int("EnableRadarTracks") >= 1
    self.rcp = get_radar_can_parser(CP, self.radar_tracks, self.radar_start_addr,
                                    self.radar_msg_count, self.radar_required_msg_count)
    self.group3_track_ids = Group3TrackIds()

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    # Group 3 spreads its object list across 30 messages, so completeness is judged on
    # the LAST address of the group rather than on any single message. The trigger is set
    # from radar_start_addr + radar_required_msg_count - 1, which does that for every
    # group uniformly.
    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    if self.radar_group3:
      return self._update_group3(ret, updated_messages)

    if self.canfd:
      return self._update_group12(ret, updated_messages)

    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_group12(self, ret, updated_messages):
    """Decode the CAN-FD radar groups 1 and 2.

    Group 2 carries one object per message with VALID/VALID_CNT gating; group 1 carries
    two (the *_1 and *_2 signal families, the second half of the address range). They
    share LONG_DIST/LAT_DIST/REL_SPEED/LAT_SPEED/REL_ACCEL signal names, so unlike the
    Mando path there is no AZIMUTH to project through - the lateral offset is already
    in metres.

    Slots beyond radar_required_msg_count are optional: they do not participate in CAN
    validity, so a stale frame must not be kept as if it were current. Ported from cp.
    """
    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = t_id

      stale = (addr >= self.radar_start_addr + self.radar_required_msg_count
               and addr not in updated_messages)

      if self.radar_group1:
        valid, track_state, d_rel, y_rel, v_rel, a_rel, yv_rel = (
          msg['VALID_CNT1'] > 10, 0, msg['LONG_DIST1'], msg['LAT_DIST1'],
          msg['REL_SPEED1'], msg['REL_ACCEL1'], msg['LAT_SPEED1'])
      else:
        valid, track_state, d_rel, y_rel, v_rel, a_rel, yv_rel = (
          msg['VALID_CNT'] > 10, int(msg['VALID']), msg['LONG_DIST'], msg['LAT_DIST'],
          msg['REL_SPEED'], msg['REL_ACCEL'], msg['LAT_SPEED'])

      valid = bool(valid) and not stale
      point = self.pts[addr]
      point.measured = valid
      if not valid:
        point.dRel, point.yRel, point.vRel = 0., 0., 0.
        point.vLead = self.v_ego
        point.aRel, point.yvRel = float('nan'), 0.
      else:
        point.dRel, point.yRel, point.vRel = float(d_rel), float(y_rel), float(v_rel)
        point.vLead = point.vRel + self.v_ego
        point.aRel, point.yvRel = float(a_rel), float(yv_rel)
      t_id += 1

    ret.points = [p for p in self.pts.values() if p.measured]
    return ret

  def _update_group3(self, ret, updated_messages):
    """Decode the group-3 object list.

    Group 3 addresses are transport slots, not identities: objects move between
    addresses, and an address can be reused for a different object. Group3TrackIds
    resolves a stable identifier per object so the downstream tracker does not see a
    new target every time an object shifts slot, and so a reused slot does not inherit
    the previous occupant's state.

    Points are keyed by slot offset from the group start, and the previous entries are
    cleared first - an unmeasured copy left behind would reset the surviving object's
    filter. Ported from cp.
    """
    objects = {
      addr: Group3Object.from_signals(self.rcp.vl[f"RADAR_TRACK_{addr:x}"])
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count)
      if addr in updated_messages
    }
    assignments = self.group3_track_ids.update(objects)

    for slot in range(32, 32 + self.radar_msg_count):
      self.pts.pop(slot, None)

    for addr, track_id in assignments.items():
      obj = objects[addr]
      point = structs.RadarData.RadarPoint()
      point.trackId = track_id
      point.radarSource = "frontRadar"
      point.measured = True
      point.dRel, point.yRel, point.vRel = obj.d_rel, obj.y, obj.v
      point.vLead, point.aRel, point.yvRel = self.v_ego + obj.v, float("nan"), 0.0
      self.pts[32 + addr - self.radar_start_addr] = point

    ret.points = list(self.pts.values())
    return ret
