import numpy as np

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags, CanBus

NO_OBJECT_ID = 0
LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Rel_Velo",
  )
  for lane in LANE_TYPES
  for idx in (1, 2)
  for prefix in (f"{lane}_0{idx}",)
)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    # With the MEB gateway harness, we do not have access to the raw points from the radar.
    # However, the camera publishes decent, albeit filtered, tracks. Two for each lane; left, center, and right.
    self.rcp: CANParser | None = None
    if CP.flags & VolkswagenFlags.MEB and not self.CP.radarUnavailable:
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], [("MEB_Distance_01", 25)], CanBus(CP).cam)

    # Macan (MLB, 非 MEB)：原厂 ACC 模块在 bus2 上报汇总雷达信号（ACC_02.Abstandsindex 距离 + ACC_04 前车速度）。
    # 雷达点数据不暴露在 CAN 上（ACC 模块内部消化），这里把汇总信号合成为单个标准雷达点，
    # 供 radard 的 get_lead 走"雷达点匹配"分支（Track 卡尔曼平滑）。
    # 标定表：2026-09-02 全量重标定（6-route，拟合26367/留出5979样本，中位相对误差
    # 低速8.23%/高速9.64%/全部8.86% vs 旧11点表15.12%）。低速区<234实测点，高速区保留原表。
    # 2026-09-02 修复：去掉 and not self.CP.radarUnavailable —— MLB 的 dbc_dict 只有
    # Bus.pt（无 Bus.radar）→ interface.py:19 判定 radarUnavailable=True，但这只是"dbc没
    # 定义雷达总线"的误标，Macan 实际有 ACC_02/04 汇总信号可合成点。原条件导致
    # _update_macan 从未被调用（A3 点从未生成→radard tracks 恒空→get_lead 永远纯视觉
    # →radarState.leadOne.radar 恒 False，0066 实测 0%）。修复后 A3 点正常注入官方链路。
    self._macan_radar = CP.carFingerprint == "PORSCHE_MACAN_MK1"
    self._macan_abstands_t = [0.81, 0.81, 0.81, 0.81, 0.81, 0.785, 0.804, 0.822, 0.837, 0.872, 0.942, 0.932, 0.899, 0.978, 1.025, 1.14, 1.168, 1.185, 1.203, 1.275, 1.288, 1.364, 1.422, 1.439, 1.459, 1.505, 1.571, 1.616, 1.682, 1.708, 1.766, 1.801, 1.828, 1.869, 1.933, 2.0, 2.028, 2.091, 2.12, 2.17, 2.246, 2.24, 2.351, 2.353, 2.394, 2.435, 2.467, 2.497, 2.553, 2.635, 2.653, 2.711, 2.774, 2.823, 2.923, 2.968, 3.031, 3.075, 3.121, 3.128, 3.197, 3.263, 3.249, 3.221, 3.299, 3.398, 3.472, 3.435, 3.37, 3.531, 3.619, 3.628, 3.684, 3.826, 3.726, 3.778, 3.884, 3.925, 3.955, 3.955, 4.074, 4.038, 4.065, 3.966, 4.014, 4.154, 4.193, 4.216, 4.406, 4.453, 4.301, 4.438, 4.562, 4.482, 4.512, 4.587, 4.71, 4.802, 4.565, 4.708, 4.835, 4.974, 4.654, 4.799, 4.81, 5.048, 4.816, 5.052, 5.098, 5.015, 5.237, 5.364, 5.606, 5.354, 5.336, 5.782, 5.384, 5.474, 5.536, 5.604, 5.377, 5.619, 5.587, 5.762, 5.787, 5.909, 5.862, 6.202, 6.331, 6.144, 5.986, 5.941, 5.893, 6.144, 6.061, 6.458, 6.146, 6.397, 6.59, 6.58, 6.536, 6.392, 6.494, 6.0, 6.94, 6.0, 6.0, 6.0, 6.0, 7.149, 6.0, 6.0, 6.0]
    self._macan_abstands_idx = [27, 32, 37, 42, 47, 52, 57, 62, 67, 72, 77, 82, 87, 92, 97, 102, 107, 112, 117, 122, 127, 132, 137, 142, 147, 152, 157, 162, 167, 172, 177, 182, 187, 192, 197, 202, 207, 212, 217, 222, 227, 232, 237, 242, 247, 252, 257, 262, 267, 272, 277, 282, 287, 292, 297, 302, 307, 312, 317, 322, 327, 332, 337, 342, 347, 352, 357, 362, 367, 372, 377, 382, 387, 392, 397, 402, 407, 412, 417, 422, 427, 432, 437, 442, 447, 452, 457, 462, 467, 472, 477, 482, 487, 492, 497, 502, 507, 512, 517, 522, 527, 532, 537, 542, 547, 552, 557, 562, 567, 572, 577, 582, 587, 592, 597, 602, 607, 612, 617, 622, 627, 632, 637, 642, 647, 652, 657, 662, 667, 672, 677, 682, 687, 692, 697, 702, 707, 712, 717, 722, 727, 732, 737, 742, 747, 752, 757, 762, 767, 772, 777, 780, 1021]

  def update(self, can_strings):
    if self.rcp is None:
      if self._macan_radar:
        return self._update_macan(can_strings)
      return super().update(None)

    self.rcp.update(can_strings)

    if len(self.rcp.vl_all["MEB_Distance_01"]["Distance_Status"]) == 0:
      return None

    return self._update()

  def _update_macan(self, can_strings):
    """Macan: bus2 ACC_02.Abstandsindex + ACC_04 前车速度 -> 合成单雷达点。
    轮速 BO_259 (ESP_*_Radgeschw, 12bit@0.1km/h) 解 v_ego 算相对速度。"""
    idx = 0
    lead_spd = 0.0
    v_sum = 0.0
    v_cnt = 0
    for msg in can_strings:
      d = msg.dat
      if msg.address == 259 and len(d) >= 8:
        # 四轮轮速 16|12 28|12 40|12 52|12 @1+ (0.1,0) km/h
        v_sum += (((d[2] | (d[3] << 8)) & 0xFFF)
                  + (((d[3] >> 4) | (d[4] << 4)) & 0xFFF)
                  + ((d[5] | (d[6] << 8)) & 0xFFF)
                  + (((d[6] >> 4) | (d[7] << 4)) & 0xFFF)) * 0.1
        v_cnt += 4
      elif msg.src == 2:
        if msg.address == 780 and len(d) >= 7:
          idx = (d[3] | (d[4] << 8)) & 0x3FF
        elif msg.address == 804 and len(d) >= 7:
          v = ((d[5] | (d[6] << 8)) & 0x3FF) * 0.32  # km/h
          if v < 320:
            lead_spd = v
    if idx <= 0 or idx >= 1021:
      return super().update(None)  # 无有效目标 -> 空雷达（视觉兜底）
    if v_cnt == 0:
      return super().update(None)  # 无轮速 -> 无法算相对速度，保守返回空
    v_ego = v_sum / v_cnt * 0.2778 * self.CP.wheelSpeedFactor  # km/h -> m/s
    # Abstandsindex -> 时距 t -> 距离（标定逆映射，低速用等效 t*5）
    t = float(np.interp(idx, self._macan_abstands_idx, self._macan_abstands_t))
    d_rel = t * max(v_ego, 5.0)
    v_lead = lead_spd / 3.6  # 前车绝对速度 (m/s)
    ret = structs.RadarData()
    point = structs.RadarData.RadarPoint()
    point.trackId = 1
    point.dRel = d_rel
    point.yRel = 0.0
    point.vRel = v_lead - v_ego
    ret.points = [point]
    return ret

  def _update(self):
    ret = structs.RadarData()

    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl["MEB_Distance_01"]

    # Can be 3 when radar sensor is obstructed
    if msg["Distance_Status"] != 0:
      ret.errors.radarUnavailableTemporary = True

    seen_ids = set()
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = int(msg[obj_id_sig])
      if obj_id == NO_OBJECT_ID:
        continue

      # We shouldn't see duplicate track ids
      if obj_id in seen_ids:
        ret.errors.radarFault = True
        return ret

      seen_ids.add(obj_id)

      if obj_id not in self.pts:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self.track_id
        self.track_id += 1
        self.pts[obj_id] = pt
      else:
        pt = self.pts[obj_id]

      pt.dRel = msg[long_sig]
      pt.yRel = msg[lat_sig]
      pt.vRel = msg[vel_sig]

    inactive_ids = self.pts.keys() - seen_ids
    for obj_id in inactive_ids:
      self.pts.pop(obj_id, None)

    ret.points = list(self.pts.values())
    return ret
