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

# Macan Abstandsindex <-> 时距标定（0910 方案B / B1 单表）：t(idx) = A*idx + B
# 干净"同目标"帧、6 route / 15,040 帧重拟合，RMS 0.61 s。
# 效果：视觉-雷达 中位差 -0.77 m、中位绝对差 1.90 m、82.0% 帧 <=5 m、A2 原厂替换率 6.3%
# （旧 0909 新线性 0.008718/+1.0178：中位差 -7.10 m、仅 17.5% <=5 m、替换率 35.3%）。
# 注意：openpilot radard.py 的 A2（_macan_t_from_idx / _macan_drel_to_idx）使用同一组
# 系数，必须同源修改，否则融合两源会落在不同尺度上。详见 ai/docs/MLB_MACAN_PLANB_FIT_0910.md
MACAN_B1_T_A = 0.008969
MACAN_B1_T_B = 0.332


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
    # 标定口径：0910 方案B / B1 单表 t = 0.008969*idx + 0.332（模块常量 MACAN_B1_T_*），
    # 与 radard.py 的 A2 同源；旧 0902 插值查表与 0909 新线性公式（+1.0178 s 截距）均已废弃。
    # 2026-09-02 修复：去掉 and not self.CP.radarUnavailable —— MLB 的 dbc_dict 只有
    # Bus.pt（无 Bus.radar）→ interface.py:19 判定 radarUnavailable=True，但这只是"dbc没
    # 定义雷达总线"的误标，Macan 实际有 ACC_02/04 汇总信号可合成点。原条件导致
    # _update_macan 从未被调用（A3 点从未生成→radard tracks 恒空→get_lead 永远纯视觉
    # →radarState.leadOne.radar 恒 False，0066 实测 0%）。修复后 A3 点正常注入官方链路。
    self._macan_radar = CP.carFingerprint == "PORSCHE_MACAN_MK1"

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
    # can_capnp_to_list 返回两级结构 [(nanos, [(addr, dat, src), ...]), ...]
    # （2026-09-02 修复：此前按 capnp 对象 msg.dat 访问导致 card 崩溃 AttributeError）
    for _ts, frames in can_strings:
      for addr, dat, src in frames:
        if addr == 259 and len(dat) >= 8:
          # 四轮轮速 16|12 28|12 40|12 52|12 @1+ (0.1,0) km/h
          v_sum += (((dat[2] | (dat[3] << 8)) & 0xFFF)
                    + (((dat[3] >> 4) | (dat[4] << 4)) & 0xFFF)
                    + ((dat[5] | (dat[6] << 8)) & 0xFFF)
                    + (((dat[6] >> 4) | (dat[7] << 4)) & 0xFFF)) * 0.1
          v_cnt += 4
        elif src == 2:
          if addr == 780 and len(dat) >= 7:
            idx = (dat[3] | (dat[4] << 8)) & 0x3FF
          elif addr == 804 and len(dat) >= 7:
            v = ((dat[5] | (dat[6] << 8)) & 0x3FF) * 0.32  # km/h
            if v < 320:
              lead_spd = v
    if idx <= 0 or idx >= 1021:
      return super().update(None)  # 无有效目标 -> 空雷达（视觉兜底）
    if v_cnt == 0:
      return super().update(None)  # 无轮速 -> 无法算相对速度，保守返回空
    v_ego = v_sum / v_cnt * 0.2778 * self.CP.wheelSpeedFactor  # km/h -> m/s
    # Abstandsindex -> 时距 t -> 距离（0910 方案B / B1 单表直线，无人工分段）
    # 旧 idx<100 锚 0.8 s / idx>560 截顶 6.0 s 已删除：前者在 idx=100 处会造成
    # 0.8 -> 1.89 s 的突跳，且两者都是与 radard A2 反解不一致的阶梯源。
    t = MACAN_B1_T_A * idx + MACAN_B1_T_B
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
