from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.radar_interface import RadarInterface
from opendbc.car.volkswagen.values import CanBus, CAR, DBC, NetworkLocation, TransmissionType, VolkswagenFlags, VolkswagenSafetyFlags

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.eco, structs.CarState.GearShifter.sport,
                    structs.CarState.GearShifter.manumatic)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volkswagen"
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    if ret.flags & VolkswagenFlags.PQ:
      # Set global PQ35/PQ46/NMS parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenPq)]
      ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1

      if 0x440 in fingerprint[0] or docs:  # Getriebe_1
        ret.transmissionType = TransmissionType.automatic
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x1A0, 0xC2)):  # Bremse_1, Lenkwinkel_1
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      ret.dashcamOnly = is_release  # Release support needs HCA timeout fix, safety validation

    elif ret.flags & VolkswagenFlags.MLB:
      # Set global MLB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMlb)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01
      ret.networkLocation = NetworkLocation.gateway
      ret.dashcamOnly = is_release  # Release support needs HCA timeout fix, safety validation, revised J533 harness

    elif ret.flags & VolkswagenFlags.MEB:
      # Set global MEB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMeb)]
      if ret.flags & VolkswagenFlags.MEB_GEN2:
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.MEB_ALT_CRC.value

      ret.transmissionType = TransmissionType.direct
      ret.steerControlType = structs.CarParams.SteerControlType.curvature
      ret.steerAtStandstill = True

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP = [10., 40.]
      ret.lateralTuning.pid.kpV = [0., 1.45]
      ret.lateralTuning.pid.kiBP = [10., 40.]
      ret.lateralTuning.pid.kiV = [0., 0.12]
      ret.lateralTuning.pid.kf = 1.

      if any(msg in fingerprint[1] for msg in (0x520, 0x86, 0xFD, 0x13D)):  # Airbag_02, LWI_01, ESP_21, QFK_01
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera
        ret.radarUnavailable = True

      ret.enableBsm = 0x24C in fingerprint[0]  # MEB_Side_Assist_01

      if 0x25D in fingerprint[0]:  # KLR_01
        ret.flags |= VolkswagenFlags.STOCK_KLR_PRESENT.value
      if 0x3DC in fingerprint[0]:  # Gateway_73
        ret.flags |= VolkswagenFlags.ALT_GEAR.value

      # only allow gateway harness to escalate Emergency Assist
      ret.dashcamOnly = ret.networkLocation == NetworkLocation.fwdCamera

    else:
      # Set global MQB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0] or docs:  # Getriebe_11
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # Motor_EV_01
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if 0x126 in fingerprint[2]:  # HCA_01
        ret.flags |= VolkswagenFlags.STOCK_HCA_PRESENT.value
      if 0x6B8 in fingerprint[0]:  # Kombi_03
        ret.flags |= VolkswagenFlags.KOMBI_PRESENT.value

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 0.4
    if ret.flags & VolkswagenFlags.PQ or ret.flags & VolkswagenFlags.MLB:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif ret.flags & VolkswagenFlags.MEB:
      ret.steerActuatorDelay = 0.3
    else:
      ret.steerActuatorDelay = 0.1
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    if ret.flags & VolkswagenFlags.MEB:
      ret.longitudinalActuatorDelay = 0.5
      ret.longitudinalTuning.kiBP = [0., 30.]
      ret.longitudinalTuning.kiV = [0.4, 0.]

    ret.alphaLongitudinalAvailable = ret.networkLocation == NetworkLocation.gateway or docs
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      safety_configs[0].safetyParam |= VolkswagenSafetyFlags.LONG_CONTROL.value
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    # Per-vehicle overrides

    if candidate == CAR.PORSCHE_MACAN_MK1:
      ret.steerActuatorDelay = 0.07

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.stopAccel = -0.55
    ret.autoResumeSng = ret.minEnableSpeed == -1

    CAN = CanBus(fingerprint=fingerprint)
    if CAN.pt >= 4:
      safety_configs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = safety_configs

    # Macan 转向系数开关（实验值18.0/0.52，跨route标定极差22%不可靠，默认关等路试修正）
    if candidate == CAR.PORSCHE_MACAN_MK1:
      try:
        from openpilot.common.params import Params
        if Params().get_bool("MacanSteerParams"):
          ret.steerRatio = 16.2  # v2标定(ESP_Gierrate,4route极差2.6%): 15.24/16.05/17.23/14.65中位≈15.7,与原厂16.2一致; 旧实验值18.0(gyro极差22%)偏大15%→城市弯转向不足
          # 动态转向比（速度相关）：4f全段29284样本拟合——<144km/h SR≈15.0（36-72:14.98/72-144:15.01）、
          # >144km/h SR≈18.71（RMSE1.75°）——保时捷PDS/EPS随速特性实锤（早前v7标定高速段22.9即此）。
          # 格式：[速度km/h..., 比值...]（前一半速度、后一半比值）；140-145km/h线性过渡；
          # 当前16.2为折中（中速偏大8%转向不足、高速偏小13%转向过度——高速段由V2修正）。
          ret.steerRatioV2 = [0.0, 140.0, 145.0, 200.0, 15.0, 15.0, 18.7, 18.7]
          if ret.lateralTuning.which() == 'torque':
            ret.lateralTuning.torque.friction = 0.52
      except Exception:
        pass
    return ret
