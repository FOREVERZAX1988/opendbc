from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, CanBus, NetworkLocation, TransmissionType, GearShifter, \
                                                      CarControllerParams, VolkswagenFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.frame = 0
    self.eps_init_complete = False
    self.cruise_recovery_timer = 0
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.esp_hold_confirmation = False
    # 原厂 ACC_05 状态（bus2 雷达域 src=2），update_mlb 每帧刷新。
    # 默认 3（激活）避免 MLB 首帧前 carcontroller 读到 None 误降级。
    self.acc05_stock_status = 3
    self.upscale_lead_car_signal = False
    self.eps_stock_values = False
    self.acc_type = 0
    self.travel_assist_available = False
    # 车距档位（ACC_02 ACC_Gesetzte_Zeitluecke，37|3）：原厂共 4 档，ZL 值=格数
    # （1=最近/1格 ... 4=最远/4格），默认 3 格（用户确认：每次上车默认 3 格，习惯按+到 4 格）。
    # 由 LS_Verstellung_Zeitluecke(20|2) 按键增减，OP 代发 ACC_02 时传给仪表。
    self.stock_zeitluecke = 3
    self.zeitluecke_key_last = 0
    self.curvature_meas = 0.
    self.esp_laengsbeschl = 0.0  # 原厂 ESP 纵向加速度（ESP_02），坡度补偿 v2 原厂主源

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # Enable OP long on falling edge of enable buttons
        if b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
          return True
    return False

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    ext_cp = pt_cp if self.CP.networkLocation == NetworkLocation.fwdCamera else cam_cp
    alt_cp = can_parsers[Bus.alt]

    if self.CP.flags & VolkswagenFlags.PQ:
      return self.update_pq(pt_cp, cam_cp, ext_cp)
    elif self.CP.flags & VolkswagenFlags.MLB:
      return self.update_mlb(pt_cp, cam_cp, ext_cp, alt_cp)
    elif self.CP.flags & VolkswagenFlags.MEB:
      return self.update_meb(pt_cp, cam_cp, ext_cp)

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    if self.CP.transmissionType == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Motor_EV_01"]["MO_Waehlpos"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Gateway_73"]["GE_Fahrstufe"], None))

    if True:
      # MQB-specific
      if self.CP.flags & VolkswagenFlags.KOMBI_PRESENT:
        self.upscale_lead_car_signal = bool(pt_cp.vl["Kombi_03"]["KBI_Variante"])  # Analog vs digital instrument cluster

      self.parse_wheel_speeds(ret,
        pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
      )

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0  # EA

      brake_pedal_pressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
      brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
      ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
      ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well

      ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

      if self.CP.enableBsm:
        # Infostufe: BSM LED on, Warnung: BSM LED flashing
        ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
        ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

      ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
      ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

      self.acc_type = ext_cp.vl["ACC_06"]["ACC_Typ"]
      self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
      acc_limiter_mode = ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"] == 0
      speed_limiter_mode = bool(pt_cp.vl["TSK_06"]["TSK_Limiter_ausgewaehlt"])

      ret.cruiseState.available = pt_cp.vl["TSK_06"]["TSK_Status"] in (2, 3, 4, 5)
      ret.cruiseState.enabled = pt_cp.vl["TSK_06"]["TSK_Status"] in (3, 4, 5)
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS if self.CP.pcmCruise else 0
      ret.accFaulted = pt_cp.vl["TSK_06"]["TSK_Status"] in (6, 7)

      ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
      ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Shared logic
    ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS

    self.parse_mlb_mqb_steering_state(ret, pt_cp)

    ret.gasPressed = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] > 0
    ret.espActive = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    ret.standstill = ret.vEgoRaw == 0
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    ret.cruiseState.nonAdaptive = acc_limiter_mode or speed_limiter_mode
    if ret.cruiseState.speed > 90:
      ret.cruiseState.speed = 0

    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    # Macan(MLB) 巡航拨杆：按 SET 时 LS_01 bit16(SET)+bit17(Hoch/+) 同时置位 → 产生
    # accelCruise 事件 → selfdrived 误判 resume_pressed → vCruise>250 → resumeBlocked
    # (NO_ENTRY "Press Set to Engage") → 无法接合。同帧出现 setCruise 时过滤 accelCruise；
    # 单独按 +/-（只有 bit17/18）不受影响，功能保留。
    # Macan(MLB) 巡航拨杆：SET 与 + 是同一物理键（bit16+bit17 同置位）→ 同帧产生
    # setCruise+accelCruise。原厂语义按巡航状态分发（用户路试确认）：
    #   未激活：该键=SET（设速+接合）→ 保留 setCruise，滤 accelCruise（防 resumeBlocked）
    #   激活中：该键=+（巡航速度+1）→ 保留 accelCruise，滤 setCruise
    if any(b.type == ButtonType.setCruise for b in button_events):
      if ret.cruiseState.enabled:
        button_events = [b for b in button_events if b.type != ButtonType.setCruise]
      else:
        button_events = [b for b in button_events if b.type != ButtonType.accelCruise]
    ret.buttonEvents = button_events

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret, ret_sp

  def update_pq(self, pt_cp, cam_cp, ext_cp) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # vEgo obtained from Bremse_1 vehicle speed rather than Bremse_3 wheel speeds because Bremse_3 isn't present on NSF
    ret.vEgoRaw = pt_cp.vl["Bremse_1"]["BR1_Rad_kmh"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # Update gas, brakes, and gearshift.
    ret.gasPressed = pt_cp.vl["Motor_3"]["MO3_Pedalwert"] > 0
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["MO2_BLS"])
    ret.parkingBrake = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])

    # Update gear and/or clutch position data.
    if self.CP.transmissionType == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_1"]["GE1_Wahl_Pos"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_BT_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HL_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HR_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HD_Hauptraste"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_Status"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapters on Front Assist with Braking and City Emergency
    # Braking for the 2016 Passat NMS
    # TODO: deferred until we can collect data on pre-MY2016 behavior, AWV message may be shorter with fewer signals
    ret.stockFcw = False
    ret.stockAeb = False

    # Update ACC radar status.
    self.acc_type = ext_cp.vl["ACC_System"]["ACS_Typ_ACC"]
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["MO5_GRA_Hauptsch"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["MO2_Sta_GRA"] in (1, 2)
    if self.CP.pcmCruise:
      ret.accFaulted = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_StaACC"] in (6, 7)
    else:
      ret.accFaulted = pt_cp.vl["Motor_2"]["MO2_Sta_GRA"] == 3

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"],
                                                                            pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    # Macan(MLB) 巡航拨杆：按 SET 时 LS_01 bit16(SET)+bit17(Hoch/+) 同时置位 → 产生
    # accelCruise 事件 → selfdrived 误判 resume_pressed → vCruise>250 → resumeBlocked
    # (NO_ENTRY "Press Set to Engage") → 无法接合。同帧出现 setCruise 时过滤 accelCruise；
    # 单独按 +/-（只有 bit17/18）不受影响，功能保留。
    if any(b.type == ButtonType.setCruise for b in button_events):
      button_events = [b for b in button_events if b.type != ButtonType.accelCruise]
    ret.buttonEvents = button_events
    self.gra_stock_values = pt_cp.vl["GRA_Neu"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["BR1_ESPASR_passive"])

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret, ret_sp

  def update_meb(self, pt_cp, cam_cp, ext_cp) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ESC_51"]["VL_Radgeschw"],
      pt_cp.vl["ESC_51"]["VR_Radgeschw"],
      pt_cp.vl["ESC_51"]["HL_Radgeschw"],
      pt_cp.vl["ESC_51"]["HR_Radgeschw"],
    )
    if self.CP.flags & VolkswagenFlags.KOMBI_PRESENT:
      ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE, 5)
    self.curvature_meas = -pt_cp.vl["QFK_01"]["Curvature"] * (1, -1)[int(pt_cp.vl["QFK_01"]["Curvature_VZ"])]
    ret.yawRate = -pt_cp.vl["ESC_50"]["Yaw_Rate"] * (1, -1)[int(pt_cp.vl["ESC_50"]["Yaw_Rate_Sign"])] * CV.DEG_TO_RAD

    if self.CP.flags & VolkswagenFlags.ALT_GEAR:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Gateway_73"]["GE_Fahrstufe"], None))
    else:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    drive_mode = ret.gearShifter == GearShifter.drive

    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["QFK_01"]["LatCon_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status, drive_mode)

    ret.carFaultedNonCritical = cam_cp.vl["EA_01"]["EA_Funktionsstatus"] in (3, 4, 5, 6)

    ret.gasPressed = pt_cp.vl["Motor_51"]["Accel_Pedal_Pressure"] > 0
    ret.brakePressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
    ret.parkingBrake = pt_cp.vl["ESC_50"]["EPB_Status"] in (1, 4) # EPB closing or closed (candidate for all platforms)
    #ret.parkingBrake = pt_cp.vl["Gateway_73"]["EPB_Status"] in (1, 4) # this signal is not working for newer models
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3
    doors = pt_cp.vl["ZV_02"] if bool(pt_cp.vl["Gateway_72"]["ZV_02_alt"]) else pt_cp.vl["Gateway_72"]
    ret.doorOpen = any([doors["ZV_FT_offen"],
                        doors["ZV_BT_offen"],
                        doors["ZV_HFS_offen"],
                        doors["ZV_HBFS_offen"],
                        doors["ZV_HD_offen"]])
    ret.espDisabled = bool(pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"])
    ret.espActive = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])

    self.acc_type = ext_cp.vl["ACC_18"]["ACC_Typ"]
    self.esp_hold_confirmation = bool(pt_cp.vl["ESC_50"]["Standstill"])
    self.travel_assist_available = bool(cam_cp.vl["TA_01"]["Travel_Assist_Available"])
    ret.stockFcw = bool(ext_cp.vl["AWV_03"]["FCW_Active"])

    ret.cruiseState.available = pt_cp.vl["Motor_51"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled = pt_cp.vl["Motor_51"]["TSK_Status"] in (3, 4, 5)
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    if self.CP.pcmCruise:
      ret.cruiseState.nonAdaptive = bool(ext_cp.vl["ACC_19"]["ACC_Limiter_Mode"])
      ret.cruiseState.speed = ext_cp.vl["ACC_19"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:  # 255 kph in m/s == no current setpoint
        ret.cruiseState.speed = 0
    else:
      ret.cruiseState.nonAdaptive = bool(pt_cp.vl["Motor_51"]["TSK_Limiter_ausgewaehlt"])
    accFaulted = (pt_cp.vl["Motor_51"]["TSK_Status"] in (6, 7) or
                  ext_cp.vl["ACC_19"]["ACC_Status_ACC"] == 6)  # reversible fault in ACC system
    ret.accFaulted = self.update_acc_fault(accFaulted, parking_brake=ret.parkingBrake, drive_mode=drive_mode,
                                            brake_pressed=ret.brakePressed)

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(240, pt_cp.vl["SMLS_01"]["BH_Blinker_li"],
                                                                            pt_cp.vl["SMLS_01"]["BH_Blinker_re"])

    if self.CP.enableBsm:
      if self.CP.flags & VolkswagenFlags.MEB_GEN2:
        ret.leftBlindspot = (bool(pt_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Driver"]) or
                             bool(pt_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Driver"]))
        ret.rightBlindspot = (bool(pt_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Passenger"]) or
                              bool(pt_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Passenger"]))
      else:
        ret.leftBlindspot = (bool(ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Left"]) or
                             bool(ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Left"]))
        ret.rightBlindspot = (bool(ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Right"]) or
                              bool(ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Right"]))

    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]
    self.klr_stock_values = pt_cp.vl["KLR_01"] if self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT else {}

    button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    # Macan(MLB) 巡航拨杆：按 SET 时 LS_01 bit16(SET)+bit17(Hoch/+) 同时置位 → 产生
    # accelCruise 事件 → selfdrived 误判 resume_pressed → vCruise>250 → resumeBlocked
    # (NO_ENTRY "Press Set to Engage") → 无法接合。同帧出现 setCruise 时过滤 accelCruise；
    # 单独按 +/-（只有 bit17/18）不受影响，功能保留。
    if any(b.type == ButtonType.setCruise for b in button_events):
      button_events = [b for b in button_events if b.type != ButtonType.accelCruise]
    ret.buttonEvents = button_events
    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret, ret_sp

  def update_mlb(self, pt_cp, cam_cp, ext_cp, alt_cp) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    # 原厂 ESP 纵向加速度（ESP_02@257 ESP_Laengsbeschl 24|10 scale0.03125 offset-16）——坡度补偿 v2 原厂主源
    try:
      self.esp_laengsbeschl = pt_cp.vl["ESP_02"]["ESP_Laengsbeschl"]
    except Exception:
      pass
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw"],
    )

    ret.gasPressed = pt_cp.vl["Motor_03"]["MO_Fahrpedalrohwert_01"] > 0
    ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(alt_cp.vl["Getriebe_03"]["GE_Waehlhebel"], None))

    # TODO: We don't have a true mainswitch state yet, might need stateful tracking on LS_01 if momentary-press is a thing
    # TSK_04.TSK_Status_GRA_ACC_02 0 = not engaged, 1 = engaged, 2 = engaged with driver accel override, 3 = fault

    #ret.cruiseState.available = alt_cp.vl["TSK_04"]["TSK_Status_GRA_ACC_02"] in (0, 1, 2)
    #ret.cruiseState.available = alt_cp.vl["TSK_04"]["TSK_Status_GRA_ACC_02"] in (1, 2)
    ret.cruiseState.available = bool(pt_cp.vl["LS_01"]["LS_Hauptschalter"])
    ret.cruiseState.enabled = alt_cp.vl["TSK_04"]["TSK_Status_GRA_ACC_02"] in (1, 2)
    ret.accFaulted = alt_cp.vl["TSK_04"]["TSK_Status_GRA_ACC_02"] == 3
    # 原厂 ACC_05 状态（bus2 雷达域 src=2）：OP 激活期间若原厂已撤力矩/退出（st∉(3,4)），
    # OP 必须同步降级，否则「OP st=3 + 原厂已退出」矛盾窗口会让 ECU 写 DTC 锁死 ACC
    # （00000033 seg0: 309.68s 原厂 Mom146→0、309.86s st3→2，OP 仍 st=3 → 310.19s accFaulted）。
    # 正常激活时原厂 src=2 st=3（287-307s 实测一致），仅原厂退出时触发降级。
    self.acc05_stock_status = int(ext_cp.vl["ACC_05"]["ACC_Status_ACC"])
    # 原厂 ACC_05 减速/停车意图（bus2 雷达域 src=2）：00000037/00000038 根因修复——
    # OP 代发加速请求与原厂雷达自身减速/停车请求矛盾 1.3~5s → 雷达自检失败报 st=6 →
    # ECU 写 DTC 锁死 ACC。OP 激活期间若原厂在请求减速（ACC_Verz_anf<0）或停车
    # （ACC_Anhalten=1），carcontroller 仲裁逻辑将禁止 OP 正加速，只能比原厂保守。
    self.acc05_stock_verz = float(ext_cp.vl["ACC_05"]["ACC_Verz_anf"])
    # 原厂力矩请求（ACC_Momentenanforderung，10bit 0-1021）：00000038 实锤——雷达的减速意图
    # 有两种表达：verz<0/anh=1（停车请求）与 mom 下降/归零（撤动力）。00000038@688.4s 雷达
    # mom 114→26→0 持续撤力，OP 无视继续拉 92→152 猛加速 → 雷达自检失败 st=6 → DTC 锁死。
    # 723f0b8f 只仲裁 verz/anh 拦不住该场景；此处透传 mom 供 carcontroller 做撤力仲裁。
    self.acc05_stock_mom = float(ext_cp.vl["ACC_05"]["ACC_Momentenanforderung"])
    self.acc05_stock_anhalten = bool(ext_cp.vl["ACC_05"]["ACC_Anhalten"])
    # 原厂 ACC_05 是否请求 ESP 介入（ACC_Beeinflussung_ESP，bus2 雷达域 src=2）：
    # 0000003f 实锤——原厂跟停（Verz=-2/Anh=1）全程 ESP=0（靠1挡怠速拖滞）；
    # OP 旧逻辑在仲裁把 accel 压到 stock_verz(-2) 后误触发 accel<-1 的 ESP 条件
    # → OP 代发 ESP=1 与原厂 ESP=0 矛盾（白耗液压预充，存在雷达 st6 锁死隐患）。
    # 修复：透传原厂 ESP 位，完全复刻原厂行为。
    self.acc05_stock_esp = bool(ext_cp.vl["ACC_05"]["ACC_Beeinflussung_ESP"])
    # 原厂 ACC_05 力矩/减速通道许可（ACC_Freigabe_Momentenanf / ACC_Freigabe_Verzanf）：
    # 00000041 seg7 实锤——原厂撤力（mom 118→0）后切 FV=1 减速通道（verz 缓降到 -0.06），
    # 旧仲裁只盯 verz<-0.15（从未触发），OP 继续发 FM=1 力矩 → 方向性矛盾 → 雷达 st6 →
    # TSK_04 st02 1→0 → controlsMismatch。透传通道位供 carcontroller 做「撤力跟随」。
    self.acc05_stock_fm = bool(ext_cp.vl["ACC_05"]["ACC_Freigabe_Momentenanf"])
    self.acc05_stock_fv = bool(ext_cp.vl["ACC_05"]["ACC_Freigabe_Verzanf"])
    # 原厂 ACC_02 目标车显示字段（bus2 雷达域 src=2）：OP 代发 ACC_02 到 bus0 时若
    # ACC_Abstandsindex/ACC_Relevantes_Objekt 恒 0，仪表盘永远显示「无目标」——
    # 即使 OP/雷达已捕捉到前车也不显示车距图标/三档距离（00000037/38 路试反馈）。
    # 修复：透传原厂雷达的目标车距离/目标存在，驱动仪表 ACC 车距显示。
    self.stock_lead_distance = int(ext_cp.vl["ACC_02"]["ACC_Abstandsindex"])
    self.stock_lead_object = int(ext_cp.vl["ACC_02"]["ACC_Relevantes_Objekt"])
    # 原厂 ACC_02 的 HUD 跟车主状态（ACC_Status_Prim_Anz，22|2，bus2 雷达域 src=2）：
    # OP 代发 ACC_02 时透传原厂值——旧逻辑重算(1 if acc_hud_status==3 else 0)在踩油门时
    # acc_hud_status 被推成4(超驰)→发0，而原厂 st=3(激活跟车)发1 → ACC_02 状态矛盾
    # → 原厂检测"HUD状态报文被改写"→ACC自检失败 st=6（00000053 seg6/7 实锤，2026-08-22）。
    self.stock_prim_anz = int(ext_cp.vl["ACC_02"]["ACC_Status_Prim_Anz"])
    # 原厂 ACC_02 其余 HUD 状态字段（bus2 雷达域 src=2）：OP 代发 ACC_02 时透传，避免
    # 重算/默认0 ≠ 原厂 → ACC_02 状态矛盾 → st=6（00000053 seg0/6/7 实锤，2026-08-22）：
    # - ACC_Status_Anzeige(61|3)：原厂激活=3/故障=6，旧逻辑用 acc_hud_status 重算（踩油门=4）
    # - ACC_Texte_Primaeranz(48|7)：原厂故障时=1（故障文本），OP 默认0 丢失
    # - ACC_Display_Prio(44|2)：原厂按 ab 判定 2/3，OP 按视觉 lead_obj → 相反
    self.stock_status_anzeige = int(ext_cp.vl["ACC_02"]["ACC_Status_Anzeige"])
    self.stock_texte_prim = int(ext_cp.vl["ACC_02"]["ACC_Texte_Primaeranz"])
    self.stock_display_prio = int(ext_cp.vl["ACC_02"]["ACC_Display_Prio"])
    # 原厂 ACC_04 目标车速度（km/h）：OP 代发 ACC_04 时透传，仪表显示目标车速度
    self.stock_lead_speed_kph = float(ext_cp.vl["ACC_04"]["ACC_Geschw_Zielfahrzeug"]) if self.CP.openpilotLongitudinalControl else 327.36
    ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS

    self.parse_mlb_mqb_steering_state(ret, pt_cp)

    brake_pedal_pressed = bool(pt_cp.vl["Motor_03"]["MO_Fahrer_bremst"])
    brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    # 轻踩刹车检测（00000033 实锤）：MO_BLS(34|1 制动灯开关) 轻踩即亮（309.737s 变1），
    # 而 MO_Fahrer_bremst/ESP_Fahrer_bremst 在 ESP_Bremsdruck<1.8bar 时仍=0 → OP 漏检
    # 轻踩刹车 → 保持 st=3 → 「刹车+ACC激活」矛盾窗口 → ECU 写 DTC 锁死 ACC/PAS。
    # BLS 为物理踏板开关，ACC 自动制动（ECD_Bremslicht 点灯）不触发，可安全区分驾驶员介入。
    brake_light_switch = bool(pt_cp.vl["Motor_03"]["MO_BLS"])
    ret.brakePressed = brake_pedal_pressed or brake_pressure_detected or brake_light_switch
    ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])
    ret.espDisabled = pt_cp.vl["ESP_01"]["ESP_Tastung_passiv"] != 0

    ret.leftBlinker = bool(pt_cp.vl["BCM"]["BLINKER_LEFT"])
    ret.rightBlinker = bool(pt_cp.vl["BCM"]["BLINKER_RIGHT"])

    ret.seatbeltUnlatched = bool(pt_cp.vl["Airbag_01"]["AB_Gurtwarn_VF"])
    ret.doorOpen = any([alt_cp.vl["Gateway_05"]["FT_Tuer_geoeffnet"],
                        alt_cp.vl["Gateway_05"]["BT_Tuer_geoeffnet"],
                        alt_cp.vl["Gateway_05"]["HL_Tuer_geoeffnet"],
                        alt_cp.vl["Gateway_05"]["HR_Tuer_geoeffnet"]])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["LS_01"]

    # 车距键消费：LS_Verstellung_Zeitluecke(20|2) 非 0 时是"按下"，边沿触发 ±1 档。
    # 值 1=减小（-）、值 2=增大（+）——方向如路试不符可对调。OP 代发 ACC_02 时
    # 通过 CS.stock_zeitluecke 驱动仪表 1-4 格显示（原厂默认 ZL=4 → 3 格）。
    zl_key = pt_cp.vl["LS_01"]["LS_Verstellung_Zeitluecke"]
    zl_edge = 0
    if zl_key != 0 and self.zeitluecke_key_last == 0:
      zl_edge = zl_key
      if zl_key == 1:
        self.stock_zeitluecke = max(1, self.stock_zeitluecke - 1)
      elif zl_key == 2:
        self.stock_zeitluecke = min(4, self.stock_zeitluecke + 1)
    self.zeitluecke_key_last = zl_key

    button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    # Macan(MLB) 巡航拨杆：按 SET 时 LS_01 bit16(SET)+bit17(Hoch/+) 同时置位
    # （route 00000004--915ebf086f seg6 实测：严格同帧置位/清零）→ 产生 accelCruise 事件
    # → selfdrived 误判 resume_pressed → vCruise>250 → resumeBlocked (NO_ENTRY "Press Set to Engage")
    # → 无法接合。过滤条件：同帧出现 setCruise 或 LS_Tip_Setzen 当前按下（防个别帧 bit17 先置位）；
    # 单独按 +/-（只有 bit17/18，bit16=0）不受影响，功能保留。
    if any(b.type == ButtonType.setCruise for b in button_events) or pt_cp.vl["LS_01"]["LS_Tip_Setzen"]:
      button_events = [b for b in button_events if b.type != ButtonType.accelCruise]
    # 车距键补发 buttonEvents（2026-08-13 修复）：MLB BUTTONS 映射指向 GRA_Neu.GRA_Zeitluecke
    # （值3），与 Macan 实际信号 LS_01.LS_Verstellung_Zeitluecke（20|2，值1=拉近/2=拉远）不匹配
    # → create_button_events 收不到距离键 → selfdrived 的驾驶风格融合（车距档→personality，
    # selfdrived.py 读 gapAdjustCruise/altButton2 跟踪 _zeitluecke）失效、无 UI 提示。
    # 这里边沿触发补发：值1→gapAdjustCruise(-1格)、值2→altButton2(+1格)，与融合逻辑对齐。
    # 单帧 pressed=True（按住期间不重复发），不会触发 altButton2 长按切 experimental。
    if zl_edge:
      be = structs.CarState.ButtonEvent()
      be.type = ButtonType.gapAdjustCruise if zl_edge == 1 else ButtonType.altButton2
      be.pressed = True
      button_events.append(be)
    ret.buttonEvents = button_events

    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    ret.standstill = ret.vEgoRaw == 0

    self.frame += 1
    return ret, ret_sp

  def update_low_speed_alert(self, v_ego: float) -> bool:
    # Low speed steer alert hysteresis logic
    if (self.CP.minSteerSpeed - 1e-3) > CarControllerParams.DEFAULT_MIN_STEER_SPEED and v_ego < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif v_ego > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    return self.low_speed_alert

  def parse_mlb_mqb_steering_state(self, ret, pt_cp, drive_mode=True):
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE

    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status, drive_mode)
    return

  def update_hca_state(self, hca_status, drive_mode=True):
    # Treat FAULT as temporary for worst likely EPS recovery time, for cars without factory Lane Assist
    # DISABLED means the EPS hasn't been configured to support Lane Assist
    self.eps_init_complete = self.eps_init_complete or (hca_status in ("DISABLED", "READY", "ACTIVE") or self.frame > 600)
    perm_fault = drive_mode and hca_status == "DISABLED" or (self.eps_init_complete and hca_status == "FAULT")
    temp_fault = drive_mode and hca_status in ("REJECTED", "PREEMPTED") or not self.eps_init_complete
    return temp_fault, perm_fault

  def update_acc_fault(self, acc_fault, parking_brake=False, drive_mode=True, brake_pressed=False, recovery_frames_max=300):
    # Ignore FAULT when not in drive mode and parked
    # do not show misleading error during ignition in parked state
    # grant a short time to recover a normal cruise state
    # after hard brake, stock system prevents acc engage for ~3 seconds
    fault = acc_fault
    if (parking_brake and not drive_mode) or brake_pressed:
      fault = False
      self.cruise_recovery_timer = self.frame
    elif self.frame - self.cruise_recovery_timer < recovery_frames_max:
      fault = False
    return fault

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_can_parsers_pq(CP)
    elif CP.flags & VolkswagenFlags.MEB:
      return CarState.get_can_parsers_meb(CP)

    # manually configure some optional and variable-rate/edge-triggered messages
    pt_messages, cam_messages, alt_messages = [], [], []

    if not CP.flags & VolkswagenFlags.MLB:
      pt_messages += [
        ("Blinkmodi_02", 1)  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ]
    if CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      cam_messages += [
        ("HCA_01", 1),  # From R242 Driver assistance camera, 50Hz if steering/1Hz if not
      ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], alt_messages, CanBus(CP).alt),
    }

  @staticmethod
  def get_can_parsers_pq(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).cam),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).alt),
    }

  @staticmethod
  def get_can_parsers_meb(CP):
    pt_messages = [
      # frequency changes too much for the CANParser to figure out
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("SMLS_01", 1),       # From Stalk Controls
    ]
    if CP.networkLocation == NetworkLocation.fwdCamera:
      pt_messages.append(("AWV_03", 1)) # Front Collision Detection (1 Hz when inactive, 50 Hz when active)

    cam_messages = []
    if CP.networkLocation == NetworkLocation.gateway:
      cam_messages.append(("AWV_03", 1)) # Front Collision Detection (1 Hz when inactive, 50 Hz when active)

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).alt),
    }
