from opendbc.car.volkswagen.mqbcan import volkswagen_mqb_meb_checksum, xor_checksum

def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  values = {s: ldw_stock_values[s] for s in [
    "LDW_SW_Warnung_links",
    "LDW_SW_Warnung_rechts",
    "LDW_Seite_DLCTLC",
    "LDW_Text",
  ]}
  values["LDW_Status_LED_gruen"] = enabled
  values["LDW_Status_LED_gelb"] = not enabled
  values["LDW_Gong"] = hud_alert
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  """ACC_Status_ACC state machine.
  Values observed from stock Macan ACC_05:
    0 = ACC off (main switch off)
    2 = ACC ready/standby (main on, no active control)
    3 = ACC active (regulating speed/distance)
    4 = ACC active + driver override (gas pedal pressed)
    5 = ACC decelerating only (brake intervention)
    6 = ACC fault (reversible)
    7 = ACC fault (permanent)
  """
  if acc_faulted:
    return 6
  elif long_active:
    return 3
  elif main_switch_on:
    return 2
  else:
    return 0


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold,
                             v_ego=0.0, engine_torque=0.0):
  """Create ACC_05 + ACC_01 messages for longitudinal control on MLB platform.

  Ported from oscarmcnulty's vw-mlb-2026-06 (verified on 2014 Audi Q5 MK1 3.0T +
  Porsche Macan MK1), which resolves the MLB ACC ECU acceleration lockout
  (accel works for a while then dies, brake-only still works, stock ACC also
  fails after, only recovers after ignition cycle).

  Why this differs from the previous torque-model implementation:
    1. ACC_01 carries ACC_Sollbeschleunigung (desired accel) as the PRIMARY request
       channel -- the ECU computes its own torque from it. This eliminates the
       "requested torque vs actual accel mismatch -> TSK/ESP consistency lockout".
    2. ACC_05 ACC_Momentenanforderung is a simple int(accel*100) mapping. No physics
       torque formula, no dependence on gear_ratio / grade / load.
    3. ACC_KD_Fehler = 1 (matches stock radar: verified from Macan drive logs that
       the factory radar sends 1 in BOTH standby and active states; DBC VAL_ comment
       "1=fault" is misleading for this bit - it is a module-present/valid bit).
       (oscar used 0 on Q5, but Macan D4 ECU may treat 0 as "ACC not participating".)
    4. ACC_ax_Getriebe passes accel through, clamped to DBC range to prevent the
       unsigned wrap (verified fix, kept from previous implementation).
  """
  commands = []

  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Freigabe_Momentenanf": 1 if accel > 0 else 0,       # increased acceleration requested?
    "ACC_Freigabe_Verzanf": 1 if accel < 0 else 0,           # decreased acceleration requested?
    "ACC_Getriebestellung_P": 0,
    "ACC_limitierte_Anfahrdyn": 0,
    "ACC_Momentenanforderung": int(accel * 100) if accel > 0 else 0,  # "torque requested"
    "ACC_zul_Regelabw": 0,
    "ACC_Verz_anf": accel if accel < 0 else 0,               # brake accel requested (ESP)
    "ACC_Loeseanforderung": starting,                        # 1 when starting again from stop
    "ACC_StartStopp_Info": acc_enabled,                       # 1 while active (stock: 1 in cruise, 0 in standby)
    "ACC_Vorbefuellung_Bremsanlage": 1 if accel < 0 else 0,
    "ACC_ax_Getriebe": max(-2.016, min(10.248, accel)),      # accel hint for TCU (clamped!)
    "ACC_Betaetigung_EPB": 0,
    "ACC_Beeinflussung_ESP": 0,
    "ACC_Anhalten": stopping,
    "ACC_KD_Fehler": 1,                                      # 1 = stock radar constant (verified from Macan drive logs: 1 in standby AND active)
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  acc_01_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Sollbeschleunigung": accel if acc_enabled else 0,
    "ACC_zul_Regelabw_unten": 0.2,
    "ACC_zul_Regelabw_oben": 0.2,
    "ACC_neg_Sollbeschl_Grad": 4.0 if acc_enabled else 0,    # jerk limit, must match carcontroller
    "ACC_pos_Sollbeschl_Grad": 4.0 if acc_enabled else 0,
    "ACC_Dynamik": 2,
    "ACC_Anhalten": stopping,
    "ACC_Minimale_Bremsung": stopping,
  }
  commands.append(packer.make_can_msg("ACC_01", bus, acc_01_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  """Create ACC_02 HUD display message. Mirrors stock radar signals for cluster display."""
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance if 0 < distance < 5 else 3,
    "ACC_Display_Prio": 2 if lead_distance > 0 else 3,
    "ACC_Abstandsindex": min(lead_distance, 1021) if lead_distance > 0 else 0,
    "ACC_Relevantes_Objekt": 1 if 0 < lead_distance < 1000 else 0,
  }
  return packer.make_can_msg("ACC_02", bus, values)


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08,  # ACC_01
    0x10E: 0x0F,  # TSK_04
    0x111: 0x10,  # TSK_05
    0x30C: 0x0F,  # ACC_02
    0x324: 0x27,  # ACC_04
    0x10B: 0x0A,  # LS_01
    0x10D: 0x0C,  # ACC_05
    0x10F: 0x0E,  # ACC_0x10F
    0x311: 0x12,  # ACC_0x311
    0x397: 0x94,  # LDW_02
    0x10C: 0x0D,  # TSK_02
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return volkswagen_mqb_meb_checksum(address, sig, d)
