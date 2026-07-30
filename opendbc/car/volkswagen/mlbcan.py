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
  """Create ACC_05 message for longitudinal control on MLB platform.

  ACC_05 is the master acceleration request from ACC to engine (torque request),
  transmission (gear selection hint via ax_Getriebe), and ESP (brake request).

  Physics-based additive torque control:

    acc_moment = cruise_torque(v) + accel * torque_gain

  cruise_torque: Quadratic fit to real Macan steady-state engine torque data.
                 This is the torque needed to hold speed on flat ground.

  torque_gain: Converts m/s² to Nm. Derived from: mass * wheel_radius.
               Fixed gain of ~720 Nm/(m/s²) → conservative, gentle launches.

  Key bug fixes vs earlier implementations:
    1. ACC_Verz_anf = 0 (not -2.0) at standstill — prevents false decel request
    2. ACC_KD_Fehler = 0 (not 1) — tells PDK/ESP no fault, acceleration allowed
    3. ACC_ax_Getriebe clamped to [-2.016, 10.248] — prevents unsigned overflow
  """
  commands = []

  # Determine braking state
  if acc_enabled:
    braking = (accel < -0.4) or stopping or (v_ego < 2.0 and accel <= 0)
  else:
    braking = False

  if acc_enabled and not braking:
    # Physics-based torque model
    # cruise_torque(v) = 0.064 * v² + 2.16 * v + 46.0  (quadratic fit to MO_Mom_Ist)
    cruise_torque = 0.064 * v_ego * v_ego + 2.16 * v_ego + 46.0

    # Fixed torque gain (Nm per m/s² of acceleration)
    # Derived from: mass * wheel_radius = 2000 * 0.36 = 720
    torque_gain = 720.0

    acc_moment = int(max(0, min(500, cruise_torque + accel * torque_gain)))
  else:
    acc_moment = 0

  # ACC_Verz_anf: deceleration request to ESP
  # IMPORTANT: At standstill, send 0, NOT -2.0!
  # Sending -2.0 with a positive torque request causes ESP_Konsistenz_TSK fault.
  # Only send decel request when genuinely braking.
  if acc_enabled and braking:
    verz_anf = max(accel, -3.5)
  elif acc_enabled:
    verz_anf = 0.0
  else:
    verz_anf = 0.0

  # ACC_ax_Getriebe: expected acceleration signal for transmission (gear selection hint)
  # DBC: [-2.016, +10.248]. 9 bits unsigned, offset -2.016.
  # Values below -2.016 WRAP around to ~+10.248 (unsigned overflow)!
  # This confuses the TCU — always clamp!
  if acc_enabled and not braking:
    if accel > 0.25:
      ax_getriebe = min(accel, 1.3)
    elif accel < -0.25:
      ax_getriebe = max(accel, -0.5)
    else:
      ax_getriebe = 0.0
  elif acc_enabled and braking:
    # Speed-dependent negative hint
    ax_getriebe = max(accel, -2.016)
  else:
    ax_getriebe = 0.0

  # Clamp to DBC range to prevent overflow
  ax_getriebe = max(-2.016, min(10.248, ax_getriebe))

  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Verz_anf": verz_anf,
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0,
    "ACC_ax_Getriebe": ax_getriebe,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    "ACC_Beeinflussung_ESP": 1 if (stopping or esp_hold or (braking and accel < -1.0)) else 0,
    "ACC_StartStopp_Info": 1 if acc_enabled else 0,
    "ACC_Anhalten": 1 if stopping else 0,
    "ACC_Betaetigung_EPB": 1 if esp_hold else 0,
    "ACC_KD_Fehler": 0,  # CRITICAL: 0 = no fault, acceleration allowed
    "ACC_Loeseanforderung": 0,
    "ACC_limitierte_Anfahrdyn": 0,
    "ACC_Getriebestellung_P": 0,
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

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
