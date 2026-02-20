from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
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
  return mqb_create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, set_increase=False, set_decrease=False):
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
    "LS_Tip_Setzen": set_increase,    # SET: forward push on stalk (with LS_Tip_Hoch for speed increase)
    "LS_Tip_Hoch": set_increase,      # UP tip: speed increase (+1 mph per short press)
    "LS_Tip_Runter": set_decrease,    # DOWN tip: speed decrease (-1 mph per short press)
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, engine_torque=0):
  commands = []

  # ACC_05: multiplicative torque control
  #
  # Cruise torque (2.5 * v_ego + 141) is the baseline needed to hold speed on flat ground.
  # Instead of adding a small gain on top (additive, weak for decel), we SCALE the baseline:
  #   accel > 0:  scale up   (more torque, car accelerates)
  #   accel = 0:  scale = 1  (cruise torque, hold speed)
  #   accel < 0:  scale down (less torque, engine braking)
  #   accel = -0.5: scale = 0 (max engine braking, transition to hydraulic)
  #
  # This eliminates the dead zone: at accel=-0.05, torque drops by ~18 Nm (vs 4 Nm additive).
  # Self-correcting: if drag formula is 5 Nm too high, planner only needs accel=-0.02 to fix it.
  # No hysteresis needed: torque is already ~0 at the hydraulic braking threshold, so there's
  # no cliff to cause brake stabs when switching modes.
  #
  # Asymmetric k: planner sends 0.8-1.5 for launches but only -0.05 to -0.1 for
  # cruise corrections. k_decel=2.0 gives effective engine braking (torque reaches 0 at -0.5).
  # k_accel ramps quadratically from 0 at standstill to 0.5 at ~40 kph to prevent harsh
  # stop-and-go launches (PDK gear 1 multiplies torque ~11x, so 230 Nm feels like a lunge).
  # Cruise_torque alone (141 Nm) still gives ~1.9 m/s² in gear 1 -- brisk, not sluggish.
  #
  # Cruise torque baseline: linear fit to stock ACC (R²=0.96, max err 6 Nm)
  #   20 km/h: 155   40 km/h: 169   60 km/h: 183   80 km/h: 196   100 km/h: 210

  # Hydraulic braking: only for significant decel (beyond engine braking range),
  # stopping, or preventing standstill creep (no torque at low speed unless planner wants to go)
  if acc_enabled:
    braking = accel < -0.5 or stopping or (v_ego < 2.0 and accel <= 0)
  else:
    braking = False

  if acc_enabled and not braking:
    cruise_torque = 2.5 * v_ego + 141
    if accel >= 0:
      k_accel = 0.5 * min(1.0, (v_ego / 11.0) ** 2)
      scale = 1.0 + accel * k_accel
    else:
      scale = max(0.0, 1.0 + accel * 2.0)
    acc_moment = int(min(500, cruise_torque * scale))
  else:
    acc_moment = 0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    # Stock ACC_Verz_anf range during braking: -2.015 to 0
    # Panda safety allows -3.5; DBC allows -7.22. Use panda limit for max braking.
    "ACC_Verz_anf": max(accel, -3.5) if braking else (0 if acc_enabled else 3.01),
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0,
    # ACC_ax_Getriebe: tells PDK what acceleration to expect (gear selection hint).
    # DBC: [-2.016, +10.248]. Values below -2.016 WRAP to ~+10 (unsigned overflow).
    # Accel > 0.25: hint positive, capped at 1.3 (prevents high-RPM downshifts)
    # Cruise/mild decel: 0 (no gear hunting)
    # Engine braking (accel < -0.25): mild negative hint, capped at -0.5
    # Hydraulic braking: speed-dependent negative, clamped to DBC min -2.016
    "ACC_ax_Getriebe": ((min(accel, 1.3) if accel > 0.25 else
                          (max(accel, -0.5) if accel < -0.25 else 0)) if not braking else
                         max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    "ACC_Beeinflussung_ESP": 1 if (stopping or esp_hold) else 0,  # Only force ESP when stopping or held at standstill (too harsh for normal braking)
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Anhalten": stopping,
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": zeitluecke,  # Mirror stock radar's ZL from ext bus (responds to DIST button)
    "ACC_Display_Prio": 2 if lead_object else 3,
    "ACC_Abstandsindex": lead_distance,
    "ACC_Relevantes_Objekt": lead_object,
  }

  return packer.make_can_msg("ACC_02", bus, values)

def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x111: 0x10, # TSK_05
    0x30C: 0x0F, # ACC_02
    0x324: 0x27, # ACC_04
    0x10B: 0xA,  # LS_01
    0x10D: 0x0C, # ACC_05
    0x10F: 0x0E, # ACC_0x10F
    0x311: 0x12, # ACC_0x311
    0x397: 0x94, # LDW_02
    0x10C: 0x0D, # TSK_02
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return volkswagen_mqb_meb_checksum(address, sig, d)
