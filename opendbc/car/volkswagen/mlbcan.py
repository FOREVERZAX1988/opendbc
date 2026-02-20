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


# Braking mode state for hysteresis (prevents rapid mode switching that causes brake stabs)
_braking_prev = False

# Dynamic drag estimation: EMA-filtered engine output torque during cruise.
# Adapts to road grade, wind, payload, engine variant -- no static formula can do this.
_dynamic_drag = 0.0
_dynamic_drag_samples = 0


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0, engine_torque=0):
  global _braking_prev, _dynamic_drag, _dynamic_drag_samples
  commands = []

  # ACC_05: accel/decel request to gearbox, ESP, EPB, and motor
  # ACC_01 is not used on MLB (Macan) -- the stock radar only sends ACC_05
  #
  # Stock radar behavior observed from Cabana at 77 km/h steady cruise:
  #   - ACC_Momentenanforderung: 173 Nm (engine torque request - primary accel control)
  #   - ACC_Verz_anf: 0.0 (zero during cruise/accel, negative during braking)
  #   - ACC_ax_Getriebe: 0.0 (zero at cruise, positive for accel, negative for braking)
  #   - ACC_Freigabe_Momentenanf: 1 (torque request enabled)
  #   - ACC_Freigabe_Verzanf: 0 (decel NOT requested during cruise)
  #   - ACC_Vorbefuellung_Bremsanlage: 0 (brake pre-fill OFF)
  #
  # Control architecture:
  #   Acceleration: engine torque via ACC_Momentenanforderung, ACC_Verz_anf = 0
  #   Braking: decel via ACC_Verz_anf (negative), ACC_Momentenanforderung = 0

  # Braking mode with hysteresis to prevent rapid mode switching.
  # Hysteresis ensures we only enter braking for meaningful decel requests (curves, stops)
  # and stay committed until the planner clearly wants to cruise/accelerate again.
  #   Enter braking: accel < -0.18 (responsive to brake requests; tighter now that
  #     the drag torque model is stock-calibrated and planner no longer oscillates at -0.2)
  #   Exit braking:  accel > -0.05  (planner clearly wants cruise/accel)
  # In between (-0.18 to -0.05), mild decel is handled by reducing engine torque.
  if acc_enabled:
    if _braking_prev:
      # At low speed, require positive accel to release brakes. Near standstill the
      # planner naturally eases off (e.g. -0.04) which isn't "wants to go" -- it's
      # just reducing brake pressure as the car slows. Without this, 45 Nm of drag
      # torque at standstill creeps the car through red lights.
      exit_threshold = -0.05 if v_ego > 2.0 else 0.0
      braking = accel < exit_threshold
    else:
      braking = accel < -0.18   # enter braking for curves/stops
    # Keep braking committed during stops -- planner accel can fluctuate near 0
    # and briefly cross -0.05, which would release brakes mid-stop without this
    if stopping:
      braking = True
  else:
    braking = False
  _braking_prev = braking

  # Engine torque request (ACC_Momentenanforderung, 0-1021 Nm)
  #
  # DRAG TORQUE:
  #   Linear fit to stock ACC median cruise torque (route 00000001--af206016dd).
  #   R²=0.96, max error 6 Nm. Stock cruise torque rises nearly linearly with speed.
  #     drag_torque = 2.5 * v_ego + 141
  #     20 km/h: 155   40 km/h: 169   60 km/h: 183   80 km/h: 196   100 km/h: 210
  #   Old: 0.2v²-4.5v+170 (R²=-1.6, 25-32 Nm short at 30-70 kph due to mid-range valley)
  #
  # ACCEL GAIN:
  #   Flat gain of 77 Nm per m/s². The planner sends higher accel values (0.8-1.5)
  #   than stock ACC's internal accel (~0.3-0.5) for the same gentle departure, so
  #   the gain must be lower to produce stock-appropriate torque.
  #   At gain=77: accel=1.0 → ~218 Nm at 0 kph (matches stock gentle launch 210-250).
  #   Old gain=150 produced 304 Nm at accel=1.0, matching stock *aggressive* launches
  #   and causing PDK to downshift + high RPMs for normal driving.
  #   Old: accel_gain = 150 (too high, caused high RPMs during normal launches)
  #   Old: accel_gain = 5.9 * v_ego + 80 (over-revved at highway)
  #   Old: accel_gain = 5 * v_ego + 63 (with * 1.18 multiplier)
  #
  # TORQUE TAPER: as accel approaches braking threshold (-0.18), torque fades to 0.
  #   -0.10: fade=1.0 (full torque)  -0.14: fade=0.5  -0.18: fade=0.0 (handoff to brakes)
  if acc_enabled and not braking:
    drag_torque = 2.5 * v_ego + 141

    accel_gain = 77  # with corrected drag formula, lower gain should work (matches stock)
    accel_torque = accel * accel_gain
    acc_moment = int(max(0, min(500, drag_torque + accel_torque)))
    if accel < -0.1:
      fade = max(0.0, (accel + 0.18) / 0.08)  # 1.0 at -0.1, 0.0 at -0.18
      acc_moment = int(acc_moment * fade)
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
    # Stock ACC_ax_Getriebe: positive during accel, negative during braking, ~0 at cruise
    # Tells the PDK what acceleration to expect, influencing gear selection.
    # DBC: 9-bit unsigned, range [-2.016, +10.248]. Values below -2.016 WRAP to ~+10
    # (unsigned overflow), sending a massive accel request to the PDK during braking!
    #   Braking: clamped to DBC min -2.016 (stock max observed: -2.015)
    #   Stock behavior (from Cabana + ChatGPT/SSP research):
    #     Cruise (~0 accel): ~0 (tight band -0.1 to +0.1)
    #     Accel to set speed: +0.3 to +2.0 (mode-dependent)
    #     Closing on lead: -0.2 to -1.5
    #     Strong braking: -1.5 to -2.016 (DBC min)
    #   Current: dead zone below 0.25 (cruise = 0, no gear hunting), conservative
    #   cap at 1.3 during accel (prevents high-RPM over-rev), braking unchanged.
    #   Braking: clamped to DBC min -2.016 (stock max observed: -2.015)
    # Old zero-during-accel: (0 if not braking else ...) -- too simple, PDK had
    #   no lookahead for acceleration phases (gear prep, preselection).
    # Old linear: 2.5x multiplier, floor 0. Scales naturally with planner accel --
    #   no artificial floor that causes unnecessary downshifts at low accel.
    #   Mild decel (torque taper zone): capped at -0.3 to signal gentle upshift.
    #   (max(accel * 2.5, -0.3) if not braking else
    #    max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    # Old quadratic: 1.8x multiplier with speed-dependent cap and 0.3 floor.
    #   (max(min(accel * 1.8, min(1.8 + 0.015 * v_ego * 3.6, 2.5)),
    #        ((0.3 if accel > 0.3 else 0.0) if accel > 0.0 else max(accel, -0.3))) if not braking else
    #    max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_ax_Getriebe": ((min(accel, 1.3) if accel > 0.25 else 0) if not braking else
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
    "ACC_Display_Prio": 3,
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
