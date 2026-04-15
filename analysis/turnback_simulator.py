"""
Turnback Simulator — "Impossible Turn" Analysis

Simulates engine failure after takeoff and the gliding turn back to the runway.
Builds the heart-shaped safe-return envelope at various failure altitudes AGL.

Physics:
  - Zero thrust (T=0) after engine failure
  - Gradient = (T-D) / W  (bank angle affects D via higher CL, not the gradient equation)
  - Drag polar: CD = CDo + ΔCDo_flap + k·CL²
  - CL = nz·W / (q·S)  to support the load factor in the turn
  - Turn rate: ω = g·tan(φ) / V_TAS
  - Turn radius: R = V_TAS² / (g·tan(φ))
  - Atmosphere varies with altitude (ISA model from flight_physics)
  - Wind shifts ground track only (does not change airmass-referenced aero)
    Convention: wind_from_deg relative to runway heading,
    0° = headwind, 90° = right crosswind, 180° = tailwind
"""

import math
from engine.flight_physics import atmos

G_FPS2 = 32.174  # ft/s²
DT = 0.05         # time step (seconds) — small for accuracy in tight turns

# Prop drag increments (ΔCDo) after engine failure
PROP_DRAG_INCREMENTS = {
    'feathered':           0.0005,  # blades edge-on — small residual (hub, spinner, blade thickness)
    'windmilling':         0.0020,  # prop spinning freely in the airstream
    'fixed_pitch_stopped': 0.0015,  # small fixed-pitch prop stopped / not spinning
    'stopped':             0.0040,  # blades flat / unfeathered — maximum drag
}


def _compute_aero_k(S, b, e, h_wl):
    """Induced drag factor k = 1/(π·e·AR_eff) with winglet correction."""
    AR_eff = b ** 2 / S * (1.0 + 1.9 * h_wl / b) if b > 0 else b ** 2 / S
    return 1.0 / (math.pi * e * AR_eff)


def best_glide_kias(config, weight, nz, field_elevation, isa_dev, cdo_override=None):
    """Compute best-glide KIAS (max L/D speed) for a given load factor.

    At max L/D: CL_opt = sqrt(CDo / k), so induced drag = parasite drag.
    V_bg = sqrt(2·nz·W / (ρ·S·CL_opt))

    Returns (vbg_kias, cl_opt, ld_max).
    """
    S = config.wing_area
    b = config.span
    e = config.oswald
    h_wl = config.winglet_height
    cdo = cdo_override if cdo_override is not None else config.cdo
    k = _compute_aero_k(S, b, e, h_wl)

    cl_opt = math.sqrt(cdo / k) if k > 0 else 1.0
    ld_max = cl_opt / (2.0 * cdo)  # L/D_max = CL_opt / (2·CDo)

    _, _, sigma, _, _, _ = atmos(field_elevation, isa_dev)
    rho = 0.002378 * sigma  # slugs/ft³

    # V_TAS for best glide at this nz
    v_tas_fps = math.sqrt(2.0 * nz * weight / (rho * S * cl_opt))
    v_ktas = v_tas_fps / 1.6878
    v_kias = v_ktas * math.sqrt(sigma)

    return v_kias, cl_opt, ld_max


def _kias_to_fps(kias, sigma, delta):
    """Convert KIAS to true airspeed in ft/s at given atmospheric conditions."""
    # Rough Mach for compressibility correction
    a0_kt = 661.5
    m_est = kias / (a0_kt * math.sqrt(delta / (delta / (delta / max(delta, 1e-9)))))
    # Simplified: at low altitude / low speed, KIAS ≈ KEAS
    correction = 1.0  # negligible below M 0.3
    vkeas = kias / correction
    vktas = vkeas / math.sqrt(max(sigma, 0.01))
    v_fps = vktas * 6076.12 / 3600.0
    return v_fps, vkeas, vktas


def _estimate_climb_distance(config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_kias):
    """Estimate horizontal distance from threshold at the moment of engine failure.

    Uses the full-thrust climb gradient at the specified airspeed to estimate
    how far the aircraft has traveled horizontally when reaching failure_alt_agl.
    """
    mid_alt = field_elevation + failure_alt_agl / 2.0
    _, _, sigma, delta, _, c_kt = atmos(mid_alt, isa_dev)
    v_fps, vkeas, _ = _kias_to_fps(airspeed_kias, sigma, delta)
    q = vkeas ** 2 / 295.0

    S = config.wing_area
    b = config.span
    e = config.oswald
    h_wl = config.winglet_height
    cdo = config.cdo
    k = _compute_aero_k(S, b, e, h_wl)

    # CL for wings-level flight
    cl = weight / (q * S) if q * S > 0 else 1.0
    cd = cdo + k * cl ** 2
    drag = cd * q * S

    # Rough thrust estimate — use the polynomial model at mid-altitude
    d_alt_mid, _, _, _, _, _ = atmos(mid_alt, isa_dev)
    m_est = airspeed_kias / max(c_kt, 100.0)
    thrust_mult = config.thrust_mult
    engines = config.engines

    # Polynomial thrust (from flight_physics, simplified here)
    eff_alt = d_alt_mid
    thrust_reg = (2785.75
                  - 1950.17 * m_est
                  - 0.05261 * eff_alt
                  - 12.9726 * m_est ** 2
                  + 0.07669 * m_est * eff_alt
                  - 0.0000001806 * eff_alt ** 2
                  + 1118.99 * m_est ** 3
                  - 0.03617 * m_est ** 2 * eff_alt
                  - 0.0000003701 * m_est * eff_alt ** 2
                  + 0.000000000003957 * eff_alt ** 3)
    thrust = thrust_reg * thrust_mult * engines
    if thrust < 100:
        thrust = 500  # minimum fallback for single-engine piston/turboprop

    gradient = (thrust - drag) / max(weight, 1.0)
    gradient = max(gradient, 0.02)  # at least 2% to avoid infinite distance

    distance_ft = failure_alt_agl / gradient
    return distance_ft


def _select_vbg_override(gear_down, landing_flaps_active, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias):
    """Pick the correct best-glide override for the current drag config.

    Returns the override speed (KIAS) or 0 if no override applies.
    """
    if gear_down and landing_flaps_active and vbg_landing_kias > 0:
        return float(vbg_landing_kias)
    if gear_down and vbg_geardown_kias > 0:
        return float(vbg_geardown_kias)
    if vbg_clean_kias > 0:
        return float(vbg_clean_kias)
    return 0


def simulate_turnback(
    config,
    weight,
    failure_alt_agl,
    airspeed_kias,
    bank_angle_deg,
    flap_setting,
    reaction_time,
    field_elevation=0.0,
    isa_dev=0.0,
    turn_direction='left',
    wind_speed_kt=0.0,
    wind_from_deg=0.0,
    runway_length=0.0,
    liftoff_distance=0.0,
    aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
):
    """Simulate the turnback maneuver after total engine failure.

    Phases:
      1. Reaction — straight wings-level glide (pilot shock / recognition)
      2. Turn — constant bank angle, turning toward the runway
      3. Return — wings-level glide back to the runway

    Args:
        config: AircraftConfig instance
        weight: aircraft weight (lbs)
        failure_alt_agl: altitude AGL at engine failure (ft)
        airspeed_kias: airspeed at failure, maintained throughout (KIAS)
        bank_angle_deg: bank angle during turn (degrees)
        flap_setting: 0=clean, 1=takeoff/approach, 2=landing
        reaction_time: pilot reaction delay (seconds)
        field_elevation: airport elevation MSL (ft)
        isa_dev: ISA temperature deviation (°C)
        turn_direction: 'left' or 'right'
        wind_speed_kt: wind speed in knots
        wind_from_deg: wind direction relative to runway heading (0°=headwind,
                       90°=right crosswind, 180°=tailwind, 270°=left crosswind)
        runway_length: total runway length (ft). 0 = no runway model.
        liftoff_distance: ground roll from departure threshold to liftoff (ft)
        aim_point: target touchdown point measured from departure threshold (ft).
                   On a turnback the pilot lands in the opposite direction,
                   so aim_point is near the far end of the runway.
        flap_on_return: if True, flaps are clean during reaction+turn phases
                        and deployed only when entering the return phase.
                        The flap_setting arg controls which flap position to use.
        speed_mode: 'fixed'          — maintain airspeed_kias throughout
                    'best_glide_1g'  — fly best-glide KIAS for 1g (wings level)
                    'best_glide_nz'  — adjust best-glide KIAS for the current
                                       load factor (faster in the turn)

    Returns:
        dict with keys:
            trajectory: list of dicts {x, y, z, heading_deg, time, phase, cl, nz}
            success: True if aircraft returns to runway vicinity with z > 0
            altitude_at_runway: altitude when crossing the runway line (ft AGL)
            stalled: True if CL exceeded CLmax during the maneuver
            stall_time: time of stall (seconds), or None
            turn_radius_ft: radius of the turn (ft)
            ground_impact: {x, y} of ground contact if unsuccessful
    """
    # Aircraft parameters
    S = config.wing_area
    b = config.span
    e = config.oswald
    h_wl = config.winglet_height
    cdo_clean = config.cdo + PROP_DRAG_INCREMENTS.get(prop_state, 0.0)
    if gear_down:
        cdo_clean += config.dcdo_gear
    k = _compute_aero_k(S, b, e, h_wl)

    # Flap drag and CLmax
    if flap_setting == 1:
        dcdo_flap = config.dcdo_flap1
        clmax_flap = config.Clmax_flaps15
    elif flap_setting == 2:
        dcdo_flap = config.dcdo_flap2
        clmax_flap = config.Clmax_flaps40
    else:
        dcdo_flap = 0.0
        clmax_flap = config.Clmax

    # If flap_on_return, start clean and switch on return phase
    if flap_on_return and flap_setting > 0:
        cdo = cdo_clean
        clmax = config.Clmax
    else:
        cdo = cdo_clean + dcdo_flap
        clmax = clmax_flap

    # Runway geometry
    use_runway = runway_length > 0 and liftoff_distance > 0

    # Pre-compute best-available landing flap config for runway model.
    # On the return phase the pilot always deploys the best flaps available
    # to steepen descent and reduce touchdown speed / rollout.
    if getattr(config, 'dcdo_flap2', 0.0) > 0:
        dcdo_landing = config.dcdo_flap2
        clmax_landing = config.Clmax_flaps40
    elif getattr(config, 'dcdo_flap1', 0.0) > 0:
        dcdo_landing = config.dcdo_flap1
        clmax_landing = config.Clmax_flaps15
    else:
        dcdo_landing = 0.0
        clmax_landing = config.Clmax

    # Approach speed for runway-model return phase: 1.3 × Vs_landing
    approach_kias = None
    if use_runway:
        _, _, _sigma_sfc, _, _, _ = atmos(field_elevation, isa_dev)
        _rho_sfc = 0.002378 * _sigma_sfc
        _vs_land_fps = math.sqrt(2.0 * weight / (_rho_sfc * S * clmax_landing)) if clmax_landing > 0 else 100.0
        _vs_land_kias = (_vs_land_fps / 1.6878) * math.sqrt(_sigma_sfc)
        approach_kias = _vs_land_kias * 1.3

    # Wind vector in ft/s (ground-referenced)
    # Convention: wind_from_deg is relative to runway heading.
    #   0° = headwind (from ahead), 90° = from the right, 180° = tailwind
    # The wind velocity vector points opposite to where it comes FROM.
    wind_fps = wind_speed_kt * 6076.12 / 3600.0
    wind_from_rad = math.radians(wind_from_deg)
    wind_x = -wind_fps * math.sin(wind_from_rad)   # lateral ft/s (+ = rightward)
    wind_y = -wind_fps * math.cos(wind_from_rad)    # along-runway ft/s (+ = with takeoff direction)

    # Bank parameters
    phi_rad = math.radians(bank_angle_deg)
    cos_phi = math.cos(phi_rad)
    nz_bank = 1.0 / cos_phi if cos_phi > 0.01 else 100.0
    tan_phi = math.tan(phi_rad) if bank_angle_deg > 0.1 else 0.0

    # Estimate starting position (ground roll + airborne climb distance)
    # For best-glide modes, compute the 1g best glide speed for the climb estimate
    if speed_mode in ('best_glide_1g', 'best_glide_nz'):
        vbg_1g, cl_best, ld_max_val = best_glide_kias(
            config, weight, 1.0, field_elevation, isa_dev, cdo_override=cdo)
        _vbg_ov = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
        if _vbg_ov > 0:
            vbg_1g = _vbg_ov
        airspeed_for_climb = vbg_1g
    else:
        vbg_1g = None
        cl_best = None
        ld_max_val = None
        airspeed_for_climb = airspeed_kias

    # Auto-compute optimal aim point for runway model.
    # Touch down far enough from the departure threshold to allow landing
    # rollout (braking to a stop).  This widens the success band compared
    # to aiming at the departure threshold.
    rollout_est = 0.0
    if use_runway:
        _, _, sigma_sfc, delta_sfc, _, _ = atmos(field_elevation, isa_dev)
        # Aim point based on approach speed (landing flaps) for accurate rollout
        td_kias_est = approach_kias if approach_kias else (vbg_1g if vbg_1g else airspeed_kias)
        v_td_est, _, _ = _kias_to_fps(td_kias_est, sigma_sfc, delta_sfc)
        rollout_est = v_td_est ** 2 / (2.0 * G_FPS2 * 0.3)  # mu_brake = 0.3
        target_y = rollout_est + touchdown_margin_ft
    else:
        target_y = 0.0

    climb_dist = _estimate_climb_distance(
        config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_for_climb
    )
    dist_from_threshold = liftoff_distance + climb_dist

    # --- Initial state ---
    x = 0.0                      # lateral offset from runway centerline (ft, + = right)
    y = dist_from_threshold      # distance along runway heading from threshold (ft)
    z = failure_alt_agl          # altitude AGL (ft)
    heading = 0.0                # radians, 0 = runway heading (+Y direction)
    t = 0.0
    phase = 'reaction'
    total_heading_change = 0.0
    stalled = False
    stall_time = None
    success = False
    altitude_at_runway = None
    orbit_heading_change = 0.0  # tracks heading change within current orbit
    # Scale orbit budget by altitude — each orbit loses ~200-400 ft;
    # generous budget ensures high altitudes always succeed.
    max_orbits = max(20, int(failure_alt_agl / 100) + 1)
    landing_direction = 'reverse'  # 'reverse' = land opposite direction; 'original' = same as takeoff
    has_orbited_reverse = False  # True once we've tried a 360° orbit while still reverse

    trajectory = []

    # Pre-compute a turn radius estimate for output (at target speed/bank)
    alt_msl_init = field_elevation + failure_alt_agl
    _, _, sigma_init, delta_init, _, _ = atmos(alt_msl_init, isa_dev)
    if speed_mode == 'best_glide_nz':
        _vbg_ov_turn = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
        if _vbg_ov_turn > 0:
            vbg_turn = _vbg_ov_turn * math.sqrt(nz_bank)
        else:
            vbg_turn, _, _ = best_glide_kias(
                config, weight, nz_bank, field_elevation, isa_dev, cdo_override=cdo)
        v_fps_init, _, _ = _kias_to_fps(vbg_turn, sigma_init, delta_init)
    elif speed_mode == 'best_glide_1g':
        v_fps_init, _, _ = _kias_to_fps(vbg_1g, sigma_init, delta_init)
    else:
        vbg_turn = None
        v_fps_init, _, _ = _kias_to_fps(airspeed_kias, sigma_init, delta_init)
    turn_radius_ft = v_fps_init ** 2 / (G_FPS2 * tan_phi) if tan_phi > 0.001 else float('inf')

    prev_y = y  # track when aircraft crosses target_y

    # --- Continuous state: actual airspeed and bank angle ---
    # The aircraft starts at climb-out speed and wings level.
    # Speed and bank ramp toward targets each timestep via energy balance.
    actual_kias = airspeed_kias           # actual airspeed (KIAS)
    actual_bank_deg = 0.0                 # actual bank angle (degrees)
    BANK_RATE_DPS = 15.0                  # roll rate limit (deg/sec)
    ACCEL_LIMIT_KT_S = 5.0               # acceleration limit (kt/sec)

    v_fps_actual, _vkeas_init, _ = _kias_to_fps(actual_kias, sigma_init, delta_init)
    q = _vkeas_init ** 2 / 295.0  # initialize dynamic pressure

    t_limit = max(900.0, failure_alt_agl * 1.0)  # generous time for high altitudes
    while z > 0 and t < t_limit:
        # --- Atmosphere at current altitude ---
        alt_msl = field_elevation + max(z, 0.0)
        _, _, sigma_now, delta_now, _, c_kt = atmos(alt_msl, isa_dev)

        # --- Phase logic (runs first so flap/nz state is current) ---
        # target_bank_deg is set per-phase; actual_bank_deg ramps toward it.
        target_bank_deg = 0.0
        if phase == 'reaction':
            target_bank_deg = 0.0
            if t >= reaction_time:
                phase = 'turn'
        elif phase == 'turn':
            target_bank_deg = bank_angle_deg
            # Check if heading now points at the aim point
            if total_heading_change > math.pi * 0.75:  # at least 135° before checking
                dx_to_aim = 0.0 - x
                dy_to_aim = target_y - y
                dist_to_aim = math.sqrt(dx_to_aim ** 2 + dy_to_aim ** 2)
                if dist_to_aim > 10.0:
                    bearing_to_aim = math.atan2(dx_to_aim, dy_to_aim)
                    heading_error = bearing_to_aim - heading
                    heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi
                    if abs(heading_error) < math.radians(8.0):
                        phase = 'return'
                        heading = bearing_to_aim
                        # Runway model: always deploy best landing flaps
                        # for steeper descent and shorter rollout.
                        if use_runway:
                            cdo = cdo_clean + dcdo_landing
                            clmax = clmax_landing
                        elif flap_on_return and flap_setting > 0:
                            cdo = cdo_clean + dcdo_flap
                            clmax = clmax_flap
        elif phase == 'return':
            target_bank_deg = 0.0
            # Steer toward the landing target (crab into wind)
            dx_to_tgt = 0.0 - x
            dy_to_tgt = target_y - y
            dist_to_tgt = math.sqrt(dx_to_tgt ** 2 + dy_to_tgt ** 2)

            # Check if we need to lose altitude.
            # If at max slip we can't reach the aim point, the priority is:
            #   1. Orbit 360° while staying on reverse heading (land RWY 18)
            #   2. If still too high after orbiting, switch to same-direction
            #      landing via a circuit (continue turning ~180° → land RWY 36)
            #   3. If still too high on original heading, orbit again
            if use_runway and z > 100 and dist_to_tgt > 200 and max_orbits > 0:
                _cl_est = weight / (q * S) if q * S > 0 else 1.0
                _cd_slip_max = 3.0 * cdo + k * _cl_est ** 2
                _sin_gamma_max_slip = -_cd_slip_max * q * S / weight if weight > 0 else -0.2
                _sin_gamma_max_slip = max(_sin_gamma_max_slip, -1.0)
                _sin_gamma_needed = -z / math.sqrt(z ** 2 + dist_to_tgt ** 2)
                if _sin_gamma_max_slip > _sin_gamma_needed:
                    # Estimate altitude loss for one orbit before committing
                    _cl_orb = nz_bank * weight / (q * S) if q * S > 0 else 1.0
                    _cd_orb = cdo + k * _cl_orb ** 2
                    _orbit_alt_loss = (2.0 * math.pi * v_fps_actual ** 2 * _cd_orb * nz_bank
                                       / (max(_cl_orb, 0.01) * G_FPS2 * max(tan_phi, 0.01)))
                    _safe_to_orbit = z > _orbit_alt_loss + 150

                    if not _safe_to_orbit:
                        pass  # not enough altitude — commit to approach with slip
                    elif landing_direction == 'reverse' and not has_orbited_reverse:
                        # First attempt: orbit 360° to lose altitude, stay reverse
                        phase = 'orbit'
                        orbit_heading_change = 0.0
                        has_orbited_reverse = True
                        max_orbits -= 1
                    elif landing_direction == 'reverse':
                        # Already orbited reverse, still too high —
                        # switch to same-direction landing (continue ~180° more)
                        landing_direction = 'original'
                        target_y = runway_length - rollout_est - touchdown_margin_ft
                        phase = 'circuit'
                        orbit_heading_change = 0.0
                        max_orbits -= 1
                    else:
                        # Already on original heading but still too high — orbit
                        phase = 'orbit'
                        orbit_heading_change = 0.0
                        max_orbits -= 1

            if phase == 'return':
                if landing_direction == 'original':
                    # Landing same direction — fly toward far-end aim point
                    dx_to_tgt = 0.0 - x
                    dy_to_tgt = target_y - y
                    dist_to_tgt = math.sqrt(dx_to_tgt ** 2 + dy_to_tgt ** 2)
                    if dist_to_tgt > 1.0:
                        heading = math.atan2(dx_to_tgt, dy_to_tgt)
                elif use_runway and y < target_y:
                    heading = math.atan2(-x, -500.0)
                else:
                    if dist_to_tgt > 1.0:
                        heading = math.atan2(dx_to_tgt, dy_to_tgt)

        elif phase == 'circuit':
            # Continue turning ~180° to align with original runway heading.
            # Same bank angle / direction as the initial turnback.
            target_bank_deg = bank_angle_deg
            if z < 200 or orbit_heading_change >= math.pi:
                # ~180° complete (or altitude too low to continue) — aim at target
                phase = 'return'
                dx_to_tgt = 0.0 - x
                dy_to_tgt = target_y - y
                dist_to_tgt = math.sqrt(dx_to_tgt ** 2 + dy_to_tgt ** 2)
                if dist_to_tgt > 1.0:
                    heading = math.atan2(dx_to_tgt, dy_to_tgt)

        elif phase == 'orbit':
            # 360° altitude-loss orbit at bank angle, same direction as initial turn
            target_bank_deg = bank_angle_deg
            if z < 200 or orbit_heading_change >= 2.0 * math.pi:
                # Orbit complete (or altitude too low to continue) — return to approach
                phase = 'return'
                dx_to_tgt = 0.0 - x
                dy_to_tgt = target_y - y
                dist_to_tgt = math.sqrt(dx_to_tgt ** 2 + dy_to_tgt ** 2)
                if dist_to_tgt > 1.0:
                    heading = math.atan2(dx_to_tgt, dy_to_tgt)

        # === Bank angle ramping ===
        # Ramp actual_bank_deg toward target_bank_deg at BANK_RATE_DPS
        bank_error = target_bank_deg - actual_bank_deg
        max_bank_change = BANK_RATE_DPS * DT
        if abs(bank_error) <= max_bank_change:
            actual_bank_deg = target_bank_deg
        elif bank_error > 0:
            actual_bank_deg += max_bank_change
        else:
            actual_bank_deg -= max_bank_change
        actual_bank_deg = max(actual_bank_deg, 0.0)  # no negative bank

        # Compute actual nz and turn rate from actual bank angle and actual speed
        actual_bank_rad = math.radians(actual_bank_deg)
        cos_bank = math.cos(actual_bank_rad)
        nz = 1.0 / cos_bank if cos_bank > 0.01 else 100.0
        tan_bank_actual = math.tan(actual_bank_rad) if cos_bank > 0.01 else 100.0
        omega = G_FPS2 * tan_bank_actual / max(v_fps_actual, 10.0) if actual_bank_deg > 0.5 else 0.0
        if turn_direction == 'left':
            omega = -omega

        # Track orbit/circuit heading from actual turn rate
        if phase in ('circuit', 'orbit'):
            orbit_heading_change += abs(omega * DT)

        # === Determine TARGET airspeed (what pilot is aiming for) ===
        if phase == 'reaction':
            target_kias = airspeed_kias
        elif use_runway and phase in ('return', 'orbit', 'circuit') and approach_kias is not None:
            target_kias = approach_kias
        elif speed_mode == 'best_glide_nz':
            _landing_flaps = phase in ('return', 'orbit', 'circuit') and (use_runway or flap_on_return)
            _vbg_ov = _select_vbg_override(gear_down, _landing_flaps, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
            if _vbg_ov > 0:
                # POH override is a 1g speed; scale by sqrt(nz) for the bank
                target_kias = _vbg_ov * math.sqrt(nz)
            else:
                target_kias, _, _ = best_glide_kias(
                    config, weight, nz, field_elevation, isa_dev, cdo_override=cdo)
        elif speed_mode == 'best_glide_1g':
            _landing_flaps = phase in ('return', 'orbit', 'circuit') and (use_runway or flap_on_return)
            _vbg_ov = _select_vbg_override(gear_down, _landing_flaps, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
            if _vbg_ov > 0:
                target_kias = _vbg_ov
            else:
                target_kias, _, _ = best_glide_kias(
                    config, weight, 1.0, field_elevation, isa_dev, cdo_override=cdo)
        else:
            if phase in ('return', 'orbit', 'circuit') and (use_runway or (flap_on_return and flap_setting > 0)):
                _rho_now = 0.002378 * sigma_now
                _vs_now = math.sqrt(2.0 * weight / (_rho_now * S * clmax)) if clmax > 0 else 100.0
                _vs_kias = (_vs_now / 1.6878) * math.sqrt(sigma_now)
                target_kias = _vs_kias * 1.3
            else:
                target_kias = airspeed_kias

        # === Energy-based speed ramping ===
        # Ramp actual_kias toward target_kias at ACCEL_LIMIT_KT_S.
        # The altitude cost/benefit of changing speed is captured in the
        # energy-balance sink rate below.
        speed_error_kt = target_kias - actual_kias
        max_speed_change = ACCEL_LIMIT_KT_S * DT
        if abs(speed_error_kt) <= max_speed_change:
            actual_kias = target_kias
        elif speed_error_kt > 0:
            actual_kias += max_speed_change
        else:
            actual_kias -= max_speed_change

        # --- Airspeed conversion (use ACTUAL speed) ---
        current_kias = actual_kias
        v_fps_actual, vkeas, vktas = _kias_to_fps(current_kias, sigma_now, delta_now)
        q = vkeas ** 2 / 295.0  # dynamic pressure (psf, from EAS in knots)

        # --- Aerodynamics (based on actual speed and actual nz) ---
        cl = nz * weight / (q * S) if q * S > 0 else 99.0

        # Stall check
        if cl > clmax and not stalled:
            stalled = True
            stall_time = t

        cd = cdo + k * cl ** 2
        drag = cd * q * S

        # --- Flight path angle (energy method) ---
        # sin(γ) = (T−D)/W — bank angle affects D through CL, not the gradient.
        sin_gamma = -drag / weight if weight > 0 else -0.5
        sin_gamma = max(sin_gamma, -1.0)
        gamma = math.asin(sin_gamma)

        # Velocities
        v_vert = v_fps_actual * sin_gamma           # ft/s, negative = descending
        v_horiz = v_fps_actual * math.cos(gamma)    # ft/s, horizontal component

        # --- Forward slip for glideslope management (runway model) ---
        # If the aircraft is above the natural glidepath to the aim point,
        # increase drag to steepen the descent (simulates a forward slip).
        slipping = False
        if use_runway and phase == 'return' and z > 5:
            dx_td = 0.0 - x
            dy_td = target_y - y
            dist_td = math.sqrt(dx_td ** 2 + dy_td ** 2)
            if dist_td > 50:
                # Required descent angle to reach aim point at z=0
                sin_gamma_needed = -z / math.sqrt(z ** 2 + dist_td ** 2)
                if sin_gamma > sin_gamma_needed:  # current descent too shallow
                    drag_needed = -sin_gamma_needed * weight
                    cd_needed = drag_needed / (q * S) if q * S > 0 else 0
                    # Cap: aggressive forward slip can roughly triple parasite drag
                    cd_max_slip = 3.0 * cdo + k * cl ** 2
                    cd_eff = min(cd_needed, cd_max_slip)
                    if cd_eff > cd:
                        slipping = True
                        drag = cd_eff * q * S
                        sin_gamma = -drag / weight
                        sin_gamma = max(sin_gamma, -1.0)
                        gamma = math.asin(sin_gamma)
                        v_vert = v_fps_actual * sin_gamma
                        v_horiz = v_fps_actual * math.cos(gamma)

        # Determine flap state label for trajectory output
        if use_runway and phase in ('return', 'orbit', 'circuit'):
            flap_state = 'landing'
        elif (flap_on_return and flap_setting > 0 and phase in ('return', 'orbit', 'circuit')):
            if flap_setting == 2:
                flap_state = 'landing'
            else:
                flap_state = 'takeoff'
        elif flap_setting == 0 or (flap_on_return and phase not in ('return', 'orbit', 'circuit')):
            flap_state = 'clean'
        elif flap_setting == 1:
            flap_state = 'takeoff'
        else:
            flap_state = 'landing'

        # Stall speed for current flap config and load factor
        _rho_traj = 0.002378 * sigma_now
        _vs_fps = math.sqrt(2.0 * nz * weight / (_rho_traj * S * clmax)) if clmax > 0 else 0.0
        _vs_kias_traj = (_vs_fps / 1.6878) * math.sqrt(sigma_now)

        # --- Record state ---
        trajectory.append({
            'x': x,
            'y': y,
            'z': z,
            'heading_deg': math.degrees(heading) % 360.0,
            'time': t,
            'phase': phase,
            'cl': cl,
            'nz': nz,
            'v_fps': v_fps_actual,
            'kias': current_kias,
            'target_kias': target_kias,
            'bank_deg': actual_bank_deg,
            'target_bank_deg': target_bank_deg,
            'vs_kias': _vs_kias_traj,
            'drag': drag,
            'gamma_deg': math.degrees(gamma),
            'roc_fpm': v_vert * 60.0,
            'flap_state': flap_state,
            'slipping': slipping,
        })

        # --- Integrate ---
        heading += omega * DT
        # Keep heading in [-2π, 2π] range
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
        elif heading < -2 * math.pi:
            heading += 2 * math.pi

        if phase == 'turn':
            total_heading_change += abs(omega * DT)

        x += (v_horiz * math.sin(heading) + wind_x) * DT
        y += (v_horiz * math.cos(heading) + wind_y) * DT
        z += v_vert * DT
        t += DT

        # --- Check runway crossing ---
        # Aircraft has returned when y crosses target_y going negative-ward
        # Must have completed significant turn (at least 135°)
        heading_back = total_heading_change > math.pi * 0.75
        if heading_back:
            if use_runway:
                # Runway model: let sim run to z=0 for physics-based touchdown.
                # Don't break here — continue until ground contact.
                pass
            else:
                # Legacy: success = cross y=0 southward with positive altitude
                crossed = (prev_y > target_y and y <= target_y)
                if crossed:
                    # Interpolate altitude at target_y
                    dy_total = prev_y - y
                    if abs(dy_total) > 0.01:
                        frac = (prev_y - target_y) / dy_total
                        altitude_at_runway = (trajectory[-1]['z'] if trajectory else z) + v_vert * DT * frac
                        # Interpolate x at target_y
                        touchdown_x = (trajectory[-1]['x'] if trajectory else x) + \
                            (v_horiz * math.sin(heading) + wind_x) * DT * frac
                    else:
                        altitude_at_runway = z
                        touchdown_x = x
                    if altitude_at_runway > 0:
                        # Must be within lateral tolerance of the runway centerline
                        # 75 ft ≈ half a standard runway width (150 ft for GA)
                        if abs(touchdown_x) < 75.0:
                            success = True
                        else:
                            success = False
                    break
        prev_y = y

    # If loop ended without crossing runway, check final state
    if altitude_at_runway is None:
        altitude_at_runway = z

    # Interpolate to the exact z=0 touchdown position for runway model.
    # This eliminates discretization artifacts where the post-integration
    # x, y shift by several feet per timestep.
    if use_runway and z <= 0 and trajectory:
        prev_z = trajectory[-1]['z']
        if prev_z > 0 and prev_z > z:
            frac = prev_z / (prev_z - z)
            x = trajectory[-1]['x'] + (x - trajectory[-1]['x']) * frac
            y = trajectory[-1]['y'] + (y - trajectory[-1]['y']) * frac
            z = 0.0

    # Runway model: determine success from actual ground-contact position
    if use_runway and not success and total_heading_change > math.pi * 0.75 and z <= 0:
        # Aircraft reached ground — check if it landed on the runway
        if 0 <= y <= runway_length and abs(x) < 75.0:
            success = True
            altitude_at_runway = 0.0

    ground_impact = {'x': x, 'y': y} if not success else None

    # Landing rollout — use approach speed if runway model, else return-phase speed
    if use_runway and approach_kias is not None:
        landing_kias = approach_kias
    else:
        landing_kias = current_kias if trajectory else airspeed_kias
    landing_rollout = 0.0
    if success:
        mu_brake = 0.3
        _, _, sigma_sfc, _, _, _ = atmos(field_elevation, isa_dev)
        v_td_fps, _, _ = _kias_to_fps(landing_kias, sigma_sfc, 1.0)
        landing_rollout = v_td_fps ** 2 / (2.0 * G_FPS2 * mu_brake)

    # Runway overrun check
    runway_overrun = False
    remaining_runway = None
    if success and use_runway:
        if landing_direction == 'original':
            # Landing in takeoff direction — runway ahead is from y to end
            remaining_runway = max(runway_length - y, 0.0)
        else:
            # Landing back toward departure threshold — y is distance available
            remaining_runway = max(y, 0.0)
        if landing_rollout > remaining_runway:
            runway_overrun = True

    # Speed info for output
    speed_info = {'mode': speed_mode}
    _vbg_ov_clean = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
    if _vbg_ov_clean > 0:
        speed_info['vbg_override_clean'] = _vbg_ov_clean
    _vbg_ov_land = _select_vbg_override(gear_down, True, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
    if _vbg_ov_land > 0:
        speed_info['vbg_override_landing'] = _vbg_ov_land
    if speed_mode in ('best_glide_1g', 'best_glide_nz'):
        speed_info['vbg_1g_kias'] = round(vbg_1g, 1)
        speed_info['cl_best'] = round(cl_best, 3) if cl_best is not None else None
        speed_info['ld_max'] = round(ld_max_val, 1) if ld_max_val is not None else None
        if speed_mode == 'best_glide_nz':
            _vbg_ov_turn = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
            if _vbg_ov_turn > 0:
                speed_info['vbg_turn_kias'] = round(_vbg_ov_turn * math.sqrt(nz_bank), 1)
            else:
                vbg_nz, _, _ = best_glide_kias(
                    config, weight, nz_bank, field_elevation, isa_dev, cdo_override=cdo)
                speed_info['vbg_turn_kias'] = round(vbg_nz, 1)

    return {
        'trajectory': trajectory,
        'success': success,
        'altitude_at_runway': altitude_at_runway,
        'stalled': stalled,
        'stall_time': stall_time,
        'turn_radius_ft': turn_radius_ft,
        'ground_impact': ground_impact,
        'dist_from_threshold': dist_from_threshold,
        'landing_rollout': landing_rollout,
        'runway_overrun': runway_overrun,
        'remaining_runway': remaining_runway,
        'computed_aim_y': target_y,
        'speed_info': speed_info,
        'landing_direction': landing_direction,
    }


# ---------------------------------------------------------------------------
#  Straight-Ahead Landing Analysis
# ---------------------------------------------------------------------------

def simulate_straight_ahead(
    config, weight, failure_alt_agl, airspeed_kias,
    reaction_time, field_elevation=0.0, isa_dev=0.0,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    runway_length=0.0, liftoff_distance=0.0,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
):
    """Simulate a straight-ahead landing after engine failure.

    The aircraft continues straight ahead on runway heading.  During the
    reaction phase it glides clean; then it deploys landing flaps and
    descends to touchdown.  Success = touchdown + rollout fit on the
    remaining runway.

    Returns dict with:
        success, trajectory, touchdown_y, dist_from_threshold,
        remaining_runway, landing_rollout, runway_overrun,
        altitude_at_runway_end, stalled, stall_time, glide_distance
    """
    S = config.wing_area
    b = config.span
    e = config.oswald
    h_wl = config.winglet_height
    cdo_clean = config.cdo + PROP_DRAG_INCREMENTS.get(prop_state, 0.0)
    if gear_down:
        cdo_clean += config.dcdo_gear
    k = _compute_aero_k(S, b, e, h_wl)

    # Best available landing-flap config
    if getattr(config, 'dcdo_flap2', 0.0) > 0:
        dcdo_landing = config.dcdo_flap2
        clmax_landing = config.Clmax_flaps40
    elif getattr(config, 'dcdo_flap1', 0.0) > 0:
        dcdo_landing = config.dcdo_flap1
        clmax_landing = config.Clmax_flaps15
    else:
        dcdo_landing = 0.0
        clmax_landing = config.Clmax

    # Start clean during reaction
    cdo = cdo_clean
    clmax = config.Clmax

    # Compute approach speed for the approach phase: 1.3 × Vs_landing
    # This is slower than best-glide, giving a steeper descent and shorter
    # rollout — what a pilot would actually fly for a straight-ahead landing.
    _, _, sigma_sfc, _, _, _ = atmos(field_elevation, isa_dev)
    rho = 0.002378 * sigma_sfc
    vs_landing_fps = math.sqrt(2.0 * weight / (rho * S * clmax_landing)) if clmax_landing > 0 else 100.0
    vs_landing_kias = (vs_landing_fps / 1.6878) * math.sqrt(sigma_sfc)
    approach_kias = vs_landing_kias * 1.3

    # Wind vector in ft/s
    wind_fps = wind_speed_kt * 6076.12 / 3600.0
    wind_from_rad = math.radians(wind_from_deg)
    wind_x = -wind_fps * math.sin(wind_from_rad)
    wind_y = -wind_fps * math.cos(wind_from_rad)

    # Determine initial airspeed
    if speed_mode in ('best_glide_1g', 'best_glide_nz'):
        vbg_1g, _, _ = best_glide_kias(
            config, weight, 1.0, field_elevation, isa_dev, cdo_override=cdo)
        _vbg_ov = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
        if _vbg_ov > 0:
            vbg_1g = _vbg_ov
        airspeed_for_climb = vbg_1g
    else:
        vbg_1g = None
        airspeed_for_climb = airspeed_kias

    climb_dist = _estimate_climb_distance(
        config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_for_climb
    )
    dist_from_threshold = liftoff_distance + climb_dist

    # Initial state
    x = 0.0
    y = dist_from_threshold
    z = failure_alt_agl
    heading = 0.0  # straight ahead
    t = 0.0
    phase = 'reaction'
    stalled = False
    stall_time = None

    trajectory = []
    flaps_deployed = False

    while z > 0 and t < 600.0:
        alt_msl = field_elevation + max(z, 0.0)
        _, _, sigma_now, delta_now, _, _ = atmos(alt_msl, isa_dev)

        # Determine airspeed
        if phase == 'reaction':
            # Pilot hasn't reacted yet — maintain climb-out speed
            current_kias = airspeed_kias
        elif phase == 'approach':
            # After flaps: fly approach speed (1.3 Vs landing)
            current_kias = approach_kias
        elif speed_mode in ('best_glide_1g', 'best_glide_nz'):
            _landing_flaps = (phase == 'approach')
            _vbg_ov = _select_vbg_override(gear_down, _landing_flaps, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
            if _vbg_ov > 0:
                current_kias = _vbg_ov
            else:
                current_kias, _, _ = best_glide_kias(
                    config, weight, 1.0, field_elevation, isa_dev, cdo_override=cdo)
        else:
            current_kias = airspeed_kias

        v_fps, vkeas, _ = _kias_to_fps(current_kias, sigma_now, delta_now)
        q = vkeas ** 2 / 295.0

        # Phase transition: deploy landing flaps after reaction time
        if phase == 'reaction' and t >= reaction_time:
            phase = 'approach'
            cdo = cdo_clean + dcdo_landing
            clmax = clmax_landing
            flaps_deployed = True

        # Aerodynamics — wings level nz=1
        nz = 1.0
        cl = nz * weight / (q * S) if q * S > 0 else 99.0

        if cl > clmax and not stalled:
            stalled = True
            stall_time = t

        cd = cdo + k * cl ** 2
        drag = cd * q * S

        sin_gamma = -drag / weight if weight > 0 else -0.5
        sin_gamma = max(sin_gamma, -1.0)
        gamma = math.asin(sin_gamma)

        v_vert = v_fps * sin_gamma
        v_horiz = v_fps * math.cos(gamma)

        trajectory.append({
            'x': x, 'y': y, 'z': z,
            'heading_deg': 0.0,
            'time': t, 'phase': phase,
            'cl': cl, 'nz': nz,
            'v_fps': v_fps, 'kias': current_kias,
            'drag': drag, 'gamma_deg': math.degrees(gamma),
            'roc_fpm': v_vert * 60.0,
        })

        # Integrate
        x += (v_horiz * math.sin(heading) + wind_x) * DT
        y += (v_horiz * math.cos(heading) + wind_y) * DT
        z += v_vert * DT
        t += DT

    # Touchdown point
    touchdown_y = y
    glide_distance = touchdown_y - dist_from_threshold
    touchdown_speed_kias = current_kias if trajectory else airspeed_kias

    # Landing rollout
    mu_brake = 0.3
    _, _, sigma_sfc, _, _, _ = atmos(field_elevation, isa_dev)
    v_td_fps, _, _ = _kias_to_fps(touchdown_speed_kias, sigma_sfc, 1.0)
    landing_rollout = v_td_fps ** 2 / (2.0 * G_FPS2 * mu_brake)

    # Can it land and stop on remaining runway?
    remaining_runway_ahead = runway_length - touchdown_y
    runway_overrun = landing_rollout > remaining_runway_ahead or remaining_runway_ahead < 0
    success = not runway_overrun and remaining_runway_ahead >= 0

    # Altitude when passing runway end (if it overflies)
    altitude_at_runway_end = None
    for i, p in enumerate(trajectory):
        if p['y'] >= runway_length:
            if i > 0:
                prev = trajectory[i - 1]
                dy = p['y'] - prev['y']
                frac = (runway_length - prev['y']) / max(dy, 0.01)
                altitude_at_runway_end = prev['z'] + frac * (p['z'] - prev['z'])
            else:
                altitude_at_runway_end = p['z']
            break

    return {
        'success': success,
        'trajectory': trajectory,
        'touchdown_y': touchdown_y,
        'dist_from_threshold': dist_from_threshold,
        'remaining_runway': remaining_runway_ahead,
        'landing_rollout': landing_rollout,
        'runway_overrun': runway_overrun,
        'altitude_at_runway_end': altitude_at_runway_end,
        'stalled': stalled,
        'stall_time': stall_time,
        'glide_distance': glide_distance,
    }


def find_straight_ahead_max_altitude(
    config, weight, airspeed_kias, reaction_time,
    field_elevation=0.0, isa_dev=0.0,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    runway_length=0.0, liftoff_distance=0.0,
    speed_mode='fixed',
    alt_low=10.0, alt_high=3000.0, tolerance=5.0,
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
):
    """Find the maximum engine-failure altitude where a straight-ahead
    landing succeeds (touchdown + rollout fit on the remaining runway).

    At low altitudes the aircraft touches down early — plenty of room.
    At high altitudes it overshoots the runway end.
    Binary search for the crossover.

    Returns max altitude (ft AGL), or 0.0 if it never works.
    """
    common = dict(
        reaction_time=reaction_time,
        field_elevation=field_elevation, isa_dev=isa_dev,
        wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
    )

    # Check lowest altitude
    result_low = simulate_straight_ahead(
        config, weight, alt_low, airspeed_kias, **common)
    if not result_low['success']:
        return 0.0  # Can't even land straight from minimum altitude

    # Check upper bound
    result_high = simulate_straight_ahead(
        config, weight, alt_high, airspeed_kias, **common)
    if result_high['success']:
        return alt_high  # Works even at max altitude (very long runway)

    # Binary search
    lo, hi = alt_low, alt_high
    for _ in range(50):
        mid = (lo + hi) / 2.0
        result = simulate_straight_ahead(
            config, weight, mid, airspeed_kias, **common)
        if result['success']:
            lo = mid
        else:
            hi = mid
        if hi - lo < tolerance:
            break

    return math.floor(lo)


def find_critical_altitude(
    config, weight, airspeed_kias, bank_angle_deg, flap_setting,
    reaction_time, field_elevation=0.0, isa_dev=0.0,
    alt_low=50.0, alt_high=3000.0, tolerance=5.0,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    turn_direction='left',
    runway_length=0.0, liftoff_distance=0.0, aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
):
    """Find the minimum failure altitude that allows a safe return.

    The success profile may be non-monotonic (FAIL -> SUCCESS -> FAIL) when
    the runway model is active, so a pure binary search could miss the
    success band.  This scans in coarse steps first and then refines.

    Returns the critical altitude in feet AGL (within +/-tolerance).
    """
    common = dict(
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        aim_point=aim_point, flap_on_return=flap_on_return,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        touchdown_margin_ft=touchdown_margin_ft,
    )

    def _sim(alt):
        return simulate_turnback(
            config, weight, alt, airspeed_kias, bank_angle_deg,
            flap_setting, reaction_time, field_elevation, isa_dev,
            turn_direction, wind_speed_kt, wind_from_deg, **common,
        )

    # Scan from low to high in coarse steps to find the first success
    scan_step = 25.0
    first_success_alt = None
    alt = alt_low
    while alt <= alt_high:
        if _sim(alt)['success']:
            first_success_alt = alt
            break
        alt += scan_step

    if first_success_alt is None:
        return alt_high  # no altitude in the range succeeded

    if first_success_alt <= alt_low:
        return alt_low

    # Binary-search downward to refine the lower boundary
    lo = max(first_success_alt - scan_step, alt_low)
    hi = first_success_alt
    for _ in range(50):
        mid = (lo + hi) / 2.0
        if _sim(mid)['success']:
            hi = mid
        else:
            lo = mid
        if hi - lo < tolerance:
            break

    return math.ceil(hi)


def build_turnback_envelope(
    config, weight, airspeed_kias, bank_angle_deg, flap_setting,
    reaction_time, field_elevation=0.0, isa_dev=0.0,
    alt_step=100, max_alt=None,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    runway_length=0.0, liftoff_distance=0.0, aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
):
    """Build the full heart-shaped envelope at multiple failure altitudes.

    When the runway model is active (runway_length > 0), also computes the
    straight-ahead landing analysis at each altitude and the maximum altitude
    where a straight-ahead landing is feasible.

    Returns:
        critical_alt: overall turnback critical altitude (max of left and right)
        envelope: list of dicts, one per altitude, each containing:
            alt_agl, left, right, straight_ahead (if runway model),
            is_critical, is_critical_left, is_critical_right
        critical_alt_left: turnback critical altitude for left turns
        critical_alt_right: turnback critical altitude for right turns
        straight_ahead_max_alt: max altitude for straight-ahead landing (0 if N/A)
    """
    use_runway = runway_length > 0 and liftoff_distance > 0

    common = dict(
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        aim_point=aim_point, flap_on_return=flap_on_return,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        touchdown_margin_ft=touchdown_margin_ft,
    )

    critical_alt_left = find_critical_altitude(
        config, weight, airspeed_kias, bank_angle_deg, flap_setting,
        reaction_time, field_elevation, isa_dev,
        wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
        turn_direction='left',
        **common,
    )
    critical_alt_right = find_critical_altitude(
        config, weight, airspeed_kias, bank_angle_deg, flap_setting,
        reaction_time, field_elevation, isa_dev,
        wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
        turn_direction='right',
        **common,
    )

    # Overall critical altitude is the higher of the two (need to clear both)
    critical_alt = max(critical_alt_left, critical_alt_right)

    # Straight-ahead max altitude (runway model only)
    straight_ahead_max_alt = 0.0
    if use_runway:
        straight_ahead_max_alt = find_straight_ahead_max_altitude(
            config, weight, airspeed_kias, reaction_time,
            field_elevation=field_elevation, isa_dev=isa_dev,
            wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
            runway_length=runway_length, liftoff_distance=liftoff_distance,
            speed_mode=speed_mode,
            prop_state=prop_state,
            gear_down=gear_down,
            vbg_clean_kias=vbg_clean_kias,
            vbg_geardown_kias=vbg_geardown_kias,
            vbg_landing_kias=vbg_landing_kias,
        )

    if max_alt is None:
        max_alt = int(critical_alt + 400)
    max_alt = max(max_alt, int(critical_alt + 200))

    envelope = []

    # Build trajectories at every alt_step, plus both critical altitudes
    altitudes = list(range(alt_step, max_alt + 1, alt_step))
    for ca in (int(critical_alt_left), int(critical_alt_right)):
        if ca not in altitudes:
            altitudes.append(ca)
    # Include straight-ahead max altitude in the sweep
    sa_max_int = int(straight_ahead_max_alt)
    if use_runway and sa_max_int > 0 and sa_max_int not in altitudes:
        altitudes.append(sa_max_int)
    altitudes.sort()

    sa_common = dict(
        reaction_time=reaction_time,
        field_elevation=field_elevation, isa_dev=isa_dev,
        wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
    )

    for alt_agl in altitudes:
        left = simulate_turnback(
            config, weight, alt_agl, airspeed_kias, bank_angle_deg,
            flap_setting, reaction_time, field_elevation, isa_dev, 'left',
            wind_speed_kt, wind_from_deg, **common,
        )
        right = simulate_turnback(
            config, weight, alt_agl, airspeed_kias, bank_angle_deg,
            flap_setting, reaction_time, field_elevation, isa_dev, 'right',
            wind_speed_kt, wind_from_deg, **common,
        )

        item = {
            'alt_agl': alt_agl,
            'left': left,
            'right': right,
            'is_critical': alt_agl == int(critical_alt),
            'is_critical_left': alt_agl == int(critical_alt_left),
            'is_critical_right': alt_agl == int(critical_alt_right),
        }

        # Straight-ahead analysis at this altitude
        if use_runway:
            sa = simulate_straight_ahead(
                config, weight, alt_agl, airspeed_kias, **sa_common)
            item['straight_ahead'] = sa
            item['is_straight_ahead_max'] = alt_agl == sa_max_int

        envelope.append(item)

    return critical_alt, envelope, critical_alt_left, critical_alt_right, straight_ahead_max_alt


# ---------------------------------------------------------------------------
#  Optimizer
# ---------------------------------------------------------------------------

FLAP_STRATEGIES = {
    'clean':        {'label': 'Clean (no flaps)',           'setting': 0, 'on_return': False},
    'flaps_return': {'label': 'Flaps on final only',        'setting': None, 'on_return': True},
}


def optimize_turnback(
    config, weight, airspeed_kias, reaction_time,
    field_elevation=0.0, isa_dev=0.0,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    runway_length=0.0, liftoff_distance=0.0,
    bank_range=(10, 60), bank_step=2,
    alt_high=3000.0,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
):
    """Sweep bank angle, turn direction, and flap strategy to find the
    combination that gives the lowest critical altitude.

    Returns a list of dicts sorted by critical_altitude ascending (best first).
    Each dict contains: turn_direction, bank_angle, flap_strategy, flap_label,
    critical_altitude, turn_radius_ft, stall_speed_kt, overrun.
    """
    from engine.flight_physics import atmos

    results = []

    # Determine available flap settings from config
    has_flaps = getattr(config, 'dcdo_flap1', 0.0) > 0 or getattr(config, 'dcdo_flap2', 0.0) > 0
    max_flap = 0
    if getattr(config, 'dcdo_flap2', 0.0) > 0:
        max_flap = 2
    elif getattr(config, 'dcdo_flap1', 0.0) > 0:
        max_flap = 1

    # Single strategy: clean during turn, flaps on final only.
    strategies = ['flaps_return']

    # Pre-compute clean stall speed for reference
    _, _, sigma0, delta0, _, _ = atmos(field_elevation, isa_dev)
    S = config.wing_area
    clmax_clean = config.Clmax

    use_runway = runway_length > 0 and liftoff_distance > 0

    # aim_point is no longer used for turnback steering (always target
    # y=0 departure threshold), so no need to sweep aim points.
    aim_points = [0.0]

    # Pre-compute best glide speeds for reporting
    vbg_1g, cl_opt_1g, ld_max_1g = best_glide_kias(config, weight, 1.0, field_elevation, isa_dev)
    _vbg_ov = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
    if _vbg_ov > 0:
        vbg_1g = _vbg_ov

    # Coordinated turns enforced: only best_glide_nz is used so that
    # turn-phase KIAS increases with g-loading (Vbg × √nz).  In straight
    # flight phases (return, approach) the load factor is 1g, so the speed
    # naturally reverts to the 1g best-glide speed.
    speed_modes = ['best_glide_nz']

    for smode in speed_modes:
        for direction in ('left', 'right'):
            for bank_deg in range(bank_range[0], bank_range[1] + 1, bank_step):
                # Check stall margin — skip if bank angle would stall at this speed
                nz = 1.0 / math.cos(math.radians(bank_deg))
                rho_sl = 0.002378
                rho = rho_sl * sigma0
                vs_turn_ktas = math.sqrt(2.0 * nz * weight / (rho * S * clmax_clean)) / 1.6878
                vs_turn_kias = vs_turn_ktas * math.sqrt(sigma0)
                # Determine effective speed for stall check
                vbg_nz, _, _ = best_glide_kias(config, weight, nz, field_elevation, isa_dev)
                _vbg_ov_nz = _select_vbg_override(gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
                if _vbg_ov_nz > 0:
                    # POH override is a 1g speed; scale by sqrt(nz) for the bank
                    vbg_nz = _vbg_ov_nz * math.sqrt(nz)
                if smode == 'best_glide_nz':
                    eff_kias = vbg_nz
                elif smode == 'best_glide_1g':
                    eff_kias = vbg_1g
                else:
                    eff_kias = airspeed_kias
                if eff_kias < vs_turn_kias * 1.20:
                    continue  # below 1.2 Vs stall margin, skip

                for strat_key in strategies:
                    strat = FLAP_STRATEGIES[strat_key]
                    flap_setting = strat['setting'] if strat['setting'] is not None else max_flap
                    flap_on_return = strat['on_return']

                    for aim_pt in aim_points:
                        crit = find_critical_altitude(
                            config, weight, airspeed_kias, bank_deg, flap_setting,
                            reaction_time, field_elevation, isa_dev,
                            alt_high=alt_high,
                            wind_speed_kt=wind_speed_kt, wind_from_deg=wind_from_deg,
                            turn_direction=direction,
                            runway_length=runway_length, liftoff_distance=liftoff_distance,
                            aim_point=aim_pt, flap_on_return=flap_on_return,
                            speed_mode=smode,
                            prop_state=prop_state,
                            gear_down=gear_down,
                            vbg_clean_kias=vbg_clean_kias,
                            vbg_geardown_kias=vbg_geardown_kias,
                            vbg_landing_kias=vbg_landing_kias,
                            touchdown_margin_ft=touchdown_margin_ft,
                        )

                        # Check overrun at critical altitude
                        overrun = False
                        if use_runway and crit < alt_high:
                            test = simulate_turnback(
                                config, weight, crit, airspeed_kias, bank_deg,
                                flap_setting, reaction_time, field_elevation, isa_dev,
                                direction, wind_speed_kt, wind_from_deg,
                                runway_length=runway_length,
                                liftoff_distance=liftoff_distance,
                                aim_point=aim_pt,
                                flap_on_return=flap_on_return,
                                speed_mode=smode,
                                prop_state=prop_state,
                                gear_down=gear_down,
                                vbg_clean_kias=vbg_clean_kias,
                                vbg_geardown_kias=vbg_geardown_kias,
                                vbg_landing_kias=vbg_landing_kias,
                                touchdown_margin_ft=touchdown_margin_ft,
                            )
                            overrun = test.get('runway_overrun', False)

                        # Turn radius at this bank angle and speed
                        v_fps = eff_kias * 1.6878 / math.sqrt(sigma0)
                        turn_radius = v_fps ** 2 / (G_FPS2 * math.tan(math.radians(bank_deg)))

                        speed_mode_labels = {
                            'fixed': f'Fixed ({airspeed_kias} KIAS)',
                            'best_glide_1g': f'Best Glide 1g ({vbg_1g:.0f} KIAS)',
                            'best_glide_nz': f'Best Glide nz ({vbg_nz:.0f} KIAS)',
                        }

                        results.append({
                            'turn_direction': direction,
                            'bank_angle': bank_deg,
                            'flap_strategy': strat_key,
                            'flap_label': strat['label'],
                            'aim_point': aim_pt,
                            'critical_altitude': crit,
                            'turn_radius_ft': turn_radius,
                            'stall_speed_turn_kias': round(vs_turn_kias, 1),
                            'load_factor': round(nz, 2),
                            'overrun': overrun,
                            'speed_kias': round(eff_kias, 1),
                            'speed_1g_kias': round(vbg_1g, 1),
                            'speed_nz_kias': round(vbg_nz, 1),
                            'ld_max': round(ld_max_1g, 1),
                            'speed_mode': smode,
                            'speed_mode_label': speed_mode_labels[smode],
                        })

    # Sort: lowest critical altitude first, then by overrun (no overrun preferred)
    results.sort(key=lambda r: (r['critical_altitude'], r['overrun'], r['bank_angle']))

    return results
