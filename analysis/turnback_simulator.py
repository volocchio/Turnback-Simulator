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

import logging
import math
import time

from engine.flight_physics import atmos

log = logging.getLogger("turnback")

G_FPS2 = 32.174  # ft/s²
# 0.10 s keeps the critical-altitude search inside its ±5 ft tolerance
# versus a 0.05 s step (checked on a C172S, 45° bank, 10 kt quartering
# wind: 638 ft vs 641 ft) and halves the pure-Python loop.
DT = 0.10
_KT_TO_FPS = 6076.12 / 3600.0  # 1 knot in ft/s

# Prop drag increments (ΔCDo) after engine failure.
#
# Two prop types × distinct rotational states:
#   FIXED-PITCH (no feather capability):
#     fp_stopped          — prop stopped (engine seized or pilot brought airspeed
#                           below the prop windmill threshold).  Less drag.
#     fp_windmilling      — prop freewheeling in the airstream.  Default outcome
#                           for fixed-pitch singles after engine failure.
#   VARIABLE-PITCH (constant-speed / feathering):
#     vp_feathered_stopped  — blades fully feathered (edge-on) AND rotation
#                              stopped.  Minimum-drag case (turboprops, CS pistons
#                              that successfully feathered + slowed prop).
#     vp_feathered_spinning — blades feathered but disc still rotating slowly
#                              (residual oil pressure or pilot didn't get airspeed
#                              low enough to stop the prop).
#     vp_unfeathered        — CS prop with oil-pressure loss or pilot did not
#                              pull the blue lever; blades stay flat to the flow.
#                              Worst case.
#
# Legacy keys ('feathered', 'windmilling', 'fixed_pitch_stopped', 'stopped') are
# preserved as aliases for back-compat with old saved sessions and tests.
PROP_DRAG_INCREMENTS = {
    'fp_stopped':            0.0015,
    'fp_windmilling':        0.0020,
    'vp_feathered_stopped':  0.0005,
    'vp_feathered_spinning': 0.0010,
    'vp_unfeathered':        0.0040,
    # legacy aliases
    'feathered':             0.0005,
    'windmilling':           0.0020,
    'fixed_pitch_stopped':   0.0015,
    'stopped':               0.0040,
}


def _compute_aero_k(S, b, e, h_wl):
    """Induced drag factor k = 1/(π·e·AR_eff) with winglet correction."""
    AR_eff = b ** 2 / S * (1.0 + 1.9 * h_wl / b) if b > 0 else b ** 2 / S
    return 1.0 / (math.pi * e * AR_eff)


def _interpolate_wind_at_altitude(alt_agl, wind_sfc_kt, wind_1000_kt, wind_2000_kt, wind_3000_kt):
    """Interpolate wind speed at given altitude AGL using piecewise linear interpolation.
    
    Args:
        alt_agl: altitude AGL (ft)
        wind_sfc_kt: wind speed at surface (ft)
        wind_1000_kt: wind speed at 1000 ft AGL
        wind_2000_kt: wind speed at 2000 ft AGL
        wind_3000_kt: wind speed at 3000 ft AGL
    
    Returns:
        wind speed at the given altitude (knots)
    """
    if alt_agl <= 0:
        return wind_sfc_kt
    elif alt_agl <= 1000:
        # Linear interpolation between surface and 1000 ft
        frac = alt_agl / 1000.0
        return wind_sfc_kt + frac * (wind_1000_kt - wind_sfc_kt)
    elif alt_agl <= 2000:
        # Linear interpolation between 1000 ft and 2000 ft
        frac = (alt_agl - 1000.0) / 1000.0
        return wind_1000_kt + frac * (wind_2000_kt - wind_1000_kt)
    elif alt_agl <= 3000:
        # Linear interpolation between 2000 ft and 3000 ft
        frac = (alt_agl - 2000.0) / 1000.0
        return wind_2000_kt + frac * (wind_3000_kt - wind_2000_kt)
    else:
        # Above 3000 ft, assume constant wind
        return wind_3000_kt


def _interpolate_wind_profile(alt_agl, wind_sfc_kt, profile):
    """Piecewise-linear lookup of wind speed for an arbitrary winds-aloft profile.

    profile: iterable of (alt_agl_ft, speed_kt) pairs in ANY order.  The surface
             value (alt 0) is supplied separately as wind_sfc_kt and is always
             prepended.  Above the highest altitude the wind is held constant.

    Charlie #4: pilots get winds aloft at non-standard altitudes (e.g. 3000,
    6000, 9000 ft AGL).  This lets them feed those rows directly instead of
    being forced into 1000/2000/3000.
    """
    # Build sorted list with surface anchor; ignore any rows ≤ 0 in the user
    # data since the surface row is the wind_speed_kt argument.
    rows = [(0.0, float(wind_sfc_kt))]
    for a, s in profile:
        try:
            a_f = float(a)
            s_f = float(s)
        except (TypeError, ValueError):
            continue
        if a_f > 0.0:
            rows.append((a_f, s_f))
    rows.sort(key=lambda r: r[0])
    # Deduplicate identical altitudes (keep last)
    dedup = {}
    for a, s in rows:
        dedup[a] = s
    rows = sorted(dedup.items(), key=lambda r: r[0])

    if alt_agl <= rows[0][0]:
        return rows[0][1]
    if alt_agl >= rows[-1][0]:
        return rows[-1][1]
    for i in range(1, len(rows)):
        a_lo, s_lo = rows[i - 1]
        a_hi, s_hi = rows[i]
        if alt_agl <= a_hi:
            frac = (alt_agl - a_lo) / (a_hi - a_lo) if a_hi > a_lo else 0.0
            return s_lo + frac * (s_hi - s_lo)
    return rows[-1][1]


def _interpolate_wind_dir_profile(alt_agl, sfc_dir_deg, profile):
    """Piecewise sin/cos interpolation of wind direction (deg) vs altitude AGL.

    Charlie #E3: wind veers with altitude (Northern hemisphere typical: surface
    wind backs ~20–30° from the gradient wind).  Linear interpolation in
    degrees breaks at the 0/360 wrap, so we interpolate the unit-vector
    components instead and convert back.

    profile: iterable of (alt_agl_ft, direction_deg_relative_to_runway).
             Surface direction is supplied separately as sfc_dir_deg.
             Above the highest altitude the direction is held constant.

    Returns direction in degrees (relative to runway) in [0, 360).
    """
    rows = [(0.0, float(sfc_dir_deg) % 360.0)]
    for a, d in profile:
        try:
            a_f = float(a)
            d_f = float(d) % 360.0
        except (TypeError, ValueError):
            continue
        if a_f > 0.0:
            rows.append((a_f, d_f))
    rows.sort(key=lambda r: r[0])
    dedup = {}
    for a, d in rows:
        dedup[a] = d
    rows = sorted(dedup.items(), key=lambda r: r[0])

    if alt_agl <= rows[0][0]:
        return rows[0][1]
    if alt_agl >= rows[-1][0]:
        return rows[-1][1]
    for i in range(1, len(rows)):
        a_lo, d_lo = rows[i - 1]
        a_hi, d_hi = rows[i]
        if alt_agl <= a_hi:
            frac = (alt_agl - a_lo) / (a_hi - a_lo) if a_hi > a_lo else 0.0
            # Interpolate as unit vectors to avoid 0/360 wrap issues
            r_lo = math.radians(d_lo)
            r_hi = math.radians(d_hi)
            sx = (1.0 - frac) * math.sin(r_lo) + frac * math.sin(r_hi)
            cy = (1.0 - frac) * math.cos(r_lo) + frac * math.cos(r_hi)
            ang = math.degrees(math.atan2(sx, cy)) % 360.0
            return ang
    return rows[-1][1]


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
    """Convert KIAS to true airspeed in ft/s.

    Compressibility (IAS versus EAS) is neglected. Below about Mach 0.3
    the difference is under one percent, which covers the light-aircraft
    turnback envelope. The middle return is therefore KEAS under that
    approximation (KIAS is treated as KEAS). Dynamic pressure used by
    the rest of the model is q (psf) = KEAS² / 295.

    ``delta`` is retained so callers stay source-compatible; it does not
    enter the low-Mach conversion.
    """
    del delta  # see docstring — kept for call-site compatibility
    try:
        kias_f = float(kias)
    except (TypeError, ValueError):
        kias_f = 0.0
    if kias_f != kias_f or kias_f < 0.0:  # NaN or negative
        kias_f = 0.0
    try:
        sigma_f = float(sigma)
    except (TypeError, ValueError):
        sigma_f = 1.0
    if sigma_f != sigma_f or sigma_f < 1e-4:
        sigma_f = 1e-4
    vkeas = kias_f
    vktas = vkeas / math.sqrt(sigma_f)
    v_fps = vktas * _KT_TO_FPS
    return v_fps, vkeas, vktas


def _finite(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return False
    return math.isfinite(v)


def _aircraft_model_key(config):
    """Model name ('C172S') when ``config`` is the library instance."""
    try:
        from engine.aircraft_config import AIRCRAFT_CONFIG
    except Exception:
        return None
    for key, cfg in AIRCRAFT_CONFIG.items():
        if cfg is config:
            return key[0]
    return None


def _landing_polar(config):
    """In-flight full-flap (ΔCDo, CLmax).

    The aircraft table's third drag increment is full-flap drag for this
    single-engine library (the dataclass comment still says "ground
    spoiler" because the schema was inherited from a jet performance
    tool). Pair it with Clmax_flaps40. Fall back to the largest increment
    that is actually populated.
    """
    d3 = float(getattr(config, "dcdo_flap3", 0.0) or 0.0)
    d2 = float(getattr(config, "dcdo_flap2", 0.0) or 0.0)
    d1 = float(getattr(config, "dcdo_flap1", 0.0) or 0.0)
    cl40 = float(getattr(config, "Clmax_flaps40", 0.0) or 0.0)
    cl15 = float(getattr(config, "Clmax_flaps15", 0.0) or 0.0)
    cl_clean = float(getattr(config, "Clmax", 0.0) or 0.0)
    if d3 > 0.0 and cl40 > 0.0:
        return d3, cl40
    if d2 > 0.0:
        return d2, (cl40 or cl15 or cl_clean)
    if d1 > 0.0:
        return d1, (cl15 or cl_clean)
    return 0.0, cl_clean


def _decimate_trajectory(traj, max_points=500):
    """Thin a trajectory for storage and plotting.

    Integration and the reported statistics use the full step history.
    The list returned to the UI is what gets pickled into the Streamlit
    session and shipped to Plotly / pydeck. Keeping every 0.1 s sample
    of a multi-orbit path is tens of megabytes and is what made the
    first page paint sit on "Computing trajectories...".

    Phase-change samples and both endpoints are always retained so the
    drawn heart and the phase coloring stay faithful.
    """
    n = len(traj)
    if n <= max_points:
        return traj
    must = {0, n - 1}
    prev_phase = traj[0].get("phase")
    for i in range(1, n):
        ph = traj[i].get("phase")
        if ph != prev_phase:
            must.add(i - 1)
            must.add(i)
            prev_phase = ph
    budget = max_points - len(must)
    if budget < 2:
        idx = sorted(must)[:max_points]
    else:
        step = max(1, int(n / budget))
        idx = sorted(must | set(range(0, n, step)))
        if len(idx) > max_points:
            stride = len(idx) / float(max_points)
            picked = [idx[min(n - 1, int(i * stride))] for i in range(max_points)]
            picked[0] = 0
            picked[-1] = n - 1
            idx = sorted(set(picked) | {0, n - 1})
    return [traj[i] for i in idx]


def _return_heading(x, y, target_y):
    """Heading (rad, 0 = +Y) that captures the centerline, then tracks it.

    Pointing straight at the touchdown target only crosses x = 0 at that
    target. A glide that reaches the pavement short of the target is then
    still offset by a large fraction of the turn diameter — beside the
    runway, inside the 75 ft lateral gate. Lead the aim by about three
    cross-track distances so the track is on the centerline before
    touchdown, which is what a pilot does on the intercept.
    """
    cross = abs(x)
    remaining = target_y - y
    if cross < 40.0 or abs(remaining) < 1.0:
        dy = remaining if abs(remaining) >= 1.0 else (1.0 if remaining >= 0.0 else -1.0)
        return math.atan2(-x, dy)
    lead = max(300.0, min(3.0 * cross, abs(remaining)))
    if remaining >= 0.0:
        aim_y = min(y + lead, target_y)
    else:
        aim_y = max(y - lead, target_y)
    dy = aim_y - y
    if abs(dy) < 1.0:
        dy = 1.0 if remaining >= 0.0 else -1.0
    return math.atan2(-x, dy)


def _aborted_result(reason):
    """Structured failure for non-physical inputs. Never raises."""
    log.warning("turnback aborted: %s", reason)
    return {
        "trajectory": [],
        "success": False,
        "altitude_at_runway": None,
        "altitude_at_departure_threshold": None,
        "altitude_at_takeoff_threshold": None,
        "stalled": False,
        "stall_time": None,
        "turn_radius_ft": float("inf"),
        "ground_impact": None,
        "dist_from_threshold": 0.0,
        "landing_rollout": 0.0,
        "runway_overrun": False,
        "remaining_runway": None,
        "computed_aim_y": 0.0,
        "speed_info": {"mode": "aborted", "reason": reason},
        "landing_direction": "reverse",
        "total_turn_deg": 0.0,
        "altitude_loss_per_180": None,
        "altitude_at_180_increments": [],
        "phase_summary": [],
    }


def _estimate_climb_distance(config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_kias,
                             wind_x_fps=0.0, wind_y_fps=0.0, climb_steering="track"):
    """Horizontal distance from liftoff to engine failure, and the time to get there.

    Positioning uses published POH rate of climb (scaled for weight and
    density) when the aircraft is in the POH table. The previous model
    scaled a light-jet thrust polynomial by ``thrust_mult``. That polynomial
    over-states piston climb thrust, which put the failure too close to the
    runway and made the turnback look easier than the POH climb gradient.

    Aircraft without a POH row still use the polynomial, with the same 2%
    gradient floor as before.

    Wind, track-hold only: the pilot crabs so the ground track stays on
    centerline. Distance along the runway is groundspeed × time. A headwind
    (``wind_y_fps`` < 0; +Y is the takeoff direction) shortens it. Heading-hold
    returns the still-air horizontal air distance; the caller adds the
    wind-drift offset on top, which is the contract the heading-hold tests use.
    """
    failure_alt_agl = max(float(failure_alt_agl), 0.0)
    mid_alt = field_elevation + failure_alt_agl / 2.0
    _, _, sigma, delta, _, c_kt = atmos(mid_alt, isa_dev)
    v_fps, vkeas, _ = _kias_to_fps(airspeed_kias, sigma, delta)
    v_fps = max(v_fps, 1.0)

    roc_fps = None
    model_key = _aircraft_model_key(config)
    if model_key:
        try:
            from engine.poh_data import roc_poh_fpm
            roc_fpm = roc_poh_fpm(model_key, config, weight, mid_alt, isa_dev)
        except Exception:
            roc_fpm = None
            log.exception("POH climb lookup failed for %s", model_key)
        if roc_fpm and roc_fpm > 50.0:
            roc_fps = roc_fpm / 60.0

    if roc_fps is None:
        q = vkeas ** 2 / 295.0
        S = config.wing_area
        k = _compute_aero_k(S, config.span, config.oswald, config.winglet_height)
        cl = weight / (q * S) if q * S > 0 else 1.0
        cd = config.cdo + k * cl ** 2
        drag = cd * q * S
        d_alt_mid = atmos(mid_alt, isa_dev)[0]
        m_est = airspeed_kias / max(c_kt, 100.0)
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
        thrust = thrust_reg * config.thrust_mult * config.engines
        if thrust < 100:
            thrust = 500
        sin_g = (thrust - drag) / max(weight, 1.0)
        sin_g = min(max(sin_g, 0.02), 0.99)
        roc_fps = v_fps * sin_g

    roc_fps = max(float(roc_fps), 0.5)
    sin_g = min(0.99, roc_fps / v_fps)
    cos_g = math.sqrt(max(0.0, 1.0 - sin_g * sin_g))
    v_horiz = v_fps * cos_g
    time_s = failure_alt_agl / roc_fps

    if climb_steering == "track":
        # Crab cancels the crosswind. Along-track airspeed is what remains.
        cross = abs(float(wind_x_fps))
        along_air = math.sqrt(max(v_horiz * v_horiz - min(cross, v_horiz) ** 2, 1.0))
        gs = along_air + float(wind_y_fps)
        distance_ft = max(0.0, gs * time_s)
    else:
        distance_ft = v_horiz * time_s
    return distance_ft, time_s


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


def vs_kias_at_load(config, weight, nz, field_elevation, isa_dev,
                    landing_flaps=False):
    """Compute Vs (KIAS) at the given load factor.

    Vs(nz) = sqrt(2·nz·W / (ρ·S·CLmax))  → KIAS via density correction.

    Used by the operational glide-speed presets recommended by Charlie
    Precourt: "Vs at the selected bank, plus 10 kt" or "1.3 × Vs at the
    selected bank".  These are the speeds we want pilots to actually fly
    in the turn — safely above stall while close enough to the
    aerodynamic optimum.

    Args:
        config: aircraft config (provides wing_area, Clmax, Clmax_flapsXX).
        weight: gross weight (lb).
        nz: load factor (1g level, 1/cos(phi) in a banked level turn).
        field_elevation: ft MSL — sets the density at evaluation altitude.
        isa_dev: ISA temperature deviation (°C).
        landing_flaps: if True, use the landing CLmax instead of clean.

    Returns:
        Vs (KIAS) at the given load factor.
    """
    from engine.flight_physics import atmos
    if landing_flaps:
        clmax = (getattr(config, 'Clmax_flaps40', 0.0) or
                 getattr(config, 'Clmax_flaps15', 0.0) or
                 config.Clmax)
    else:
        clmax = config.Clmax
    if clmax <= 0:
        return 0.0
    _, _, sigma, _, _, _ = atmos(field_elevation, isa_dev)
    rho = 0.002378 * sigma
    vs_fps = math.sqrt(2.0 * nz * weight / (rho * config.wing_area * clmax))
    vs_keas = vs_fps / 1.6878
    vs_kias = vs_keas * math.sqrt(sigma)
    return vs_kias


def operational_turn_speeds(config, weight, bank_angle_deg, field_elevation,
                            isa_dev, landing_flaps=False):
    """Return the two operational turn-speed targets at a given bank.

    Returns dict with:
        nz, vs_kias, vs_plus_10_kias, vs_x_1p3_kias
    """
    nz = 1.0 / math.cos(math.radians(bank_angle_deg)) if bank_angle_deg < 89.5 else 100.0
    vs = vs_kias_at_load(config, weight, nz, field_elevation, isa_dev,
                         landing_flaps=landing_flaps)
    return {
        'nz': nz,
        'vs_kias': vs,
        'vs_plus_10_kias': vs + 10.0,
        'vs_x_1p3_kias': vs * 1.3,
    }


def _compute_turn_statistics(trajectory, failure_alt_agl):
    """Compute training-output stats from a trajectory:
        - total_turn_deg:  sum of |Δheading| over the entire maneuver
        - altitude_loss_per_180:  average altitude loss per 180° of turning
          (None if the maneuver doesn't include a full 180° of turn)
        - altitude_at_180_increments: list of altitudes AGL at each
          180° increment of cumulative turning, starting at 0° = failure_alt_agl

    Charlie Precourt asked for this so a pilot can practice the maneuver
    at altitude and match the per-half-turn altitude loss before applying
    a personal safety factor.
    """
    if not trajectory:
        return {
            'total_turn_deg': 0.0,
            'altitude_loss_per_180': None,
            'altitude_at_180_increments': [],
        }

    total_turn_deg = trajectory[-1].get('cum_turn_deg', 0.0)

    increments = [{'turn_deg': 0, 'altitude_agl': failure_alt_agl}]
    next_target_deg = 180
    while next_target_deg <= total_turn_deg + 0.01:
        # Find the first trajectory point with cum_turn_deg >= next_target_deg
        for i, pt in enumerate(trajectory):
            if pt.get('cum_turn_deg', 0.0) >= next_target_deg:
                # Linear interpolation between i-1 and i for accuracy
                if i == 0:
                    z_at = pt['z']
                else:
                    prev = trajectory[i - 1]
                    d0 = prev.get('cum_turn_deg', 0.0)
                    d1 = pt.get('cum_turn_deg', 0.0)
                    if d1 > d0:
                        frac = (next_target_deg - d0) / (d1 - d0)
                    else:
                        frac = 0.0
                    z_at = prev['z'] + frac * (pt['z'] - prev['z'])
                increments.append({'turn_deg': next_target_deg,
                                    'altitude_agl': z_at})
                break
        next_target_deg += 180

    if len(increments) >= 2:
        # Average altitude loss per 180° = (z0 - z_last) / number_of_180s
        z0 = increments[0]['altitude_agl']
        z_last = increments[-1]['altitude_agl']
        n_180 = (len(increments) - 1)
        loss_per_180 = (z0 - z_last) / n_180
    else:
        loss_per_180 = None

    return {
        'total_turn_deg': round(total_turn_deg, 1),
        'altitude_loss_per_180': round(loss_per_180, 1) if loss_per_180 is not None else None,
        'altitude_at_180_increments': increments,
    }


# --- Per-phase summary (Charlie #F4) ----------------------------------------
def _compute_phase_summary(trajectory, failure_alt_agl):
    """Return ordered list of phase blocks for the data-card breakdown.

    Each block: {phase, t_start_s, t_end_s, dt_s, z_start_agl, z_end_agl,
                 dz_loss_ft, heading_change_deg, explainer}

    The explainer is a short one-liner the pilot can read on the card.
    Uses the same phase tags emitted by simulate_turnback ('reaction',
    'turn', 'return', 'orbit').
    """
    if not trajectory:
        return []

    EXPLAINERS = {
        'reaction': "Engine quits — wings level, pitch for glide, identify the failure. "
                    "No turn yet; altitude is bleeding off in straight flight.",
        'turn': "Coordinated bank toward the runway.  This is where most altitude "
                "is lost — sink rate scales with 1/cos(bank).",
        'return': "Wings rolled out on a track back to the runway.  Flaps may "
                  "deploy here to bleed energy and slow the approach.",
        'orbit': "Aircraft arrived high or off-line — extra ~360° to bleed "
                 "energy before lining up on the runway in the takeoff direction.",
    }

    blocks = []
    cur_phase = trajectory[0].get('phase', '?')
    seg_start_idx = 0
    seg_start_z = failure_alt_agl
    seg_start_t = trajectory[0].get('time', 0.0)
    seg_start_hdg = trajectory[0].get('heading_deg', 0.0)
    cum_start = trajectory[0].get('cum_turn_deg', 0.0)

    def _emit(end_idx, end_phase_label):
        end_pt = trajectory[end_idx]
        z_end = end_pt['z']
        t_end = end_pt.get('time', 0.0)
        cum_end = end_pt.get('cum_turn_deg', 0.0)
        blocks.append({
            'phase': end_phase_label,
            't_start_s': round(seg_start_t, 1),
            't_end_s': round(t_end, 1),
            'dt_s': round(t_end - seg_start_t, 1),
            'z_start_agl': round(seg_start_z, 0),
            'z_end_agl': round(z_end, 0),
            'dz_loss_ft': round(seg_start_z - z_end, 0),
            'heading_change_deg': round(cum_end - cum_start, 0),
            'explainer': EXPLAINERS.get(end_phase_label, ""),
        })

    for i in range(1, len(trajectory)):
        ph = trajectory[i].get('phase', '?')
        if ph != cur_phase:
            _emit(i - 1, cur_phase)
            cur_phase = ph
            seg_start_z = trajectory[i - 1]['z']
            seg_start_t = trajectory[i - 1].get('time', 0.0)
            cum_start = trajectory[i - 1].get('cum_turn_deg', 0.0)
    # Final segment
    _emit(len(trajectory) - 1, cur_phase)
    return blocks


# --- Departure-end-threshold altitude (Charlie #F3) -------------------------
def _altitude_at_y(trajectory, target_y):
    """Find the altitude (ft AGL) when the trajectory first crosses y=target_y
    AFTER the maneuver has begun a turn (cum_turn_deg > 90°).

    Returns None if no such crossing exists.  Linear-interpolates between
    integration steps for accuracy.
    """
    if not trajectory or len(trajectory) < 2:
        return None
    prev = trajectory[0]
    for pt in trajectory[1:]:
        # Only look after the aircraft has started turning back
        if pt.get('cum_turn_deg', 0.0) < 90.0:
            prev = pt
            continue
        py = prev['y']
        cy = pt['y']
        if (py - target_y) * (cy - target_y) <= 0 and py != cy:
            frac = (target_y - py) / (cy - py)
            return prev['z'] + frac * (pt['z'] - prev['z'])
        prev = pt
    return None


# ---------------------------------------------------------------------------
#  Maneuver classification (May 2026)
#
#  Each successful sim run lands either on the takeoff runway (same-direction
#  "circuit", landing_direction='original') or on the reciprocal end
#  ('reverse').  Within the reverse case, the airplane may add a 360° orbit
#  before landing if it has excess altitude — yielding ~540° of total turn.
#  The pilot needs to know the MINIMUM altitude that buys each option.
# ---------------------------------------------------------------------------

MANEUVER_KINDS = ('180', '270', '540', 'circuit')

# A continuous turn onto the reciprocal final routinely passes 180° and
# finishes between ~210° and ~300° with NO extra circle. That is the
# classic "270" alignment (Rogers / AFH), not a 540. Calling everything
# at or above 270° a "540" mislabeled that geometry as an orbit.
# A real 540 is an explicit orbit phase, or a heading change large
# enough that a full extra circle was flown on top of the reversal
# (180 + most of 360 ≈ 400°).
_TEARDROP_TURN_MIN_DEG = 210.0
_ORBIT_TURN_THRESHOLD_DEG = 400.0


def classify_maneuver(sub: dict) -> str | None:
    """Classify a single sim sub-result into one of MANEUVER_KINDS.

    Returns None if the sim did not succeed.

    '180'     — reversal, heading change under 210°, land opposite.
    '270'     — continued turn to align on final, no orbit, land opposite.
    '540'     — an orbit was flown (or heading change ≥ 400°) then opposite.
    'circuit' — landing in the original takeoff direction.
    """
    if not sub or not sub.get('success'):
        return None
    if sub.get('landing_direction') == 'original':
        return 'circuit'
    phases = {b.get('phase') for b in (sub.get('phase_summary') or [])}
    total = float(sub.get('total_turn_deg') or 0.0)
    if 'orbit' in phases or total >= _ORBIT_TURN_THRESHOLD_DEG:
        return '540'
    if total >= _TEARDROP_TURN_MIN_DEG:
        return '270'
    return '180'


def find_min_alt_per_maneuver(envelope) -> dict:
    """Walk an envelope (list of altitude rows) and return, for each
    maneuver kind, the lowest altitude at which at least one side
    (LEFT or RIGHT) succeeds with that kind.

    Returns a dict ``{kind: {'alt': int, 'side': str, 'sub': dict}}``.
    Kinds with no successful row are omitted from the dict.
    """
    out: dict = {}
    for row in envelope or []:
        for side in ('left', 'right'):
            sub = row.get(side) or {}
            kind = classify_maneuver(sub)
            if kind is None:
                continue
            if kind not in out or row.get('alt_agl', 0) < out[kind]['alt']:
                out[kind] = {
                    'alt': int(row.get('alt_agl', 0)),
                    'side': side,
                    'sub': sub,
                }
    return out




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
    wind_1000_kt=0.0,
    wind_2000_kt=0.0,
    wind_3000_kt=0.0,
    wind_profile=None,
    wind_dir_profile=None,
    runway_length=0.0,
    liftoff_distance=0.0,
    aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
    runway_friction=1.0,
    climb_steering='track',
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
    if (not _finite(weight) or weight <= 0
            or not _finite(getattr(config, "wing_area", None)) or config.wing_area <= 0):
        return _aborted_result("weight and wing area must be finite and positive")
    if (not _finite(failure_alt_agl) or not _finite(airspeed_kias)
            or not _finite(reaction_time) or not _finite(field_elevation)):
        return _aborted_result("altitude, airspeed, reaction time, and elevation must be finite")
    if airspeed_kias <= 0:
        return _aborted_result("airspeed must be positive")
    if not _finite(bank_angle_deg):
        bank_angle_deg = 0.0
    # Past 89° the level-turn load factor 1/cos(φ) is no longer a coordinated
    # turn a pilot can fly. Clamp rather than emit NaN from tan/cos.
    bank_angle_deg = min(max(float(bank_angle_deg), 0.0), 89.0)
    failure_alt_agl = max(float(failure_alt_agl), 0.0)
    reaction_time = max(float(reaction_time), 0.0)

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
        dcdo_flap, clmax_flap = _landing_polar(config)
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

    # Full-flap polar for the return. Previously this paired Clmax_flaps40
    # with the *middle* drag increment (dcdo_flap2), so "landing / full"
    # had full-flap lift and only approach-flap drag. The glide on final
    # was too flat.
    dcdo_landing, clmax_landing = _landing_polar(config)

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
    # NOTE: Wind speed will be interpolated per timestep based on altitude.
    wind_from_rad = math.radians(wind_from_deg)
    # Surface wind velocity. +X is right of centerline, +Y is the takeoff
    # direction. wind_from 0° is a headwind, so the velocity vector points
    # in −Y. 90° (from the right) blows toward −X.
    _wind_fps_sfc = wind_speed_kt * _KT_TO_FPS
    _wind_x_sfc = -_wind_fps_sfc * math.sin(wind_from_rad)
    _wind_y_sfc = -_wind_fps_sfc * math.cos(wind_from_rad)

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
        # Braking friction coefficient scaled by runway_friction
        # runway_friction = 1.0 for dry asphalt (mu = 0.3)
        # runway_friction = 0.7 for wet asphalt (mu ≈ 0.21)
        mu_brake = 0.3 * runway_friction  # scale baseline mu by runway condition
        rollout_est = v_td_est ** 2 / (2.0 * G_FPS2 * mu_brake)
        target_y = rollout_est + touchdown_margin_ft
    else:
        target_y = 0.0

    climb_dist, climb_time_s = _estimate_climb_distance(
        config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_for_climb,
        wind_x_fps=_wind_x_sfc, wind_y_fps=_wind_y_sfc, climb_steering=climb_steering,
    )
    dist_from_threshold = liftoff_distance + climb_dist

    # E2: climb steering toggle.  In 'track' mode (default) the pilot crabs
    # into wind so the ground track stays on centerline (x_init = 0) and the
    # along-track distance above already includes headwind/tailwind.
    # In 'heading' mode the nose is held on runway heading and the airplane
    # drifts downwind during the climb — by failure altitude the aircraft
    # is offset from centerline by wind × climb_time.  Surface wind is used
    # (the climb is mostly low-altitude).
    if climb_steering == 'heading':
        climb_drift_x = _wind_x_sfc * climb_time_s
        climb_drift_y = _wind_y_sfc * climb_time_s
    else:
        climb_drift_x = 0.0
        climb_drift_y = 0.0

    # --- Initial state ---
    x = climb_drift_x            # lateral offset from runway centerline (ft, + = right)
    y = intersection_offset_ft + dist_from_threshold + climb_drift_y
    z = failure_alt_agl          # altitude AGL (ft)
    # Heading at engine failure depends on climb-steering mode (May 2026 fix):
    #  - 'heading' hold: nose on runway heading the whole climb → heading = 0.
    #  - 'track' hold (crab): pilot points into the wind so the ground track
    #    stays on centerline; at failure the nose is offset from runway
    #    heading by the crab angle, where  V_TAS · sin(crab) + wind_x = 0.
    #    For a right crosswind (wind_x < 0) this gives a positive crab
    #    (nose right of runway).  Turning INTO the wind from this heading
    #    therefore requires LESS than 180° of heading change — pilots turning
    #    into the wind get a free head start equal to the crab angle.
    if climb_steering == 'track' and abs(_wind_x_sfc) > 0.5:
        try:
            _, _, _sigma_climb, _delta_climb, _, _ = atmos(field_elevation, isa_dev)
            _vclimb_fps, _, _ = _kias_to_fps(airspeed_for_climb, _sigma_climb, _delta_climb)
            if _vclimb_fps > 1.0:
                _sin_crab = max(-0.99, min(0.99, -_wind_x_sfc / _vclimb_fps))
                heading = math.asin(_sin_crab)
            else:
                heading = 0.0
        except Exception:
            heading = 0.0
    else:
        heading = 0.0                # radians, 0 = runway heading (+Y direction)
    t = 0.0
    phase = 'reaction'
    total_heading_change = 0.0
    cum_turn_rad = 0.0           # cumulative |Δheading| across ALL phases (for per-180° loss)
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
    gear_retracted = False  # E2: tracks whether post-failure gear retraction has fired

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
    elif speed_mode in ('vs_plus_10', 'vs_x_1p3'):
        # Operational presets: target = Vs(nz_bank) +10 kt or × 1.3
        _vs_turn = vs_kias_at_load(config, weight, nz_bank,
                                    field_elevation, isa_dev,
                                    landing_flaps=False)
        if speed_mode == 'vs_plus_10':
            vbg_turn = _vs_turn + 10.0
        else:
            vbg_turn = _vs_turn * 1.3
        v_fps_init, _, _ = _kias_to_fps(vbg_turn, sigma_init, delta_init)
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

        # --- Interpolate wind at current altitude ---
        if wind_profile:
            wind_speed_at_alt = _interpolate_wind_profile(
                z, wind_speed_kt, wind_profile
            )
        else:
            wind_speed_at_alt = _interpolate_wind_at_altitude(
                z, wind_speed_kt, wind_1000_kt, wind_2000_kt, wind_3000_kt
            )
        # E3: optional per-altitude wind direction profile (rel. to runway).
        # Falls back to the constant surface wind_from_rad if not supplied.
        if wind_dir_profile:
            _dir_deg_at_alt = _interpolate_wind_dir_profile(
                z, wind_from_deg, wind_dir_profile
            )
            _wind_from_rad_now = math.radians(_dir_deg_at_alt)
        else:
            _wind_from_rad_now = wind_from_rad
        # E2: gear-retraction event during the post-failure trajectory.
        if (gear_retract_time_s is not None and gear_down
                and t >= gear_retract_time_s and config.dcdo_gear > 0
                and not gear_retracted):
            cdo_clean -= config.dcdo_gear
            cdo -= config.dcdo_gear
            gear_retracted = True
        wind_fps = wind_speed_at_alt * 6076.12 / 3600.0
        wind_x = -wind_fps * math.sin(_wind_from_rad_now)   # lateral ft/s (+ = rightward)
        wind_y = -wind_fps * math.cos(_wind_from_rad_now)    # along-runway ft/s (+ = with takeoff direction)

        # --- Phase logic (runs first so flap/nz state is current) ---
        # target_bank_deg is set per-phase; actual_bank_deg ramps toward it.
        target_bank_deg = 0.0
        if phase == 'reaction':
            target_bank_deg = 0.0
            if t >= reaction_time:
                phase = 'turn'
        elif phase == 'turn':
            target_bank_deg = bank_angle_deg
            # Check if heading now points at any usable point on the runway.
            # SMART AIM POINT (P1, May 2026): Instead of always aiming at the
            # original threshold-end target, project the airplane's current
            # velocity vector onto the runway centerline (x = 0) and use that
            # intersection as the aim point if it falls within usable runway
            # bounds.  This minimizes the total heading change required and
            # exploits any pre-failure crosswind drift — heading-hold climbouts
            # automatically benefit from a shorter turn.  Falls back to the
            # original 8°-tolerance check on the threshold-end aim if the
            # smart projection isn't usable yet.
            if total_heading_change > math.pi * 0.75:  # at least 135° before checking
                smart_exit = False
                # Smart aim only applies when a runway model is active —
                # without a defined runway we have no meaningful aim-range
                # constraint, so retain the original behavior.
                if use_runway and runway_length > 0:
                    sin_h = math.sin(heading)
                    cos_h = math.cos(heading)
                    if abs(sin_h) > 1e-3:
                        s_to_centerline = -x / sin_h  # forward distance to x=0 line
                        if s_to_centerline > 0:
                            aim_y_candidate = y + cos_h * s_to_centerline
                            # Usable aim range along runway centerline.  For
                            # a reverse landing the airplane crosses the far
                            # end heading -y and must touch down with at
                            # least rollout_est + margin of runway ahead.
                            aim_y_min = rollout_est + touchdown_margin_ft
                            aim_y_max = runway_length - touchdown_margin_ft
                            if aim_y_min <= aim_y_candidate <= aim_y_max:
                                smart_exit = True
                                target_y = aim_y_candidate
                                # heading already aligned with bearing-to-aim
                                # by construction (that's how we picked aim_y).
                if smart_exit:
                    phase = 'return'
                    if use_runway:
                        cdo = cdo_clean + dcdo_landing
                        clmax = clmax_landing
                    elif flap_on_return and flap_setting > 0:
                        cdo = cdo_clean + dcdo_flap
                        clmax = clmax_flap
                else:
                    # Fallback: original threshold-end aim with 8° tolerance.
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
                if use_runway and landing_direction != 'original' and y < target_y:
                    # Past the reverse aim point — keep a nose toward the
                    # centerline while the rollout check decides success.
                    heading = math.atan2(-x, -500.0)
                elif abs(target_y - y) > 1.0 or abs(x) > 1.0:
                    heading = _return_heading(x, y, target_y)

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
        elif speed_mode in ('vs_plus_10', 'vs_x_1p3'):
            # Operational presets: target = Vs(nz) +10 kt or × 1.3
            # Use current local density (sigma_now) and current CLmax
            # (which already reflects flap state) so the target tracks
            # configuration changes during the maneuver.
            _rho_now = 0.002378 * sigma_now
            _vs_now_fps = math.sqrt(2.0 * nz * weight / (_rho_now * S * clmax)) if clmax > 0 else 100.0
            _vs_now_kias = (_vs_now_fps / 1.6878) * math.sqrt(sigma_now)
            if speed_mode == 'vs_plus_10':
                target_kias = _vs_now_kias + 10.0
            else:
                target_kias = _vs_now_kias * 1.3
        else:
            if phase in ('return', 'orbit', 'circuit') and (use_runway or (flap_on_return and flap_setting > 0)):
                _rho_now = 0.002378 * sigma_now
                _vs_now = math.sqrt(2.0 * weight / (_rho_now * S * clmax)) if clmax > 0 else 100.0
                _vs_kias = (_vs_now / 1.6878) * math.sqrt(sigma_now)
                target_kias = _vs_kias * 1.3
            else:
                target_kias = airspeed_kias

        # === Speed ramping ===
        # Ramp actual_kias toward target_kias at ACCEL_LIMIT_KT_S.
        # Holding a new speed trades kinetic and potential energy:
        #   Δh = (V_before² − V_after²) / (2g)
        # evaluated at the current density, so a density-driven TAS change
        # at constant KIAS is not mistaken for a pitch change. Decelerating
        # to glide speed gives altitude back; accelerating into the bank
        # (Vbg × √nz) spends it. The steady sink rate below is the
        # constant-speed term V·sinγ = −V·D/W and does not include this.
        kias_before_ramp = actual_kias
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
        energy_dh = 0.0
        if abs(actual_kias - kias_before_ramp) > 1e-6:
            v_before, _, _ = _kias_to_fps(kias_before_ramp, sigma_now, delta_now)
            energy_dh = (v_before * v_before - v_fps_actual * v_fps_actual) / (2.0 * G_FPS2)
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
            'cum_turn_deg': math.degrees(cum_turn_rad),
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
        cum_turn_rad += abs(omega * DT)
        # Keep heading in [-2π, 2π] range
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
        elif heading < -2 * math.pi:
            heading += 2 * math.pi

        if phase == 'turn':
            total_heading_change += abs(omega * DT)

        x += (v_horiz * math.sin(heading) + wind_x) * DT
        y += (v_horiz * math.cos(heading) + wind_y) * DT
        z += v_vert * DT + energy_dh
        t += DT
        if stalled:
            # Parabolic polar is not valid past CLmax. Stop rather than
            # keep integrating a wing that has departed.
            success = False
            break
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            log.warning("non-finite state at t=%.2fs; stopping integration", t)
            success = False
            break

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

    # A stall is not a successful return, even if the extrapolated polar
    # would have put the airplane on the pavement.
    if stalled:
        success = False

    # Runway model: determine success from actual ground-contact position
    if use_runway and not success and not stalled and total_heading_change > math.pi * 0.75 and z <= 0:
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
        mu_brake = 0.3 * runway_friction
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
    elif speed_mode in ('vs_plus_10', 'vs_x_1p3'):
        # Surface the operational targets so the data card can show them
        ops = operational_turn_speeds(config, weight, bank_angle_deg,
                                       field_elevation, isa_dev,
                                       landing_flaps=False)
        speed_info['vs_at_bank_kias'] = round(ops['vs_kias'], 1)
        speed_info['vs_plus_10_kias'] = round(ops['vs_plus_10_kias'], 1)
        speed_info['vs_x_1p3_kias'] = round(ops['vs_x_1p3_kias'], 1)
        speed_info['nz_bank'] = round(ops['nz'], 3)
        speed_info['target_kias'] = round(
            ops['vs_plus_10_kias'] if speed_mode == 'vs_plus_10' else ops['vs_x_1p3_kias'],
            1,
        )

    # ── Turn-statistics for training output (Charlie #7) ──
    # Walk the trajectory finding altitude at each 180° increment of
    # cumulative turning, so the pilot can practice "lose this much
    # altitude per half-turn at altitude" and pick a safety factor.
    turn_stats = _compute_turn_statistics(trajectory, failure_alt_agl)

    # ── Per-phase summary (Charlie #F4) ──
    phase_summary = _compute_phase_summary(trajectory, failure_alt_agl)

    # ── Departure-end-threshold altitude (Charlie #F3) ──
    # When the pilot lands DOWNWIND on the original departure direction,
    # the trajectory crosses the FAR end of the runway (y = intersection_offset_ft + runway_length).
    # When landing reverse, the trajectory crosses the takeoff end (y = intersection_offset_ft).
    # Surface both so the data card can report whichever applies, and the
    # opposite end as supplemental information.
    _far_end_y = intersection_offset_ft + (runway_length if runway_length else 0.0)
    _near_end_y = intersection_offset_ft
    altitude_at_departure_threshold = (
        _altitude_at_y(trajectory, _far_end_y) if runway_length else None
    )
    altitude_at_takeoff_threshold = _altitude_at_y(trajectory, _near_end_y)

    # Statistics above used the full step history. The list handed to the
    # UI is thinned so Streamlit and the map traces stay small.
    trajectory = _decimate_trajectory(trajectory)

    return {
        'trajectory': trajectory,
        'success': success,
        'altitude_at_runway': altitude_at_runway,
        'altitude_at_departure_threshold': altitude_at_departure_threshold,
        'altitude_at_takeoff_threshold': altitude_at_takeoff_threshold,
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
        'total_turn_deg': turn_stats['total_turn_deg'],
        'altitude_loss_per_180': turn_stats['altitude_loss_per_180'],
        'altitude_at_180_increments': turn_stats['altitude_at_180_increments'],
        'phase_summary': phase_summary,
    }


# ---------------------------------------------------------------------------
#  Straight-Ahead Landing Analysis
# ---------------------------------------------------------------------------

def simulate_straight_ahead(
    config, weight, failure_alt_agl, airspeed_kias,
    reaction_time, field_elevation=0.0, isa_dev=0.0,
    wind_speed_kt=0.0, wind_from_deg=0.0,
    wind_1000_kt=0.0, wind_2000_kt=0.0, wind_3000_kt=0.0,
    wind_dir_profile=None,
    wind_profile=None,
    runway_length=0.0, liftoff_distance=0.0,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    runway_friction=1.0,
    climb_steering='track',
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

    # Same full-flap polar as the turnback return (see _landing_polar).
    dcdo_landing, clmax_landing = _landing_polar(config)

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

    climb_dist, climb_time_s = _estimate_climb_distance(
        config, weight, field_elevation, isa_dev, failure_alt_agl, airspeed_for_climb,
        wind_x_fps=wind_x, wind_y_fps=wind_y, climb_steering=climb_steering,
    )
    dist_from_threshold = liftoff_distance + climb_dist

    # E2 climb-steering drift (see simulate_turnback for derivation).
    if climb_steering == 'heading':
        climb_drift_x = wind_x * climb_time_s
        climb_drift_y = wind_y * climb_time_s
    else:
        climb_drift_x = 0.0
        climb_drift_y = 0.0

    # Initial state
    x = climb_drift_x
    y = intersection_offset_ft + dist_from_threshold + climb_drift_y  # E1: intersection departure
    z = failure_alt_agl
    # Heading at engine failure depends on climb-steering mode (May 2026 fix):
    # track-hold pilot has been crabbing into the wind, so the nose is offset
    # from runway heading by the crab angle.  See simulate_turnback for details.
    if climb_steering == 'track' and abs(wind_x) > 0.5:
        try:
            _, _, _sigma_climb, _delta_climb, _, _ = atmos(field_elevation, isa_dev)
            _vclimb_fps, _, _ = _kias_to_fps(airspeed_for_climb, _sigma_climb, _delta_climb)
            if _vclimb_fps > 1.0:
                _sin_crab = max(-0.99, min(0.99, -wind_x / _vclimb_fps))
                heading = math.asin(_sin_crab)
            else:
                heading = 0.0
        except Exception:
            heading = 0.0
    else:
        heading = 0.0  # straight ahead
    t = 0.0
    phase = 'reaction'
    stalled = False
    stall_time = None
    gear_retracted = False  # E2: tracks post-failure gear-up event

    trajectory = []
    flaps_deployed = False

    while z > 0 and t < 600.0:
        alt_msl = field_elevation + max(z, 0.0)
        _, _, sigma_now, delta_now, _, _ = atmos(alt_msl, isa_dev)

        # Wind speed varies with altitude the same way the turnback does.
        # Previously only the direction profile was applied, and the speed
        # stayed at the surface value for the whole glide.
        if wind_profile:
            wind_speed_at_alt = _interpolate_wind_profile(z, wind_speed_kt, wind_profile)
        else:
            wind_speed_at_alt = _interpolate_wind_at_altitude(
                z, wind_speed_kt, wind_1000_kt, wind_2000_kt, wind_3000_kt
            )
        wind_fps = wind_speed_at_alt * _KT_TO_FPS
        if wind_dir_profile:
            _dir_deg_at_alt = _interpolate_wind_dir_profile(
                z, wind_from_deg, wind_dir_profile
            )
            _wind_from_rad_now = math.radians(_dir_deg_at_alt)
        else:
            _wind_from_rad_now = wind_from_rad
        wind_x = -wind_fps * math.sin(_wind_from_rad_now)
        wind_y = -wind_fps * math.cos(_wind_from_rad_now)
        # E2: gear retraction event during the post-failure trajectory.
        if (gear_retract_time_s is not None and gear_down
                and t >= gear_retract_time_s and config.dcdo_gear > 0
                and not gear_retracted):
            cdo_clean -= config.dcdo_gear
            cdo -= config.dcdo_gear
            gear_retracted = True

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
    mu_brake = 0.3 * runway_friction
    _, _, sigma_sfc, _, _, _ = atmos(field_elevation, isa_dev)
    v_td_fps, _, _ = _kias_to_fps(touchdown_speed_kias, sigma_sfc, 1.0)
    landing_rollout = v_td_fps ** 2 / (2.0 * G_FPS2 * mu_brake)

    # Can it land and stop on remaining runway?
    remaining_runway_ahead = runway_length - touchdown_y
    runway_overrun = landing_rollout > remaining_runway_ahead or remaining_runway_ahead < 0
    success = (not stalled) and (not runway_overrun) and remaining_runway_ahead >= 0

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

    trajectory = _decimate_trajectory(trajectory)

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
    wind_1000_kt=0.0, wind_2000_kt=0.0, wind_3000_kt=0.0,
    wind_profile=None,
    wind_dir_profile=None,
    runway_length=0.0, liftoff_distance=0.0,
    speed_mode='fixed',
    alt_low=10.0, alt_high=3000.0, tolerance=5.0,
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    runway_friction=1.0,
    climb_steering='track',
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
        wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
        wind_dir_profile=wind_dir_profile,
        wind_profile=wind_profile,
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        gear_retract_time_s=gear_retract_time_s,
        intersection_offset_ft=intersection_offset_ft,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        runway_friction=runway_friction,
        climb_steering=climb_steering,
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
    wind_1000_kt=0.0, wind_2000_kt=0.0, wind_3000_kt=0.0,
    wind_profile=None,
    wind_dir_profile=None,
    turn_direction='left',
    runway_length=0.0, liftoff_distance=0.0, aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
    runway_friction=1.0,
    climb_steering='track',
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
        gear_retract_time_s=gear_retract_time_s,
        intersection_offset_ft=intersection_offset_ft,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        touchdown_margin_ft=touchdown_margin_ft,
        wind_1000_kt=wind_1000_kt,
        wind_2000_kt=wind_2000_kt,
        wind_3000_kt=wind_3000_kt,
        wind_profile=wind_profile,
        wind_dir_profile=wind_dir_profile,
        runway_friction=runway_friction,
        climb_steering=climb_steering,
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
    wind_1000_kt=0.0, wind_2000_kt=0.0, wind_3000_kt=0.0,
    wind_profile=None,
    wind_dir_profile=None,
    runway_length=0.0, liftoff_distance=0.0, aim_point=0.0,
    flap_on_return=False,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
    runway_friction=1.0,
    climb_steering='track',
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
    t0 = time.perf_counter()
    log.info(
        "envelope start weight=%.0f lb bank=%.0f° reaction=%.1fs "
        "wind=%.0fkt@%.0f° alt_step=%s speed_mode=%s",
        weight, bank_angle_deg, reaction_time, wind_speed_kt, wind_from_deg,
        alt_step, speed_mode,
    )

    common = dict(
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        aim_point=aim_point, flap_on_return=flap_on_return,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        gear_retract_time_s=gear_retract_time_s,
        intersection_offset_ft=intersection_offset_ft,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        touchdown_margin_ft=touchdown_margin_ft,
        wind_1000_kt=wind_1000_kt,
        wind_2000_kt=wind_2000_kt,
        wind_3000_kt=wind_3000_kt,
        wind_profile=wind_profile,
        wind_dir_profile=wind_dir_profile,
        runway_friction=runway_friction,
        climb_steering=climb_steering,
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
            wind_dir_profile=wind_dir_profile,
            runway_length=runway_length, liftoff_distance=liftoff_distance,
            speed_mode=speed_mode,
            prop_state=prop_state,
            gear_down=gear_down,
            gear_retract_time_s=gear_retract_time_s,
            intersection_offset_ft=intersection_offset_ft,
            vbg_clean_kias=vbg_clean_kias,
            vbg_geardown_kias=vbg_geardown_kias,
            vbg_landing_kias=vbg_landing_kias,
            climb_steering=climb_steering,
        )

    if max_alt is None:
        # Auto-ceiling: extend high enough that the Decision Ladder can
        # discover 540° (orbit) and full-circuit options, not just 180°.
        # An orbit needs ~3× the altitude of a 180°; a full circuit
        # needs ~5-6×.  Floor at 3,000 ft AGL for trainer-class profiles
        # so the full-circuit card has a chance to populate.
        max_alt = int(max(critical_alt * 6.0 + 200, 3000))
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
        wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
        wind_profile=wind_profile,
        wind_dir_profile=wind_dir_profile,
        runway_length=runway_length, liftoff_distance=liftoff_distance,
        speed_mode=speed_mode,
        prop_state=prop_state,
        gear_down=gear_down,
        gear_retract_time_s=gear_retract_time_s,
        intersection_offset_ft=intersection_offset_ft,
        vbg_clean_kias=vbg_clean_kias,
        vbg_geardown_kias=vbg_geardown_kias,
        vbg_landing_kias=vbg_landing_kias,
        climb_steering=climb_steering,
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

    log.info(
        "envelope done in %.2fs critical L/R=%.0f/%.0f ft rows=%d straight_ahead_max=%.0f",
        time.perf_counter() - t0, critical_alt_left, critical_alt_right,
        len(envelope), straight_ahead_max_alt,
    )
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
    wind_1000_kt=0.0, wind_2000_kt=0.0, wind_3000_kt=0.0,
    wind_profile=None,
    wind_dir_profile=None,
    runway_length=0.0, liftoff_distance=0.0,
    bank_range=(10, 60), bank_step=2,
    alt_high=3000.0,
    speed_mode='fixed',
    prop_state='feathered',
    gear_down=True,
    gear_retract_time_s=None,
    intersection_offset_ft=0.0,
    vbg_clean_kias=0,
    vbg_geardown_kias=0,
    vbg_landing_kias=0,
    touchdown_margin_ft=0.0,
    runway_friction=1.0,
    climb_steering='track',
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
                            wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                            wind_profile=wind_profile,
                            wind_dir_profile=wind_dir_profile,
                            turn_direction=direction,
                            runway_length=runway_length, liftoff_distance=liftoff_distance,
                            aim_point=aim_pt, flap_on_return=flap_on_return,
                            speed_mode=smode,
                            prop_state=prop_state,
                            gear_down=gear_down,
                            gear_retract_time_s=gear_retract_time_s,
                            intersection_offset_ft=intersection_offset_ft,
                            vbg_clean_kias=vbg_clean_kias,
                            vbg_geardown_kias=vbg_geardown_kias,
                            vbg_landing_kias=vbg_landing_kias,
                            touchdown_margin_ft=touchdown_margin_ft,
                            runway_friction=runway_friction,
                            climb_steering=climb_steering,
                        )

                        # Check overrun at critical altitude
                        overrun = False
                        if use_runway and crit < alt_high:
                            test = simulate_turnback(
                                config, weight, crit, airspeed_kias, bank_deg,
                                flap_setting, reaction_time, field_elevation, isa_dev,
                                direction, wind_speed_kt, wind_from_deg,
                                wind_1000_kt, wind_2000_kt, wind_3000_kt,
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
                                runway_friction=runway_friction,
                                climb_steering=climb_steering,
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
