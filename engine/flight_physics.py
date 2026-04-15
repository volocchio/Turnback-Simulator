"""
Flight Physics Module

This module contains functions for calculating atmospheric conditions, thrust, drag,
V-speeds, and other physics-related parameters for flight simulation.
"""

import numpy as np
from math import sqrt, exp, sin

try:
    from .thrust_deck import get_thrust_for_aircraft, ThrustDeck
    THRUST_DECK_AVAILABLE = True
except ImportError:
    THRUST_DECK_AVAILABLE = False

# Flat rate model: FJ44 thermo/flat ratio (back-solved from AFM WAT data)
# Below flat rate temp (ISA+16.7°C), thrust = f(pressure_alt)
# Above flat rate temp, thrust reduced by theta ratio raised to exponent n
FLAT_RATE_RATIO = 1.017

# Theta exponent for thrust lapse above flat rate temp
# T = T_ISA * (theta_ISA / theta_actual)^n
# Back-solved from full 0-10k F15+F0 AFM WAT data (RMSE 3.4°C)
THETA_EXPONENT = 2.70

THETA_EXPONENT_SLOPE = -1.00
DA_BLEND = 0.95

# OEI asymmetric flight drag penalty (rudder deflection, sideslip)
# Back-solved from full 0-10k F15+F0 AFM data with ratio=1.058, n=2.85
DCDO_OEI_ASYMMETRIC = 0.002

# Takeoff thrust boost: TO-rated thrust is higher than MCT (Max Continuous Thrust).
# The thrust deck represents MCT.  Takeoff rating adds ~7% for 5 min AEO / 10 min OEI.
# Applied in segments 0-2 (ground roll, rotation, initial climb to 400 ft).
TAKEOFF_THRUST_BOOST = 1.07

# OEI thrust derating: remaining engine produces less effective thrust due to
# increased bleed air load, accessory extraction, and installation effects.
# Linear model back-solved from CJ Tamarack Flap 0 AFM WAT break temps (0-7k ft).
# factor = OEI_THRUST_FACTOR_SL + OEI_THRUST_FACTOR_SLOPE * pressure_alt
# Fits all 8 AFM break points within ±0.3%.
OEI_THRUST_FACTOR_SL = 0.9228
OEI_THRUST_FACTOR_SLOPE = -0.00001045  # per foot of pressure altitude

def crossover_altitude(kias, mach, isa_diff=0.0):
    """Compute the pressure altitude where a given KIAS and Mach coincide.

    Pure atmospheric calculation, independent of aircraft.
    The crossover depends only on pressure ratio (delta), so it is
    independent of temperature / ISA deviation.  (isa_diff kept in the
    signature for API compatibility but has no effect.)

    Returns pressure altitude in feet, or None if no crossover exists.
    """
    if kias is None or mach is None or kias <= 0 or mach <= 0:
        return None

    # Sea-level speed of sound (knots) — used only via a0*sqrt(delta)
    _a0 = (1.4 * 287.0 * 288.15) ** 0.5 * 1.94384  # ≈ 661.5 kt

    def _kias_at_alt(alt_ft):
        # Pressure ratio from pressure altitude (temperature-independent)
        if alt_ft <= 36089.0:
            delta = (1.0 - 6.8756e-6 * alt_ft) ** 5.2559
        else:
            delta = 0.22336 * exp(-4.80637e-5 * (alt_ft - 36089.0))
        # KEAS = M × a₀ × √δ  (temperature cancels out of EAS)
        vkeas = mach * _a0 * sqrt(delta)
        # Compressibility correction  EAS → IAS
        corr = 1 + 0.125 * (1 - delta) * mach**2 \
             + (3.0/640.0) * (1 - 10*delta + 9*delta**2) * mach**4
        return vkeas * corr

    lo, hi = 1000.0, 45000.0
    kias_lo = _kias_at_alt(lo)
    kias_hi = _kias_at_alt(hi)

    # At low altitude KIAS-from-Mach should be > kias; at high altitude < kias.
    # If sign doesn't flip, there's no crossover in range.
    if kias_lo < kias and kias_hi < kias:
        return None
    if kias_lo > kias and kias_hi > kias:
        return None

    for _ in range(50):            # bisection — converges to < 1 ft
        mid = 0.5 * (lo + hi)
        kias_mid = _kias_at_alt(mid)
        if kias_mid > kias:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1.0:
            break
    return round(0.5 * (lo + hi))


def haversine_with_bearing(lat1, lon1, lat2, lon2):
    R = 3440.065  # Nautical miles
    lat1_rad = np.radians(lat1)
    lon1_rad = np.radians(lon1)
    lat2_rad = np.radians(lat2)
    lon2_rad = np.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = np.sin(dlat / 2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon / 2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    distance = R * c

    y = np.sin(dlon) * np.cos(lat2_rad)
    x = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon)
    bearing = (np.degrees(np.arctan2(y, x)) + 360) % 360

    return distance, bearing


def atmos(alt, isa_diff):
    """Standard atmosphere with ISA deviation.

    Args:
        alt: Pressure altitude (ft)
        isa_diff: ISA temperature deviation (deg C)

    Returns:
        d_alt: Density altitude (ft) via Newton-Raphson
        theta: Actual temperature ratio (OAT / 288.15 K)
        sigma: Density ratio (rho / rho_SL)
        delta: Pressure ratio (P / P_SL) from pressure altitude
        k_temp: Actual static temperature (K)
        c: Speed of sound (knots) from actual temperature
    """
    # Actual temperature
    if alt <= 36089:
        isa_temp_k = 288.15 * (1.0 - 6.8756e-6 * alt)
    else:
        isa_temp_k = 216.65
    oat_k = isa_temp_k + isa_diff
    if oat_k < 1.0:
        oat_k = 1.0
    theta = oat_k / 288.15

    # Pressure ratio from pressure altitude only
    if alt <= 36089:
        delta = (1.0 - 6.8756e-6 * alt) ** 5.2559
    else:
        delta = 0.22336 * exp(-4.80637e-5 * (alt - 36089.0))

    # Density ratio (exact ideal-gas relation)
    sigma = delta / theta

    # Density altitude via Newton-Raphson: find h where ISA sigma(h) = sigma
    h = float(alt)
    for _ in range(20):
        if h <= 36089:
            base = 1.0 - 6.8756e-6 * h
            if base < 1e-12:
                base = 1e-12
            sig_isa = base ** 4.2559
            dsig_dh = 4.2559 * base ** 3.2559 * (-6.8756e-6)
        else:
            sig_isa = 0.2971 * exp(-4.80637e-5 * (h - 36089.0))
            dsig_dh = sig_isa * (-4.80637e-5)
        if abs(dsig_dh) < 1e-20:
            break
        h_new = h - (sig_isa - sigma) / dsig_dh
        if abs(h_new - h) < 0.1:
            h = h_new
            break
        h = h_new
    d_alt = h

    # Temperature and speed of sound from actual OAT
    k_temp = oat_k
    c = (1.4 * 287.0 * k_temp) ** 0.5 * 1.94384
    return d_alt, theta, sigma, delta, k_temp, c


def _isa_temp_c(alt_ft):
    """ISA temperature (°C) at pressure altitude."""
    if alt_ft <= 36089:
        return 15.0 - 0.0019812 * alt_ft
    return -56.5


def thrust_calc(d_alt, m, thrust_mult, engines, thrust_factor, segment, use_thrust_deck=True, aircraft_model=None, pressure_alt=None, isa_dev=None):
    """
    Calculate thrust using either table-based thrust deck or legacy polynomial model.
    Applies flat rate theta correction when pressure_alt and isa_dev are provided.
    
    Args:
        d_alt: Density altitude in feet
        m: Mach number
        thrust_mult: Thrust multiplier for engine variant
        engines: Number of engines
        thrust_factor: Throttle setting (0.0 to 1.0)
        segment: Flight segment number
        use_thrust_deck: If True and available, use thrust deck; otherwise use polynomial
        aircraft_model: Aircraft model string for thrust deck lookup
        pressure_alt: Pressure altitude in feet (for flat rate model)
        isa_dev: ISA deviation in °C (for flat rate model)
    
    Returns:
        Thrust in lbf
    """
    # Determine effective altitude for thrust calculation
    # Flat rate model: below flat rate temp use pressure alt, above use theta correction
    eff_alt = d_alt  # default: density altitude
    theta_correction = 1.0
    if pressure_alt is not None and isa_dev is not None:
        isa_t = _isa_temp_c(pressure_alt)
        flat_rate_C = (isa_t + 273.15) * FLAT_RATE_RATIO - 273.15
        oat_c = isa_t + isa_dev
        n_eff = THETA_EXPONENT + THETA_EXPONENT_SLOPE * (float(pressure_alt) / 10000.0)
        if n_eff < 0.1:
            n_eff = 0.1
        if n_eff > 6.0:
            n_eff = 6.0
        if oat_c <= flat_rate_C:
            # Below flat rate: full rated thrust at pressure altitude
            eff_alt = pressure_alt
        else:
            # Above flat rate: thrust at pressure alt, reduced by theta ratio^n
            try:
                eff_alt = float(pressure_alt) + DA_BLEND * (float(d_alt) - float(pressure_alt))
            except Exception:
                eff_alt = pressure_alt
            _ratio = (isa_t + 273.15) / max(1.0, oat_c + 273.15)
            _ratio = max(0.01, min(2.0, _ratio))
            theta_correction = _ratio ** n_eff
            if not np.isfinite(theta_correction):
                theta_correction = 1.0
    
    # Polynomial model
    thrust_reg = (2785.75 -
                  1950.17 * m -
                  0.05261 * eff_alt -
                  12.9726 * m ** 2 +
                  0.07669 * m * eff_alt -
                  0.0000001806 * eff_alt ** 2 +
                  1118.99 * m ** 3 -
                  0.03617 * m ** 2 * eff_alt -
                  0.0000003701 * m * eff_alt ** 2 +
                  0.000000000003957 * eff_alt ** 3)
    poly_thrust = thrust_reg * thrust_mult * engines * theta_correction * thrust_factor

    # Thrust deck (table interpolation)
    deck_thrust = None
    if use_thrust_deck and THRUST_DECK_AVAILABLE and aircraft_model is not None:
        try:
            base_thrust = get_thrust_for_aircraft(
                aircraft_model, 
                eff_alt, 
                m, 
                num_engines=engines,
                thrust_multiplier=thrust_mult
            )
            deck_thrust = base_thrust * theta_correction * thrust_factor
        except Exception:
            pass

    # Above FL350 the deck has sparse data (40k, 45k, 50k) and is slightly
    # pessimistic. Use the higher of deck and polynomial so the more accurate
    # polynomial governs at high altitude.
    if deck_thrust is not None and eff_alt > 35000:
        thrust = max(poly_thrust, deck_thrust)
    elif deck_thrust is not None:
        thrust = deck_thrust
    else:
        thrust = poly_thrust

    # Takeoff thrust boost: segments 0-2 use TO-rated thrust (above MCT).
    # The FJ44 thrust deck/polynomial represent MCT; takeoff rating is ~7% higher.
    # The PW2040 (C-17) deck is already scaled to published TO-rated thrust,
    # so the boost is only applied to non-native-deck aircraft.
    if segment in (0, 1, 2) and aircraft_model not in ('C-17', 'C-17 ER'):
        thrust *= TAKEOFF_THRUST_BOOST

    if thrust < 100:
        thrust = 100
    return thrust


def drag_calc(w, cdo, dcdo_flap1, dcdo_flap2, dcdo_flap3, dcdo_gear, m, k, cl, q, s, segment, flap, skip_cdnp=False):
    try:
        _m = float(m)
    except Exception:
        _m = 0.0
    if not np.isfinite(_m):
        _m = 0.0
    if _m < 0.0:
        _m = 0.0
    if _m > 1.5:
        _m = 1.5

    try:
        _cl = float(cl)
    except Exception:
        _cl = 0.0
    if not np.isfinite(_cl):
        _cl = 0.0
    if _cl > 50.0:
        _cl = 50.0
    if _cl < -50.0:
        _cl = -50.0

    cdnp = 0.0
    if _m > 0.5 and not skip_cdnp:
        try:
            _exp_arg = 6.0 * (_cl * _cl)
            if not np.isfinite(_exp_arg):
                _exp_arg = 50.0
            if _exp_arg > 50.0:
                _exp_arg = 50.0
            if _exp_arg < -50.0:
                _exp_arg = -50.0
            _exp_val = exp(_exp_arg)
            cdnp = (6.667 * _m ** 4 - 15.733 * _m ** 3 + 13.923 * _m ** 2 - 5.464 * _m + 0.8012) * (_exp_val / 4.0)
        except Exception:
            cdnp = 0.0
    if segment in [0, 13]:
        cdi = 0
    else:
        try:
            _k = float(k)
        except Exception:
            _k = 0.0
        if not np.isfinite(_k):
            _k = 0.0
        cdi = _k * (_cl * _cl)
    if segment == 0:
        cd = cdo + cdi
    elif segment == 1:
        cd = cdo + cdi + dcdo_gear + dcdo_flap1 * flap
    elif segment == 2:
        cd = cdo + cdi + dcdo_flap1 * flap
    elif segment == 3:
        cd = cdo + cdi
    elif segment == 11:
        cd = cdo + cdi + dcdo_flap1
    elif segment == 12:
        cd = cdo + cdi + dcdo_flap2 + dcdo_gear
    elif segment == 13:
        cd = cdo + cdi + dcdo_flap3 + dcdo_gear
    else:
        cd = cdo + cdi + cdnp
    drag = q * s * cd
    return drag, cd


def vspeeds(w, s, clmax, clmax_1, clmax_2, delta, m, flap, segment):
    try:
        # Calculate clmax_factor
        clmax_factor = 7.432 * m ** 6 - 12.59 * m ** 5 + 5.0847 * m ** 4 + 0.7356 * m ** 3 - 0.9942 * m ** 2 + 0.1147 * m + 0.9994
        
        # Calculate clmax_adjusted based on flap setting
        if flap == 0:  # Clean
            clmax_adjusted = clmax * clmax_factor
        elif flap == 1:  # Approach
            clmax_adjusted = clmax_1 * clmax_factor
        elif flap == 2:  # Landing
            clmax_adjusted = clmax_2 * clmax_factor
        else:
            clmax_adjusted = clmax * clmax_factor

        # Calculate stall speed in EAS (no altitude correction - EAS is constant with altitude)
        vs = sqrt(295 * (w / (s * clmax_adjusted)))
        
        # Calculate VR, V1, V2, V3 as multiples of stall speed
        vr = 1.05 * vs   # Rotation speed
        v1 = 1.0 * vr    # Decision speed (at or below VR)
        v2 = 1.2 * vs    # Takeoff safety speed
        v3 = 1.3 * vs    # Single-engine best ROC speed
        
        # Calculate VAPP and VREF
        if segment in (11, 12):  # Approach and landing segments
            correction = 1 + (0.01 * (w / 1000 - 100) / 10)  # Small correction factor
            approach_denom = s * clmax_1 * clmax_factor
            landing_denom = s * clmax_2 * clmax_factor
            
            vapp = 20 + (1.3 * sqrt(295 * (w / approach_denom))) * correction
            vref = (1.3 * sqrt(295 * (w / landing_denom))) * correction
        else:
            vapp = None
            vref = None
            
        return vr, v1, v2, v3, vapp, vref
        
    except Exception as e:
        # Return None values to indicate calculation failure
        return None, None, None, None, None, None


# Aircraft whose Cdo was calibrated from OEM PPH/performance data — cdnp
# compressibility drag is already baked into the drag polar.
_SKIP_CDNP_MODELS = frozenset((
    'C-17', 'C-17 ER',
    'CJ', 'CJ1', 'CJ1+', 'M2', 'CJ2', 'CJ2+', 'CJ3', 'CJ3+', 'CJ4',
    'Mustang',
))


def physics(t_inc, gamma, sigma, delta, w, m, c, vkias, roc_fpm, roc_goal, speed_goal, thrust_factor, engines, d_alt,
            thrust_mult, cdo, dcdo_flap1, dcdo_flap2, dcdo_flap3, dcdo_gear, k, s, segment, mu_lnd, mu_to, climb_trigger, p, mmo, v_true_fps, turboprop=None, aircraft_model=None, engines_orig=2, drift_down=False, pressure_alt=None, isa_dev=None, takeoff_flap=1):
    # Skip Citation-jet cdnp compressibility model for aircraft whose drag polar
    # was calibrated from OEM performance data (Cdo already includes wave drag).
    _skip_cdnp = aircraft_model in _SKIP_CDNP_MODELS
    drag_factor = 1
    # Add windmilling drag if operating on fewer engines than installed
    # Typical windmilling drag coefficient increment: 0.0020 per failed engine
    dcdo_windmill = 0.0020 * (engines_orig - engines) if engines < engines_orig else 0.0
    # Add OEI asymmetric flight drag (rudder deflection + sideslip)
    dcdo_oei = DCDO_OEI_ASYMMETRIC * (engines_orig - engines) if engines < engines_orig else 0.0
    # Turboprop thrust path: compute thrust from shaft power and prop efficiency if params provided
    if turboprop is not None:
        # Constants and parameters
        rho0 = 0.0023769  # slug/ft^3
        rho = sigma * rho0
        D = turboprop.get('prop_diameter_ft', 10.8)
        rpm = turboprop.get('prop_rpm', 1900.0)
        n = rpm / 60.0  # rev/s
        P_rated = turboprop.get('P_rated_shp', 675.0) * engines  # shp
        alpha = turboprop.get('alpha_lapse', 0.6)
        C_T0 = turboprop.get('C_T0', 0.10)
        J_curve = turboprop.get('eta_curve_J', [0.0, 0.4, 0.8, 1.0, 1.2, 1.4])
        eta_curve = turboprop.get('eta_curve_eta', [0.00, 0.70, 0.83, 0.86, 0.82, 0.70])

        # Interpolate efficiency eta(J)
        def lerp(x0, y0, x1, y1, x):
            if x1 == x0:
                return y0
            t = max(0.0, min(1.0, (x - x0) / (x1 - x0)))
            return y0 + t * (y1 - y0)

        def eta_of_J(J):
            if J <= J_curve[0]:
                return eta_curve[0]
            for i in range(1, len(J_curve)):
                if J <= J_curve[i]:
                    return lerp(J_curve[i-1], eta_curve[i-1], J_curve[i], eta_curve[i], J)
            return eta_curve[-1]

        V = max(0.0, v_true_fps)
        J = V / (n * D) if n * D > 1e-6 else 0.0
        eta = eta_of_J(J)

        # Available power with simple density lapse and throttle mapping
        P_avail_shp = P_rated * (sigma ** alpha) * max(0.0, min(1.0, thrust_factor))

        # Thrust from power, blend with static thrust at very low speeds
        T_power = (eta * P_avail_shp * 550.0) / max(V, 1e-3)  # 1 shp = 550 ft*lbf/s
        T_static = C_T0 * rho * (n ** 2) * (D ** 4) * engines
        # Blend region between 10 and 80 ft/s
        V_lo, V_hi = 10.0, 80.0
        if V <= V_lo:
            thrust = T_static
        elif V >= V_hi:
            thrust = T_power
        else:
            w_blend = (V - V_lo) / (V_hi - V_lo)
            thrust = (1 - w_blend) * T_static + w_blend * T_power
        if thrust < 100:
            thrust = 100
    else:
        thrust = thrust_calc(d_alt, m, thrust_mult, engines, thrust_factor, segment, use_thrust_deck=True, aircraft_model=aircraft_model, pressure_alt=pressure_alt, isa_dev=isa_dev)

    # Apply OEI thrust derating when operating on fewer engines than installed.
    # The derating model was calibrated from Citation CJ (2-engine) WAT data and
    # accounts for increased bleed/accessory load on the remaining engine.
    # For 4-engine aircraft losing 1 engine the penalty per surviving engine is
    # much smaller, so we skip the derating (3 engines still share the load).
    if engines < engines_orig and engines_orig <= 2:
        _pa = float(pressure_alt) if pressure_alt is not None else max(0.0, float(d_alt))
        oei_factor = OEI_THRUST_FACTOR_SL + OEI_THRUST_FACTOR_SLOPE * _pa
        oei_factor = max(0.3, min(1.0, oei_factor))
        thrust *= oei_factor

    if p == 0:
        m = 0

    vkeas = vkias / (1 + 1/8 * (1 - delta) * m ** 2 + 3/640 * (1 - 10 * delta + 9 * delta ** 2) * m ** 4)
    q = vkeas ** 2 / 295
    if segment == 0:
        cl = 0
    else:
        cl = 0 if q == 0 else w / (q * s)
        if cl > 2.0:
            cl = 2.0
    drag, cd = drag_calc(w, cdo, dcdo_flap1, dcdo_flap2, dcdo_flap3, dcdo_gear, m, k, cl, q, s, segment, takeoff_flap, skip_cdnp=_skip_cdnp)
    # Add windmilling drag
    drag += (dcdo_windmill + dcdo_oei) * q * s
    drag *= drag_factor
    if segment == 0:
        drag_gnd = mu_to * w
    elif segment == 13:
        drag_gnd = mu_lnd * w
    else:
        drag_gnd = 0
    
    # Drift down: Cap thrust at drag ONLY during OEI level cruise (segments 6/7)
    # to maintain equilibrium at the drift-down ceiling. Do NOT cap during descent
    # (segment 8+) — the aircraft needs (T-D) < 0 to actually descend.
    if drift_down and segment in (6, 7) and engines == 1 and thrust >= drag:
        thrust = drag
    
    acc_x = 32.2 * (thrust - (drag + drag_gnd)) / w
    # Drift-down in segment 8: energy deficit goes into descent (gamma),
    # not horizontal deceleration. Zero acc_x so speed converges to target
    # via the speed-target logic below, not from the T-D imbalance.
    if drift_down and segment == 8:
        acc_x = 0.0

    if speed_goal < 1 and segment != 13:
        target_v_fps = speed_goal * c * (6076.12 / 3600)
    else:
        correction = 1 + 1/8 * (1 - delta) * m ** 2 + 3/640 * (1 - 10 * delta + 9 * delta ** 2) * m ** 4
        vkeas = speed_goal / correction
        vktas = vkeas / sqrt(sigma)
        target_v_fps = vktas * (6076.12 / 3600)

    if v_true_fps < target_v_fps:
        if v_true_fps + acc_x * t_inc > target_v_fps:
            acc_x = (target_v_fps - v_true_fps) / t_inc
        elif segment in (8, 9, 10, 11, 12) and acc_x < 0:
            # During descent, potential energy converts to kinetic energy.
            # Don't decelerate when already below target speed — the descent
            # path provides the energy to maintain speed.
            acc_x = 0
    elif segment == 5 or (v_true_fps > target_v_fps and segment != 13):
        if v_true_fps + acc_x * t_inc > target_v_fps:
            acc_x = (target_v_fps - v_true_fps) / t_inc
            if acc_x < -3 and segment != 13:
                acc_x = -3
                climb_trigger = 1
            else:
                climb_trigger = 0
    elif segment in [8, 9, 10, 11, 12, 13]:
        if v_true_fps + acc_x * t_inc > target_v_fps:
            acc_x = (target_v_fps - v_true_fps) / t_inc
            if acc_x < -3 and segment != 13:
                acc_x = -3
            if segment == 13:
                acc_x = 32.2 * (thrust - (drag + drag_gnd)) / w
                v_true_fps = v_true_fps + acc_x * t_inc
    else:
        acc_x = 0
        thrust = drag

    v_true_fps += acc_x * t_inc
    if v_true_fps < 0:
        v_true_fps = 0
    vktas = v_true_fps / (6076.12 / 3600)
    m = vktas / c
    if m > mmo:
        m = mmo
    vkeas = vktas * sqrt(sigma)
    correction = 1 + 1/8 * (1 - delta) * m ** 2 + 3/640 * (1 - 10 * delta + 9 * delta ** 2) * m ** 4
    if correction > 1000:
        correction = 1000
    vkias = vkeas * correction
    return cl, q, drag, cd, vkeas, vktas, v_true_fps, thrust, drag_gnd, vkias, m


def predict_roc(next_step_alt, alt, w, m, thrust, drag, vktas, thrust_mult, engines, thrust_factor, cdo, dcdo_flap1, dcdo_flap2, dcdo_flap3, dcdo_gear, k, s, isa_diff, speed_goal, segment, alt_goal, aircraft_model=None, pressure_alt=None, isa_dev=None, thrust_bias_frac=0.0):
    _skip_cdnp = aircraft_model in _SKIP_CDNP_MODELS
    new_d_alt, new_theta, new_sigma, new_delta, new_k_temp, new_c = atmos(next_step_alt, isa_diff)
    if speed_goal < 1:
        new_m = speed_goal
        new_vktas = new_m * new_c
        new_vkeas = new_vktas * sqrt(new_sigma)
    else:
        correction = 1 + 1/8 * (1 - new_delta) * m ** 2 + 3/640 * (1 - 10 * new_delta + 9 * new_delta ** 2) * m ** 4
        new_vkeas = speed_goal / correction
        new_vktas = new_vkeas / sqrt(new_sigma)
        new_m = new_vktas / new_c
    v_true_fps_new = new_vktas * 6076.12 / 3600
    # Use pressure_alt and isa_dev for flat-rate theta correction (consistent with main sim loop)
    new_thrust = thrust_calc(new_d_alt, new_m, thrust_mult, engines, thrust_factor, segment,
                             use_thrust_deck=True, aircraft_model=aircraft_model,
                             pressure_alt=next_step_alt if pressure_alt is not None else None,
                             isa_dev=isa_dev)
    # Apply thrust bias (same as main sim loop)
    new_thrust = new_thrust * (1.0 + thrust_bias_frac)
    new_q = new_vkeas ** 2 / 295
    if new_q <= 0:
        new_cl = 0
        new_drag = 0
    else:
        new_cl = w / (new_q * s)
        new_drag, new_cd = drag_calc(w, cdo, dcdo_flap1, dcdo_flap2, dcdo_flap3, dcdo_gear, new_m, k, new_cl, new_q, s, segment, 1, skip_cdnp=_skip_cdnp)
    new_tx = (new_thrust - new_drag) / w
    new_gamma = np.arcsin(new_tx) if -1 <= new_tx <= 1 else (np.pi/2 if new_tx > 1 else -np.pi/2)
    new_roc_fps = new_gamma * v_true_fps_new / (6076.12 / 3600)
    new_roc_fpm = new_roc_fps * 60
    if new_roc_fpm < 0:
        new_roc_fpm = 0
    return new_roc_fpm


def next_step_altitude(current_alt, final_cruise_alt, previous_step_alt):
    rounded_current_alt = round(current_alt / 1000) * 1000
    rounded_final_cruise_alt = round(final_cruise_alt / 1000) * 1000
    if current_alt >= rounded_final_cruise_alt:
        return rounded_final_cruise_alt
    if previous_step_alt > 0 and current_alt < previous_step_alt:
        return previous_step_alt
    final_is_even = (rounded_final_cruise_alt // 1000) % 2 == 0
    if final_is_even:
        if (rounded_current_alt // 1000) % 2 != 0:
            next_alt = rounded_current_alt + 1000
        else:
            next_alt = rounded_current_alt + 2000
    else:
        if (rounded_current_alt // 1000) % 2 == 0:
            next_alt = rounded_current_alt + 1000
        else:
            next_alt = rounded_current_alt + 2000
    if next_alt > rounded_final_cruise_alt:
        next_alt = rounded_final_cruise_alt
    return round(next_alt / 1000) * 1000