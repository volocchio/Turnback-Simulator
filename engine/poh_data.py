"""POH-Derived Takeoff Ground-Roll Estimator (Charlie #5b).

This module provides a reference table of sea-level / ISA / MTOW takeoff
ground-roll distances for the single-engine aircraft in the simulator,
plus a scaling routine that adjusts those reference numbers for the
selected weight and density altitude.

Numbers are drawn from published manufacturer POHs / pilot-operating
handbooks and well-known third-party performance summaries.  Where the
POH publishes a 50-ft obstacle distance, the **ground-roll-only** column
was used because the turnback model reasons about wheels-off (liftoff)
distance.  Sources are cited inline.

Scaling (industry-standard light-aircraft approximation):

    SR(W, DA) = SR_ref * (W / MTOW)^2 * (rho_SL / rho_DA)^1.0

  - Weight exponent of 2 follows from V_lof² ∝ W and constant CLmax.
  - Density correction is linear (1/σ), a conservative first-order
    approximation that matches the ~10%-per-1000-ft DA rule of thumb
    for normally-aspirated light aircraft.  Turboprops with FADEC
    thrust normalization are slightly less density-sensitive but the
    1/σ assumption keeps us on the safe side.

NOT a substitute for the actual POH chart.  Always verify against the
specific aircraft's published performance data for go/no-go decisions.
"""

from __future__ import annotations
from dataclasses import dataclass

from engine.flight_physics import atmos


# ---------------------------------------------------------------------------
#  Reference POH ground-roll table (sea level, ISA, MTOW, no wind)
# ---------------------------------------------------------------------------
#
# key:  ground_roll_ft
#
# Sources (all values rounded to nearest 10 ft):
#   J3Cub        Piper J-3 Cub Owner's Handbook                        ~  370 ft
#   Husky        Aviat A-1B Husky published spec sheet                  ~  200 ft
#   RV-6A/7A/8   Van's Aircraft published kit performance data          ~  500 ft
#   RV-10        Van's Aircraft RV-10 kit performance summary           ~  500 ft
#   RV-12        Van's Aircraft RV-12iS published spec sheet            ~  500 ft
#   RV-14A       Van's Aircraft RV-14A published spec sheet             ~  500 ft
#   C172S        Cessna 172S Skyhawk POH (180 hp, MTOW 2,550 lb)        ~  960 ft
#   C150         Cessna 150 Owner's Manual (100 hp, MTOW 1,600 lb)      ~  735 ft
#   C152         Cessna 152 POH (110 hp, MTOW 1,670 lb)                 ~  725 ft
#   C182T        Cessna 182T Skylane POH (MTOW 3,100 lb)                ~  795 ft
#   C182RG       Cessna 182RG Skylane POH (MTOW 3,100 lb)               ~  820 ft
#   SR22         Cirrus SR22 POH (MTOW 3,400 lb, normally aspirated)    ~ 1080 ft
#   SR22T        Cirrus SR22T POH (MTOW 3,600 lb, turbocharged)         ~ 1082 ft
#   TTx          Cessna TTx (T240) POH (MTOW 3,600 lb)                  ~ 1370 ft
#   A36          Beechcraft Bonanza A36 POH (MTOW 3,650 lb)             ~  960 ft
#   PA28         Piper PA-28-181 Archer III POH (MTOW 2,550 lb)         ~  870 ft
#   M20V         Mooney M20V Acclaim Ultra POH (MTOW 3,374 lb)          ~ 1025 ft
#   Mirage       Piper PA-46-350P Mirage POH (MTOW 4,340 lb, piston)    ~ 1390 ft
#   TBM960/910/850 Daher TBM 850/910/960 POH (ground-roll only)         ~  995 ft
#   Meridian     Piper PA-46-500TP Meridian POH (MTOW 5,092 lb)         ~ 1366 ft
#   M600         Piper M600 SLS POH (MTOW 6,000 lb)                     ~ 1325 ft
#   M700         Piper M700 Fury POH (MTOW 6,000 lb)                    ~ 1325 ft
#   PC12         Pilatus PC-12 NG/NGX POH (MTOW 10,450 lb)              ~ 1830 ft
#   C208         Cessna 208 Caravan POH (MTOW 8,000 lb)                 ~ 1395 ft
#   C208B        Cessna 208B Grand Caravan POH (MTOW 8,750 lb)          ~ 1405 ft
#   C208EX       Cessna 208B Grand Caravan EX POH (MTOW 8,807 lb)       ~ 1366 ft
#   Kodiak100    Daher Kodiak 100 POH (MTOW 7,255 lb)                   ~  934 ft
#   Kodiak900    Daher Kodiak 900 POH (MTOW 8,000 lb)                   ~ 1078 ft
#   Denali       Beechcraft Denali POH (MTOW 8,500 lb, target spec)     ~ 1370 ft
#   E1000        Epic E1000 GX POH (MTOW 8,000 lb)                      ~ 1180 ft
#
# Twins (engines >= 2) are intentionally not included — Charlie's #5b
# feedback is single-engine focused, and OEI takeoff distance is its
# own conversation.

POH_GROUND_ROLL_FT = {
    'J3Cub':   370,
    'Husky':   200,
    'RV-6A':   500,
    'RV-7A':   500,
    'RV-8':    500,
    'RV-10':   500,
    'RV-12':   500,
    'RV-14A':  500,
    'C172S':   960,
    'C150':    735,
    'C152':    725,
    'PA-38':   820,   # Piper Tomahawk POH ground roll @ MTOW SL ISA
    'DA20':    700,   # Diamond DA20-C1 Eclipse POH ground roll @ MTOW SL ISA
    'C182T':   795,
    'C182RG':  820,
    'SR22':   1080,
    'SR22T':  1082,   # Cirrus SR22T POH (TN, MTOW 3,600 lb)
    'TTx':    1370,   # Cessna TTx / Corvalis T240 POH @ MTOW SL ISA
    'A36':     960,
    'PA28':    870,
    'M20V':   1025,   # Mooney M20V Acclaim Ultra POH @ MTOW SL ISA
    'Mirage': 1390,   # Piper PA-46-350P Mirage piston POH @ MTOW SL ISA
    'TBM960':  995,
    'TBM910':  995,
    'TBM850':  995,
    'Meridian': 1366,
    'M600':   1325,
    'M700':   1325,
    'PC12':   1830,   # Pilatus PC-12 NG/NGX POH @ MTOW SL ISA
    'C208':   1395,   # Cessna 208 Caravan POH @ MTOW SL ISA
    'C208B':  1405,   # Cessna 208B Grand Caravan POH @ MTOW SL ISA
    'C208EX': 1366,   # Cessna 208B Grand Caravan EX POH @ MTOW SL ISA
    'Kodiak100': 934, # Daher Kodiak 100 POH @ MTOW SL ISA
    'Kodiak900': 1078,# Daher Kodiak 900 POH @ MTOW SL ISA
    'Denali': 1370,   # Beechcraft Denali published target spec @ MTOW SL ISA
    'E1000':  1180,   # Epic E1000 GX POH @ MTOW SL ISA
}


@dataclass
class POHGroundRollEstimate:
    """Result of a POH-based ground-roll estimate."""
    ground_roll_ft: float
    base_sl_mtow_ft: float
    weight_factor: float           # (W / MTOW)^2
    density_factor: float          # rho_SL / rho_DA
    density_altitude_ft: float
    sigma: float                   # rho / rho_SL at field DA
    aircraft_key: str
    cited: bool                    # True if base value came from the table


def _density_altitude_ft(field_elev_ft: float, isa_dev_c: float) -> float:
    """Approximate density altitude using the standard 120 ft / °C ISA-dev rule.

    Good to within a few hundred feet for typical GA conditions.
    """
    return field_elev_ft + 120.0 * isa_dev_c


def estimate_ground_roll(
    aircraft_key: str,
    config,
    weight_lb: float,
    field_elev_ft: float = 0.0,
    isa_dev_c: float = 0.0,
) -> POHGroundRollEstimate | None:
    """Return a POH-scaled ground-roll estimate for the given aircraft.

    Returns None if `aircraft_key` is not in the table — the caller should
    then fall back to a manual user input.

    Args:
        aircraft_key: model key from AIRCRAFT_CONFIG (e.g. 'C172S', 'SR22')
        config:       AircraftConfig (used for MTOW)
        weight_lb:    actual takeoff weight
        field_elev_ft: airport elevation MSL
        isa_dev_c:    ISA deviation in °C

    Returns:
        POHGroundRollEstimate or None
    """
    base = POH_GROUND_ROLL_FT.get(aircraft_key)
    if base is None:
        return None

    mtow = float(config.MTOW)
    weight_factor = (float(weight_lb) / mtow) ** 2 if mtow > 0 else 1.0

    # Density at the actual field
    _, _, sigma, _, _, _ = atmos(float(field_elev_ft), float(isa_dev_c))
    density_factor = 1.0 / sigma if sigma > 0 else 1.0

    da = _density_altitude_ft(field_elev_ft, isa_dev_c)
    ground_roll = base * weight_factor * density_factor

    return POHGroundRollEstimate(
        ground_roll_ft=ground_roll,
        base_sl_mtow_ft=float(base),
        weight_factor=weight_factor,
        density_factor=density_factor,
        density_altitude_ft=da,
        sigma=sigma,
        aircraft_key=aircraft_key,
        cited=True,
    )


# ---------------------------------------------------------------------------
#  POH Best-Glide Speed table (KIAS at MTOW, clean configuration)
# ---------------------------------------------------------------------------
#
# Vbg in the POH is published at MTOW and is essentially independent of
# density altitude (it's an IAS).  At lighter weights, the AERODYNAMIC
# best-glide speed scales as sqrt(W/MTOW) (same CL, lower lift required
# → lower speed).  We expose three configurations matching the existing
# sidebar overrides:
#
#   clean      — gear up, flaps up
#   geardown   — gear down, flaps up (retract aircraft only)
#   landing    — gear down, flaps full
#
# For fixed-gear aircraft, geardown == clean.  Landing-config Vbg is
# typically ~10–15% below clean (more drag → lower L/Dmax airspeed).
# Where the POH does not publish a landing-config Vbg, we estimate
# 0.88 × clean as a conservative placeholder.
#
# Sources: aircraft POHs and well-known third-party performance summaries.

POH_VBG_KIAS = {
    # key       :  (clean, geardown, landing)
    'J3Cub':      ( 60,  60,  55),
    'Husky':      ( 65,  65,  60),
    'RV-6A':      ( 87,  87,  78),
    'RV-7A':      ( 91,  91,  82),
    'RV-8':       ( 91,  91,  82),
    'RV-10':      (100, 100,  90),
    'RV-12':      ( 70,  70,  65),
    'RV-14A':     ( 95,  95,  85),
    'C172S':      ( 68,  68,  60),
    'C150':       ( 60,  60,  55),
    'C152':       ( 60,  60,  55),
    'PA-38':      ( 70,  70,  62),   # Tomahawk
    'DA20':       ( 73,  73,  65),
    'C182T':      ( 76,  76,  68),
    'C182RG':     ( 70,  60,  60),   # retractable: gear-down adds drag
    'SR22':       ( 88,  88,  79),
    'SR22T':      ( 88,  88,  79),
    'TTx':        ( 95,  90,  82),
    'A36':        (110,  95,  90),   # retract
    'PA28':       ( 73,  73,  65),
    'M20V':       (105,  90,  85),   # retract
    'Mirage':     ( 88,  78,  72),   # PA-46 piston, retract
    'TBM850':     (124, 110,  98),
    'TBM910':     (124, 110,  98),
    'TBM960':     (124, 110,  98),
    'Meridian':   (105,  92,  85),
    'M600':       (110,  98,  90),
    'M700':       (115, 100,  92),
    'PC12':       (130, 115, 105),
    'C208':       ( 95,  95,  82),   # fixed gear
    'C208B':      ( 95,  95,  82),
    'C208EX':     ( 95,  95,  82),
    'Kodiak100':  ( 88,  88,  78),
    'Kodiak900':  ( 92,  92,  82),
    'Denali':     (105,  92,  85),   # target spec
    'E1000':      (118, 105,  95),
}


def vbg_poh_kias(
    aircraft_key: str,
    config_or_mtow,
    weight_lb: float,
    configuration: str = 'clean',
) -> float | None:
    """Return POH best-glide speed adjusted for weight, or None if not in table.

    Args:
        aircraft_key: model key (e.g. 'C172S')
        config_or_mtow: AircraftConfig or float MTOW
        weight_lb: actual weight
        configuration: 'clean' | 'geardown' | 'landing'

    Returns:
        Vbg in KIAS, scaled by sqrt(W/MTOW) from the POH MTOW value.
    """
    row = POH_VBG_KIAS.get(aircraft_key)
    if row is None:
        return None
    idx = {'clean': 0, 'geardown': 1, 'landing': 2}.get(configuration, 0)
    vbg_mtow = float(row[idx])
    mtow = float(getattr(config_or_mtow, 'MTOW', config_or_mtow))
    if mtow <= 0:
        return vbg_mtow
    # Vbg scales as sqrt(W/MTOW) at constant CL_opt.
    import math as _m
    return vbg_mtow * _m.sqrt(max(0.0, float(weight_lb) / mtow))


# ---------------------------------------------------------------------------
#  POH Climb performance (Vy and rate of climb at MTOW, sea level, ISA)
# ---------------------------------------------------------------------------
#
# Vy (best-rate climb speed) is published at MTOW and scales weakly with
# weight; we treat it as constant at the POH value (engineering convention
# for light aircraft).
#
# Rate of climb scales with:
#   ROC(W, ρ) ≈ ROC_MTOW_SL × (MTOW/W) × σ
# for normally-aspirated airframes (excess power ~ thrust×velocity, both
# of which drop with σ; weight scaling is conservative — the actual ROC
# curve is close to (MTOW/W) but with some non-linearity).  Turbocharged
# / turboprop airframes hold thrust to higher altitude — for those we
# use σ^0.5 instead of σ as a less-aggressive density penalty.
#
# Sources: aircraft POHs and manufacturer spec sheets.

# (Vy_KIAS, ROC_fpm_at_MTOW_SL_ISA, density_exponent)
#   density_exponent = 1.0 → normally-aspirated (full σ penalty)
#   density_exponent = 0.5 → turbocharged / turboprop (sqrt σ)
POH_CLIMB = {
    'J3Cub':      (  55,  450, 1.0),
    'Husky':      (  70, 1500, 1.0),
    'RV-6A':      (  87, 1700, 1.0),
    'RV-7A':      (  90, 1800, 1.0),
    'RV-8':       (  90, 1800, 1.0),
    'RV-10':      (  98, 1500, 1.0),
    'RV-12':      (  72,  900, 1.0),
    'RV-14A':     (  95, 1500, 1.0),
    'C172S':      (  74,  730, 1.0),
    'C150':       (  68,  670, 1.0),
    'C152':       (  67,  715, 1.0),
    'PA-38':      (  70,  700, 1.0),
    'DA20':       (  68,  890, 1.0),
    'C182T':      (  80,  924, 1.0),
    'C182RG':     (  88, 1140, 1.0),
    'SR22':       (  96, 1270, 1.0),
    'SR22T':      (  96, 1203, 0.5),  # turbocharged
    'TTx':        ( 100, 1400, 0.5),  # turbocharged
    'A36':        ( 100, 1230, 1.0),
    'PA28':       (  76,  667, 1.0),
    'M20V':       ( 105, 1240, 0.5),  # turbocharged
    'Mirage':     ( 110, 1500, 0.5),  # turbocharged
    'TBM850':     ( 124, 2380, 0.5),  # turboprop
    'TBM910':     ( 124, 2380, 0.5),
    'TBM960':     ( 124, 2380, 0.5),
    'Meridian':   ( 110, 1500, 0.5),
    'M600':       ( 120, 2000, 0.5),
    'M700':       ( 125, 2048, 0.5),
    'PC12':       ( 120, 1920, 0.5),
    'C208':       (  95,  975, 0.5),
    'C208B':      (  95,  975, 0.5),
    'C208EX':     ( 104, 1234, 0.5),
    'Kodiak100':  (  85, 1374, 0.5),
    'Kodiak900':  (  91, 1670, 0.5),
    'Denali':     ( 124, 2500, 0.5),
    'E1000':      ( 130, 4000, 0.5),
}


def vy_poh_kias(aircraft_key: str) -> float | None:
    """Return POH Vy (best-rate climb speed) in KIAS, or None if not in table."""
    row = POH_CLIMB.get(aircraft_key)
    return float(row[0]) if row else None


def roc_poh_fpm(
    aircraft_key: str,
    config_or_mtow,
    weight_lb: float,
    field_elev_ft: float = 0.0,
    isa_dev_c: float = 0.0,
) -> float | None:
    """Return POH ROC (fpm) scaled to actual weight and density altitude.

    Scaling:  ROC(W, ρ) = ROC_MTOW_SL × (MTOW/W) × σ^n
    where n = 1.0 for normally-aspirated, 0.5 for turbo/turboprop.

    Args:
        aircraft_key: model key
        config_or_mtow: AircraftConfig or float MTOW
        weight_lb: actual weight
        field_elev_ft: field elevation MSL
        isa_dev_c: ISA deviation in °C

    Returns:
        ROC in fpm at the requested conditions, or None if not in table.
    """
    row = POH_CLIMB.get(aircraft_key)
    if row is None:
        return None
    _, roc_mtow_sl, density_exp = row
    mtow = float(getattr(config_or_mtow, 'MTOW', config_or_mtow))
    weight_factor = (mtow / float(weight_lb)) if weight_lb > 0 else 1.0
    _, _, sigma, _, _, _ = atmos(float(field_elev_ft), float(isa_dev_c))
    density_factor = max(sigma, 0.01) ** float(density_exp)
    return float(roc_mtow_sl) * weight_factor * density_factor


# ---------------------------------------------------------------------------
#  POH Stall Speed table (KIAS at MTOW, power-off)
# ---------------------------------------------------------------------------
#
# Vs (Vs1) — clean, power-off, MTOW
# Vs0     — full landing flaps (and gear, if retract), power-off, MTOW
#
# Stall speed scales as sqrt(W/MTOW) at constant CLmax.
#
# Sources: aircraft POHs (Section 5 / Performance, or §1 Limitations).

POH_VS_KIAS = {
    # key       :  (Vs_clean, Vs0_landing)
    'J3Cub':      ( 33,  33),   # no flaps
    'Husky':      ( 43,  37),
    'RV-6A':      ( 49,  47),
    'RV-7A':      ( 51,  46),
    'RV-8':       ( 51,  46),
    'RV-10':      ( 60,  55),
    'RV-12':      ( 45,  41),
    'RV-14A':     ( 55,  50),
    'C172S':      ( 48,  40),
    'C150':       ( 42,  35),
    'C152':       ( 43,  35),
    'PA-38':      ( 50,  44),
    'DA20':       ( 42,  37),
    'C182T':      ( 49,  41),
    'C182RG':     ( 50,  41),
    'SR22':       ( 70,  60),
    'SR22T':      ( 70,  60),
    'TTx':        ( 70,  60),
    'A36':        ( 65,  55),
    'PA28':       ( 53,  45),
    'M20V':       ( 60,  53),
    'Mirage':     ( 69,  58),
    'TBM850':     ( 79,  65),
    'TBM910':     ( 79,  65),
    'TBM960':     ( 79,  65),
    'Meridian':   ( 75,  60),
    'M600':       ( 75,  61),
    'M700':       ( 75,  61),
    'PC12':       ( 80,  67),
    'C208':       ( 70,  53),
    'C208B':      ( 71,  54),
    'C208EX':     ( 71,  54),
    'Kodiak100':  ( 60,  50),
    'Kodiak900':  ( 65,  53),
    'Denali':     ( 75,  61),
    'E1000':      ( 80,  67),
}


def vs_poh_kias(
    aircraft_key: str,
    config_or_mtow,
    weight_lb: float,
    configuration: str = 'clean',
) -> float | None:
    """Return POH stall speed scaled to weight, or None if not in table.

    Args:
        aircraft_key: model key
        config_or_mtow: AircraftConfig or float MTOW
        weight_lb: actual weight
        configuration: 'clean' (Vs1) | 'landing' (Vs0)

    Returns:
        Vs in KIAS, scaled by sqrt(W/MTOW) from the POH MTOW value.
    """
    row = POH_VS_KIAS.get(aircraft_key)
    if row is None:
        return None
    idx = 1 if configuration == 'landing' else 0
    vs_mtow = float(row[idx])
    mtow = float(getattr(config_or_mtow, 'MTOW', config_or_mtow))
    if mtow <= 0:
        return vs_mtow
    import math as _m
    return vs_mtow * _m.sqrt(max(0.0, float(weight_lb) / mtow))


# ---------------------------------------------------------------------------
#  POH Landing ground-roll table (sea level, ISA, MLW, dry paved)
# ---------------------------------------------------------------------------
#
# Landing ground-roll only (not the 50-ft obstacle distance), at MLW
# (≈ MTOW for most light aircraft) and with maximum braking on dry paved
# runway.  Use the same scaling family as takeoff:
#
#     LR(W, DA) = LR_ref × (W/MLW)^2 × (1/σ)
#
# This is conservative — actual landing distance scales somewhat better
# with weight than takeoff, but the (W/MLW)^2 form keeps us on the safe
# side for go/no-go decisions.

POH_LANDING_ROLL_FT = {
    'J3Cub':      290,
    'Husky':      350,
    'RV-6A':      500,
    'RV-7A':      500,
    'RV-8':       500,
    'RV-10':      525,
    'RV-12':      350,
    'RV-14A':     500,
    'C172S':      575,
    'C150':       445,
    'C152':       475,
    'PA-38':      635,
    'DA20':       620,
    'C182T':      590,
    'C182RG':     600,
    'SR22':      1140,
    'SR22T':     1140,
    'TTx':       1100,
    'A36':       1000,
    'PA28':       625,
    'M20V':      1080,
    'Mirage':    1100,
    'TBM850':    2100,
    'TBM910':    2100,
    'TBM960':    2100,
    'Meridian':  2050,
    'M600':      1968,
    'M700':      1968,
    'PC12':      2150,
    'C208':      1735,
    'C208B':     1745,
    'C208EX':    1625,
    'Kodiak100': 1485,
    'Kodiak900': 1605,
    'Denali':    1850,
    'E1000':     1875,
}


def landing_roll_poh_ft(
    aircraft_key: str,
    config_or_mtow,
    weight_lb: float,
    field_elev_ft: float = 0.0,
    isa_dev_c: float = 0.0,
) -> float | None:
    """Return POH landing ground-roll scaled to weight + DA, or None."""
    base = POH_LANDING_ROLL_FT.get(aircraft_key)
    if base is None:
        return None
    mtow = float(getattr(config_or_mtow, 'MTOW', config_or_mtow))
    weight_factor = (float(weight_lb) / mtow) ** 2 if mtow > 0 else 1.0
    _, _, sigma, _, _, _ = atmos(float(field_elev_ft), float(isa_dev_c))
    density_factor = 1.0 / sigma if sigma > 0 else 1.0
    return float(base) * weight_factor * density_factor

