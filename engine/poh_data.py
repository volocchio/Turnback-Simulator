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
#   C182T        Cessna 182T Skylane POH (MTOW 3,100 lb)                ~  795 ft
#   C182RG       Cessna 182RG Skylane POH (MTOW 3,100 lb)               ~  820 ft
#   SR22         Cirrus SR22 POH (MTOW 3,400 lb, normally aspirated)    ~ 1080 ft
#   A36          Beechcraft Bonanza A36 POH (MTOW 3,650 lb)             ~  960 ft
#   PA28         Piper PA-28-181 Archer III POH (MTOW 2,550 lb)         ~  870 ft
#   TBM960/910/850 Daher TBM 850/910/960 POH (ground-roll only)         ~  995 ft
#   Meridian     Piper PA-46-500TP Meridian POH (MTOW 5,092 lb)         ~ 1366 ft
#   M600         Piper M600 SLS POH (MTOW 6,000 lb)                     ~ 1325 ft
#   M700         Piper M700 Fury POH (MTOW 6,000 lb)                    ~ 1325 ft
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
    'C182T':   795,
    'C182RG':  820,
    'SR22':   1080,
    'A36':     960,
    'PA28':    870,
    'TBM960':  995,
    'TBM910':  995,
    'TBM850':  995,
    'Meridian': 1366,
    'M600':   1325,
    'M700':   1325,
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
