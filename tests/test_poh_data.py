"""Tests for the POH-based ground-roll estimator (Charlie #5b)."""

import pytest
from engine.poh_data import (
    POH_GROUND_ROLL_FT,
    estimate_ground_roll,
)
from engine.aircraft_config import AIRCRAFT_CONFIG


def test_unknown_aircraft_returns_none():
    cfg = AIRCRAFT_CONFIG[('C172S', 'Flatwing')]
    assert estimate_ground_roll('NOT_A_REAL_PLANE', cfg, 2000) is None


def test_sl_mtow_returns_base():
    """At sea level / ISA / MTOW, the estimator must return ~POH base ±2 ft."""
    cfg = AIRCRAFT_CONFIG[('C172S', 'Flatwing')]
    est = estimate_ground_roll('C172S', cfg, cfg.MTOW, 0.0, 0.0)
    assert est is not None
    assert est.base_sl_mtow_ft == POH_GROUND_ROLL_FT['C172S']
    assert est.ground_roll_ft == pytest.approx(est.base_sl_mtow_ft, abs=2.0)


def test_lighter_weight_shorter_roll():
    cfg = AIRCRAFT_CONFIG[('SR22', 'Flatwing')]
    full = estimate_ground_roll('SR22', cfg, cfg.MTOW, 0.0, 0.0)
    light = estimate_ground_roll('SR22', cfg, cfg.MTOW * 0.8, 0.0, 0.0)
    assert light.ground_roll_ft < full.ground_roll_ft
    # Weight factor squared: 0.8² = 0.64
    assert light.weight_factor == pytest.approx(0.64, abs=0.001)


def test_high_density_altitude_lengthens_roll():
    """Sedona-ish: 4,800 ft elev + ISA+20 → big DA penalty."""
    cfg = AIRCRAFT_CONFIG[('C172S', 'Flatwing')]
    sl = estimate_ground_roll('C172S', cfg, cfg.MTOW, 0.0, 0.0)
    sez = estimate_ground_roll('C172S', cfg, cfg.MTOW, 4830.0, 20.0)
    assert sez.ground_roll_ft > sl.ground_roll_ft
    # At ~7000 ft DA, density ratio σ ≈ 0.81, so density_factor ≈ 1.23
    assert sez.density_factor > 1.15


def test_table_covers_main_singles():
    """Sanity-check that the curated table covers the popular GA singles."""
    for key in ['C172S', 'SR22', 'A36', 'PA28', 'TBM850', 'M600']:
        assert key in POH_GROUND_ROLL_FT
        assert POH_GROUND_ROLL_FT[key] > 0


def test_density_factor_is_one_at_sea_level_isa():
    cfg = AIRCRAFT_CONFIG[('C172S', 'Flatwing')]
    est = estimate_ground_roll('C172S', cfg, cfg.MTOW, 0.0, 0.0)
    assert est.density_factor == pytest.approx(1.0, abs=0.005)
    assert est.sigma == pytest.approx(1.0, abs=0.005)
