"""Tests for the flexible winds-aloft profile interpolator (Charlie #4)."""

import pytest
from analysis.turnback_simulator import (
    _interpolate_wind_profile,
    simulate_turnback,
)
from engine.aircraft_config import AIRCRAFT_CONFIG


def test_surface_returned_at_zero():
    assert _interpolate_wind_profile(0, 5, [(1000, 20)]) == 5


def test_constant_above_top():
    assert _interpolate_wind_profile(9999, 0, [(3000, 25)]) == 25


def test_linear_interp_between_rows():
    # Surface=0, 1000=10, 2000=20: at 1500 expect 15
    profile = [(1000, 10), (2000, 20)]
    assert _interpolate_wind_profile(1500, 0, profile) == pytest.approx(15.0)


def test_unsorted_profile_handled():
    profile = [(3000, 30), (1000, 10), (2000, 20)]
    assert _interpolate_wind_profile(1500, 0, profile) == pytest.approx(15.0)
    assert _interpolate_wind_profile(2500, 0, profile) == pytest.approx(25.0)


def test_non_standard_altitudes():
    # ForeFlight gives 3000, 6000, 9000 ft AGL — must work
    profile = [(3000, 5), (6000, 15), (9000, 30)]
    assert _interpolate_wind_profile(4500, 0, profile) == pytest.approx(10.0)
    assert _interpolate_wind_profile(7500, 0, profile) == pytest.approx(22.5)


def test_garbage_rows_ignored():
    profile = [(1000, 10), (None, 50), ("bad", 99), (2000, 20)]
    # Should still interpolate from valid rows
    assert _interpolate_wind_profile(1500, 0, profile) == pytest.approx(15.0)


def test_sim_accepts_wind_profile():
    keys = [k for k, c in AIRCRAFT_CONFIG.items() if c.engines == 1]
    cfg = AIRCRAFT_CONFIG[keys[0]]
    profile = [(1000, 10), (3000, 20), (6000, 25)]
    r = simulate_turnback(cfg, cfg.MTOW, 1500, 90, 45, 0, 3.0,
                          wind_speed_kt=5, wind_from_deg=0,
                          wind_profile=profile, speed_mode='fixed')
    # Just confirm it runs and returns the standard keys
    assert 'trajectory' in r
    assert 'success' in r
