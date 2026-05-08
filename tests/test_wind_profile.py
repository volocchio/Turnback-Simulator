"""Tests for the flexible winds-aloft profile interpolator (Charlie #4)."""

import pytest
from analysis.turnback_simulator import (
    _interpolate_wind_profile,
    _interpolate_wind_dir_profile,
    simulate_turnback,
    find_critical_altitude,
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


# --- E3: wind direction interpolation --------------------------------------

def test_dir_interp_constant_above_top():
    assert _interpolate_wind_dir_profile(5000, 45, [(1000, 100)]) == pytest.approx(100.0)


def test_dir_interp_handles_360_wrap():
    """350 deg at surface, 10 deg at 1000 ft.  Midpoint should be ~0/360 deg,
    NOT the broken linear-degree result of 180 deg."""
    mid = _interpolate_wind_dir_profile(500, 350, [(1000, 10)])
    # Allow either 0 or 360 (atan2 may return either)
    assert min(abs(mid - 0.0), abs(mid - 360.0)) < 0.5


def test_dir_interp_returns_surface_at_zero():
    assert _interpolate_wind_dir_profile(0, 270, [(1000, 90)]) == pytest.approx(270.0)


# --- E1: intersection departure --------------------------------------------

def test_intersection_offset_changes_critical_altitude():
    cfg = AIRCRAFT_CONFIG[('C172S', 'Flatwing')]
    common = dict(
        airspeed_kias=70, bank_angle_deg=45, flap_setting=0,
        reaction_time=3.0, field_elevation=0.0, isa_dev=0.0,
        runway_length=4000, liftoff_distance=900,
        speed_mode='vs_plus_10',
    )
    crit_full = find_critical_altitude(cfg, 2400, **common)
    crit_intx = find_critical_altitude(cfg, 2400, intersection_offset_ft=1500.0, **common)
    # Intersection departure shrinks runway-remaining for straight-ahead and
    # shifts turnback geometry - critical altitude should differ.
    assert crit_full != crit_intx


# --- E2: gear retraction after failure -------------------------------------

def test_gear_retract_lowers_critical_altitude():
    cfg = AIRCRAFT_CONFIG[('C182RG', 'Flatwing')]
    common = dict(
        airspeed_kias=75, bank_angle_deg=45, flap_setting=0,
        reaction_time=3.0, field_elevation=0.0, isa_dev=0.0,
        speed_mode='vs_plus_10',
    )
    crit_keep = find_critical_altitude(cfg, 3000, gear_down=True, **common)
    crit_retr = find_critical_altitude(cfg, 3000, gear_down=True,
                                        gear_retract_time_s=1.0, **common)
    # Removing gear drag during the turn should yield equal or lower crit alt.
    assert crit_retr <= crit_keep
