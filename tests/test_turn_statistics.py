"""Tests for turn-statistics post-processing (#7 — training output)."""

import pytest
from analysis.turnback_simulator import simulate_turnback
from engine.aircraft_config import AIRCRAFT_CONFIG


@pytest.fixture
def cfg():
    keys = [k for k, c in AIRCRAFT_CONFIG.items() if c.engines == 1]
    return AIRCRAFT_CONFIG[keys[0]]


def test_turn_stats_present(cfg):
    r = simulate_turnback(cfg, cfg.MTOW, 1500, 90, 45, 0, 3.0, speed_mode='fixed')
    assert 'total_turn_deg' in r
    assert 'altitude_loss_per_180' in r
    assert 'altitude_at_180_increments' in r


def test_total_turn_deg_at_least_180(cfg):
    """Any complete turnback must turn at least 180° (the maneuver definition)."""
    r = simulate_turnback(cfg, cfg.MTOW, 4000, 130, 45, 0, 3.0, speed_mode='fixed')
    # success may be marginal depending on config but the accumulator must
    # show at least a half-turn was flown.
    assert r['total_turn_deg'] >= 180.0


def test_increments_start_at_failure_altitude(cfg):
    r = simulate_turnback(cfg, cfg.MTOW, 1500, 90, 45, 0, 3.0, speed_mode='fixed')
    incs = r['altitude_at_180_increments']
    assert len(incs) >= 1
    assert incs[0]['turn_deg'] == 0
    assert incs[0]['altitude_agl'] == pytest.approx(1500, abs=1.0)


def test_loss_per_180_positive(cfg):
    """A turnback that includes 180° of turn must show positive altitude loss."""
    r = simulate_turnback(cfg, cfg.MTOW, 2000, 90, 45, 0, 3.0, speed_mode='fixed')
    if r['altitude_loss_per_180'] is not None:
        assert r['altitude_loss_per_180'] > 0
