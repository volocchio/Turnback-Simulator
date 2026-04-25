"""Tests for operational glide-speed targets (Charlie's recommended speeds)."""

import math
import pytest

from analysis.turnback_simulator import (
    vs_kias_at_load,
    operational_turn_speeds,
    simulate_turnback,
)
from engine.aircraft_config import AIRCRAFT_CONFIG


@pytest.fixture
def cfg():
    # Use any single-engine config available
    keys = [k for k, c in AIRCRAFT_CONFIG.items() if c.engines == 1]
    assert keys
    return AIRCRAFT_CONFIG[keys[0]]


def test_vs_at_1g_matches_clean_stall(cfg):
    """Vs(nz=1) should match the published clean stall to within rounding."""
    vs = vs_kias_at_load(cfg, cfg.MTOW, 1.0, 0, 0)
    expected = math.sqrt(295.0 * cfg.MTOW / (cfg.wing_area * cfg.Clmax))
    assert abs(vs - expected) < 1.0


def test_vs_scales_with_sqrt_load_factor(cfg):
    """Vs(nz) = Vs(1) × sqrt(nz)."""
    vs1 = vs_kias_at_load(cfg, cfg.MTOW, 1.0, 0, 0)
    vs2 = vs_kias_at_load(cfg, cfg.MTOW, 2.0, 0, 0)
    assert abs(vs2 - vs1 * math.sqrt(2.0)) < 0.5


def test_operational_turn_speeds_45_bank(cfg):
    """At 45° bank, nz=√2; vs(φ)+10 and 1.3·vs(φ) should follow."""
    ops = operational_turn_speeds(cfg, cfg.MTOW, 45, 0, 0)
    assert abs(ops['nz'] - math.sqrt(2.0)) < 0.001
    assert ops['vs_plus_10_kias'] == pytest.approx(ops['vs_kias'] + 10.0, abs=0.01)
    assert ops['vs_x_1p3_kias'] == pytest.approx(ops['vs_kias'] * 1.3, abs=0.01)


def test_operational_turn_speeds_30_bank_lower_than_45(cfg):
    """A shallower bank → lower nz → lower Vs in the turn."""
    ops30 = operational_turn_speeds(cfg, cfg.MTOW, 30, 0, 0)
    ops45 = operational_turn_speeds(cfg, cfg.MTOW, 45, 0, 0)
    assert ops30['vs_kias'] < ops45['vs_kias']


def test_simulate_with_vs_plus_10_mode(cfg):
    """End-to-end: simulator accepts the new mode and returns speed_info."""
    r = simulate_turnback(cfg, cfg.MTOW, 1500, 90, 45, 0, 3.0,
                          speed_mode='vs_plus_10')
    si = r.get('speed_info', {})
    assert si['mode'] == 'vs_plus_10'
    assert 'vs_plus_10_kias' in si
    assert 'target_kias' in si
    assert si['target_kias'] == pytest.approx(si['vs_plus_10_kias'], abs=0.5)


def test_simulate_with_vs_x_1p3_mode(cfg):
    r = simulate_turnback(cfg, cfg.MTOW, 1500, 90, 45, 0, 3.0,
                          speed_mode='vs_x_1p3')
    si = r.get('speed_info', {})
    assert si['mode'] == 'vs_x_1p3'
    assert si['target_kias'] == pytest.approx(si['vs_x_1p3_kias'], abs=0.5)
