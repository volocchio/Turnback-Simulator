"""E2: heading-vs-track climb steering toggle."""
from analysis.turnback_simulator import simulate_turnback
from engine.aircraft_config import AIRCRAFT_CONFIG


def _config():
    return AIRCRAFT_CONFIG[("C172S", "Flatwing")]


def test_track_hold_starts_on_centerline():
    """Default (track) puts the airplane on the runway centerline at failure."""
    cfg = _config()
    res = simulate_turnback(
        cfg, 2300.0, 800.0, 70.0, 45.0, 0, 2.0,
        field_elevation=0.0, isa_dev=0.0, turn_direction='left',
        wind_speed_kt=20.0, wind_from_deg=90.0,  # 90deg = right crosswind
        liftoff_distance=900.0,
        climb_steering='track',
    )
    traj = res['trajectory']
    assert abs(traj[0]['x']) < 1.0  # x_init should be 0


def test_heading_hold_drifts_with_crosswind():
    """In heading-hold mode the airplane is offset downwind at failure."""
    cfg = _config()
    res = simulate_turnback(
        cfg, 2300.0, 800.0, 70.0, 45.0, 0, 2.0,
        field_elevation=0.0, isa_dev=0.0, turn_direction='left',
        wind_speed_kt=20.0, wind_from_deg=90.0,  # right crosswind
        liftoff_distance=900.0,
        climb_steering='heading',
    )
    traj = res['trajectory']
    # Wind-from 90deg => wind_x is negative => airplane drifts left (-x)
    assert traj[0]['x'] < -10.0


def test_heading_hold_zero_wind_matches_track():
    """With no wind both modes produce identical initial offset."""
    cfg = _config()
    args = dict(
        weight=2300.0, failure_alt_agl=800.0, airspeed_kias=70.0,
        bank_angle_deg=45.0, flap_setting=0, reaction_time=2.0,
        field_elevation=0.0, isa_dev=0.0, turn_direction='left',
        wind_speed_kt=0.0, wind_from_deg=0.0, liftoff_distance=900.0,
    )
    a = simulate_turnback(cfg, climb_steering='track', **args)
    b = simulate_turnback(cfg, climb_steering='heading', **args)
    assert abs(a['trajectory'][0]['x'] - b['trajectory'][0]['x']) < 0.5
    assert abs(a['trajectory'][0]['y'] - b['trajectory'][0]['y']) < 0.5


# ---------------------------------------------------------------------------
# P1 (smart aim point) — heading-hold should yield equal-or-lower critical
# altitude than track-hold in a crosswind, because the airplane is already
# offset toward the favorable side at engine failure and the smart aim
# point exploits that geometry to reduce required heading change.
# ---------------------------------------------------------------------------
def test_smart_aim_heading_hold_beats_track_in_crosswind():
    """Wind from 90° (right crosswind) on a heading-hold climb drifts the
    airplane LEFT of centerline at engine failure.  Turning RIGHT (into the
    wind) then brings the airplane back toward the runway with less than
    180° of heading change — the smart aim point exploits this geometry.

    The teaching rule: ALWAYS TURN INTO THE WIND.  In a heading-hold
    climbout, the wind has already pre-positioned the airplane on the
    favorable side of centerline; turning into the wind closes the gap.
    Turning downwind in the same scenario is actively worse.

    Critical altitude (turn into wind, heading-hold) should be equal or
    lower than the track-hold baseline."""
    from analysis.turnback_simulator import find_critical_altitude
    cfg = _config()
    common = dict(
        airspeed_kias=70, bank_angle_deg=45, flap_setting=0,
        reaction_time=2.0, field_elevation=0.0, isa_dev=0.0,
        runway_length=4000, liftoff_distance=900,
        wind_speed_kt=15.0, wind_from_deg=90.0,  # right crosswind
        turn_direction='right',                  # into the wind
        speed_mode='vs_plus_10',
    )
    crit_track = find_critical_altitude(cfg, 2300, climb_steering='track', **common)
    crit_hdg = find_critical_altitude(cfg, 2300, climb_steering='heading', **common)
    # Heading-hold should never be WORSE than track-hold when turning into
    # the wind.  Small tolerance for altitude-quantum jitter.
    assert crit_hdg <= crit_track + 5, (
        f"heading-hold ({crit_hdg} ft) should be <= track-hold "
        f"({crit_track} ft) when turning into the wind"
    )


def test_smart_aim_picks_runway_aim_point():
    """When the smart aim activates, the chosen target_y on the runway
    centerline should be within usable bounds."""
    cfg = _config()
    res = simulate_turnback(
        cfg, 2300.0, 1500.0, 70.0, 45.0, 0, 2.0,
        field_elevation=0.0, isa_dev=0.0, turn_direction='left',
        wind_speed_kt=15.0, wind_from_deg=90.0,
        runway_length=4000.0, liftoff_distance=900.0,
        climb_steering='heading',
        speed_mode='vs_plus_10',
    )
    if res.get('success') and res.get('trajectory'):
        # Last waypoint should land somewhere on the runway (0..length)
        final_y = res['trajectory'][-1]['y']
        assert 0 <= final_y <= 4000, f"final y={final_y} off runway"
