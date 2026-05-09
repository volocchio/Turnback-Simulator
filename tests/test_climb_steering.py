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
