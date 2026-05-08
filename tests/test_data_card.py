"""Tests for the Takeoff Data Card HTML generator."""
from analysis.data_card import build_takeoff_data_card, _recommended_turn_direction
from engine.aircraft_config import AIRCRAFT_CONFIG


def _sample_res():
    cfg = AIRCRAFT_CONFIG[('Meridian', 'Flatwing')]
    return {
        'config': cfg,
        'ac_key': ('Meridian', 'Flatwing'),
        'weight': 4500,
        'airport_ident': 'KAPA',
        'runway_ident': '17L',
        'field_elev': 5800,
        'isa_dev': 10,
        'altimeter_inhg': 30.05,
        'wind_speed': 8,
        'wind_from_deg': 90,
        'wind_profile': [(1000, 12), (3000, 18), (6000, 24)],
        'airspeed': 95,
        'bank_angle': 45,
        'takeoff_flap_setting': 1,
        'flap_setting': 0,
        'reaction_time': 4.0,
        'speed_mode': 'fixed',
        'runway_length': 7000,
        'liftoff_distance': 1800,
        'critical_alt_left': 700,
        'critical_alt_right': 950,
        'straight_ahead_max_alt': 200,
    }


def _sample_crit():
    return {
        'total_turn_deg': 270,
        'altitude_loss_per_180': 350,
        'altitude_at_runway': 50,
        'turn_radius_ft': 850,
        'landing_direction': 'opposite',
    }


def test_card_renders_html():
    html = build_takeoff_data_card(_sample_res(), _sample_crit())
    assert html.startswith("<!doctype html>")
    assert "Takeoff Data Card" in html
    assert "Meridian Flatwing" in html
    assert "KAPA" in html
    assert "17L" in html


def test_card_includes_critical_decision_numbers():
    html = build_takeoff_data_card(_sample_res(), _sample_crit())
    # Lower critical alt should be recommended → LEFT
    assert "LEFT" in html
    # Critical altitudes should appear
    assert "700 ft AGL" in html
    assert "950 ft AGL" in html
    # Threshold-crossing margin
    assert "50 ft AGL" in html
    # Loss per 180
    assert "350 ft" in html


def test_card_includes_winds_aloft():
    html = build_takeoff_data_card(_sample_res(), _sample_crit())
    assert "1000 ft AGL" in html
    assert "12 kt" in html
    assert "6000 ft AGL" in html


def test_card_handles_missing_crit_result():
    html = build_takeoff_data_card(_sample_res(), None)
    assert "<!doctype html>" in html


def test_card_handles_empty_wind_profile():
    res = _sample_res()
    res['wind_profile'] = []
    html = build_takeoff_data_card(res, _sample_crit())
    assert "No winds-aloft profile" in html


def test_recommended_direction_lower_wins():
    # LEFT lower → recommend LEFT
    direction, _ = _recommended_turn_direction(700, 950, 90, 8)
    assert direction == "LEFT"
    # RIGHT lower → recommend RIGHT
    direction, _ = _recommended_turn_direction(950, 700, 90, 8)
    assert direction == "RIGHT"


def test_recommended_direction_tie_uses_wind():
    # Equal critical altitudes, right crosswind → turn RIGHT
    direction, rationale = _recommended_turn_direction(800, 800, 90, 10)
    assert direction == "RIGHT"
    assert "right" in rationale.lower()
    # Equal critical altitudes, left crosswind → turn LEFT
    direction, rationale = _recommended_turn_direction(800, 800, 270, 10)
    assert direction == "LEFT"


def test_recommended_direction_no_wind_tie_default_left():
    direction, _ = _recommended_turn_direction(800, 800, 0, 0)
    assert direction == "LEFT"


def test_card_dead_zone_warning():
    res = _sample_res()
    # Make a clear dead zone: SA max 200, crit min 700, gap=500 > 50
    html = build_takeoff_data_card(res, _sample_crit())
    assert "DEAD ZONE" in html


def test_card_no_dead_zone_when_overlap():
    res = _sample_res()
    res['straight_ahead_max_alt'] = 800  # > crit_min 700
    html = build_takeoff_data_card(res, _sample_crit())
    assert "NO DEAD ZONE" in html

# --- Sprint D1 (F3/F4/F5): MSL alongside AGL, departure-end threshold, phase summary ---

def test_card_renders_phase_summary():
    crit = dict(_sample_crit())
    crit['phase_summary'] = [
        {'phase': 'reaction', 'dt_s': 3.0, 'dz_loss_ft': 30, 'heading_change_deg': 0,
         'z_start_agl': 1000, 'z_end_agl': 970, 'explainer': 'Engine quits.'},
        {'phase': 'turn', 'dt_s': 13.7, 'dz_loss_ft': 200, 'heading_change_deg': 180,
         'z_start_agl': 970, 'z_end_agl': 770, 'explainer': 'Bank toward runway.'},
    ]
    html = build_takeoff_data_card(_sample_res(), crit)
    assert "Sequence of Events" in html
    assert "1. Reaction" in html
    assert "2. Turn back" in html
    assert "Engine quits" in html


def test_card_renders_departure_threshold_msl():
    crit = dict(_sample_crit())
    crit['altitude_at_departure_threshold'] = 590
    crit['altitude_at_takeoff_threshold'] = 120
    html = build_takeoff_data_card(_sample_res(), crit)
    # field_elev=5800, so MSL = 590+5800=6390, 120+5800=5920
    assert "590 ft AGL" in html
    assert "6,390 ft MSL" in html
    assert "120 ft AGL" in html
    assert "5,920 ft MSL" in html
    assert "downwind landing" in html.lower()


def test_card_threshold_alt_includes_msl():
    # Existing 'altitude_at_runway' = 50 in _sample_crit; MSL = 5850
    html = build_takeoff_data_card(_sample_res(), _sample_crit())
    assert "5,850 ft MSL" in html

def test_card_flap_retract_row_shown():
    res = _sample_res()
    res['flap_retract_alt_ft'] = 400
    html = build_takeoff_data_card(res, _sample_crit())
    assert "Flap-retract altitude" in html
    assert "400 ft AGL" in html


def test_card_flap_still_out_warning_when_crit_below_retract():
    res = _sample_res()
    res['flap_retract_alt_ft'] = 800  # > crit_min 700
    html = build_takeoff_data_card(res, _sample_crit())
    assert "FLAPS STILL OUT" in html


def test_card_no_flap_warning_when_crit_above_retract():
    res = _sample_res()
    res['flap_retract_alt_ft'] = 300  # < crit_min 700
    html = build_takeoff_data_card(res, _sample_crit())
    assert "FLAPS STILL OUT" not in html
