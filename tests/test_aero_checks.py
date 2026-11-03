"""Hand-checkable aerodynamics and a smoke test of the app.

These lock the relationships a pilot or certification engineer can verify
with a calculator: stall speed scales with sqrt(n), turn radius is
V^2 / (g tan phi), and a steady glide loses altitude at V * D/W.
"""

import math

import pytest

from analysis.turnback_simulator import (
    DT,
    G_FPS2,
    _kias_to_fps,
    _return_heading,
    build_turnback_envelope,
    classify_maneuver,
    simulate_straight_ahead,
    simulate_turnback,
)
from engine.aircraft_config import AIRCRAFT_CONFIG
from engine.flight_physics import _density_altitude_ft, atmos


@pytest.fixture
def c172():
    return AIRCRAFT_CONFIG[("C172S", "Flatwing")]


def test_stall_speed_scales_with_sqrt_load_factor(c172):
    """Vs(phi) = Vs * sqrt(n), n = 1/cos(phi).

    45° is +19% (n = sqrt(2), sqrt(n) = 2^0.25), not +41%.
    60° is +41% (n = 2, sqrt(n) = sqrt(2)).
    """
    from analysis.turnback_simulator import vs_kias_at_load

    vs1 = vs_kias_at_load(c172, c172.MTOW, 1.0, 0.0, 0.0)
    for bank, expected_ratio in ((30, 1 / math.sqrt(math.cos(math.radians(30)))),
                                 (45, math.sqrt(math.sqrt(2.0))),
                                 (60, math.sqrt(2.0))):
        n = 1.0 / math.cos(math.radians(bank))
        vs = vs_kias_at_load(c172, c172.MTOW, n, 0.0, 0.0)
        assert vs == pytest.approx(vs1 * math.sqrt(n), rel=1e-6)
        assert vs / vs1 == pytest.approx(expected_ratio, rel=1e-6)
    # The common mix-up: treating n itself as the stall-speed multiplier.
    n45 = 1.0 / math.cos(math.radians(45))
    vs45 = vs_kias_at_load(c172, c172.MTOW, n45, 0.0, 0.0)
    assert vs45 / vs1 == pytest.approx(1.189, abs=0.01)
    assert vs45 / vs1 != pytest.approx(1.41, abs=0.02)


def test_turn_radius_matches_level_turn_formula(c172):
    """Reported radius is V_TAS^2 / (g tan phi), in feet."""
    bank = 30.0
    kias = 90.0
    alt = 3000.0
    r = simulate_turnback(
        c172, 2300.0, alt, kias, bank, 0, 1.0,
        speed_mode="fixed", prop_state="feathered",
    )
    _, _, sigma, delta, _, _ = atmos(alt, 0.0)
    v_fps, _, _ = _kias_to_fps(kias, sigma, delta)
    expected = v_fps ** 2 / (G_FPS2 * math.tan(math.radians(bank)))
    assert r["turn_radius_ft"] == pytest.approx(expected, rel=0.02)


def test_integrated_ground_track_radius_uses_horizontal_speed(c172):
    """A gliding turn's ground-track radius is the level-turn radius times cos(gamma).

    omega = g tan(phi) / V_TAS is the coordinated gliding-turn rate.
    The ground track then has R = V_horiz / omega = V^2 cos(gamma) / (g tan phi).
    """
    bank = 30.0
    r = simulate_turnback(
        c172, 2300.0, 5000.0, 90.0, bank, 0, 0.2,
        speed_mode="fixed", prop_state="feathered",
    )
    pts = [p for p in r["trajectory"]
           if p["phase"] == "turn" and abs(p["bank_deg"] - bank) < 0.5]
    assert len(pts) >= 8
    ds = 0.0
    for a, b in zip(pts, pts[1:]):
        ds += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    dpsi = math.radians(pts[-1]["cum_turn_deg"] - pts[0]["cum_turn_deg"])
    assert dpsi > math.radians(20)
    r_ground = ds / dpsi
    gamma = math.radians(pts[len(pts) // 2]["gamma_deg"])
    # Level-turn radius at the true airspeed actually being flown.
    v = pts[len(pts) // 2]["v_fps"]
    r_level = v ** 2 / (G_FPS2 * math.tan(math.radians(bank)))
    assert r_ground == pytest.approx(r_level * math.cos(gamma), rel=0.08)


def test_steady_wings_level_glide_sink(c172):
    """Wings-level, constant KIAS: altitude loss rate is V * D/W.

    sin(gamma) = -D/W with T = 0. Bank is zero so n = 1 and there is no
    speed change, so the kinetic-energy term is zero.
    """
    weight = 2400.0
    kias = 80.0
    r = simulate_turnback(
        c172, weight, 4000.0, kias, 0.0, 0, 15.0,
        speed_mode="fixed", prop_state="feathered", gear_down=True,
    )
    pts = r["trajectory"]
    assert len(pts) > 5
    # A few seconds in, still in the reaction (wings level, target = failure speed).
    p0 = pts[0]
    p1 = min(pts, key=lambda p: abs(p["time"] - 8.0))
    assert p1["time"] == pytest.approx(8.0, abs=0.6)
    assert p1["phase"] == "reaction"
    assert p1["kias"] == pytest.approx(kias, abs=0.2)

    _, _, sigma, delta, _, _ = atmos(4000.0, 0.0)
    v_fps, vkeas, _ = _kias_to_fps(kias, sigma, delta)
    q = vkeas ** 2 / 295.0
    S = c172.wing_area
    from analysis.turnback_simulator import PROP_DRAG_INCREMENTS, _compute_aero_k
    cdo = c172.cdo + PROP_DRAG_INCREMENTS["feathered"]
    k = _compute_aero_k(S, c172.span, c172.oswald, c172.winglet_height)
    cl = weight / (q * S)
    drag = (cdo + k * cl ** 2) * q * S
    sin_gamma = -drag / weight
    expected_dz = v_fps * sin_gamma * p1["time"]  # negative
    assert p1["z"] - p0["z"] == pytest.approx(expected_dz, rel=0.05)


def test_speed_change_exchanges_kinetic_energy(c172):
    """Slowing from the failure speed to the glide target gives altitude back.

    The recorded roc_fpm is the constant-speed term V*sin(gamma). The
    integrated altitude must sit above that integral by (V1^2 - V2^2)/(2g).
    """
    r = simulate_turnback(
        c172, 2300.0, 4000.0, 100.0, 30.0, 0, 0.0,
        speed_mode="vs_plus_10", prop_state="feathered",
    )
    pts = [p for p in r["trajectory"] if p["time"] <= 12.0]
    assert pts[-1]["kias"] < pts[0]["kias"] - 5.0
    z_from_drag = pts[0]["z"]
    for a, b in zip(pts, pts[1:]):
        z_from_drag += (a["roc_fpm"] / 60.0) * (b["time"] - a["time"])
    energy_ft = (pts[0]["v_fps"] ** 2 - pts[-1]["v_fps"] ** 2) / (2.0 * G_FPS2)
    extra = pts[-1]["z"] - z_from_drag
    assert energy_ft > 30.0  # this case really does slow down
    assert extra == pytest.approx(energy_ft, abs=25.0)


def test_headwind_shortens_track_hold_climb(c172):
    """Track-hold: a headwind reduces groundspeed, so the failure is closer."""
    args = dict(weight=2400.0, failure_alt_agl=800.0, airspeed_kias=74.0,
                bank_angle_deg=30.0, flap_setting=0, reaction_time=1.0,
                liftoff_distance=900.0, climb_steering="track", speed_mode="fixed")
    calm = simulate_turnback(c172, wind_speed_kt=0.0, wind_from_deg=0.0, **args)
    head = simulate_turnback(c172, wind_speed_kt=20.0, wind_from_deg=0.0, **args)
    assert head["trajectory"][0]["y"] < calm["trajectory"][0]["y"] - 500.0
    # Headwind is along the runway, not a lateral drift.
    assert abs(head["trajectory"][0]["x"]) < 1.0


def test_straight_ahead_uses_aloft_wind(c172):
    """A tailwind that exists only aloft must move the touchdown downrange.

    The straight-ahead path used to keep the surface wind speed for the
    whole glide and ignore the winds-aloft profile.
    """
    args = dict(weight=2400.0, failure_alt_agl=1200.0, airspeed_kias=74.0,
                reaction_time=1.0, runway_length=20000.0, liftoff_distance=900.0,
                speed_mode="fixed")
    calm = simulate_straight_ahead(c172, wind_speed_kt=0.0, wind_from_deg=0.0, **args)
    aloft = simulate_straight_ahead(
        c172, wind_speed_kt=0.0, wind_from_deg=180.0,
        wind_profile=[(800, 25.0), (2000, 25.0)], **args,
    )
    assert aloft["touchdown_y"] > calm["touchdown_y"] + 200.0


def test_classify_270_is_not_a_540():
    """A 250° alignment with no orbit is a 270, not a 540."""
    base = {"success": True, "landing_direction": "reverse", "phase_summary": [{"phase": "turn"}]}
    assert classify_maneuver({**base, "total_turn_deg": 190}) == "180"
    assert classify_maneuver({**base, "total_turn_deg": 250}) == "270"
    assert classify_maneuver({**base, "total_turn_deg": 250,
                              "phase_summary": [{"phase": "turn"}, {"phase": "orbit"}]}) == "540"
    assert classify_maneuver({**base, "landing_direction": "original",
                              "total_turn_deg": 400}) == "circuit"
    assert classify_maneuver({"success": False, "total_turn_deg": 300}) is None


def test_return_heading_leads_the_centerline():
    """From well left of the runway, the intercept aims short of the touchdown target."""
    # y=8000, target_y=400, x=-800 (left). Lead should be toward decreasing y
    # but not all the way to 400, so the heading is more than a few degrees
    # off the reciprocal.
    hdg = _return_heading(-800.0, 8000.0, 400.0)
    # atan2(dx, dy): dx = +800 (toward centerline), dy negative.
    assert math.pi / 2 < hdg < math.pi
    direct = math.atan2(800.0, 400.0 - 8000.0)
    # More intercept angle than the pure bearing to the far aim point.
    assert hdg < direct


def test_density_altitude_matches_isa_definition():
    """Closed-form density altitude reproduces sigma(h) = sigma by construction."""
    for alt, isa in ((0, 0), (0, 20), (5000, -10), (10000, 15), (18000, 0)):
        d_alt, theta, sigma, delta, _, _ = atmos(alt, isa)
        assert sigma == pytest.approx(delta / theta, rel=1e-12)
        # Round trip: ISA density ratio at the reported density altitude
        # equals the actual density ratio.
        if d_alt <= 36089:
            sig_back = (1.0 - 6.8756e-6 * d_alt) ** 4.2559
        else:
            sig_back = 0.2971 * math.exp(-4.80637e-5 * (d_alt - 36089.0))
        assert sig_back == pytest.approx(sigma, rel=1e-6)
        assert d_alt == pytest.approx(_density_altitude_ft(sigma), abs=0.05)


def test_nonphysical_inputs_do_not_raise(c172):
    r = simulate_turnback(c172, float("nan"), 1000, 70, 45, 0, 2.0)
    assert r["success"] is False
    assert r["trajectory"] == []
    r = simulate_turnback(c172, 2300, 1000, 70, 120, 0, 2.0)  # bank past 90°, clamped
    assert math.isfinite(r["turn_radius_ft"])
    assert r["trajectory"]


def test_app_imports_and_envelope_runs(c172):
    """Smoke: the Streamlit module imports, and a short envelope is finite."""
    import turnback_app
    assert callable(turnback_app.run_turnback_page)
    crit, env, left, right, _sa = build_turnback_envelope(
        c172, 2200.0, 74.0, 45.0, 0, 2.0,
        alt_step=500, max_alt=1000,
        runway_length=5000, liftoff_distance=900,
        speed_mode="vs_plus_10",
    )
    assert math.isfinite(crit)
    assert math.isfinite(left) and math.isfinite(right)
    assert env
    assert all(math.isfinite(row["alt_agl"]) for row in env)
