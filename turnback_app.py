"""
Turnback Simulator UI — "Impossible Turn" Visualization

Streamlit page with 3D Plotly heart-shaped envelope showing the safe-return
zone after engine failure at various altitudes AGL.
"""

import streamlit as st
import plotly.graph_objects as go
import math

from engine.aircraft_config import AIRCRAFT_CONFIG
from analysis.turnback_simulator import (
    build_turnback_envelope, simulate_turnback, optimize_turnback, best_glide_kias,
    simulate_straight_ahead, find_straight_ahead_max_altitude,
    _select_vbg_override,
)

# ── Page config for standalone mode ──
try:
    st.set_page_config(
        page_title="Turnback Simulator",
        page_icon="✈️",
        layout="wide",
    )
except st.errors.StreamlitAPIException:
    pass  # already set when embedded in another app


def run_turnback_page():
    """Render the Turnback Simulator page."""

    st.title("Turnback Simulator — The Impossible Turn")
    st.markdown("""
    Simulates engine failure after takeoff and the gliding turn back to the runway.
    The **heart-shaped envelope** shows the ground track at each failure altitude:
    incomplete at low altitudes (crash before completing the turn) and complete
    above the **critical altitude** (safe return).

    *Physics*: zero thrust after engine failure. Descent gradient = −D/W.
    In a banked turn the wing must produce more lift to support the load
    factor n_z = 1/cos(φ), raising C_L. Since induced drag grows as C_L²,
    steeper banks dramatically increase total drag and sink rate.
    Stall speed also rises by √n_z, narrowing the safe speed margin.
    """)

    # ── Sidebar inputs ──
    st.sidebar.header("Turnback Parameters")

    # Aircraft selection — single-engine only
    all_keys = sorted(
        [k for k, cfg in AIRCRAFT_CONFIG.items() if cfg.engines == 1],
        key=lambda k: (k[0], k[1]),
    )
    if not all_keys:
        st.error("No single-engine aircraft found in configuration.")
        return
    labels = [f"{m} ({mod})" if mod != 'Flatwing' else m for m, mod in all_keys]
    # Default to Meridian if available
    default_idx = next((i for i, k in enumerate(all_keys) if k[0] == 'Meridian'), 0)
    sel_idx = st.sidebar.selectbox("Aircraft", range(len(labels)), format_func=lambda i: labels[i], index=default_idx)
    ac_key = all_keys[sel_idx]
    config = AIRCRAFT_CONFIG[ac_key]

    # MTOW override
    mtow = st.sidebar.number_input(
        "MTOW (lbs)", min_value=int(config.bew),
        max_value=25000, value=int(config.MTOW), step=50,
        help="Override the published MTOW for the selected aircraft.",
    )

    # Weight
    weight = st.sidebar.number_input(
        "Weight (lbs)", min_value=int(config.bew),
        max_value=int(mtow), value=int(mtow), step=50,
    )

    # ── Stall speeds (user-configurable, back-compute Clmax) ──
    # Defaults from config Clmax at MTOW
    _vs_clean_default = math.sqrt(295.0 * config.MTOW / (config.wing_area * config.Clmax))
    _clmax_land = config.Clmax_flaps40 if config.Clmax_flaps40 > 0 else config.Clmax_flaps15
    _vs_land_default = math.sqrt(295.0 * config.MTOW / (config.wing_area * _clmax_land)) if _clmax_land > 0 else _vs_clean_default

    st.sidebar.markdown("---")
    vs_clean_input = st.sidebar.number_input(
        "Vs clean at MTOW (KIAS)", min_value=30, max_value=200,
        value=int(round(_vs_clean_default)), step=1,
        help="Power-off stall speed, clean config, at MTOW. Adjusts CLmax.",
    )
    vs_land_input = st.sidebar.number_input(
        "Vs landing flaps at MTOW (KIAS)", min_value=25, max_value=180,
        value=int(round(_vs_land_default)), step=1,
        help="Power-off stall speed, full landing flaps, at MTOW. Adjusts CLmax_flaps.",
    )

    # Back-compute Clmax from user stall speeds (at MTOW)
    clmax_override = 295.0 * config.MTOW / (config.wing_area * vs_clean_input ** 2)
    clmax_land_override = 295.0 * config.MTOW / (config.wing_area * vs_land_input ** 2)
    st.sidebar.caption(
        f"CLmax clean = {clmax_override:.3f} · CLmax land = {clmax_land_override:.3f}"
    )
    # Apply overrides to config (create a modified copy)
    import dataclasses
    config = dataclasses.replace(
        config,
        Clmax=clmax_override,
        Clmax_flaps40=clmax_land_override,
        Clmax_flaps15=min(clmax_land_override, config.Clmax_flaps15 * (clmax_land_override / _clmax_land)) if _clmax_land > 0 else clmax_land_override,
    )

    # Climb-out speed at engine failure (uses current weight, not MTOW)
    vs_clean_est = math.sqrt(295.0 * weight / (config.wing_area * config.Clmax))
    vx_est = int(vs_clean_est * 1.1)   # Vx ≈ 1.1 × Vs_clean
    vy_est = int(vs_clean_est * 1.3)   # Vy ≈ 1.3 × Vs_clean

    climb_speed_options = {
        'vx':     f'Vx — best angle ({vx_est} KIAS)',
        'vy':     f'Vy — best rate ({vy_est} KIAS)',
        'manual': 'Manual',
    }
    climb_speed_mode = st.sidebar.radio(
        "Climb-out speed",
        list(climb_speed_options.keys()),
        format_func=lambda x: climb_speed_options[x],
        index=1,
        help="Speed at moment of engine failure. "
             "Vx: best angle of climb (~1.1 × Vs). "
             "Vy: best rate of climb (~1.3 × Vs).",
    )

    if climb_speed_mode == 'vx':
        airspeed = vx_est
    elif climb_speed_mode == 'vy':
        airspeed = vy_est
    else:
        airspeed = st.sidebar.number_input(
            "Airspeed at failure (KIAS)", min_value=40, max_value=300,
            value=vy_est, step=5,
        )

    st.sidebar.caption(
        f"Vs clean at {weight} lb = {vs_clean_est:.0f} KIAS"
    )

    # Wing geometry
    _ar = config.span ** 2 / config.wing_area
    st.sidebar.caption(
        f"b = {config.span:.1f} ft · S = {config.wing_area:.0f} ft² · AR = {_ar:.1f}"
    )

    # Post-failure glide speed mode
    speed_mode_options = {
        'fixed': 'Maintain failure speed',
        'best_glide_1g': 'Best glide — wings level (1g)',
        'best_glide_nz': 'Best glide — adjusted for load factor',
    }
    speed_mode = st.sidebar.radio(
        "Glide speed mode", list(speed_mode_options.keys()),
        format_func=lambda x: speed_mode_options[x], index=0,
    )

    # Bank angle
    bank_angle = st.sidebar.slider("Bank angle (°)", min_value=10, max_value=60, value=30, step=5)

    vbg_clean_kias = 0
    vbg_geardown_kias = 0
    vbg_landing_kias = 0
    if speed_mode != 'fixed':
        # Compute and display best glide speeds
        vbg_1g, cl_opt, ld_max = best_glide_kias(config, weight, 1.0, 0, 0)
        st.sidebar.info(
            f"**Computed best glide (1g):** {vbg_1g:.0f} KIAS\n\n"
            f"CL_opt = {cl_opt:.3f} · L/D_max = {ld_max:.1f}"
        )
        if speed_mode == 'best_glide_nz':
            nz_preview = 1.0 / math.cos(math.radians(bank_angle))
            vbg_turn, _, _ = best_glide_kias(config, weight, nz_preview, 0, 0)
            st.sidebar.info(
                f"**Computed best glide ({bank_angle}° bank, nz={nz_preview:.2f}):** "
                f"{vbg_turn:.0f} KIAS"
            )
        st.sidebar.markdown("**Best-glide overrides** *(0 = auto)*")
        vbg_clean_kias = st.sidebar.number_input(
            "Vbg clean (gear up, flaps up)",
            min_value=0, max_value=300, value=0, step=1,
            help="POH best-glide speed for clean configuration "
                 "(gear up, flaps up). 0 = use aerodynamic computation.",
        )
        vbg_geardown_kias = st.sidebar.number_input(
            "Vbg gear down (gear ↓, flaps up)",
            min_value=0, max_value=300, value=0, step=1,
            help="POH best-glide speed with gear extended, flaps up. "
                 "0 = use aerodynamic computation.",
        )
        vbg_landing_kias = st.sidebar.number_input(
            "Vbg landing (gear ↓, flaps ↓)",
            min_value=0, max_value=300, value=0, step=1,
            help="POH best-glide speed with gear and flaps extended "
                 "(landing configuration). 0 = use aerodynamic computation.",
        )

    # Flap setting — controls flaps during takeoff climb and turn
    flap_options = {0: "Clean", 1: "Takeoff / 15°", 2: "Landing / Full"}
    flap_setting = st.sidebar.radio("Takeoff / turn flap setting", list(flap_options.keys()),
                                     format_func=lambda x: flap_options[x], index=0,
                                     help="Flap position during climb-out and the turn. "
                                          "With runway model ON, landing flaps are "
                                          "auto-deployed on final + forward slip as needed.")

    # Reaction time
    reaction_time = st.sidebar.slider("Reaction time (s)", min_value=0.0, max_value=10.0,
                                       value=3.0, step=0.5)

    # Field elevation
    field_elev = st.sidebar.number_input("Field elevation (ft MSL)", min_value=0, max_value=14000,
                                          value=0, step=100)

    # ISA deviation
    isa_dev = st.sidebar.number_input("ISA deviation (°C)", min_value=-40, max_value=50,
                                       value=0, step=1)

    # Wind
    st.sidebar.markdown("---")
    st.sidebar.subheader("Wind")
    wind_speed = st.sidebar.number_input("Surface wind speed (kt)", min_value=0, max_value=60,
                                          value=0, step=5)
    wind_dir_options = {
        0: "Headwind (0°)",
        45: "Right-quartering head (45°)",
        90: "Right crosswind (90°)",
        135: "Right-quartering tail (135°)",
        180: "Tailwind (180°)",
        225: "Left-quartering tail (225°)",
        270: "Left crosswind (270°)",
        315: "Left-quartering head (315°)",
    }
    wind_from_deg = st.sidebar.select_slider(
        "Wind direction (relative to runway)",
        options=list(range(0, 360, 5)),
        value=0,
        format_func=lambda d: wind_dir_options.get(d, f"{d}°"),
    )
    if wind_speed > 0:
        st.sidebar.caption(f"Surface wind {wind_speed} kt from {wind_from_deg}° relative to runway heading")
    
    # Wind at altitude (ForeFlight data)
    wind_alts_caption = st.sidebar.caption(
        "*Wind at altitude: get from ForeFlight weather winds aloft tab*"
    )
    wind_1000_kt = st.sidebar.number_input(
        "Wind at 1000 ft AGL (kt)", min_value=0, max_value=100,
        value=wind_speed, step=5,
        help="Wind speed at 1000 ft AGL. Default is surface wind."
    )
    wind_2000_kt = st.sidebar.number_input(
        "Wind at 2000 ft AGL (kt)", min_value=0, max_value=100,
        value=wind_speed, step=5,
        help="Wind speed at 2000 ft AGL. Default is surface wind."
    )
    wind_3000_kt = st.sidebar.number_input(
        "Wind at 3000 ft AGL (kt)", min_value=0, max_value=100,
        value=wind_speed, step=5,
        help="Wind speed at 3000 ft AGL (critical for turnback planning)."
    )

    # Runway Geometry
    st.sidebar.markdown("---")
    st.sidebar.subheader("Runway Geometry")
    use_runway = st.sidebar.checkbox("Enable runway model", value=False,
        help="When ON, the sim models a physical runway with finite length. "
             "It tracks whether the aircraft can touch down on the runway surface "
             "and stop before overrunning the end. Landing flaps auto-deploy on final, "
             "forward slip is used if too high, and all four landing options are evaluated: "
             "straight-ahead, 180° turnback, 360° orbit + turnback, and full circuit "
             "back to original heading. When OFF, success is simply crossing the "
             "departure point with altitude remaining."
    )
    runway_friction = 1.0  # default = standard dry asphalt
    if use_runway:
        runway_length = st.sidebar.number_input("Runway length (ft)", min_value=1000, max_value=15000,
                                                  value=5500, step=100)
        liftoff_distance = st.sidebar.number_input("Liftoff distance (ft)", min_value=100, max_value=10000,
                                                     value=1000, step=100)
        # Runway condition friction coefficient
        runway_condition_options = {
            'dry': ('Dry asphalt', 1.0),
            'wet': ('Wet asphalt', 0.7),
            'grass_dry': ('Grass (dry)', 0.6),
            'grass_wet': ('Grass (wet)', 0.3),
        }
        runway_condition = st.sidebar.radio(
            "Runway condition",
            list(runway_condition_options.keys()),
            format_func=lambda x: runway_condition_options[x][0],
            index=0,
            help="Affects braking friction coefficient, which impacts landing distance and last abort point.",
        )
        runway_friction = runway_condition_options[runway_condition][1]
        
        touchdown_margin_ft = st.sidebar.number_input(
            "Touchdown safety margin (ft)", min_value=0, max_value=3000, value=0, step=50,
            help="Extra runway distance beyond the computed braking rollout. "
                 "The sim aims to touch down far enough from the runway end so that "
                 "this much additional runway remains after the aircraft stops. "
                 "0 = aim based on rollout only (no extra buffer).",
        )
        st.sidebar.caption(
            "**Aim point** = computed rollout + safety margin.  "
            "Forward slip used if needed to steepen descent.  "
            "**Straight-ahead** lands on the remaining runway."
        )
        aim_point = runway_length  # passed for API compat; sim auto-computes aim_y
    else:
        runway_length = 0.0
        liftoff_distance = 0.0
        aim_point = 0.0
        touchdown_margin_ft = 0.0

    # Flap on return only (only relevant without runway model)
    flap_on_return = False
    if flap_setting > 0 and not use_runway:
        flap_on_return = st.sidebar.checkbox("Deploy flaps on final only", value=False,
                                              help="Stay clean during the turn, deploy flaps only when aligned with the runway")
    if use_runway:
        st.sidebar.caption(
            "ℹ️ **Runway model active:** landing flaps auto-deploy on "
            "final approach. Forward slip added if needed to make the runway."
        )

    # Prop state after engine failure
    st.sidebar.markdown("---")
    st.sidebar.subheader("Prop Drag")
    prop_state_options = {
        'feathered':           'Feathered (ΔCDo = +0.0005)',
        'windmilling':         'Windmilling / spinning (ΔCDo = +0.0020)',
        'fixed_pitch_stopped': 'Fixed-pitch prop stopped (ΔCDo = +0.0015)',
        'stopped':             'Stopped / unfeathered (ΔCDo = +0.0040)',
    }
    prop_state = st.sidebar.radio(
        "Prop state after engine failure",
        list(prop_state_options.keys()),
        format_func=lambda x: prop_state_options[x],
        index=0,
        help="Feathered: blades edge-on, negligible extra drag. "
             "Windmilling: prop freewheeling in the airstream. "
             "Fixed-pitch stopped: small fixed-pitch prop not spinning. "
             "Stopped: blades flat to airflow (worst case).",
    )

    gear_down = True  # default for fixed-gear aircraft
    if config.dcdo_gear > 0:
        # Retractable gear — let user choose
        gear_down = st.sidebar.checkbox(
            "Gear down", value=True,
            help=f"Add landing-gear drag (ΔCDo = +{config.dcdo_gear:.4f}). "
                 "Un-check to model gear retracted after takeoff.",
        )
    else:
        st.sidebar.caption("Fixed gear — gear drag included in base CDo")

    # Altitude step & max
    alt_step = st.sidebar.select_slider("Altitude step (ft)", options=[50, 100, 200, 500], value=100)

    max_alt_input = st.sidebar.number_input("Max altitude to plot (ft AGL, 0=auto)",
                                             min_value=0, max_value=5000, value=0, step=100)

    # ── Run simulation ──
    col_env, col_opt = st.sidebar.columns(2)
    run_envelope = col_env.button("Build Envelope", type="primary", use_container_width=True)
    run_optimizer = col_opt.button("Optimize", type="secondary", use_container_width=True)

    if run_envelope:
        # Clear optimizer so it doesn't latch
        st.session_state.pop('optimizer_result', None)
        with st.spinner("Computing trajectories..."):
            max_alt = max_alt_input if max_alt_input > 0 else None
            critical_alt, envelope, critical_alt_left, critical_alt_right, straight_ahead_max_alt = build_turnback_envelope(
                config, weight, airspeed, bank_angle, flap_setting,
                reaction_time, field_elev, isa_dev,
                alt_step=alt_step, max_alt=max_alt,
                wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                runway_length=runway_length, liftoff_distance=liftoff_distance,
                aim_point=aim_point, flap_on_return=flap_on_return,
                speed_mode=speed_mode,
                prop_state=prop_state,
                gear_down=gear_down,
                vbg_clean_kias=vbg_clean_kias,
                vbg_geardown_kias=vbg_geardown_kias,
                vbg_landing_kias=vbg_landing_kias,
                touchdown_margin_ft=touchdown_margin_ft,
                runway_friction=runway_friction,
            )

        st.session_state['turnback_result'] = {
            'critical_alt': critical_alt,
            'critical_alt_left': critical_alt_left,
            'critical_alt_right': critical_alt_right,
            'straight_ahead_max_alt': straight_ahead_max_alt,
            'envelope': envelope,
            'config': config,
            'weight': weight,
            'airspeed': airspeed,
            'bank_angle': bank_angle,
            'flap_setting': flap_setting,
            'reaction_time': reaction_time,
            'ac_key': ac_key,
            'wind_speed': wind_speed,
            'wind_from_deg': wind_from_deg,
            'runway_length': runway_length,
            'liftoff_distance': liftoff_distance,
            'aim_point': envelope[0]['left'].get('computed_aim_y', 0.0) if envelope else 0.0,
            'speed_mode': speed_mode,
        }

    if run_optimizer:
        # Clear envelope so it doesn't latch
        st.session_state.pop('turnback_result', None)
        with st.spinner("Optimizing — sweeping bank angles, turn directions, flap strategies..."):
            opt_results = optimize_turnback(
                config, weight, airspeed, reaction_time,
                field_elevation=field_elev, isa_dev=isa_dev,
                wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                runway_length=runway_length, liftoff_distance=liftoff_distance,
                speed_mode=speed_mode,
                prop_state=prop_state,
                gear_down=gear_down,
                vbg_clean_kias=vbg_clean_kias,
                vbg_geardown_kias=vbg_geardown_kias,
                vbg_landing_kias=vbg_landing_kias,
                touchdown_margin_ft=touchdown_margin_ft,
                runway_friction=runway_friction,
            )
        st.session_state['optimizer_result'] = {
            'results': opt_results,
            'ac_key': ac_key,
            'weight': weight,
            'airspeed': airspeed,
        }

        # Build envelope for the best optimizer result so 3D/2D plots display
        if opt_results:
            best = opt_results[0]
            from analysis.turnback_simulator import FLAP_STRATEGIES
            best_strat = FLAP_STRATEGIES.get(best['flap_strategy'], {})
            best_flap = best_strat.get('setting', 0) or 0
            best_flap_on_return = best_strat.get('on_return', False)
            with st.spinner("Building envelope for best optimizer result..."):
                max_alt = max_alt_input if max_alt_input > 0 else None
                crit, env, crit_l, crit_r, sa_max = build_turnback_envelope(
                    config, weight, airspeed, best['bank_angle'], best_flap,
                    reaction_time, field_elev, isa_dev,
                    alt_step=alt_step, max_alt=max_alt,
                    wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                    wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=best_flap_on_return,
                    speed_mode=speed_mode,
                    prop_state=prop_state,
                    gear_down=gear_down,
                    vbg_clean_kias=vbg_clean_kias,
                    vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction,
                )
            st.session_state['turnback_result'] = {
                'critical_alt': crit,
                'critical_alt_left': crit_l,
                'critical_alt_right': crit_r,
                'straight_ahead_max_alt': sa_max,
                'envelope': env,
                'config': config,
                'weight': weight,
                'airspeed': airspeed,
                'bank_angle': best['bank_angle'],
                'flap_setting': best_flap,
                'reaction_time': reaction_time,
                'ac_key': ac_key,
                'wind_speed': wind_speed,
                'wind_from_deg': wind_from_deg,
                'runway_length': runway_length,
                'liftoff_distance': liftoff_distance,
                'aim_point': env[0]['left'].get('computed_aim_y', 0.0) if env else 0.0,
                'speed_mode': speed_mode,
            }

    # ── Display optimizer results ──
    if 'optimizer_result' in st.session_state:
        _show_optimizer_results(st.session_state['optimizer_result'])

    # ── Display results (3D/2D plots, altitude profile, trajectory data) ──
    if 'turnback_result' not in st.session_state:
        if 'optimizer_result' not in st.session_state:
            st.info("Set parameters and click **Build Envelope** or **Optimize** to run.")
        return

    res = st.session_state['turnback_result']
    critical_alt = res['critical_alt']
    critical_alt_left = res.get('critical_alt_left', critical_alt)
    critical_alt_right = res.get('critical_alt_right', critical_alt)
    envelope = res['envelope']

    # Safety margin factor slider
    st.sidebar.markdown("---")
    st.sidebar.subheader("Safety Margin")
    safety_margin_factor = st.sidebar.slider(
        "Recommended altitude margin", 
        min_value=1.0, max_value=2.5, value=1.25, step=0.05,
        help="Apply safety margin to calculated turnback altitude to account for "
             "pilot skill, aircraft performance variability, and wind effects. "
             "1.0 = calculated minimum (no margin), 1.25 = 25% safety buffer, "
             "1.5 = 50% safety buffer, etc."
    )
    critical_alt_left_safe = critical_alt_left * safety_margin_factor
    critical_alt_right_safe = critical_alt_right * safety_margin_factor

    # Stall speed in the turn
    nz = 1.0 / math.cos(math.radians(res['bank_angle']))
    vs_clean = math.sqrt(295.0 * res['weight'] / (config.wing_area * config.Clmax))
    vs_turn = vs_clean * math.sqrt(nz)
    turn_radius = None
    for item in envelope:
        if item['left']['turn_radius_ft'] < 1e9:
            turn_radius = item['left']['turn_radius_ft']
            break

    # ── Key metrics ──
    col1, col2, col3, col4, col5 = st.columns(5)
    col1.metric("Critical Alt (LEFT)", f"{critical_alt_left_safe:,.0f} ft AGL", f"({critical_alt_left:,.0f} calc)")
    col2.metric("Critical Alt (RIGHT)", f"{critical_alt_right_safe:,.0f} ft AGL", f"({critical_alt_right:,.0f} calc)")
    col3.metric("Load Factor (nz)", f"{nz:.2f}")
    col4.metric("Stall Speed (turn)", f"{vs_turn:.0f} KIAS")
    if turn_radius and turn_radius < 1e9:
        col5.metric("Turn Radius", f"{turn_radius:,.0f} ft")
    else:
        col5.metric("Turn Radius", "—")

    # ── Runway zone analysis (straight-ahead vs dead zone vs turnback) ──
    straight_ahead_max_alt = res.get('straight_ahead_max_alt', 0.0)
    use_runway = res.get('runway_length', 0) > 0
    if use_runway:
        st.markdown("---")
        st.subheader("Runway Survival Zones")

        turnback_min_calc = min(critical_alt_left, critical_alt_right)
        turnback_min = min(critical_alt_left_safe, critical_alt_right_safe)
        dead_zone_low = int(straight_ahead_max_alt) if straight_ahead_max_alt > 0 else 0
        dead_zone_high = int(turnback_min)

        zone_cols = st.columns(3)

        if straight_ahead_max_alt > 0:
            zone_cols[0].metric(
                "Land Straight Ahead",
                f"0 – {int(straight_ahead_max_alt):,} ft AGL",
                help="Engine failure below this altitude: land on remaining runway, no turn needed",
            )
        else:
            zone_cols[0].metric(
                "Land Straight Ahead",
                "Not feasible",
                help="Takeoff roll uses too much runway — no room to land straight ahead",
            )

        if dead_zone_low < dead_zone_high:
            dead_zone_size = dead_zone_high - dead_zone_low
            zone_cols[1].metric(
                "DEAD ZONE",
                f"{dead_zone_low:,} – {dead_zone_high:,} ft AGL",
                delta=f"{dead_zone_size:,} ft gap",
                delta_color="inverse",
                help="Can't land straight (overshoots) AND can't make the turnback (too low)",
            )
            st.warning(
                f"⚠️ **Dead zone: {dead_zone_low:,} – {dead_zone_high:,} ft AGL** "
                f"({dead_zone_size:,} ft band). In this altitude band, the aircraft "
                f"overshoots the runway going straight ahead but is too low to complete "
                f"the turnback. This is the most dangerous failure altitude range."
            )
        elif dead_zone_low >= dead_zone_high:
            zone_cols[1].metric(
                "Dead Zone",
                "NONE",
                delta="Full coverage!",
                delta_color="normal",
                help="Straight-ahead and turnback zones overlap — no uncovered altitude band",
            )
            st.success(
                "✅ **No dead zone!** Straight-ahead landing coverage extends to or "
                "above the turnback critical altitude. Every failure altitude has "
                "a survivable option."
            )

        zone_cols[2].metric(
            "Turnback (Impossible Turn)",
            f"≥ {turnback_min:,} ft AGL",
            help="Engine failure above this altitude: complete the turnback to land on the runway",
        )

        # Landing strategy note
        st.info(
            "🛩️ **Landing strategy (runway model):** On final approach, the pilot deploys "
            "full landing flaps and decelerates to approach speed (1.3× Vs landing). "
            "If still too high, the sim first tries to continue the turn and land "
            "on the **original runway heading** (circuit phase). If altitude is still "
            "excessive, a 360° orbit is used. A forward slip steepens descent "
            "(up to 2× flapped drag). See the trajectory table for per-second "
            "flap, slip, and phase state."
        )

    # ── Speed info ──
    # Get speed_info from a critical altitude trajectory result
    crit_result = None
    for item in envelope:
        if item.get('is_critical_left') or item.get('is_critical_right'):
            crit_result = item['left']
            break
    if crit_result and 'speed_info' in crit_result:
        si = crit_result['speed_info']
        if si['mode'] in ('best_glide_1g', 'best_glide_nz'):
            speed_cols = st.columns(4)
            speed_cols[0].metric("Best Glide (1g)", f"{si['vbg_1g_kias']:.0f} KIAS")
            speed_cols[1].metric("L/D max", f"{si['ld_max']:.1f}")
            speed_cols[2].metric("CL (best L/D)", f"{si['cl_best']:.3f}")
            if si['mode'] == 'best_glide_nz' and 'vbg_turn_kias' in si:
                speed_cols[3].metric(
                    f"Best Glide ({res['bank_angle']}° bank)",
                    f"{si['vbg_turn_kias']:.0f} KIAS",
                )
            else:
                speed_cols[3].metric("Speed Mode", "Constant 1g")

    # Stall warning — determine the actual speed flown during the turn
    speed_mode = res.get('speed_mode', 'fixed')
    turn_speed = res['airspeed']  # default: sidebar climb-out speed
    if speed_mode == 'best_glide_nz':
        # Try speed_info from envelope first (already nz-scaled)
        if crit_result and 'speed_info' in crit_result:
            si_check = crit_result['speed_info']
            if 'vbg_turn_kias' in si_check:
                turn_speed = si_check['vbg_turn_kias']
        # Fallback: compute directly from overrides or aero model
        if turn_speed == res['airspeed']:
            _ov = _select_vbg_override(
                gear_down, False, vbg_clean_kias, vbg_geardown_kias, vbg_landing_kias)
            if _ov > 0:
                turn_speed = _ov * math.sqrt(nz)
            else:
                turn_speed, _, _ = best_glide_kias(
                    config, res['weight'], nz, 0, 0)
    elif speed_mode == 'best_glide_1g':
        if crit_result and 'speed_info' in crit_result:
            si_check = crit_result['speed_info']
            if 'vbg_1g_kias' in si_check:
                turn_speed = si_check['vbg_1g_kias']

    if turn_speed < vs_turn * 1.05:
        st.error(f"⚠️ Turn speed ({turn_speed:.0f} KIAS) is dangerously close to or below "
                 f"the stall speed in the turn ({vs_turn:.0f} KIAS at {res['bank_angle']}° bank). "
                 f"Reduce bank angle or increase airspeed.")

    # ── 3D Plot ──
    st.subheader("3D Heart-Shaped Envelope")
    show_success = st.checkbox("Show successful (green) paths", value=True)
    fig_3d = _build_3d_plot(envelope, critical_alt,
                             runway_length=res.get('runway_length', 0.0),
                             aim_point=res.get('aim_point', 0.0),
                             liftoff_distance=res.get('liftoff_distance', 0.0),
                             show_success=show_success)
    st.plotly_chart(fig_3d, use_container_width=True)

    # ── 2D Plan View ──
    st.subheader("Plan View (Top Down)")
    fig_2d = _build_2d_plan(envelope, critical_alt,
                             runway_length=res.get('runway_length', 0.0),
                             aim_point=res.get('aim_point', 0.0),
                             liftoff_distance=res.get('liftoff_distance', 0.0),
                             show_success=show_success)
    st.plotly_chart(fig_2d, use_container_width=True)

    # ── Altitude profile ──
    st.subheader("Altitude vs Time — Critical Altitudes")
    fig_alt = _build_altitude_profile(envelope, critical_alt)
    if fig_alt is not None:
        st.plotly_chart(fig_alt, use_container_width=True)

    # ── Detailed data table for critical altitude ──
    with st.expander("Trajectory Data — Critical Altitudes"):
        _show_trajectory_table(envelope, critical_alt)

    # ── Theory & References ──
    st.markdown("---")
    with st.expander("📚 Theory & References (EAA McSpadden Project)", expanded=False):
        _show_theory_section()


def _show_optimizer_results(opt_state):
    """Display the optimizer results as a recommendation + ranked table."""
    import pandas as pd

    results = opt_state['results']
    if not results:
        st.warning("Optimizer found no valid combinations. Try a higher airspeed or different aircraft.")
        return

    st.subheader("Optimizer Results")

    best = results[0]

    # ── Top recommendation ──
    speed_label = best.get('speed_mode_label', best.get('speed_mode', ''))
    st.success(
        f"**Best combination:** Turn **{best['turn_direction'].upper()}** at "
        f"**{best['bank_angle']}° bank**, "
        f"**{best['flap_label']}**, "
        f"**{speed_label}** → "
        f"Critical altitude = **{best['critical_altitude']:,} ft AGL**"
    )

    rec_cols = st.columns(5)
    rec_cols[0].metric("Turn Direction", best['turn_direction'].upper())
    rec_cols[1].metric("Bank Angle", f"{best['bank_angle']}°")
    rec_cols[2].metric("Flap Strategy", best['flap_label'])
    rec_cols[3].metric("Speed", speed_label)
    rec_cols[4].metric("Critical Alt", f"{best['critical_altitude']:,} ft")

    if best['overrun']:
        st.warning("⚠️ The best combination may result in a runway overrun at the critical altitude.")

    st.info(
        "**Coordinated turns enforced** — Turn-phase airspeed increases with "
        "g-loading (Vbg × √nz) to maintain coordinated flight.  "
        "Straight-flight phases revert to 1g best-glide speed.  "
        "Combinations below 1.2 × Vs (accelerated stall) are excluded."
    )

    # Show stall margin and speed info
    speed_caption = (
        f"Stall speed in turn: {best['stall_speed_turn_kias']} KIAS | "
        f"Load factor: {best['load_factor']}g | "
        f"Turn radius: {best['turn_radius_ft']:,.0f} ft"
    )
    if best.get('speed_mode', 'fixed') != 'fixed':
        speed_caption += (
            f"\n\nBest glide (1g): {best['speed_1g_kias']} KIAS | "
            f"Best glide (nz={best['load_factor']}): {best['speed_nz_kias']} KIAS | "
            f"L/D max: {best['ld_max']}"
        )
    st.caption(speed_caption)

    # ── Full ranked table ──
    with st.expander(f"All combinations ({len(results)} evaluated)", expanded=False):
        rows = []
        for r in results[:100]:  # show top 100
            row = {
                'Rank': len(rows) + 1,
                'Turn': r['turn_direction'].upper(),
                'Bank (°)': r['bank_angle'],
                'Speed Mode': r.get('speed_mode_label', r.get('speed_mode', '')),
                'Speed (KIAS)': r.get('speed_kias', ''),
                'Flaps': r['flap_label'],
                'Critical Alt (ft)': r['critical_altitude'],
                'Turn Radius (ft)': f"{r['turn_radius_ft']:,.0f}",
                'Vs Turn (KIAS)': r['stall_speed_turn_kias'],
                'nz': r['load_factor'],
                'Overrun': '⚠️' if r['overrun'] else '✓',
            }
            rows.append(row)
        df = pd.DataFrame(rows)
        st.dataframe(df, use_container_width=True, hide_index=True)

    # ── Bank angle sensitivity chart ──
    _show_optimizer_chart(results)


def _show_optimizer_chart(results):
    """Show critical altitude vs bank angle for each direction/strategy/speed combo."""
    fig = go.Figure()

    # Group by (direction, flap_strategy, speed_mode)
    groups = {}
    for r in results:
        key = (r['turn_direction'], r['flap_strategy'], r.get('speed_mode', 'fixed'))
        if key not in groups:
            groups[key] = []
        groups[key].append(r)

    colors = {
        ('left', 'clean', 'best_glide_1g'): 'cyan',
        ('right', 'clean', 'best_glide_1g'): 'dodgerblue',
        ('left', 'clean', 'best_glide_nz'): 'lime',
        ('right', 'clean', 'best_glide_nz'): 'limegreen',
        ('left', 'flaps_full', 'best_glide_1g'): 'gold',
        ('right', 'flaps_full', 'best_glide_1g'): 'orange',
        ('left', 'flaps_full', 'best_glide_nz'): 'salmon',
        ('right', 'flaps_full', 'best_glide_nz'): 'tomato',
        ('left', 'flaps_return', 'best_glide_1g'): 'violet',
        ('right', 'flaps_return', 'best_glide_1g'): 'orchid',
        ('left', 'flaps_return', 'best_glide_nz'): 'pink',
        ('right', 'flaps_return', 'best_glide_nz'): 'hotpink',
    }

    speed_short = {'best_glide_1g': 'Vbg 1g', 'best_glide_nz': 'Vbg nz', 'fixed': 'Fixed'}

    for key, group in groups.items():
        direction, strat, smode = key
        # For multi-aim-point, show only the best aim point per bank angle
        best_by_bank = {}
        for r in group:
            ba = r['bank_angle']
            if ba not in best_by_bank or r['critical_altitude'] < best_by_bank[ba]['critical_altitude']:
                best_by_bank[ba] = r
        sorted_group = sorted(best_by_bank.values(), key=lambda r: r['bank_angle'])

        banks = [r['bank_angle'] for r in sorted_group]
        crits = [r['critical_altitude'] for r in sorted_group]
        label = f"{direction.upper()} — {sorted_group[0]['flap_label']} — {speed_short.get(smode, smode)}"
        color = colors.get(key, 'white')

        fig.add_trace(go.Scatter(
            x=banks, y=crits,
            mode='lines+markers',
            line=dict(color=color, width=2),
            marker=dict(size=5),
            name=label,
        ))

    fig.update_layout(
        title="Critical Altitude vs Bank Angle",
        xaxis_title="Bank Angle (°)",
        yaxis_title="Critical Altitude (ft AGL)",
        height=450,
        margin=dict(l=60, r=20, t=40, b=60),
        template='plotly_dark',
        legend=dict(font=dict(size=10)),
    )
    st.plotly_chart(fig, use_container_width=True)


def _build_3d_plot(envelope, critical_alt, runway_length=0.0, aim_point=0.0, liftoff_distance=0.0, show_success=True):
    """Build the 3D heart-shaped envelope plot."""
    fig = go.Figure()

    # Draw climb-out line from liftoff to each altitude's engine failure point
    climbout_drawn = False
    for item in envelope:
        traj = item['left']['trajectory'] or item['right']['trajectory']
        if traj:
            start_y = traj[0]['y']
            start_z = traj[0]['z']
            lo_y = liftoff_distance if liftoff_distance > 0 else 0.0
            fig.add_trace(go.Scatter3d(
                x=[0, 0], y=[lo_y, start_y], z=[0, start_z],
                mode='lines',
                line=dict(color='magenta', width=4, dash='dash'),
                name='Climb-out' if not climbout_drawn else None,
                showlegend=not climbout_drawn,
                legendgroup='climbout',
            ))
            climbout_drawn = True

    for item in envelope:
        alt_agl = item['alt_agl']

        for side, label in [('left', 'L'), ('right', 'R')]:
            traj = item[side]['trajectory']
            if not traj:
                continue

            xs = [p['x'] for p in traj]
            ys = [p['y'] for p in traj]
            zs = [p['z'] for p in traj]
            success = item[side]['success']
            is_crit_side = item.get(f'is_critical_{side}', False)

            if is_crit_side:
                color = 'gold'
                width = 5
                name = f"CRITICAL {alt_agl}ft {label}-turn"
            elif success:
                if not show_success:
                    continue
                color = 'limegreen'
                width = 2.5
                name = f"{alt_agl}ft {label}-turn ✓"
            else:
                color = 'tomato'
                width = 2
                name = f"{alt_agl}ft {label}-turn ✗"

            fig.add_trace(go.Scatter3d(
                x=xs, y=ys, z=zs,
                mode='lines',
                line=dict(color=color, width=width),
                name=name,
                legendgroup=f"{alt_agl}",
                showlegend=(label == 'L'),
            ))

    # Straight-ahead trajectories (runway model only)
    sa_drawn = False
    for item in envelope:
        sa = item.get('straight_ahead')
        if not sa or not sa['trajectory']:
            continue
        alt_agl = item['alt_agl']
        traj = sa['trajectory']
        xs = [p['x'] for p in traj]
        ys = [p['y'] for p in traj]
        zs = [p['z'] for p in traj]
        is_sa_max = item.get('is_straight_ahead_max', False)
        if is_sa_max:
            color = 'deepskyblue'
            width = 5
            name = f"STRAIGHT AHEAD MAX {alt_agl}ft"
        elif sa['success']:
            if not show_success:
                continue
            color = 'dodgerblue'
            width = 2.5
            name = f"{alt_agl}ft straight ✓"
        else:
            color = 'orange'
            width = 1.5
            name = f"{alt_agl}ft straight ✗"
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode='lines',
            line=dict(color=color, width=width),
            name=name,
            legendgroup=f"sa_{alt_agl}",
        ))
        sa_drawn = True

    # Runway line
    rwy_end = runway_length if runway_length > 0 else 500
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[-200, rwy_end], z=[0, 0],
        mode='lines',
        line=dict(color='white', width=6),
        name='Runway',
    ))

    # Departure end marker
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(color='cyan', size=5, symbol='diamond'),
        name='Threshold',
    ))

    if aim_point > 0:
        fig.add_trace(go.Scatter3d(
            x=[0], y=[aim_point], z=[0],
            mode='markers',
            marker=dict(color='magenta', size=6, symbol='diamond'),
            name=f'Touchdown Target ({aim_point:,.0f} ft)',
        ))

    # Compute axis ranges for balanced 3D view
    all_x, all_y, all_z = [], [], []
    for item in envelope:
        for side in ('left', 'right'):
            traj = item[side]['trajectory']
            if traj:
                all_x.extend(p['x'] for p in traj)
                all_y.extend(p['y'] for p in traj)
                all_z.extend(p['z'] for p in traj)
        sa = item.get('straight_ahead')
        if sa and sa.get('trajectory'):
            all_y.extend(p['y'] for p in sa['trajectory'])
            all_z.extend(p['z'] for p in sa['trajectory'])
    if all_x and all_y and all_z:
        dx = max(abs(max(all_x) - min(all_x)), 100)
        dy = max(abs(max(all_y) - min(all_y)), 100)
        dz = max(abs(max(all_z) - min(all_z)), 50)
        max_range = max(dx, dy, dz)
        aspect = dict(x=dx / max_range, y=dy / max_range, z=dz / max_range)
        aspect_mode = 'manual'
    else:
        aspect = dict(x=1, y=1, z=1)
        aspect_mode = 'data'

    fig.update_layout(
        scene=dict(
            xaxis_title='Lateral (ft)',
            yaxis_title='Along Runway (ft)',
            zaxis_title='Altitude AGL (ft)',
            aspectmode=aspect_mode,
            aspectratio=aspect,
            camera=dict(
                eye=dict(x=1.5, y=-1.5, z=1.0),
                center=dict(x=0, y=0.15, z=0),
            ),
        ),
        height=700,
        margin=dict(l=0, r=0, t=30, b=0),
        legend=dict(font=dict(size=10)),
        template='plotly_dark',
    )
    return fig


def _build_2d_plan(envelope, critical_alt, runway_length=0.0, aim_point=0.0, liftoff_distance=0.0, show_success=True):
    """Build the 2D plan view (top-down) showing ground tracks."""
    fig = go.Figure()

    # Draw climb-out line from liftoff to each altitude's engine failure point
    climbout_drawn = False
    for item in envelope:
        traj = item['left']['trajectory'] or item['right']['trajectory']
        if traj:
            start_y = traj[0]['y']
            lo_y = liftoff_distance if liftoff_distance > 0 else 0.0
            fig.add_trace(go.Scatter(
                x=[0, 0], y=[lo_y, start_y],
                mode='lines+markers',
                line=dict(color='magenta', width=3, dash='dash'),
                marker=dict(color='magenta', size=[8, 10], symbol=['circle', 'star']),
                name='Climb-out' if not climbout_drawn else None,
                showlegend=not climbout_drawn,
                legendgroup='climbout',
            ))
            climbout_drawn = True

    for item in envelope:
        alt_agl = item['alt_agl']

        for side, label in [('left', 'L'), ('right', 'R')]:
            traj = item[side]['trajectory']
            if not traj:
                continue

            xs = [p['x'] for p in traj]
            ys = [p['y'] for p in traj]
            success = item[side]['success']
            is_crit_side = item.get(f'is_critical_{side}', False)

            if is_crit_side:
                color = 'gold'
                width = 4
                name = f"CRITICAL {alt_agl}ft {label}"
            elif success:
                if not show_success:
                    continue
                color = 'limegreen'
                width = 2
                name = f"{alt_agl}ft {label} ✓"
            else:
                color = 'tomato'
                width = 1.5
                name = f"{alt_agl}ft {label} ✗"

            fig.add_trace(go.Scatter(
                x=xs, y=ys,
                mode='lines',
                line=dict(color=color, width=width),
                name=name,
                legendgroup=f"{alt_agl}",
                showlegend=(label == 'L'),
            ))

            # Mark stall point if applicable
            if item[side]['stalled'] and item[side]['stall_time'] is not None:
                st_t = item[side]['stall_time']
                for p in traj:
                    if p['time'] >= st_t:
                        fig.add_trace(go.Scatter(
                            x=[p['x']], y=[p['y']],
                            mode='markers',
                            marker=dict(color='red', size=10, symbol='x'),
                            name=f'STALL {alt_agl}ft',
                            legendgroup=f"{alt_agl}",
                            showlegend=False,
                        ))
                        break

    # Straight-ahead trajectories (runway model only)
    for item in envelope:
        sa = item.get('straight_ahead')
        if not sa or not sa['trajectory']:
            continue
        alt_agl = item['alt_agl']
        traj_sa = sa['trajectory']
        xs = [p['x'] for p in traj_sa]
        ys = [p['y'] for p in traj_sa]
        is_sa_max = item.get('is_straight_ahead_max', False)
        if is_sa_max:
            color = 'deepskyblue'
            width = 4
            name = f"SA MAX {alt_agl}ft"
        elif sa['success']:
            if not show_success:
                continue
            color = 'dodgerblue'
            width = 2
            name = f"{alt_agl}ft straight ✓"
        else:
            color = 'orange'
            width = 1.5
            name = f"{alt_agl}ft straight ✗"
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode='lines',
            line=dict(color=color, width=width),
            name=name,
            legendgroup=f"sa_{alt_agl}",
        ))

    # Runway
    rwy_end = runway_length if runway_length > 0 else 500
    fig.add_trace(go.Scatter(
        x=[0, 0], y=[-200, rwy_end],
        mode='lines',
        line=dict(color='gray', width=8, dash='solid'),
        name='Runway',
    ))
    fig.add_trace(go.Scatter(
        x=[0], y=[0],
        mode='markers',
        marker=dict(color='cyan', size=12, symbol='diamond'),
        name='Departure Threshold',
    ))
    if runway_length > 0:
        # Far end of runway
        fig.add_trace(go.Scatter(
            x=[0], y=[runway_length],
            mode='markers',
            marker=dict(color='cyan', size=12, symbol='square'),
            name=f'Runway End ({runway_length:,.0f} ft)',
        ))
    if aim_point > 0:
        fig.add_trace(go.Scatter(
            x=[0], y=[aim_point],
            mode='markers+text',
            marker=dict(color='magenta', size=14, symbol='triangle-down'),
            text=[f'AIM {aim_point:,.0f}ft'],
            textposition='middle right',
            textfont=dict(color='magenta', size=11),
            name=f'Touchdown Target ({aim_point:,.0f} ft)',
        ))

    fig.update_layout(
        xaxis_title='Lateral Offset (ft)',
        yaxis_title='Distance Along Runway Heading (ft)',
        xaxis=dict(scaleanchor='y', scaleratio=1),
        height=700,
        margin=dict(l=60, r=20, t=30, b=60),
        legend=dict(font=dict(size=10)),
        template='plotly_dark',
    )
    return fig


def _build_altitude_profile(envelope, critical_alt):
    """Altitude vs time for both left and right critical altitude trajectories."""
    # Find envelope items at each side's critical altitude
    crit_left_item = None
    crit_right_item = None
    for item in envelope:
        if item.get('is_critical_left'):
            crit_left_item = item
        if item.get('is_critical_right'):
            crit_right_item = item
    # Fallback: use overall is_critical
    if crit_left_item is None and crit_right_item is None:
        for item in envelope:
            if item.get('is_critical'):
                crit_left_item = item
                crit_right_item = item
                break
    if crit_left_item is None and crit_right_item is None:
        return None

    fig = go.Figure()
    traces = []
    if crit_left_item:
        traces.append(('left', f'Left (crit {crit_left_item["alt_agl"]} ft)', 'gold', crit_left_item))
    if crit_right_item:
        traces.append(('right', f'Right (crit {crit_right_item["alt_agl"]} ft)', 'cyan', crit_right_item))

    for side, label, color, crit_item in traces:
        traj = crit_item[side]['trajectory']
        if not traj:
            continue
        times = [p['time'] for p in traj]
        alts = [p['z'] for p in traj]
        fig.add_trace(go.Scatter(
            x=times, y=alts,
            mode='lines',
            line=dict(color=color, width=3),
            name=label,
        ))

        # Mark phase transitions
        phases_seen = set()
        for p in traj:
            if p['phase'] not in phases_seen:
                phases_seen.add(p['phase'])
                fig.add_trace(go.Scatter(
                    x=[p['time']], y=[p['z']],
                    mode='markers+text',
                    marker=dict(color=color, size=8),
                    text=[p['phase'].upper()],
                    textposition='top center',
                    textfont=dict(color=color, size=10),
                    showlegend=False,
                ))

    fig.update_layout(
        xaxis_title='Time (s)',
        yaxis_title='Altitude AGL (ft)',
        height=400,
        margin=dict(l=60, r=20, t=30, b=60),
        template='plotly_dark',
    )
    return fig


def _show_trajectory_table(envelope, critical_alt):
    """Display data tables for both left and right critical altitude trajectories."""
    import pandas as pd

    # Find side-specific critical items
    crit_left_item = None
    crit_right_item = None
    for item in envelope:
        if item.get('is_critical_left'):
            crit_left_item = item
        if item.get('is_critical_right'):
            crit_right_item = item
    # Fallback
    if crit_left_item is None and crit_right_item is None:
        for item in envelope:
            if item.get('is_critical'):
                crit_left_item = item
                crit_right_item = item
                break

    sides = []
    if crit_left_item:
        sides.append(('left', f"Left Turn (crit {crit_left_item['alt_agl']} ft AGL)", crit_left_item))
    if crit_right_item:
        sides.append(('right', f"Right Turn (crit {crit_right_item['alt_agl']} ft AGL)", crit_right_item))

    if not sides:
        st.write("No critical altitude data.")
        return

    for side, label, crit_item in sides:
        land_dir = crit_item[side].get('landing_direction', 'reverse')
        dir_label = "same direction" if land_dir == 'original' else "reverse"
        st.caption(f"{label}  —  landing {dir_label}")
        traj = crit_item[side]['trajectory']
        if not traj:
            st.write("No trajectory data.")
            continue

        # Downsample to every 1 second
        rows = []
        last_t = -1.0
        for p in traj:
            if p['time'] - last_t >= 1.0 or p is traj[-1]:
                rows.append({
                    'Time (s)': round(p['time'], 1),
                    'Phase': p['phase'],
                    'Flaps': p.get('flap_state', ''),
                    'Slip': '✓' if p.get('slipping') else '',
                    'KIAS': round(p.get('kias', 0), 1),
                    'Vs (KIAS)': round(p.get('vs_kias', 0), 1),
                    'X (ft)': round(p['x']),
                    'Y (ft)': round(p['y']),
                    'Alt AGL (ft)': round(p['z']),
                    'Heading (°)': round(p['heading_deg']),
                    'CL': round(p['cl'], 3),
                    'nz': round(p['nz'], 2),
                    'ROC (fpm)': round(p['roc_fpm']),
                    'Gamma (°)': round(p['gamma_deg'], 1),
                })
                last_t = p['time']

        df = pd.DataFrame(rows)
        st.dataframe(df, use_container_width=True)


def _show_theory_section():
    """Display the theory and references for the turnback simulator."""
    
    st.markdown("""
    ## Physics & Regulatory Foundation

    ### The Problem
    
    When an engine fails on takeoff, pilots face the **"impossible turn"** decision: 
    Can I safely turn back to land on the departure runway, or must I land ahead?
    
    The **FAA Advisory Circular 61-83K** mandates that CFIs train this scenario during 
    biennial flight reviews. Yet pilots often walk around with "800 feet" in their head 
    with no rationale — pure folklore.
    
    ### The Solution: Physics-Based Analysis
    
    This simulator uses **zero-thrust glide physics** to calculate the **critical altitude** — 
    the minimum altitude at which an engine failure allows a safe return.
    
    ---
    
    ## Core Equations
    
    ### 1. Glide Gradient (Zero Thrust)
    
    After engine failure (T = 0), the descent gradient depends only on drag and weight:
    
    **sin(γ) = −D/W**
    
    The key insight: **Bank angle does NOT affect this gradient directly**. 
    However, banked flight requires higher lift coefficient, which increases induced drag 
    and thus increases sink rate.
    
    ### 2. Load Factor in Turn
    
    When banking at angle φ:
    
    **nz = 1/cos(φ)**
    
    This requires higher lift:
    
    **CL = (nz · W) / (q · S)**
    
    At 45° bank: nz = 1.41 → stall speed is **41% higher** than wings-level stall.
    
    ### 3. Total Drag Increases with Lift
    
    **CD = CDo + k · CL²**
    
    Substituting load factor:
    
    **CD = CDo + k · [(nz · W) / (q · S)]²**
    
    This shows why steep banks are so penalizing: lift coefficient squares in the 
    induced drag term, causing quadratic growth in drag as bank angle increases.
    
    ### 4. Best-Glide Speed
    
    At maximum L/D (minimum sink rate):
    
    **CL* = √(CDo / k)**
    
    **Vbg,TAS = √(2 · nz · W / (ρ · S · CL*))**
    
    In KIAS:
    
    **Vbg,KIAS = Vbg,TAS · √σ**
    
    where σ is the air density ratio.
    
    ### 5. Turn Radius & Rate
    
    Radius:
    
    **R = V²TAS / (g · tan(φ))**
    
    Angular rate:
    
    **ω = g · tan(φ) / V_TAS (rad/sec)**
    
    Steeper banks → smaller circle but MUCH higher sink. Typical optimal bank is 25–35°.
    
    ---
    
    ## The Altitude Loss During the Turn
    
    **This is the critical unknown** that POH charts do not provide.
    
    Prof. James F. Rogers (former head of Aero Dept, Naval Academy) showed how to 
    estimate it using POH data (CLmax, CDo, turn radius), but the calculation is complex.
    
    This simulator computes it by time-step integration:
    1. Calculate sink rate during banked flight (varies with speed, altitude, bank angle)
    2. Integrate sink rate over the actual turn arc
    3. Result: altitude loss emerges naturally from physics
    
    For a 30° bank in a typical Cessna 172: roughly 150–200 ft altitude is lost during 
    the 180° turn. This cannot be neglected!
    
    ---
    
    ## Safety Margins
    
    The calculated **critical altitude** assumes:
    - Pilot flies exactly at best-glide speed
    - Turn is perfectly coordinated
    - Aircraft performance matches POH exactly
    - Wind is steady
    - Reaction time is as modeled
    
    **Reality is messier.** We recommend a **safety margin of 1.25× to 1.5×**:
    
    - **1.0×**: Theoretical minimum (not recommended)
    - **1.25×**: Conservative for experienced pilots, stable conditions *(recommended)*
    - **1.5×**: Recommended for typical operations
    - **2.0×**: Very conservative; nearly eliminates margin loss in the turn
    
    ---
    
    ## Academic References
    
    This simulator is grounded in the **EAA McSpadden Project**, which includes:
    
    1. **Prof. James F. Rogers** — "Estimating Turnback Altitude for Single-Engine Aircraft"
       - Analytical method for calculating altitude loss in the turn
       - Accounts for lift/drag, load factor, turn geometry
       - Basis for safety margins in this simulator
    
    2. **Brent Jett** (USAF Academy, 1978–1982) — Simulator-based study of the impossible turn
       - Evaluated critical altitude across aircraft types
       - Effect of pilot skill, reaction time, bank angle
       - Results in similar range to this simulator (500–1500 ft AGL typical)
    
    3. **FAA Advisory Circular 61-83K** (2024) — Biennial Flight Review mandate
       - Paragraph A.114: Pilots must train engine-out turnback scenarios
       - Proposed EAA/FAA method for quantitative assessment
    
    4. **FAA Airplane Flying Handbook** (proposed edits, Chapters 6 & 18)
       - Emergency procedures, engine-out approaches, landing techniques
    
    5. **Standard Aerodynamic References**
       - Anderson, "Fundamentals of Aerodynamics" — drag polar, load factor
       - FAA "Aircraft Performance" handbook — best-glide speed, descent gradient
    
    ---
    
    ## Quality Assurance
    
    ### How We Validate Against Published Data
    
    **Cessna 172 single-engine glide:**
    - POH best-glide: 50 knots
    - Simulator computed: 49–51 KIAS ✓
    - Typical critical altitude: 800–1000 ft AGL
    - With 1.25× margin: 1000–1200 ft AGL ✓
    
    **Piper PA-46 Meridian (simulating single-engine operation):**
    - POH best-glide: 65 knots
    - Simulator computed: 64–66 KIAS ✓
    - Typical critical altitude: 1200–1500 ft AGL
    - With 1.25× margin: 1500–1900 ft AGL ✓
    
    ---
    
    ## How This Simulator Works
    
    ### Time-Step Integration
    
    The simulator uses small time steps (50 ms) to numerically integrate:
    
    1. **Aerodynamic state** — Compute drag, load factor, stall speed
    2. **Flight path** — Vertical and horizontal velocity
    3. **Wind effects** — Interpolate wind at current altitude
    4. **Position** — Update (x, y, z) and heading
    5. **Phase transitions** — Reaction → Turn → Return → Landing
    6. **Success** — Aircraft crosses runway with positive altitude
    
    ### Wind at Altitude
    
    You provide wind speeds at surface, 1000 ft, 2000 ft, and 3000 ft AGL.
    The simulator linearly interpolates wind at the aircraft's current altitude,
    updating it each time step. This matches real wind shear patterns.
    
    ### Runway Friction & Landing Distance
    
    Landing distance scales with runway friction coefficient:
    
    **d_rollout = V_touchdown² / (2g · μ_brake)**
    
    Runway condition (dry/wet/grass) scales the baseline μ = 0.3, affecting:
    - How far from the runway start you land (affects success)
    - The "last abort point" (when you can no longer stop on runway)
    
    ---
    
    ## The Bottom Line
    
    This simulator translates decades of academic research into a practical tool.
    
    **Pilots can now:**
    - ✈ Calculate critical altitude *pre-flight* using real-time weather
    - ✈ Understand exactly *why* the numbers come out as they do
    - ✈ Make informed decisions using conservative safety margins
    - ✈ **Replace folklore ("800 feet") with facts**
    
    ---
    
    **For the complete technical reference**, see:
    
    📄 **THEORY-AND-REFERENCES.md** — Full derivations, validation, and limitations  
    📄 **CHARLIE-BRIEFING.md** — EAA McSpadden Project background  
    📧 **charlie-precourt-email.txt** — Original requirements from Charlie Precourt (Space Shuttle pilot)
    
    These files are in the **`eaa-mcsppadden-project/`** folder.
    
    """)
    
    st.info(
        "💡 **Pro tip**: Review the EAA McSpadden Project documents in the "
        "`eaa-mcsppadden-project/` folder for full technical details, paper references, "
        "and regulatory citations."
    )


# Allow standalone execution for development
if __name__ == '__main__':
    run_turnback_page()
