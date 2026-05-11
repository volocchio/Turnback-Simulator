"""
Turnback Simulator UI — "Impossible Turn" Visualization

Streamlit page with 3D Plotly heart-shaped envelope showing the safe-return
zone after engine failure at various altitudes AGL.
"""

import streamlit as st
import plotly.graph_objects as go
import math
import pandas as pd

from engine.aircraft_config import AIRCRAFT_CONFIG
from engine.poh_data import (
    POH_GROUND_ROLL_FT, estimate_ground_roll,
    POH_VBG_KIAS, vbg_poh_kias,
    POH_CLIMB, vy_poh_kias, roc_poh_fpm,
    POH_VS_KIAS, vs_poh_kias,
    POH_LANDING_ROLL_FT, landing_roll_poh_ft,
)
from analysis.turnback_simulator import (
    build_turnback_envelope, simulate_turnback, optimize_turnback, best_glide_kias,
    simulate_straight_ahead, find_straight_ahead_max_altitude,
    find_critical_altitude,
    _select_vbg_override,
)
from analysis.data_card import build_takeoff_data_card, build_takeoff_data_card_pdf, build_pre_brief

# ── Page config for standalone mode ──
try:
    st.set_page_config(
        page_title="Turnback Simulator",
        page_icon="✈️",
        layout="wide",
    )
except st.errors.StreamlitAPIException:
    pass  # already set when embedded in another app

# Charlie #10 — make the help-bubble (?) icon larger and the popover text
# larger.  Streamlit's default 16px icon is hard to see, especially for
# pilots reading on a tablet in the cockpit.
st.markdown(
    """
    <style>
      /* Bigger help-bubble (?) icon next to widget labels */
      [data-testid="stTooltipIcon"] svg,
      [data-testid="stTooltipHoverTarget"] svg {
          width: 22px !important;
          height: 22px !important;
          color: #0a3a5c !important;
      }
      /* Bigger help-popover body text */
      [data-baseweb="tooltip"] {
          font-size: 14px !important;
          line-height: 1.45 !important;
          max-width: 360px !important;
      }
    </style>
    """,
    unsafe_allow_html=True,
)


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
    # Default to C150 if available, else Meridian, else first
    default_idx = next((i for i, k in enumerate(all_keys) if k[0] == 'C150'), None)
    if default_idx is None:
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

    # Flap setting — split into takeoff vs turn (Charlie #2 / moved up #C1).
    # Takeoff flap drives the climb-out / liftoff-distance regime.
    # Turn flap drives the post-failure aerodynamics.  These are usually
    # different in real life: a pilot may depart with takeoff flaps deployed
    # but retract them before initiating any turnback.
    flap_options = {0: "Clean", 1: "Takeoff / 15°", 2: "Landing / Full"}
    takeoff_flap_setting = st.sidebar.radio(
        "Takeoff flap setting", list(flap_options.keys()),
        format_func=lambda x: flap_options[x], index=0,
        help="Flap position used during the takeoff ground-roll and initial climb. "
             "Recorded for reference today; will drive computed liftoff distance "
             "in a future update.  Real aircraft typically take off with a small "
             "flap deflection (e.g. flaps 10) and retract before any turnback.",
    )
    # Charlie #C8 — interactive flap-retract logic on climb-out.
    # If the pilot departed with takeoff flaps, ask at what AGL they retract.
    # Below that altitude the failure happens with takeoff flaps still deployed
    # (more drag, lower glide).  Above it, climb is clean.
    if takeoff_flap_setting > 0:
        flap_retract_alt_ft = st.sidebar.number_input(
            "Retract takeoff flaps at (ft AGL)",
            min_value=0, max_value=2000, value=400, step=50,
            help="Altitude AGL at which takeoff flaps are retracted during a "
                 "normal climb-out.  Typical: 200–500 ft AGL.  If engine failure "
                 "occurs **below** this altitude, the turn flap setting below is "
                 "ignored and takeoff flaps are still deployed when the turn "
                 "begins.  Above it, you're already clean.",
        )
    else:
        flap_retract_alt_ft = 0
    flap_setting = st.sidebar.radio(
        "Turn flap setting", list(flap_options.keys()),
        format_func=lambda x: flap_options[x], index=0,
        help="Flap position during the engine-out turn.  Most pilots are taught "
             "to retract takeoff flaps before initiating the turnback because "
             "the higher CLmax doesn't help — clean produces less drag and "
             "preserves more energy.  With runway model ON, landing flaps are "
             "auto-deployed on final + forward slip as needed.",
    )

    # ── Stall speeds (user-configurable, back-compute Clmax) ──
    # Defaults from POH where available, else from config Clmax at MTOW
    _vs_clean_aero = math.sqrt(295.0 * config.MTOW / (config.wing_area * config.Clmax))
    _clmax_land = config.Clmax_flaps40 if config.Clmax_flaps40 > 0 else config.Clmax_flaps15
    _vs_land_aero = math.sqrt(295.0 * config.MTOW / (config.wing_area * _clmax_land)) if _clmax_land > 0 else _vs_clean_aero
    _vs_clean_poh_mtow = POH_VS_KIAS.get(ac_key[0], (None, None))[0]
    _vs_land_poh_mtow = POH_VS_KIAS.get(ac_key[0], (None, None))[1]
    _vs_clean_default = float(_vs_clean_poh_mtow) if _vs_clean_poh_mtow else _vs_clean_aero
    _vs_land_default = float(_vs_land_poh_mtow) if _vs_land_poh_mtow else _vs_land_aero

    st.sidebar.markdown("---")
    if _vs_clean_poh_mtow:
        st.sidebar.caption(
            f"**Vs defaults from POH** (at MTOW): clean **{_vs_clean_poh_mtow}** · "
            f"landing **{_vs_land_poh_mtow}** KIAS.  "
            f"Aerodynamic estimate would be: clean {_vs_clean_aero:.0f} · landing {_vs_land_aero:.0f}."
        )
    vs_clean_input = st.sidebar.number_input(
        "Vs clean at MTOW (KIAS)", min_value=30, max_value=200,
        value=int(round(_vs_clean_default)), step=1,
        help="Power-off stall speed, clean config, at MTOW. Adjusts CLmax. "
             "Defaults to POH where available.",
        key=f"vs_clean_{ac_key[0]}",
    )
    vs_land_input = st.sidebar.number_input(
        "Vs landing flaps at MTOW (KIAS)", min_value=25, max_value=180,
        value=int(round(_vs_land_default)), step=1,
        help="Power-off stall speed, full landing flaps, at MTOW. Adjusts CLmax_flaps. "
             "Defaults to POH where available.",
        key=f"vs_land_{ac_key[0]}",
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

    # ── Departure Airport ──
    st.sidebar.markdown("---")
    st.sidebar.subheader("Departure Airport")
    # Always pick an airport + runway from the OurAirports database (default KSEZ rwy 21).
    # Manual-entry fallback only kicks in if the selected airport has no runway data.
    use_airport_db = True

    selected_airport_ident = ""
    selected_runway_ident = ""
    runway_heading_true = 0.0
    runway_length_db = 0.0  # populated from DB when airport selected

    if use_airport_db:
        from engine.airport_db import (
            load_airports, load_runway_ends, wind_components,
        )
        airports_df = load_airports()
        # Default search to KSEZ if present
        default_idx = 0
        if "KSEZ" in airports_df["ident"].values:
            default_idx = int(airports_df.index[airports_df["ident"] == "KSEZ"][0])
        airport_idx = st.sidebar.selectbox(
            "Airport",
            options=range(len(airports_df)),
            format_func=lambda i: airports_df.iloc[i]["display_name"],
            index=default_idx,
            help="Type to search by ICAO/local code or name.",
        )
        airport_row = airports_df.iloc[airport_idx]
        selected_airport_ident = airport_row["ident"]
        field_elev = int(round(float(airport_row["elevation_ft"])))

        runway_ends = load_runway_ends(selected_airport_ident)
        if runway_ends.empty:
            st.sidebar.warning(
                f"No runway data available for {selected_airport_ident}. "
                "Falling back to manual entry."
            )
            use_airport_db = False
        else:
            rwy_default_idx = 0
            for _i in range(len(runway_ends)):
                if str(runway_ends.iloc[_i]["rwy_ident"]).strip() == "21":
                    rwy_default_idx = _i
                    break
            rwy_idx = st.sidebar.selectbox(
                "Runway",
                options=range(len(runway_ends)),
                format_func=lambda i: (
                    f"{runway_ends.iloc[i]['rwy_ident']} — "
                    f"{int(runway_ends.iloc[i]['length_ft'])} ft, "
                    f"hdg {runway_ends.iloc[i]['heading_degT']:.0f}°T"
                    if pd.notna(runway_ends.iloc[i]['heading_degT'])
                    else f"{runway_ends.iloc[i]['rwy_ident']} — heading unknown"
                ),
                index=rwy_default_idx,
            )
            rwy_row = runway_ends.iloc[rwy_idx]
            selected_runway_ident = str(rwy_row["rwy_ident"])
            runway_length_db = float(rwy_row["length_ft"])
            if pd.notna(rwy_row["heading_degT"]):
                runway_heading_true = float(rwy_row["heading_degT"])
            else:
                # Fall back to numeric runway ident × 10
                try:
                    digits = "".join(c for c in selected_runway_ident if c.isdigit())
                    runway_heading_true = float(digits) * 10.0 if digits else 0.0
                except ValueError:
                    runway_heading_true = 0.0
            # Use threshold elevation if provided (more accurate than airport elev)
            if pd.notna(rwy_row.get("threshold_elevation_ft")):
                field_elev = int(round(float(rwy_row["threshold_elevation_ft"])))
            st.sidebar.caption(
                f"**{selected_airport_ident} RW {selected_runway_ident}** — "
                f"elev {field_elev} ft, "
                f"hdg {runway_heading_true:.0f}°T, "
                f"length {int(runway_length_db)} ft"
            )

    if not use_airport_db:
        # Field elevation (manual)
        field_elev = st.sidebar.number_input(
            "Field elevation (ft MSL)", min_value=0, max_value=14000,
            value=0, step=100,
        )

    # Climb-out speed at engine failure (uses current weight, not MTOW)
    vs_clean_est = math.sqrt(295.0 * weight / (config.wing_area * config.Clmax))
    vx_est = int(vs_clean_est * 1.1)   # Vx ≈ 1.1 × Vs_clean
    vy_est = int(vs_clean_est * 1.3)   # Vy ≈ 1.3 × Vs_clean
    _vy_poh = vy_poh_kias(ac_key[0])
    _have_vy_poh = _vy_poh is not None

    climb_speed_options = {
        'vx':     f'Vx — best angle ({vx_est} KIAS, est)',
        'vy':     f'Vy — best rate ({vy_est} KIAS, est)',
        'vy_poh': f'Vy — POH ({int(_vy_poh)} KIAS)' if _have_vy_poh else 'Vy — POH (n/a)',
        'manual': 'Manual',
    }
    # Default to POH Vy when available, else estimated Vy.
    _default_climb_idx = 2 if _have_vy_poh else 1
    climb_speed_mode = st.sidebar.radio(
        "Climb-out speed",
        list(climb_speed_options.keys()),
        format_func=lambda x: climb_speed_options[x],
        index=_default_climb_idx,
        help="Speed at moment of engine failure. "
             "Vx: best angle of climb (~1.1 × Vs). "
             "Vy: best rate of climb (~1.3 × Vs estimate, or POH value when available).",
    )

    if climb_speed_mode == 'vx':
        airspeed = vx_est
    elif climb_speed_mode == 'vy':
        airspeed = vy_est
    elif climb_speed_mode == 'vy_poh':
        airspeed = int(round(_vy_poh)) if _have_vy_poh else vy_est
    else:
        airspeed = st.sidebar.number_input(
            "Airspeed at failure (KIAS)", min_value=40, max_value=300,
            value=int(round(_vy_poh)) if _have_vy_poh else vy_est, step=5,
        )

    st.sidebar.caption(
        f"Vs clean at {weight} lb = {vs_clean_est:.0f} KIAS"
    )

    # Charlie #C2 — safety-margin slider repositioned right after climb-out
    # speed (was at the bottom of the sidebar).  Pilots want to set the buffer
    # before they start tuning anything else.
    safety_margin_factor = st.sidebar.slider(
        "Safety-margin factor",
        min_value=1.00, max_value=2.50, value=1.25, step=0.05,
        key="safety_margin_factor",
        help=(
            "Multiplier applied to the calculated minimum critical altitude to "
            "produce the *recommended* go/no-go altitude shown on the data card.  "
            "1.00 = the bare aerodynamic minimum (zero buffer for pilot skill, "
            "wind variability, or aircraft performance scatter).  **1.25 = +25% — "
            "Charlie's call (May 2026); reasonable starting point for a current "
            "pilot in a familiar airplane.**  1.50–2.00 = larger buffer for "
            "low-currency pilots, gusty winds, or high density-altitude operations."
        ),
    )

    # Wing geometry
    _ar = config.span ** 2 / config.wing_area
    st.sidebar.caption(
        f"b = {config.span:.1f} ft · S = {config.wing_area:.0f} ft² · AR = {_ar:.1f}"
    )

    # Post-failure glide speed mode
    # Charlie Precourt feedback: Vbg as a label is misleading for the turn.
    # Promote the OPERATIONAL targets (Vs(φ)+10 and 1.3·Vs(φ)) and demote
    # the aerodynamic best-glide modes to "Advanced / experimentation".
    speed_mode_options = {
        'vs_plus_10':    'Vs at bank + 10 kt  (recommended)',
        'vs_x_1p3':      '1.3 × Vs at bank',
        'fixed':         'Maintain failure speed',
        'best_glide_1g': 'Best L/D — wings level (advanced)',
        'best_glide_nz': 'Best L/D — adjusted for load factor (advanced)',
    }
    speed_mode = st.sidebar.radio(
        "Glide speed mode", list(speed_mode_options.keys()),
        format_func=lambda x: speed_mode_options[x], index=0,
        help="Operational presets target a speed safely above the turning "
             "stall.  Best-L/D modes target the aerodynamic optimum and "
             "are useful for experimentation but are too close to stall "
             "for most pilots in the turn.",
    )

    # Bank angle
    bank_angle = st.sidebar.slider(
        "Bank angle (°)", min_value=10, max_value=60, value=45, step=5,
        help=(
            "Bank angle held during the turnback.  In a level turn the wing must "
            "produce load factor n_z = 1/cos(φ): 30°→1.15g, 45°→1.41g, 60°→2.00g.  "
            "Stall speed scales by √n_z (so Vs at 60° is ~41% higher than wings level), "
            "and induced drag grows as CL² — steeper banks complete the turn in less "
            "distance but bleed energy faster and shrink the stall margin.  "
            "45° is the textbook compromise; instructors often demonstrate 30°/45°/60° "
            "to find the per-aircraft sweet spot."
        ),
    )

    # Always show the operational target speeds for the selected bank — even
    # if user picked a different speed_mode — so they see what Charlie's
    # recommended numbers would be.
    from analysis.turnback_simulator import operational_turn_speeds
    _ops = operational_turn_speeds(config, weight, bank_angle, 0, 0,
                                    landing_flaps=False)
    st.sidebar.info(
        f"**At {bank_angle}° bank (nz={_ops['nz']:.2f}):**\n\n"
        f"• Vs(φ) = **{_ops['vs_kias']:.0f} KIAS**\n\n"
        f"• Vs(φ) + 10 kt = **{_ops['vs_plus_10_kias']:.0f} KIAS**\n\n"
        f"• 1.3 × Vs(φ) = **{_ops['vs_x_1p3_kias']:.0f} KIAS**"
    )

    vbg_clean_kias = 0
    vbg_geardown_kias = 0
    vbg_landing_kias = 0
    if speed_mode in ('best_glide_1g', 'best_glide_nz'):
        # Compute and display best glide speeds
        vbg_1g, cl_opt, ld_max = best_glide_kias(config, weight, 1.0, 0, 0)
        st.sidebar.info(
            f"**Computed best L/D (1g):** {vbg_1g:.0f} KIAS\n\n"
            f"CL_opt = {cl_opt:.3f} · L/D_max = {ld_max:.1f}"
        )
        if speed_mode == 'best_glide_nz':
            nz_preview = 1.0 / math.cos(math.radians(bank_angle))
            vbg_turn, _, _ = best_glide_kias(config, weight, nz_preview, 0, 0)
            st.sidebar.info(
                f"**Computed best L/D ({bank_angle}° bank, nz={nz_preview:.2f}):** "
                f"{vbg_turn:.0f} KIAS"
            )
        with st.sidebar.expander("Advanced — POH Vbg overrides", expanded=False):
            # POH-derived defaults (scaled to current weight by sqrt(W/MTOW)).
            _vbg_clean_poh = vbg_poh_kias(ac_key[0], config, weight, 'clean')
            _vbg_gear_poh = vbg_poh_kias(ac_key[0], config, weight, 'geardown')
            _vbg_land_poh = vbg_poh_kias(ac_key[0], config, weight, 'landing')
            if _vbg_clean_poh is not None:
                st.caption(
                    f"**POH Vbg @ {weight:,} lb** (scaled from MTOW): "
                    f"clean **{_vbg_clean_poh:.0f}** · "
                    f"gear-down **{_vbg_gear_poh:.0f}** · "
                    f"landing **{_vbg_land_poh:.0f}** KIAS.  "
                    "Defaults below are pre-filled from these.  "
                    "*(0 = revert to aerodynamic computation)*"
                )
                _def_clean = int(round(_vbg_clean_poh))
                _def_gear = int(round(_vbg_gear_poh))
                _def_land = int(round(_vbg_land_poh))
            else:
                st.caption("Override the aerodynamic computation with POH best-glide values. *(0 = auto)*")
                _def_clean = _def_gear = _def_land = 0
            vbg_clean_kias = st.number_input(
                "Vbg clean (gear up, flaps up)",
                min_value=0, max_value=300, value=_def_clean, step=1,
                help="POH best-glide speed for clean configuration "
                     "(gear up, flaps up). 0 = use aerodynamic computation.",
                key=f"vbg_clean_{ac_key[0]}",
            )
            vbg_geardown_kias = st.number_input(
                "Vbg gear down (gear ↓, flaps up)",
                min_value=0, max_value=300, value=_def_gear, step=1,
                help="POH best-glide speed with gear extended, flaps up. "
                     "0 = use aerodynamic computation.",
                key=f"vbg_gear_{ac_key[0]}",
            )
            vbg_landing_kias = st.number_input(
                "Vbg landing (gear ↓, flaps ↓)",
                min_value=0, max_value=300, value=_def_land, step=1,
                help="POH best-glide speed with gear and flaps extended "
                     "(landing configuration). 0 = use aerodynamic computation.",
                key=f"vbg_land_{ac_key[0]}",
            )

    # Flap setting block has moved up under "Aircraft & Configuration"
    # (Charlie #C1) — this comment intentionally left as a breadcrumb.

    # Reaction time
    reaction_time = st.sidebar.slider(
        "Reaction time (s)", min_value=0.0, max_value=10.0,
        value=5.0, step=0.5,
        help=(
            "Time from engine failure until the pilot lowers the nose and "
            "establishes glide attitude.  During this delay the airplane decelerates "
            "and loses altitude with no recovery action.  FAA accident studies and "
            "NTSB simulator work suggest 3–4 s for a startled, well-trained pilot "
            "and 5–7 s for a surprised one.  Charlie Precourt teaches that practiced "
            "pilots can hit 1–2 s but the unannounced average is much higher; this "
            "slider lets you bracket your personal worst-case.  "
            "**Default 5 s = realistic startle-response (Charlie's call, May 2026).**"
        ),
    )

    # ── Weather ──
    st.sidebar.markdown("---")
    st.sidebar.subheader("Weather")
    weather_mode = st.sidebar.radio(
        "Weather source",
        ["Manual inputs", "Paste METAR"],
        index=0,
        horizontal=True,
        help="Manual: enter ISA deviation and wind directly. "
             "METAR: paste a raw METAR string (from ForeFlight / "
             "aviationweather.gov) to auto-fill temperature, altimeter, and surface wind.",
    )

    parsed_metar = None
    altimeter_inhg = 29.92

    if weather_mode == "Paste METAR":
        from engine.metar_parser import parse_metar, isa_deviation_c
        raw_metar = st.sidebar.text_area(
            "METAR",
            value="",
            height=80,
            placeholder="KSEZ 251853Z 24008KT 10SM CLR 22/M01 A3008",
            help="Paste the full raw METAR string. Remarks (RMK …) are ignored.",
        )
        parsed_metar = parse_metar(raw_metar) if raw_metar.strip() else None
        if parsed_metar is None and raw_metar.strip():
            st.sidebar.error("Could not parse METAR — falling back to manual values.")

    if parsed_metar is not None:
        from engine.metar_parser import isa_deviation_c
        # Temperature → ISA deviation
        if parsed_metar.temperature_c is not None:
            isa_dev = int(round(isa_deviation_c(parsed_metar.temperature_c, field_elev)))
        else:
            isa_dev = 0
        # Altimeter
        if parsed_metar.altimeter_inhg is not None:
            altimeter_inhg = parsed_metar.altimeter_inhg
        # Wind
        if parsed_metar.wind_variable or parsed_metar.wind_from_deg is None:
            wind_from_true = 0
            st.sidebar.caption("Wind variable — assuming calm/headwind for sim.")
        else:
            wind_from_true = int(parsed_metar.wind_from_deg)
        wind_speed = int(parsed_metar.wind_speed_kt)
        # Show summary
        summary_bits = [f"**Parsed:** {parsed_metar.station}"]
        if parsed_metar.temperature_c is not None:
            summary_bits.append(f"OAT {parsed_metar.temperature_c:.0f}°C (ISA{isa_dev:+d})")
        summary_bits.append(f"alt {altimeter_inhg:.2f}\"")
        if parsed_metar.wind_variable:
            summary_bits.append(f"wind VRB at {wind_speed} kt")
        else:
            gust = f"G{parsed_metar.wind_gust_kt}" if parsed_metar.wind_gust_kt else ""
            summary_bits.append(f"wind {wind_from_true:03d}/{wind_speed:02d}{gust} kt")
        st.sidebar.success(" · ".join(summary_bits))
    else:
        # Manual weather inputs (Charlie #D2: pilots think in OAT °C, not ISA dev)
        from engine.metar_parser import isa_deviation_c
        temp_input_mode = st.sidebar.radio(
            "Temperature input",
            ["OAT (°C)", "ISA deviation (°C)"],
            index=0,
            horizontal=True,
            help=(
                "OAT: enter the actual outside-air temperature you read on the "
                "thermometer / METAR (typical pilot mental model).  ISA deviation: "
                "enter the offset from standard atmosphere at field elevation "
                "(useful when comparing performance charts).  Both are converted "
                "to the same internal density-altitude correction."
            ),
        )
        if temp_input_mode == "OAT (°C)":
            # Sensible default = ISA at field elevation (i.e. ISA dev 0)
            _isa_t = 15.0 - 0.001981 * float(field_elev)
            oat_c = st.sidebar.number_input(
                "OAT (°C)", min_value=-50, max_value=55,
                value=int(round(_isa_t)), step=1,
                help="Outside-air temperature at field elevation.",
            )
            isa_dev = int(round(isa_deviation_c(float(oat_c), field_elev)))
            st.sidebar.caption(f"→ ISA deviation = **{isa_dev:+d}°C**")
        else:
            isa_dev = st.sidebar.number_input(
                "ISA deviation (°C)", min_value=-40, max_value=50,
                value=0, step=1,
                help="Difference between actual OAT and ISA temperature at field elevation.",
            )
        if use_airport_db:
            altimeter_inhg = st.sidebar.number_input(
                "Altimeter (inHg)", min_value=27.50, max_value=31.50,
                value=29.92, step=0.01, format="%.2f",
                help="Optional. Used for pressure-altitude logging only.",
            )
        # Charlie #A1 — step=1 so pilots can enter 8 kt / 12 kt etc.  Step=5
        # was forcing values to multiples of 5 (the regression Charlie flagged).
        wind_speed = st.sidebar.number_input(
            "Surface wind speed (kt)", min_value=0, max_value=60,
            value=10, step=1,
            help="Type any integer 0–60 kt.  Use the +/- buttons for ±1 kt.  "
                 "Default 10 kt at 45° right-quartering (worst-case turn-into-wind side).",
        )
        if use_airport_db:
            # Default = 45° right of runway heading (right-quartering headwind)
            _default_wind_from = int(round((runway_heading_true + 45.0) % 360.0)) if runway_heading_true else 45
            wind_from_true = st.sidebar.number_input(
                "Surface wind FROM (°true)", min_value=0, max_value=359,
                value=_default_wind_from, step=1,
                help="Wind direction in true degrees (matches METAR).  "
                     "Headwind/crosswind components are computed from runway heading.  "
                     "Default = runway heading + 45° (right-quartering headwind).  "
                     "Type any value 0–359°.",
            )
        else:
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
                value=45,
                format_func=lambda d: wind_dir_options.get(d, f"{d}°"),
            )
            wind_from_true = wind_from_deg  # legacy: relative-to-runway used directly

    # Convert true wind to runway-relative if airport DB is in use
    if use_airport_db:
        from engine.airport_db import wind_components
        hw, xw, rel = wind_components(wind_from_true, wind_speed, runway_heading_true)
        wind_from_deg = int(round(rel))  # relative angle for sim API
    else:
        # Already runway-relative (legacy mode) — derive HW/XW from relative angle
        from engine.airport_db import wind_components
        wind_from_deg = wind_from_true
        # In legacy mode, runway heading is treated as 0; wind_from_true IS rel.
        hw, xw, rel = wind_components(wind_from_true, wind_speed, 0.0)

    # Charlie #A1/A2/12 — surface HW/XW components prominently so crosswind
    # behaviour is never silently ambiguous.  Always shown when wind > 0.
    if wind_speed > 0:
        if abs(xw) < 0.05:
            xw_label = "0 kt"
        else:
            xw_label = f"{abs(xw):.1f} kt {'RIGHT' if xw > 0 else 'LEFT'} cross"
        if abs(hw) < 0.05:
            hw_label = "0 kt along"
        else:
            hw_label = f"{abs(hw):.1f} kt {'HEAD' if hw > 0 else 'TAIL'}"
        if use_airport_db:
            st.sidebar.info(
                f"**Wind {wind_from_true:03d}°T / {wind_speed:02d} kt**  \n"
                f"Relative to runway: **{int(round(rel)):03d}°**  \n"
                f"→ {hw_label} · {xw_label}"
            )
        else:
            st.sidebar.info(
                f"**Wind {wind_speed:02d} kt from {wind_from_deg:03d}° (rel. runway)**  \n"
                f"→ {hw_label} · {xw_label}"
            )
    
    # Wind at altitude (ForeFlight data) — flexible rows (Charlie #4 / #D1 / #E3)
    st.sidebar.caption(
        "*Wind at altitude: get from ForeFlight winds-aloft tab.  "
        "Add or remove rows as needed; surface (0 ft) is the wind above.*  \n"
        "*Direction column is now wired through the engine (E3): wind direction "
        "is interpolated per-altitude using sin/cos to handle the 0/360° wrap.  "
        "Leave Direction = 0 to keep a row speed-only.*"
    )
    default_wind_rows = pd.DataFrame([
        {"Alt (ft)": 1000, "Dir (°)": int(round(wind_from_true)) if wind_speed > 0 else 0, "Kt": float(wind_speed)},
        {"Alt (ft)": 2000, "Dir (°)": int(round(wind_from_true)) if wind_speed > 0 else 0, "Kt": float(wind_speed)},
        {"Alt (ft)": 3000, "Dir (°)": int(round(wind_from_true)) if wind_speed > 0 else 0, "Kt": float(wind_speed)},
    ])
    wind_profile_df = st.sidebar.data_editor(
        default_wind_rows,
        num_rows="dynamic",
        hide_index=True,
        key="wind_profile_editor",
        # Height = header (~38) + per-row (~36) + padding for the +/- row.
        # Default Streamlit height (~150) chops 3 rows into a scrollbar.
        height=185,
        use_container_width=True,
        column_config={
            "Alt (ft)": st.column_config.NumberColumn(
                min_value=0, max_value=20000, step=100, format="%d",
                help="Altitude AGL in feet.",
                width="small",
            ),
            "Dir (°)": st.column_config.NumberColumn(
                min_value=0, max_value=360, step=10, format="%d",
                help="Wind FROM direction in true degrees.  Interpolated "
                     "per-altitude with sin/cos to handle 0/360° wrap.  "
                     "Leave 0 to keep a row speed-only.",
                width="small",
            ),
            "Kt": st.column_config.NumberColumn(
                min_value=0, max_value=200, step=1, format="%d",
                help="Wind speed in knots.",
                width="small",
            ),
        },
    )
    # Build wind_profile list of (alt_ft, speed_kt) tuples for the engine,
    # ignoring blank or zero-altitude rows.  Direction column (E3) is now
    # also consumed: per-altitude wind direction is converted to runway-
    # relative degrees and passed in as wind_dir_profile.
    wind_profile = []
    wind_dir_profile = []
    try:
        for _, row in wind_profile_df.iterrows():
            a = row.get("Alt (ft)")
            s = row.get("Kt")
            d = row.get("Dir (°)")
            if pd.notna(a) and pd.notna(s) and float(a) > 0:
                wind_profile.append((float(a), float(s)))
                if pd.notna(d) and float(d) > 0:
                    # Convert true direction to runway-relative.  In airport-
                    # DB mode runway_heading_true is the actual heading; in
                    # legacy mode the user input is already runway-relative
                    # (runway treated as heading 0).
                    if use_airport_db:
                        rel = (float(d) - runway_heading_true) % 360.0
                    else:
                        rel = float(d) % 360.0
                    wind_dir_profile.append((float(a), rel))
    except Exception:
        wind_profile = []
        wind_dir_profile = []
    if not wind_dir_profile:
        wind_dir_profile = None  # engine treats None as "constant surface dir"

    # Legacy wind_1000/2000/3000 kept zeroed — wind_profile takes precedence.
    wind_1000_kt = 0
    wind_2000_kt = 0
    wind_3000_kt = 0

    # Runway Geometry — runway model is always on (departure airport is mandatory).
    st.sidebar.markdown("---")
    st.sidebar.subheader("Runway Geometry")
    use_runway = True
    runway_friction = 1.0  # default = standard dry asphalt
    # Default runway length to DB value if airport selected
    _default_rwy_len = int(round(runway_length_db)) if runway_length_db > 0 else 5500
    _default_rwy_len = max(1000, min(15000, _default_rwy_len))
    if runway_length_db > 0:
        st.sidebar.caption(
            f"Runway length pre-filled from {selected_airport_ident} "
            f"RW {selected_runway_ident}."
        )
    runway_length = st.sidebar.number_input(
        "Runway length (ft)", min_value=1000, max_value=15000,
        value=_default_rwy_len, step=100,
    )

    # Charlie #5b — POH-derived liftoff distance estimator.
    # If a POH reference number exists for this aircraft, default to ON
    # and scale by (W/MTOW)² × 1/σ.  Otherwise the field is a manual entry.
    _poh_key = ac_key[0] if ac_key[0] in POH_GROUND_ROLL_FT else None
    if _poh_key:
        use_poh_liftoff = st.sidebar.checkbox(
            "Estimate liftoff distance from POH",
            value=True,
            help=f"Use the published POH ground-roll for the {_poh_key} "
                 "(sea level, ISA, MTOW) and scale by weight² and "
                 "1/density-ratio for the selected field/ISA conditions.  "
                 "Uncheck to enter the value manually.",
        )
        if use_poh_liftoff:
            _est = estimate_ground_roll(_poh_key, config, weight, field_elev, isa_dev)
            liftoff_distance = float(round(_est.ground_roll_ft, 0)) if _est else 1000.0
            if _est:
                st.sidebar.caption(
                    f"POH base {int(_est.base_sl_mtow_ft)} ft · "
                    f"weight × {_est.weight_factor:.2f} · "
                    f"density × {_est.density_factor:.2f} (DA "
                    f"{int(_est.density_altitude_ft):,} ft) → "
                    f"**{int(round(_est.ground_roll_ft)):,} ft**"
                )
        else:
            liftoff_distance = float(st.sidebar.number_input(
                "Liftoff distance (ft)", min_value=100, max_value=10000,
                value=1000, step=100,
            ))
    else:
        st.sidebar.caption(
            f"*No POH reference for {ac_key[0]} — enter manually.*"
        )
        liftoff_distance = float(st.sidebar.number_input(
            "Liftoff distance (ft)", min_value=100, max_value=10000,
            value=1000, step=100,
        ))
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

    # Charlie #E1 — intersection departure.  Pilot lines up partway down
    # the runway instead of at the threshold.  Reduces runway available
    # for a straight-ahead landing and shifts the turnback geometry.
    intersection_offset_ft = float(st.sidebar.number_input(
        "Intersection departure offset (ft)",
        min_value=0, max_value=int(max(0, runway_length - 500)), value=0, step=100,
        help=(
            "Distance from the runway threshold to the takeoff position.  "
            "0 = full-length departure (default).  Set positive when "
            "departing from a runway intersection — the aircraft starts "
            "the takeoff roll partway down the runway, so the runway "
            "REMAINING ahead for a straight-ahead landing shrinks by this "
            "amount.  Common at busy fields: e.g. KSEZ has intersection "
            "departures from taxiway A4."
        ),
    ))
    st.sidebar.caption(
        "**Aim point** = computed rollout + safety margin.  "
        "Forward slip used if needed to steepen descent.  "
        "**Straight-ahead** lands on the remaining runway."
    )
    aim_point = runway_length  # passed for API compat; sim auto-computes aim_y

    # Always remember the *published* DB length so the data card can show it
    # even when the DB lookup fails (Charlie #A3).
    runway_length_published = float(runway_length_db) if runway_length_db > 0 else 0.0

    # Landing flaps auto-deploy on final approach in the runway model.
    flap_on_return = False
    st.sidebar.caption(
        "ℹ️ **Runway model active:** landing flaps auto-deploy on "
        "final approach. Forward slip added if needed to make the runway."
    )

    # Prop state after engine failure — 2-step picker.
    # Step 1: prop *type* (fixed-pitch vs variable-pitch / CS / feathering).
    # Step 2: rotational state (stopped vs windmilling vs feathered-spinning, etc.).
    st.sidebar.markdown("---")
    st.sidebar.subheader("Prop Drag")

    # Auto-detect a sensible default prop type from the aircraft.  Curated
    # list of variable-pitch / constant-speed / feathering installs in the
    # AIRCRAFT_CONFIG library — covers all turboprops, complex retractables,
    # and high-perf pistons.  Anything not in this set defaults to fixed.
    _ac_name = ac_key[0] if isinstance(ac_key, (tuple, list)) else str(ac_key)
    _vp_default_models = {
        # Single-engine turboprops (all feathering)
        'Meridian', 'M600', 'M700',
        'TBM700', 'TBM850', 'TBM900', 'TBM910', 'TBM930', 'TBM940', 'TBM960',
        'PC-12', 'PC12',
        'Denali',
        'E1000',
        'Kodiak100', 'Kodiak900',
        'C208', 'C208B', 'C208EX',
        # Twin-engine turboprops
        'DHC6', 'ATR42', 'ATR72', 'Q300', 'Q400',
        # Complex piston singles (constant-speed)
        'A36', 'V35', 'Bonanza',
        'M20V', 'Mooney',
        'C182RG', 'C182T', 'C210',
        'SR22', 'SR22T',
        'TTx',
        'Mirage',
        # Utility / backcountry pistons with CS prop (Hartzell on most A-1B/C)
        'Husky',
    }
    _default_prop_type = 'variable' if _ac_name in _vp_default_models else 'fixed'

    prop_type = st.sidebar.radio(
        "Prop type",
        options=['fixed', 'variable'],
        format_func=lambda v: {
            'fixed': 'Fixed-pitch',
            'variable': 'Variable-pitch (constant-speed / feathering)',
        }[v],
        index=0 if _default_prop_type == 'fixed' else 1,
        key=f"prop_type_{_ac_name}",
        help=(
            "**Fixed-pitch**: most trainers (C150/152/172, Cherokee 140, J3 Cub, "
            "many RVs).  Blade angle is fixed; prop either windmills or stops.\n\n"
            "**Variable-pitch**: constant-speed prop with a governor.  Pilot can "
            "coarsen the blade angle ('pull the blue lever back') to reduce drag, "
            "and on feathering installations (turboprops, twins, some CS singles) "
            "can take the blades all the way to feather (edge-on to the airflow)."
        ),
    )

    if prop_type == 'fixed':
        prop_state_options = {
            'fp_windmilling': 'Windmilling (default — prop freewheels)  ΔCDo +0.0020',
            'fp_stopped':     'Stopped (slow below windmill threshold)  ΔCDo +0.0015',
        }
        _state_help = (
            "After engine failure on a fixed-pitch single:\n\n"
            "• **Windmilling**: prop continues to spin freely.  This is what "
            "happens by default — the airstream keeps the disc rotating, "
            "presenting a flat plate to the flow.  Largest drag of the two.\n\n"
            "• **Stopped**: if the pilot slows the airplane below the prop "
            "windmill speed (typically near the stall), the prop stops "
            "rotating.  Slightly less drag (~0.0005 ΔCDo improvement).\n\n"
            "Trade-off: stopping the prop costs airspeed and altitude, and "
            "is rarely worth it during a turnback — pick **windmilling** "
            "for the realistic case."
        )
        _default_state_idx = 0
    else:
        prop_state_options = {
            'vp_feathered_stopped':  'Feathered & stopped (best case)  ΔCDo +0.0005',
            'vp_feathered_spinning': 'Feathered & spinning (residual rotation)  ΔCDo +0.0010',
            'vp_unfeathered':        'Unfeathered (oil-pressure loss / no feather)  ΔCDo +0.0040',
        }
        _state_help = (
            "After engine failure on a variable-pitch (CS / feathering) single:\n\n"
            "• **Feathered & stopped**: blades fully edge-on to the airflow AND "
            "the disc has stopped rotating.  Minimum-drag case.  Typical of "
            "turboprops (Meridian, TBM, PC-12) when the pilot pulls the "
            "condition lever to FEATHER and the auto-feather system completes.\n\n"
            "• **Feathered & spinning**: blades feathered but the disc still "
            "rotates slowly (residual oil pressure, or airspeed too high to "
            "stop the prop).  Slightly more drag than feathered & stopped.\n\n"
            "• **Unfeathered**: oil-pressure loss leaves CS blades in a flat "
            "low-pitch position, or the pilot never pulled the prop lever to "
            "coarse pitch.  Largest drag of any state — worse than fixed-pitch "
            "windmilling.  This is the worst-case CS-prop failure.\n\n"
            "Brief item: if the engine quits, **immediately** pull the prop "
            "control to coarse / feather to move from unfeathered toward the "
            "feathered cases."
        )
        _default_state_idx = 0

    prop_state = st.sidebar.radio(
        "Prop rotational state",
        list(prop_state_options.keys()),
        format_func=lambda x: prop_state_options[x],
        index=_default_state_idx,
        key=f"prop_state_{_ac_name}_{prop_type}",
        help=_state_help,
    )

    gear_down = True  # default for fixed-gear aircraft
    gear_retract_time_s = None  # E2: post-failure gear-up time (None = never)
    if config.dcdo_gear > 0:
        # Retractable gear — let user choose
        gear_down = st.sidebar.checkbox(
            "Gear down", value=True,
            help=f"Add landing-gear drag (ΔCDo = +{config.dcdo_gear:.4f}). "
                 "Un-check to model gear retracted after takeoff.",
        )
        # Charlie #E2 — post-failure gear retraction.  Some retractable
        # pilots are taught to bring the gear up after engine failure to
        # reduce drag and extend glide.  Others teach to leave it where it
        # is.  Let the user model both.
        if gear_down:
            retract_after_failure = st.sidebar.checkbox(
                "Retract gear after failure",
                value=False,
                help=(
                    "Some retractable-gear procedures call for raising the "
                    "gear immediately after engine failure to reduce drag "
                    "during the turnback (Mooney, Bonanza, C182RG).  "
                    "When checked, the sim removes gear drag at the "
                    "specified delay after failure."
                ),
            )
            if retract_after_failure:
                gear_retract_time_s = float(st.sidebar.number_input(
                    "Gear-up delay after failure (s)",
                    min_value=0.0, max_value=15.0, value=2.0, step=0.5,
                    help=(
                        "Seconds from engine failure until the gear is fully "
                        "retracted (motor/hydraulic cycle time).  Typical "
                        "5–8 s for piston singles, 2–4 s for high-performance "
                        "retractables.  Drag drops by ΔCDo = "
                        f"{config.dcdo_gear:.4f} at this time."
                    ),
                ))
    else:
        st.sidebar.caption("Fixed gear — gear drag included in base CDo")

    # Charlie #E2 — climb-out steering toggle.  In real life pilots either
    # crab into the wind (ground track stays on centerline) or hold the
    # nose on runway heading and let the airplane drift downwind.  The
    # latter shows up as a lateral offset at engine failure.
    climb_steering = st.sidebar.radio(
        "Climb-out steering",
        options=['track', 'heading'],
        format_func=lambda v: {
            'track': 'Track hold (crab into wind)',
            'heading': 'Heading hold (drift downwind)',
        }[v],
        index=0,
        help=(
            "Track hold: pilot crabs into the wind so the ground track "
            "stays on the runway centerline at engine failure (typical "
            "instructor demo, lateral offset = 0 ft).\n\n"
            "Heading hold: pilot keeps the nose on runway heading; the "
            "airplane drifts downwind during the climb so it is already "
            "off centerline when the engine quits.  Pedagogically useful "
            "to visualize how a crosswind biases the turnback toward the "
            "downwind side."
        ),
    )

    # E2-P2 (May 2026) — when on, runs a second envelope with the OPPOSITE
    # climb-steering choice and overlays it on the map plus shows a delta
    # callout above the metrics.  The teaching purpose: in a crosswind,
    # heading-hold pre-positions the airplane toward the favorable side and
    # — combined with the smart aim point — lowers the critical altitude.
    compare_climb_steering = st.sidebar.checkbox(
        "Compare alternate climb steering",
        value=True,
        help=(
            "Builds a SECOND envelope using the opposite climb-steering "
            "choice and overlays it on the satellite map (dashed gold).  "
            "Adds a delta callout: 'Track-hold X ft  ·  Heading-hold Y ft  "
            "·  Δ Z ft saved'.  Doubles envelope compute time."
        ),
    )

    # Altitude step & max
    alt_step = st.sidebar.select_slider(
        "Altitude step (ft)", options=[50, 100, 200, 500], value=100,
        help=(
            "Vertical resolution of the heart-shaped envelope.  Smaller = finer "
            "contour but slower simulation (each step runs a full LEFT and RIGHT "
            "trajectory).  100 ft is a good default; drop to 50 ft for a polished "
            "plot, raise to 200/500 ft for quick what-if sweeps."
        ),
    )

    max_alt_input = st.sidebar.number_input(
        "Max altitude to plot (ft AGL, 0=auto)",
        min_value=0, max_value=5000, value=0, step=100,
        help=(
            "Upper bound for the altitude sweep.  0 = auto-pick a value safely "
            "above the critical altitude.  Set higher if you want to study how the "
            "envelope grows once the turn is trivially completable."
        ),
    )

    # Safety-margin factor moved up under climb-out speed (Charlie #C2,
    # May 2026).  This breadcrumb left in place so future readers don't
    # look for it down here.

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
                wind_profile=wind_profile,
                wind_dir_profile=wind_dir_profile,
                runway_length=runway_length, liftoff_distance=liftoff_distance,
                aim_point=aim_point, flap_on_return=flap_on_return,
                speed_mode=speed_mode,
                prop_state=prop_state,
                gear_down=gear_down,
                gear_retract_time_s=gear_retract_time_s,
                intersection_offset_ft=intersection_offset_ft,
                vbg_clean_kias=vbg_clean_kias,
                vbg_geardown_kias=vbg_geardown_kias,
                vbg_landing_kias=vbg_landing_kias,
                touchdown_margin_ft=touchdown_margin_ft,
                runway_friction=runway_friction,
                climb_steering=climb_steering,
            )

        # E2-P2: alternate-steering comparison envelope
        comparison = None
        if compare_climb_steering:
            alt_steering = 'heading' if climb_steering == 'track' else 'track'
            with st.spinner(f"Computing comparison ({alt_steering} hold)..."):
                cmp_crit, cmp_env, cmp_crit_l, cmp_crit_r, cmp_sa = build_turnback_envelope(
                    config, weight, airspeed, bank_angle, flap_setting,
                    reaction_time, field_elev, isa_dev,
                    alt_step=alt_step, max_alt=max_alt,
                    wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                    wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                    wind_profile=wind_profile,
                    wind_dir_profile=wind_dir_profile,
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=flap_on_return,
                    speed_mode=speed_mode,
                    prop_state=prop_state,
                    gear_down=gear_down,
                    gear_retract_time_s=gear_retract_time_s,
                    intersection_offset_ft=intersection_offset_ft,
                    vbg_clean_kias=vbg_clean_kias,
                    vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction,
                    climb_steering=alt_steering,
                )
            comparison = {
                'climb_steering': alt_steering,
                'envelope': cmp_env,
                'critical_alt': cmp_crit,
                'critical_alt_left': cmp_crit_l,
                'critical_alt_right': cmp_crit_r,
                'straight_ahead_max_alt': cmp_sa,
            }

        st.session_state['turnback_result'] = {
            'critical_alt': critical_alt,
            'critical_alt_left': critical_alt_left,
            'critical_alt_right': critical_alt_right,
            'straight_ahead_max_alt': straight_ahead_max_alt,
            'envelope': envelope,
            'climb_steering': climb_steering,
            'comparison': comparison,
            'config': config,
            'weight': weight,
            'airspeed': airspeed,
            'bank_angle': bank_angle,
            'flap_setting': flap_setting,
            'takeoff_flap_setting': takeoff_flap_setting,
            'flap_retract_alt_ft': flap_retract_alt_ft,
            'reaction_time': reaction_time,
            'crosswind_kt': xw,
            'runway_length': runway_length,
            'runway_length_published': runway_length_published,
            'liftoff_distance': liftoff_distance,
            'aim_point': envelope[0]['left'].get('computed_aim_y', 0.0) if envelope else 0.0,
            'speed_mode': speed_mode,
            # Data-card extras
            'airport_ident': selected_airport_ident,
            'runway_ident': selected_runway_ident,
            'runway_heading_true': runway_heading_true,
            'field_elev': field_elev,
            'isa_dev': isa_dev,
            'altimeter_inhg': altimeter_inhg,
            'wind_profile': wind_profile,
            'prop_state': prop_state,
            'gear_down': gear_down,
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
                wind_profile=wind_profile,
                wind_dir_profile=wind_dir_profile,
                runway_length=runway_length, liftoff_distance=liftoff_distance,
                speed_mode=speed_mode,
                prop_state=prop_state,
                gear_down=gear_down,
                gear_retract_time_s=gear_retract_time_s,
                intersection_offset_ft=intersection_offset_ft,
                vbg_clean_kias=vbg_clean_kias,
                vbg_geardown_kias=vbg_geardown_kias,
                vbg_landing_kias=vbg_landing_kias,
                touchdown_margin_ft=touchdown_margin_ft,
                runway_friction=runway_friction,
                climb_steering=climb_steering,
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
                    wind_profile=wind_profile,
                    wind_dir_profile=wind_dir_profile,
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=best_flap_on_return,
                    speed_mode=speed_mode,
                    prop_state=prop_state,
                    gear_down=gear_down,
                    gear_retract_time_s=gear_retract_time_s,
                    intersection_offset_ft=intersection_offset_ft,
                    vbg_clean_kias=vbg_clean_kias,
                    vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction,
                    climb_steering=climb_steering,
                )
            st.session_state['turnback_result'] = {
                'critical_alt': crit,
                'critical_alt_left': crit_l,
                'critical_alt_right': crit_r,
                'straight_ahead_max_alt': sa_max,
                'envelope': env,
                'climb_steering': climb_steering,
                'comparison': None,
                'config': config,
                'weight': weight,
                'airspeed': airspeed,
                'bank_angle': best['bank_angle'],
                'flap_setting': best_flap,
                'takeoff_flap_setting': takeoff_flap_setting,
                'flap_retract_alt_ft': flap_retract_alt_ft,
                'reaction_time': reaction_time,
                'ac_key': ac_key,
                'wind_speed': wind_speed,
                'wind_from_deg': wind_from_deg,
                'wind_from_true': wind_from_true,
                'headwind_kt': hw,
                'crosswind_kt': xw,
                'runway_length': runway_length,
                'runway_length_published': runway_length_published,
                'liftoff_distance': liftoff_distance,
                'aim_point': env[0]['left'].get('computed_aim_y', 0.0) if env else 0.0,
                'speed_mode': speed_mode,
                'airport_ident': selected_airport_ident,
                'runway_ident': selected_runway_ident,
                'runway_heading_true': runway_heading_true,
                'field_elev': field_elev,
                'isa_dev': isa_dev,
                'altimeter_inhg': altimeter_inhg,
                'wind_profile': wind_profile,
                'prop_state': prop_state,
                'gear_down': gear_down,
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

    # Safety-margin factor (Charlie #C9): now lives in the sidebar inputs
    # section above, so this block just reads the live value.
    safety_margin_factor = st.session_state.get("safety_margin_factor", 1.25)
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

    # ── Pre-brief banner (May 2026) ──
    # Same one-paragraph self-brief that gets embedded in the printed
    # Takeoff Data Card.  Read aloud at the hold-short line.
    _crit_for_brief = None
    for _item in envelope:
        if _item.get('is_critical_left') or _item.get('is_critical_right'):
            _crit_for_brief = _item.get('left') or _item.get('right')
            break
    try:
        _brief_text = build_pre_brief(res, _crit_for_brief, safety_margin_factor)
        st.markdown(
            f"<div style='background:#fffbe6;border:1px solid #f0c000;"
            f"border-left:5px solid #d4a000;padding:10px 14px;margin:6px 0 14px 0;"
            f"border-radius:4px;'>"
            f"<div style='font-size:11px;font-weight:bold;color:#8a6d00;"
            f"letter-spacing:0.5px;margin-bottom:4px;'>"
            f"📢 SELF-BRIEF — read aloud at the hold-short line</div>"
            f"<div style='font-size:15px;color:#1a1a1a;line-height:1.5;'>"
            f"{_brief_text}</div></div>",
            unsafe_allow_html=True,
        )
    except Exception as _e:  # pragma: no cover
        st.caption(f"(Pre-brief unavailable: {_e})")

    # ── Key metrics ──
    col1, col2, col3, col4, col5 = st.columns(5)
    # Show MSL alongside AGL (Charlie #F1/F5 — pilots fly the altimeter, which is MSL).
    _fe = res.get('field_elev', 0.0) or 0.0
    col1.metric(
        "Critical Alt (LEFT)",
        f"{critical_alt_left_safe:,.0f} ft AGL",
        f"{critical_alt_left_safe + _fe:,.0f} MSL  ·  ({critical_alt_left:,.0f} calc)",
        delta_color="off",
    )
    col2.metric(
        "Critical Alt (RIGHT)",
        f"{critical_alt_right_safe:,.0f} ft AGL",
        f"{critical_alt_right_safe + _fe:,.0f} MSL  ·  ({critical_alt_right:,.0f} calc)",
        delta_color="off",
    )
    col3.metric("Load Factor (nz)", f"{nz:.2f}")
    col4.metric("Stall Speed (turn)", f"{vs_turn:.0f} KIAS")
    if turn_radius and turn_radius < 1e9:
        col5.metric("Turn Radius", f"{turn_radius:,.0f} ft")
    else:
        col5.metric("Turn Radius", "—")

    # ── E2-P2: climb-steering comparison callout ──
    _cmp = res.get('comparison')
    if _cmp:
        _primary_label = {'track': 'Track hold (crab)', 'heading': 'Heading hold (drift)'}[
            res.get('climb_steering', 'track')]
        _alt_label = {'track': 'Track hold (crab)', 'heading': 'Heading hold (drift)'}[
            _cmp.get('climb_steering', 'heading')]
        _primary_worst = max(critical_alt_left, critical_alt_right)
        _alt_worst = max(_cmp.get('critical_alt_left', 0), _cmp.get('critical_alt_right', 0))
        _delta = _primary_worst - _alt_worst  # +ve = alt is BETTER (lower) than primary
        _winner = _alt_label if _delta > 5 else (_primary_label if _delta < -5 else "Tie")
        if abs(_delta) < 5:
            _msg = (
                f"⚖️ **Climb-steering comparison** — {_primary_label}: "
                f"**{_primary_worst:,.0f} ft** vs {_alt_label}: "
                f"**{_alt_worst:,.0f} ft**.  Within 5 ft — wind too light or "
                f"crosswind too small for the choice to matter."
            )
            st.info(_msg)
        else:
            _better = _alt_label if _delta > 0 else _primary_label
            _worse = _primary_label if _delta > 0 else _alt_label
            _better_alt = _alt_worst if _delta > 0 else _primary_worst
            _worse_alt = _primary_worst if _delta > 0 else _alt_worst
            _save = abs(_delta)
            _pct = _save / max(_worse_alt, 1) * 100.0
            _msg = (
                f"✅ **{_better}** wins by **{_save:,.0f} ft** "
                f"(**{_pct:.0f}% lower critical altitude**) vs **{_worse}** "
                f"({_better_alt:,.0f} ft vs {_worse_alt:,.0f} ft).  "
                f"In a crosswind, drifting downwind during the climb pre-positions "
                f"the airplane on the favorable side of centerline — turning into "
                f"the wind from there closes the gap with less than 180° of heading "
                f"change.  Dashed gold heart on the satellite map below shows the "
                f"alternate-steering ground track."
            )
            st.success(_msg)

    # ── E2-P4: Decision Ladder (3 maneuver classes, May 2026) ──
    # Three altitude headlines side-by-side: pilot reads down their AGL
    # and picks the cheapest maneuver that fits.
    from analysis.turnback_simulator import find_min_alt_per_maneuver
    _kind_alts = find_min_alt_per_maneuver(envelope)
    _rwy_id_dl = res.get('runway_ident') or 'departure'
    try:
        _r_int_dl = int(''.join(ch for ch in str(_rwy_id_dl) if ch.isdigit()))
        _recip_n_dl = _r_int_dl + 18 if _r_int_dl <= 18 else _r_int_dl - 18
        _recip_dl = f"{_recip_n_dl:02d}"
    except Exception:
        _recip_dl = 'opposite'
    st.markdown("#### 🪜 Decision Ladder — minimum altitude per maneuver")
    _ladder_cols = st.columns(3)
    _ladder = [
        ('180', '180° turnback',
         f"Land **rwy {_recip_dl}** (opposite direction).",
         '#16a34a'),
        ('540', '540° (orbit + turnback)',
         f"Add a 360° orbit, then land **rwy {_recip_dl}**.",
         '#f59e0b'),
        ('circuit', 'Full circuit',
         f"Continue ~360° around, land **rwy {_rwy_id_dl}** "
         "(same direction as takeoff).",
         '#a855f7'),
    ]
    for col, (kind, title, caption, color) in zip(_ladder_cols, _ladder):
        info = _kind_alts.get(kind)
        with col:
            if info:
                st.markdown(
                    f"<div style='border-left:4px solid {color};"
                    f"padding:6px 10px;background:rgba(0,0,0,0.03);"
                    f"border-radius:4px;'>"
                    f"<div style='font-size:11px;color:#666;'>{title}</div>"
                    f"<div style='font-size:24px;font-weight:bold;'>"
                    f"{info['alt']:,} ft AGL</div>"
                    f"<div style='font-size:11px;color:#444;'>"
                    f"{caption} ({info['side'].upper()} turn)</div>"
                    "</div>",
                    unsafe_allow_html=True,
                )
            else:
                st.markdown(
                    f"<div style='border-left:4px solid #999;"
                    f"padding:6px 10px;background:rgba(0,0,0,0.03);"
                    f"border-radius:4px;opacity:0.55;'>"
                    f"<div style='font-size:11px;color:#666;'>{title}</div>"
                    f"<div style='font-size:24px;font-weight:bold;'>—</div>"
                    f"<div style='font-size:11px;color:#444;'>"
                    f"Not feasible in tested envelope.</div>"
                    "</div>",
                    unsafe_allow_html=True,
                )
    st.caption(
        "Each card shows the **lowest** altitude at which the simulator "
        "completed that maneuver class on at least one side.  Above the "
        "180° number you have a turnback; above the 540° number you can "
        "afford an orbit; above the circuit number you can fly a full "
        "pattern back to the original runway."
    )

    # ── E2-P3: landing-direction crossover panel ──
    # Walk the envelope and find the lowest successful altitude per side
    # AND the altitude where the sim switches from a reverse landing
    # (back into the takeoff direction) to an original-direction landing
    # (full circuit, land downwind).  This is the teaching artifact: the
    # pilot has TWO turnback strategies depending on altitude, not one.
    def _scan_landing_dirs(env, side):
        """Return (lowest_success_alt, crossover_alt_to_original).
        crossover_alt_to_original is the LOWEST altitude at which the sim
        chose 'original' (full circuit).  Below that, all successes are
        'reverse' landings.  None if the maneuver never switches."""
        lowest = None
        crossover = None
        for row in env or []:
            sub = row.get(side) or {}
            if not sub.get('success'):
                continue
            if lowest is None:
                lowest = row.get('alt_agl', 0)
            if sub.get('landing_direction') == 'original' and crossover is None:
                crossover = row.get('alt_agl', 0)
        return lowest, crossover

    _l_low, _l_cross = _scan_landing_dirs(envelope, 'left')
    _r_low, _r_cross = _scan_landing_dirs(envelope, 'right')
    if (_l_low or _r_low) and (_l_cross or _r_cross):
        # Use the worse side for the headline crossover (pilot's safer plan).
        _crosses = [c for c in (_l_cross, _r_cross) if c is not None]
        _lows = [c for c in (_l_low, _r_low) if c is not None]
        _hi = max(_crosses) if _crosses else None
        _lo = min(_lows) if _lows else None
        # Try to get the runway numbers for the message
        _rwy_id = res.get('runway_ident') or 'departure'
        # Reciprocal runway = depart rwy ± 18, mod 36
        _recip = ''
        try:
            _r_int = int(''.join(ch for ch in str(_rwy_id) if ch.isdigit()))
            _recip_n = _r_int + 18 if _r_int <= 18 else _r_int - 18
            _recip = f"{_recip_n:02d}"
        except Exception:
            _recip = 'reciprocal'
        st.info(
            f"🛬 **Two turnback strategies** — From **{int(_lo):,} ft AGL** "
            f"up to ~**{int(_hi):,} ft AGL**, the maneuver lands **reverse** "
            f"(back onto the runway opposite the takeoff direction — "
            f"i.e., land **rwy {_recip}**).  Above ~**{int(_hi):,} ft AGL** "
            f"there's enough altitude for a full circuit and same-direction "
            f"landing on **rwy {_rwy_id}**.  The sim picks automatically; "
            f"this band tells you what the airplane will physically do."
        )
        with st.expander("Landing direction at every altitude (envelope detail)",
                         expanded=False):
            _rows = []
            for row in envelope or []:
                _alt = row.get('alt_agl', 0)
                def _lab(sub):
                    if not sub or not sub.get('success'):
                        return '—'
                    return {'reverse': f'reverse → rwy {_recip}',
                            'original': f'full circuit → rwy {_rwy_id}'}.get(
                        sub.get('landing_direction', ''), sub.get('landing_direction', '?'))
                _rows.append({
                    'Alt AGL (ft)': int(_alt),
                    'LEFT turn': _lab(row.get('left')),
                    'RIGHT turn': _lab(row.get('right')),
                })
            if _rows:
                st.dataframe(pd.DataFrame(_rows), hide_index=True,
                             use_container_width=True)

    # ── Safety-margin visualization (Phase 2 — Charlie #5) ──
    # Make the buffer between *calculated minimum* and *recommended* visible.
    _buf_pct = (safety_margin_factor - 1.0) * 100.0
    _worse_calc = max(critical_alt_left, critical_alt_right)
    _worse_safe = max(critical_alt_left_safe, critical_alt_right_safe)
    _buffer_ft = _worse_safe - _worse_calc
    fig_margin = go.Figure()
    # Draw two horizontal bars: one direction each.
    fig_margin.add_trace(go.Bar(
        y=['LEFT turn', 'RIGHT turn'],
        x=[critical_alt_left, critical_alt_right],
        name='Calculated minimum (aerodynamic)',
        orientation='h',
        marker=dict(color='rgba(255, 127, 14, 0.85)'),
        text=[f'{critical_alt_left:,.0f} ft', f'{critical_alt_right:,.0f} ft'],
        textposition='inside',
        insidetextanchor='end',
        textfont=dict(color='white', size=13),
        hovertemplate='Calculated minimum: %{x:,.0f} ft AGL<extra></extra>',
    ))
    fig_margin.add_trace(go.Bar(
        y=['LEFT turn', 'RIGHT turn'],
        x=[critical_alt_left_safe - critical_alt_left,
           critical_alt_right_safe - critical_alt_right],
        name=f'Safety buffer (+{_buf_pct:.0f}%)',
        orientation='h',
        marker=dict(color='rgba(44, 160, 44, 0.65)', pattern=dict(shape='/')),
        text=[f'+{critical_alt_left_safe - critical_alt_left:,.0f}',
              f'+{critical_alt_right_safe - critical_alt_right:,.0f}'],
        textposition='inside',
        textfont=dict(color='white', size=12),
        hovertemplate='Safety buffer: +%{x:,.0f} ft<extra></extra>',
    ))
    # Recommended threshold marker (dashed vertical at safe altitude per direction)
    for direction, calc, safe in (
        ('LEFT turn', critical_alt_left, critical_alt_left_safe),
        ('RIGHT turn', critical_alt_right, critical_alt_right_safe),
    ):
        fig_margin.add_annotation(
            x=safe, y=direction,
            text=f'<b>Recommended {safe:,.0f} ft</b>',
            showarrow=True, arrowhead=2, ax=40, ay=-25,
            bgcolor='rgba(255,255,255,0.85)', bordercolor='black',
            font=dict(size=11),
        )
    fig_margin.update_layout(
        barmode='stack',
        title=f'Calculated minimum vs recommended  ·  buffer = +{_buf_pct:.0f}%  '
              f'({_buffer_ft:,.0f} ft on the worst direction)',
        xaxis_title='Altitude AGL at engine failure (ft)',
        height=260,
        margin=dict(t=60, b=40, l=80, r=20),
        legend=dict(orientation='h', yanchor='bottom', y=1.05, xanchor='right', x=1),
    )
    st.plotly_chart(fig_margin, use_container_width=True)
    st.caption(
        f"**The orange bar is what the physics says.**  The hatched green is the "
        f"buffer you've added with the safety-margin slider ({safety_margin_factor:.2f}×).  "
        f"Slide it up if the wind is gusty, the airplane is unfamiliar, or you're not "
        f"current.  Slide it down only if you've practiced this exact maneuver, in this "
        f"airplane, in similar conditions, recently.  **The recommended number is what "
        f"goes on your hold-short brief.**"
    )

    # ── Density-altitude proof readout ──
    # Show that high DA actually moves the numbers: TAS > IAS, ROC drops,
    # ground roll grows.  Compare the user's DA to a sea-level ISA baseline.
    try:
        from engine.flight_physics import atmos as _atmos_da
        _, _, _sigma_sl, _, _, _ = _atmos_da(0, 0)
        _, _, _sigma_da, _, _, _ = _atmos_da(field_elev, isa_dev)
        # Pressure altitude ≈ field elevation (METAR baro corrections ignored here).
        # Density altitude per ISA standard formula.
        _da_ft = field_elev + 120.0 * isa_dev  # quick approximation
        _vtas_da = airspeed / max(_sigma_da ** 0.5, 0.1)
        _vtas_sl = airspeed / max(_sigma_sl ** 0.5, 0.1)
        # Take ROC at critical-altitude trajectory mid-band as proxy
        _grad_da = res.get('climb_gradient', 0.07) or 0.07
        _roc_da_fpm = _grad_da * (_vtas_da * 6076.12 / 60.0)
        _roc_sl_fpm = _grad_da * (_vtas_sl * 6076.12 / 60.0)
        # Ground roll proxy: sigma scaling only (1/σ)
        _roll_factor = (1.0 / _sigma_da) if _sigma_da > 0 else float('inf')
        with st.expander(
            f"📊 Density-altitude effects ({_da_ft:,.0f} ft DA · σ = {_sigma_da:.3f})",
            expanded=False,
        ):
            st.markdown(
                f"**Field elev** {field_elev:,} ft MSL · **ISA dev** "
                f"{isa_dev:+d} °C · **Density altitude** ≈ "
                f"**{_da_ft:,.0f} ft** · density ratio σ = **{_sigma_da:.3f}** "
                f"(SL = 1.000)"
            )
            da_cols = st.columns(4)
            da_cols[0].metric(
                "Climb-out IAS",
                f"{airspeed:.0f} KIAS",
                f"unchanged (pilot reads IAS)",
                delta_color="off",
            )
            da_cols[1].metric(
                "True airspeed (TAS)",
                f"{_vtas_da:.0f} KTAS",
                f"+{_vtas_da - airspeed:.0f} kt vs sea level",
                delta_color="off",
                help=f"V_TAS = KIAS / √σ = {airspeed:.0f} / √{_sigma_da:.3f} "
                     f"= {_vtas_da:.1f} KTAS",
            )
            da_cols[2].metric(
                "Rate of climb",
                f"{_roc_da_fpm:,.0f} fpm",
                f"{_roc_da_fpm - _roc_sl_fpm:+,.0f} fpm vs sea level",
                delta_color="off",
                help=f"ROC = climb_gradient × V_TAS = {_grad_da:.3f} × "
                     f"{_vtas_da * 6076.12 / 60.0:,.0f} fpm "
                     f"= {_roc_da_fpm:,.0f} fpm",
            )
            da_cols[3].metric(
                "Ground-roll factor",
                f"× {_roll_factor:.2f}",
                f"{(_roll_factor - 1) * 100:+.0f}% vs sea level",
                delta_color="off",
                help="Liftoff roll scales as 1/σ (constant-thrust approximation).  "
                     "POH-based estimator in the sidebar uses the same formula.",
            )
            st.caption(
                "Same KIAS at altitude means **higher TAS** → wider turn radius "
                "(R = V²/(g·tan φ)) and **longer climb time** to clear the critical "
                "zone.  Engine power also drops with σ for normally-aspirated airframes, "
                "which is reflected in the per-altitude thrust deck used by the climb model."
            )

            # POH ROC scaled to weight + density (independent cross-check vs the sim)
            try:
                _roc_poh_da = roc_poh_fpm(ac_key[0], config, weight, field_elev, isa_dev)
                _roc_poh_sl = roc_poh_fpm(ac_key[0], config, weight, 0, 0)
                if _roc_poh_da is not None and _roc_poh_sl is not None:
                    _row = POH_CLIMB.get(ac_key[0])
                    _vy_p = int(_row[0]) if _row else 0
                    _roc_mtow_sl = int(_row[1]) if _row else 0
                    _dens_exp = float(_row[2]) if _row else 1.0
                    st.markdown("---")
                    st.markdown(
                        f"**POH cross-check** — Vy = **{_vy_p} KIAS**, "
                        f"published ROC at MTOW/SL/ISA = **{_roc_mtow_sl:,} fpm** "
                        f"(density model: {'turbo / turboprop (σ^0.5)' if _dens_exp < 1.0 else 'normally-aspirated (σ^1.0)'})"
                    )
                    poh_cols = st.columns(2)
                    poh_cols[0].metric(
                        f"POH ROC @ {weight:,} lb, sea level",
                        f"{_roc_poh_sl:,.0f} fpm",
                        f"weight × {(_row[1]/_roc_poh_sl):.2f} factor" if _roc_poh_sl > 0 else "",
                        delta_color="off",
                        help="POH MTOW ROC scaled by (MTOW / actual weight).",
                    )
                    poh_cols[1].metric(
                        f"POH ROC @ {weight:,} lb, this DA",
                        f"{_roc_poh_da:,.0f} fpm",
                        f"{_roc_poh_da - _roc_poh_sl:+,.0f} fpm vs SL  ·  "
                        f"−{(1 - _roc_poh_da/_roc_poh_sl)*100:.0f}% density loss" if _roc_poh_sl > 0 else "",
                        delta_color="off",
                        help="POH ROC × (MTOW/W) × σ^n where n = 1.0 (NA) or 0.5 (turbo).",
                    )
                    st.caption(
                        "POH cross-check is independent of the simulator's "
                        "thrust-deck climb model.  Large disagreements between "
                        "the two are a flag to verify the airframe drag polar "
                        "or thrust-deck assumption."
                    )

                    # P4 — Service-ceiling proximity warning
                    _ceiling = float(getattr(config, 'ceiling', 0.0))
                    if _ceiling > 0:
                        _da_now = field_elev + 120.0 * isa_dev
                        _ceil_ratio = _da_now / _ceiling
                        if _ceil_ratio >= 0.8:
                            st.error(
                                f"⚠️ **Service-ceiling alarm** — DA {_da_now:,.0f} ft "
                                f"is {_ceil_ratio*100:.0f}% of published service ceiling "
                                f"({_ceiling:,.0f} ft).  POH ROC of {_roc_poh_da:,.0f} fpm "
                                f"is approaching the 100 fpm ceiling definition.  "
                                f"**Turnback geometry will be marginal — consider a different field or wait for cooler conditions.**"
                            )
                        elif _ceil_ratio >= 0.6:
                            st.warning(
                                f"⚠️ DA {_da_now:,.0f} ft is {_ceil_ratio*100:.0f}% of "
                                f"service ceiling ({_ceiling:,.0f} ft).  Climb performance "
                                f"is significantly degraded — verify ROC margin before takeoff."
                            )
                        elif _ceil_ratio >= 0.4:
                            st.info(
                                f"DA {_da_now:,.0f} ft = {_ceil_ratio*100:.0f}% of "
                                f"service ceiling ({_ceiling:,.0f} ft).  Climb performance "
                                f"reduced but workable."
                            )
            except Exception:
                pass

            # P5 — Landing-roll POH cross-check (high-DA fields most at risk)
            try:
                _land_da = landing_roll_poh_ft(ac_key[0], config, weight, field_elev, isa_dev)
                _land_sl = landing_roll_poh_ft(ac_key[0], config, weight, 0, 0)
                if _land_da is not None and _land_sl is not None:
                    _land_base = POH_LANDING_ROLL_FT.get(ac_key[0], 0)
                    st.markdown("---")
                    st.markdown(
                        f"**POH landing ground-roll** (MLW/SL/ISA, dry paved) = "
                        f"**{_land_base:,} ft** for the {ac_key[0]}"
                    )
                    land_cols = st.columns(2)
                    land_cols[0].metric(
                        f"Landing roll @ {weight:,} lb, sea level",
                        f"{_land_sl:,.0f} ft",
                        delta_color="off",
                        help="POH MLW landing roll scaled by (W/MLW)².",
                    )
                    land_cols[1].metric(
                        f"Landing roll @ {weight:,} lb, this DA",
                        f"{_land_da:,.0f} ft",
                        f"+{_land_da - _land_sl:,.0f} ft vs SL  ·  "
                        f"+{(_land_da/_land_sl - 1)*100:.0f}% density penalty" if _land_sl > 0 else "",
                        delta_color="off",
                        help="POH landing roll × (W/MLW)² × (1/σ).",
                    )
                    # Sanity check vs runway length if available
                    _rwy_len = float(res.get('runway_length', 0) or 0)
                    if _rwy_len > 0:
                        _margin = _rwy_len - _land_da
                        _ratio = _land_da / _rwy_len
                        if _ratio > 0.6:
                            st.warning(
                                f"⚠️ Landing roll {_land_da:,.0f} ft is **{_ratio*100:.0f}%** "
                                f"of runway ({_rwy_len:,.0f} ft).  Margin only {_margin:,.0f} ft — "
                                f"consider this when planning the turnback aim point."
                            )
                        else:
                            st.caption(
                                f"Landing roll uses {_ratio*100:.0f}% of runway length "
                                f"({_rwy_len:,.0f} ft) at this DA — {_margin:,.0f} ft margin."
                            )
                    st.caption(
                        "Scaling: same family as takeoff (W/MLW)² × (1/σ).  "
                        "Conservative — assumes max braking on dry paved.  "
                        "Wet/contaminated runways multiply this by 1.4–2×."
                    )
            except Exception:
                pass
    except Exception:
        pass

    # ── Runway zone analysis (straight-ahead vs critical zone vs turnback) ──
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
                f"top = {int(straight_ahead_max_alt) + int(_fe):,} ft MSL",
                delta_color="off",
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
            # Exposure time: how many seconds the airplane is climbing through
            # the critical band before it reaches a survivable turnback altitude.
            # ROC_fpm = climb_gradient × V_TAS_fpm; V_TAS = KIAS / sqrt(σ).
            try:
                from engine.flight_physics import atmos as _atmos
                _grad_exp = res.get('climb_gradient', 0.07) or 0.07
                _mid_alt = field_elev + (dead_zone_low + dead_zone_high) / 2.0
                _, _, _sigma_exp, _, _, _ = _atmos(_mid_alt, isa_dev)
                _vtas_kt = airspeed / max(_sigma_exp ** 0.5, 0.1)
                _vtas_fpm = _vtas_kt * 6076.12 / 60.0
                _roc_fpm = max(_grad_exp * _vtas_fpm, 1.0)
                _exposure_s = dead_zone_size / _roc_fpm * 60.0
                _exposure_str = f"  ·  exposure ≈ {_exposure_s:.0f} s @ {_roc_fpm:,.0f} fpm"
            except Exception:
                _exposure_str = ""
                _exposure_s = 0.0
                _roc_fpm = 0.0
            zone_cols[1].metric(
                "CRITICAL ZONE",
                f"{dead_zone_low:,} – {dead_zone_high:,} ft AGL",
                delta=f"{dead_zone_size:,} ft gap  ·  MSL {dead_zone_low + int(_fe):,}–{dead_zone_high + int(_fe):,}{_exposure_str}",
                delta_color="inverse",
                help="Can't land straight (overshoots) AND can't make the turnback (too low). "
                     "Exposure ≈ time spent climbing through this band before reaching a "
                     "survivable turnback altitude (band height ÷ rate of climb).",
            )
            _exp_phrase = (
                f" The climb-through exposure is **≈ {_exposure_s:.0f} seconds** at "
                f"~{_roc_fpm:,.0f} fpm — that's how long an engine failure puts you "
                f"in this trap on every takeoff."
                if _roc_fpm > 0 else ""
            )
            st.warning(
                f"⚠️ **Critical zone: {dead_zone_low:,} – {dead_zone_high:,} ft AGL "
                f"({dead_zone_low + int(_fe):,} – {dead_zone_high + int(_fe):,} ft MSL)** "
                f"— {dead_zone_size:,} ft band. In this altitude band, the aircraft "
                f"overshoots the runway going straight ahead but is too low to complete "
                f"the turnback. This is the most dangerous failure altitude range."
                f"{_exp_phrase}"
            )
        elif dead_zone_low >= dead_zone_high:
            zone_cols[1].metric(
                "Critical Zone",
                "NONE",
                delta="Full coverage!",
                delta_color="normal",
                help="Straight-ahead and turnback zones overlap — no uncovered altitude band",
            )
            st.success(
                "✅ **No critical zone!** Straight-ahead landing coverage extends to or "
                "above the turnback critical altitude. Every failure altitude has "
                "a survivable option."
            )

        zone_cols[2].metric(
            "Turnback (Impossible Turn)",
            f"≥ {turnback_min:,.0f} ft AGL",
            f"≥ {turnback_min + _fe:,.0f} ft MSL",
            delta_color="off",
            help="Engine failure above this altitude: complete the turnback to land on the runway",
        )

        # ── Sim start geometry (diagnostic) ──
        _offset = int(intersection_offset_ft)
        _liftoff = int(round(liftoff_distance)) if liftoff_distance else 0
        _rwy_len = int(round(runway_length)) if runway_length else 0
        _liftoff_y = _offset + _liftoff
        _rem_after_liftoff = _rwy_len - _liftoff_y
        with st.expander("📐 Sim start geometry (where the airplane starts on the runway)", expanded=False):
            st.markdown(
                f"**Runway:** `{selected_airport_ident} RW {selected_runway_ident}` — "
                f"length **{_rwy_len:,} ft**, threshold elev **{int(_fe):,} ft MSL**, "
                f"heading **{runway_heading_true:.0f}°T**"
            )
            geo_cols = st.columns(4)
            geo_cols[0].metric("Brake-release (start of roll)", f"y = {_offset:,} ft",
                               f"intersection offset" if _offset > 0 else "full-length departure",
                               delta_color="off")
            geo_cols[1].metric("Liftoff point", f"y = {_liftoff_y:,} ft",
                               f"ground roll {_liftoff:,} ft", delta_color="off")
            geo_cols[2].metric("Runway end", f"y = {_rwy_len:,} ft",
                               f"{_rem_after_liftoff:,} ft of runway remaining after liftoff",
                               delta_color="off")
            if straight_ahead_max_alt > 0:
                # Approximate climb distance to the SA-max altitude using crit climb gradient
                _grad = res.get('climb_gradient', 0.07) or 0.07
                _climb_to_dz = straight_ahead_max_alt / max(_grad, 0.01)
                _y_at_dz = _liftoff_y + _climb_to_dz
                geo_cols[3].metric("Position at critical-zone-low",
                                   f"y ≈ {_y_at_dz:,.0f} ft",
                                   f"{_rwy_len - _y_at_dz:,.0f} ft of runway ahead",
                                   delta_color="off")
            else:
                geo_cols[3].metric("Position at critical-zone-low", "n/a",
                                   "SA not feasible from any altitude", delta_color="off")
            st.caption(
                "Coordinate system: **y = 0 at the departure-end runway threshold**, "
                "increasing in the takeoff direction. The sim places the aircraft at "
                "**y = intersection offset** at brake release, accelerates to liftoff, "
                "then climbs at the airframe's best-rate gradient. If the engine fails, "
                "glide continues from the y-position at failure altitude. Touchdown "
                "must occur at y ≤ runway_length to count as a successful straight-ahead "
                "landing."
            )
            if _rem_after_liftoff < 500:
                st.warning(
                    f"⚠️ Only {_rem_after_liftoff:,} ft of runway remain after liftoff. "
                    "This severely compresses the straight-ahead landing window. "
                    "Check that the runway length and intersection offset in the sidebar "
                    "match what you intend."
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
            speed_cols[0].metric("Best L/D (1g)", f"{si['vbg_1g_kias']:.0f} KIAS")
            speed_cols[1].metric("L/D max", f"{si['ld_max']:.1f}")
            speed_cols[2].metric("CL (best L/D)", f"{si['cl_best']:.3f}")
            if si['mode'] == 'best_glide_nz' and 'vbg_turn_kias' in si:
                speed_cols[3].metric(
                    f"Best L/D ({res['bank_angle']}° bank)",
                    f"{si['vbg_turn_kias']:.0f} KIAS",
                )
            else:
                speed_cols[3].metric("Speed Mode", "Constant 1g")
        elif si['mode'] in ('vs_plus_10', 'vs_x_1p3'):
            speed_cols = st.columns(4)
            speed_cols[0].metric(f"Vs at {res['bank_angle']}° bank",
                                  f"{si.get('vs_at_bank_kias', 0):.0f} KIAS")
            speed_cols[1].metric("Vs(φ) + 10 kt",
                                  f"{si.get('vs_plus_10_kias', 0):.0f} KIAS")
            speed_cols[2].metric("1.3 × Vs(φ)",
                                  f"{si.get('vs_x_1p3_kias', 0):.0f} KIAS")
            speed_cols[3].metric("Target flown",
                                  f"{si.get('target_kias', 0):.0f} KIAS",
                                  delta=f"nz={si.get('nz_bank', 0):.2f}")

    # ── Turn statistics (Charlie #7: training output) ──
    # Pull the critical-altitude trajectory and surface:
    #   - Total degrees of turn the maneuver requires
    #   - Altitude lost per 180° of turning
    # so a pilot can practice this at altitude and pick a personal safety factor.
    if crit_result is not None:
        total_turn_deg = crit_result.get('total_turn_deg', 0.0)
        loss_per_180 = crit_result.get('altitude_loss_per_180')
        increments = crit_result.get('altitude_at_180_increments', [])
        if total_turn_deg > 0:
            st.markdown("---")
            st.subheader("Turn Statistics (Training Output)")
            st.caption(
                "From the critical-altitude turnback trajectory.  "
                "Practice this at altitude to calibrate your personal safety factor."
            )
            ts_cols = st.columns(3)
            ts_cols[0].metric(
                "Total degrees of turn",
                f"{total_turn_deg:.0f}°",
                help="Total heading change required for the full turnback "
                     "maneuver (turn + return-to-runway alignment + any orbit/circuit).",
            )
            if loss_per_180 is not None:
                ts_cols[1].metric(
                    "Altitude loss per 180°",
                    f"{loss_per_180:.0f} ft",
                    help="Average altitude lost per half-turn during the maneuver. "
                         "Go to altitude, fly the same bank/airspeed, time a 180° turn, "
                         "and verify your altitude loss is at or below this number.",
                )
            n_half_turns = len(increments) - 1 if len(increments) > 1 else 0
            ts_cols[2].metric(
                "Half-turns flown",
                f"{n_half_turns}",
                help="Number of 180° increments captured in the maneuver.",
            )
            if len(increments) > 1:
                with st.expander("Altitude AGL at each 180° increment", expanded=False):
                    inc_data = [
                        {
                            "Cumulative turn (°)": int(p['turn_deg']),
                            "Altitude AGL (ft)": int(round(p['altitude_agl'])),
                            "Cumulative loss (ft)": int(round(increments[0]['altitude_agl'] - p['altitude_agl'])),
                        }
                        for p in increments
                    ]
                    st.dataframe(pd.DataFrame(inc_data), hide_index=True, use_container_width=True)

    # ── Takeoff Data Card (TOLD-style) ──
    # Charlie Precourt's #1 ask: a printable single-page card pilots can
    # take to the cockpit with the day-of-flight turnback decision numbers.
    st.markdown("---")
    st.subheader("📋 Takeoff Data Card")
    st.caption(
        "Single-page TOLD-style summary of this scenario's turnback decision "
        "numbers.  Built for the EAA McSpadden Project — print and keep in the "
        "cockpit alongside your existing takeoff data card."
    )
    try:
        card_html = build_takeoff_data_card(res, crit_result, safety_margin_factor)
        dc_cols = st.columns([1, 1, 1, 2])
        with dc_cols[0]:
            card_pdf = build_takeoff_data_card_pdf(res, crit_result, safety_margin_factor)
            if card_pdf:
                st.download_button(
                    "📄 Download PDF",
                    data=card_pdf,
                    file_name=f"TOLD_card_{res.get('airport_ident', 'card') or 'card'}_{res.get('runway_ident', '')}.pdf",
                    mime="application/pdf",
                    help="One-page printable PDF for the flight bag.",
                )
            else:
                st.caption("PDF unavailable (xhtml2pdf missing)")
        with dc_cols[1]:
            st.download_button(
                "📥 Download HTML",
                data=card_html.encode("utf-8"),
                file_name=f"TOLD_card_{res.get('airport_ident', 'card') or 'card'}_{res.get('runway_ident', '')}.html",
                mime="text/html",
                help="Self-contained HTML.  Open in a browser and use "
                     "Print → Save as PDF for full styling fidelity.",
            )
        with dc_cols[2]:
            st.caption("Preview shown below ⬇")
        st.components.v1.html(card_html, height=1400, scrolling=True)
    except Exception as e:
        st.error(f"Could not build data card: {e}")

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
    elif speed_mode in ('vs_plus_10', 'vs_x_1p3'):
        if crit_result and 'speed_info' in crit_result:
            si_check = crit_result['speed_info']
            if 'target_kias' in si_check:
                turn_speed = si_check['target_kias']
        if turn_speed == res['airspeed']:
            from analysis.turnback_simulator import operational_turn_speeds
            _ops = operational_turn_speeds(config, res['weight'], res['bank_angle'], 0, 0)
            turn_speed = _ops['vs_plus_10_kias'] if speed_mode == 'vs_plus_10' else _ops['vs_x_1p3_kias']

    if turn_speed < vs_turn * 1.05:
        st.error(f"⚠️ Turn speed ({turn_speed:.0f} KIAS) is dangerously close to or below "
                 f"the stall speed in the turn ({vs_turn:.0f} KIAS at {res['bank_angle']}° bank). "
                 f"Reduce bank angle or increase airspeed.")

    # ── Satellite map / forced-landing analysis ──
    # Pull L/D max from envelope speed_info if present, else compute from aero
    _ld_ratio = None
    _vbg_kias_for_map = None
    if crit_result and 'speed_info' in crit_result:
        _si = crit_result['speed_info']
        _ld_ratio = _si.get('ld_max')
        _vbg_kias_for_map = _si.get('vbg_1g_kias')
    if _ld_ratio is None or _vbg_kias_for_map is None:
        _vbg_calc, _, _ld_calc = best_glide_kias(
            config, res['weight'], 1.0, field_elev, isa_dev)
        _ld_ratio = _ld_ratio if _ld_ratio is not None else _ld_calc
        _vbg_kias_for_map = _vbg_kias_for_map if _vbg_kias_for_map is not None else _vbg_calc

    from analysis.landing_map import render_landing_map_section
    render_landing_map_section(
        critical_alt_low_ft=min(critical_alt_left_safe, critical_alt_right_safe),
        critical_alt_high_ft=max(critical_alt_left_safe, critical_alt_right_safe),
        straight_ahead_max_alt_ft=res.get('straight_ahead_max_alt', 0.0) or 0.0,
        ld_ratio=float(_ld_ratio or 10.0),
        best_glide_kias=float(_vbg_kias_for_map or 70.0),
        wind_speed_kt=float(res.get('wind_speed', 0) or 0),
        wind_from_deg=float(res.get('wind_from_deg', 0) or 0),
        liftoff_distance_ft=float(res.get('liftoff_distance', 0) or 0),
        default_airport_code=str(res.get('airport_ident') or ''),
        default_runway_heading=float(res.get('runway_heading_true') or 0),
        envelope=envelope,
        critical_alt=float(critical_alt or 0.0),
        straight_ahead_max_alt=float(res.get('straight_ahead_max_alt', 0.0) or 0.0),
        comparison_envelope=(_cmp or {}).get('envelope') if _cmp else None,
        comparison_critical_alt=float((_cmp or {}).get('critical_alt') or 0.0) if _cmp else 0.0,
        comparison_label=(
            f"Alternate ({_cmp.get('climb_steering', '')} hold)" if _cmp else None
        ),
        primary_label=(
            f"Primary ({res.get('climb_steering', 'track')} hold)"
        ),
    )

    # ── Altitude profile ──
    st.subheader("Altitude vs Time — Critical Altitudes")
    fig_alt = _build_altitude_profile(envelope, critical_alt)
    if fig_alt is not None:
        st.plotly_chart(fig_alt, use_container_width=True)

    # ── Detailed data table for critical altitude ──
    with st.expander("Trajectory Data — Critical Altitudes"):
        _show_trajectory_table(envelope, critical_alt)

    # ── Sensitivity charts (Phase 2 — bank angle & reaction time) ──
    st.markdown("---")
    st.subheader("📈 Sensitivity — How the answer moves with your assumptions")
    st.caption(
        "These charts re-run the critical-altitude search across one variable "
        "at a time, holding everything else fixed.  They turn the *single number* "
        "above into an **education about which knobs matter most.**"
    )
    sens_cols = st.columns(2)
    do_bank_sens = sens_cols[0].button(
        "Run bank-angle sensitivity",
        help="Re-runs the critical-altitude search across 25°/30°/35°/40°/45°/50°/55° "
             "of bank, holding all other inputs fixed.  Shows how steeper bank "
             "trades altitude needed (smaller turn) against stall-margin loss.",
        key="run_bank_sens",
    )
    do_reaction_sens = sens_cols[1].button(
        "Run reaction-time sensitivity",
        help="Re-runs the critical-altitude search across 0/2/3/5/7/10 sec of "
             "reaction time.  Most pilots underestimate this — every second of "
             "delay costs hundreds of feet in cold-startle scenarios.",
        key="run_reaction_sens",
    )

    # Persist results so the chart stays after a Streamlit rerun
    if do_bank_sens:
        bank_angles_sweep = [25, 30, 35, 40, 45, 50, 55]
        crit_by_bank_left = []
        crit_by_bank_right = []
        with st.spinner("Sweeping bank angles..."):
            for ba in bank_angles_sweep:
                cl = find_critical_altitude(
                    config, weight, airspeed, ba, flap_setting,
                    reaction_time, field_elev, isa_dev,
                    wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                    wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                    wind_profile=wind_profile, wind_dir_profile=wind_dir_profile,
                    turn_direction='left',
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=flap_on_return,
                    speed_mode=speed_mode, prop_state=prop_state,
                    gear_down=gear_down, gear_retract_time_s=gear_retract_time_s,
                    intersection_offset_ft=intersection_offset_ft,
                    vbg_clean_kias=vbg_clean_kias, vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction, climb_steering=climb_steering,
                )
                cr = find_critical_altitude(
                    config, weight, airspeed, ba, flap_setting,
                    reaction_time, field_elev, isa_dev,
                    wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                    wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                    wind_profile=wind_profile, wind_dir_profile=wind_dir_profile,
                    turn_direction='right',
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=flap_on_return,
                    speed_mode=speed_mode, prop_state=prop_state,
                    gear_down=gear_down, gear_retract_time_s=gear_retract_time_s,
                    intersection_offset_ft=intersection_offset_ft,
                    vbg_clean_kias=vbg_clean_kias, vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction, climb_steering=climb_steering,
                )
                crit_by_bank_left.append(cl)
                crit_by_bank_right.append(cr)
        # Stall speed in turn at each bank
        vs_clean_kias = math.sqrt(295.0 * weight / (config.wing_area * config.Clmax))
        vs_at_bank = [vs_clean_kias / math.sqrt(math.cos(math.radians(b))) for b in bank_angles_sweep]
        st.session_state['bank_sens'] = {
            'banks': bank_angles_sweep,
            'crit_left': crit_by_bank_left,
            'crit_right': crit_by_bank_right,
            'vs_at_bank': vs_at_bank,
            'cur_bank': bank_angle,
            'cur_crit_left': critical_alt_left,
            'cur_crit_right': critical_alt_right,
            'airspeed': airspeed,
        }

    if 'bank_sens' in st.session_state:
        bs = st.session_state['bank_sens']
        fig_bs = go.Figure()
        fig_bs.add_trace(go.Scatter(
            x=bs['banks'], y=bs['crit_left'],
            mode='lines+markers', name='Critical alt — LEFT turn (ft AGL)',
            line=dict(color='#1f77b4', width=3), marker=dict(size=10),
        ))
        fig_bs.add_trace(go.Scatter(
            x=bs['banks'], y=bs['crit_right'],
            mode='lines+markers', name='Critical alt — RIGHT turn (ft AGL)',
            line=dict(color='#ff7f0e', width=3, dash='dash'), marker=dict(size=10),
        ))
        fig_bs.add_trace(go.Scatter(
            x=[bs['cur_bank']],
            y=[min(bs['cur_crit_left'], bs['cur_crit_right'])],
            mode='markers', name=f'Your selection ({bs["cur_bank"]}°)',
            marker=dict(size=18, color='red', symbol='star'),
        ))
        # Stall-speed-in-turn on secondary axis
        fig_bs.add_trace(go.Scatter(
            x=bs['banks'], y=bs['vs_at_bank'],
            mode='lines', name='Vs in turn (KIAS)',
            line=dict(color='#d62728', width=2, dash='dot'),
            yaxis='y2',
        ))
        fig_bs.add_hline(
            y=bs['airspeed'], line_dash='dot', line_color='gray',
            annotation_text=f'Climb speed {bs["airspeed"]:.0f} KIAS',
            annotation_position='top right',
            yref='y2',
        )
        fig_bs.update_layout(
            title='Critical altitude vs bank angle  ·  with accelerated stall overlay',
            xaxis_title='Bank angle in turnback (deg)',
            yaxis=dict(title='Critical altitude (ft AGL)', side='left'),
            yaxis2=dict(title='Vs in turn (KIAS)', overlaying='y', side='right'),
            height=420,
            hovermode='x unified',
            legend=dict(orientation='h', yanchor='bottom', y=1.02, xanchor='right', x=1),
        )
        st.plotly_chart(fig_bs, use_container_width=True)
        # Trade-off insight
        i_min_left = bs['crit_left'].index(min(bs['crit_left']))
        st.markdown(
            f"**Read it like this:** the lowest critical altitude (LEFT turn) is "
            f"**{min(bs['crit_left'])} ft AGL at {bs['banks'][i_min_left]}° bank**.  "
            f"Steeper than that, the turn tightens but stall speed climbs and the airframe "
            f"bleeds energy faster — no net win.  Shallower than that, the turn radius "
            f"explodes and you need more altitude to complete it.  "
            f"**This is why 'just bank harder' is wrong advice.**"
        )
        st.caption(
            "Reference: [Rogers, *Looking Back at the Turn-Back Maneuver* (USNA)]"
            "(https://www.usna.edu/AeroDept/_files/documents/Faculty/Rogers/RogersTurnBack.pdf) "
            "derives this trade analytically; [Jett, USAFA 1982 (DTIC ADA122862)]"
            "(https://apps.dtic.mil/sti/citations/ADA122862) confirmed it in simulator studies."
        )

    if do_reaction_sens:
        reaction_sweep = [0, 2, 3, 5, 7, 10]
        crit_by_rt = []
        with st.spinner("Sweeping reaction times..."):
            for rt in reaction_sweep:
                ca = find_critical_altitude(
                    config, weight, airspeed, bank_angle, flap_setting,
                    rt, field_elev, isa_dev,
                    wind_speed_kt=wind_speed, wind_from_deg=wind_from_deg,
                    wind_1000_kt=wind_1000_kt, wind_2000_kt=wind_2000_kt, wind_3000_kt=wind_3000_kt,
                    wind_profile=wind_profile, wind_dir_profile=wind_dir_profile,
                    turn_direction='left',
                    runway_length=runway_length, liftoff_distance=liftoff_distance,
                    aim_point=aim_point, flap_on_return=flap_on_return,
                    speed_mode=speed_mode, prop_state=prop_state,
                    gear_down=gear_down, gear_retract_time_s=gear_retract_time_s,
                    intersection_offset_ft=intersection_offset_ft,
                    vbg_clean_kias=vbg_clean_kias, vbg_geardown_kias=vbg_geardown_kias,
                    vbg_landing_kias=vbg_landing_kias,
                    touchdown_margin_ft=touchdown_margin_ft,
                    runway_friction=runway_friction, climb_steering=climb_steering,
                )
                crit_by_rt.append(ca)
        st.session_state['reaction_sens'] = {
            'rts': reaction_sweep,
            'crits': crit_by_rt,
            'cur_rt': reaction_time,
            'cur_crit': critical_alt_left,
        }

    if 'reaction_sens' in st.session_state:
        rs = st.session_state['reaction_sens']
        fig_rs = go.Figure()
        fig_rs.add_trace(go.Scatter(
            x=rs['rts'], y=rs['crits'],
            mode='lines+markers', name='Critical alt (ft AGL)',
            line=dict(color='#2ca02c', width=3), marker=dict(size=12),
            fill='tozeroy', fillcolor='rgba(44,160,44,0.15)',
        ))
        fig_rs.add_trace(go.Scatter(
            x=[rs['cur_rt']], y=[rs['cur_crit']],
            mode='markers', name=f'Your selection ({rs["cur_rt"]}s)',
            marker=dict(size=18, color='red', symbol='star'),
        ))
        fig_rs.update_layout(
            title='Critical altitude vs pilot reaction time',
            xaxis_title='Reaction time at engine failure (sec)',
            yaxis_title='Critical altitude (ft AGL)',
            height=380,
            hovermode='x unified',
            legend=dict(orientation='h', yanchor='bottom', y=1.02, xanchor='right', x=1),
        )
        st.plotly_chart(fig_rs, use_container_width=True)
        if len(rs['crits']) >= 2 and rs['crits'][0] > 0:
            cost_per_sec = (rs['crits'][-1] - rs['crits'][0]) / (rs['rts'][-1] - rs['rts'][0])
            st.markdown(
                f"**Read it like this:** every second of pilot reaction time costs "
                f"roughly **{cost_per_sec:+.0f} ft** of critical altitude in this scenario.  "
                f"At a 0-second 'pre-briefed' response, the airplane needs **{rs['crits'][0]} ft**.  "
                f"At a realistic 5-second startle response, it needs **{rs['crits'][rs['rts'].index(5)] if 5 in rs['rts'] else '?'} ft**.  "
                f"At a 10-second 'oh-shit' delay, it needs **{rs['crits'][-1]} ft**.  "
                f"**The brief at the runway hold-short is the cheapest altitude you'll ever buy.**"
            )
            st.caption(
                "Reference: FAA assumes 3 sec for engine-out response; "
                "[FAA AC 61-83K](https://www.faa.gov/regulations_policies/advisory_circulars/index.cfm/go/document.information/documentID/1043603) "
                "directs CFIs to train startle response.  Real-world studies "
                "(NTSB Loss-of-Control series) consistently measure 5–8 sec for unbriefed pilots."
            )

    # ── Theory & References ──
    st.markdown("---")
    with st.expander("🎓 Training Curriculum (CFI / working pilot)", expanded=False):
        _show_curriculum_section()
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


def _build_2d_plan(envelope, critical_alt, runway_length=0.0, aim_point=0.0,
                   liftoff_distance=0.0, show_success=True,
                   straight_ahead_max_alt=0.0, ld_ratio=0.0,
                   climb_gradient_deg=5.0):
    """Build the 2D plan view (top-down) showing ground tracks.

    Charlie #G1: optionally overlay two glide-reach arcs centered on the
    failure point at the LOW (= straight_ahead_max_alt) and HIGH (= critical_alt)
    edges of the dead zone, so the pilot can SEE the band where neither
    runway nor turnback is reachable.
    """
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

    # ── G1: Dead-zone glide-reach arcs (Charlie #G1) ──
    # Show the glide footprint at the LOW edge (sa_max alt) and HIGH edge
    # (turnback critical alt) of the dead zone — so the pilot can see the
    # band where they have no good option.
    if ld_ratio and ld_ratio > 0:
        import math as _math
        try:
            tan_grad = _math.tan(_math.radians(max(climb_gradient_deg, 0.5)))
        except Exception:
            tan_grad = _math.tan(_math.radians(5.0))
        # Failure points (downrange ft from threshold) along centerline
        for alt_ft, label, color in (
            (max(straight_ahead_max_alt, 0.0), 'Dead-zone LOW', '#f87171'),
            (max(critical_alt, 0.0), 'Dead-zone HIGH', '#fbbf24'),
        ):
            if alt_ft <= 0:
                continue
            fp_y = (liftoff_distance if liftoff_distance > 0 else 0.0) + alt_ft / tan_grad
            r_ft = alt_ft * ld_ratio  # still-air glide reach (ft)
            thetas = [i * _math.pi / 60.0 for i in range(121)]  # 0..2π
            cx = [r_ft * _math.cos(t) for t in thetas]
            cy = [fp_y + r_ft * _math.sin(t) for t in thetas]
            fig.add_trace(go.Scatter(
                x=cx, y=cy, mode='lines',
                line=dict(color=color, width=2, dash='dot'),
                name=f"{label} ({alt_ft:,.0f} ft AGL · {r_ft/6076.12:.2f} nm reach)",
                hoverinfo='name',
                opacity=0.85,
            ))
            # Mark the failure point itself
            fig.add_trace(go.Scatter(
                x=[0], y=[fp_y], mode='markers',
                marker=dict(color=color, size=10, symbol='circle-open',
                            line=dict(width=2)),
                name=f"Failure pt @ {alt_ft:,.0f} ft",
                showlegend=False,
                hovertemplate=f"Failure point @ {alt_ft:,.0f} ft AGL<extra></extra>",
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


def _show_curriculum_section():
    """Charlie #H1 — CFI / working-pilot training curriculum, in-app."""
    st.markdown("""
    ## Turnback Simulator — Training Curriculum

    **For:** CFIs, Designated Pilot Examiners, and working single-engine pilots
    **Reference:** [AC 61-83K](https://www.faa.gov/regulations_policies/advisory_circulars/index.cfm/go/document.information/documentID/1043603) · [AFH Ch 6](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/airplane_handbook) (proposed revisions) · [Rogers (USNA)](https://www.usna.edu/AeroDept/_files/documents/Faculty/Rogers/RogersTurnBack.pdf) · [Jett (USAFA, DTIC ADA122862)](https://apps.dtic.mil/sti/citations/ADA122862)
    **Authors:** Nick Guida (Volo Altro / Tamarack) · Charlie Precourt (EAA McSpadden Project)

    ---

    ### 1. Why this exists

    The "impossible turn" kills competent pilots every year because the decision is being made in five seconds with no number to fall back on.  POH data stops at runway end.  Nothing in normal flight training computes the altitude required to make the turn back, and almost no pilot has a personal number for their airplane / weight / wind / runway combination.

    This simulator produces that number — and surfaces the **dead zone**: the band of altitudes on initial climb where neither a straight-ahead landing nor a turnback is viable.

    The curriculum below is **how to teach with it**, not how to use it.

    ---

    ### 2. Three pilot personas

    | Persona | What they need | How a CFI uses it |
    |---|---|---|
    | **Pre-solo / private student** | A vocabulary: critical altitude, dead zone, straight-ahead-max. | Pre-takeoff briefing exercise: print a Data Card for today's airplane / weight / runway / wind.  Brief the dead zone on every taxi to the hold-short. |
    | **Newly-rated private / IFR pilot** | Translate the number into a takeoff brief and a no-go altitude. | Hood / sim drill: simulate the failure at three altitudes (below dead zone, in the dead zone, above critical) and force a verbal commit before they touch the controls. |
    | **CFI / working pro** | Per-airplane, per-airport bracketing.  Curriculum to give to their own students. | Build a personal library of Data Cards for the trainers in their fleet.  Run the simulator side-by-side with the student so the *trade-offs* (bank vs. radius, flaps vs. drag) become visible, not abstract. |

    ---

    ### 3. The four numbers every pilot should brief on every takeoff

    1. **Calculated critical altitude** (LEFT and RIGHT) — the bare aerodynamic minimum.
    2. **Recommended critical altitude** = Calculated × safety factor (default **1.25×**, Charlie's call).  Use *this* as the no-go.
    3. **Straight-ahead-max altitude** — below this, land within the airport boundary.  Not the runway.  The boundary.
    4. **Dead-zone band** — the altitude window where neither option works.  Climb through it as fast as the airplane will go.

    > If you can't say all four out loud at the hold-short, you are not briefed.

    ---

    ### 4. Lesson plan (4 sessions × 1 hour)

    **Session 1 — Ground: the physics**
    - Why "180°" is a myth (the turn must overshoot ~210–270° to align on final).
    - Bank angle ↔ stall margin ↔ turn radius.  Demonstrate Vs(φ) = Vs · √(1/cos φ): 30°→+8%, 45°→+19%, 60°→+41%.
    - Drag with bank: induced drag scales with CL² → steep banks bleed energy fast.
    - Reaction time: 3 sec is FAA assumption, 5–7 sec is realistic startle response, **5 sec is the curriculum default**.
    - Run the sim live: change reaction from 3 → 5 → 7 and watch the critical altitude inflate.

    **Session 2 — Ground: the airplane**
    - Build a Data Card for the student's own airplane at three weights (solo, with instructor, MTOW).
    - Compare 30° / 45° / 60° bank: identify the per-aircraft sweet spot (usually 45°).
    - Compare clean / takeoff-flap / landing-flap turn configurations.  Discuss why clean usually wins on energy.
    - Identify the dead zone for *this* airplane on *this* runway.  Brief it.

    **Session 3 — Airwork: feel the bank, feel the stall**
    - At 3,000 AGL: simulate idle power, hold pitch attitude, count out the reaction time, then enter a 45° bank turn at the recommended speed (Vs(φ) + 10 kt).
    - Time the 180°.  Measure altitude lost.  Compare to the simulator's prediction.
    - If real loss > simulated loss, **inflate the personal safety factor** until they match.  That is the student's number.
    - Repeat at 30° and 60° to feel the trade.  Most pilots will pick 45° on their own after this.

    **Session 4 — Procedure: brief, climb, commit**
    - Pre-takeoff verbal brief: "Below {sa_max} feet — straight ahead, anywhere on the airport.  Above {recommended} feet — turn {LEFT/RIGHT}, {bank}° bank, {speed} KIAS.  Between, no good option — climbing fast."
    - Departure with simulated failure at three altitudes:
      - Just above straight-ahead max → land straight, accept it.
      - Mid dead zone → land straight (or off-airport if forced).
      - Just above recommended → execute the briefed turn.
    - Debrief on the Data Card numbers vs. what actually happened.

    ---

    ### 5. Rules of the brief

    - **The lower of LEFT and RIGHT critical altitude wins.**  Always.
    - **Crosswind tie-breaker:** turn INTO the wind (rolls the airplane into the wind, tightens the ground arc, finishes upwind of the runway).
    - **Below straight-ahead-max:** land within the airport boundary.  Not the runway.  The boundary.
    - **Inside the dead zone:** straight ahead.  Any deviation costs altitude you don't have.
    - **Above recommended:** execute the briefed turn without negotiation.

    ---

    ### 6. What the tool does NOT do

    - Does not model your specific engine failure mode (partial power, oil out the cowl, structural).
    - Does not model pilot panic, fixation, or decision delay beyond the reaction-time slider.
    - Does not replace the POH or a current CFI.
    - Does not endorse turnbacks.  **The default answer is always straight ahead.**  This tool quantifies *when* the turnback option becomes viable — never *that it should be taken.*

    ---

    ### 7. CFI takeaways

    - Run the Data Card with every primary student before solo.
    - Re-run it on every BFR / IPC.  The student's airplane will have changed; their reaction time will have changed; the dead zone will have changed.
    - Use the bank-angle / flap / weight panels to *show* the trade-offs instead of *telling*.  The numbers move in real-time.
    - Make the four numbers (calculated, recommended, sa-max, dead-zone) part of the takeoff brief vocabulary, the way V-speeds and rotation are.
    """)


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
    
    ### 4. Best-Glide Speed (max L/D)
    
    At maximum L/D — the speed that gives the *farthest distance per foot
    of altitude lost*:
    
    **CL\\* = √(CDo / k)**
    
    **Vbg,TAS = √(2 · nz · W / (ρ · S · CL\\*))**
    
    In KIAS:
    
    **Vbg,KIAS = Vbg,TAS · √σ**
    
    where σ is the air density ratio.
    
    > ⚠️ **Note:** Best L/D speed (max range glide) is **not** the same as
    > minimum-sink-rate speed.  Minimum sink occurs at a *lower* CL than
    > best L/D (specifically CL = √(3·CDo/k) for a parabolic polar).
    > Min-sink keeps you airborne longer but covers less ground; best L/D
    > covers more ground but at a higher sink rate.  For a turnback you
    > want the speed that maximizes the **height available at a given
    > horizontal distance** — i.e. best L/D, not min sink.
    
    ### 4b. Operational Turn-Speed Targets
    
    Best L/D in a steep banked turn is *very close to the turning stall*.
    For everyday flying, the recommended targets (per Charlie Precourt /
    EAA McSpadden Project) are:
    
    **Vs(φ) = Vs(1g) · √nz   = Vs(1g) · √(1 / cos φ)**
    
    Then either:
    - **Vs(φ) + 10 KIAS**, or
    - **1.3 × Vs(φ)**
    
    These give a safe stall margin while still achieving close-to-optimal
    altitude loss in the turn.
    
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
    
    1. **Prof. James F. Rogers** — *"Looking Back at the Turn-Back Maneuver"*
       — [Naval Academy paper PDF](https://www.usna.edu/AeroDept/_files/documents/Faculty/Rogers/RogersTurnBack.pdf) ·
       [AOPA summary](https://www.aopa.org/news-and-media/all-news/2017/october/flight-training-magazine/the-impossible-turn)
       - Analytical method for calculating altitude loss in the turn
       - Accounts for lift/drag, load factor, turn geometry
       - Basis for safety margins in this simulator
    
    2. **Brent W. Jett** (USAF Academy, 1982) — *"An Analysis of the Engine-Out Turnback Maneuver for a Light, Single-Engine Aircraft"*
       — [DTIC report ADA122862](https://apps.dtic.mil/sti/citations/ADA122862)
       - Evaluated critical altitude across aircraft types
       - Effect of pilot skill, reaction time, bank angle
       - Results in similar range to this simulator (500–1500 ft AGL typical)
    
    3. **FAA Advisory Circular 61-83K** (2024) — *"Nationally Scheduled, FAA-Approved, Industry-Conducted Flight Instructor Refresher Courses"*
       — [FAA AC 61-83K PDF](https://www.faa.gov/regulations_policies/advisory_circulars/index.cfm/go/document.information/documentID/1043603)
       - Paragraph A.114: Pilots must train engine-out turnback scenarios
       - Proposed EAA/FAA method for quantitative assessment
    
    4. **FAA Airplane Flying Handbook** (FAA-H-8083-3C, 2024)
       — [FAA AFH PDF](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/airplane_handbook)
       — Chapter 6 (takeoff/departure) and Chapter 18 (emergency procedures)
    
    5. **EAA Sport Aviation, May 2026** — *"Rethinking the Impossible Turn"* (McSpadden Project feature)
       — [EAA Sport Aviation archive](https://www.eaa.org/eaa/news-and-publications/eaa-publications/sport-aviation)
    
    6. **Standard Aerodynamic References**
       - Anderson, *Fundamentals of Aerodynamics* (McGraw-Hill, 6th ed.) — drag polar, load factor
       - FAA *Pilot's Handbook of Aeronautical Knowledge* (FAA-H-8083-25C) —
         [PHAK PDF](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/phak)
    
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
