"""Takeoff Data Card (TOLD-style) generator.

Produces a printer-friendly, single-page HTML card summarizing the turnback
analysis for a specific aircraft / airport / runway / weather scenario.

Designed for Charlie Precourt's EAA McSpadden Project: pilots fill in their
day-of-flight conditions, run the sim, then print this card to keep in the
cockpit alongside their existing TOLD card.

Usage from turnback_app.py:
    html = build_takeoff_data_card(res, crit_result)
    st.download_button("Download Data Card (HTML)", html, "TOLD_card.html",
                       mime="text/html")
"""

from __future__ import annotations
from datetime import datetime
import html as _html


_FLAP_LABELS = {0: "Clean", 1: "Takeoff / 15°", 2: "Landing / Full"}


def _fmt_int(v, suffix=""):
    if v is None:
        return "—"
    try:
        return f"{int(round(float(v))):,}{suffix}"
    except (TypeError, ValueError):
        return "—"


def _fmt_dec(v, places=1, suffix=""):
    if v is None:
        return "—"
    try:
        return f"{float(v):,.{places}f}{suffix}"
    except (TypeError, ValueError):
        return "—"


# Charlie #F4 — per-phase explainer rendering for the data card.
_PHASE_TITLES = {
    'reaction': "1. Reaction",
    'turn': "2. Turn back",
    'return': "3. Return / approach",
    'orbit': "4. Orbit (energy bleed)",
}


def _render_phase_summary(phase_summary, e) -> str:
    """Render the per-phase breakdown table for the data card.

    Each phase shows: title, dt, altitude lost, heading change, explainer.
    Returns empty string when no phase data is available.
    """
    if not phase_summary:
        return ""
    rows = []
    for blk in phase_summary:
        title = _PHASE_TITLES.get(blk.get('phase', ''), blk.get('phase', '?').title())
        dt = blk.get('dt_s', 0.0)
        dz = blk.get('dz_loss_ft', 0.0)
        dh = blk.get('heading_change_deg', 0.0)
        z0 = blk.get('z_start_agl', 0.0)
        z1 = blk.get('z_end_agl', 0.0)
        explainer = blk.get('explainer', '') or ''
        rows.append(
            f"<tr>"
            f"<td class='label' style='width:22%;'>{e(title)}</td>"
            f"<td style='width:14%;'>{_fmt_dec(dt, 1, ' s')}</td>"
            f"<td style='width:18%;'>−{_fmt_int(dz, ' ft')} <span class='muted'>({_fmt_int(z0, '')} → {_fmt_int(z1, ' ft AGL')})</span></td>"
            f"<td style='width:14%;'>{_fmt_int(dh, '°')}</td>"
            f"<td>{e(explainer)}</td>"
            f"</tr>"
        )
    return (
        "<h2>Sequence of Events (from critical-altitude trajectory)</h2>"
        "<table>"
        "<thead><tr>"
        "<th>Phase</th><th>Duration</th><th>Altitude lost</th><th>Heading Δ</th><th>What's happening</th>"
        "</tr></thead>"
        f"<tbody>{''.join(rows)}</tbody>"
        "</table>"
    )


def _recommended_turn_direction(critical_alt_left: float,
                                critical_alt_right: float,
                                wind_from_deg: float,
                                wind_speed_kt: float) -> tuple[str, str]:
    """Return (direction, rationale) for the recommended turn.

    Logic:
      - The lower critical altitude wins.
      - If the two are within 5% of each other, pick "into the surface wind"
        because the headwind component during the back-leg shortens ground
        track and gives margin.
      - wind_from_deg is RELATIVE to the runway heading (0 = headwind on
        takeoff, 90 = right crosswind, 270 = left crosswind).
    """
    if critical_alt_left <= 0 and critical_alt_right <= 0:
        return ("—", "No turnback solution found at any altitude.")

    delta = abs(critical_alt_left - critical_alt_right)
    base = max(critical_alt_left, critical_alt_right) or 1.0
    if delta / base < 0.05:
        # Roughly tied — break with crosswind direction
        if wind_speed_kt < 1.0:
            return ("LEFT", "Critical altitudes are equivalent (no wind tiebreaker); "
                            "default LEFT is conventional pattern direction.")
        # wind_from_deg=90 → right crosswind → turning LEFT puts the headwind
        # on the back-leg (because the aircraft will be flying back towards
        # the original takeoff direction tail-on the wind).  Charlie's rule:
        # turn into the wind.
        if 1 <= wind_from_deg <= 179:
            return ("RIGHT", f"Critical altitudes are equivalent.  "
                              f"Surface wind is from the right — turn RIGHT to put "
                              f"the headwind on your turnback ground track.")
        elif 181 <= wind_from_deg <= 359:
            return ("LEFT", f"Critical altitudes are equivalent.  "
                             f"Surface wind is from the left — turn LEFT to put "
                             f"the headwind on your turnback ground track.")
        else:
            return ("LEFT", "Critical altitudes are equivalent (pure headwind); "
                            "default LEFT is conventional pattern direction.")

    if critical_alt_left < critical_alt_right:
        return ("LEFT", f"LEFT requires only "
                        f"{int(round(critical_alt_left))} ft AGL vs "
                        f"{int(round(critical_alt_right))} ft AGL for RIGHT.")
    return ("RIGHT", f"RIGHT requires only "
                     f"{int(round(critical_alt_right))} ft AGL vs "
                     f"{int(round(critical_alt_left))} ft AGL for LEFT.")


def build_takeoff_data_card(res: dict, crit_result: dict | None,
                             safety_margin_factor: float = 1.25) -> str:
    """Render the TOLD card as a self-contained, printable HTML document."""
    cfg = res.get('config')
    ac_key = res.get('ac_key', ('—', ''))
    ac_label = " ".join(str(p) for p in ac_key if p) if isinstance(ac_key, (tuple, list)) else str(ac_key)

    weight = res.get('weight', 0)
    mtow = getattr(cfg, 'MTOW', 0) if cfg else 0
    airport = res.get('airport_ident') or "—"
    runway = res.get('runway_ident') or "—"
    field_elev = res.get('field_elev', 0)
    isa_dev = res.get('isa_dev', 0)
    altimeter = res.get('altimeter_inhg')

    wind_speed = res.get('wind_speed', 0)
    wind_from_deg = res.get('wind_from_deg', 0)
    wind_profile = res.get('wind_profile') or []

    airspeed = res.get('airspeed', 0)
    bank = res.get('bank_angle', 0)
    flap_takeoff = res.get('takeoff_flap_setting', 0)
    flap_turn = res.get('flap_setting', 0)
    reaction = res.get('reaction_time', 0)
    speed_mode = res.get('speed_mode', 'fixed')

    runway_length = res.get('runway_length', 0)
    runway_length_published = res.get('runway_length_published', 0) or runway_length
    liftoff_distance = res.get('liftoff_distance', 0)

    crit_left = res.get('critical_alt_left', 0)
    crit_right = res.get('critical_alt_right', 0)
    sa_max = res.get('straight_ahead_max_alt', 0)

    # Wind components (Charlie #A1) — always available now from app
    headwind_kt = res.get('headwind_kt')
    crosswind_kt = res.get('crosswind_kt')
    wind_from_true = res.get('wind_from_true')

    # Derived recommended direction
    rec_dir, rec_rationale = _recommended_turn_direction(
        crit_left, crit_right, wind_from_deg, wind_speed
    )

    # Recommended turnback altitude with safety margin
    crit_min = min(c for c in (crit_left, crit_right) if c > 0) if max(crit_left, crit_right) > 0 else 0
    crit_recommend = crit_min * safety_margin_factor

    # Charlie #F1 — pilots read MSL on the altimeter, so show MSL alongside AGL
    crit_recommend_msl = crit_recommend + (field_elev or 0) if crit_recommend else 0
    crit_min_msl = crit_min + (field_elev or 0) if crit_min else 0
    crit_left_msl = crit_left + (field_elev or 0) if crit_left else 0
    crit_right_msl = crit_right + (field_elev or 0) if crit_right else 0

    # Turn statistics from critical-altitude trajectory
    total_turn_deg = crit_result.get('total_turn_deg', 0) if crit_result else 0
    loss_per_180 = crit_result.get('altitude_loss_per_180') if crit_result else None
    threshold_alt = crit_result.get('altitude_at_runway') if crit_result else None
    turn_radius = crit_result.get('turn_radius_ft') if crit_result else None
    landing_dir = crit_result.get('landing_direction', '') if crit_result else ''
    # Charlie #F3 / #F5 — altitude profile at departure-end threshold for the
    # downwind landing case, and MSL alongside AGL for runway crossings.
    dep_thresh_alt = crit_result.get('altitude_at_departure_threshold') if crit_result else None
    takeoff_thresh_alt = crit_result.get('altitude_at_takeoff_threshold') if crit_result else None
    threshold_alt_msl = (threshold_alt + field_elev) if (threshold_alt is not None and field_elev is not None) else None
    dep_thresh_alt_msl = (dep_thresh_alt + field_elev) if (dep_thresh_alt is not None and field_elev is not None) else None
    takeoff_thresh_alt_msl = (takeoff_thresh_alt + field_elev) if (takeoff_thresh_alt is not None and field_elev is not None) else None
    phase_summary = crit_result.get('phase_summary', []) if crit_result else []
    _landing_dir_label = {
        'reverse': "approach end (landing back into the takeoff direction)",
        'original': "departure end (landing downwind in the takeoff direction)",
    }.get(str(landing_dir).lower(), "—")

    # Wind profile rendering
    if wind_profile:
        wind_rows = "".join(
            f"<tr><td>{int(round(a))} ft AGL</td><td>{int(round(s))} kt</td></tr>"
            for a, s in wind_profile
        )
        wind_table = f"<table class='wp'><thead><tr><th>Altitude</th><th>Speed</th></tr></thead><tbody>{wind_rows}</tbody></table>"
    else:
        wind_table = "<p class='muted'>No winds-aloft profile entered.</p>"

    speed_mode_label = {
        'fixed': f"Fixed {airspeed:.0f} KIAS",
        'vs_plus_10': f"Vs(φ) + 10 KIAS",
        'vs_x_1p3': f"1.3 × Vs(φ) KIAS",
        'best_glide_1g': "Best L/D (1g, wings level)",
        'best_glide_nz': f"Best L/D for {bank:.0f}° bank",
    }.get(speed_mode, speed_mode)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    altimeter_str = f"{altimeter:.2f} inHg" if altimeter else "—"

    # Decisional warning based on altitude vs straight-ahead band
    warning_html = ""
    if sa_max and crit_min:
        if sa_max + 50 < crit_min:
            warning_html = (
                f"<p class='warn'><strong>⚠ DEAD ZONE:</strong> Engine failure between "
                f"<strong>{int(round(sa_max)):,} ft</strong> and "
                f"<strong>{int(round(crit_min)):,} ft AGL</strong> = no good options. "
                f"Plan to climb through this band as quickly as possible.</p>"
            )
        else:
            warning_html = (
                "<p class='ok'><strong>✓ NO DEAD ZONE:</strong> Straight-ahead and "
                "turnback coverage overlap.  Every failure altitude has an option.</p>"
            )

    css = """
    <style>
      @page { size: letter; margin: 0.5in; }
      body { font-family: -apple-system, Segoe UI, Helvetica, Arial, sans-serif; color: #111; max-width: 7.5in; margin: 0 auto; }
      h1 { font-size: 18pt; margin: 0 0 4px 0; border-bottom: 3px solid #0a3a5c; padding-bottom: 4px; }
      h2 { font-size: 11pt; margin: 12px 0 4px 0; color: #0a3a5c; border-bottom: 1px solid #ccc; padding-bottom: 2px; text-transform: uppercase; letter-spacing: 0.5px; }
      .header-meta { color: #666; font-size: 9pt; margin-bottom: 8px; }
      table { width: 100%; border-collapse: collapse; font-size: 10pt; margin-bottom: 6px; }
      td, th { padding: 3px 6px; text-align: left; vertical-align: top; }
      td.label { font-weight: 600; color: #444; width: 40%; }
      td.value { color: #000; }
      .grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px 16px; }
      .big { background: #f3f7fc; border-left: 4px solid #0a3a5c; padding: 8px 12px; margin: 8px 0; }
      .big .label { font-size: 9pt; color: #555; text-transform: uppercase; letter-spacing: 0.5px; }
      .big .number { font-size: 24pt; font-weight: 700; color: #0a3a5c; line-height: 1.1; }
      .big .sub { font-size: 9pt; color: #666; }
      .warn { background: #fff4e5; border-left: 4px solid #d84315; padding: 6px 10px; font-size: 10pt; }
      .ok { background: #e8f5e9; border-left: 4px solid #2e7d32; padding: 6px 10px; font-size: 10pt; }
      .muted { color: #888; font-style: italic; font-size: 9pt; }
      table.wp { width: auto; }
      table.wp th, table.wp td { border: 1px solid #ddd; padding: 2px 8px; }
      .footer { margin-top: 12px; font-size: 8pt; color: #777; border-top: 1px solid #ddd; padding-top: 6px; }
      @media print {
        body { font-size: 10pt; }
        .no-print { display: none; }
      }
    </style>
    """

    e = _html.escape

    # Build wind component string for the conditions table
    if headwind_kt is not None and crosswind_kt is not None and wind_speed > 0:
        if abs(crosswind_kt) < 0.05:
            xw_str = "0 kt"
        else:
            xw_str = f"{abs(crosswind_kt):.1f} kt {'RIGHT' if crosswind_kt > 0 else 'LEFT'} cross"
        if abs(headwind_kt) < 0.05:
            hw_str = "0 kt along"
        else:
            hw_str = f"{abs(headwind_kt):.1f} kt {'HEAD' if headwind_kt > 0 else 'TAIL'}wind"
        wind_components_html = f"{hw_str} · {xw_str}"
    else:
        wind_components_html = "—"
    if wind_from_true is not None and wind_speed > 0:
        wind_surface_html = (
            f"{_fmt_int(wind_speed, ' kt')} from {int(round(wind_from_true)):03d}° true "
            f"(rel runway {int(round(wind_from_deg)):03d}°)"
        )
    else:
        wind_surface_html = f"{_fmt_int(wind_speed, ' kt')} from {_fmt_int(wind_from_deg, '°')} (rel. runway)"

    body = f"""
    <h1>Takeoff Data Card — Turnback Analysis</h1>
    <div class='header-meta'>Generated {e(timestamp)} · Turnback Simulator (turnback.voloaltro.tech)</div>

    <h2>Aircraft &amp; Conditions</h2>
    <div class='grid2'>
      <table>
        <tr><td class='label'>Aircraft</td><td class='value'>{e(ac_label)}</td></tr>
        <tr><td class='label'>Gross weight</td><td class='value'>{_fmt_int(weight, ' lb')} (MTOW {_fmt_int(mtow, ' lb')})</td></tr>
        <tr><td class='label'>Airport</td><td class='value'>{e(airport)}</td></tr>
        <tr><td class='label'>Runway</td><td class='value'>{e(runway)}</td></tr>
        <tr><td class='label'>Field elevation</td><td class='value'>{_fmt_int(field_elev, ' ft MSL')}</td></tr>
        <tr><td class='label'>ISA deviation</td><td class='value'>{_fmt_dec(isa_dev, 0, ' °C')}</td></tr>
        <tr><td class='label'>Altimeter</td><td class='value'>{e(altimeter_str)}</td></tr>
      </table>
      <table>
        <tr><td class='label'>Surface wind</td><td class='value'>{wind_surface_html}</td></tr>
        <tr><td class='label'>Wind components</td><td class='value'><strong>{wind_components_html}</strong></td></tr>
        <tr><td class='label'>Climb-out KIAS</td><td class='value'>{_fmt_int(airspeed, ' KIAS')}</td></tr>
        <tr><td class='label'>Takeoff flap</td><td class='value'>{e(_FLAP_LABELS.get(flap_takeoff, str(flap_takeoff)))}</td></tr>
        <tr><td class='label'>Turn flap</td><td class='value'>{e(_FLAP_LABELS.get(flap_turn, str(flap_turn)))}</td></tr>
        <tr><td class='label'>Bank angle (turnback)</td><td class='value'>{_fmt_int(bank, '°')}</td></tr>
        <tr><td class='label'>Turnback speed mode</td><td class='value'>{e(speed_mode_label)}</td></tr>
        <tr><td class='label'>Reaction time</td><td class='value'>{_fmt_dec(reaction, 1, ' sec')}</td></tr>
      </table>
    </div>

    <h2>Winds Aloft</h2>
    {wind_table}

    <h2>Runway</h2>
    <table>
      <tr><td class='label'>Available runway length</td><td class='value'>{_fmt_int(runway_length_published, ' ft')}</td></tr>
      <tr><td class='label'>Liftoff distance (POH-scaled)</td><td class='value'>{_fmt_int(liftoff_distance, ' ft')}</td></tr>
      <tr><td class='label'>Last-abort point on runway</td><td class='value'>—  <span class='muted'>(future feature)</span></td></tr>
      <tr><td class='label'>Straight-ahead landing limit</td><td class='value'>0 – {_fmt_int(sa_max, ' ft AGL')} <span class='muted'>(failure below this altitude → land straight ahead, staying within the airport boundary — not just the runway asphalt)</span></td></tr>
    </table>

    <h2>Turnback — Critical Decision Numbers</h2>
    <div class='grid2'>
      <div class='big'>
        <div class='label'>Recommended minimum altitude</div>
        <div class='number'>{_fmt_int(crit_recommend, ' ft AGL')}</div>
        <div class='sub'><strong>{_fmt_int(crit_recommend_msl, ' ft MSL')}</strong> on your altimeter.<br>
        Calc {_fmt_int(crit_min)} ft × <strong>{safety_margin_factor:.2f} safety factor</strong>.<br>
        Below this altitude on takeoff: <strong>land straight ahead.</strong></div>
      </div>
      <div class='big'>
        <div class='label'>Recommended turn direction</div>
        <div class='number'>{e(rec_dir)}</div>
        <div class='sub'>{e(rec_rationale)}</div>
      </div>
    </div>

    <table>
      <tr><td class='label'>Critical altitude — turn LEFT</td><td class='value'>{_fmt_int(crit_left, ' ft AGL')} <span class='muted'>(<strong>{_fmt_int(crit_left_msl, ' ft MSL')}</strong>)</span></td></tr>
      <tr><td class='label'>Critical altitude — turn RIGHT</td><td class='value'>{_fmt_int(crit_right, ' ft AGL')} <span class='muted'>(<strong>{_fmt_int(crit_right_msl, ' ft MSL')}</strong>)</span></td></tr>
      <tr><td class='label'>Threshold-crossing altitude (margin)</td><td class='value'>{_fmt_int(threshold_alt, ' ft AGL')} <span class='muted'>(<strong>{_fmt_int(threshold_alt_msl, ' ft MSL')}</strong>) — {e(_landing_dir_label)}</span></td></tr>
      <tr><td class='label'>Crossing the takeoff-end threshold</td><td class='value'>{_fmt_int(takeoff_thresh_alt, ' ft AGL')} <span class='muted'>(<strong>{_fmt_int(takeoff_thresh_alt_msl, ' ft MSL')}</strong>)</span></td></tr>
      <tr><td class='label'>Crossing the departure-end threshold</td><td class='value'>{_fmt_int(dep_thresh_alt, ' ft AGL')} <span class='muted'>(<strong>{_fmt_int(dep_thresh_alt_msl, ' ft MSL')}</strong>) — relevant for downwind landing case</span></td></tr>
      <tr><td class='label'>Total degrees of turn required</td><td class='value'>{_fmt_int(total_turn_deg, '°')}</td></tr>
      <tr><td class='label'>Altitude lost per 180° of turn</td><td class='value'>{_fmt_int(loss_per_180, ' ft')}</td></tr>
      <tr><td class='label'>Turn radius (at chosen bank)</td><td class='value'>{_fmt_int(turn_radius, ' ft')}</td></tr>
      <tr><td class='label'>Landing direction</td><td class='value'>{e(str(landing_dir).title() or '—')}</td></tr>
    </table>

    {warning_html}

    {_render_phase_summary(phase_summary, e)}

    <h2>Practice Drill</h2>
    <ol style='font-size: 10pt; line-height: 1.4;'>
      <li>Climb to a safe altitude (3,000 ft AGL or higher).</li>
      <li>Reduce to climb-out KIAS shown above ({_fmt_int(airspeed, ' KIAS')}), simulate idle power, wait the {_fmt_dec(reaction, 1, ' sec')} reaction time.</li>
      <li>Roll into a {_fmt_int(bank, '°')} bank turn at the indicated turn-flap setting.</li>
      <li>Time a 180° turn and read the altimeter delta.</li>
      <li>Compare to the <strong>{_fmt_int(loss_per_180, ' ft')}</strong> figure above.  If your real loss is higher, increase your personal safety factor.</li>
    </ol>

    <div class='footer'>
      Sources: Rogers (Estimating Turnback Altitude), Jett (USNA), FAA AC 61-83K para A.114, EAA Sport Aviation.<br>
      <strong>This card is a planning tool — not a substitute for the aircraft POH or pilot judgment.</strong>
    </div>
    """

    return f"<!doctype html><html><head><meta charset='utf-8'><title>Takeoff Data Card</title>{css}</head><body>{body}</body></html>"
