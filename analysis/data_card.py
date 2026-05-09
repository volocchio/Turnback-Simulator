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
import math


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
        # wind_from_deg=90 → wind FROM the right at takeoff.  Turning RIGHT
        # rolls the aircraft INTO the wind during the turn (tightens ground
        # arc, wind helps push the nose around) and finishes the 180° upwind
        # of the runway centerline — i.e. blown back toward the field rather
        # than away from it.  This is the "turn into the wind" pilot rule.
        # (Note: on the back-leg itself the crosswind is symmetric; the
        # benefit is in the *turn* and in the post-turn position.)
        if 1 <= wind_from_deg <= 179:
            return ("RIGHT", f"Critical altitudes are equivalent.  "
                              f"Surface wind is from the right — turning RIGHT "
                              f"rolls you INTO the wind, tightening the ground "
                              f"arc and finishing the 180° upwind of the runway.")
        elif 181 <= wind_from_deg <= 359:
            return ("LEFT", f"Critical altitudes are equivalent.  "
                             f"Surface wind is from the left — turning LEFT "
                             f"rolls you INTO the wind, tightening the ground "
                             f"arc and finishing the 180° upwind of the runway.")
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
    if isinstance(ac_key, (tuple, list)):
        # Drop the wing-mod variant tag from the user-facing label (only meaningful internally).
        _parts = [str(p) for p in ac_key if p and str(p) != 'Flatwing']
        ac_label = " ".join(_parts) if _parts else str(ac_key[0])
    else:
        ac_label = str(ac_key)

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
    flap_retract_alt_ft = res.get('flap_retract_alt_ft', 0) or 0
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
                f"<p class='warn'><strong>⚠ CRITICAL ZONE:</strong> Engine failure between "
                f"<strong>{int(round(sa_max)):,} ft</strong> and "
                f"<strong>{int(round(crit_min)):,} ft AGL</strong> = no good options. "
                f"Plan to climb through this band as quickly as possible.</p>"
            )
        else:
            warning_html = (
                "<p class='ok'><strong>✓ NO CRITICAL ZONE:</strong> Straight-ahead and "
                "turnback coverage overlap.  Every failure altitude has an option.</p>"
            )

    # Charlie #C8 — flap-still-deployed warning.  If the critical-turnback altitude
    # is below the flap-retract altitude, the pilot is starting the turn with
    # takeoff flaps still extended (regardless of what they selected for "Turn flap").
    if (flap_takeoff > 0 and flap_retract_alt_ft > 0
            and crit_min and crit_min < flap_retract_alt_ft):
        warning_html += (
            f"<p class='warn'><strong>⚠ FLAPS STILL OUT:</strong> Critical turnback "
            f"altitude ({int(round(crit_min)):,} ft AGL) is BELOW your flap-retract "
            f"altitude ({int(round(flap_retract_alt_ft)):,} ft AGL).  At engine "
            f"failure you will still have <strong>{_FLAP_LABELS.get(flap_takeoff, str(flap_takeoff))}</strong> "
            f"deployed — the \"Turn flap\" selection above assumes you've already "
            f"cleaned up.  Real-world drag will be higher; expect more altitude loss "
            f"than the table shows.</p>"
        )

    css = """
    <style>
      @page { size: letter; margin: 0.5in; }
      body { font-family: Helvetica, Arial, sans-serif; color: #111; }
      h1 { font-size: 18pt; margin: 0 0 4px 0; border-bottom: 3px solid #0a3a5c; padding-bottom: 4px; }
      h2 { font-size: 11pt; margin: 12px 0 4px 0; color: #0a3a5c; border-bottom: 1px solid #ccc; padding-bottom: 2px; -pdf-keep-with-next: true; }
      h3 { font-size: 10pt; margin: 6px 0 3px 0; color: #0a3a5c; }
      .header-meta { color: #666; font-size: 9pt; margin-bottom: 8px; }
      table { width: 100%; border-collapse: collapse; font-size: 10pt; margin-bottom: 6px; }
      td, th { padding: 3px 6px; text-align: left; vertical-align: top; }
      td.label { font-weight: bold; color: #444; width: 40%; }
      td.value { color: #000; }
      /* xhtml2pdf doesn't support CSS grid — use a 2-column table wrapper */
      table.cols2 { width: 100%; border: none; }
      table.cols2 > tbody > tr > td { width: 50%; vertical-align: top; padding: 0 6px; border: none; }
      .big { background: #f3f7fc; border-left: 4px solid #0a3a5c; padding: 8px 12px; margin: 8px 0; }
      .big .label { font-size: 9pt; color: #555; font-weight: bold; }
      .big .number { font-size: 22pt; font-weight: bold; color: #0a3a5c; }
      .big .sub { font-size: 9pt; color: #666; }
      .hero { background: #0a3a5c; color: #ffffff; padding: 10px 14px; margin: 4px 0 10px 0; }
      .hero .htitle { font-size: 11pt; font-weight: bold; }
      .hero .hsub { font-size: 9pt; color: #cfe1f0; }
      .hero .hkey { font-size: 24pt; font-weight: bold; margin-top: 4px; }
      .hero .hkeysub { font-size: 9pt; color: #cfe1f0; }
      .warn { background: #fff4e5; border-left: 4px solid #d84315; padding: 6px 10px; font-size: 10pt; }
      .ok { background: #e8f5e9; border-left: 4px solid #2e7d32; padding: 6px 10px; font-size: 10pt; }
      .muted { color: #888; font-style: italic; font-size: 9pt; }
      table.wp { width: 50%; }
      table.wp th, table.wp td { border: 1px solid #ddd; padding: 2px 8px; }
      .footer { margin-top: 12px; font-size: 8pt; color: #777; border-top: 1px solid #ddd; padding-top: 6px; }
      .pgbreak { -pdf-keep-in-frame-mode: shrink; }
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

    <div class='hero'>
      <div class='htitle'>{e(ac_label)} · {e(airport)} · RWY {e(runway)}</div>
      <div class='hsub'>Weight {_fmt_int(weight, ' lb')} · Field {_fmt_int(field_elev, ' ft MSL')} · ISA{_fmt_dec(isa_dev, 0, '°C') if isa_dev >= 0 else _fmt_dec(isa_dev, 0, '°C')} · Wind {_fmt_int(wind_speed, ' kt')} from {_fmt_int(wind_from_true if wind_from_true is not None else wind_from_deg, '°')}</div>
      <div class='hkey'>{_fmt_int(crit_recommend, ' ft AGL')}</div>
      <div class='hkeysub'>Recommended turnback minimum (× {safety_margin_factor:.2f} safety factor) · {e(rec_dir)} turn · {_fmt_int(crit_recommend_msl, ' ft MSL')} on the altimeter</div>
    </div>

    <h2>Aircraft &amp; Conditions</h2>
    <table class='cols2'><tr><td>
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
        <tr><td class='label'>Flap-retract altitude</td><td class='value'>{(_fmt_int(flap_retract_alt_ft, ' ft AGL') + " <span class='muted'>(climb-out: takeoff flaps until this altitude, then clean)</span>") if flap_takeoff > 0 and flap_retract_alt_ft > 0 else "<span class='muted'>n/a (clean takeoff)</span>"}</td></tr>
        <tr><td class='label'>Turn flap</td><td class='value'>{e(_FLAP_LABELS.get(flap_turn, str(flap_turn)))}</td></tr>
        <tr><td class='label'>Bank angle (turnback)</td><td class='value'>{_fmt_int(bank, '°')}</td></tr>
        <tr><td class='label'>Turnback speed mode</td><td class='value'>{e(speed_mode_label)}</td></tr>
        <tr><td class='label'>Reaction time</td><td class='value'>{_fmt_dec(reaction, 1, ' sec')}</td></tr>
      </table>
    </td></tr></table>

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
    <table class='cols2'><tr>
      <td>
        <div class='big'>
          <div class='label'>Recommended minimum altitude</div>
          <div class='number'>{_fmt_int(crit_recommend, ' ft AGL')}</div>
          <div class='sub'><strong>{_fmt_int(crit_recommend_msl, ' ft MSL')}</strong> on your altimeter.<br/>
          Calc {_fmt_int(crit_min)} ft × <strong>{safety_margin_factor:.2f} safety factor</strong>.<br/>
          Below this altitude on takeoff: <strong>land straight ahead.</strong></div>
        </div>
      </td>
      <td>
        <div class='big'>
          <div class='label'>Recommended turn direction</div>
          <div class='number'>{e(rec_dir)}</div>
          <div class='sub'>{e(rec_rationale)}</div>
        </div>
      </td>
    </tr></table>

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

    <h2>Training Brief — what these numbers mean</h2>
    <div style='font-size: 10pt; line-height: 1.45;'>
      <p><strong>Critical altitude</strong> is the lowest AGL at which the airplane can complete the turnback and reach the runway.  Below this, the turnback fails — the airplane runs out of altitude before the runway is in reach.  The LEFT/RIGHT split exists because crosswind drift, slope, and obstacles make the two directions asymmetric.  <strong>Always pick the lower of the two.</strong></p>
      <p><strong>The 180° figure</strong> is half-mythology in the cockpit.  In reality the turn must overshoot the runway centerline (typically 200–270°) so the airplane is aligned on final, not perpendicular.  The "Sequence of Events" table above shows the actual heading change.</p>
      <p><strong>Safety margin × Calculated</strong> = recommended go/no-go.  The calculated number is the bare aerodynamic minimum; the recommended number ({_fmt_int(crit_recommend, ' ft AGL')}) is what you brief.  Below the recommended altitude on takeoff, plan to land straight ahead within the airport boundary.</p>
      <p><strong>Reaction time</strong> is the killer.  Three seconds doubles or triples the critical-zone altitude band.  Brief it on every takeoff: "engine quits below {_fmt_int(crit_recommend, ' ft AGL')} → straight ahead, no negotiation."</p>
      <p><strong>Bank angle</strong> trades stall margin for turn radius.  At {_fmt_int(bank, '°')} bank, stall speed is {(_fmt_dec(((1.0 / max(0.05, math.cos(math.radians(bank))))**0.5 - 1.0) * 100.0, 0, '%')) if bank > 0 else '0%'} higher than wings-level.  Steeper than 45° is rarely worth it: the radius gain is small but the stall margin shrinks fast.</p>
    </div>

    <h2>Practice Drill</h2>
    <ol style='font-size: 10pt; line-height: 1.4;'>
      <li>Climb to a safe altitude (3,000 ft AGL or higher).</li>
      <li>Reduce to climb-out KIAS shown above ({_fmt_int(airspeed, ' KIAS')}), simulate idle power, wait the {_fmt_dec(reaction, 1, ' sec')} reaction time.</li>
      <li>Roll into a {_fmt_int(bank, '°')} bank turn at the indicated turn-flap setting.</li>
      <li>Time a 180° turn and read the altimeter delta.</li>
      <li>Compare to the <strong>{_fmt_int(loss_per_180, ' ft')}</strong> figure above.  If your real loss is higher, increase your personal safety factor.</li>
    </ol>

    <div class='footer'>
      Sources: Rogers (USNA, <i>Looking Back at the Turn-Back Maneuver</i>), Jett (USAFA, DTIC ADA122862), FAA AC 61-83K para A.114, FAA AFH Ch.6, EAA Sport Aviation May 2026.<br/>
      <strong>This card is a planning tool — not a substitute for the aircraft POH or pilot judgment.</strong>
    </div>
    """

    return f"<!doctype html><html><head><meta charset='utf-8'><title>Takeoff Data Card</title>{css}</head><body>{body}</body></html>"


def build_takeoff_data_card_pdf(res: dict, crit_result: dict | None,
                                safety_margin_factor: float = 1.5) -> bytes | None:
    """Render the takeoff data card to PDF bytes (US Letter, 0.5in margin).

    Uses xhtml2pdf (pure-Python, reportlab-based — no system deps).  Returns
    None if xhtml2pdf is not available or rendering fails, so callers can
    gracefully fall back to the HTML download.
    """
    try:
        from xhtml2pdf import pisa
        import io
    except Exception:
        return None

    html = build_takeoff_data_card(res, crit_result, safety_margin_factor)

    # xhtml2pdf has a CSS subset and a limited Unicode font.  Map characters
    # the default font can't render to ASCII equivalents.
    safe_html = (
        html
        .replace("⚠", "[!]")
        .replace("📋", "")
        .replace("✓", "OK")
        .replace("→", "->")
        .replace("·", "-")
        .replace("×", "x")
        .replace("—", "-")
        .replace("–", "-")
        .replace("≈", "~")
        .replace("σ", "sigma")
        .replace("γ", "gamma")
        .replace("φ", "phi")
        .replace("ρ", "rho")
        .replace("²", "^2")
        .replace("√", "sqrt")
        .replace("Δ", "delta ")
        .replace("\u00a0", " ")
        .replace("\u2009", " ")
        .replace("\u200b", "")
    )

    buf = io.BytesIO()
    try:
        result = pisa.CreatePDF(src=safe_html, dest=buf, encoding="utf-8")
    except Exception:
        return None
    if getattr(result, "err", 1):
        return None
    return buf.getvalue()
