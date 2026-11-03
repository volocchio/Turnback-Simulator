# Turnback Simulator — review

Reviewed against the engine-out turnback literature used by this project (Rogers, *Estimating Turnback Altitude* / *Looking Back at the Turn-Back Maneuver*; Jett, USAFA, DTIC ADA122862; FAA Airplane Flying Handbook Ch. 3 load factor and Ch. 6 / Ch. 18 drafts in `docs/references/`; AC 61-83K; the equations already written up in `eaa-mcsppadden-project/`). The running model is `analysis/turnback_simulator.py`. `engine/flight_physics.py` is an older jet-performance deck; the turnback path uses its atmosphere function and, previously, its thrust polynomial for climb position.

Findings are ranked by severity. Each one says what was wrong, what this branch did about it, and what a pilot or certification engineer will actually see.

## 1. Critical — the return could be scored as a miss when the airplane had the energy to reach the runway

On the return the model pointed the nose at the touchdown aim and held wings level. A straight line to that aim crosses the centerline **at the aim**. If the glide reached the pavement short of the aim, the airplane was still offset by a large fraction of the turn diameter and failed the 75 ft lateral gate. It was a parallel arrival in the grass, scored as a failed turnback.

With the old, too-steep climb (finding 2) the failure point was still near the runway and the error was partly hidden. With a realistic climb gradient it dominated: a C172 at 800 ft AGL, 5,000 ft runway, was touching down near y = 3,400 ft but x ≈ −320 ft and was marked unsuccessful.

**Fix.** The return now leads the aim by about three cross-track distances, so the track captures the centerline and then tracks it. The same 800 ft case touches down near x = −20 ft and is a success. This is the geometry the AFH describes (turn to align, then fly final), and it is what Rogers' "270" is for: you do not stop the turn parallel to the runway, offset by 2R.

## 2. Critical — climb position used a light-jet thrust polynomial

`_estimate_climb_distance` scaled the FJ44-style polynomial in `flight_physics.thrust_calc` by `thrust_mult`. For a C172 that produced an airborne climb of about 2,400 ft to reach 800 ft AGL, which is a climb angle near 19°. The POH best-rate climb is about 730 fpm at 74 KIAS, closer to 8,000 ft of airborne distance for the same 800 ft. The failure point was much too close to the threshold, so the turnback looked easier than the airplane can fly.

**Fix.** When the aircraft is in `POH_CLIMB`, distance and time come from that rate of climb, scaled by the existing weight and density rule (`ROC × MTOW/W × σ^n`). The polynomial remains only for a type with no POH row. Track-hold distance is groundspeed × time, so a headwind shortens it and a tailwind lengthens it. Heading-hold still adds the wind drift on top of the still-air air distance (that was already the contract, and the tests cover it).

Checked: 20 kt headwind, C172, 800 ft, track-hold, moves the failure about 2,200 ft closer than calm. That is 20 kt × (800 ft / 730 fpm).

## 3. High — the speed change did not trade kinetic energy, and the comment said it did

After the reaction timer the speed ramps toward the glide target at 5 kt/s. The comment above that ramp said the altitude cost was in the energy balance. The integrator only applied the constant-speed sink `V · sin γ = −V · D/W`. Decelerating from Vy toward Vs(φ)+10 returns on the order of `(V₁² − V₂²)/(2g)`, which for a 15–30 kt bleed is roughly 50–200 ft. Accelerating into a steeper bank spends the same term. For a tool whose answer is a few hundred feet, that is a first-order omission.

**Fix.** At each step, `Δh = (V_before² − V_after²)/(2g)` is added, with both speeds evaluated at the current density so a density-driven TAS change at constant KIAS is not treated as a pitch change. The drag sink is unchanged when speed is held. A test integrates `roc_fpm` and checks that the extra altitude matches that expression.

## 4. High — "landing / full" used approach drag with full-flap CLmax

Flap setting 2 and the automatic landing polar on final used `Clmax_flaps40` with `dcdo_flap2` (the middle increment). For the C172 table that is CLmax = 2.30 with ΔCDo = 0.015 instead of the full-flap 0.025. Final was too flat, so the "can I get down" and slip/orbit decision was optimistic about glide ratio and pessimistic about drag.

The third increment is labeled "ground spoiler" in the dataclass because the schema came from the jet deck. In this single-engine table the three increments are monotonic flap drag, and the UI label is "Landing / Full".

**Fix.** Full flap is `_landing_polar`: ΔCDo_flap3 with Clmax_flaps40, with fallbacks if an increment is zero. Turns flown clean are unchanged until the return, where the runway model deploys landing flaps.

## 5. High — a 270° alignment was labeled a 540° orbit

`classify_maneuver` called every reverse landing with at least 270° of heading change a "540". The comment said a pure 180 rarely exceeds ~210°. That threshold then swept up the maneuver the literature actually argues about.

Rogers and the AFH drafts distinguish:

- about 180° of heading change, which leaves you parallel to the runway and offset by about two turn radii;
- continuing to roughly 210–300° so the turn itself intercepts the final course (the "270");
- 180° plus a 360° orbit when you are high (a 540);
- a full circuit back to the original direction.

A 250° turn with no orbit phase is a 270, not a 540. The code already flew that geometry (it keeps turning until the nose points at a usable aim point, and it will not call the turn done before 135°). Only the label was wrong, and the decision ladder had no card for it.

**Fix.** Classes are now `180` (under 210°, no orbit), `270` (210° or more, no orbit), `540` (an `orbit` phase, or heading change ≥ 400°), and `circuit` (original direction). The ladder and the 3-D track legend show the 270.

## 6. Medium — straight-ahead ignored the winds-aloft speed

The turning sim interpolated wind speed with altitude. The straight-ahead sim kept the surface speed for the whole glide and only updated direction when a direction profile was passed. `wind_profile` was accepted by `find_straight_ahead_max_altitude` and then dropped. A tailwind that lives at 1,000 ft did not move the touchdown.

**Fix.** Straight-ahead uses the same speed and direction interpolation as the turn. The envelope passes the profile through.

## 7. Medium — the theory text had the 45° stall-speed factor wrong

Stall speed scales with √n, and n = 1/cos φ.

| Bank | n | Vs multiplier | Increase |
| --- | --- | --- | --- |
| 30° | 1.15 | 1.08 | +8% |
| 45° | 1.41 | 1.19 | +19% |
| 60° | 2.00 | 1.41 | +41% |

The in-app theory section said "at 45° bank, nz = 1.41, so stall speed is 41% higher." That applies nz itself instead of √nz. It is the 60° result. The curriculum block and the sidebar help already had 30/45/60 right, and `vs_kias_at_load` was right. The contradiction is what a certification reader would catch.

**Fix.** The theory section and `eaa-mcsppadden-project/THEORY-AND-REFERENCES.md` now say +19% at 45° and +41% at 60°.

## 8. Medium — "Computing trajectories..." covered a lot that was not the integrator

On this machine a full auto-ceiling envelope (C150, 100 ft steps, up to ~3,800 ft, both directions, straight-ahead) was about 2–3 s and produced ~210,000 stored samples. Building the Plotly figures from that was another ~2.5 s and a 23 MB JSON document. Streamlit keeps the current spinner up until the script reaches the next status element, so the label "Computing trajectories..." stayed on screen through figure construction.

On top of that, the first paint also:

- built a second full envelope, because "Compare alternate climb steering" defaulted on;
- queried OpenStreetMap for landing candidates (25 s timeout) and the airport perimeter (15 s timeout), both defaulted on.

An empty container log follows from the same gap: nothing was written to stdout while that work ran, and Streamlit's own log level was left at the default.

**Fix.**

- Integration step is 0.10 s. On a C172, 45°, 10 kt quartering wind, critical altitude moved from 638 ft at 0.05 s to 641 ft at 0.10 s, inside the search tolerance of 5 ft. The reported statistics are still taken from the full step history.
- The trajectory stored for the UI is thinned to at most 500 points, keeping phase changes and both ends. The same envelope is now ~41,000 points, the 3-D figure is ~3.4 MB, and the envelope itself is under 1 s here.
- Comparison and both OSM queries default off. They are still on the page.
- The process logs to stdout: startup, envelope start (weight, bank, wind, speed mode), envelope finish (seconds, critical altitudes). `PYTHONUNBUFFERED` was already set. Compose now passes `--logger.level=info`.

## 9. Medium — a stall did not end the maneuver

`CL > CLmax` set a flag and then kept integrating the parabolic polar, which does not describe a stalled wing and can still reach the pavement. A success with `stalled=True` is not a return a pilot can fly.

**Fix.** The integration stops at the stall and `success` is forced false. The flag and the time are still in the result.

## 10. Low — non-finite inputs could spin

A NaN weight produced NaN state and the loop ran until the time limit (`max(900 s, altitude)`). Bank past 90° makes `1/cos φ` and `tan φ` change sign.

**Fix.** Non-finite or non-positive weight, wing area, altitude, airspeed, or reaction time returns a structured failure and logs a warning. Bank is clamped to [0°, 89°]. The loop also stops if x, y, or z becomes non-finite.

## 11. Low — compressibility is neglected, and a dead formula pretended otherwise

`_kias_to_fps` built a Mach estimate by nested division that simplifies to `KIAS / (a0 √δ)`, then threw it away and set the IAS→EAS correction to 1. Dynamic pressure `q = V_KEAS² / 295` psf is the right sea-level identity (½ρ₀ × (1.6878)² ≈ 1/295). Treating KIAS as KEAS is within about 1% below Mach 0.3, which is the light-aircraft turnback. It is not adequate for a turbine single near its climb speed at altitude. The April dashboard note that compressibility was verified in this path overstated what the code does.

**Fix.** The dead estimate is gone. The approximation is stated in the function and in the on-screen assumptions. Not "corrected" into a fake compressibility model.

## 12. Stated approximations that were checked and left in place

These match Rogers and the AFH level-turn treatment. They are now written on the "How it works" panel so they are not implicit.

- **Load factor** `n = 1/cos φ` omits `cos γ`. For an 8° glide, `cos γ ≈ 0.99`. The along-track balance `sin γ = −D/W` is the unaccelerated equation and does **not** divide by n. Bank shows up only because CL, and therefore D, is higher. An earlier bug (`sin γ = −D/(n W)`) is described in `_workspace/DASHBOARD.md` as fixed in April; the running loop uses `−D/W`. The README had regressed to a dimensionally wrong `H_dot = −D/(n W)`. The README now matches the code.
- **Turn rate** `ω = g tan φ / V_TAS` is the coordinated gliding turn (bank about the velocity vector). The ground-track radius is `V² cos γ / (g tan φ)`, not the level-turn `V² / (g tan φ)`. The number reported as "turn radius" is the level-turn value pilots look up. Both are tested.
- **Wind** does not change the air-mass polar. Sign convention, checked in the headwind test: 0° is a headwind, the wind velocity is in −Y, +X is right, +Y is the takeoff direction. 90° (from the right) drifts a heading-hold climb to −X.
- **Propeller drag** is a constant ΔCDo (0.0005 feathered and stopped through 0.004 unfeathered) on a CDo of about 0.03. A windmilling fixed-pitch prop is often a larger share of parasite drag than +0.002. These were not retuned. Doing so without a flight-test drag increment would move every critical altitude on a guess. The panel says the increment is a constant, not a speed-dependent windmill model.
- **Reaction delay** holds the failure speed, wings level. It does not model a frozen stick. The old help text said the airplane decelerates during the delay; that sentence was wrong for the model. The timer behavior is unchanged. The pitch to the glide target, and the energy trade, starts when the timer ends.
- **Safety-margin factor** (default 1.25) is a multiplier, not a statistical tolerance or a flight-test scatter band.
- **`find_critical_altitude` returns the search ceiling** when nothing in the band succeeds. The UI now says so, instead of presenting the ceiling as a demonstrated minimum.
- **Atmosphere.** Troposphere σ and the density altitude are the standard lapse formulas already in `atmos` (6.8756×10⁻⁶ / ft, exponent 4.2559 / 5.2559). Density altitude is now the closed form of the old Newton loop, same constants. Below the tropopause it round-trips. `g = 32.174 ft/s²`. Sea-level density used in the turnback polar is 0.002378 slug/ft³; the jet deck uses 0.0023769. The difference is under 0.1% and was left alone so the two modules do not silently diverge.

## Security and dependencies

- Compose binds `127.0.0.1:8510` and disables CORS and XSRF. That is a real weakening if anything other than the trusted proxy can reach the port. It is also what a Streamlit app behind Caddy often needs, and this branch does not change it. The flags are commented in `docker-compose.yml`.
- There is no authentication. Acceptable only while the proxy is the access control.
- Overpass queries are hardcoded URLs, not user-supplied hosts.
- Removed unused direct dependencies that the turnback process never imports: `kaleido` (Plotly image export; this app uses xhtml2pdf), `pdfplumber`, `openpyxl`, and `scipy`. SciPy was imported only by `engine/thrust_deck.py`, which the turnback loop does not call. If that import fails, `flight_physics` already falls back. Remaining direct dependencies are pinned to the versions the tests ran on.

## Documents

Nothing under `engine/`, `analysis/`, or `turnback_app.py` referenced the Word, PDF, or PowerPoint paths. They are reference material, not inputs. They now live in `docs/references/`:

- AC Draft v4.docx
- Staff Summary v3.docx
- AFH Ch6 Draft Revisions v1.docx
- AFH Ch18 Draft Revisions v1.docx
- FAA Advisory Circular on CFI guidelines AC_61-83K.pdf
- Estimating Turnback Altitude.pdf
- Brent Jett 180 Back.pdf
- Segmented Turnback Analysis V2.pptx

`eaa-mcsppadden-project/` stays where it is (briefing notes and the theory write-up). `_workspace/DASHBOARD.md` path list was updated. The Docker build context ignores `docs/` so the image does not carry the drafts.

## Image

The deployed image was reported at **2.12 GB**. Rebuilding that Dockerfile from this tree today (same `python:3.12-slim` base, current wheels, context without `.git`) produced **1.44 GB**. The June image is larger than that rebuild; the Dockerfile as it stood still installed `gcc` and the Cairo dev packages and `COPY . .`, which sent the drafts, both CSV and Parquet airport tables, and the unused Python packages into the image.

The replacement image is runtime Cairo/Pango only, wheels only, and a copy of `engine/`, `analysis/`, the Parquet tables, and `turnback_app.py`. Measured size: **757 MB**.

## User-visible changes

Behavior is otherwise the same screens, the same inputs, and the same aircraft library. What a user will see:

1. Critical altitudes and touchdown positions move where the climb gradient, the energy trade, the full-flap drag, the centerline intercept, or the winds-aloft speed was wrong. The 1.25 safety-margin factor is still applied on top of the calculated number and is labeled separately. Default teaching case, C150, KSEZ runway 21, 10 kt from 271° (about 45° right of the runway), screenshots in the pull request:

   | | Before | After |
   | --- | --- | --- |
   | Recommended LEFT / RIGHT (1.25×) | 985 / 590 ft AGL | 661 / 340 ft AGL |
   | Calculated LEFT / RIGHT | 788 / 472 ft | 529 / 272 ft |
   | Straight-ahead maximum | 325 ft (dead zone 325–472) | 351 ft (the favored turn now overlaps straight-ahead) |
   | Ladder | 180° at 472, called-540 at 2,100, circuit at 4,000 | 180° at 272, 270° at 529, 540° at 1,700, circuit not inside the ceiling |

   A separate C172S check (2,550 lb, 74 KIAS, 45°, 5 s, 10 kt at 45° to a 5,132 ft runway, no field elevation) moved from about 638 / 513 ft calculated to 660 / 382 ft. The right turn drops because that quartering wind is a headwind component, which now shortens the climb ground run, and because the return can get onto the centerline instead of being scored in the grass.
2. The decision ladder has a **270° alignment** card between the 180° reversal and the 540° orbit. A turn that lines up on final without an orbit is no longer called a 540.
3. If the search hits the ceiling without a successful return, the page says the number is the search limit.
4. "Compare alternate climb steering" and both OpenStreetMap queries start unchecked, so the first result is not gated on a second envelope or on Overpass. Turn them back on for the comparison callout and the landing-area overlay.
5. The theory blurb no longer says a 45° bank raises stall speed by 41%. The "How it works" panel states the assumptions above, including that the reaction delay holds the failure speed.
6. The reaction-time chart no longer uses the phrase "oh-shit delay."
7. Plotted tracks are thinned. The shape, the critical altitude, and the phase altitudes are from the full integration.

Not changed: aircraft data, prop-drag increments, the 1.25 default margin, bank and reaction defaults, teaching-mode auto-run, the hero, the data card, XSRF/CORS flags, and the loopback publish port.
