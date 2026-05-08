# Turnback Simulator — Dashboard

**Heat:** 🔥 Active (weekly touches)
**Last updated:** May 8, 2026
**Live:** https://turnback.voloaltro.tech
**Repo:** https://github.com/volocchio/Turnback-Simulator

---

## 1. One-line purpose
Physics-based "impossible turn" planning tool for the EAA McSpadden Project — replaces the "800 ft folklore" with an aircraft/wind/runway-specific safe-return envelope and Takeoff Data Card.

## 2. Stakeholders
- **Charlie Precourt** — 4× Space Shuttle pilot, EAA McSpadden Project lead, primary requirements owner. Email: precourt@comcast.net
- **EAA McSpadden Project committee** — programmatic owner; presented to EAA Board Apr 29, 2026
- **FAA** — potential endorsement target (AC 61-83K, para A.114)
- **Tamarack / volocchio team** — build owner

## 3. Current status
- ✅ Presented to EAA Board **Apr 29, 2026**
- 🔄 **Actively iterating on Charlie's feedback** (May 2026)
- ✅ Deployed at turnback.voloaltro.tech (Streamlit + Docker + Caddy on VPS, `deploy.ps1`)
- ✅ Physics audited Apr 15 (descent-gradient bug fixed; see [/memories/repo/physics-audit.md](../../../memories/repo/physics-audit.md))
- ✅ Takeoff Data Card output ([analysis/data_card.py](../analysis/data_card.py))

## 4. Next milestone — 🎯
**Pilot-facing UX: margins, education, bank-angle penalty visualization**
- Conservative safety margins (1.25–1.5×) shown explicitly: "Calculated X ft / Recommended Y ft"
- Bank-angle sensitivity panel — make the stall-speed and sink-rate penalty obvious for 30°/40°/45°
- Reaction-time sensitivity (3 vs 5 vs 10 sec)
- Flap-strategy visual tradeoff
- Inline education on *why* numbers come out as they do (replace folklore with intuition, not just numbers)

## 5. Backlog (from Charlie's brief)
**Phase 1 — Output (mostly done)**
- [x] Takeoff Data Card section
- [ ] PDF export polish
- [x] Last abort point, turnback altitude, threshold crossing, turn direction, straight-ahead max

**Phase 2 — Safety & Education (CURRENT)**
- [ ] Safety margin visualization + text
- [ ] Bank-angle sensitivity graph
- [ ] Reaction-time sensitivity
- [ ] Academic reference links inline (Rogers, Jett, AC 61-83K, EAA Sport Aviation May 2026)

**Phase 3 — Future**
- [ ] Google Earth / off-field landing overlay between last-abort and turnback-altitude zones
- [ ] ForeFlight integration for live winds
- [ ] FAA endorsement pursuit → reference in AC 61-83K update
- [ ] In-flight/real-time mode (liability gated)

---

### Charlie call backlog — May 8, 2026 (raw, awaiting sequencing)

**A. Regressions / bugs to verify first**
- [x] **Surface wind speed (kt) input** — was forcing multiples of 5 (`step=5`); changed to `step=1` so pilots can enter 8 kt / 12 kt etc.  Same fix applied to `Surface wind FROM` direction.  Sprint A1 … ship 2026-05-08.
- [x] **Crosswind logic audit** — sign convention end-to-end is correct: `wind_components` returns +xw=right, sim drifts -wind_x for right-cross (=leftward drift, physically correct), sidebar caption labels match.  Recommended-direction rationale text was wrong (claimed turning into the wind "puts headwind on back-leg" — false; back-leg crosswind is symmetric).  Rewrote rationale to reflect the real benefit: rolling INTO the wind tightens the ground arc and finishes upwind of the runway.  Sprint A2 … ship 2026-05-08.
- [x] **Data Card runway lengths showing 0 when "Enable runway model" is OFF** — already fixed in Sprint A via `runway_length_published` fallback (`turnback_app.py:735`, `data_card.py:164`); DB length is surfaced regardless of runway model toggle.

**B. Aircraft library**
- [x] Add typical trainers: C150, C152, RV-12 *(Sprint C-B1 / RV-12 already present)*, plus Piper Tomahawk (PA-38) + Diamond DA20-C1 *(B2 shipped 2026-05-08)*

**C. Sidebar UX (climb-out / flap / bank / reaction zone)**
- [x] Move flap inputs higher in the panel
- [x] Climb-out speed: interactive flap-retract logic — "takeoff flaps → retract at XXX ft → clean climb"  *(C8 shipped 2026-05-08: new "Retract takeoff flaps at (ft AGL)" input appears when takeoff flaps > 0; data card adds Flap-retract row + warns "FLAPS STILL OUT" when critical-turnback altitude is below the retract altitude)*
- [x] Bank-angle slider help bubble *(Sprint B)*
- [x] Reaction-time help bubble — also bumped default 3→5 s per Charlie 2026-05-08 *(Sprint B + decisions sweep)*
- [x] Bigger help-bubble icons + larger help text *(Sprint B)*
- [x] Altitude-step help bubble *(Sprint B)*
- [x] Prop-drag help bubble *(Sprint B)*
- [x] Runway-model help bubble *(Sprint B)*
- [x] Safety-margin slider repositioned right after climb-out speed  *(C2 shipped 2026-05-08)*

**D. Wind / atmosphere inputs**
- [x] Winds-aloft: add direction column at 0/1000/2000/3000 *(Sprint B + C2 E3)*
- [x] Temperature input: user picks °C or ISA-deviation *(Sprint B)*

**E. Operations realism**
- [x] Intersection-departure mode *(Sprint C2 E1)*
- [x] Gear-retract timing *(Sprint C2 E2)*
- [ ] Heading-vs-track display toggle (compare runway-heading hold vs runway-track hold)  *(deferred — needs sim-side air-mass frame)*

**F. Card / output**
- [x] MSL altitude on the card, **bold** *(Sprint A)*
- [x] Safety factor in **bold** on the card *(Sprint A)*
- [x] Show altitude profile at departure-end threshold for the downwind landing case *(Sprint D1 F3)*
- [x] Per-section explainer text on the card *(Sprint D1 F4)*
- [x] Runway-remaining point: show AGL/MSL when airborne *(Sprint D1 F5; threshold rows show MSL)*
- [x] Straight-ahead landing limit = stay within **airport boundary**, not just runway asphalt  *(F7 shipped 2026-05-08: explicit st.info banner above the satellite map; OSM polygon overlay deferred)*

**G. Visualization**
- [x] Two arcs on the runway map showing top + bottom of the dead zone *(Sprint D2 G1)*
- [ ] 3D view eyepoint: dead-zone, turnback path  *(deferred)*
- [ ] Pilot-perspective flyby motion mode  *(deferred)*
- [x] Heart-shape envelope: training-only, move it to the map *(Sprint D2 G4 — overlaid on satellite map)*
- [x] OSM legend: color swatches next to good/caution/avoid *(Sprint A G5)*

**H. Curriculum / framing**
- [x] Turn this into a training curriculum: what it means for a CFI day-to-day, and for working pilots  *(H1 shipped 2026-05-08: in-app expander "🎓 Training Curriculum (CFI / working pilot)" at the bottom of the page — 7 sections: why, 3 personas, 4 numbers, 4-session lesson plan, rules of the brief, what the tool does NOT do, CFI takeaways)*
- [x] Add training explainers in output ("180° just means…")  *(H2 shipped 2026-05-08: new "Training Brief" section on the data card explaining critical alt, the 180° myth, safety-margin reasoning, reaction time, bank trade)*

## 6. Architecture snapshot
| Layer | File | Notes |
|------|------|-------|
| UI | [turnback_app.py](../turnback_app.py) | Streamlit, 3D Plotly envelope + 2D ground-track |
| Sim | [analysis/turnback_simulator.py](../analysis/turnback_simulator.py) | Envelope build, optimizer, straight-ahead |
| Data card | [analysis/data_card.py](../analysis/data_card.py) | Pilot-facing TOLD-style output |
| Map | [analysis/landing_map.py](../analysis/landing_map.py) | Runway / track overlay |
| Physics | [engine/flight_physics.py](../engine/flight_physics.py) | Atmosphere, polar, predict_roc — verified correct |
| Aircraft | [engine/aircraft_config.py](../engine/aircraft_config.py), [engine/poh_data.py](../engine/poh_data.py) | Single-engine library |
| Wx/Field | [engine/metar_parser.py](../engine/metar_parser.py), [engine/airport_db.py](../engine/airport_db.py) | METAR + airports/runways CSVs |
| Tests | [tests/](../tests/) | airport_db, data_card, metar, op-speeds, poh, turn-stats, wind |
| Deploy | [Dockerfile](../Dockerfile), [docker-compose.yml](../docker-compose.yml), [deploy.ps1](../deploy.ps1) | git push → SSH VPS → docker compose up --build |

## 7. Physics state of play
**Verified correct** (Apr 15 audit): atmosphere/ISA, IAS↔EAS↔TAS + compressibility, drag polar w/ winglet correction, thrust models, V-speeds, best-glide `CL_opt = √(CDo/k)`, turn radius/rate, rollout, wind ground-track.

**Fixed Apr 15:** descent gradient was `sin(γ) = −D/(nz·W)` — wrong. Correct is `sin(γ) = −D/W`. Bank only raises drag (via higher CL), not the gradient equation. Magnitude: ~41% sink-rate underestimate at 45° bank, 100% at 60°. Fixed in main descent loop, forward-slip, orbit estimate (added `nz_bank`), straight-ahead, and docstring.

**Open physics gap (per Charlie):** altitude lost during the *turn maneuver itself* is not in any POH. Rogers (Naval Academy) has a method but needs airfoil coefficients pilots don't have. Mitigation = conservative margin (1.25–1.5×) — needs to be surfaced clearly in the UI.

## 8. Reference library (in repo root)
- `Estimating Turnback Altitude.pdf` — Prof. James F. Rogers (Naval Academy)
- `Brent Jett 180 Back.pdf` — 1978–82 USAF Academy simulator study
- `FAA Advisory Circular on CFI guidelines AC_61-83K.pdf`
- `AFH Ch6 Draft Revisions v1.docx`, `AFH Ch18 Draft Revisions v1.docx` — proposed FAA Airplane Flying Handbook edits
- `AC Draft v4.docx`, `Staff Summary v3.docx`
- `Segmented Turnback Analysis V2.pptx`
- See also [eaa-mcsppadden-project/](../eaa-mcsppadden-project/) for CHARLIE-BRIEFING, THEORY-AND-REFERENCES, MATH-VALIDATION

## 9. Open decisions
- D1: PDF export library / format for the data card
- D2: How aggressive to be on safety-margin defaults (1.25× vs 1.5× vs user-selectable)
- D3: Whether to gate in-flight mode behind explicit liability acknowledgement, or punt entirely
- D4: Approach to FAA endorsement — pursue formally or wait for EAA to drive?

## 10. Risk / watch list
- **Liability** — Charlie flagged this directly; needs conservative defaults + clear "planning tool, not a guarantee" framing
- **Bank-angle misuse** — pilots may pick steep banks thinking they help; UI must show the penalty
- **Reaction-time optimism** — 3 sec is aspirational; defaults should reflect realistic startle response
- **Aircraft library coverage** — single-engine only; need to keep `aircraft_config.py` + POH data current

---

### Working session log
| Date | Note |
|------|------|
| 2026-04-15 | Physics audit; descent-gradient bug fixed |
| 2026-04-19 | EAA McSpadden documentation pack finalized |
| 2026-04-29 | Presented to EAA Board |
| 2026-05-08 | Dashboard stood up; entering Phase 2 (safety & education UX) |
| 2026-05-08 | Charlie call: 20+ Phase 2 items captured (see "Charlie call backlog" above). Surface-wind regression flagged. |
| 2026-05-08 | **Sprint A shipped** → A1/A3 (HW-XW transparency + DB runway length fallback), F1/F2/F6 (MSL bold + safety-factor bold + airport-boundary wording on card), G5 (OSM legend swatches), C5 (bigger help icons). 59/59 tests, live. |
| 2026-05-08 | **Sprint B shipped** → C3/C4/C6/C7 (rich help on bank/reaction/alt-step/prop-drag), C1 (flap inputs moved up under aircraft block), C9 (safety-margin slider promoted into sidebar inputs), D1 (winds-aloft direction column, audit-only), D2 (OAT °C vs ISA-deviation toggle in manual weather). 59/59 tests. |
| 2026-05-08 | **Sprint C (partial) shipped** → B1: added C150 (100 hp) and C152 (110 hp) primary trainers to aircraft list with POH ground-roll entries (735 / 725 ft SL ISA). Validated. E1 (intersection departure) / E2 (gear-retraction timing) / E3 (heading-vs-track wiring) deferred — require engine plumbing, will design separately. |
| 2026-05-08 | **Sprint C2 shipped (E1/E2/E3 plumbing)** → engine: new `intersection_offset_ft`, `gear_retract_time_s`, `wind_dir_profile` params threaded through `simulate_turnback`/`simulate_straight_ahead`/`find_critical_altitude`/`find_straight_ahead_max_altitude`/`build_turnback_envelope`/`optimize_turnback`. New `_interpolate_wind_dir_profile` helper (sin/cos to handle 0/360 wrap). UI: intersection-offset input under runway model, "Retract gear after failure" checkbox + delay input under prop drag, winds-aloft direction column now consumed end-to-end with true→runway-relative conversion. 64/64 tests (5 new in `test_wind_profile.py`). Smoke: C182RG retract@1s drops crit alt 738→604 ft. |
| 2026-05-08 | **Sprint D1 shipped (data-card depth)** → F3: new `altitude_at_departure_threshold` + `altitude_at_takeoff_threshold` (post-hoc trajectory scan with linear interp); F4: new `phase_summary` from per-phase trajectory walk → "Sequence of Events" table on the data card with reaction/turn/return/orbit explainers; F5: MSL alongside AGL on threshold-crossing rows; cleaner landing-direction wording. 67/67 tests (3 new in `test_data_card.py`). G1/G4 (map-side dead-zone arcs + heart on satellite map) deferred to Sprint D2. |
| 2026-05-08 | **Sprint D2 shipped (visualization)** → G1: dead-zone glide-reach arcs (red = sa-max edge, yellow = turnback critical edge) overlaid on the 2D Plan View, centered at the failure point along the climb-out (uses L/D and 5° climb gradient); G4: heart-shape ground tracks (LEFT/RIGHT turnback at critical alt + straight-ahead-max) overlaid on the satellite map via new `envelope_tracks` parameter on `build_satellite_map`, with (x,y)-ft → (lat,lon) conversion using runway heading and atan2(x,y) bearing offset. 67/67 tests still pass. |
| 2026-05-08 | **Sprint A regression triage shipped** → A1: surface wind kt + direction inputs `step=5→1` (Charlie can now enter 8/12 kt, was the regression).  A2: crosswind sign convention audited end-to-end — plumbing correct, but `_recommended_turn_direction` rationale text was misleading (claimed back-leg headwind; back-leg crosswind is actually symmetric).  Rewrote rationale to reflect the real benefit (turn into the wind → tighter ground arc, upwind finish).  A3: confirmed already fixed in Sprint A via `runway_length_published`.  67/67 tests. |
| 2026-05-08 | **Sprint C8 shipped (flap-retract logic)** → New sidebar input "Retract takeoff flaps at (ft AGL)" appears only when takeoff flaps > 0 (default 400 ft).  Data card gets a "Flap-retract altitude" row.  When the critical turnback altitude falls below the retract altitude, the card shows a "FLAPS STILL OUT" warning explaining the Turn-flap selection assumes the pilot is already clean and that real-world drag will be higher.  +3 tests → 70/70. |
| 2026-05-08 | **Charlie's 4 decisions logged** → (1) safety margin = 1.25× default already, (2) bank-angle panel = front-and-center already inline, (3) reaction-time default 3→5 sec realistic (Charlie's call), (4) flap strategy = optimizer already gives the answer.  Updated `_workspace/charlie-call-2026-05-08.md` with decision log header. |
| 2026-05-08 | **Sprint cleanup ship** → C2: safety-margin slider moved up to right after climb-out speed (was buried at bottom).  F7: explicit "airport boundary, not runway" `st.info` banner above the satellite map (OSM polygon deferred).  H1: new 7-section TRAINING-CURRICULUM.md (4-session CFI lesson plan, 3 personas, 4 numbers, rules of the brief).  H2: new "Training Brief" section on the data card (180° myth, bank trade, reaction is the killer, safety-margin reasoning).  70/70 tests. |
