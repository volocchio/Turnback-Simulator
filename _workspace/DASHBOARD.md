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
- [ ] **Surface wind speed (kt) input** — Charlie says it was good before, now broken. Need to repro and diff against last-known-good (commit `b775e22` Charlie feedback, or earlier).
- [ ] Crosswind logic "weird" — re-check sign convention end-to-end (sidebar → sim → recommended-direction → card rationale)
- [ ] Data Card runway lengths showing 0 when "Enable runway model" is OFF — surface DB length anyway

**B. Aircraft library**
- [ ] Add typical trainers: C150, C152, RV-12, plus inventory pass (Piper Tomahawk, Diamond DA20, etc.)

**C. Sidebar UX (climb-out / flap / bank / reaction zone)**
- [ ] Move flap inputs higher in the panel
- [ ] Climb-out speed: interactive flap-retract logic — "takeoff flaps → retract at XXX ft → clean climb"
- [ ] Bank-angle slider help bubble (stall ×√n_z, sink-rate cost)
- [ ] Reaction-time help bubble (3 sec = FAA assumption; 5–7 sec realistic)
- [ ] Bigger help-bubble icons + larger help text (current ones are hard to see)
- [ ] Altitude-step help bubble
- [ ] Prop-drag help bubble: feathered/stopped/windmilling logic. Default to user's choice and show 3 comparison rows
- [ ] Runway-model help bubble: glide angle 5/6/7°, no-flap stretch, no flare margin, hammer "preserve flaps for flare, kill higher sink"
- [ ] Safety-margin slider repositioned right after climb-out speed

**D. Wind / atmosphere inputs**
- [ ] Winds-aloft: add direction column at 0/1000/2000/3000 (currently speed only)
- [ ] Temperature input: user picks °C or ISA-deviation (default = °C / OAT)

**E. Operations realism**
- [ ] Intersection-departure mode (not recommended but sometimes ATC-directed) — let user pick takeoff start point along the runway
- [ ] Gear-retract timing: when's the best time for gear back out? Show penalty for leaving it down
- [ ] Heading-vs-track display toggle (compare runway-heading hold vs runway-track hold)

**F. Card / output**
- [ ] MSL altitude on the card, **bold** (pilot reads MSL on the altimeter)
- [ ] Safety factor in **bold** on the card
- [ ] Show altitude profile at departure-end threshold for the downwind landing case
- [ ] Per-section explainer text on the card ("180° section is for X")
- [ ] Runway-remaining point: show AGL/MSL when airborne — "how will the pilot know?"
- [ ] Straight-ahead landing limit = stay within **airport boundary**, not just runway asphalt

**G. Visualization**
- [ ] Two arcs on the runway map showing top + bottom of the dead zone (instead of top-down only)
- [ ] 3D view eyepoint: dead-zone, turnback path
- [ ] Pilot-perspective flyby motion mode
- [ ] Heart-shape envelope: training-only, move it to the map
- [ ] OSM legend: color swatches next to good/caution/avoid

**H. Curriculum / framing**
- [ ] Turn this into a training curriculum: what it means for a CFI day-to-day, and for working pilots
- [ ] Add training explainers in output ("180° just means…")

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
