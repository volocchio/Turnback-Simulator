# Charlie Precourt & EAA McSpadden Project — Turnback Simulator Requirements

## Who Is Charlie Precourt?

**Charlie Precourt** is a retired Space Shuttle pilot (4 missions: STS-41, STS-50, STS-78, STS-91). The Space Shuttle is a **100% deadstick glider** after MECO (Main Engine Cutoff) — it has zero propulsion for reentry and landing. Charlie has real experience managing the ultimate engine-out scenario: controlling a 100-ton hypersonic glider descending at 10,000 fpm with a 5,000-foot landing envelope.

**His expertise on this topic is unquestionable.**

---

## The EAA McSpadden Project Mission

Charlie is leading the EAA's effort to replace folklore with facts. The problem:
- **Folklore number**: Pilots walk around with "800 feet" in their head as safe turnback altitude — with no rationale.
- **FAA gap**: AC 61-83K (2024) mandates training pilots on "if, when, and how" to return to the airport after power loss on takeoff, but provides NO supporting materials.
- **Real situation**: An off-duty Cherokee pilot discovered through testing that his aircraft couldn't turnback from ANY altitude — CFIs had steered him wrong.
- **The tragedy**: Rich McSpadden (USAF Thunderbird, AOPA Safety Foundation) died attempting a turnback in a Cessna Cardinal when the engine quit on takeoff.

---

## Charlie's Vision for the Simulator

### High-Level Goal
> "Without an app, we're not going to move the needle. We need pilots to have a tool to **apply the academics** to their daily flying."

Charlie wants a simple, pilot-friendly takeoff-day planning tool that:
1. Takes real-time conditions and aircraft parameters
2. Outputs a "Takeoff Data Card" (like TOLD cards on jets) with decision criteria
3. Tells the pilot: "Land ahead, don't try it" OR "You can turnback IF..."

---

## Required Inputs (From Charlie's Email)

Your simulator should accept:

| Category | Parameters |
|----------|-----------|
| **Aircraft** | Aircraft Type, Gross Weight, Flap Setting (takeoff vs. retracted) |
| **Airport & Runway** | Airport, Runway in use, Runway condition (dry/wet) |
| **Weather (METAR)** | Temperature, Winds, Altimeter setting |
| **Upper-Level Winds** | Winds at 3,000 ft AGL (pilot gets from ForeFlight) |
| **Pilot Inputs** | Climb speed choice (Vx/Vy/other), Reaction time (3/5/10 sec), Bank angle (30/40/45°), Turnback speed (Vg, Vs+5, other), Flap strategy |

---

## Required Outputs (Takeoff Data Card)

Charlie wants these specific data points:

| Output | Purpose |
|--------|---------|
| **Takeoff Distance to 50 ft AGL** | Reference point for calculations |
| **Last Abort Point on Runway** | If engine fails here or later, you **cannot** stay on the runway; alternative is attempt turnback or land ahead |
| **Turnback Altitude** | Minimum altitude where a successful turnback to the runway is possible |
| **Threshold Crossing Altitude** | Expected altitude over the threshold during turnback (gives pilot landing margin) |
| **Turn Direction** | Which way to turn based on runway heading and upper winds |
| **Straight-Ahead Max Altitude** | If aborting the turnback and going straight ahead (useful for obstacle clearance planning) |

### Secondary Feature (Future)
- Google Earth overlay showing off-field landing candidates between last-abort-point and turnback-altitude zone

---

## Critical Physics Challenge: Turnback Altitude Loss

Charlie highlights a **gap in available data**:

> "One element is not provided in any flight test data and that's altitude loss in the turnback."

**The Problem:**
- POH charts give climb rate, glide ratio, stall speeds — but NOT the altitude **lost during the actual turn maneuver** itself.
- Prof. Rogers (Naval Academy Aero Dept) has published methods to estimate this, but they require airfoil coefficients not typically available to pilots.

**Charlie's Concern:**
- Any simulator will have a weakness here.
- **Solution needed**: Apply appropriate safety margins so pilots get reliable results without needing "exceptional flying skills."
- Consider seeking **FAA endorsement** of the tool so it can be safely recommended in their AC.

---

## How This Informs Your Simulator

### ✅ Already Implemented
Your current [turnback_app.py](../turnback_app.py) and [flight_physics.py](../engine/flight_physics.py) already handle:

1. **Physics-based glide descent**: `H_dot = -D / (nz * W)` — you have zero-thrust descent.
2. **Load factor in turn**: `nz = 1 / cos(φ)` — you account for increased stall speed and drag.
3. **Turn radius**: `R = V_TAS² / (g * tan(φ))` — critical for the ground track.
4. **Best glide calculation**: You derive optimal `V_bg` from lift/drag envelope.
5. **Runway model**: Finite-length runway with landing-flap deployment and forward slip.
6. **Wind effects**: Wind relative to runway heading and upper-level winds at altitude.
7. **Aircraft library**: `aircraft_config.py` has detailed aerodynamic coefficients.
8. **Optimizer**: Recommends best bank angle and flap strategy for max turnback altitude.

### 🔶 Partially Addressed
- **Flap strategy**: You simulate flaps down vs. retracted, but UI might not make it obvious that:
  - Takeoff flaps increase drag and sink rate (bad for initial climb-out)
  - Retracted flaps are typically used by the time turnback becomes feasible
  - Charlie wants pilots to understand this tradeoff visually

- **Reaction time**: Included in your input (`expected_reaction_time`), but the UI should emphasize its importance — 3 vs. 10 seconds can be the difference between doable and impossible.

### ⚠️ Key Gaps to Address

1. **Takeoff Data Card Output Format**
   - Your simulator produces a 3D envelope and 2D visualization — **excellent**.
   - But Charlie wants a **simple, printer-friendly data card** that a pilot can fill out and keep in the cockpit (like a TOLD card).
   - **Suggested feature**: Add a "Download Takeoff Data Card" button that outputs a clean PDF or table with the key numbers.

2. **Safety Margin / Confidence Intervals**
   - For the "Turnback Altitude" output, add a **margin notification** (e.g., "Calculated 800 ft, but recommend 1000 ft minimum to account for tracking error and sub-optimal flying").
   - Tie this to the theoretical work by Prof. Rogers on altitude loss during the turn.

3. **Straight-Ahead Performance**
   - Your UI should show not just "turnback possible at altitude X" but also "if you abort and go straight ahead, you'll clear obstacle at Y ft."
   - This is the backup plan when turnback fails.

4. **Bank Angle Education**
   - The UI mentions bank angle (30/40/45°), but pilots often don't understand why steep banks kill performance.
   - Show side-by-side stall speeds and sink rates for 30° vs. 40° vs. 45° — make the penalty obvious.

5. **Academic References**
   - Add links or footnotes to the papers Charlie mentioned:
     - **Prof. Rogers**: "Estimating Turnback Altitude"
     - **Brent Jett (astronaut)**: Late-70s/early-80s simulator study of the impossible turn
     - **FAA AC 61-83K** (para A.114): The regulatory mandate
     - **EAA Sport Aviation article** on power loss on takeoff

---

## Recommended Next Steps

### Phase 1: Output Format (High Priority)
- [ ] Add "Takeoff Data Card" section with clean, pilot-friendly output table
- [ ] Include PDF export option
- [ ] Show turnback altitude, last abort point, threshold crossing altitude, turn direction

### Phase 2: Safety & Education (Medium Priority)
- [ ] Add safety margin visualization and text guidance
- [ ] Bank angle sensitivity graph (stall speed, sink rate vs. bank)
- [ ] Reaction time sensitivity (show how 3 sec vs. 10 sec changes outcome)
- [ ] Link to academic papers and FAA AC

### Phase 3: Integration (Lower Priority, Future)
- [ ] Google Earth overlay for off-field landing zones
- [ ] ForeFlight integration for real-time wind input
- [ ] Real-time in-flight mode (require explicit user opt-in due to liability)

---

## Connection to Shuttle Experience

Charlie's shuttle background is **directly relevant**:

- **Shuttle glide ratio**: ~4:1 (roughly equivalent to a Cessna 172)
- **Commitment altitude**: STS has a point of no return during approach where you cannot abort to a runway and must land on the intended runway — exactly like a turnback decision.
- **Sideslip / slip control**: Shuttle uses forward slips to steepen descent angle without increasing airspeed — same tool small-plane pilots would use.
- **Crew coordination under stress**: Shuttle training emphasizes discipline and procedures under emergency stress — applies to any engine-out decision.

Charlie knows from 4 shuttle missions that **the numbers don't lie**, and pilots often overestimate what's actually possible. This drives his commitment to replacing folklore with facts.

---

## Liability & FAA Endorsement

Charlie raised an important point:
> "I want to have something we (EAA) could put out to the community without a lot of concern for liability… or perhaps we get an FAA endorsement to refer to it in their AC."

**Suggestion**: 
- Build in conservative safety margins (1.5x to 2x the calculated altitude)
- Document the margins clearly ("Calculated minimum: X ft | Recommended minimum: Y ft")
- Pursue FAA review and possible endorsement in AC 61-83K update
- Emphasize in documentation that simulator is a planning tool, not a guarantee, and depends on pilot skill

---

## References (From Charlie's Email)

1. **FAA AC 61-83K** (2024): Advisory Circular on Biennial Flight Review — paragraph A.114
2. **Prof. Rogers Paper**: "Estimating Turnback Altitude" — methods for calculating altitude loss in the turn
3. **Brent Jett Paper**: Late 1970s/early 1980s study using simulators
4. **EAA Sport Aviation Article**: May 2026 edition (linked in Charlie's email)
5. **TLAR Pilot Tool**: www.TLARpilot.com — reference implementation (more complex UI, but good for comparison)

---

## Bottom Line

Your simulator is **already 80% of what Charlie needs**. The physics is solid. The visualization is excellent. The key gaps are:

1. **Output format**: Make it pilot-friendly with a downloadable data card
2. **Safety messaging**: Add margin guidance and caveats  
3. **Education**: Explain *why* things work the way they do (bank angle, reaction time, wind effects)

Charlie has 4 shuttle missions under his belt. He won't accept folklore. Your simulator gives him the tool to replace it with facts — but only if the output is accessible and actionable to the average pilot.

---

**Last Updated**: April 19, 2026  
**Prepared for**: EAA McSpadden Project Oshkosh Board Meeting (April 29, 2026)
