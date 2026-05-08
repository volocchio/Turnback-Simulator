# Turnback Simulator — Training Curriculum

**For:** CFIs, Designated Pilot Examiners, and working single-engine pilots
**Tool:** [turnback.voloaltro.tech](https://turnback.voloaltro.tech)
**Reference:** AC 61-83K · AFH Ch 6 (proposed revisions) · Rogers (USNA) · Jett (USAFA)
**Authors:** Nick Guida (Volo Altro / Tamarack) · Charlie Precourt (EAA McSpadden Project)

---

## 1. Why this exists

The "impossible turn" kills competent pilots every year because the decision is being made in five seconds with no number to fall back on. POH data stops at runway end. Nothing in normal flight training computes the altitude required to make the turn back, and almost no pilot has a personal number for their airplane / weight / wind / runway combination.

This simulator produces that number — and surfaces the **dead zone**: the band of altitudes on initial climb where neither a straight-ahead landing nor a turnback is viable.

The curriculum below is **how to teach with it**, not how to use it.

---

## 2. Three pilot personas

| Persona | What they need from the tool | How a CFI uses it with them |
|---|---|---|
| **Pre-solo / private student** | A vocabulary: critical altitude, dead zone, straight-ahead-max. | Pre-takeoff briefing exercise: print a Data Card for today's airplane / weight / runway / wind. Brief the dead zone on every taxi to the hold-short. |
| **Newly-rated private / IFR pilot** | Translate the number into a takeoff brief and a no-go altitude. | Hood / sim drill: simulate the failure at three altitudes (below dead zone, in the dead zone, above critical) and force a verbal commit before they touch the controls. |
| **CFI / working pro** | Per-airplane, per-airport bracketing. Curriculum to give to their own students. | Build a personal library of Data Cards for the trainers in their fleet. Run the simulator side-by-side with the student so the *trade-offs* (bank vs. radius, flaps vs. drag) become visible, not abstract. |

---

## 3. The four numbers every pilot should brief on every takeoff

1. **Calculated critical altitude** (LEFT and RIGHT) — the bare aerodynamic minimum.
2. **Recommended critical altitude** = Calculated × safety factor (default **1.25×**, Charlie's call). Use *this* as the no-go.
3. **Straight-ahead-max altitude** — below this, land within the airport boundary. Not the runway. The boundary.
4. **Dead-zone band** — the altitude window where neither option works. Climb through it as fast as the airplane will go.

> If you can't say all four out loud at the hold-short, you are not briefed.

---

## 4. Lesson plan (4 sessions × 1 hour)

### Session 1 — Ground: the physics
- Why "180°" is a myth (the turn must overshoot ~210–270° to align on final).
- Bank angle ↔ stall margin ↔ turn radius. Demonstrate Vs(φ) = Vs · √(1/cos φ): 30°→+8%, 45°→+19%, 60°→+41%.
- Drag with bank: induced drag scales with CL² → steep banks bleed energy fast.
- Reaction time: 3 sec is FAA assumption, 5–7 sec is realistic startle response, **5 sec is the curriculum default**.
- Run the sim live: change reaction from 3 → 5 → 7 and watch the critical altitude inflate.

### Session 2 — Ground: the airplane
- Build a Data Card for the student's own airplane at three weights (solo, with instructor, MTOW).
- Compare 30° / 45° / 60° bank: identify the per-aircraft sweet spot (usually 45°).
- Compare clean / takeoff-flap / landing-flap turn configurations. Discuss why clean usually wins on energy.
- Identify the dead zone for *this* airplane on *this* runway. Brief it.

### Session 3 — Airwork: feel the bank, feel the stall
- At 3,000 AGL: simulate idle power, hold pitch attitude, count out the reaction time, then enter a 45° bank turn at the recommended speed (Vs(φ) + 10 kt).
- Time the 180°. Measure altitude lost. Compare to the simulator's prediction.
- If real loss > simulated loss, **inflate the personal safety factor** until they match. That is the student's number.
- Repeat at 30° and 60° to feel the trade. Most pilots will pick 45° on their own after this.

### Session 4 — Procedure: brief, climb, commit
- Pre-takeoff verbal brief: "Below {sa_max} feet — straight ahead, anywhere on the airport. Above {recommended} feet — turn {LEFT/RIGHT}, {bank}° bank, {speed} KIAS. Between, no good option — climbing fast."
- Departure with simulated failure at three altitudes:
  - Just above straight-ahead max → land straight, accept it.
  - Mid dead zone → land straight (or off-airport if forced).
  - Just above recommended → execute the briefed turn.
- Debrief on the Data Card numbers vs. what actually happened.

---

## 5. Rules of the brief

- **The lower of LEFT and RIGHT critical altitude wins.** Always.
- **Crosswind tie-breaker:** turn INTO the wind (rolls the airplane into the wind, tightens the ground arc, finishes upwind of the runway).
- **Below straight-ahead-max:** land within the airport boundary. Not the runway. The boundary.
- **Inside the dead zone:** straight ahead. Any deviation costs altitude you don't have.
- **Above recommended:** execute the briefed turn without negotiation.

---

## 6. What the tool does NOT do

- Does not model your specific engine failure mode (partial power, oil out the cowl, structural).
- Does not model pilot panic, fixation, or decision delay beyond the reaction-time slider.
- Does not replace the POH or a current CFI.
- Does not endorse turnbacks. **The default answer is always straight ahead.** This tool quantifies *when* the turnback option becomes viable — never *that it should be taken.*

---

## 7. CFI takeaways

- Run the Data Card with every primary student before solo.
- Re-run it on every BFR / IPC. The student's airplane will have changed; their reaction time will have changed; the dead zone will have changed.
- Use the bank-angle / flap / weight panels to *show* the trade-offs instead of *telling*. The numbers move in real-time.
- Make the four numbers (calculated, recommended, sa-max, dead-zone) part of the takeoff brief vocabulary, the way V-speeds and rotation are.
