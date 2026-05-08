# Turnback Simulator — Charlie Call, May 8, 2026

**turnback.voloaltro.tech** · single-engine, physics-based, EAA McSpadden Project

---

## Where we are

| ✅ Done | 🔄 In flight | 🎯 Next |
|---------|-------------|---------|
| Live deployment | Phase 2 pilot UX | FAA endorsement path |
| Physics audit (Apr 15) | Safety-margin surfacing | Off-field landing overlay |
| Takeoff Data Card | Bank-angle education | PDF export polish |
| EAA Board presentation (Apr 29) | Reaction-time defaults | |

**Critical fix since we last spoke:** descent-gradient equation was wrong (had `sin γ = −D/(n_z·W)`, correct is `sin γ = −D/W`). Sink rate was being underestimated **41% at 45° bank, 100% at 60°**. Pilots would have planned with rosier numbers than reality. Now correct everywhere — main descent loop, forward-slip, orbit estimate, straight-ahead.

---

## Four decisions I need from you

### 1. Default safety margin
Rogers' altitude-loss-in-the-turn isn't in any POH. We multiply the calculated turnback altitude by a margin and surface it as **"Calculated 800 ft / Recommended ___ ft."**
> **Aggressive 1.25× ··· Middle 1.35× ··· Conservative 1.5×**

### 2. Bank-angle panel — front and center, or "Why?" disclosure?
Pilots intuitively pick steep banks. Side-by-side stall-speed (×√n_z) and sink-rate penalty for 30° / 40° / 45° makes the cost obvious.
> **Default panel ··· Hidden behind a "Why bank matters" expand**

### 3. Reaction-time default
3 sec is aspirational. Startle-response literature says 5–7 sec realistic.
> **3 sec optimistic ··· 5 sec realistic ··· 7 sec conservative**

### 4. Flap strategy
Optimizer already picks. Question is what we show the pilot.
> **Just give them the answer ··· Show the tradeoff visually**

---

## Three things I want to ask you

1. **Apr 29 board reaction** — any specific committee asks I should fold into Phase 2?
2. **EAA Sport Aviation May 2026 article** — published? Driving traffic to the tool?
3. **FAA endorsement** — do you want to drive the AC 61-83K reference, or do you want me to draft an outreach package?

---

## Future scope (parking lot)

- Google Earth overlay — off-field landing zones between last-abort and turnback-altitude
- ForeFlight integration — live winds at 3,000 AGL
- In-flight/real-time mode — only behind explicit liability acknowledgement
- TLAR Pilot head-to-head comparison doc

---

## Reference library now in repo
Rogers · Brent Jett · AC 61-83K · AFH Ch6 + Ch18 draft revisions · Segmented Turnback Analysis V2 · Staff Summary v3
