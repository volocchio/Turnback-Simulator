# EAA McSpadden Project: Document Quick Reference

## What's in This Folder?

This folder contains the academic and reference materials for the Turnback Simulator, commissioned by the **EAA McSpadden Project** to replace folklore with facts.

---

## 📄 Documents

### 1. **CHARLIE-BRIEFING.md**
**Purpose**: Executive overview  
**Audience**: CFIs, flight instructors, program administrators  
**Length**: ~10 min read

**What it covers**:
- Who is Charlie Precourt (4× Space Shuttle pilot)
- The EAA McSpadden Project mission (replacing "800 feet" folklore)
- High-level requirements for the simulator
- Key gaps in current pilot training

**Read this if**: You want to understand the *why* behind the project.

---

### 2. **THEORY-AND-REFERENCES.md**
**Purpose**: Complete physics and theory documentation  
**Audience**: Engineers, CFI-Aeronautical Knowledge students, aircraft designers  
**Length**: ~30 min read

**What it covers**:
- Core aerodynamic equations (with derivations)
- Load factor, stall speed, drag polar
- Best-glide speed formula
- Turn radius and turn rate
- Altitude loss during the turn (the critical unknown)
- Regulatory foundation (FAA AC 61-83K)
- Academic references (Rogers, Jett, FAA Handbook)
- Validation against published data

**Read this if**: You want to understand the *math* behind the simulator.

---

### 3. **MATH-VALIDATION.md**
**Purpose**: Detailed comparison of simulator to academic publications  
**Audience**: Flight engineers, simulator developers, academics  
**Length**: ~20 min read

**What it covers**:
- Equation-by-equation comparison (our code vs. FAA Handbook)
- How we estimate altitude loss (time-step integration vs. closed-form)
- Validation against Brent Jett's 1970s–1980s simulator study
- Validation against TLAR tool (industry standard)
- Why safety margins are 1.25–1.5× (not a guess)
- Known limitations and future improvements

**Read this if**: You want to verify that our math is correct.

---

### 4. **charlie-precourt-email.txt**
**Purpose**: Original requirements from Charlie Precourt  
**Audience**: Program stakeholders, project historians  
**Length**: ~15 min read

**What it includes**:
- Charlie's background (shuttle pilot, EAA committee member)
- The problem: "800 feet" folklore with no rationale
- The tragedy: Rich McSpadden died attempting a turnback
- Specific input/output requirements for the simulator
- References to attachments (Prof. Rogers' paper, Brent Jett's study, EAA white papers)

**Read this if**: You want to understand the original stakeholder needs.

---

## 📊 How These Documents Relate

```
Charlie-Precourt-Email (Requirements)
    ↓
CHARLIE-BRIEFING (Executive Summary)
    ↓
THEORY-AND-REFERENCES (Academic Foundation)
    ↓
MATH-VALIDATION (Verification)
    ↓
Simulator UI (Collapsible "Theory & References" section)
```

---

## 🎯 For Specific Audiences

### Flight Instructors / Pilots
**Start here**: CHARLIE-BRIEFING.md  
**Then**: Collapsible theory section in the simulator UI  
**Why**: Understand the *problem* and the *safe approach*

---

### Aircraft Engineers / Simulationists
**Start here**: THEORY-AND-REFERENCES.md  
**Then**: MATH-VALIDATION.md  
**Why**: Verify the physics is correct, understand modeling choices

---

### CFI-Aeronautical Knowledge Students
**Start here**: THEORY-AND-REFERENCES.md  
**Then**: MATH-VALIDATION.md (equation-by-equation)  
**Then**: FAA Airplane Flying Handbook (Chapters 3, 6, 18)  
**Why**: Deep understanding of engine-out handling

---

### Program Managers / Stakeholders
**Start here**: CHARLIE-BRIEFING.md  
**Then**: charlie-precourt-email.txt  
**Then**: THEORY-AND-REFERENCES.md (just the regulatory section)  
**Why**: Understand mission, scope, and regulatory basis

---

## 🔗 External References

Charlie mentioned several attachments that should be obtained separately:

1. **FAA Advisory Circular 61-83K** (2024)
   - URL: https://www.faa.gov/regulations_policies/advisory_circulars/
   - Key section: Paragraph A.114 (Biennial Flight Review requirements)

2. **Prof. James F. Rogers: "Estimating Turnback Altitude"**
   - Source: EAA McSpadden Project committee
   - Contact: Charlie Precourt (precourt@comcast.net)

3. **Brent Jett: "The Impossible Turn" (1978–1982 simulator study)**
   - Source: USAF Academy Senior Research Project
   - Published via: EAA materials, McSpadden Project

4. **EAA Sport Aviation Magazine (May 2026)**
   - Article: "Power Loss on Takeoff"
   - Link: https://acrobat.adobe.com/id/urn:aaid:sc:VA6C2:22408eb6-7c51-4b06-9343-87dbc8f8aa26

5. **TLAR Pilot Tool**
   - Website: www.TLARpilot.com
   - Purpose: Reference implementation (industry standard for impossible turn analysis)
   - Note: More complex UI than our simulator; recommend "Pro" version for turnback feature

6. **FAA Airplane Flying Handbook**
   - Chapters 6 (Emergency Procedures) & 18 (Engine Failure on Takeoff)
   - URL: https://www.faa.gov/sites/faa.gov/files/regulations_policies/handbooks_manuals/aviation/airplane_handbook/
   - Proposed edits under EAA McSpadden Project

---

## 💡 Key Takeaways

### The Problem
Pilots walk around with "800 feet" in their head, with no understanding of where this number comes from. The FAA mandate to train engine-out turnback scenarios has no supporting teaching materials.

### The Solution
A physics-based simulator that:
- Calculates the true critical altitude (minimum altitude to safely complete turnback)
- Shows why the numbers come out as they do (bank angle, wind, reaction time effects)
- Provides conservative safety margins (1.25–1.5×) to account for pilot skill variability
- Directly supports the FAA regulatory mandate (AC 61-83K, para A.114)

### The Validation
- Our math aligns with standard aerodynamics (FAA Handbook)
- Altitude loss estimation follows Prof. Rogers' methodology
- Results validate against published studies (Jett, TLAR)
- Safety margins are justified, not guesses

### The Impact
Pilots can now make informed, data-driven decisions about whether a turnback is feasible, replacing folklore with facts.

---

## 📧 Questions?

For questions about the physics, theory, or validation:
- **Theory & References**: See THEORY-AND-REFERENCES.md
- **Math Details**: See MATH-VALIDATION.md (equation-by-equation)
- **Original Requirements**: See charlie-precourt-email.txt

For questions about the simulator itself:
- See the collapsible **"Theory & References"** section in the simulator UI
- Check the README.md in the project root for setup/deployment

For questions about EAA McSpadden Project:
- Contact: Charlie Precourt (precourt@comcast.net)
- Oshkosh presentation: April 29, 2026 (EAA Board Meeting)

---

**Last Updated**: April 19, 2026  
**Simulator Version**: 1.0  
**Status**: Ready for EAA Board Presentation
