# Checking Every Motor & Matching Real ↔ Sim — Simple Explanation First

**Read order:** Section 1 first (the concept, no jargon). Sections 2–4 are
the deep detail, for when you need the actual numbers or code. Section 5 is
the action list — what to actually go do next.

---

## 1. The Concept — In Plain Words

### Step 1: Check each real motor works, at all
Before matching anything to the simulation, we first need proof that each
of the 6 motors on the real arm actually moves properly, on its own — does
it reach where we tell it to, does it hold still, does it struggle when the
arm is extended vs. folded.

### Step 2: Find each motor's min and max
Every motor has a **minimum position** and a **maximum position** it can
physically reach — its "empty to full" range. We find this once per motor,
per arm, using a calibration wizard: you physically move each joint to
both of its extremes, and the software records those two numbers.

### Step 3: Use that min/max to translate into the simulator
The simulator has its **own** idea of min/max for each joint (based on the
3D model's geometry, not the real motor). So we built one script that does
this translation:

> *"The real motor is currently 62% of the way between ITS min and max.
> So put the simulated joint at 62% of the way between ITS OWN min and
> max."*

That's it. That's the entire matching trick — **percentage of travel**,
not a raw number copied across. It works for any joint the same way,
because it only ever needs that joint's own min and max.

### Step 4 (the part still incomplete): Prove it BEHAVES the same, not just LOOKS the same
Matching min/max only proves "the joint ends up in the right spot." It
doesn't prove the joint **moves** the same way — same speed, same
smoothness, same struggle under weight. That needs a separate, deeper
check per motor: run it through real tests (small moves, big moves, back
and forth, loaded and unloaded) and use those results to make the
simulated version behave the same way, not just look the same at rest.

**This deeper check has only been done for 2 of the 6 motors so far**
(elbow and shoulder-lift). The other 4 have the min/max matching done
(Step 3), but not the deeper behavior check (Step 4) — they're currently
just *assumed* to behave similarly, because it's physically the same part.
That assumption hasn't been tested yet.

---

## 2. Every Motor — What We Know Right Now

All 6 motors are the same part number (**Feetech STS3215**), just used in
different joints with different jobs (some spin the whole arm, some just
open/close a gripper). Each one gets its own ID number and its own
calibration.

| # | Joint (motor location) | Motor ID | Real min↔max (ticks) | Real range (degrees) | Sim range (degrees) | Behavior fully tested? |
|---|---|:--:|---|---:|---:|:--:|
| 1 | `shoulder_pan` — base rotation | 1 | 1057 ↔ 3085 | 178.2° | 220.0° | ❌ No |
| 2 | `shoulder_lift` — shoulder up/down | 2 | 822 ↔ 3252 | 213.6° | 200.0° | ✅ **Yes** |
| 3 | `elbow_flex` — elbow bend | 3 | 905 ↔ 3136 | 196.1° | 193.7° | ✅ **Yes** |
| 4 | `wrist_flex` — wrist up/down | 4 | 821 ↔ 2971 | 189.0° | 190.0° | ❌ No |
| 5 | `wrist_roll` — wrist spin | 5 | 0 ↔ 4095 | 359.9° (full spin, no stop) | 314.4° | ❌ No |
| 6 | `gripper` — open/close jaw | 6 | 1822 ↔ 3381 | 137.0° | 110.0° | ❌ No |

**How to read this table:**
- **Motor ID** — every motor on the daisy-chain bus has its own address
  (1 through 6), so commands go to the right one.
- **Real min↔max (ticks)** — the raw sensor reading recorded during
  calibration for THIS specific physical arm. Another arm of the same
  design would get slightly different numbers here (measured proof of
  this exists — see §3.1).
  - **`shoulder_pan` (id 1)**, ~1057–3085 — from `shoulder_pan.pos` on the SO101Follower calibration file.
- **Real range (degrees)** — that tick range converted to degrees, just
  for human readability.
- **Sim range (degrees)** — what the simulator's own 3D model allows for
  that joint, completely independent of the real motor's calibration.
- **Behavior fully tested?** — did we actually run this motor through the
  "does it speed up/slow down/struggle under load the way we expect"
  tests, or are we just trusting the min/max match and assuming the rest?

**Notice the real and sim ranges never match exactly, for any joint.**
That's expected, not a bug — see §3.2 for why.

---

## 3. Deep Detail (skip this section if the table above already answered your question)

### 3.1 Why "min and max" differs per physical arm, even same model

The motor's internal sensor doesn't have a built-in "zero" that means
anything physical — it's just wherever the sensor happened to be pointing
when the part was bolted together at the factory/assembly. Two arms built
the same way can have their sensor's "zero" rotated slightly differently.

We proved this directly: two of our own arms measured 23° apart on the
exact same joint (`shoulder_pan`) purely from this kind of assembly
variation. **This is why every new physical arm needs its own
calibration** — you cannot reuse another arm's numbers.

### 3.2 Why real range and sim range never match exactly

- **Real range** = however far a human happened to physically sweep that
  joint during calibration, bounded by wherever the joint's hard mechanical
  stop actually is on that specific unit.
- **Sim range** = however far the 3D model's designer decided the joint
  should be *allowed* to rotate in the digital model, based on idealized
  geometry.

These are two independently-decided numbers. They're close, but not
identical, by nature — see the table above, every row differs by a few to
several degrees. `wrist_roll` is the extreme case: it physically spins in
a full unbroken circle (no real stop at all), so its "real range" of
359.9° isn't really a measured limit — it's just "the whole dial."

### 3.3 The actual matching script (Step 3, already built and tested)

File: `scripts/digital_twin_env/real_sim_mapping_test/real_sim_joint_mapping.py`

```python
def real_to_sim(joint_name, real_normalized, sim_range_rad):
    lo, hi = sim_range_rad
    if joint_name == "gripper":
        frac = real_normalized / 100.0        # gripper: 0..100
    else:
        frac = (real_normalized + 100) / 200  # others: -100..100
    return lo + frac * (hi - lo)
```

This one function is used **identically for all 6 joints** — it was never
written to be joint-specific. Give it any joint's own min/max and it
works. This is why min/max matching (Step 3) is already done and correct
for every motor, even the 4 whose deeper behavior hasn't been tested yet.

Proven correct with a headless test (no hardware needed) that checks:
boundary values land exactly right, converting there-and-back returns the
original number, and values outside the real range get capped instead of
producing a nonsense sim angle.

### 3.4 What "behavior tested" (Step 4) actually involves

For the 2 motors that HAVE been fully tested (`elbow_flex`,
`shoulder_lift`), here's what "tested" concretely means — and what it
would take to do the same for the other 4.

**The 5 things measured per motor:**

| What's measured | Elbow result | Shoulder-lift result | What it tells you |
|---|---:|---:|---|
| Delay before it starts moving | 65.1 ms | (measured, similar order) | Reaction time — turned out to be fixed, doesn't change with load |
| How far off target it settles | 0.128° (no load) / 0.472° (loaded) | Worse — ~10x higher error than elbow | Whether the joint "arrives" accurately |
| Top speed | 191.6°/s | Notably lower | How fast it can move |
| Mechanical play/slack (backlash) | 1.196° | 1.955° | Looseness in the gears — bigger number = more slop |
| Behavior under load (arm extended vs. folded) | Error nearly quadrupled under load, speed barely changed | Load affects it MORE than the elbow | Whether gravity/weight throws off accuracy |

**The key finding that matters for the other 4 motors:** even though the
elbow and shoulder-lift use the exact same motor part, their real-world
behavior turned out meaningfully different — shoulder-lift is worse across
almost every measurement, because it physically carries more weight
(it's holding up the whole rest of the arm, not just itself). **This is
direct proof that "same motor part" does NOT guarantee "same behavior."**
So assuming the other 4 motors will behave like the elbow, without
actually testing them, is a real gap — not a safe shortcut.

---

## 4. What Happens If You Skip Step 4 (why it matters practically)

Right now, if you drive `shoulder_pan`, `wrist_flex`, `wrist_roll`, or the
`gripper` in the mirrored real+sim view (`twin.py mirror --source real`):

- The **position** shown in sim will be correct — because Step 3's min/max
  matching works for every joint already.
- The **way it moves** (how fast it speeds up, whether it overshoots,
  whether it stutters near the end) might NOT match reality — because the
  simulator is using the elbow's fitted behavior numbers as a stand-in,
  never verified against that specific joint.

This matters most for training/dataset work later: a policy trained
against a sim that gets positions right but movement-*feel* wrong is
learning something subtly false about how the real arm responds.

---

## 5. What To Do About It — Action List

**For each of the 4 untested motors** (`shoulder_pan`, `wrist_flex`,
`wrist_roll`, `gripper`), repeat the exact same 2-step process already
proven on the elbow:

### Step A — Run the real hardware test (arm connected, ~30–60 min per joint)
```powershell
python scripts\twin.py characterize --joint shoulder_pan
python scripts\twin.py characterize --joint wrist_flex
python scripts\twin.py characterize --joint wrist_roll
python scripts\twin.py characterize --joint gripper
```
This commands the joint through the same 5 experiment types already used
on the elbow (small steps, reverse direction, ramps, triangle zig-zag,
loaded vs. unloaded), logging real position/load/current the whole time.

### Step B — Analyze the results (no hardware needed)
```powershell
python scripts\twin.py analyze --joint shoulder_pan
```
Produces the same 5 headline numbers shown in the table above (latency,
settling error, top speed, backlash, load effect) for that joint.

### Step C — Fit the simulator's behavior to match (no hardware needed)
```powershell
python scripts\twin.py tune --joint shoulder_pan
```
Finds the simulator settings that make the simulated joint move like the
real one moved in Step A, and checks the fit against data it wasn't shown
during fitting (so it's not just memorizing, it's genuinely matching).

### Priority order — do these in order, not all at once
1. **`gripper` first** — it's flagged as the most likely to behave
   differently from the elbow (different job entirely: squeezing an
   object vs. rotating a link), so it has the most to gain from being
   tested rather than assumed.
2. **`wrist_flex`** — similar rotating-joint job to the elbow, so lower
   risk, but still unverified.
3. **`shoulder_pan`** — rotates the whole arm's base; different loading
   than the elbow, worth checking.
4. **`wrist_roll`** — lowest priority; it has no hard mechanical stop
   (full spin), so it's the least likely to have the same
   backlash/overshoot issues the other joints have, but still unconfirmed.

**After all 4 are done:** update `docs/PROJECT_STATUS.md`'s Phase A
tracker from "2 of 6" to "6 of 6," and the per-motor table in this file
(§2) can have every row marked ✅.
