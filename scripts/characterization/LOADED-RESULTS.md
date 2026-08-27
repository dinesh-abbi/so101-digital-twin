# Loaded staircase — `shoulder_lift`

Measured 2026-08-26, arm extended roughly horizontal (mass held away from the
joints), base clamped, gripper pointing across the table. 3 staircase repeats,
~93 Hz.

Sweep reduced to **±30 units (±34°)** from the default ±50 for safety: the
shoulder carries the whole extended arm through this motion, and a 113° sweep
from that pose risked the forearm striking the table. That decision was right
for the hardware and costly for the gravity fit — see below.

## Backlash grew under load

| | no load (folded) | loaded (extended) |
|---|---:|---:|
| mean gap, whole sweep | 1.955° | **2.479°** |
| gap at the 0° rung | 2.578° | **2.812°** |

At **0°, the only command both runs share**, backlash grew **+9%**
(2.578 → 2.812). The whole-sweep means differ by +27%, but the two runs used
different rung spacings — the no-load set includes wide rungs (±42°) where the
gap is smaller — so +27% partly reflects which rungs were sampled, not load.
**+9% is the defensible number.**

That backlash grows at all is the finding. Pure gear slop is a fixed
mechanical clearance and should not care about load. A gap that widens under
torque means part of what is being measured is **compliance** — flex in the
gear teeth, the linkage and the horn under load — not just play. The two are
not separable from position data alone.

Per-rung, loaded:

| command | from below | from above | gap |
|---:|---:|---:|---:|
| −22.66° | −22.911 | −20.947 | −1.963 |
| −11.33° | −12.100 | −9.492 | −2.607 |
| +0.00° | −0.791 | +2.022 | −2.812 |
| +11.33° | +11.162 | +13.301 | −2.139 |
| +22.66° | +21.768 | +24.639 | −2.871 |

## The gravity fit failed, and the tool said so

Best R² was **0.010** — neither sin nor cos describes the loaded mean error,
which is essentially flat (range 0.367°, sd 0.138° across five rungs). The
analyzer refused to report an amplitude and printed a warning instead of a
confident number.

The cause is the narrowed sweep. Gravity droop varies as sin(angle), so the
signal available depends on how much sin changes across the swept range:

| run | span | sin variation |
|---|---:|---:|
| no load | ±42.5° | 1.351 |
| loaded | ±22.7° | 0.771 |

Roughly half the leverage, against a flat baseline — not enough to fit. The
no-load run's gravity figure (1.009°, R² 0.880) stands; this run adds nothing
to it.

**To measure loaded gravity properly**, the sweep needs to be wide again
(±50 units) from a pose where that is mechanically safe — most likely with
the forearm folded so the shoulder's arc stays clear of the table, while some
other mass keeps the joint loaded. That is a pose-design problem, not a
measurement one.

## `elbow_flex` loaded: not measured

Deliberately skipped. With the arm extended, `elbow_flex` sits at **+98
units**, essentially at its limit — that is the pose that holds the forearm
horizontal. A probe sweep from there runs to −50 units, a **145° swing** that
would drop the whole forearm onto the table.

Loaded elbow data needs a different pose: forearm hanging down rather than
extended, so its sweep stays in free space.

## Procedural note

The probe was run twice in succession; the second overwrote the first. Between
them the arm went limp and **sagged from −78.8 to −92.9 units**, so the two
runs did not start from the same pose. The analysed data is the second run.
This did not affect the result — every staircase begins with `move_gently` to
the first rung — but it is worth knowing that the "current pose" line differs
between otherwise identical invocations.

## What stands

| quantity | value | confidence |
|---|---:|---|
| `shoulder_lift` backlash, no load | 1.955° | good (sd 0.372, 7 rungs) |
| `shoulder_lift` backlash, loaded | 2.479° | good, but +9% vs +27% caveat above |
| `shoulder_lift` gravity, no load | 1.009° | good (sin R² 0.880) |
| `shoulder_lift` gravity, loaded | — | **not measured** (R² 0.010) |
| `elbow_flex` backlash, no load | 1.196° | average only — varies with angle |
| `elbow_flex` gravity, no load | 1.379° | good (sin R² 0.975) |
| `elbow_flex`, loaded | — | not attempted, unsafe from this pose |
