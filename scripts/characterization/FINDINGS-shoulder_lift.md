# shoulder_lift — why the elbow's gains do not transfer

Recorded 2026-08-26, `twin_follower` motor id 2, no-load (arm folded in the
home pose), ~94.5 Hz, 6 experiments.

## The observation

`shoulder_lift` settles ~10x worse than `elbow_flex`:

| | elbow_flex | shoulder_lift |
|---|---:|---:|
| settled error (steps) | 0.06–0.26° | 0.65–1.5° |
| holding current (raw) | 2.1 | 5.1 |
| error variation across range | 0.12° | **1.19°** |

The arm was undamaged throughout: 36 °C steady, 12.1 V, no stalls, no
current spikes. The motion looks worse because it *is* worse, not because
anything is wrong with the hardware.

## What it is not

Three hypotheses were tested and rejected. Recording them because each looks
plausible and each would have produced a wrong "fix".

**Not a gain problem.** Sweeping kp/kv "improved" fit-set RMSE 18%
(0.972 → 0.793) at kp=60 kv=6 — but held-out RMSE got *worse* (1.352 →
1.367). Decomposing the error shows why: at kp=400 the settled error runs
+1.29° low / +0.09° high (spread 1.205°); at kp=60 it runs +0.77° / −0.42°
(spread 1.194°). **The spread is unchanged.** Soft gains only slide the whole
curve down to straddle zero, hiding a constant in the RMSE without modelling
anything. Applying kp=60 would have been fitting a control gain to
compensate a physical effect, and would then be wrong at every other pose.

**Not a stale calibration.** The recalibration earlier the same day widened
this joint's span 6.8%, which would produce exactly this kind of scale error
— but the check recovers 1.13291 deg/unit from the CSVs against 1.13291
now. The data was recorded *after* the recalibration and matches it exactly.
`check_calibration_drift()` in `tune_actuator.py` now tests this
automatically on every fit.

**Not a mass error.** Scaling the mass of every link distal to the shoulder
from 1x to 3.5x moves the spread by 0.03° (1.205 → 1.228) and makes RMSE
worse throughout. Beyond ~4x the sim destabilises. Gravity magnitude is not
the free parameter here.

## What it is

The real servo settles where its motor torque balances the gravity load, and
that balance point moves with joint angle. Measured in raw encoder ticks,
where no unit conversion can hide it:

| command | goal ticks | settled ticks | offset |
|---|---:|---:|---:|
| elbow_flex +0.00° | 1987.0 | 1973.0 | +14.0 |
| elbow_flex +19.58° | 2209.8 | 2194.4 | +15.4 |
| shoulder_lift +0.00° | 2031.0 | 2046.6 | −15.6 |
| shoulder_lift +22.66° | 2288.8 | 2290.9 | −2.1 |

The elbow's offset is **constant** (14.0 → 15.4 ticks) and therefore
harmless — a constant offset cancels. The shoulder's **varies by 13.5
ticks** (−15.6 → −2.1) across the same size of move. That variation is the
entire finding.

## Why the sim cannot currently reproduce it

The model does have gravity, and its torque on this joint does vary with
angle — 0.569 N·m at 0° rising to 0.682 N·m at 20°, a 20% change. The
problem is the *response* to that change. Commanding a static pose and
letting the sim settle:

| gains | droop at 0° | droop at 25° | variation |
|---|---:|---:|---:|
| kp=998.22 kv=2.731 | −0.033° | −0.040° | 0.007° |
| kp=400 kv=25 (current) | −0.082° | −0.100° | **0.018°** |
| kp=60 kv=6 | −0.547° | −0.668° | 0.121° |

The real joint varies **1.19°** over that range — 66x more than the current
model, and still 10x more than the softest gains tested. A MuJoCo `position`
actuator is a pure PD law: its steady-state droop is exactly
`torque / kp`, so matching 1.19° of variation would need a kp roughly 66x
lower, which would destroy the transient response that kp=400 gets right.

**No single kp can satisfy both.** That is the real conclusion: the STS3215's
own position loop is not a PD law. It has internal deadband and no integral
term, so it is far more compliant under sustained load than under transient
error. One linear gain cannot express both behaviours.

## Correction: it may be backlash, not gravity (2026-08-26, later)

The reading above — that this is gravity-dependent compliance — **is not
established by this data**, and the attempt to build a gravity feed-forward
on it uncovered why.

Splitting the settled error by approach direction instead of by angle:

| joint | arrived from below | arrived from above | gap |
|---|---:|---:|---:|
| shoulder_lift | −0.037° | +1.300° | **1.337°** |
| elbow_flex | −0.163° | +0.003° | 0.166° |

That looks like decisive backlash. But it is confounded: **command 0 is the
only one ever approached from above**, and every other command only from
below. "Arrived from above" and "sitting at the lowest, most gravity-loaded
pose" are the *same set of samples*. Both explanations fit identically:

- **backlash** — 0° lands on the far side of the gear slop
- **gravity** — 0° is where the load torque is largest

The earlier "angle-dependent droop" reading was an artifact of the same
confound: low commands are reached from above, high commands from below, so
the apparent correlation with angle (r = −0.61, and non-monotonic) was really
a correlation with approach direction.

**The Task 1 trajectories cannot separate these.** No amount of reanalysis
will fix that; it needs a different trajectory.

`backlash_probe.py` runs the staircase that resolves it — up through every
rung, then back down through the same rungs, so 7 rungs spanning ±42° are
each settled from both directions. `analyze_backlash.py` then reads backlash
from the direction gap and gravity from the mean level, because each effect
cancels out of the other statistic (verified on synthetic data with known
answers). Until that runs, treat the mechanism as **unknown**.

## RESOLVED: both, and backlash is the larger (2026-08-26)

The staircase ran — 3 repeats, 7 rungs spanning ±42.5°, each settled from both
directions, ~93 Hz. It separates cleanly:

| effect | size | read from | fit quality |
|---|---:|---|---|
| **backlash** | **1.955°** | direction gap | consistent across rungs |
| **gravity droop** | **1.009°** | mean vs angle | R² 0.880 |

Per-rung, the direction gap:

| command | from below | from above | gap |
|---:|---:|---:|---:|
| −42.48° | −43.224 | −41.426 | −1.798 |
| −28.32° | −28.828 | −27.304 | −1.524 |
| −14.16° | −14.355 | −12.903 | −1.452 |
| +0.00° | −0.147 | +2.432 | −2.578 |
| +14.16° | +14.385 | +16.582 | −2.197 |
| +28.32° | +28.711 | +30.645 | −1.934 |
| +42.48° | +42.686 | +44.886 | −2.201 |

Mean 1.955°, or **22.2 encoder ticks** — physically sensible for a plastic
geartrain, and consistent with the 13.5-tick offset variation measured
independently from the Task 1 step data. Excluding the 0° rung (the only one
the arm also traverses during `move_gently`, and the visible outlier at
−2.578) the gap is 1.851° ± 0.294 — the near-constant profile backlash should
have.

### The trig basis matters, and cos was wrong

Gravity torque goes as the sine of the angle **from vertical**. The first
analyzer assumed cosine, which implicitly assumes the joint's zero is
horizontal. It is not — this arm sits folded at home, so zero is near
vertical:

    err = -0.160 + 1.164 * cos(angle)    R^2 0.056   <- wrong basis
    err = +0.867 + 1.009 * sin(angle)    R^2 0.880   <- correct

The cos fit still reported a confident 1.164° amplitude while explaining
almost none of the variance. `analyze_backlash.py` now fits both, uses the
better, prints both R² values, and warns when neither exceeds 0.5 — so a bad
fit cannot pass as a measurement again.

### What this means for the model

Backlash is the larger effect and **is not what the earlier sections
guessed**. The mechanism was genuinely unknown until this run; the "gravity
compliance" reading was a confound, and so was the "1.337° directional gap"
that replaced it.

`elbow_flex` has not been measured this way. Its 0.166° gap from the Task 1
runs carries the same confound and is a **lower bound only** — it needs its
own staircase before any claim about it is made.

## What to do about it

Do **not** change the `sts3215` gains. `kp=400 kv=25` is fitted against real
transient data on `elbow_flex` and validated on held-out runs; it remains the
best available transient model, and this joint's steady-state problem is not
something gains can fix.

The honest options, in order of increasing effort:

1. **Accept it and document the bound.** Sim steady-state error on loaded
   joints is optimistic by up to ~1.2°. Fine for reachability and collision
   work; not fine for precision placement.
2. **Add a gravity feed-forward term** to whatever drives the sim, so the
   commanded position is offset by the expected droop. Cheap, and it makes
   sim and real agree without corrupting the actuator model.
3. **Replace the `position` actuator with a custom one** that reproduces
   deadband plus load-dependent compliance. Most faithful, most work, and
   needs load-condition data on this joint first.

Option 2 is the recommended next step, and it needs a **loaded** run on this
joint to calibrate against — the arm was folded into its home pose for this
one, which is the lightest configuration `shoulder_lift` ever sees.

## Caveat on this dataset

The arm was in the home pose (folded, mass close to the joint axis). This is
genuinely "no load" and a valid baseline, but it means the measured 1.19°
variation is the **minimum** this joint exhibits. Extended horizontally it
will be substantially larger.
