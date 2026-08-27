# Backlash and gravity droop — both joints

Measured 2026-08-26 with `backlash_probe.py`: a staircase that walks up
through 9 rungs and back down through the same rungs, so 7 intermediate rungs
are each settled from **both** directions. 3 repeats per joint, ~93 Hz.

This is the measurement the Task 1 trajectories could not make. Those always
stepped up from 0 and returned down to 0, so command 0 was the only one ever
approached from above — meaning "approach direction" and "joint angle" were
perfectly confounded, and backlash could not be separated from gravity droop.

## Results

| joint | backlash | (ticks) | gravity droop | gravity fit R² |
|---|---:|---:|---:|---:|
| `shoulder_lift` | **1.955°** | 22.2 | **1.009°** | 0.880 |
| `elbow_flex` | **1.196°** | 13.6 | **1.379°** | 0.975 |

Both joints carry roughly 1–2° of mechanical slop and 1–1.4° of
gravity-dependent droop. Neither is negligible against the Task 2 sim-to-real
RMSE of 0.434°.

### How each number is read

Backlash reverses sign with approach direction, so it survives the
**difference** between directions and cancels in the mean. Gravity droop
pushes both directions the same way, so it survives the **mean** and cancels
in the difference. Each effect is invisible to the other statistic — verified
on synthetic data with known answers (pure 1.2° backlash → reports 1.199 /
0.007; pure 1.2° gravity → reports 0.002 / 1.193).

### The trig basis is not a free choice

Gravity torque goes as the sine of the angle **from vertical**. Assuming
cosine implicitly assumes the joint's zero is horizontal, which is false for
this arm — it sits folded at home, so zero is near vertical:

| joint | cos fit R² | sin fit R² |
|---|---:|---:|
| `shoulder_lift` | 0.056 | **0.880** |
| `elbow_flex` | 0.002 | **0.975** |

The cos fit still reported a confident 1.164° amplitude for the shoulder
while explaining 5% of the variance. `analyze_backlash.py` now fits both,
uses the better, prints both R², and warns when neither exceeds 0.5.

## `elbow_flex` backlash is not constant — unresolved

The shoulder's gap is near-constant across rungs (sd 0.372 on a 1.955 mean),
which is what pure gear slop looks like. The elbow's is not:

| command | from below | from above | gap |
|---:|---:|---:|---:|
| −36.91° | −38.846 | −36.855 | −1.990 |
| −24.61° | −26.191 | −24.580 | −1.611 |
| −12.30° | −13.529 | −12.129 | −1.400 |
| +0.00° | −0.586 | −0.088 | **−0.498** |
| +12.30° | +12.217 | +12.744 | **−0.527** |
| +24.61° | +24.609 | +25.635 | −1.025 |
| +36.91° | +36.826 | +38.144 | −1.318 |

The gap is **smallest near 0° and grows toward both extremes**, roughly
symmetric in |angle| (fit against |angle| gives R² 0.569 vs 0.000 for a
constant). Constant backlash does not do this.

Checked and ruled out:

- **Not travel-limit saturation.** 558 ticks of headroom remain at both ends,
  and settled ticks are evenly spaced across all 9 rungs (1427, 1556, 1698,
  1841, 1983, 2129, 2273, 2413, 2547).

Still open. Candidates worth testing: position-dependent friction in the
geartrain, or a real backlash that only fully opens once enough torque
accumulates away from the neutral pose. Until it is understood, **1.196° is
an average, not a constant** — a deadband model using it will be right near
the middle of travel and wrong at the extremes.

The shoulder's 1.955° is the better-behaved number and the safer one to model
first.

## Note on the load telemetry

`load_raw` values above ~1000 in these CSVs are the sign-magnitude direction
bit (0x400 = 1024), not torque magnitudes. The characterisation script applies
`signed()` to speed and load, but the raw values logged here still need that
reading. This does not affect any position measurement — every number above
comes from `actual_deg` / `raw_ticks`.

## The model — `servo_model.py`

Both effects are implemented as a drive layer, not in `so101.xml`. MuJoCo has
no gear-backlash primitive for a hinge, and both are properties of *this
arm's servos* rather than of the kinematic model, which is shared with the
other arm.

Backlash is hysteresis with memory: the joint rests half the slop off the
commanded position, on the side it travelled *away* from, and a repeated
command produces no motion. Gravity is `offset + amplitude * sin(angle)`.

Validated against the recorded staircases — data the coefficients were fitted
to only as summary statistics, not sample-by-sample:

| joint | no model | backlash only | gravity only | **both** |
|---|---:|---:|---:|---:|
| `shoulder_lift` | 1.337° | 0.999° | 1.174° | **0.628° (−53%)** |
| `elbow_flex` | 0.802° | 0.630° | 0.749° | **0.409° (−49%)** |

Both effects matter; neither alone gets close.

### A sign error worth recording

The first implementation had backlash inverted — it assumed travelling up
leaves the joint sitting high. The measured gap says the opposite
(`from_below − from_above = −1.955`): the joint **lags behind** the direction
of travel, as slop does, since the driven flank trails the driving one.

Inverted, the model made predictions **53% worse than no model at all**
(1.337° → 2.040°) — while every internal consistency check still passed,
because those only tested the model against its own arithmetic. The self-test
now validates against the recordings and asserts the error actually goes
down.

### Where it must not be used

**Do not stack this on the Task 2 kp/kv fit.** Those gains were fitted against
step runs visiting only two commands (0 and +19.6°), so the backlash at those
points is already inside the gains: measured backlash 1.196°, but the Task 2
residual direction gap is only 0.118° — about 90% already absorbed. Applying
the model there double-counts and makes settled error **245% worse**.

It also describes where the joint comes to **rest**, so applying it
mid-transient is invalid (9% worse during the move).

Use it for new command sequences, for compensating commands so the real arm
lands where intended, and for trajectories the gain fit never saw.

### Bonus: the staircases are held-out validation

`replay_in_sim.py` now picks up the staircase runs, which the kp/kv fit never
saw and which sweep ±49° against the original ±20°:

    staircase 1/2/3   RMSE 0.820 / 0.828 / 0.840 deg
    step_20deg        RMSE 0.396 deg
    triangle          RMSE 0.536 deg

Consistent across all three repeats. The gains generalise to unseen
trajectories, roughly 2x looser over the wider sweep. The headline no_load
figure therefore moved from 0.434 to 0.638 deg — not a regression, just a
harder and more honest test set.

## What to model, in order

1. **`shoulder_lift` backlash, 1.955°** — largest effect, cleanest measurement.
2. **Gravity droop, ~1.0–1.4° both joints** — feed-forward proportional to
   `sin(angle)`, coefficients now measured rather than assumed.
3. **`elbow_flex` backlash** — only after the angle dependence is understood.

## Reproduce

```powershell
cd D:\robotics\so101-digital-twin\scripts\characterization
..\..\.venv\Scripts\python.exe backlash_probe.py   --joint shoulder_lift
..\..\.venv\Scripts\python.exe analyze_backlash.py --joint shoulder_lift
```

Note the `..\..\.venv\Scripts\python.exe` prefix — a bare `python` picks up a
system interpreter without numpy.
