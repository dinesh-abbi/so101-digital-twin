# SO-101 elbow_flex — characterisation and sim fit

Measured 2026-08-26 on the `twin_follower` arm (COM9), motor id 3.
~10,800 samples per load condition at ~94.5 Hz.

## Task 1 — what the real joint does

| Parameter | No load | Loaded | Change |
|---|---:|---:|---:|
| Command-to-motion latency | 65.1 ms | 64.8 ms | −0.3 ms (0%) |
| Steady-state error | 0.128° | 0.472° | **+269%** |
| Maximum velocity | 191.6 °/s | 189.7 °/s | −1% |
| Overshoot | 0.140° | 0.189° | +35% |
| Forward/reverse hysteresis | unmeasured | unmeasured | — |

**Latency is a fixed cost.** 65 ms regardless of load, sd 3.6–3.8 ms over 69
measurements each. That is the servo's internal control loop plus the serial
round-trip, not mechanics — it will not improve with a lighter payload.

**Load hits accuracy, not speed.** Velocity moved 1%; steady-state error
nearly quadrupled. The joint still arrives just as fast, it just stops
slightly short, because gravity opposes the final approach and the position
loop settles where torque balances load. This contradicts the illustrative
expectation in the Task 1 brief (latency 40→65 ms, velocity 90→72 °/s) and is
a genuine property of this actuator.

**Velocity scales with move size** rather than saturating: 52 °/s for a
5-unit step, 149 °/s for 20 units, ~190 °/s peak.

### Hysteresis is unmeasured, not absent

Every experiment steps **up** from 0 and returns **down** to 0. So 0 is only
ever approached from above and every other command only from below: no
command is settled from both directions anywhere in the dataset, and backlash
cannot be extracted from it.

What the data does bound is repeatability — 33 settled arrivals at 0°, sd
**0.088°**. Backlash cannot be larger than a gap that spread would have
revealed. To measure it properly a trajectory must approach the *same*
non-zero command from both sides and hold long enough to settle.

Two earlier analyses reported false hysteresis (11.3°, then 9.3°) by
comparing samples taken while the joint was still in flight. Both are fixed;
the guards are documented in `analyze_joint_response.py` and `make_plots.py`.

## Task 2 — closing the sim-to-real gap

Replaying the recorded command signal in MuJoCo at the recorded timestamps
(`replay_in_sim.py`) showed the model was accurate at rest — median |E|
0.115° — and wrong in flight. 94% of samples were within 2°; the 6% that were
not all fell 170–220 ms after a command.

On a 20-unit step the shipped model overshot to **23.8°** against a 19.6°
target and rang, peaking at **243 °/s** where the real joint manages 150 °/s.
Endpoints right, dynamics far too aggressive.

The cause was in the model all along: `so101.xml` carried
`kp=998.22 kv=2.731`, derived from a formula assuming a servo proportional
gain of 16 and, by its own comment, "not a 1-to-1 mapping" of the LeRobot
gains. It had never been checked against hardware.

### Fitted gains

`tune_actuator.py` sweeps kp/kv against the recorded step responses, fitting
on the three step experiments and validating on ramp, reverse and triangle:

|  | Fit set | Held out |
|---|---:|---:|
| kp=998.22 kv=2.731 (shipped) | 1.148° | 1.470° |
| **kp=400 kv=25 (fitted)** | **0.366°** | **0.561°** |
| improvement | 68% | 62% |

Held-out error improving alongside fit error is what separates a real fit
from overfitting. Peak overshoot on the 20-unit step drops 8.05° → 2.19°.

The RMSE surface is a flat valley along **kp/kv ≈ 16** — the critical-damping
ridge — so the *ratio* is what matters. Every pair on it scores within 0.01°.
kp=400 sits at the minimum.

Applied to `so101_assets/so101.xml` class `sts3215`; the previous file is at
`so101.xml.bak`. All 4 scenes still load and step.

### Whole-dataset result

| | Shipped gains | Fitted gains |
|---|---:|---:|
| no_load RMSE | 1.254° | **0.434°** (−65%) |
| loaded RMSE | 1.544° | **0.850°** (−45%) |
| worst \|E\| | 10.76° | **4.87°** |

### What is still unmodelled

Loaded runs carry a consistent **−0.6° bias**; no-load runs only −0.2°. The
sim does not reproduce gravity droop — the same effect Task 1 measured as
steady-state error rising 269% under load. That is the next gap, and it is
now isolated from the dynamics error that used to mask it.

Caveat: gains were fitted against `elbow_flex` only and applied to the whole
`sts3215` class, since every joint uses the same servo. The shoulder carries
far more inertia and may want its own fit. Re-run `tune_actuator.py` per
joint as each is characterised.

## Files

```
so101_joint_characterization.py   drives the real arm, logs telemetry
analyze_joint_response.py         the five Task 1 numbers
replay_in_sim.py                  replays real commands in MuJoCo, writes E(t)
tune_actuator.py                  fits kp/kv against the real step responses
make_plots.py                     all six figures

raw_data/     12 CSVs, 6 experiments x 2 load conditions
sim_data/     12 CSVs, per-sample real vs sim vs error
plots/        step_response, velocity_response, hysteresis,
              load_comparison, sim_vs_real, error_over_time
```

Reproduce the analysis without hardware:

```powershell
cd D:\robotics\so101-digital-twin\scripts\characterization
..\..\.venv\Scripts\python.exe analyze_joint_response.py
..\..\.venv\Scripts\python.exe replay_in_sim.py
..\..\.venv\Scripts\python.exe make_plots.py
```
