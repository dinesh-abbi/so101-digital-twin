# Leader-arm teleop runbook — calibration to leader + follower + sim

The end-to-end procedure for driving a real SO-101 follower from a real
SO-101 leader while a MuJoCo sim mirrors the follower, plus what to do when
the arms or the table change.

Written from a full session (2026-09-08) that hit almost every failure mode
worth knowing about. Every "why" below is something that actually went
wrong, not a hypothetical.

---

## 0. Vocabulary, because two things are easy to confuse

| term | meaning |
|---|---|
| **leader** | The arm you move by hand. Never powered to hold position. LeRobot calls it a `teleoperator`. |
| **follower** | The arm that copies the leader. Torque on. LeRobot calls it a `robot`. |
| **calibration id** | A *name* (e.g. `twin_leader_2`) for a saved per-arm calibration file. Not a serial number, not tied to hardware — you pick it. |
| **COM port** | Which USB serial adapter Windows assigned. **Changes between plug-ins.** Never assume. |

---

## 1. Find the ports — every single time

Windows renumbers COM ports whenever devices are replugged. During one
session the same follower appeared as COM13, COM14, and COM15.

```powershell
Get-PnpDevice -Class Ports -PresentOnly | Where-Object { $_.FriendlyName -match "CH34" } | Select-Object FriendlyName, InstanceId
```

Two arms → two `USB-Enhanced-SERIAL CH343` rows. The `InstanceId` tail
(e.g. `5AB0158158`) **is** stable per adapter, unlike the COM number — note
it once and you can identify that arm forever.

**To tell leader from follower with certainty:** unplug one, re-run the
command, see which row disappears. Guessing here means driving the wrong
arm.

### Confirm the bus before trusting anything

```powershell
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe D:\robotics\so101-digital-twin\scripts\scan_leader_bus.py --port COM14
```

Read-only, no writes. Expect `IDs found = [1, 2, 3, 4, 5, 6]`.

- **Empty result** → servo power is off. USB powers the adapter, *not* the
  servos.
- **Hangs with no progress** → stale serial handle. Unplug/replug the USB.
- **Missing IDs** → wiring, not software.

---

## 2. Calibrate both arms

Calibration records each joint's real min/max travel and where its zero is.
Everything downstream — the mapping, the safe envelope, the sim pose —
is derived from it.

```powershell
# leader
D:\robotics\so101-digital-twin\.venv\Scripts\lerobot-calibrate.exe --teleop.type=so101_leader --teleop.port=COM15 --teleop.id=twin_leader_2

# follower
D:\robotics\so101-digital-twin\.venv\Scripts\lerobot-calibrate.exe --robot.type=so101_follower --robot.port=COM10 --robot.id=twin_follower_3
```

**Type names are `so101_leader` / `so101_follower`**, not `so_leader` /
`so_follower`. The *directories* the files land in are named `so_leader/`
and `so_follower/`, which is a genuine trap — the CLI rejects the directory
names.

Type `c` + Enter to calibrate fresh; plain Enter reuses the existing file.

### Sweeping properly matters more than it looks

When prompted, move **each joint slowly to both hard stops**. Two rushed
sweeps in one session produced `elbow_flex` ranges where the recorded
position sat 13–24 ticks from the recorded max — meaning the joint never
actually left one end, and the "range" was fiction.

`wrist_roll` is excluded by the prompt: it has **no hard stop** (continuous
rotation, full 0–4095) and correctly calibrates to the whole encoder range.

### Sanity-check the result

```powershell
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe -c "import json; d=json.load(open(r'C:/Users/abbid/.cache/huggingface/lerobot/calibration/robots/so_follower/twin_follower_3.json')); [print('%-14s min=%5d max=%5d span=%5d homing=%6d' % (k,v['range_min'],v['range_max'],v['range_max']-v['range_min'],v['homing_offset'])) for k,v in d.items()]"
```

Two checks worth doing:

1. **Spans should be ~2000–2500 ticks** for bounded joints. Much less means
   an incomplete sweep.
2. **`(min+max)/2 + homing_offset` should land inside `min..max`.** When it
   doesn't, the joint's "zero" is a position it cannot physically reach —
   this was seen live and is exactly what makes `--recover` walk a joint
   the wrong way indefinitely.

Leader and follower spans should agree within a few percent. A 16% gripper
mismatch is tolerable (gripper motion just won't map 1:1); a 20% arm-joint
mismatch means one arm was swept badly.

---

## 3. Get the follower into a safe pose

The teleop scripts refuse to start if any joint sits outside ±50 normalized
units (gripper: 20–65). This is a real guard, not a formality — commanding
from outside the envelope clamps hard against a servo that may already be
against a mechanical stop.

```powershell
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe D:\robotics\so101-digital-twin\scripts\m6_keyboard_real.py --joints elbow_flex --port COM10 --id twin_follower_3 --recover
```

Torque stays on throughout, so the arm never sags mid-recovery.

**A gripper reading ~17 is the single most common trip-up** — that's just a
mostly-closed jaw, an entirely normal resting position, and it fails the
20–65 window on the low side. Nothing is wrong; run `--recover`.

**If recovery aborts with a travel-cap message**, a joint moved further than
its own full range without arriving. Do not retry — re-check calibration
first (§2's second sanity check). That abort exists because an earlier
version of this loop spun `wrist_roll` continuously until a cable pulled
out.

---

## 4. Run leader → follower → sim

```powershell
D:\robotics\so101-digital-twin\.venv\Scripts\python.exe D:\robotics\so101-digital-twin\scripts\m_lerobot_teleop_sim.py --leader-port COM15 --leader-id twin_leader_2 --follower-port COM10 --follower-id twin_follower_3
```

Press Enter at both calibration prompts. Useful flags:

| flag | effect |
|---|---|
| `--fps 60` | Smoother, doubles bus traffic. Watch for `no status packet` errors. |
| `--no-corner` | Sim robot stays centred instead of at the table corner. Cosmetic only. |
| `--no-sim` | Control only, no MuJoCo window. |
| `--max-relative-target` | Per-command clamp (default 4.0). |
| `--debug-track` | Print `real`/`ctrl`/`qpos` every 10 ticks. Use whenever the sim looks wrong — see "Reading the live output" below. |

**Lift the follower's gripper clear of the table before starting.** The
sim's base sits *on* the tabletop, so a folded-down real pose seeds the sim
gripper *inside* the table — measured at 38 contacts, 18 mm deep. MuJoCo
then spends every tick pushing the jaw out instead of tracking, and because
teleop moves targets gradually, the joint never gets a command large enough
to break free. Symptom: `ctrl` follows the real arm perfectly while `qpos`
sits ~150° away, looking exactly like a frozen sim. The script now warns at
startup when this happens.

No pose alignment is needed between the arms — the script captures whatever
offset exists at startup and tracks relative motion from there.

### Reading the live output

Run with `--debug-track` (off by default — one line per 10 ticks would
otherwise bury real warnings):

```
t  490 real  -98.4 ctrl  -95.3 qpos  -95.5 run=True
```

| column | meaning |
|---|---|
| `real` | follower's measured position (normalized units) |
| `ctrl` | what the sim is being *told* to do |
| `qpos` | where the sim arm actually *is* (degrees) |

- `real` moves, `ctrl` frozen → mapping problem
- `ctrl` moves, `qpos` frozen → **sim is stuck in collision** (see above)
- All three move, window static → rendering, not logic

---

## 5. The lag is real, and mostly correct

A small delay between real and sim is expected:

| source | delay |
|---|---|
| real servo command-to-motion (measured, `RESULTS.md`) | **65 ms** |
| one frame at 30 fps | 33 ms |
| sim actuator convergence (`kp=400`) | ~100–200 ms |

The sim mirrors the follower's **measured** position, so it inherits the
real 65 ms and adds its own settling. **A zero-lag sim would be wrong** —
it would show motion the hardware cannot actually produce.

`--fps 60` halves the frame contribution. The rest is physics, and matching
it is the point of the twin.

Small steady-state angle differences are also expected: the mapping is
*fraction-of-range*, so when a joint's calibrated span differs from the
model's kinematic span, the same fraction is a different angle. See
`REAL_SIM_MAPPING_DEEP_DIVE.md`.

---

## 6. Swapping to different arms or a different table

**Different arms** — the only thing that must change is calibration:

1. Find the new ports (§1) and note the `InstanceId` tails.
2. Calibrate both arms under **new ids** (`--teleop.id` / `--robot.id`).
   Never reuse another arm's id: two physically identical SO-101s measured
   198.1° and 175.2° on `shoulder_pan`, a real 23° difference from how the
   servo horn was splined at assembly.
3. Pass the new ports and ids on the command line.

**Never edit `so101_assets/so101.xml` to match a calibration.** The model is
shared by every arm; per-arm differences belong in the calibration file,
which is exactly what the mapping layer rescales against.

**Different table** — the sim's table is a modelled IKEA LINNMON/ADILS
(100×60 cm). If the real table differs, the arm still tracks correctly
(joint angles are in the arm's own frame), but the *scene* no longer matches
reality. That only matters for object/scene parity work — see
`ROADMAP_TELEOP_TO_DATASET.md` Phase B. For teleop alone, a different table
changes nothing except how much the picture resembles the room.

---

## 7. Quick failure reference

| symptom | cause |
|---|---|
| `Could not connect on port 'COM9'` | Wrong port, or defaults used instead of `--port` |
| `invalid choice: 'so_leader'` | Use `so101_leader` / `so101_follower` |
| `Missing motor IDs` | Servo power off |
| Scan hangs | Stale serial handle — replug USB |
| `no status packet` mid-run | Transient bus dropout. Re-scan; if it repeats 3×, check wiring/power |
| `gripper start outside envelope` | Normal resting jaw. Run `--recover` |
| `--recover` aborts on travel cap | Calibration mismatch — re-check §2 |
| Sim frozen, `ctrl` tracks but `qpos` doesn't | Sim gripper inside the table. Lift the arm, restart |
| Sim pose differs from real | Corner placement (`--no-corner` to compare) or span mismatch (§5) |
| Console floods with clamp warnings | Leader outran the per-command clamp. Suppressed by default now |

---

## Related

- `README.md` → "Working with real hardware" — M6/M7 keyboard control
- `docs/TELEOP_SESSION_LOG_AND_PLAN.md` — the debugging log behind this
  runbook: what broke, what was tried, what was reverted and why. Read it
  before re-attempting the sim table height or self-collision
- `docs/PROJECT_STATUS.md` — phase status
- `docs/REAL_SIM_MAPPING_DEEP_DIVE.md` — the mapping math
- `docs/ROADMAP_TELEOP_TO_DATASET.md` — what comes after teleop
- `scripts/characterization/RESULTS.md` — the 65 ms latency measurement
