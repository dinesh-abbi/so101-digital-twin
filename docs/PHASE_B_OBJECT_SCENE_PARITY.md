# Phase B — Object/Scene Parity: What, Why, and Current State

**Status as of this writing: step 1 of 4, not yet complete.** Targets are
generated. Camera intrinsic calibration has not been successfully run yet.
No pose data has been produced. No direction-matching has happened, because
there is nothing to match yet — this document exists to make that concrete
rather than let "we're working on Phase B" hide how early it actually is.

---

## What Phase B is

Everything through M7 and the leader-arm teleop work solved one problem:
**does the simulated ARM match the real arm's joints.** It does — that's
the whole `real_sim_joint_mapping.py` + Task 1/2 characterisation effort,
validated on hardware.

Phase B is a *different* problem: **does the simulated SCENE match the real
scene** — where objects actually are on the table. Right now it does not,
because nothing in this codebase reads a real object's position. `spawn_cube.py`
(M2) places a cube in MuJoCo at coordinates you type in or generate randomly,
with zero connection to physical reality.

## Why it matters

`docs/ROADMAP_TELEOP_TO_DATASET.md` calls this "the single largest gap"
between the two things this project has already gotten right (arm parity)
and the two things it needs before recording trainable data (scene parity,
then the recording itself). Concretely: if you record a demonstration —
arm moves correctly, tracked joint-for-joint — but the sim's cube is
sitting somewhere the real cube never was, the resulting dataset teaches a
policy a relationship between "what it sees" and "what it should do" that
never actually held. That failure is much harder to catch after the fact
than a wrong joint angle, because there's no simple number (like an RMSE)
that says "this cube's sim position is wrong" the way `replay_in_sim.py`
can say a joint's position is wrong.

## The chosen approach: fiducial markers

Of the options the roadmap lists (fiducial markers, depth camera + shape
segmentation, manual entry), this project is using **ArUco markers** — small
printed/displayed patterns, each with a unique ID, that a camera can detect
and turn into a 3D position + orientation via `cv2.aruco` and `solvePnP`.
Chosen because it's cheap, deterministic, and easy to sanity-check (a
marker's reported position can be checked against a ruler by hand) — the
same "trust nothing without a look" discipline used throughout Task 1/2.

---

## The four steps, and where each one stands

### Step 1 — Camera intrinsic calibration
**Status: script written, not yet successfully run.**

Every camera's lens has its own focal length and distortion — not
something you can look up, has to be measured for this specific C920.
Without this, every pose computed later is confidently wrong in a way
nothing will flag.

- Script: `scripts/vision/calibrate_camera.py`
- Requires: a checkerboard (`scripts/vision/targets/checkerboard.png`,
  already generated) shown to the camera at 15-20 varied angles
- You hold the board (currently: displayed on a phone screen), press SPACE
  to capture each detected view, press C to solve
- Output: `scripts/vision/camera_intrinsics.json` (focal length, optical
  centre, lens distortion coefficients) — **does not exist yet**

### Step 2 — Marker detection → camera-frame pose
**Status: not started. No script exists yet.**

Once the camera's own distortion is known, a detected marker's pixel
corners can be converted into an actual 3D position and orientation
*relative to the camera* via `cv2.solvePnP`. This step's job is ONLY to
prove the camera can correctly report "this marker is X mm away, at this
orientation" — validated the same way M3/M4 validated joint ranges: check
a known distance with a ruler and confirm the number matches.

### Step 3 — Camera frame → table frame
**Status: not started. No script exists yet.**

A camera-frame pose is relative to wherever the camera happens to be
sitting, which is not a useful reference for the sim. `marker_00.png`
(printed, taped flat, NEVER moved) defines the table's origin. Every other
marker's camera-frame pose gets re-expressed relative to that origin
marker, the same way `real_sim_joint_mapping.py` re-expresses a servo's
raw ticks relative to its own calibrated range rather than some absolute
zero.

This is the step most tempting to skip, and the roadmap explicitly warns
against that (see `REAL_SIM_MAPPING_DEEP_DIVE.md`'s framing for why a
"looks right" shortcut here produces the same silent-wrongness problem the
joint mapping had to solve carefully).

### Step 4 — Table frame → sim
**Status: sim-side plumbing already exists and needs no camera to test.**

`CubeSpawner.spawn_cube(x, y, z)` in `scripts/digital_twin_env/
spawn_cube_test/spawn_cube.py` already writes directly into MuJoCo's free
joint `qpos` — this is most of the mechanism. What's missing is the
function that takes a table-frame `(x, y, z)` + orientation from step 3
and calls this correctly, plus a headless unit test using made-up
coordinates (no camera needed) before ever trusting a camera-derived
number — same bring-up discipline as M7's `--source sim` before
`--source real`.

---

## What YOU physically need to do

1. **Print the four ArUco markers** (`scripts/vision/targets/marker_00.png`
   through `marker_03.png`) at **100% / Actual size** — not "fit to page,"
   which silently rescales and makes every downstream distance wrong with
   no warning.
2. **Measure one printed marker's black square edge with a ruler.** Should
   read close to 40mm. The measured number is what the software uses, not
   the requested one.
3. **Tape `marker_00` flat on the table** at a spot you're willing to call
   permanent — this is the origin, and moving it later invalidates the
   step-3 calibration silently.
4. **Stick `marker_01/02/03` on objects** you want tracked, flat faces
   only (a marker on a curved surface reads badly).
5. **Mount the C920 so it sees the table surface**, and leave it there once
   step 3 (table calibration) is done — moving it afterward is the same
   silent-invalidation problem as moving the origin marker.
6. **For step 1 specifically (in progress now):** hold the checkerboard
   (currently: phone screen, fixed zoom, max brightness, auto-rotate off)
   in front of the camera, capture 15-20 varied/tilted views.

## What happens in the codebase, in order

1. `scripts/vision/make_targets.py` — already run, generates the printable
   PNGs. Done, no need to re-run unless marker size/count changes.
2. `scripts/vision/calibrate_camera.py` — **run this next.** Produces
   `camera_intrinsics.json`. This is the current blocking step.
3. *(to be written)* a marker-detection script using
   `cv2.aruco.ArucoDetector` + `cv2.solvePnP` + the intrinsics from step 2,
   validated against a hand-measured distance before trusting it further.
4. *(to be written)* the camera→table re-basing math, using `marker_00` as
   origin — structurally parallel to how `real_sim_joint_mapping.py`
   re-bases servo ticks against a calibration file rather than an absolute
   zero.
5. *(to be written)* the table→sim bridge function, wrapping
   `CubeSpawner.spawn_cube`, unit-tested with synthetic coordinates first
   (no camera needed), then validated live: place a real object at a known
   position, confirm the sim shows it there, log the numeric error the same
   way `replay_in_sim.py` logs joint tracking error.

## Direction/orientation matching — where that question actually applies

Not yet, and here's precisely why: direction-matching (does "left" in
reality show up as "left" in sim, does a marker's measured rotation match
its physical rotation) is a **step 2/3 concern** — it only becomes
checkable once a marker produces an actual pose. Right now, step 1 hasn't
produced camera intrinsics yet, so nothing has attempted to compute a pose
at all. This is the same order the joint mapping followed: get the
raw-to-normalized math correct and unit-tested (M6-prep) *before* the
first real direction/zero check with hardware connected (M6 itself). Phase
B is currently still in its "get the raw math right" phase, one level
earlier than direction-checking.

## What "done" looks like for Phase B

Per `docs/ROADMAP_TELEOP_TO_DATASET.md` §6: place a real object at a known
position; the sim shows it there within an agreed tolerance, confirmed
visually first, then backed by a numeric error logged the same way
`replay_in_sim.py` logs joint tracking error — not just "it looked right
once."
