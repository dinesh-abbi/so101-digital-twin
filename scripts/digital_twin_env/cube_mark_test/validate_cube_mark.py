#!/usr/bin/env python3
"""Headless checks for cube_mark.py -- no hardware.

1. The cube compiles into the bare scene without disturbing the arm's joints.
2. It rests on the floor and never collides with the arm.
3. GripDetector against every real-follower recording on this bench.
4. End to end on recordings/teleop_ref_pick_place.csv: mark the cube with
   the jaws around it, replay from the start, and check it stays put, is
   grabbed, carried, released and lands where the real jaws opened.
5. Persistence and the recording sidecar.

    python validate_cube_mark.py
"""

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parents[2]
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import numpy as np                               # noqa: E402

from cube_mark import (                          # noqa: E402
    CUBE_HALF, MarkedCube, GripDetector, add_cube)
from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES, real_to_sim_vector, sim_joint_ranges_from_model,
    widen_shoulder_lift)

BARE_SCENE = _HERE.parent / "robot_bare_test" / "robot_bare_scene.xml"
RECORDINGS = _ROOT / "recordings"
REF = RECORDINGS / "teleop_ref_pick_place.csv"
# Jaw midpoint where the reference recording opened over the box
# (validate_pick_place.py, RELEASE_TIP_MID).
RELEASE_XY = np.array([0.364, 0.033])

# Real cube grabs per recording, read off the gripper traces 2026-09-16.
EXPECTED_GRABS = {
    "teleop_log_60fps.csv": 1, "teleop_log_pick2.csv": 1,
    "teleop_log_pick3.csv": 1, "teleop_log_pick4.csv": 1,
    "teleop_log_v10.csv": 1, "teleop_log_v9.csv": 1,
    "teleop_ref_pick_place.csv": 1, "teleop_touch_check.csv": 1,
    "teleop_touch_check_1.csv": 1,
    "teleop_after_wrist_fix.csv": 0, "teleop_log_pick_place_cube.csv": 0,
    "teleop_pick_v1.csv": 0, "teleop_touch8.csv": 0,
}

failures = 0


def check(ok, label, detail=""):
    global failures
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" -- {detail}" if detail else ""))
    if not ok:
        failures += 1


def build(with_cube=True):
    spec = mujoco.MjSpec.from_file(str(BARE_SCENE))
    widen_shoulder_lift(spec)
    if with_cube:
        add_cube(spec)
    model = spec.compile()
    return model, mujoco.MjData(model)


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def cube_contact_partners(model, data):
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    partners = set()
    for i in range(data.ncon):
        g1, g2 = data.contact[i].geom1, data.contact[i].geom2
        if cube_geom in (g1, g2):
            other = g2 if g1 == cube_geom else g1
            partners.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
                         or f"geom{other}")
    return partners


print("=" * 70)
print("  1. compile")
print("=" * 70)
model, data = build()
arm_adr = [int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)])
           for j in JOINT_NAMES]
check(arm_adr == list(range(6)), "arm joints still occupy qpos[0:6]", str(arm_adr))
check(model.nu == 6, "still exactly 6 actuators", str(model.nu))
cube_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
check(cube_jid != -1 and model.jnt_type[cube_jid] == mujoco.mjtJoint.mjJNT_FREE,
      "cube has a free joint")
sim_ranges = sim_joint_ranges_from_model(model)

print("\n" + "=" * 70)
print("  2. collision: floor yes, arm never")
print("=" * 70)
body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
mujoco.mj_forward(model, data)
start = data.xpos[body].copy()
while data.time < 2.0:
    mujoco.mj_step(model, data)
end = data.xpos[body]
check(abs(end[2] - CUBE_HALF) < 0.001, "cube rests on the floor",
      f"z={end[2]:.4f} (expect {CUBE_HALF:.4f})")
check(np.linalg.norm(end[:2] - start[:2]) < 0.001, "and does not drift",
      f"{np.linalg.norm(end[:2] - start[:2]) * 1000:.2f} mm")
check(cube_contact_partners(model, data) == {"floor"}, "its only contact is the floor",
      str(cube_contact_partners(model, data)))

# Jam the cube into the gripper: every tip sphere and the gripper box overlap it.
rows = load_rows(REF)
t0 = float(rows[0]["wall_time"])
grab_row = min(rows, key=lambda r: abs(float(r["wall_time"]) - t0 - 30.1))
pose = {j: float(grab_row[f"{j}_real_norm"]) for j in JOINT_NAMES}
data.qpos[:6] = real_to_sim_vector(pose, sim_ranges)
mujoco.mj_forward(model, data)
tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_jaw_sph_tip{i}")
        for s in ("fixed", "moving") for i in (1, 2, 3)]
a = int(model.jnt_qposadr[cube_jid])
data.qpos[a:a + 3] = data.geom_xpos[tips].mean(axis=0)
data.qpos[a + 3:a + 7] = (1, 0, 0, 0)
mujoco.mj_forward(model, data)
overlapping = sum(np.all(np.abs(data.geom_xpos[g] - data.xpos[body]) < CUBE_HALF + 0.001)
                  for g in tips)
partners = cube_contact_partners(model, data)
check(overlapping >= 1 and not (partners - {"floor"}),
      "cube pushed INTO the gripper makes no arm contact",
      f"{overlapping} tip spheres inside it, contacts={partners or 'none'}")

print("\n" + "=" * 70)
print("  3. grab detector vs every real-follower recording")
print("=" * 70)
for name, expected in EXPECTED_GRABS.items():
    path = RECORDINGS / name
    if not path.exists():
        print(f"  [skip] {name} not on disk")
        continue
    det = GripDetector()
    grabs, releases, jaws, t_grab, t_release = 0, 0, [], None, None
    rr = load_rows(path)
    rt0 = float(rr[0]["wall_time"])
    for r in rr:
        t = float(r["wall_time"]) - rt0
        ev = det.step(float(r["gripper_real_norm"]), float(r["gripper_leader_cmd"]), t)
        if ev == "grab":
            grabs += 1
            jaws.append(det.jaw_at_grab)
            t_grab = t_grab if t_grab is not None else t
        elif ev == "release":
            releases += 1
            t_release = t_release if t_release is not None else t
    detail = f"{grabs} grab(s)"
    if jaws:
        detail += f", jaw at grab {min(jaws):.1f}-{max(jaws):.1f}"
    ok = grabs == expected and releases == grabs and all(10.0 <= j <= 12.5 for j in jaws)
    check(ok, f"{name:32s} expect {expected}", detail)
    if name == REF.name:
        check(t_grab is not None and 29.9 <= t_grab <= 30.4,
              "  reference grab lands on the real close", f"t={t_grab}")
        check(t_release is not None and 33.0 <= t_release <= 33.6,
              "  reference release lands on the real open", f"t={t_release}")

print("\n" + "=" * 70)
print("  4. end to end: mark, go home, pick (teleop_ref_pick_place.csv)")
print("=" * 70)


def replay(with_cube, mark_file=None, mark_at_s=29.46):
    model, data = build(with_cube)
    sim_ranges = sim_joint_ranges_from_model(model)
    frames = [(float(r["wall_time"]) - t0,
               {j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES},
               float(r["gripper_leader_cmd"])) for r in rows]
    out = {"model": model, "data": data, "events": [], "arm_q": []}
    cube = None
    if with_cube:
        cube = MarkedCube(model, data, mark_file)
        refuse_q = real_to_sim_vector(frames[0][1], sim_ranges)
        out["refused"] = cube.mark_here(refuse_q)
        high_frame = min(frames, key=lambda f: abs(f[0] - 33.5))
        out["refused_high"] = cube.mark_here(real_to_sim_vector(high_frame[1], sim_ranges))
        out["file_after_refusal"] = Path(mark_file).exists()
        mark_frame = min(frames, key=lambda f: abs(f[0] - mark_at_s))
        out["mark_msg"] = cube.mark_here(real_to_sim_vector(mark_frame[1], sim_ranges))
        out["marked_xy"] = data.xpos[cube._body][:2].copy()
        out["cube"] = cube

    q0 = real_to_sim_vector(frames[0][1], sim_ranges)
    data.qpos[:6] = q0
    data.ctrl[:6] = q0
    mujoco.mj_forward(model, data)

    pre_grab_drift = 0.0
    hold_offsets, peak_z = [], 0.0
    for t, pose, cmd in frames:
        data.ctrl[:6] = real_to_sim_vector(pose, sim_ranges)
        while data.time < t:
            mujoco.mj_step(model, data)
        out["arm_q"].append(data.qpos[:6].copy())
        if cube is None:
            continue
        msg = cube.update(pose["gripper"], cmd, data.ctrl[:6], t)
        if msg:
            out["events"].append((t, msg))
        x, y, z, held = cube.pose()
        if not any("GRABBED" in m for _, m in out["events"]):
            pre_grab_drift = max(pre_grab_drift,
                                 float(np.linalg.norm(np.array([x, y]) - out["marked_xy"])))
        if held:
            tipmid = data.geom_xpos[cube._fixed_tips + cube._moving_tips].mean(axis=0)
            if cube._blend >= 1.0:
                hold_offsets.append(math.hypot(x - tipmid[0], y - tipmid[1]))
            peak_z = max(peak_z, z)
    # let it land
    settle_until = data.time + 2.0
    while data.time < settle_until:
        mujoco.mj_step(model, data)
    out.update(pre_grab_drift=pre_grab_drift, hold_offsets=hold_offsets, peak_z=peak_z)
    return out


with tempfile.TemporaryDirectory() as tmp:
    mark_file = Path(tmp) / "cube_marked.json"
    res = replay(True, mark_file)
    cube = res["cube"]

    check("NOT marked" in res["refused"] and "apart" in res["refused"],
          "mark refused with the arm folded at rest (jaws closed)",
          res["refused"].strip())
    check("NOT marked" in res["refused_high"] and "above the floor" in res["refused_high"],
          "mark refused with open jaws up in the air",
          res["refused_high"].strip())
    check(not res["file_after_refusal"], "refusals save nothing")
    check("MARKED" in res["mark_msg"], "mark accepted with the open jaws around the cube",
          res["mark_msg"].strip())
    saved = json.loads(mark_file.read_text()) if mark_file.exists() else {}
    check(bool(saved) and saved["tips_above_floor_mm"] < 50, "mark saved to disk", str(saved))
    check(res["pre_grab_drift"] < 0.002,
          "cube does not move while the arm reaches in (the arm cannot bump it)",
          f"max drift {res['pre_grab_drift'] * 1000:.2f} mm")

    grabbed = [(t, m) for t, m in res["events"] if "GRABBED" in m]
    released = [(t, m) for t, m in res["events"] if "RELEASED" in m]
    for t, m in res["events"]:
        print(f"         t={t:6.2f}s {m.strip()}")
    grab_mm = None
    if grabbed:
        grab_mm = float(grabbed[0][1].split("were ")[1].split(" mm")[0])
    check(len(grabbed) == 1 and grab_mm is not None and grab_mm <= 15,
          "grabbed once, sim jaws within 15 mm of the marked cube", f"{grab_mm} mm")
    check(res["hold_offsets"] and max(res["hold_offsets"]) < 0.005,
          "carried centred in the jaws",
          f"worst {max(res['hold_offsets'] or [0]) * 1000:.2f} mm sideways")
    check(res["peak_z"] > 0.05, "carried up off the floor",
          f"peak cube height {res['peak_z'] * 100:.1f} cm")
    check(len(released) == 1, "released once when the real jaws opened")

    d, body = res["data"], cube._body
    final = d.xpos[body]
    check(abs(final[2] - CUBE_HALF) < 0.002, "landed resting on the floor",
          f"z={final[2]:.4f}")
    miss = float(np.linalg.norm(final[:2] - RELEASE_XY))
    check(miss < 0.04, "landed under where the real jaws opened",
          f"{miss * 100:.1f} cm from {RELEASE_XY.tolist()}")

    base = replay(False)
    diff = max(float(np.max(np.abs(a - b))) for a, b in zip(res["arm_q"], base["arm_q"]))
    check(diff < 1e-6, "the cube does not change the mirrored arm at all",
          f"max joint difference {diff:.2e} rad")

    print("\n" + "=" * 70)
    print("  5. persistence and recording sidecar")
    print("=" * 70)
    model2, data2 = build()
    cube2 = MarkedCube(model2, data2, mark_file)
    xy2 = data2.xpos[cube2._body][:2]
    check(cube2.mark == saved and np.allclose(xy2, [saved["x"], saved["y"]], atol=1e-6),
          "a fresh session puts the cube back at the saved mark",
          f"x={xy2[0]:+.4f} y={xy2[1]:+.4f}")
    csv_path = Path(tmp) / "episode.csv"
    cube2.write_recording_note(csv_path)
    note = json.loads(Path(str(csv_path) + ".cube.json").read_text())
    check(note.get("mark_at_start") == saved, "recording sidecar holds the mark")
    fresh = MarkedCube(*build(), Path(tmp) / "missing.json")
    check(fresh.mark is None and "no cube marked" in fresh.startup_message(),
          "no saved mark: cube stays parked and says so")

print("\n" + "=" * 70)
print("  6. replay: cube stays in the SHOWN jaws even when the arm lags")
print("=" * 70)
# A replayed real arm trails its recording. The held flags a --cube
# recording logs are exactly what GripDetector produces live, so derive them
# from the reference recording, then replay with the shown arm 0 s and 0.4 s
# behind the recording.
det = GripDetector()
held_flags = []
for r in rows:
    det.step(float(r["gripper_real_norm"]), float(r["gripper_leader_cmd"]),
             float(r["wall_time"]) - t0)
    held_flags.append(det.holding)
times = np.array([float(r["wall_time"]) - t0 for r in rows])
poses = [{j: float(r[f"{j}_real_norm"]) for j in JOINT_NAMES} for r in rows]

# (lag, extra): the shown arm trails the recording by `lag` seconds, and its
# jaw stops `extra` units wider than recorded while holding -- 1.8 is past
# the old +/-1.2 match that only grabbed some of the time.
for lag, extra in ((0.0, 0.0), (0.4, 0.0), (0.4, 1.8)):
    model, data = build()
    ranges = sim_joint_ranges_from_model(model)
    cube = MarkedCube(model, data, mark_file=None)
    cube.place_mark(saved)
    q0 = real_to_sim_vector(poses[0], ranges)
    data.qpos[:6] = q0
    data.ctrl[:6] = q0
    mujoco.mj_forward(model, data)
    events, offsets, heights, peak_z, release_tips = [], [], [], 0.0, None
    grab_jump, cube_steps, tip_steps, prev = None, [], [], None
    for i, t in enumerate(times):
        shown = poses[max(0, int(np.searchsorted(times, t - lag)))]
        data.ctrl[:6] = real_to_sim_vector(shown, ranges)
        while data.time < t:
            mujoco.mj_step(model, data)
        shown_jaw = shown["gripper"] + (extra if held_flags[i] else 0.0)
        before = data.xpos[cube._body].copy()
        msg = cube.follow_recording(held_flags[i], shown_jaw, poses[i]["gripper"])
        tips = data.geom_xpos[cube._fixed_tips + cube._moving_tips].mean(axis=0)
        if msg:
            events.append(msg)
            if "GRABBED" in msg:
                grab_jump = float(np.linalg.norm(data.xpos[cube._body] - before))
            if "RELEASED" in msg:
                release_tips = tips.copy()
        x, y, z, held = cube.pose()
        if held:
            if prev is not None:
                cube_steps.append(math.dist((x, y, z), prev[0]))
                tip_steps.append(float(np.linalg.norm(tips - prev[1])))
            prev = ((x, y, z), tips.copy())
            if cube._blend >= 1.0:
                offsets.append(math.hypot(x - tips[0], y - tips[1]))
                heights.append(abs(z - max(tips[2], CUBE_HALF)))
            peak_z = max(peak_z, z)
        else:
            prev = None
    settle = data.time + 2.0
    while data.time < settle:
        mujoco.mj_step(model, data)
    final = data.xpos[cube._body]
    grabs = sum("GRABBED" in e for e in events)
    rels = sum("RELEASED" in e for e in events)
    tag = f"lag {lag:.1f} s, jaw +{extra:.1f}:"
    check(grabs == 1 and rels == 1, f"{tag} grabbed once, released once",
          "; ".join(e.strip() for e in events))
    check(grab_jump is not None and grab_jump < 0.001, f"{tag} cube does not jump at the grab",
          f"moved {(grab_jump or 0) * 1000:.2f} mm on the grab tick")
    if cube_steps:
        check(max(cube_steps) <= 2 * max(tip_steps) + 0.0005,
              f"{tag} eases into the jaws no faster than the lift",
              f"largest step {max(cube_steps) * 1000:.1f} mm vs fingertips {max(tip_steps) * 1000:.1f} mm")
    check(bool(offsets) and max(offsets) < 0.005, f"{tag} carried centred in the shown jaws",
          f"worst {max(offsets or [0]) * 1000:.2f} mm sideways")
    check(bool(heights) and max(heights) < 0.005, f"{tag} carried at finger height",
          f"worst {max(heights or [0]) * 1000:.2f} mm off the fingertips")
    check(peak_z > 0.05, f"{tag} lifted", f"peak {peak_z * 100:.1f} cm")
    check(abs(final[2] - CUBE_HALF) < 0.002, f"{tag} landed on the floor", f"z={final[2]:.4f}")
    if release_tips is not None:
        miss = math.hypot(final[0] - release_tips[0], final[1] - release_tips[1])
        check(miss < 0.03, f"{tag} landed under where the shown jaws opened",
              f"{miss * 100:.1f} cm")

print("\n" + "=" * 70)
print(f"  {'ALL CHECKS PASSED' if failures == 0 else f'{failures} CHECK(S) FAILED'}")
print("=" * 70)
sys.exit(1 if failures else 0)
