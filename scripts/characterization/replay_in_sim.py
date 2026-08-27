#!/usr/bin/env python3
"""
Task 2 - replay the real command trajectories in MuJoCo.

Reads the CSVs produced by so101_joint_characterization.py and drives the
SAME joint in simulation with the SAME command signal at the SAME wall-clock
times, then writes a per-sample comparison so that

    E(t) = q_real(t) - q_sim(t)

is a true like-for-like difference rather than a comparison against an
idealised trajectory we invented afterwards.

Why replay the recorded command rather than rebuild it
------------------------------------------------------
The real run's waypoint timing was set by `hold` durations that the serial
loop only approximately honoured - a write could retry, a read could be
dropped. The CSV records when each command ACTUALLY landed. Rebuilding the
trajectory from EXPERIMENTS would reproduce the intent, not the event, and
every few-millisecond discrepancy would show up as fake model error.

Units
-----
The real CSV is in degrees of the joint's own CALIBRATED travel:
LeRobot normalises ticks to -100..100 across range_min..range_max, and the
characterisation script converted that to degrees via the tick span. The sim
joint is in radians of TRUE joint angle.

These are only the same thing if the calibrated span matches the modelled
span. For this arm's elbow they very nearly do:

    real : 3086 - 858 = 2228 ticks = 195.82 deg
    sim  : 2 * 1.69 rad            = 193.64 deg

a 1.1% difference. That is close enough to compare directly, but it is a
measured coincidence rather than a guarantee, so the ratio is reported and
--scale-span can correct for it explicitly.

Latency
-------
The real joint takes ~65 ms to begin moving after a command lands. The sim
actuator responds on the next 5 ms step. Replaying without accounting for
this makes the sim look "wrong" by a fixed 65 ms everywhere, which tells us
nothing we did not already measure. So the delay is applied as an explicit,
adjustable model input (--latency-ms) and the comparison is reported both
with and without it.
"""

import argparse
import csv
import json
import math
import pathlib
import sys

import mujoco
import numpy as np

# The scene that sets meshdir correctly. so101.xml on its own cannot resolve
# its meshes, and scene.xml only resolves them when cwd happens to be right.
DEFAULT_SCENE = (pathlib.Path(__file__).parents[1] / "digital_twin_env" /
                 "robot_keyboard_test" / "keyboard_robot_scene.xml")

DEFAULT_CAL = (pathlib.Path.home() /
               ".cache/huggingface/lerobot/calibration/robots/so_follower" /
               "twin_follower.json")

TICKS_PER_REV = 4096.0

# Measured on this arm in Task 1. Used as the default so a plain run
# reproduces the real timing; override to explore the model's sensitivity.
DEFAULT_LATENCY_MS = 65.0


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def real_span_deg(cal, joint):
    jc = cal[joint]
    span_ticks = jc["range_max"] - jc["range_min"]
    return span_ticks / TICKS_PER_REV * 360.0


def sim_span_deg(model, joint):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if jid < 0:
        raise SystemExit(f"joint '{joint}' not present in the model")
    lo, hi = model.jnt_range[jid]
    return math.degrees(hi - lo)


def replay(model, data, joint, rows, latency_s, scale, settle_s=0.5):
    """Drive the sim joint with the recorded command signal.

    Returns one dict per real sample, carrying the sim angle at that instant.

    The sim is stepped on its own fixed 5 ms clock and sampled whenever the
    real log has a sample, rather than being stepped once per real sample.
    The real loop ran at ~94.5 Hz with jitter; stepping the physics at that
    rate would make the integrator's behaviour depend on serial timing noise.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
    if aid < 0:
        raise SystemExit(f"actuator '{joint}' not present in the model")
    qadr = model.jnt_qposadr[jid]

    # Command schedule: (time_the_command_landed, target_radians).
    # Only transitions matter - a repeated identical command is not an event.
    sched = []
    last = None
    for r in rows:
        c = float(r["commanded_deg"])
        if c != last:
            sched.append((float(r["time"]) + latency_s, math.radians(c * scale)))
            last = c
    if not sched:
        raise SystemExit("no commands found in this CSV")

    # Start the sim at the real joint's initial pose and let it settle, so
    # the first step is not contaminated by the arm falling into place.
    start_rad = math.radians(float(rows[0]["actual_deg"]) * scale)
    mujoco.mj_resetData(model, data)
    data.qpos[qadr] = start_rad
    data.ctrl[aid] = start_rad
    for _ in range(int(settle_s / model.opt.timestep)):
        mujoco.mj_step(model, data)
    # Settling advanced the clock; rezero so sim time lines up with CSV time.
    data.time = 0.0

    out = []
    si = 0                       # next scheduled command
    data.ctrl[aid] = sched[0][1] if sched[0][0] <= 0 else start_rad
    prev_q = data.qpos[qadr]
    prev_t = 0.0

    for r in rows:
        t_target = float(r["time"])

        # Advance physics to this sample's timestamp, applying any command
        # that falls due along the way.
        while data.time < t_target:
            while si < len(sched) and sched[si][0] <= data.time:
                data.ctrl[aid] = sched[si][1]
                si += 1
            mujoco.mj_step(model, data)

        q = float(data.qpos[qadr])
        dt = data.time - prev_t
        vel = math.degrees(q - prev_q) / scale / dt if dt > 0 else 0.0
        prev_q, prev_t = q, data.time

        sim_deg = math.degrees(q) / scale
        real_deg = float(r["actual_deg"])
        out.append({
            "time": r["time"],
            "commanded_deg": r["commanded_deg"],
            "real_deg": f"{real_deg:.4f}",
            "sim_deg": f"{sim_deg:.4f}",
            "error_deg": f"{real_deg - sim_deg:.4f}",
            "real_vel_deg_s": r.get("measured_vel_deg_s", ""),
            "sim_vel_deg_s": f"{vel:.3f}",
            "experiment_id": r.get("experiment_id", ""),
            "load_condition": r.get("load_condition", ""),
        })
    return out


def stats(rows):
    e = np.array([float(r["error_deg"]) for r in rows])
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "mae": float(np.mean(np.abs(e))),
        "max_abs": float(np.max(np.abs(e))),
        "bias": float(np.mean(e)),
        "n": len(e),
    }


def main():
    ap = argparse.ArgumentParser(description="Replay real trajectories in MuJoCo.")
    ap.add_argument("--scene", default=str(DEFAULT_SCENE))
    ap.add_argument("--joint", default="elbow_flex")
    ap.add_argument("--cal", default=str(DEFAULT_CAL))
    ap.add_argument("--data-dir", default=None,
                    help="Where the real CSVs live (default ./raw_data)")
    ap.add_argument("--out-dir", default=None,
                    help="Where to write the comparison CSVs (default ./sim_data)")
    ap.add_argument("--latency-ms", type=float, default=DEFAULT_LATENCY_MS,
                    help=f"Command delay applied to the sim (default {DEFAULT_LATENCY_MS})")
    ap.add_argument("--scale-span", action="store_true",
                    help="Correct for the real/sim calibrated-span mismatch")
    args = ap.parse_args()

    root = pathlib.Path(__file__).parent
    data_dir = pathlib.Path(args.data_dir) if args.data_dir else root / "raw_data"
    out_dir = pathlib.Path(args.out_dir) if args.out_dir else root / "sim_data"

    csvs = sorted(data_dir.glob(f"{args.joint}_*.csv"))
    if not csvs:
        raise SystemExit(f"no CSVs for '{args.joint}' in {data_dir}")

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    cal = json.load(open(args.cal))
    if args.joint not in cal:
        raise SystemExit(f"joint '{args.joint}' not in calibration {args.cal}")

    r_span = real_span_deg(cal, args.joint)
    s_span = sim_span_deg(model, args.joint)
    ratio = s_span / r_span
    scale = ratio if args.scale_span else 1.0

    print("=" * 72)
    print(f"  Task 2 - replaying {args.joint} in MuJoCo")
    print("=" * 72)
    print(f"  scene            : {pathlib.Path(args.scene).name}")
    print(f"  sim timestep     : {model.opt.timestep * 1000:.1f} ms")
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, args.joint)
    print(f"  actuator kp/kv   : {model.actuator_gainprm[aid][0]:.2f} / "
          f"{-model.actuator_biasprm[aid][2]:.3f}")
    print(f"  force limit      : +/- {model.actuator_forcerange[aid][1]:.2f} Nm")
    print(f"  real span        : {r_span:.2f} deg (calibrated)")
    print(f"  sim span         : {s_span:.2f} deg (modelled)")
    print(f"  span ratio       : {ratio:.4f}  "
          f"({'applied' if args.scale_span else 'NOT applied - see --scale-span'})")
    print(f"  command latency  : {args.latency_ms:.1f} ms")
    print()

    out_dir.mkdir(parents=True, exist_ok=True)
    summary = []

    for path in csvs:
        rows = load_rows(path)
        if not rows:
            continue
        cmp_rows = replay(model, data, args.joint, rows,
                          args.latency_ms / 1000.0, scale)
        out_path = out_dir / path.name.replace(".csv", "_vs_sim.csv")
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(cmp_rows[0].keys()))
            w.writeheader()
            w.writerows(cmp_rows)

        s = stats(cmp_rows)
        s["name"] = path.stem
        s["load"] = rows[0].get("load_condition", "")
        summary.append(s)
        print(f"  {path.name}")
        print(f"    RMSE {s['rmse']:6.3f} deg   MAE {s['mae']:6.3f}   "
              f"max |E| {s['max_abs']:6.3f}   bias {s['bias']:+6.3f}")

    print()
    print("  " + "-" * 68)
    print("  SUMMARY - sim-to-real gap E(t) = q_real - q_sim")
    print("  " + "-" * 68)
    for load in ("no_load", "loaded"):
        grp = [s for s in summary if s["load"] == load]
        if not grp:
            continue
        rmse = math.sqrt(sum(s["rmse"] ** 2 * s["n"] for s in grp) /
                         sum(s["n"] for s in grp))
        mae = sum(s["mae"] * s["n"] for s in grp) / sum(s["n"] for s in grp)
        print(f"    {load:<10}  RMSE {rmse:6.3f} deg   MAE {mae:6.3f} deg   "
              f"worst {max(s['max_abs'] for s in grp):6.3f} deg")

    print(f"\n  Comparison CSVs written to: {out_dir}")


if __name__ == "__main__":
    main()
