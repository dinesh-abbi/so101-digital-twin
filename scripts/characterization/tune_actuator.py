#!/usr/bin/env python3
"""
Find the actuator parameters that make the sim elbow move like the real one.

The first replay showed the model is accurate at rest (median |E| 0.115 deg)
but wrong in flight: on a 20 deg step the sim peaks at 243 deg/s against the
real 149 deg/s, overshoots to 23.8 deg against a 19.6 deg target, then
settles correctly. Endpoints right, dynamics too aggressive.

That is the signature of a position servo whose gains were never fitted to
this hardware. kp=998.22 with kv=2.731 is heavily underdamped: for the elbow's
reflected inertia the critical damping would be far higher, so the joint
slams into the target and rings.

Rather than hand-pick a "better looking" number, this sweeps kp, kv and the
force limit against the recorded step responses and reports the combination
that minimises RMSE over the real data. The search is deliberately coarse and
its objective is stated: fit the STEP experiments (clean, repeated, ten
transitions each), then the result is checked against the ramp and triangle
runs it was NOT fitted to, so we can tell fitting from overfitting.
"""

import argparse
import csv
import json
import math
import pathlib

import mujoco
import numpy as np

DEFAULT_SCENE = (pathlib.Path(__file__).parents[1] / "digital_twin_env" /
                 "robot_keyboard_test" / "keyboard_robot_scene.xml")
DEFAULT_CAL = (pathlib.Path.home() /
               ".cache/huggingface/lerobot/calibration/robots/so_follower" /
               "twin_follower.json")
LATENCY_S = 0.065


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def simulate(model, data, aid, jid, qadr, rows, latency_s):
    """Replay one recorded run and return (real, sim) angle arrays in degrees."""
    sched, last = [], None
    for r in rows:
        c = float(r["commanded_deg"])
        if c != last:
            sched.append((float(r["time"]) + latency_s, math.radians(c)))
            last = c

    start = math.radians(float(rows[0]["actual_deg"]))
    mujoco.mj_resetData(model, data)
    data.qpos[qadr] = start
    data.ctrl[aid] = start
    for _ in range(int(0.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    data.time = 0.0

    si = 0
    real = np.empty(len(rows))
    sim = np.empty(len(rows))
    for k, r in enumerate(rows):
        t_target = float(r["time"])
        while data.time < t_target:
            while si < len(sched) and sched[si][0] <= data.time:
                data.ctrl[aid] = sched[si][1]
                si += 1
            mujoco.mj_step(model, data)
        real[k] = float(r["actual_deg"])
        sim[k] = math.degrees(data.qpos[qadr])
    return real, sim


def set_gains(model, aid, kp, kv, frc):
    """A MuJoCo <position> actuator is gain kp, bias (0, -kp, -kv)."""
    model.actuator_gainprm[aid][0] = kp
    model.actuator_biasprm[aid][1] = -kp
    model.actuator_biasprm[aid][2] = -kv
    model.actuator_forcerange[aid][0] = -frc
    model.actuator_forcerange[aid][1] = frc


def score(model, data, aid, jid, qadr, runs):
    """RMSE in degrees across every sample of every supplied run."""
    tot, n = 0.0, 0
    for rows in runs:
        real, sim = simulate(model, data, aid, jid, qadr, rows, LATENCY_S)
        tot += float(np.sum((real - sim) ** 2))
        n += len(real)
    return math.sqrt(tot / n)


def check_calibration_drift(rows, joint):
    """Warn if the CSVs were recorded against a different calibration.

    The characterisation script converts normalised units to degrees using the
    joint's calibrated span at RECORDING time, then stores degrees. Recalibrate
    afterwards and the stored degrees no longer correspond to the current
    mapping - every command looks systematically too large or too small.

    This is not hypothetical. Recalibrating on 2026-08-26 widened
    shoulder_lift's span by 6.8%, and the shoulder CSVs recorded just before it
    then showed a -5.5% scale error against the model. It looked exactly like a
    gravity or gain problem: settled error varied 1.2 deg across the joint's
    range, and the gain sweep "improved" 18% by softening kp to absorb it while
    held-out error got worse. Removing the scale error collapsed the spread to
    0.05 deg with the ORIGINAL gains, proving the actuator was never at fault.

    Recovering the recording-time span from the data: a segment commanded to N
    normalised units was stored as N * (span_deg / 200). Comparing the largest
    stored command against what the current calibration would produce for the
    same experiment recovers the ratio.
    """
    try:
        cal = json.load(open(DEFAULT_CAL))
    except Exception:
        return
    if joint not in cal:
        return
    jc = cal[joint]
    now_deg_per_unit = (jc["range_max"] - jc["range_min"]) / 4096.0 * 360.0 / 200.0

    # EXPERIMENTS command whole numbers of units; the recorded degree value for
    # the largest command divided by that unit count gives deg-per-unit at
    # recording time.
    cmds = sorted({abs(float(r["commanded_deg"])) for r in rows})
    if not cmds or cmds[-1] == 0:
        return
    peak_deg = cmds[-1]
    units = round(peak_deg / now_deg_per_unit)
    if units <= 0:
        return
    then_deg_per_unit = peak_deg / units
    drift = then_deg_per_unit / now_deg_per_unit - 1.0
    if abs(drift) > 0.01:
        print()
        print(f"  WARNING: these CSVs look {drift * 100:+.1f}% out of step with the")
        print(f"  current calibration for '{joint}'.")
        print(f"    deg per unit at recording time : {then_deg_per_unit:.5f}")
        print(f"    deg per unit now               : {now_deg_per_unit:.5f}")
        print("  The joint was almost certainly recalibrated after recording.")
        print("  Any fit against this data will absorb the difference as a fake")
        print("  scale error. Re-record the data, or exclude this joint.")
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(DEFAULT_SCENE))
    ap.add_argument("--joint", default="elbow_flex")
    ap.add_argument("--data-dir", default=None)
    args = ap.parse_args()

    root = pathlib.Path(__file__).parent
    d = pathlib.Path(args.data_dir) if args.data_dir else root / "raw_data"

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, args.joint)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
    qadr = model.jnt_qposadr[jid]

    kp0 = float(model.actuator_gainprm[aid][0])
    kv0 = float(-model.actuator_biasprm[aid][2])
    frc0 = float(model.actuator_forcerange[aid][1])

    # Fit on the step runs only; hold the rest back as a check.
    fit = [load_rows(p) for p in sorted(d.glob(f"{args.joint}_step_*_no_load.csv"))]
    held = [load_rows(p) for p in sorted(d.glob(f"{args.joint}_*_no_load.csv"))
            if "step_" not in p.name]
    if not fit:
        raise SystemExit(f"no step CSVs in {d}")

    check_calibration_drift(fit[0], args.joint)

    print("=" * 72)
    print(f"  Actuator fit - {args.joint}")
    print("=" * 72)
    print(f"  fitting on {len(fit)} step runs, checking on {len(held)} held-out runs")
    print(f"  starting point: kp={kp0:.2f} kv={kv0:.3f} force={frc0:.2f}")

    base = score(model, data, aid, jid, qadr, fit)
    base_held = score(model, data, aid, jid, qadr, held) if held else float("nan")
    print(f"  baseline RMSE : {base:.3f} deg (fit)   {base_held:.3f} deg (held-out)\n")

    # Coarse grid. kp far below the shipped value, kv far above - the observed
    # overshoot says the model is both too stiff and too lightly damped.
    kps = [20, 40, 60, 80, 120, 180, 260, 400, 600, 998.22]
    kvs = [1, 2, 4, 6, 9, 13, 18, 25, 35, 50]
    frcs = [2.94]

    best = (base, kp0, kv0, frc0)
    print(f"  sweeping {len(kps) * len(kvs) * len(frcs)} combinations...")
    for kp in kps:
        row = []
        for kv in kvs:
            for frc in frcs:
                set_gains(model, aid, kp, kv, frc)
                s = score(model, data, aid, jid, qadr, fit)
                row.append(s)
                if s < best[0]:
                    best = (s, kp, kv, frc)
        print(f"    kp {kp:7.2f} | " + " ".join(f"{v:5.2f}" for v in row))

    s, kp, kv, frc = best
    print(f"\n  best on fit set: kp={kp} kv={kv} force={frc}  RMSE {s:.3f} deg")
    print(f"    improvement over shipped gains: {base:.3f} -> {s:.3f} deg "
          f"({(1 - s / base) * 100:.0f}% better)")

    if held:
        set_gains(model, aid, kp, kv, frc)
        h = score(model, data, aid, jid, qadr, held)
        print(f"  held-out runs  : {base_held:.3f} -> {h:.3f} deg "
              f"({(1 - h / base_held) * 100:+.0f}%)")
        # State the verdict rather than asserting success unconditionally: an
        # earlier version printed "held-out improving too means this is a real
        # fit" even when held-out had got WORSE, which is exactly the case the
        # check exists to catch.
        if h < base_held:
            print("    held-out improved too - this is a real fit, not overfitting.")
        else:
            print("    WARNING: held-out did NOT improve. The fit set gained but")
            print("    unseen trajectories did not, which is what overfitting looks")
            print("    like. Do not apply these gains without understanding why.")
            print("    A common cause is a systematic offset in the data that soft")
            print("    gains absorb without modelling anything - check whether the")
            print("    settled error varies with joint angle, and whether the")
            print("    calibration used to record the CSVs still matches the")
            print("    current one (see check_calibration_drift below).")

    print(f"\n  To apply, in so101_assets/so101.xml class 'sts3215':")
    print(f"    <position kp=\"{kp}\" kv=\"{kv}\" forcerange=\"-{frc} {frc}\"/>")


if __name__ == "__main__":
    main()
