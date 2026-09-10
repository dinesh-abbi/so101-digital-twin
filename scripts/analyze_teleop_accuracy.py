"""Analyze a --record CSV from m_lerobot_teleop_sim.py: how well does the
sim track the real follower during actual leader-driven teleop?

Two separate comparisons, on purpose:
  - real_sim_deg vs sim_ctrl_deg  -> MAPPING accuracy (real_to_sim's own
    fraction-of-range math against the follower's measured position). This
    should be ~exact every tick; a nonzero gap here means the mapping or
    the CSV logging itself is wrong, not the physics.
  - real_sim_deg vs sim_qpos_deg  -> TWIN accuracy end to end: does the
    position-actuator model (kp=400 kv=25 / per-joint overrides) actually
    converge to where the real arm is, under live teleop rather than the
    scripted step responses it was originally fitted against.

Also estimates LAG via cross-correlation between the real trace and the
sim qpos trace, per joint -- this project's characterization work
documents a fixed 65 ms real command-to-motion latency
(scripts/characterization/RESULTS.md); the sim inherits that plus its own
settling time, so some lag here is correct twin behaviour, not a bug.

Usage:
    python analyze_teleop_accuracy.py teleop_log.csv
"""

import argparse
import csv
import sys

import numpy as np

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")

    t = np.array([float(r["wall_time"]) for r in rows])
    t -= t[0]
    data = {}
    for j in JOINT_NAMES:
        data[j] = {
            "leader_cmd": np.array([float(r[f"{j}_leader_cmd"]) for r in rows]),
            "real_norm": np.array([float(r[f"{j}_real_norm"]) for r in rows]),
            "real_sim_deg": np.array([float(r[f"{j}_real_sim_deg"]) for r in rows]),
            "sim_ctrl_deg": np.array([float(r[f"{j}_sim_ctrl_deg"]) for r in rows]),
            "sim_qpos_deg": np.array([float(r[f"{j}_sim_qpos_deg"]) for r in rows]),
        }
    return t, data


def err_stats(a, b):
    e = np.abs(a - b)
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "median": float(np.median(e)),
        "p90": float(np.percentile(e, 90)),
        "max": float(np.max(e)),
    }


def estimate_lag_ms(t, real, sim, max_lag_s=0.5):
    """Cross-correlation lag: how many seconds does `sim` trail `real` by?

    Resamples onto a uniform grid first since wall-clock ticks are not
    perfectly evenly spaced (precise_sleep clamps to >=0 on a slow tick,
    it never catches up).
    """
    if t[-1] <= 0:
        return None
    dt = 0.01  # 100 Hz resample grid
    grid = np.arange(0, t[-1], dt)
    if len(grid) < 10:
        return None
    real_r = np.interp(grid, t, real)
    sim_r = np.interp(grid, t, sim)
    real_r -= real_r.mean()
    sim_r -= sim_r.mean()
    if np.std(real_r) < 1e-6 or np.std(sim_r) < 1e-6:
        return None  # joint never moved -- lag undefined

    max_lag_n = int(max_lag_s / dt)
    corr = np.correlate(sim_r, real_r, mode="full")
    lags = np.arange(-len(real_r) + 1, len(sim_r))
    center = len(corr) // 2
    window = slice(center - max_lag_n, center + max_lag_n + 1)
    best = lags[window][np.argmax(corr[window])]
    return float(best * dt * 1000.0)  # positive = sim trails real


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path")
    args = ap.parse_args()

    t, data = load(args.csv_path)
    duration = t[-1]
    n = len(t)
    print(f"loaded {n} ticks, {duration:.1f} s "
          f"({n / duration:.1f} Hz average)\n")

    print(f"{'joint':<14}{'mapping RMSE':>14}{'twin RMSE':>12}"
          f"{'median':>9}{'p90':>8}{'max':>8}{'lag(ms)':>9}")
    print("-" * 74)

    all_mapping_err = []
    all_twin_err = []
    for j in JOINT_NAMES:
        d = data[j]
        mapping = err_stats(d["real_sim_deg"], d["sim_ctrl_deg"])
        twin = err_stats(d["real_sim_deg"], d["sim_qpos_deg"])
        lag = estimate_lag_ms(t, d["real_sim_deg"], d["sim_qpos_deg"])
        lag_str = f"{lag:+.0f}" if lag is not None else "  n/a"

        print(f"{j:<14}{mapping['rmse']:>13.3f}°{twin['rmse']:>11.3f}°"
              f"{twin['median']:>8.3f}°{twin['p90']:>7.3f}°"
              f"{twin['max']:>7.3f}°{lag_str:>9}")

        all_mapping_err.extend(np.abs(d["real_sim_deg"] - d["sim_ctrl_deg"]).tolist())
        all_twin_err.extend(np.abs(d["real_sim_deg"] - d["sim_qpos_deg"]).tolist())

    all_mapping_err = np.array(all_mapping_err)
    all_twin_err = np.array(all_twin_err)
    print("-" * 74)
    print(f"{'OVERALL':<14}{np.sqrt(np.mean(all_mapping_err**2)):>13.3f}°"
          f"{np.sqrt(np.mean(all_twin_err**2)):>11.3f}°")

    print()
    print("mapping RMSE  = real_to_sim(follower) vs sim ctrl target")
    print("                should be ~0; nonzero means a mapping/logging bug")
    print("twin RMSE     = real_to_sim(follower) vs where the sim actually")
    print("                settled -- this is the number to compare against")
    print("                this project's own held-out fit RMSE (~0.6-0.8°,")
    print("                see scripts/characterization/RESULTS.md)")
    print("lag(ms)       = how far the sim trails the real arm by")
    print("                (cross-correlation; positive = sim is late).")
    print("                Some lag is CORRECT: real servo latency is a")
    print("                measured, fixed 65 ms regardless of load.")


if __name__ == "__main__":
    main()
