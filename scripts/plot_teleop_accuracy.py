#!/usr/bin/env python3
"""Plot real vs sim joint angles from a --record CSV.

analyze_teleop_accuracy.py gives the numbers; this shows them. Two rows
per joint:
  - the real arm's angle and the sim's, overlaid (they should sit on top
    of each other)
  - the error between them over time, so it is obvious WHERE the error
    happens rather than just how big it is on average

That distinction matters for this project: wrist_flex and elbow_flex have
a large RMSE but a small median, because the error is concentrated in the
short windows where the arm folds tightly enough to self-collide in sim
but not in reality. A single RMSE number hides that; the error trace makes
it visible at a glance.

Usage:
    python plot_teleop_accuracy.py teleop_log_pick2.csv
    python plot_teleop_accuracy.py teleop_log_pick2.csv --out plot.png
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # write a file; no GUI needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path")
    ap.add_argument("--out", default=None,
                    help="Output image path (default: <csv name>_accuracy.png)")
    args = ap.parse_args()

    with open(args.csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{args.csv_path}: no data rows")

    t = np.array([float(r["wall_time"]) for r in rows])
    t -= t[0]

    fig, axes = plt.subplots(len(JOINT_NAMES), 2, figsize=(14, 16),
                             sharex=True)
    fig.suptitle(f"Real vs MuJoCo sim -- {Path(args.csv_path).name}",
                 fontsize=13)

    for i, j in enumerate(JOINT_NAMES):
        real = np.array([float(r[f"{j}_real_sim_deg"]) for r in rows])
        sim = np.array([float(r[f"{j}_sim_qpos_deg"]) for r in rows])
        err = np.abs(real - sim)
        rmse = float(np.sqrt(np.mean(err ** 2)))
        med = float(np.median(err))

        ax = axes[i][0]
        ax.plot(t, real, label="real arm", linewidth=1.2)
        ax.plot(t, sim, label="sim", linewidth=1.0, alpha=0.8, linestyle="--")
        ax.set_ylabel(f"{j}\n(degrees)", fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(alpha=0.3)

        ax = axes[i][1]
        ax.plot(t, err, color="crimson", linewidth=0.9)
        ax.axhline(1.0, color="green", linestyle=":", linewidth=1,
                   label="1 deg")
        ax.set_ylabel("|error| deg", fontsize=8)
        ax.set_title(f"RMSE {rmse:.2f}deg   median {med:.2f}deg",
                     fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(alpha=0.3)

    axes[-1][0].set_xlabel("time (s)")
    axes[-1][1].set_xlabel("time (s)")
    plt.tight_layout(rect=[0, 0, 1, 0.985])

    out = args.out or str(Path(args.csv_path).with_suffix("")) + "_accuracy.png"
    plt.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
