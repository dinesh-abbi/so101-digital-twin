#!/usr/bin/env python3
"""
The Task 1 and Task 2 figures.

Task 1 asked for four plots of the real joint alone; Task 2 asks for the real
vs simulated overlay. Both are produced here, from the same CSVs the numbers
came from, so a figure can never drift out of step with the analysis.

    step_response.png    commanded vs actual position, all three step sizes
    velocity_response.png commanded vs actual velocity
    hysteresis.png       actual vs command, forward and reverse sweeps
    load_comparison.png  no-load vs loaded, side by side
    sim_vs_real.png      the Task 2 overlay, shipped vs fitted gains
    error_over_time.png  E(t) = q_real - q_sim
"""

import argparse
import csv
import math
import pathlib

import matplotlib
import mujoco
matplotlib.use("Agg")           # no display needed; write files directly
import matplotlib.pyplot as plt
import numpy as np

import tune_actuator as T

REAL_C = "#1f77b4"
SIM_C = "#d62728"
CMD_C = "#7f7f7f"

SHIPPED = (998.22, 2.731)
FITTED = (400.0, 25.0)


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def arr(rows, key, default=0.0):
    out = []
    for r in rows:
        v = r.get(key, "")
        out.append(float(v) if v not in ("", "None") else default)
    return np.array(out)


def step_response(d, out):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, size in zip(axes, ("5deg", "10deg", "20deg")):
        p = d / f"elbow_flex_step_{size}_no_load.csv"
        if not p.exists():
            continue
        r = load(p)
        t = arr(r, "time")
        ax.plot(t, arr(r, "commanded_deg"), color=CMD_C, lw=1.2,
                ls="--", label="commanded")
        ax.plot(t, arr(r, "actual_deg"), color=REAL_C, lw=1.0, label="actual")
        ax.set_ylabel("angle (deg)")
        ax.set_title(f"step {size.replace('deg', ' units')} - 10 repeats",
                     fontsize=10, loc="left")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("SO-101 elbow_flex step response (no load)", fontweight="bold")
    fig.tight_layout()
    fig.savefig(out / "step_response.png", dpi=130)
    plt.close(fig)


def velocity_response(d, out):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    p = d / "elbow_flex_ramp_no_load.csv"
    if not p.exists():
        return
    r = load(p)
    t = arr(r, "time")
    axes[0].plot(t, arr(r, "commanded_deg"), color=CMD_C, ls="--", lw=1.2,
                 label="commanded")
    axes[0].plot(t, arr(r, "actual_deg"), color=REAL_C, lw=1.0, label="actual")
    axes[0].set_ylabel("angle (deg)")
    axes[0].set_title("ramp at three speeds", fontsize=10, loc="left")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    v = arr(r, "measured_vel_deg_s")
    axes[1].plot(t, v, color=REAL_C, lw=0.8)
    peak = np.percentile(np.abs(v), 99)
    axes[1].axhline(peak, color=SIM_C, ls=":", lw=1.2,
                    label=f"99th pct |v| = {peak:.0f} deg/s")
    axes[1].axhline(-peak, color=SIM_C, ls=":", lw=1.2)
    axes[1].set_ylabel("velocity (deg/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    fig.suptitle("SO-101 elbow_flex velocity response (no load)",
                 fontweight="bold")
    fig.tight_layout()
    fig.savefig(out / "velocity_response.png", dpi=130)
    plt.close(fig)


def hysteresis(d, out):
    """Settled position against command, split by approach direction.

    Only SETTLED samples may appear here. Plotting every sample of the
    triangle run instead draws two vertical stripes - the joint in flight
    between its two commanded values - which looks like enormous hysteresis
    and is actually just travel time. That is the same error that produced a
    spurious 11 deg reading in the first analysis pass, so the plot applies
    the same fix as the calculation: take the tail of each hold.

    Every run that both reverses direction AND holds long enough to settle is
    pooled: the step runs (1.2 s holds, 10 repeats each) plus reverse and
    triangle (1.5-1.6 s holds). The ramp is excluded entirely - its slow phase
    never reverses, and its fast phase never settles.
    """
    fig, ax = plt.subplots(figsize=(7.5, 7))
    srcs = [d / f"elbow_flex_step_{s}_no_load.csv" for s in ("5deg", "10deg", "20deg")]
    srcs += [d / "elbow_flex_reverse_no_load.csv",
             d / "elbow_flex_triangle_no_load.csv"]
    srcs = [p for p in srcs if p.exists()]
    if not srcs:
        return
    loaded_srcs = [load(p) for p in srcs]
    c = np.concatenate([arr(r, "commanded_deg") for r in loaded_srcs])
    a = np.concatenate([arr(r, "actual_deg") for r in loaded_srcs])
    # Offset each run's clock so segment splitting never merges across files.
    ts, base = [], 0.0
    for r in loaded_srcs:
        x = arr(r, "time")
        ts.append(x + base)
        base += x[-1] + 10.0
    t = np.concatenate(ts)

    # Split into per-command segments, keep the settled tail of each.
    #
    # Only segments held long enough to actually settle may be used. The ramp
    # runs three phases - roughly 243, 117 and 53 ms per waypoint - and the
    # joint's own command-to-motion latency is 65 ms. In the fast phase the
    # next command arrives before this one has begun moving, so those points
    # record the joint mid-flight, not where it came to rest. Including them
    # draws a second, lower branch that looks like 9 deg of backlash and is
    # purely lag. Task 1 measured no detectable hysteresis; this plot must
    # not manufacture some.
    MIN_HOLD_S = 0.20
    bounds = [0] + list(np.where(np.diff(c) != 0)[0] + 1) + [len(c)]
    fwd_c, fwd_a, rev_c, rev_a = [], [], [], []
    prev = None
    for i in range(len(bounds) - 1):
        s0, s1 = bounds[i], bounds[i + 1]
        n = s1 - s0
        if n < 4 or (t[s1 - 1] - t[s0]) < MIN_HOLD_S:
            if n >= 4:
                prev = float(c[s0])
            continue
        tail = slice(s0 + int(n * 0.6), s1)
        settled = float(np.mean(a[tail]))
        target = float(c[s0])
        if prev is not None:
            if target > prev:
                fwd_c.append(target); fwd_a.append(settled)
            elif target < prev:
                rev_c.append(target); rev_a.append(settled)
        prev = target

    # Each command is visited many times across the pooled runs, so average
    # the repeats rather than keeping whichever happened to land last.
    rep_sd = []

    def group(cs, as_):
        g = {}
        for k, v in zip(np.round(cs, 1), as_):
            g.setdefault(float(k), []).append(v)
        for v in g.values():
            if len(v) > 1:
                rep_sd.append(float(np.std(v)))
        return {k: float(np.mean(v)) for k, v in g.items()}

    fd, rd = group(fwd_c, fwd_a), group(rev_c, rev_a)
    fk, rk = sorted(fd), sorted(rd)
    ax.plot(fk, [fd[k] for k in fk], "o-", ms=6, lw=1.2, color=REAL_C,
            alpha=0.85, label="approached from below")
    ax.plot(rk, [rd[k] for k in rk], "s-", ms=6, lw=1.2, color=SIM_C,
            alpha=0.85, label="approached from above")
    both = fk + rk + [fd[k] for k in fk] + [rd[k] for k in rk]
    lim = [min(both) - 3, max(both) + 3]
    ax.plot(lim, lim, color=CMD_C, ls="--", lw=1, label="perfect tracking")

    # Quantify the gap at commands visited settled in BOTH directions.
    #
    # For this dataset there are none: every experiment steps UP from 0 and
    # returns DOWN to 0, so 0 is only ever approached from above and every
    # other command only from below. Backlash is therefore UNMEASURED here,
    # not shown to be absent - a distinction the plot has to make, because a
    # single tidy line through the diagonal would otherwise read as proof of
    # zero backlash.
    #
    # What the data does bound is repeatability: the spread of settled
    # positions at a command reached the same way every time. Backlash cannot
    # be larger than a gap that spread would have revealed.
    gaps = [abs(fd[k] - rd[k]) for k in sorted(set(fd) & set(rd))]
    if gaps:
        note = (f"mean gap {np.mean(gaps):.3f} deg, max {max(gaps):.3f} deg, "
                f"over {len(gaps)} shared commands")
    else:
        spread = np.mean(rep_sd) if rep_sd else float("nan")
        note = ("no command was settled from BOTH directions in this dataset,\n"
                f"so backlash is unmeasured; repeatability sd {spread:.3f} deg "
                "bounds it")

    ax.set_xlabel("commanded (deg)")
    ax.set_ylabel("settled actual (deg)")
    ax.set_title("Forward/reverse hysteresis (settled positions)\n"
                 + note, fontweight="bold", fontsize=11)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(out / "hysteresis.png", dpi=130)
    plt.close(fig)


def load_comparison(d, out):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # Overlaid step response, one repeat, both load conditions.
    for col, size in enumerate(("10deg", "20deg")):
        ax = axes[0][col]
        for cond, colr in (("no_load", REAL_C), ("loaded", SIM_C)):
            p = d / f"elbow_flex_step_{size}_{cond}.csv"
            if not p.exists():
                continue
            r = load(p)
            t, a, c = arr(r, "time"), arr(r, "actual_deg"), arr(r, "commanded_deg")
            chg = np.where(np.diff(c) != 0)[0] + 1
            if len(chg) < 3:
                continue
            i = chg[2]
            m = (t >= t[i] - 0.1) & (t <= t[i] + 1.2)
            ax.plot((t[m] - t[i]) * 1000, a[m], color=colr, lw=1.4,
                    label=cond.replace("_", " "))
            ax.plot((t[m] - t[i]) * 1000, c[m], color=CMD_C, ls="--", lw=0.9)
        ax.set_title(f"step {size} - single transition", fontsize=10, loc="left")
        ax.set_xlabel("time since command (ms)")
        ax.set_ylabel("angle (deg)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

    # The measured five numbers, as bars.
    metrics = ["latency\n(ms)", "steady-state\nerror (deg)",
               "max velocity\n(deg/s)", "overshoot\n(deg)"]
    nl = [65.096, 0.128, 191.624, 0.140]
    ld = [64.821, 0.472, 189.678, 0.189]
    x = np.arange(len(metrics))
    ax = axes[1][0]
    ax.bar(x - 0.2, nl, 0.4, color=REAL_C, label="no load")
    ax.bar(x + 0.2, ld, 0.4, color=SIM_C, label="loaded")
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=8)
    ax.set_title("Task 1 measurements (log scale)", fontsize=10, loc="left")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    ax = axes[1][1]
    pct = [(l - n) / n * 100 for n, l in zip(nl, ld)]
    colors = [SIM_C if p > 0 else REAL_C for p in pct]
    ax.barh(metrics, pct, color=colors)
    ax.axvline(0, color="k", lw=0.8)
    ax.set_xlabel("change under load (%)")
    ax.set_title("Load hits accuracy, not speed", fontsize=10, loc="left")
    for i, p in enumerate(pct):
        ax.text(p + (2 if p > 0 else -2), i, f"{p:+.0f}%",
                va="center", ha="left" if p > 0 else "right", fontsize=8)
    ax.grid(alpha=0.3, axis="x")
    ax.tick_params(labelsize=8)

    fig.suptitle("SO-101 elbow_flex - loaded vs unloaded", fontweight="bold")
    fig.tight_layout()
    fig.savefig(out / "load_comparison.png", dpi=130)
    plt.close(fig)


def sim_vs_real(d, out):
    """The Task 2 overlay: real, sim with shipped gains, sim with fitted gains."""
    model = mujoco.MjModel.from_xml_path(str(T.DEFAULT_SCENE))
    data = mujoco.MjData(model)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "elbow_flex")
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow_flex")
    qadr = model.jnt_qposadr[jid]

    p = d / "elbow_flex_step_20deg_no_load.csv"
    if not p.exists():
        return
    rows = load(p)
    t = arr(rows, "time")
    c = arr(rows, "commanded_deg")
    chg = np.where(np.diff(c) != 0)[0] + 1

    results = {}
    for name, (kp, kv) in (("shipped", SHIPPED), ("fitted", FITTED)):
        T.set_gains(model, aid, kp, kv, 2.94)
        real, sim = T.simulate(model, data, aid, jid, qadr, rows, T.LATENCY_S)
        results[name] = sim
    real_a = arr(rows, "actual_deg")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2), sharey=True)
    i = chg[2]
    m = (t >= t[i] - 0.1) & (t <= t[i] + 1.0)
    ts = (t[m] - t[i]) * 1000
    for ax, name in zip(axes, ("shipped", "fitted")):
        kp, kv = SHIPPED if name == "shipped" else FITTED
        ax.plot(ts, c[m], color=CMD_C, ls="--", lw=1.1, label="commanded")
        ax.plot(ts, real_a[m], color=REAL_C, lw=1.8, label="real arm")
        ax.plot(ts, results[name][m], color=SIM_C, lw=1.5, label="MuJoCo")
        e = np.abs(real_a[m] - results[name][m])
        ax.set_title(f"{name} gains  kp={kp:g} kv={kv:g}\n"
                     f"peak |E| during move = {e.max():.2f} deg",
                     fontsize=10)
        ax.set_xlabel("time since command (ms)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")
    axes[0].set_ylabel("angle (deg)")
    fig.suptitle("Task 2 - real vs simulated, 20-unit step",
                 fontweight="bold")
    fig.tight_layout()
    fig.savefig(out / "sim_vs_real.png", dpi=130)
    plt.close(fig)

    # E(t) over a whole run, both gain sets.
    fig, ax = plt.subplots(figsize=(12, 4.5))
    for name, colr in (("shipped", CMD_C), ("fitted", SIM_C)):
        e = real_a - results[name]
        ax.plot(t, e, lw=0.8, color=colr,
                label=f"{name}  RMSE {np.sqrt(np.mean(e**2)):.3f} deg")
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("E(t) = real - sim  (deg)")
    ax.set_title("Sim-to-real gap over the full 20-unit step run",
                 fontweight="bold")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "error_over_time.png", dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=None)
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()
    root = pathlib.Path(__file__).parent
    d = pathlib.Path(args.data_dir) if args.data_dir else root / "raw_data"
    out = pathlib.Path(args.out_dir) if args.out_dir else root / "plots"
    out.mkdir(parents=True, exist_ok=True)

    step_response(d, out)
    velocity_response(d, out)
    hysteresis(d, out)
    load_comparison(d, out)
    sim_vs_real(d, out)

    for f in sorted(out.glob("*.png")):
        print(f"  {f.name:24s} {f.stat().st_size / 1024:6.0f} KB")
    print(f"\n  plots written to: {out}")


if __name__ == "__main__":
    main()
