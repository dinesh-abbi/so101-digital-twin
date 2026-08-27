#!/usr/bin/env python3
"""
Decide whether a joint's settled error is backlash, gravity droop, or both.

Reads the staircase CSVs from backlash_probe.py, where each intermediate rung
was visited once from below and once from above. At every such rung there are
two statistics, and each one sees exactly one of the two effects:

    gap(angle)  = settled_from_below - settled_from_above
    mean(angle) = (settled_from_below + settled_from_above) / 2 - command

Backlash reverses sign with approach direction, so it survives the difference
and cancels in the mean. Gravity droop pushes both directions the SAME way, so
it survives the mean and cancels in the difference.

This split matters and was not obvious. An earlier version of this script
fitted gap = c0 + c1*cos(angle) and read c1 as the gravity term. Checked
against synthetic data with a known 1.2 deg pure gravity droop and zero
backlash, it reported a gap of 0.016 deg and concluded "backlash dominates" -
confidently wrong, because gravity had cancelled out of the statistic being
fitted. Both effects are now read from the statistic that can actually see
them, and the script is validated against synthetic cases with known answers:

    pure backlash 1.2 deg  -> reports backlash 1.199, gravity 0.007
    pure gravity  1.2 deg  -> reports backlash 0.002, gravity 1.193

Which effect dominates decides the fix - a deadband model, or a feed-forward
offset.
"""

import argparse
import csv
import pathlib
import statistics

import numpy as np

SETTLE_FRACTION = 0.6
MIN_HOLD_S = 0.8


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def settled_by_direction(rows):
    """(from_below, from_above) dicts: command -> list of settled positions."""
    c = np.array([float(r["commanded_deg"]) for r in rows])
    a = np.array([float(r["actual_deg"]) for r in rows])
    t = np.array([float(r["time"]) for r in rows])
    bounds = [0] + list(np.where(np.diff(c) != 0)[0] + 1) + [len(c)]

    below, above = {}, {}
    prev = None
    for i in range(len(bounds) - 1):
        s0, s1 = bounds[i], bounds[i + 1]
        n = s1 - s0
        if n < 10 or (t[s1 - 1] - t[s0]) < MIN_HOLD_S:
            if n >= 10:
                prev = float(c[s0])
            continue
        target = float(c[s0])
        settled = float(np.mean(a[s0 + int(n * SETTLE_FRACTION):s1]))
        if prev is not None and abs(target - prev) > 1e-6:
            key = round(target, 2)
            (below if target > prev else above).setdefault(key, []).append(settled)
        prev = target
    return below, above


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="shoulder_lift")
    ap.add_argument("--dir", default=None)
    ap.add_argument("--load-condition", default="no_load")
    args = ap.parse_args()

    d = pathlib.Path(args.dir) if args.dir else pathlib.Path(__file__).parent / "raw_data"
    paths = sorted(d.glob(f"{args.joint}_staircase*_{args.load_condition}.csv"))
    if not paths:
        raise SystemExit(
            f"no staircase CSVs for {args.joint} ({args.load_condition}) in {d}\n"
            f"Run backlash_probe.py --joint {args.joint} first.")

    below, above = {}, {}
    for p in paths:
        b, a = settled_by_direction(load(p))
        for k, v in b.items():
            below.setdefault(k, []).extend(v)
        for k, v in a.items():
            above.setdefault(k, []).extend(v)

    shared = sorted(set(below) & set(above))
    print("=" * 72)
    print(f"  Backlash vs gravity - {args.joint} ({args.load_condition})")
    print("=" * 72)
    print(f"  {len(paths)} staircase run(s), {len(shared)} rungs visited both ways")
    if not shared:
        raise SystemExit("  no rung was settled from BOTH directions - "
                         "check hold time and rung spacing")

    print()
    print(f"  {'command':>9s}{'from below':>13s}{'from above':>13s}{'gap':>10s}{'n':>7s}")
    print("  " + "-" * 62)
    angles, gaps = [], []
    for k in shared:
        lo = statistics.mean(below[k])
        hi = statistics.mean(above[k])
        gap = lo - hi
        angles.append(k)
        gaps.append(gap)
        print(f"  {k:+9.2f}{lo:13.3f}{hi:13.3f}{gap:+10.3f}"
              f"{len(below[k]) + len(above[k]):7d}")

    angles = np.array(angles)
    gaps = np.array(gaps)

    print()
    print("  " + "-" * 62)
    print(f"  mean gap      {gaps.mean():+.3f} deg")
    print(f"  spread        {gaps.max() - gaps.min():.3f} deg "
          f"(min {gaps.min():+.3f}, max {gaps.max():+.3f})")

    # The GAP isolates backlash. Gravity droop pushes both directions the same
    # way, so it cancels in a difference - verified on synthetic data: a pure
    # 1.2 deg gravity droop produces a gap of 0.016 deg. Gravity therefore has
    # to be read from the MEAN level, not the gap.
    #
    #   gap(angle)  = backlash                  (gravity cancels)
    #   mean(angle) = gravity droop + offset    (backlash cancels)
    #
    # Both are reported, each from the statistic that can actually see it.
    means = np.array([(statistics.mean(below[k]) + statistics.mean(above[k])) / 2 - k
                      for k in shared])

    if len(shared) >= 3:
        # Which trig basis describes the gravity term depends on where the
        # joint's zero sits. Gravity torque on a revolute joint goes as the
        # sine of the angle FROM VERTICAL, equivalently the cosine of the angle
        # from horizontal - so assuming one or the other silently assumes a
        # mounting orientation.
        #
        # Getting this wrong is not a small error. Fitting cos to this arm's
        # shoulder_lift gave R^2 0.056 while reporting a confident 1.164 deg
        # amplitude; sin gives R^2 0.880 and 1.009 deg. The joint's zero is
        # near vertical (the arm sits folded at home), so sin is correct here -
        # but that is a property of this arm's calibration, not a universal
        # fact. Both are fitted and the better one is used, with the R^2 shown
        # so a poor fit cannot masquerade as a measurement.
        ones = np.ones_like(angles, dtype=float)
        rad = np.radians(angles)

        def fit(y, basis):
            A = np.column_stack(basis)
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            pred = A @ coef
            ss_res = float(np.sum((y - pred) ** 2))
            ss_tot = float(np.sum((y - y.mean()) ** 2))
            r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
            return coef, r2

        cands = {
            "sin(angle)": [ones, np.sin(rad)],
            "cos(angle)": [ones, np.cos(rad)],
        }
        fits = {name: fit(means, b) for name, b in cands.items()}
        best = max(fits, key=lambda n: fits[n][1])
        mean_c, mean_r2 = fits[best]

        backlash = abs(gaps.mean())
        gravity = abs(mean_c[1])

        print()
        print("  BACKLASH - from the gap between directions")
        print(f"    mean gap {backlash:.3f} deg, varies {gaps.max() - gaps.min():.3f} "
              f"across the range")
        print(f"    half-width (one-sided slop) {backlash / 2:.3f} deg")
        if gaps.std() > 0.25 * backlash:
            print(f"    NOTE: gap scatter is high (sd {gaps.std():.3f}); the gap is "
                  "less constant")
            print("    than pure backlash would predict - see the per-rung table.")

        print()
        print("  GRAVITY DROOP - from the mean settled error vs angle")
        for name, (c, r2) in sorted(fits.items(), key=lambda kv: -kv[1][1]):
            mark = "  <- used" if name == best else ""
            print(f"    err = {c[0]:+.4f} {c[1]:+.4f} * {name:<11s} R^2 {r2:+.3f}{mark}")
        print(f"    amplitude {gravity:.3f} deg "
              f"(droop varies this much across the swept range)")
        if mean_r2 < 0.5:
            print(f"    WARNING: best R^2 is only {mean_r2:.3f}. The mean error does")
            print("    not follow either gravity shape, so this amplitude is not a")
            print("    reliable gravity measurement.")
        for k, m in zip(shared, means):
            print(f"      cmd {k:+7.2f} deg -> mean settled error {m:+.3f} deg")

        print()
        if backlash < 0.05 and gravity < 0.05:
            print("  VERDICT: neither effect is measurable. The joint tracks well.")
        elif gravity < 0.2 * backlash:
            print(f"  VERDICT: BACKLASH dominates ({backlash:.3f} deg vs "
                  f"{gravity:.3f} deg gravity).")
            print("    -> model a deadband; a gravity feed-forward will not help.")
        elif backlash < 0.2 * gravity:
            print(f"  VERDICT: GRAVITY DROOP dominates ({gravity:.3f} deg vs "
                  f"{backlash:.3f} deg backlash).")
            print("    -> a feed-forward offset proportional to load torque is")
            print("       the right fix.")
        else:
            print("  VERDICT: BOTH are present at comparable size.")
            print(f"    -> backlash {backlash:.3f} deg (deadband model)")
            print(f"    -> gravity  {gravity:.3f} deg (feed-forward)")
            print("       Fix the larger one first.")

    print()
    print("  NOTE: a positive gap means the joint sits HIGHER when it arrived")
    print("  from below. Gravity cancels in the gap and shows only in the mean,")
    print("  which is why the two are reported from different statistics.")


if __name__ == "__main__":
    main()
