#!/usr/bin/env python3
"""
Extract the five Task 1 numbers from the characterisation CSVs.

    command-to-motion latency, steady-state error, maximum velocity,
    forward/reverse hysteresis, loaded vs unloaded difference

Reads whatever CSVs exist in raw_data/ and reports per load condition, so it
can be run after the no-load set and again after the loaded set.
"""

import argparse
import csv
import pathlib
import statistics

# A step is considered "begun" once the joint has moved this far from where it
# was sitting - large enough to clear encoder noise, small enough to catch the
# true onset of motion.
MOTION_THRESHOLD_DEG = 0.3

# Settling window: the last part of each hold, by which time a well-behaved
# servo has arrived.
SETTLE_FRACTION = 0.6


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def segments(rows):
    """Split a run into per-command segments: (start_idx, end_idx, target)."""
    out = []
    start = 0
    cur = rows[0]["commanded_deg"]
    for i, r in enumerate(rows):
        if r["commanded_deg"] != cur:
            out.append((start, i, float(cur)))
            start, cur = i, r["commanded_deg"]
    out.append((start, len(rows), float(cur)))
    return out


def latency(rows):
    """Time from a new command until the joint measurably moves."""
    vals = []
    for s, e, target in segments(rows):
        if s == 0:
            continue
        t_cmd = float(rows[s]["time"])
        start_pos = float(rows[s]["actual_deg"])
        if abs(target - start_pos) < 1.0:      # no real move requested
            continue
        for j in range(s, e):
            if abs(float(rows[j]["actual_deg"]) - start_pos) > MOTION_THRESHOLD_DEG:
                vals.append((float(rows[j]["time"]) - t_cmd) * 1000)
                break
    return vals


def steady_state_error(rows):
    """How far short (or past) the target the joint settles."""
    vals = []
    for s, e, target in segments(rows):
        n = e - s
        if n < 20:
            continue
        tail = rows[s + int(n * SETTLE_FRACTION):e]
        final = statistics.mean(float(r["actual_deg"]) for r in tail)
        vals.append(final - target)
    return vals


def overshoot(rows):
    """Peak excursion past the target during the approach."""
    vals = []
    for s, e, target in segments(rows):
        if e - s < 20:
            continue
        start_pos = float(rows[s]["actual_deg"])
        if abs(target - start_pos) < 1.0:
            continue
        seg = [float(r["actual_deg"]) for r in rows[s:e]]
        peak = max(seg) if target > start_pos else min(seg)
        ov = (peak - target) if target > start_pos else (target - peak)
        if ov > 0:
            vals.append(ov)
    return vals


def max_velocity(rows):
    """Fastest sustained motion, as the 99th percentile of |velocity|.

    The outright max would be dominated by single-sample encoder jitter, so a
    high percentile is used instead.
    """
    v = sorted(abs(float(r.get("measured_vel_deg_s") or 0)) for r in rows)
    return v[int(len(v) * 0.99)] if v else 0.0


def hysteresis(rows):
    """Backlash: the difference in SETTLED position at the same command,
    depending on which direction the joint arrived from.

    Only settled samples count. Comparing every sample at a given command
    measures travel time, not backlash - during a 30 deg sweep the joint is
    mid-flight at most sample points, which produced a spurious 11 deg
    "hysteresis" on the first analysis pass.
    """
    fwd, rev = {}, {}
    prev_target = None
    for s, e, target in segments(rows):
        n = e - s
        if n < 20 or prev_target is None:
            prev_target = target
            continue
        # Settled position only: the tail of the hold.
        tail = rows[s + int(n * SETTLE_FRACTION):e]
        settled = statistics.mean(float(r["actual_deg"]) for r in tail)
        key = round(target, 1)
        if target > prev_target:
            fwd.setdefault(key, []).append(settled)
        elif target < prev_target:
            rev.setdefault(key, []).append(settled)
        prev_target = target

    gaps = []
    for c in set(fwd) & set(rev):
        gaps.append(abs(statistics.mean(fwd[c]) - statistics.mean(rev[c])))
    return gaps


def summarise(tag, files):
    print(f"\n{'=' * 68}")
    print(f"  {tag}")
    print("=" * 68)

    all_lat, all_sse, all_ov, all_hys = [], [], [], []
    vmax = 0.0

    for path in files:
        rows = load(path)
        if not rows:
            continue
        lat = latency(rows)
        sse = steady_state_error(rows)
        ov = overshoot(rows)
        v = max_velocity(rows)
        vmax = max(vmax, v)

        # The ramp issues a new waypoint every 60-250 ms, which is comparable
        # to the joint's own ~64 ms latency - a command lands while the
        # previous move is still running, so onset times measured there are
        # not clean. Its velocity still counts; its latency and settling do
        # not.
        is_ramp = "ramp" in path.name
        if not is_ramp:
            all_lat += lat
            all_sse += sse
            all_ov += ov

        # Backlash needs settled reversals: the step and reverse runs hold
        # long enough, the ramp does not.
        if "reverse" in path.name or "triangle" in path.name or "step" in path.name:
            all_hys += hysteresis(rows)

        print(f"\n  {path.name}")
        print(f"    samples {len(rows):5d}   duration {float(rows[-1]['time']):6.2f} s")
        if lat:
            print(f"    latency        {statistics.mean(lat):6.1f} ms  "
                  f"(sd {statistics.pstdev(lat):4.1f}, n={len(lat)})")
        if sse:
            print(f"    steady error   {statistics.mean(sse):+6.3f} deg "
                  f"(|mean| {statistics.mean(abs(x) for x in sse):.3f})")
        if ov:
            print(f"    overshoot      {statistics.mean(ov):6.3f} deg  max {max(ov):.3f}")
        print(f"    peak velocity  {v:6.1f} deg/s")

        # Report the median band, not the raw min/max. A handful of corrupt
        # packets read absurd values (one sample in 2280 showed 72 C while
        # 1835 showed 35 C), and a naive max() turns that into a false
        # overheating alarm.
        temps = sorted(float(r["temperature"]) for r in rows
                       if r["temperature"] not in ("", "None"))
        if temps:
            lo = temps[int(len(temps) * 0.02)]
            hi = temps[int(len(temps) * 0.98)]
            spurious = sum(1 for t in temps if t > hi + 5)
            note = f"  ({spurious} corrupt sample{'s' if spurious != 1 else ''} ignored)" if spurious else ""
            print(f"    temperature    {lo:.0f} - {hi:.0f} C{note}")

    print(f"\n  {'-' * 64}")
    print(f"  SUMMARY - {tag}")
    print(f"  {'-' * 64}")
    res = {}
    if all_lat:
        res["latency_ms"] = statistics.mean(all_lat)
        print(f"    command-to-motion latency   {res['latency_ms']:8.1f} ms   "
              f"(sd {statistics.pstdev(all_lat):.1f}, n={len(all_lat)})")
    if all_sse:
        res["sse_deg"] = statistics.mean(abs(x) for x in all_sse)
        print(f"    steady-state error          {res['sse_deg']:8.3f} deg")
    res["vmax"] = vmax
    print(f"    maximum velocity            {vmax:8.1f} deg/s")
    if all_hys:
        res["hys_deg"] = statistics.mean(all_hys)
        print(f"    forward/reverse hysteresis  {res['hys_deg']:8.3f} deg  "
              f"(max {max(all_hys):.3f})")
    if all_ov:
        res["overshoot_deg"] = statistics.mean(all_ov)
        print(f"    mean overshoot              {res['overshoot_deg']:8.3f} deg")
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=None)
    ap.add_argument("--joint", default="elbow_flex",
                    help="Which joint to summarise, or 'all' to loop over each "
                         "joint found (default elbow_flex)")
    args = ap.parse_args()
    d = pathlib.Path(args.dir) if args.dir else pathlib.Path(__file__).parent / "raw_data"

    # Summarise ONE joint at a time. Globbing every CSV pools joints together,
    # which silently averages a light joint with a heavy one - shoulder_lift
    # settles ~10x worse than elbow_flex, so a combined "steady-state error"
    # describes neither.
    # Read the joint from the CSV's own joint_name column rather than parsing
    # the filename - the column is authoritative and survives renaming.
    joints = set()
    for p in d.glob("*.csv"):
        rows = load(p)
        if rows and rows[0].get("joint_name"):
            joints.add(rows[0]["joint_name"])
    joints = sorted(joints)
    if not joints:
        raise SystemExit(f"no CSVs found in {d}")

    if args.joint == "all":
        targets = joints
    elif args.joint in joints:
        targets = [args.joint]
    else:
        raise SystemExit(f"no data for '{args.joint}'. Have: {', '.join(joints)}")

    for ji, joint in enumerate(targets):
        if len(targets) > 1:
            print(f"\n{'#' * 68}\n#  {joint}\n{'#' * 68}")
        run_one(d, joint)


def run_one(d, joint):
    no_load = sorted(d.glob(f"{joint}_*_no_load.csv"))
    loaded = sorted(d.glob(f"{joint}_*_loaded.csv"))

    if not no_load and not loaded:
        print(f"  no CSVs for {joint}")
        return

    a = summarise(f"{joint} - NO LOAD", no_load) if no_load else None
    b = summarise(f"{joint} - LOADED", loaded) if loaded else None

    if a and b:
        print(f"\n{'=' * 68}")
        print("  LOAD COMPARISON - the fifth Task 1 number")
        print("=" * 68)
        print(f"  {'metric':<28}{'no load':>12}{'loaded':>12}{'change':>14}")
        print("  " + "-" * 64)
        for key, label, unit in (
            ("latency_ms", "command-to-motion latency", "ms"),
            ("sse_deg", "steady-state error", "deg"),
            ("vmax", "maximum velocity", "deg/s"),
            ("hys_deg", "hysteresis", "deg"),
            ("overshoot_deg", "overshoot", "deg"),
        ):
            if key in a and key in b:
                delta = b[key] - a[key]
                pct = (delta / a[key] * 100) if a[key] else 0
                print(f"  {label:<28}{a[key]:>12.3f}{b[key]:>12.3f}"
                      f"{delta:>+9.3f} {unit} ({pct:+.0f}%)")
    elif a:
        print("\n  Loaded condition not yet recorded. Re-run with")
        print("  --load-condition loaded to complete the comparison.")


if __name__ == "__main__":
    main()
