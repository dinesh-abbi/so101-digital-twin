#!/usr/bin/env python3
"""
Separate backlash from gravity droop - the one thing the Task 1 trajectories
cannot do.

The problem
-----------
Every Task 1 experiment steps UP from 0 and returns DOWN to 0. So command 0
is the only one ever approached from above, and every other command only from
below. Pooling runs shows shoulder_lift settling +1.300 deg when arriving from
above and -0.037 deg from below - a 1.337 deg gap. But "arriving from above"
and "sitting at the lowest, most gravity-loaded pose" are the same set of
samples, so two different explanations fit the data equally well:

    backlash - the joint lands on the other side of the gear slop
    gravity  - the joint droops most where the load torque is largest

They demand different fixes (a deadband model vs a feed-forward term), so
guessing is not acceptable.

The fix
-------
Approach the SAME set of non-zero commands from both directions, and hold long
enough to settle. Then at each command:

    gap(angle) = settled_from_below - settled_from_above

If gap is roughly CONSTANT with angle       -> backlash (mechanical slop)
If gap SCALES with gravity torque (~cos)    -> gravity droop
If both                                     -> the constant is backlash, the
                                               slope is gravity

A staircase does this: walk up through every rung, then back down through the
same rungs. Each intermediate rung is then visited from both sides, holding
still each time.

Safety
------
Identical envelope and guards to so101_joint_characterization.py: one joint
only, every goal clamped to SAFE_MIN..SAFE_MAX, gentle approach to the start
pose, retried writes, torque released on Ctrl-C.
"""

import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from so101_joint_characterization import (  # noqa: E402
    JointBus, move_gently, run_trajectory, SAFE_MIN, SAFE_MAX,
)


def staircase(lo, hi, step, hold):
    """Up through every rung, then back down through the same rungs.

    Every rung except the two endpoints is visited once from below and once
    from above, which is exactly the comparison the Task 1 runs never made.
    """
    rungs = []
    v = lo
    while v <= hi + 1e-9:
        rungs.append(round(v, 3))
        v += step
    wp = [(r, hold) for r in rungs]
    wp += [(r, hold) for r in reversed(rungs[:-1])]
    return wp


def main():
    ap = argparse.ArgumentParser(description="Separate backlash from gravity droop.")
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--id", default="twin_follower")
    ap.add_argument("--joint", default="shoulder_lift")
    ap.add_argument("--load-condition", default="no_load",
                    choices=["no_load", "loaded"])
    # Span matters more than it looks. Separating a constant (backlash) from a
    # cos-shaped (gravity) term needs the cosine to actually vary: over +/-20
    # units cos changes by only 0.06 and the two terms are nearly collinear
    # (condition number 60), so noise decides the split. Over +/-50 units cos
    # changes by 0.45 and the condition number falls to 8.6. Hence the wide
    # default, still 10 units inside the +/-60 safe envelope.
    ap.add_argument("--lo", type=float, default=-50.0,
                    help="Lowest rung, normalised units (default -50)")
    ap.add_argument("--hi", type=float, default=50.0,
                    help="Highest rung (default +50)")
    ap.add_argument("--step", type=float, default=12.5,
                    help="Rung spacing (default 12.5 -> 9 rungs, 7 both ways)")
    ap.add_argument("--hold", type=float, default=1.6,
                    help="Seconds per rung - must be well over the ~65 ms "
                         "latency plus settling (default 1.6)")
    ap.add_argument("--repeats", type=int, default=3,
                    help="Staircase repeats, for averaging (default 3)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    root = pathlib.Path(__file__).parent
    out_dir = pathlib.Path(args.out) if args.out else root / "raw_data"

    cal_path = (pathlib.Path.home() /
                ".cache/huggingface/lerobot/calibration/robots/so_follower" /
                f"{args.id}.json")
    if not cal_path.exists():
        sys.exit(f"calibration not found: {cal_path}")
    cal = json.load(open(cal_path))
    if args.joint not in cal:
        sys.exit(f"joint '{args.joint}' not in calibration. Have: {list(cal)}")

    if args.lo < SAFE_MIN or args.hi > SAFE_MAX:
        sys.exit(f"requested {args.lo}..{args.hi} is outside the safe envelope "
                 f"{SAFE_MIN}..{SAFE_MAX}")

    jc = cal[args.joint]
    bus = JointBus(args.port, jc["id"], jc)

    wp = staircase(args.lo, args.hi, args.step, args.hold)
    rungs = sorted({w[0] for w in wp})
    both_ways = rungs[1:-1]

    print("=" * 66)
    print(f"  Backlash vs gravity probe - {args.joint}")
    print("=" * 66)
    print(f"  port           : {args.port}   motor id {jc['id']}")
    print(f"  load condition : {args.load_condition}")
    print(f"  rungs          : {', '.join(f'{r:+.0f}' for r in rungs)} units")
    print(f"  1 unit         = {bus.norm_to_deg(1.0):.4f} deg")
    print(f"  span           : {bus.norm_to_deg(args.lo):+.1f} .. "
          f"{bus.norm_to_deg(args.hi):+.1f} deg")
    print(f"  visited BOTH ways: {len(both_ways)} rungs "
          f"({', '.join(f'{r:+.0f}' for r in both_ways)})")
    print(f"  {len(wp)} waypoints x {args.repeats} repeats, "
          f"{len(wp) * args.hold * args.repeats:.0f} s total")

    st = bus.read_state()
    if st["ticks"] is None:
        bus.close()
        sys.exit("could not read the joint - is the arm powered?")
    print(f"  current pose   : {bus.ticks_to_norm(st['ticks']):+.1f} units, "
          f"{st['voltage']} V, {st['temperature']} C")
    print()
    print("  The arm will now move. Keep a hand near the 12V connector.")
    print("  Only this one joint is commanded; the others are untouched.")
    input("  Press ENTER to begin, or Ctrl-C to abort... ")
    print()

    try:
        if not bus.set_torque(True):
            bus.close()
            sys.exit("could not enable torque")
        print("  Moving gently to the start pose...")
        move_gently(bus, args.lo)

        for rep in range(args.repeats):
            print(f"\n  [staircase {rep + 1}/{args.repeats}]")
            move_gently(bus, args.lo)
            run_trajectory(
                bus, wp,
                out_dir / f"{args.joint}_staircase{rep + 1}_{args.load_condition}.csv",
                experiment_id=f"staircase{rep + 1}",
                load_condition=args.load_condition,
                joint_name=args.joint,
                hold_s=None,
            )

        print("\n  Returning to the midpoint...")
        move_gently(bus, 0.0)
    except KeyboardInterrupt:
        print("\n\n  Interrupted - releasing torque.")
    finally:
        bus.set_torque(False)
        bus.close()
        print("\n  Torque released, port closed. SUPPORT THE ARM - it is limp.")

    print(f"\n  CSVs written to: {out_dir}")
    print("  Now run:  python analyze_backlash.py --joint " + args.joint)


if __name__ == "__main__":
    main()
