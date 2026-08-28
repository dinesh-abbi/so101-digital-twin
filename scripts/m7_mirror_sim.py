#!/usr/bin/env python3
"""
M7 - keyboard -> REAL SO-101 follower + MuJoCo sim, mirrored.

M5 drove the sim. M6 drove the real arm. This drives both, and shows the
gap between them.

WHICH ARM IS AUTHORITATIVE
--------------------------
The sim mirrors the real arm's MEASURED position, not the commanded target.

That is the whole point of a digital twin here. Task 1 measured backlash at
1.955 deg and gravity droop at 1.009 deg on shoulder_lift; Task 2 fitted the
sim's actuator gains (kp=400 kv=25) against real step responses. If the sim
chased the same target as the real arm, both would look correct and every
one of those measured errors would be invisible. Mirroring the measured
position instead makes the tracking error the thing you actually see.

So: keyboard -> real arm target -> servos move -> read back what they DID ->
that becomes the sim's pose. The sim is a display of reality, not a
parallel simulation of it.

--source sim flips this for step 3 of the bring-up (see below): no hardware,
the sim mirrors the commanded target so the wiring can be proven with
nothing that can break.

BRING-UP ORDER
--------------
Step 3 (no hardware, do this first):

    python scripts/m7_mirror_sim.py --source sim --joints elbow_flex

  Keyboard drives a target, the mapping converts it to radians, the sim
  shows it. The real arm is never opened. Proves the mapping and the
  viewer without risk.

Step 4 (hardware):

    python scripts/m7_mirror_sim.py --source real --joints elbow_flex

  Same thing, but the target goes to the servos and the sim mirrors what
  they measured.

THE ZERO CHECK THIS ENABLES
---------------------------
real_sim_joint_mapping.py maps real -100..100 onto the sim's radian range
LINEARLY. Equal FRACTIONS of travel map together, so the two arms match in
shape and direction -- but the real calibrated spans and the sim kinematic
spans are not equal (wrist_roll: 359.9 deg real vs 314.4 deg sim), so a
given real angle and its sim angle differ in the middle of the range.

That is by design, not a bug, but it means "the sim looks like the arm" has
to be confirmed by eye once, side by side. This script prints both values
per joint every tick so that check is just reading the terminal.

SAFETY
------
Inherited from M6 unchanged, and it all still applies: focus gate, LeRobot's
max_relative_target clamp, the +/-SAFE_LIMIT envelope, and one joint live at
a time by default. The arm goes LIMP when this exits. Support it.

Out-of-range joints are refused the same way M6 refuses them; recover with
M6 first, which keeps torque on throughout:

    python scripts/m6_keyboard_real.py --joints <joint> --recover
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE / "digital_twin_env" / "robot_keyboard_test"))
sys.path.insert(0, str(_HERE / "digital_twin_env" / "real_sim_mapping_test"))

import mujoco                                    # noqa: E402
import mujoco.viewer                             # noqa: E402

from real_sim_joint_mapping import (             # noqa: E402
    JOINT_NAMES,
    load_sim_joint_ranges_rad,
    real_to_sim_vector,
)

# The scene that resolves meshdir correctly. so101.xml alone cannot find its
# meshes; scene.xml only works when cwd happens to be right. Same scene
# replay_in_sim.py uses, for the same reason.
SCENE_PATH = (_HERE / "digital_twin_env" / "robot_keyboard_test"
              / "keyboard_robot_scene.xml")

# --- Constants carried over from M6, unchanged. See that file for the
# --- reasoning behind each; they were tuned against this hardware.
SAFE_LIMIT = 50.0
MAX_RELATIVE_TARGET = 4.0
CONTROL_HZ = 20.0
STEP_PER_TICK = 0.25
KEY_DECAY_S = 0.6

KEYMAP = {
    "q": ("shoulder_pan", +1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "up": ("elbow_flex", +1), "down": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
    "y": ("gripper", +1), "h": ("gripper", -1),
}

# Where a joint sits when the real arm is not connected (--source sim).
# Normalised units: 0 is mid-range for the arm joints, the gripper is 0..100.
SIM_ONLY_START = {name: 0.0 for name in JOINT_NAMES}
SIM_ONLY_START["gripper"] = 50.0


def parse_args():
    ap = argparse.ArgumentParser(
        description="M7: keyboard -> real SO-101 + MuJoCo, mirrored.")
    ap.add_argument("--source", choices=("real", "sim"), default="sim",
                    help="'sim': no hardware, sim mirrors the commanded "
                         "target (bring-up step 3, the safe default). "
                         "'real': drive the arm, sim mirrors its MEASURED "
                         "position (step 4).")
    ap.add_argument("--joints", default="elbow_flex",
                    help="Comma-separated live joints, or 'all'. "
                         "Default: elbow_flex only.")
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--id", default="twin_follower")
    ap.add_argument("--require-focus", default=None,
                    help="Keys act only while the focused window title "
                         "contains this. Real hardware only. Default: "
                         "auto-detect whichever window is focused at "
                         "startup (same as M6).")
    ap.add_argument("--step", type=float, default=STEP_PER_TICK)
    ap.add_argument("--safe-limit", type=float, default=SAFE_LIMIT)
    ap.add_argument("--max-relative-target", type=float,
                    default=MAX_RELATIVE_TARGET)
    ap.add_argument("--record", default=None,
                    help="Log every control tick to this CSV: tick, then "
                         "per live joint target/real/sim_deg (real is only "
                         "present with --source real). Recording is a plain "
                         "log, not a LeRobot Dataset -- no extra dependency, "
                         "just a comparison trace of target vs measured vs "
                         "mirrored sim over time.")
    return ap.parse_args()


def detect_focus_title():
    """The title of whatever window is focused right now.

    Copied from m6_keyboard_real.py rather than imported: hardcoding a
    window title (previously "VR-SO-101 - Antigravity ", left over from a
    different project's window name) silently ate every keystroke, since
    that substring never appeared in this project's actual window title.
    Detecting beats guessing -- see M6's version of this function for the
    IDE-vs-terminal story.
    """
    if not sys.platform.startswith("win"):
        return None
    try:
        import ctypes
    except ImportError:
        return None
    user32 = ctypes.windll.user32
    hwnd = user32.GetForegroundWindow()
    if not hwnd:
        return None
    n = user32.GetWindowTextLengthW(hwnd)
    if n <= 0:
        return None
    buf = ctypes.create_unicode_buffer(n + 1)
    user32.GetWindowTextW(hwnd, buf, n + 1)
    title = buf.value.strip()
    return title[:24] if title else None


def resolve_live(spec):
    if spec.strip().lower() == "all":
        return list(JOINT_NAMES)
    live = [j.strip() for j in spec.split(",") if j.strip()]
    unknown = [j for j in live if j not in JOINT_NAMES]
    if unknown:
        sys.exit(f"unknown joint(s): {', '.join(unknown)}\n"
                 f"valid: {', '.join(JOINT_NAMES)}")
    return live


def main():
    args = parse_args()
    live = resolve_live(args.joints)

    if args.source == "real" and args.require_focus is None:
        args.require_focus = detect_focus_title()
        if not args.require_focus:
            sys.exit("could not detect the focused window title.\n"
                     "Pass --require-focus '<part of your terminal title>' "
                     "explicitly.")

    print("=" * 70)
    print(f"  M7 - keyboard -> {'REAL + SIM (mirrored)' if args.source == 'real' else 'SIM ONLY (no hardware)'}")
    print("=" * 70)

    # The mapping's sim ranges come from the compiled model, never hardcoded,
    # so a so101.xml change is picked up automatically.
    sim_ranges = load_sim_joint_ranges_rad(str(SCENE_PATH))

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    robot = None
    keys = None

    try:
        # ---- Source setup -------------------------------------------------
        if args.source == "real":
            from lerobot.robots.so_follower import (
                SOFollower, SOFollowerRobotConfig)

            robot = SOFollower(SOFollowerRobotConfig(
                port=args.port, id=args.id,
                max_relative_target=args.max_relative_target))
            print(f"  port                 : {args.port}")
            print(f"  calibration id       : {args.id}")
            print(f"  LIVE joints          : {', '.join(live)}")
            print(f"  safe envelope        : +/-{args.safe_limit:.0f} units")
            print("\n  Connecting...")
            robot.connect(calibrate=False)
            print("  Connected.")

            obs = robot.get_observation()
            start = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}
        else:
            print("  No hardware will be opened.")
            print(f"  LIVE joints          : {', '.join(live)}")
            start = dict(SIM_ONLY_START)

        print("\n  Starting pose (normalised units):")
        outside = []
        for j in JOINT_NAMES:
            flag = "  <- LIVE" if j in live else ""
            warn = ""
            # The gripper is 0..100, so the +/-SAFE_LIMIT test does not
            # apply to it the way it does to the arm joints.
            if j != "gripper" and abs(start[j]) > args.safe_limit:
                warn = f"   *** OUTSIDE +/-{args.safe_limit:.0f} ***"
                outside.append(j)
            print(f"    {j:15s} {start[j]:+7.1f}{flag}{warn}")

        if outside and args.source == "real":
            print(f"\n  {', '.join(outside)} start outside the safe envelope.")
            print("  Commanding from here would clamp hard. Recover first --")
            print("  M6 does it without ever releasing torque:")
            print(f"    python scripts/m6_keyboard_real.py "
                  f"--joints {args.joints} --recover")
            return

        target = dict(start)

        # ---- Seed the sim at the starting pose ----------------------------
        qpos = real_to_sim_vector(target, sim_ranges)
        data.qpos[:6] = qpos
        data.ctrl[:6] = qpos
        mujoco.mj_forward(model, data)

        # ---- Keyboard -----------------------------------------------------
        from keyboard_robot import KeyboardInput

        if args.source == "real":
            # Real hardware: the focus gate is mandatory, exactly as in M6.
            keys = KeyboardInput(require_focus=args.require_focus)
        else:
            # Sim only: nothing can be damaged, and requiring a terminal
            # title makes the viewer awkward to use.
            keys = KeyboardInput()
        keys.start()

        print("\n  " + "-" * 66)
        print("  Q/A pan   W/S lift   E/D elbow (or Up/Down)   R/F wristflex   T/G roll   Y/H grip")
        print("  Esc = quit")
        if args.source == "real":
            print(f"  Keys act ONLY while a '{args.require_focus}' window is focused.")
            print("  Ready. Keep a hand near the 12V connector.")
        print("  " + "-" * 66 + "\n")

        period = 1.0 / CONTROL_HZ
        last_print = 0.0

        record_file = None
        record_writer = None
        if args.record:
            import csv
            record_path = Path(args.record)
            record_file = open(record_path, "w", newline="")
            record_writer = csv.writer(record_file)
            header = ["tick"]
            for j in live:
                header += [f"{j}_target", f"{j}_real", f"{j}_sim_deg"]
            record_writer.writerow(header)
            print(f"  --record: logging to {record_path}")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                tick = time.perf_counter()

                if getattr(keys, "quit_requested", False):
                    break

                # ---- Read the real arm BEFORE commanding it, so the sim
                # ---- shows what the servos actually did last tick.
                live_obs = None
                if args.source == "real":
                    obs = robot.get_observation()
                    live_obs = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES}

                # ---- Accumulate the keyboard onto the target --------------
                for name in keys.snapshot_held(KEY_DECAY_S):
                    m = KEYMAP.get(name)
                    if not m:
                        continue
                    joint, direction = m
                    if joint not in live:
                        continue
                    v = target[joint] + direction * args.step
                    if joint == "gripper":
                        target[joint] = max(0.0, min(100.0, v))
                    else:
                        target[joint] = max(-args.safe_limit,
                                            min(args.safe_limit, v))

                # ---- Leash the target to the arm's real position ----------
                # Without this a held key walks the target away faster than
                # the joint can follow, every command gets clamped inside
                # LeRobot, and releasing the key leaves a large queued
                # motion the arm then executes on its own. M6's reasoning,
                # unchanged.
                if live_obs is not None:
                    leash = args.max_relative_target * 0.9
                    for j in live:
                        now = live_obs[j]
                        target[j] = max(now - leash, min(now + leash, target[j]))

                    robot.send_action({f"{j}.pos": target[j]
                                       for j in JOINT_NAMES})

                # ---- Drive the sim ---------------------------------------
                # real: mirror what the servos MEASURED (the twin).
                # sim:  mirror the commanded target (no hardware to measure).
                mirror = live_obs if live_obs is not None else target
                data.ctrl[:6] = real_to_sim_vector(mirror, sim_ranges)
                mujoco.mj_step(model, data)
                viewer.sync()

                if record_writer is not None:
                    row = [f"{tick:.3f}"]
                    for j in live:
                        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        sim_deg = float(np.rad2deg(data.qpos[model.jnt_qposadr[jid]]))
                        real_val = live_obs[j] if live_obs is not None else ""
                        row += [f"{target[j]:.3f}",
                               f"{real_val:.3f}" if real_val != "" else "",
                               f"{sim_deg:.3f}"]
                    record_writer.writerow(row)

                # ---- Report ----------------------------------------------
                if tick - last_print > 0.5:
                    cells = []
                    for j in live:
                        jid = mujoco.mj_name2id(
                            model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        sim_deg = np.rad2deg(data.qpos[model.jnt_qposadr[jid]])
                        if live_obs is not None:
                            cells.append(f"{j[:9]} cmd{target[j]:+6.1f} "
                                         f"real{live_obs[j]:+6.1f} "
                                         f"sim{sim_deg:+6.1f}d")
                        else:
                            cells.append(f"{j[:9]} cmd{target[j]:+6.1f} "
                                         f"sim{sim_deg:+6.1f}d")
                    print("\r  " + " | ".join(cells) + "   ",
                          end="", flush=True)
                    last_print = tick

                rem = period - (time.perf_counter() - tick)
                if rem > 0:
                    time.sleep(rem)

        print("\n\n  Stopping.")

    except KeyboardInterrupt:
        print("\n\n  Interrupted.")
    finally:
        if keys is not None:
            try:
                keys.stop()
            except Exception:
                pass
        if record_file is not None:
            record_file.close()
            print(f"  record log written: {args.record}")
        if robot is not None:
            robot.disconnect()
            print("  Disconnected. THE ARM IS LIMP - SUPPORT IT.")


if __name__ == "__main__":
    main()
