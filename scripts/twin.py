#!/usr/bin/env python3
"""
twin.py - single entry point for every script in this project.

Instead of remembering which of a dozen files to run, from which directory,
with which flags: `python scripts/twin.py <command> [flags...]`. Run with no
arguments (or -h) for the full list.

This is a DISPATCHER, not a rewrite. Each command below launches the
existing, already-validated script as a subprocess with your extra flags
forwarded verbatim (everything after the command name goes straight to
argparse in that script) - so `--help` on any command shows that script's
own real options, not a guess, and nothing about how M6/M7/characterization
actually work has changed.

No arguments at all opens an INTERACTIVE MENU: pick a command by number,
answer a couple of prompts for its most common flags (joints/port/id/...),
Enter accepts the default shown in [brackets]. Nothing runs until you
confirm the assembled command line.

Examples (non-interactive / scriptable)
----------------------------------------
    python scripts/twin.py sim                                  # M5: keyboard -> sim only
    python scripts/twin.py real --joints elbow_flex              # M6: keyboard -> real arm only
    python scripts/twin.py real --joints all --recover           # M6 recovery
    python scripts/twin.py mirror --source sim --joints all      # M7 step 3: no hardware
    python scripts/twin.py mirror --source real --joints all --record recordings/session.csv
    python scripts/twin.py check-pose                            # read-only pose check
    python scripts/twin.py home --joints shoulder_lift            # gentle move to safe pose
    python scripts/twin.py calibrate --port COM9 --id twin_follower
    python scripts/twin.py validate                              # headless scene load-check
    python scripts/twin.py characterize --joint elbow_flex --experiment step
    python scripts/twin.py replay --joint elbow_flex
    python scripts/twin.py tune --joint elbow_flex

Every command accepts `--` before its own flags if a flag name would
otherwise be swallowed by twin.py itself (none currently are, but this is
future-proofing for argparse ambiguity): e.g.
    python scripts/twin.py real -- --joints all
"""

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable  # the interpreter running twin.py IS the venv - reuse it

DTE = HERE / "digital_twin_env"
CHAR = HERE / "characterization"


def run(script, args, cwd=None):
    """Launch `script` with `args` forwarded, using this same interpreter.

    A subprocess (not an in-process import + main()) so that each script's
    own sys.path insertion, argparse --help, and sys.exit() calls behave
    exactly as they do when run directly - twin.py adds a name, not a
    reimplementation.
    """
    cmd = [PY, str(script), *args]
    result = subprocess.run(cmd, cwd=str(cwd) if cwd else None)
    sys.exit(result.returncode)


# command -> (script path, help text, default cwd or None for HERE's parent)
COMMANDS = {
    # --- Sim-only, no hardware -------------------------------------------
    "table": (DTE / "run_table.py",
              "M1: view the LINNMON/ADILS table alone."),
    "cube": (DTE / "spawn_cube_test" / "spawn_cube.py",
             "M2: runtime cube spawn + physics drop."),
    "robot-on-table": (DTE / "robot_on_table_test" / "run_robot_on_table.py",
                       "M3: SO-101 mounted on the table, static viewer."),
    "sim": (DTE / "robot_keyboard_test" / "keyboard_robot.py",
            "M5: keyboard -> simulated SO-101 joints only. No hardware."),

    # --- Real hardware -----------------------------------------------------
    "real": (HERE / "m6_keyboard_real.py",
             "M6: keyboard -> REAL SO-101 follower arm only."),
    "mirror": (HERE / "m7_mirror_sim.py",
               "M7: keyboard -> real arm + mirrored sim together. "
               "--source sim|real (default sim = no hardware)."),

    # --- Hardware setup / recovery / diagnostics ----------------------------
    "calibrate": (None, "Run LeRobot's calibration wizard for this arm "
                        "(wraps lerobot-calibrate --robot.type=so101_follower)."),
    "check-pose": (HERE / "check_pose.py",
                  "Read-only: print current pose vs the +/-50 safe envelope."),
    "home": (HERE / "goto_home.py",
             "Gently move the arm to a safe pose under power (torque released "
             "on exit unless --hold). Prefer 'real --recover' if you're about "
             "to drive the arm right after."),
    "key-test": (HERE / "key_test.py",
                "Diagnostic: print every keypress/focus-gate state. "
                "No robot, nothing moves."),

    # --- Validation ----------------------------------------------------------
    "validate": (HERE / "validate_scenes.py",
                "Headless load-check for all four MuJoCo scenes."),
    "map-check": (DTE / "real_sim_mapping_test" / "validate_real_sim_mapping.py",
                 "Unit-test the real<->sim joint mapping math, headless."),

    # --- Characterization pipeline (Task 1 / Task 2) -------------------------
    "characterize": (CHAR / "so101_joint_characterization.py",
                     "Task 1: command a real joint through trajectories, "
                     "log the response. Needs hardware."),
    "analyze": (CHAR / "analyze_joint_response.py",
               "Analyze characterize's CSVs: latency, hysteresis, load effect."),
    "backlash": (CHAR / "backlash_probe.py",
                "Staircase probe to separate backlash from gravity droop. "
                "Needs hardware."),
    "backlash-analyze": (CHAR / "analyze_backlash.py",
                         "Analyze backlash probe CSVs."),
    "tune": (CHAR / "tune_actuator.py",
            "Task 2: fit sim actuator kp/kv against real step responses."),
    "replay": (CHAR / "replay_in_sim.py",
              "Replay recorded real trajectories in MuJoCo, report RMSE."),
    "plots": (CHAR / "make_plots.py",
             "Render the characterization plots (step_response.png, etc)."),
}

# Grouped for the help listing AND the interactive menu - COMMANDS above is
# the source of truth for what a name maps to; this is just display order.
GROUPS = [
    ("Sim only (no hardware)", ["table", "cube", "robot-on-table", "sim"]),
    ("Real hardware", ["real", "mirror"]),
    ("Setup / recovery / diagnostics",
     ["calibrate", "check-pose", "home", "key-test"]),
    ("Validation", ["validate", "map-check"]),
    ("Characterization (Task 1 / Task 2)",
     ["characterize", "analyze", "backlash", "backlash-analyze",
      "tune", "replay", "plots"]),
]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll", "gripper"]

# Interactive-menu prompts per command: each is (flag, question, default).
# Deliberately only the flag or two someone actually changes call to call -
# everything else keeps that script's own default, and `--help` (also on
# the menu, per command) shows the rest for scripting later.
PROMPTS = {
    "real": [
        ("--joints", "Which joint(s) live? (comma list, or 'all')",
         "elbow_flex"),
        ("--recover", "Recover out-of-range joints first before handing "
                      "over to the keyboard? (y/N)", "n"),
    ],
    "mirror": [
        ("--source", "Source: 'sim' (no hardware, bring-up) or 'real'?",
         "sim"),
        ("--joints", "Which joint(s) live? (comma list, or 'all')",
         "elbow_flex"),
        ("--record", "Record CSV path (blank = don't record)", ""),
    ],
    "home": [
        ("--joints", "Which joint(s)? (comma list, or 'all')", "all"),
        ("--hold", "Keep torque ON at the end instead of releasing? (y/N)",
         "n"),
    ],
    "characterize": [
        ("--joint", "Which joint?", "elbow_flex"),
        ("--experiment", "Which experiment ('all', 'step', 'ramp', "
                         "'reverse', 'triangle', 'staircase')", "all"),
        ("--load-condition", "Load condition ('no_load' or 'loaded')",
         "no_load"),
    ],
    "backlash": [
        ("--joint", "Which joint?", "shoulder_lift"),
        ("--load-condition", "Load condition ('no_load' or 'loaded')",
         "no_load"),
    ],
    "tune": [("--joint", "Which joint?", "elbow_flex")],
    "replay": [("--joint", "Which joint?", "elbow_flex")],
    "analyze": [("--joint", "Which joint?", "elbow_flex")],
    "backlash-analyze": [
        ("--joint", "Which joint?", "shoulder_lift"),
        ("--load-condition", "Load condition ('no_load' or 'loaded')",
         "no_load"),
    ],
    "calibrate": [
        ("--robot.port", "Serial port", "COM9"),
        ("--robot.id", "Calibration id (arm name)", "twin_follower"),
    ],
    "check-pose": [
        ("--port", "Serial port", "COM9"),
        ("--id", "Calibration id (arm name)", "twin_follower"),
    ],
}


def print_help():
    print(__doc__)
    print("Commands")
    print("=" * 70)
    for group_name, names in GROUPS:
        print(f"\n{group_name}:")
        for name in names:
            _, help_text = COMMANDS[name]
            print(f"  {name:18s} {help_text}")
    print("\nRun `python scripts/twin.py <command> --help` for that "
          "command's real options.")


def cmd_calibrate(args):
    """Wraps lerobot-calibrate. Kept separate from COMMANDS' subprocess-of-a-
    .py-file shape because this launches an installed console script, not a
    file in this repo, and needs its own default flags assembled."""
    import shutil
    exe = shutil.which("lerobot-calibrate")
    if exe is None:
        # shutil.which() only searches the PATH env var, which does not
        # include a venv's own Scripts/ folder unless the venv was
        # "activated" in this shell. Every command here is instead invoked
        # via the venv's python.exe directly (an absolute path, per this
        # project's own convention - see CLAUDE.md), so the console script
        # sits right next to sys.executable even though PATH never mentions
        # it. Check there before giving up.
        candidate = Path(sys.executable).parent / "lerobot-calibrate.exe"
        if candidate.exists():
            exe = str(candidate)
    if exe is None:
        sys.exit(
            "lerobot-calibrate not found on PATH.\n"
            "M6/M7 dependencies are not installed in this venv yet - run:\n"
            "    .\\setup_m6.ps1\n"
            "first (adds LeRobot to THIS project's .venv; never touches "
            "the so101-vr project's venv)."
        )

    # Defaults match every other real-hardware command in this project
    # (m6_keyboard_real.py, m7_mirror_sim.py, check_pose.py, goto_home.py),
    # so a bare `twin.py calibrate` calibrates the same arm those commands
    # default to. Anything the user passes after `calibrate` overrides these.
    port = "COM9"
    robot_id = "twin_follower"
    passthrough = list(args)
    if not any(a.startswith("--robot.port") for a in passthrough):
        passthrough = [f"--robot.port={port}", *passthrough]
    if not any(a.startswith("--robot.id") for a in passthrough):
        passthrough = [f"--robot.id={robot_id}", *passthrough]
    if not any(a.startswith("--robot.type") for a in passthrough):
        passthrough = ["--robot.type=so101_follower", *passthrough]

    print("=" * 70)
    print("  LeRobot calibration wizard")
    print("=" * 70)
    print("  Sweep EVERY joint fully to both physical extremes, including")
    print("  the gripper fully shut and fully open - a sweep that misses a")
    print("  true extreme reads a wrong percentage at that end for the rest")
    print("  of the session (see README, this bit the gripper once).")
    print()
    result = subprocess.run([exe, *passthrough])
    sys.exit(result.returncode)


def dispatch(name, rest):
    """Run one command by name with `rest` as its already-built arg list.
    Shared by both the direct CLI path and the interactive menu, so the two
    never drift into behaving differently for the same command."""
    if name == "calibrate":
        cmd_calibrate(rest)
        return

    entry = COMMANDS.get(name)
    if entry is None:
        import difflib
        close = difflib.get_close_matches(name, COMMANDS, n=3)
        hint = f" Did you mean: {', '.join(close)}?" if close else ""
        sys.exit(f"unknown command '{name}'.{hint}\n"
                 f"Run `python scripts/twin.py --help` for the full list.")

    script, _ = entry
    if not script.exists():
        sys.exit(f"'{name}' maps to {script}, which does not exist on this "
                 f"machine. See CLAUDE.md for what's known missing.")
    run(script, rest)


def _prompt(question, default):
    suffix = f" [{default}]" if default != "" else " [none]"
    try:
        answer = input(f"    {question}{suffix}: ").strip()
    except EOFError:
        answer = ""
    return answer if answer else default


def _yesno_to_flag(flag, answer):
    """A (y/N)-style prompt answer -> either [flag] or [] for a store_true
    flag. '--recover'/'--hold' are boolean switches in their scripts, so an
    empty arg here (not '--recover=n') is what argparse actually expects."""
    return [flag] if answer.strip().lower() in ("y", "yes") else []


def interactive_menu():
    print("=" * 70)
    print("  so101-digital-twin - interactive launcher")
    print("=" * 70)

    numbered = []
    for group_name, names in GROUPS:
        print(f"\n{group_name}:")
        for name in names:
            numbered.append(name)
            _, help_text = COMMANDS[name]
            print(f"  {len(numbered):2d}) {name:18s} {help_text}")

    print()
    choice = _prompt("Pick a number (or command name), 'q' to quit", "")
    if not choice or choice.lower() in ("q", "quit", "exit"):
        sys.exit(0)

    if choice.isdigit() and 1 <= int(choice) <= len(numbered):
        name = numbered[int(choice) - 1]
    elif choice in COMMANDS:
        name = choice
    else:
        sys.exit(f"not a valid choice: '{choice}'")

    args = []
    for flag, question, default in PROMPTS.get(name, []):
        answer = _prompt(question, default)
        if flag in ("--recover", "--hold"):
            args += _yesno_to_flag(flag, answer)
        elif answer:
            args += [flag, answer]

    print()
    display_script = "lerobot-calibrate" if name == "calibrate" else \
        COMMANDS[name][0].name
    print(f"  -> {display_script} {' '.join(args)}".rstrip())
    confirm = _prompt("Run this now? (Y/n)", "y")
    if confirm.strip().lower() not in ("y", "yes"):
        print("  cancelled.")
        sys.exit(0)
    print()

    dispatch(name, args)


def main():
    argv = sys.argv[1:]
    if not argv:
        interactive_menu()
        return
    if argv[0] in ("-h", "--help"):
        print_help()
        sys.exit(0)

    name, rest = argv[0], argv[1:]
    if rest and rest[0] == "--":
        rest = rest[1:]
    dispatch(name, rest)


if __name__ == "__main__":
    main()
