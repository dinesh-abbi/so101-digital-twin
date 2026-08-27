#!/usr/bin/env python3
"""
Show exactly what the keyboard listener sees. No robot, nothing moves.

Run this when a key "does not work" in m6_keyboard_real.py: it prints every
press and release, the resulting held-set, and whether the focus gate is open,
so the difference between "the key never arrived", "the key arrived but the
gate was shut" and "the key arrived and something else overrode it" is visible
rather than guessed at.

    python key_test.py

Press E, D, Space in turn. Esc quits.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "digital_twin_env" / "robot_keyboard_test"))

KEYMAP = {
    "q": "shoulder_pan +", "a": "shoulder_pan -",
    "w": "shoulder_lift +", "s": "shoulder_lift -",
    "e": "elbow_flex +", "d": "elbow_flex -",
    "r": "wrist_flex +", "f": "wrist_flex -",
    "t": "wrist_roll +", "g": "wrist_roll -",
    "y": "gripper +", "h": "gripper -",
}


def main():
    from keyboard_robot import KeyboardInput, _make_focus_gate

    title = None
    try:
        import ctypes
        u = ctypes.windll.user32
        h = u.GetForegroundWindow()
        n = u.GetWindowTextLengthW(h)
        b = ctypes.create_unicode_buffer(n + 1)
        u.GetWindowTextW(h, b, n + 1)
        title = b.value.strip()[:24]
    except Exception:
        pass

    print("=" * 68)
    print("  Keyboard listener test - NOTHING MOVES, no robot involved")
    print("=" * 68)
    print(f"  focus gate: title contains {title!r}")
    print()
    print("  Press E, then D, then Space. Watch what is reported.")
    print("  Esc quits.")
    print()
    print("  NOTE: keys also reach whatever has focus, so this terminal will")
    print("  fill with 'eeee'/'dddd'. That is expected and harmless here -")
    print("  it is also why a real run should NOT be driven from a shell")
    print("  prompt that is accepting input.")
    print()

    keys = KeyboardInput(require_focus=title)
    keys.start()

    last = None
    last_reset = False
    try:
        while not keys.quit_requested:
            gate = keys._gate_open()
            held = keys.snapshot_held()
            mapped = {k: KEYMAP[k] for k in sorted(held) if k in KEYMAP}
            state = (tuple(sorted(held)), gate, keys.reset_requested)
            if state != last:
                bits = []
                bits.append("GATE OPEN " if gate else "GATE SHUT ")
                bits.append(f"held={sorted(held) or '[]'}")
                if mapped:
                    bits.append("-> " + ", ".join(mapped.values()))
                if keys.reset_requested:
                    bits.append("*** SPACE/RESET REQUESTED ***")
                print("  " + "  ".join(bits))
                last = state
            if keys.reset_requested and not last_reset:
                keys.reset_requested = False
            last_reset = keys.reset_requested
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        print("\n  done")


if __name__ == "__main__":
    main()
