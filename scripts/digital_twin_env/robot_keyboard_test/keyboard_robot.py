"""M5: Keyboard -> MuJoCo joint control for the SO-101 on the LINNMON table.

Loads keyboard_robot_scene.xml unmodified (robot XML and table geometry are
never touched here -- see that file's docstring). The robot starts at the
established safe REST_QPOS_RAD pose (reused from
scripts/closed_loop_scripted_pick.py / live_viewer_closed_loop_pick.py).

Design (per the project development plan, section 8/20 rule 4): the
keyboard is only an input device. It produces a normalized six-joint
TARGET COMMAND (JointCommand below), and a separate, input-source-agnostic
apply_command() pushes that target into the simulation. When the real
leader arm (or a replay file, or VR) replaces the keyboard later, only the
code that nudges JointCommand.target needs to change -- JointCommand and
apply_command() stay the same.

Why pynput instead of mujoco.viewer's key_callback:
Two real problems were hit with key_callback: (1) MuJoCo's built-in
Simulate viewer reserves a large set of letters/numbers as global hotkeys
(W=wireframe, S=shadows, T=transparency, and others incl. the number row
for geom-group toggles) -- our callback can't suppress them, so joint keys
kept colliding with viewer toggles no matter which keys were picked; (2)
key_callback only fires on discrete press events with no key-up/repeat
forwarded, so held-key continuous motion never worked. `pynput` listens to
the keyboard at the OS level, independent of which window has focus and
independent of MuJoCo's viewer entirely, giving real key-down/key-up
tracking (true hold-to-move). Requires `pip install pynput` in the venv
actually running this script (see run_keyboard_robot_container.sh).

NOTE: an earlier version of this docstring claimed "zero possible collisions
with any viewer-reserved key". That is not true on Windows. pynput's hook
receives the key, but it does not PREVENT MuJoCo from receiving it too: with
the viewer focused, joint keys move the joints AND fire MuJoCo's own hotkeys
(W wireframe, S shadows, T transparency, the number row for geom groups).
Measured on this machine. Workaround for sim-only work: keep the TERMINAL
focused, not the viewer.

SAFETY -- the global hook and real hardware:
pynput's hook is system-wide, so by default every keystroke on the machine
reaches this script no matter which window has focus. That is harmless in
simulation and is why it was chosen. It is NOT harmless once these keys drive
a real arm: a 'w' typed into a browser or an editor would move real servos.

Construct KeyboardInput(require_focus="<window title substring>") to gate
every key on that window being focused. The gate is checked on press and
again at the point of use, and any held key is dropped the moment focus is
lost, so a joint cannot keep moving after you alt-tab away. Esc is
deliberately left ungated so quitting always works; space (reset to rest
pose) IS gated, since it commands a large motion.

Simulation keeps the ungated default. Anything that drives hardware must pass
require_focus.

Keyboard controls (sim default: any window may have focus -- pynput listens
system-wide; with require_focus set, only the named window):
    Q / A -> shoulder_pan   +/-
    W / S -> shoulder_lift  +/-
    E / D -> elbow_flex     +/-
    R / F -> wrist_flex     +/-
    T / G -> wrist_roll     +/-
    Y / H -> gripper        +/-
    Space -> reset robot to REST_QPOS_RAD
    Esc   -> quit

Holding a key moves the target continuously (one small increment per
simulation step, ~0.1 deg at the default timestep) until released; a quick
tap moves it by one increment.

Usage:
    python keyboard_robot.py
"""

import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from pynput import keyboard as pynput_keyboard

SCENE_PATH = Path(__file__).resolve().parent / "keyboard_robot_scene.xml"

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Established safe rest/home pose, reused from scripts/closed_loop_scripted_pick.py
# and scripts/live_viewer_closed_loop_pick.py (degrees: pan, lift, elbow, wrist_flex,
# wrist_roll, gripper).
REST_QPOS_RAD = np.deg2rad([0.0, -50.0, 48.0, 76.0, 0.0, 0.0])

# Target moves this many radians per SIMULATION STEP while a key is held
# down (continuous, via pynput's real key-down/key-up tracking). Small and
# rate-limited by design (plan section 8: "do not make keyboard presses
# cause dangerous or huge joint jumps"). At the model's 5ms timestep this is
# ~20 deg/s while a key is held.
JOINT_STEP_RAD_PER_SIM_STEP = np.deg2rad(0.1)

# Key -> (joint index, direction) map implementing the Q/A W/S E/D R/F T/G
# Y/H scheme from the plan. Free to use letters again: pynput listens at
# the OS level, not through MuJoCo's viewer, so there is no collision with
# MuJoCo's built-in Simulate hotkeys.
KEY_JOINT_MAP = {
    "q": (0, +1), "a": (0, -1),
    "w": (1, +1), "s": (1, -1),
    "e": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "t": (4, +1), "g": (4, -1),
    "y": (5, +1), "h": (5, -1),
}


class JointCommand:
    """Normalized six-joint target command, independent of input source.

    A future leader-arm/replay-file/VR input source would only need to call
    `nudge()` (or set `.target` directly) from its own event loop -- this
    class and apply_command() below stay the same regardless of where the
    target values come from.
    """

    def __init__(self, joint_ranges: np.ndarray, initial_target: np.ndarray):
        self.joint_ranges = joint_ranges  # shape (6, 2): [lo, hi] per joint
        self.target = initial_target.copy()

    def nudge(self, joint_idx: int, direction: int, step: float) -> bool:
        """Move one joint's target by +/-step, clamped to its limits.
        Returns True if the joint limit was hit (target got clamped)."""
        lo, hi = self.joint_ranges[joint_idx]
        requested = self.target[joint_idx] + direction * step
        clamped = float(np.clip(requested, lo, hi))
        self.target[joint_idx] = clamped
        return clamped != requested

    def reset(self, pose: np.ndarray) -> None:
        self.target[:] = pose


def apply_command(data: mujoco.MjData, command: JointCommand) -> None:
    """Push the current normalized target into the simulation's actuator
    controls. Input-source-agnostic: works the same whether `command.target`
    was produced by keyboard, a leader arm, or a replay file."""
    data.ctrl[:6] = command.target


def _make_focus_gate(title_contains):
    """Return a predicate: is a window whose title matches currently focused?

    Windows only. Returns None if the check is unavailable, so callers can
    decide whether that is acceptable rather than silently running ungated.
    """
    import sys as _sys

    if not _sys.platform.startswith("win"):
        return None
    try:
        import ctypes
    except ImportError:
        return None

    user32 = ctypes.windll.user32
    needle = title_contains.lower()

    def focused():
        hwnd = user32.GetForegroundWindow()
        if not hwnd:
            return False
        n = user32.GetWindowTextLengthW(hwnd)
        if n <= 0:
            return False
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        return needle in buf.value.lower()

    focused()  # fail at construction, not at the first keystroke
    return focused


class KeyboardInput:
    """OS-level key-down/key-up tracking via pynput.

    pynput installs a GLOBAL Win32 hook: it sees every keystroke on the
    machine regardless of which window has focus. That is what gives true
    hold-to-move, and in simulation it is harmless.

    It is NOT harmless once this drives a real arm. Ungated, a 'w' typed into
    a browser, an editor or a chat window moves real servos. Passing
    `require_focus=<substring>` gates every key on a window whose title
    contains that substring being focused.

    The gate applies to BOTH press and release, and the held set is cleared
    whenever focus is lost - otherwise a key held while focus changes stays
    latched down and the joint keeps moving.
    """

    def __init__(self, require_focus=None):
        self.held: set[str] = set()
        # Last time each key was PRESSED. Auto-repeat refreshes this, which is
        # what lets snapshot_held(decay_s=...) treat a repeating key as held.
        self._last_press: dict[str, float] = {}
        self._lock = threading.Lock()
        self.reset_requested = False
        self.quit_requested = False
        self._focused = None
        if require_focus:
            self._focused = _make_focus_gate(require_focus)
            if self._focused is None:
                raise RuntimeError(
                    "require_focus was requested but the window-focus check is "
                    "unavailable on this platform. Refusing to run a global "
                    "keyboard hook ungated while a real arm may be attached."
                )
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

    def _gate_open(self) -> bool:
        """True if keys should be accepted right now."""
        if self._focused is None:
            return True
        if self._focused():
            return True
        with self._lock:
            # Never leave a key latched across focus loss - clear the decay
            # timestamps as well, or a key would stay "active" for decay_s
            # after the gate shut.
            self.held.clear()
            self._last_press.clear()
        return False

    # Special (non-character) keys mapped to the same string names a
    # KEYMAP can use, so 'up'/'down' work as keys alongside 'e'/'d'. Extend
    # this if another script wants a different special key recognised.
    _SPECIAL_KEYS = {
        pynput_keyboard.Key.up: "up",
        pynput_keyboard.Key.down: "down",
        pynput_keyboard.Key.left: "left",
        pynput_keyboard.Key.right: "right",
    }

    def _key_name(self, key) -> str | None:
        if isinstance(key, pynput_keyboard.KeyCode) and key.char:
            return key.char.lower()
        return self._SPECIAL_KEYS.get(key)

    def _on_press(self, key) -> None:
        # Esc is deliberately NOT gated: quitting must work even if the
        # window was closed or focus went somewhere unexpected. Reset (space)
        # IS gated - it commands a large motion to the rest pose.
        if key == pynput_keyboard.Key.esc:
            self.quit_requested = True
            return
        if not self._gate_open():
            return
        if key == pynput_keyboard.Key.space:
            self.reset_requested = True
            return
        name = self._key_name(key)
        if name is not None:
            with self._lock:
                self.held.add(name)
                self._last_press[name] = time.monotonic()

    def _on_release(self, key) -> None:
        # Releases are processed even when the gate is shut: a key pressed
        # while focused and released after focus moved away must still clear,
        # or the joint keeps moving. (_gate_open also clears the whole set on
        # focus loss; this covers the ordering where release arrives first.)
        name = self._key_name(key)
        if name is not None:
            with self._lock:
                self.held.discard(name)

    def snapshot_held(self, decay_s: float = 0.0) -> set:
        """Keys currently down AND allowed to act.

        The gate is checked here as well as on press, so it is enforced at the
        point of use: focus can change between a press and the step that reads
        it, and polling here stops the joint within one step of focus being
        lost rather than whenever the key happens to be released.

        `decay_s` keeps a key active for that long after its last PRESS,
        instead of requiring it to still be in `held`. Holding a key down was
        observed to produce alternating press/release pairs rather than a
        sustained hold - a listener test showed held=['e'], held=[], held=['e']
        on every poll - so a strict "is it in held right now" test sees the key
        for a single tick at a time and continuous motion never happens.
        Auto-repeat refreshes the timestamp, so a genuinely held key stays
        active; a released one lapses after decay_s. Leave at 0 for exact
        key-down semantics (the simulation path relies on that).
        """
        if not self._gate_open():
            return set()
        with self._lock:
            if decay_s <= 0:
                return set(self.held)
            now = time.monotonic()
            return {k for k, t in self._last_press.items()
                    if k in self.held or (now - t) <= decay_s}

    def start(self) -> None:
        self._listener.start()

    def stop(self) -> None:
        self._listener.stop()


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
    assert all(j != -1 for j in joint_ids), "Missing one or more SO-101 joints"
    joint_ranges = np.array([model.jnt_range[j] for j in joint_ids])

    data.qpos[:6] = REST_QPOS_RAD
    data.ctrl[:6] = REST_QPOS_RAD
    mujoco.mj_forward(model, data)

    command = JointCommand(joint_ranges, REST_QPOS_RAD.copy())

    print("M5: Keyboard -> MuJoCo joint control (pynput, OS-level, hold-to-move)")
    print("Controls: Q/A=shoulder_pan  W/S=shoulder_lift  E/D=elbow_flex")
    print("          R/F=wrist_flex    T/G=wrist_roll     Y/H=gripper")
    print("          Space=reset to rest pose   Esc=quit")
    print()
    print("Joint limits (deg):")
    for name, (lo, hi) in zip(JOINT_NAMES, joint_ranges):
        print(f"  {name}: [{np.rad2deg(lo):.1f}, {np.rad2deg(hi):.1f}]")

    kb = KeyboardInput()
    kb.start()

    last_print_time = {name: 0.0 for name in JOINT_NAMES}
    PRINT_INTERVAL_S = 0.2  # throttle terminal spam while a key is held

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                step_start = time.time()

                if kb.quit_requested:
                    break

                if kb.reset_requested:
                    command.reset(REST_QPOS_RAD.copy())
                    data.qpos[:6] = REST_QPOS_RAD
                    mujoco.mj_forward(model, data)
                    print("RESET to rest pose:", np.rad2deg(REST_QPOS_RAD).round(1))
                    kb.reset_requested = False

                held = kb.snapshot_held()
                for key_name, (joint_idx, direction) in KEY_JOINT_MAP.items():
                    if key_name in held:
                        hit_limit = command.nudge(joint_idx, direction, JOINT_STEP_RAD_PER_SIM_STEP)
                        now = time.time()
                        if now - last_print_time[JOINT_NAMES[joint_idx]] >= PRINT_INTERVAL_S:
                            limit_note = "  (LIMIT REACHED)" if hit_limit else ""
                            current_deg = np.rad2deg(command.target[joint_idx])
                            print(f"{JOINT_NAMES[joint_idx]} target = {current_deg:.2f} deg{limit_note}")
                            last_print_time[JOINT_NAMES[joint_idx]] = now

                apply_command(data, command)
                mujoco.mj_step(model, data)
                viewer.sync()

                dt = model.opt.timestep - (time.time() - step_start)
                if dt > 0:
                    time.sleep(dt)
    finally:
        kb.stop()


if __name__ == "__main__":
    main()
