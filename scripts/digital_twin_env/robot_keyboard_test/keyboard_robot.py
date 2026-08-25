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
tracking (true hold-to-move) and zero possible collisions with any
viewer-reserved key. Requires `pip install pynput` in the venv actually
running this script (see run_keyboard_robot_container.sh).

Keyboard controls (any window may have focus -- pynput listens system-wide):
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


class KeyboardInput:
    """OS-level key-down/key-up tracking via pynput, independent of any
    window's focus or MuJoCo's viewer. Runs its own listener thread; the
    sim loop reads `.held` (a live set of currently-down key names) once
    per step -- this is what gives true continuous hold-to-move."""

    def __init__(self):
        self.held: set[str] = set()
        self._lock = threading.Lock()
        self.reset_requested = False
        self.quit_requested = False
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

    def _key_name(self, key) -> str | None:
        if isinstance(key, pynput_keyboard.KeyCode) and key.char:
            return key.char.lower()
        return None

    def _on_press(self, key) -> None:
        if key == pynput_keyboard.Key.space:
            self.reset_requested = True
            return
        if key == pynput_keyboard.Key.esc:
            self.quit_requested = True
            return
        name = self._key_name(key)
        if name is not None:
            with self._lock:
                self.held.add(name)

    def _on_release(self, key) -> None:
        name = self._key_name(key)
        if name is not None:
            with self._lock:
                self.held.discard(name)

    def snapshot_held(self) -> set:
        with self._lock:
            return set(self.held)

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
