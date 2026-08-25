"""Runtime cube-spawning experiment on the LINNMON/ADILS table.

Demonstrates that a MuJoCo object's runtime pose can be changed many times
(via qpos/qvel edits + mj_forward) without ever touching or recompiling the
XML. The model is compiled once at startup; spawn_cube() only mutates
simulation state.

Keyboard controls (viewer window must have focus):
    R   Random spawn above the table (new XY each press), z=1.0, falls.
    D   Fixed "drop test" spawn at x=0.10, y=0.05, z=1.00, falls.
    C   Spawn above table center (x=0, y=0, z=0.80), falls and settles.
    T   Reset simulation to its initial state (mj_resetData).

Usage:
    python spawn_cube.py
"""

import random
import time
from pathlib import Path

import glfw
import mujoco
import mujoco.viewer

SCENE_PATH = Path(__file__).resolve().parent / "spawn_cube_scene.xml"

# Tabletop top surface Z (from table_scene.xml / spawn_cube_scene.xml).
TABLETOP_TOP_Z = 0.74

# Usable spawn region for random drops, inset from the tabletop edges
# (tabletop is 1.00 x 0.60 m, cube is 0.05 m wide).
RANDOM_X_RANGE = (-0.40, 0.40)
RANDOM_Y_RANGE = (-0.20, 0.20)
SPAWN_DROP_Z = 1.0

DROP_TEST_POS = (0.10, 0.05, 1.00)
CENTER_TEST_POS = (0.0, 0.0, 0.80)

# Free-camera framing matching table_scene.xml's fixed "table_view" camera,
# used to snap the viewer back onto the table (Home key) if mouse
# panning/zooming drifts it away -- R/D/C/T only move the cube, never the
# camera, so drift persists across key presses until reset.
HOME_CAM_LOOKAT = (0.0, 0.0, 0.5)
HOME_CAM_DISTANCE = 2.6
HOME_CAM_AZIMUTH = 225.0
HOME_CAM_ELEVATION = -25.0

# How long to wait (sim time) after a spawn before printing the settled
# position, so the printed value reflects a landed cube rather than one
# still mid-fall.
SETTLE_REPORT_DELAY = 2.0


class CubeSpawner:
    """Wraps qpos/qvel access for the cube's free joint."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
        if joint_id == -1:
            raise RuntimeError("cube_freejoint not found in model")

        # A free joint contributes 7 qpos entries (3 pos + 4 quat) and 6
        # qvel entries (3 linear + 3 angular). qpos_adr/dof_adr give the
        # starting index of each within the flat qpos/qvel arrays.
        self.qpos_adr = model.jnt_qposadr[joint_id]
        self.qvel_adr = model.jnt_dofadr[joint_id]

        self.pending_report_until = None

    def spawn_cube(self, x: float, y: float, z: float) -> None:
        """Move the cube to (x, y, z) with zero velocity and identity
        orientation, then push the change into the simulation state.

        Does not edit or recompile the XML -- this only mutates the
        already-compiled model's per-step data (mjData.qpos / qvel).
        """
        qpos_slice = slice(self.qpos_adr, self.qpos_adr + 7)
        qvel_slice = slice(self.qvel_adr, self.qvel_adr + 6)

        # Position (3) + quaternion (4, identity = no rotation).
        self.data.qpos[qpos_slice] = [x, y, z, 1.0, 0.0, 0.0, 0.0]
        # Linear velocity (3) + angular velocity (3), both zeroed.
        self.data.qvel[qvel_slice] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Recompute all derived quantities (contacts, kinematics, etc.)
        # from the new qpos/qvel before the next mj_step.
        mujoco.mj_forward(self.model, self.data)

        print("SPAWN CUBE")
        print(f"x = {x:.3f}")
        print(f"y = {y:.3f}")
        print(f"z = {z:.3f}")

        self.pending_report_until = self.data.time + SETTLE_REPORT_DELAY

    def maybe_report_settled(self) -> None:
        if self.pending_report_until is None:
            return
        if self.data.time >= self.pending_report_until:
            pos = self.data.qpos[self.qpos_adr:self.qpos_adr + 3]
            print(f"Cube settled position: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}")
            self.pending_report_until = None


def random_spawn_position() -> tuple:
    x = random.uniform(*RANDOM_X_RANGE)
    y = random.uniform(*RANDOM_Y_RANGE)
    return x, y, SPAWN_DROP_Z


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    spawner = CubeSpawner(model, data)

    # TEST 1: initial cube position (x=0, y=0, z=1.0) is already set by
    # the XML's <body pos="0 0 1.0">; just forward the initial state so
    # contacts etc. are valid before the first mj_step.
    mujoco.mj_forward(model, data)
    print("TEST 1: initial cube spawned from XML at x=0.000 y=0.000 z=1.000, falling onto table.")
    spawner.pending_report_until = data.time + SETTLE_REPORT_DELAY

    # viewer is created below; this holder lets key_callback (built before
    # the viewer exists) reach it once launch_passive returns.
    viewer_holder = {}

    def reset_camera() -> None:
        cam = viewer_holder["viewer"].cam
        cam.lookat[:] = HOME_CAM_LOOKAT
        cam.distance = HOME_CAM_DISTANCE
        cam.azimuth = HOME_CAM_AZIMUTH
        cam.elevation = HOME_CAM_ELEVATION
        print("Camera reset to table-framing view.")

    def key_callback(keycode: int) -> None:
        if keycode == glfw.KEY_R:
            x, y, z = random_spawn_position()
            spawner.spawn_cube(x, y, z)
        elif keycode == glfw.KEY_D:
            spawner.spawn_cube(*DROP_TEST_POS)
        elif keycode == glfw.KEY_C:
            spawner.spawn_cube(*CENTER_TEST_POS)
        elif keycode == glfw.KEY_T:
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            print("RESET simulation to initial state.")
            spawner.pending_report_until = data.time + SETTLE_REPORT_DELAY
        elif keycode == glfw.KEY_HOME:
            reset_camera()

    print("Keyboard controls: R=random spawn, D=drop test, C=center spawn, T=reset, Home=reset camera")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer_holder["viewer"] = viewer
        reset_camera()

        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            spawner.maybe_report_settled()
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
