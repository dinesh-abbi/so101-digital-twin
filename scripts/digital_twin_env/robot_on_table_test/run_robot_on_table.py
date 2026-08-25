"""Load robot_on_table_scene.xml and launch the MuJoCo passive viewer.

Holds the robot at the established safe REST_QPOS_RAD pose (reused from
scripts/closed_loop_scripted_pick.py / live_viewer_closed_loop_pick.py) via
position actuators so it's easy to visually confirm the robot sits
correctly on the table. No keyboard control yet (M5).

Usage:
    python run_robot_on_table.py
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

SCENE_PATH = Path(__file__).resolve().parent / "robot_on_table_scene.xml"

REST_QPOS_RAD = np.deg2rad([0.0, -50.0, 48.0, 76.0, 0.0, 0.0])


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    data.qpos[:6] = REST_QPOS_RAD
    data.ctrl[:6] = REST_QPOS_RAD
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
