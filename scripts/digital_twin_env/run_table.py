"""Load table_scene.xml and launch the MuJoCo passive viewer.

Usage:
    python run_table.py
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer

SCENE_PATH = Path(__file__).resolve().parent / "table_scene.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

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
