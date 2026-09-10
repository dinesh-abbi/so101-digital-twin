#!/usr/bin/env python3
"""View the SO-101 sitting in a table CORNER instead of the centre.

Sim-only visualization variant. robot_corner_scene.xml is byte-identical to
../robot_keyboard_test/keyboard_robot_scene.xml -- the corner offset is
applied HERE, in Python, after loading, not in the XML.

WHY THIS WAY: so101.xml is a complete, self-contained MJCF document
(its own <option>/<default>/<asset>/<worldbody>/<actuator>), included via
MuJoCo's <include>, a literal text splice rather than a scoped import. It
cannot be wrapped in a positioning <body> -- tried, and MuJoCo rejects it
("unrecognized element 'option'") because so101.xml's top-level elements
land inside a <body>, where they aren't legal. so101.xml itself hardcodes
its "base" body at pos="0 0 0" with no positionable parent, and per this
project's convention is never edited directly.

What DOES work, and what this script does: "base"'s parent is the WORLD
body (id 0) -- confirmed via model.body_parentid -- so overwriting
model.body_pos[base_id] before the first mj_forward() rigidly translates
the ENTIRE kinematic chain. Confirmed by checking a downstream body (the
gripper) moved by the same offset, not just the base geom.

The offset below is a SIM-ONLY choice, not measured against any real
hardware. Table half-extents here are 0.50 x 0.30 (1.0m x 0.6m full,
copied unmodified from the keyboard scene), so the arm is placed inset
0.10m from the true front-right corner (0.50, -0.30) -- enough that the
base plate and the arm's swing radius stay on the tabletop.
"""

import numpy as np
import mujoco
import mujoco.viewer

SCENE_PATH = "robot_corner_scene.xml"

# Sim-only visualization choice -- see module docstring. Not a
# hardware-measured position.
#
# Tabletop half-extents: x in [-0.50, 0.50], y in [-0.30, 0.30] (1.0m x
# 0.6m full, from the shared table geometry).
#
# Measured max horizontal reach from base (sweeping shoulder_lift and
# elbow_flex across their full ranges, script used to find this is in the
# conversation history, not kept as a separate file): 0.381 m.
#
# Placed at the CENTRE of the near LONG EDGE (x=0, table's true edge
# midpoint is (0, -0.30) -- these tabletop half-extents are the real
# LINNMON dimensions, 100cm x 60cm, not a placeholder).
#
# The base's own mesh geometry (its mounting plate) extends 0.031m BEHIND
# its origin along the arm's -Y axis once rotated -- measured directly by
# transforming the base body's mesh vertices to world coordinates, not
# assumed from a bounding box. Placing the origin exactly at y=-0.30 would
# push 3.1cm of that plate off the table into open air. y=-0.264 (edge +
# 0.031 rear extent + 0.005 safety margin) is the closest the base can sit
# to the true edge while keeping its whole footprint on the tabletop.
# The arm faces +y (inward) via CORNER_QUAT below.
CORNER_OFFSET = (0.0, -0.264, 0.0)

# Unrotated, the arm's rest pose reaches almost purely along +x (confirmed
# by reading data.xpos of the gripper body at identity rotation). Rotating
# the base +90 deg about Z turns that reach toward +y instead -- verified
# by computing both +90/-90 and checking the resulting gripper direction
# from base, not assumed from the right-hand rule alone.
_THETA = np.radians(90)
CORNER_QUAT = (np.cos(_THETA / 2), 0, 0, np.sin(_THETA / 2))


def main():
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    if base_id == -1:
        raise RuntimeError("body 'base' not found in the compiled model")
    if model.body_parentid[base_id] != 0:
        raise RuntimeError(
            "'base' is no longer a direct child of the world body -- the "
            "body_pos override this script relies on assumes that. "
            "so101.xml may have changed upstream; re-check before trusting "
            "the corner placement.")

    model.body_pos[base_id] = CORNER_OFFSET
    model.body_quat[base_id] = CORNER_QUAT
    mujoco.mj_forward(model, data)

    print(f"base moved to {CORNER_OFFSET}")
    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    if gripper_id != -1:
        print(f"gripper world pos: {data.xpos[gripper_id]}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
