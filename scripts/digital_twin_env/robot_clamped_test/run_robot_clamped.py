#!/usr/bin/env python3
"""Real-setup parity viewer: SO-101 clamped to the table edge, with cables.

Sim-only visualisation. Extends robot_corner_test/run_robot_corner.py (arm
placed at the table's near edge) with a table clamp and cable runs, so the
scene reads as "arm bolted to a table" rather than an arm floating at the
origin:

        SO-101
          |
        clamp
          |
    ==============  table
          |
       wires

WHAT THIS DOES NOT TOUCH
------------------------
Nothing here affects control, mapping, actuators or gains. It is a viewer.
The clamp and wires are contype=0/conaffinity=0 (visual only), so they add
ZERO contacts -- verified in validate_robot_clamped.py, which compares
contact counts against the un-clamped corner scene at every pose.

That matters for the digital-twin work: this project is investigating why
the real arm's measured position differs from its commanded position under
the low-gain servo fit, so any scene change that silently added contacts
would corrupt exactly the signal being measured. Cosmetic geometry must
stay cosmetic.

WHY THE CLAMP IS VISUAL-ONLY
----------------------------
A real clamp grips the base plate, i.e. it physically overlaps it. Modelled
with collision on, that overlap is a permanent deep contact pair between
`base` and the clamp -- the same failure mode that made the table-vs-arm
contacts jam tracking by up to 38.9 deg (see m_lerobot_teleop_sim.py). A
visual clamp shows the mounting without paying that cost.

Usage:
    python run_robot_clamped.py
    python run_robot_clamped.py --pose folded
    python run_robot_clamped.py --no-wires
    python run_robot_clamped.py --centre      # arm at table centre, no edge
"""

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "real_sim_mapping_test"))
from real_sim_joint_mapping import widen_shoulder_lift  # noqa: E402

SCENE_PATH = Path(__file__).resolve().parent / "robot_clamped_scene.xml"

# ===========================================================================
# CONFIG -- the one place to change scene dimensions/positions.
# Everything below is metres / radians. The XML carries the same numbers as
# placeholders; these overwrite them through MjSpec before compiling, so
# edit HERE, not in the XML.
# ===========================================================================

# --- TABLE ---------------------------------------------------------------
# Real IKEA LINNMON tabletop: 100 x 60 cm, 3 cm thick. MuJoCo boxes take
# HALF-extents, hence the /2 at use. Kept identical to the corner/keyboard
# scenes so all three stay visually comparable.
TABLE_LENGTH = 1.00      # along X
TABLE_WIDTH = 0.60       # along Y
TABLE_THICKNESS = 0.03
TABLE_HEIGHT = 0.74      # floor to tabletop TOP surface

# Project-wide convention (see robot_on_table_scene.xml's header): the
# tabletop's TOP surface is world z = 0, because so101.xml hardcodes the
# robot base at 0 0 0 and cannot be moved in XML. The floor therefore sits
# at -TABLE_HEIGHT. Changing this breaks that convention -- don't.
TABLETOP_TOP_Z = 0.0

# --- ROBOT BASE ----------------------------------------------------------
# Copied from robot_corner_test/run_robot_corner.py, which derived it by
# transforming the base body's mesh vertices to world coords: the mounting
# plate extends 0.031 m behind the base origin, so an origin exactly at the
# edge (y = -TABLE_WIDTH/2) would hang 3.1 cm of plate off the table.
# -0.264 = edge + 0.031 rear extent + 0.005 margin.
ROBOT_BASE_POSITION = (0.0, -0.264, 0.0)
ROBOT_BASE_YAW = math.radians(90)   # faces +Y, inward across the table

# --- CLAMP ---------------------------------------------------------------
# MEASURED against the base mesh vertices (not rbound): the base plate
# occupies X -0.0555..+0.0555, Y -0.295..-0.199, with its underside at
# z = -0.0024, i.e. flush on the tabletop. The clamp must therefore be
# centred under the MIDDLE of that plate -- y = -0.247 -- with its top pad
# immediately beneath it. An earlier y = -0.272 put the clamp outboard of
# the plate, so it visibly gripped nothing.
CLAMP_POSITION = (0.0, -0.247, TABLETOP_TOP_Z)

# Pad spans most of the plate's 0.111 m width so it reads as gripping it.
CLAMP_PAD_HALF = (0.042, 0.030, 0.005)    # top pad / bottom jaw half-extents
CLAMP_SPINE_HALF = (0.042, 0.009, 0.038)  # vertical back of the C
CLAMP_SCREW_RADIUS = 0.005
CLAMP_SCREW_HALF_LEN = 0.014
CLAMP_HANDLE_RADIUS = 0.004
CLAMP_HANDLE_HALF_LEN = 0.022

# --- SERIAL BUS DRIVER BOARD --------------------------------------------
# The real arm carries a Feetech/Waveshare serial bus servo driver on the
# base, and every servo cable terminates THERE, not at the table. Mounted on
# the `base` BODY (not the world) so it rides the arm if the base is ever
# repositioned.
#
# NOTE THE FRAME: these are LOCAL to `base`, which build_model() yaws by
# ROBOT_BASE_YAW (+90 deg). so101.xml's base has its long axis along local
# Y, so the board goes at local -Y (behind the arm, over the table edge)
# with its long side along local X. Specifying it in world coords instead
# put the board a quarter-metre off to the side of the robot.
DRIVER_BOARD_POSITION = (-0.062, 0.0, 0.030)   # local to `base`
DRIVER_BOARD_HALF = (0.010, 0.033, 0.022)      # ~20 x 66 x 44 mm
DRIVER_BOARD_RGBA = (0.10, 0.20, 0.12, 1.0)    # PCB green
DRIVER_PORT_HALF = (0.003, 0.008, 0.004)       # the servo-cable headers
DRIVER_PORT_RGBA = (0.85, 0.85, 0.88, 1.0)

# --- WIRES = THE SERVO SERIAL-BUS CHAIN ----------------------------------
# "Wires" in this project means the Feetech STS3215 servo cables: the
# daisy-chain linking each actuator to the next and terminating at the
# serial bus driver board on the base. NOT power leads, NOT environmental
# cabling -- nothing runs to the table or the floor.
#
# Radius/colour match m_lerobot_teleop_sim.py's WIRE_RADIUS/WIRE_RGBA.
WIRE_RADIUS = 0.004
WIRE_RGBA = (0.05, 0.05, 0.05, 1.0)

# The chain, distal -> proximal, exactly as the real loom runs: each servo's
# cable goes to the NEXT servo down the arm, and the last hop lands on the
# driver board. Each entry is (body, child_body): one capsule drawn inside
# `body`'s frame from its origin to the child's attachment point (the same
# local vector so101.xml uses to place that child), so it rides that body
# rigidly through mj_step with no per-tick bookkeeping.
WIRE_SEGMENTS = (
    ("shoulder", "upper_arm"),
    ("upper_arm", "lower_arm"),
    ("lower_arm", "wrist"),
    ("wrist", "gripper"),
)

# Final hop: shoulder servo -> driver board. In the `base` frame (same
# local axes as DRIVER_BOARD_POSITION above), since both ends are fixed
# relative to it. The mid waypoint fakes cable slack so it reads as a
# cable rather than a strut.
WIRE_BASE_TO_BOARD = (
    (0.0, 0.030, 0.056),                      # shoulder servo cable exit
    (-0.035, 0.010, 0.060),                   # slack loop over the plate
    (DRIVER_BOARD_POSITION[0] + DRIVER_BOARD_HALF[0],   # board header face
     DRIVER_BOARD_POSITION[1],
     DRIVER_BOARD_POSITION[2] + 0.010),
)

# ===========================================================================

REST_QPOS_RAD = np.deg2rad([0.0, -50.0, 48.0, 76.0, 0.0, 0.0])

# Same folded parking pose as robot_on_table_test/run_robot_on_table.py --
# see that file for why shoulder_lift goes to -110 rather than so101.xml's
# -100 (both real arms on this bench travel further, and the real rest pose
# lays the upper arm onto the base).
FOLDED_QPOS_RAD = np.deg2rad([0.0, -110.0, 92.0, 60.0, 0.0, 0.0])

POSES = {"rest": REST_QPOS_RAD, "folded": FOLDED_QPOS_RAD}


def apply_table_config(spec):
    """Push the CONFIG table numbers into the scene's geoms."""
    half_l, half_w = TABLE_LENGTH / 2.0, TABLE_WIDTH / 2.0
    half_t = TABLE_THICKNESS / 2.0

    # The table BODY sits at -TABLE_HEIGHT and its geoms are offset back up,
    # matching the shared scene layout; keep that structure so this scene
    # stays diffable against robot_corner_scene.xml.
    table = spec.body("table")
    table.pos = [0, 0, TABLETOP_TOP_Z - TABLE_HEIGHT]

    top = spec.geom("tabletop")
    top.size = [half_l, half_w, half_t]
    top.pos = [0, 0, TABLE_HEIGHT - half_t]

    leg_half_h = (TABLE_HEIGHT - TABLE_THICKNESS) / 2.0
    inset = 0.08
    for name, sx, sy in (
        ("table_leg_front_left", 1, 1),
        ("table_leg_front_right", 1, -1),
        ("table_leg_back_left", -1, 1),
        ("table_leg_back_right", -1, -1),
    ):
        leg = spec.geom(name)
        leg.size = [0.025, leg_half_h, 0]
        leg.pos = [sx * (half_l - inset), sy * (half_w - inset), leg_half_h]

    spec.geom("floor").pos = [0, 0, TABLETOP_TOP_Z - TABLE_HEIGHT]


def apply_clamp_config(spec):
    """Push the CONFIG clamp numbers into the clamp geoms.

    The clamp body sits at the tabletop surface (z=0), under the base plate.
    Geometry is built DOWNWARD from there: the upper jaw grips the tabletop
    top (immediately under the robot's plate), the spine wraps the near
    edge, and the lower jaw + screw close underneath. Everything is derived
    from TABLE_THICKNESS so it still wraps correctly if the table changes.
    """
    spec.body("clamp").pos = list(CLAMP_POSITION)

    pad_x, pad_y, pad_z = CLAMP_PAD_HALF
    spine_x, spine_y, spine_z = CLAMP_SPINE_HALF

    # Upper jaw: pressed against the tabletop top, directly beneath the
    # robot's base plate (whose underside is flush at z=0).
    top = spec.geom("clamp_top_pad")
    top.size = [pad_x, pad_y, pad_z]
    top.pos = [0, 0, -pad_z]

    # Spine: hangs off the outboard (-Y) face and must span the tabletop
    # thickness plus both jaws, or the C will not visibly wrap the edge.
    spine_half_h = (TABLE_THICKNESS + 2 * pad_z + 0.010) / 2.0
    spine = spec.geom("clamp_spine")
    spine.size = [spine_x, spine_y, spine_half_h]
    spine.pos = [0, -(pad_y + spine_y), -spine_half_h]

    # Lower jaw: below the tabletop underside.
    bottom_z = -(2 * spine_half_h) + pad_z
    bottom = spec.geom("clamp_bottom_jaw")
    bottom.size = [pad_x, pad_y, pad_z]
    bottom.pos = [0, 0, bottom_z]

    # Screw rises from the lower jaw toward the table underside.
    screw = spec.geom("clamp_screw")
    screw.size = [CLAMP_SCREW_RADIUS, CLAMP_SCREW_HALF_LEN, 0]
    screw.pos = [0, 0, bottom_z + pad_z + CLAMP_SCREW_HALF_LEN]

    handle_z = bottom_z - pad_z - CLAMP_HANDLE_RADIUS
    handle = spec.geom("clamp_screw_handle")
    handle.size = [CLAMP_HANDLE_RADIUS, CLAMP_HANDLE_HALF_LEN, 0]
    handle.fromto = [-CLAMP_HANDLE_HALF_LEN, 0, handle_z,
                     CLAMP_HANDLE_HALF_LEN, 0, handle_z]


def add_driver_board(spec):
    """The serial bus servo driver board on the arm's base.

    Added to the `base` BODY, so it travels with the arm rather than being
    pinned to a world position. Visual only, like the clamp.
    """
    base = spec.body("base")
    bx, by, bz = DRIVER_BOARD_POSITION

    base.add_geom(
        name="driver_board",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(DRIVER_BOARD_HALF),
        pos=[bx, by, bz],
        rgba=DRIVER_BOARD_RGBA,
        contype=0,
        conaffinity=0,
        group=1,
    )

    # Header block the servo bus plugs into -- gives the cable chain a
    # visible termination instead of ending in mid-air.
    base.add_geom(
        name="driver_board_header",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(DRIVER_PORT_HALF),
        pos=[bx + DRIVER_BOARD_HALF[0] + DRIVER_PORT_HALF[0], by, bz + 0.010],
        rgba=DRIVER_PORT_RGBA,
        contype=0,
        conaffinity=0,
        group=1,
    )


def add_wire_geoms(spec):
    """The servo serial-bus cable chain.

    These are the Feetech servo cables: each actuator daisy-chained to the
    next, with the final hop landing on the driver board. Nothing routes to
    the table or floor -- that is not what this loom does.

    Visual only -- contype/conaffinity 0 on every capsule, so these add no
    contacts and need no exclude() entries.
    """
    for parent_name, child_name in WIRE_SEGMENTS:
        parent = spec.body(parent_name)
        child = spec.body(child_name)
        parent.add_geom(
            name=f"wire_{parent_name}_{child_name}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[WIRE_RADIUS, 0, 0],
            fromto=[0, 0, 0, *child.pos],
            rgba=WIRE_RGBA,
            contype=0,
            conaffinity=0,
            group=2,
        )

    # Last hop: shoulder servo down to the board's header. In the `base`
    # frame -- both ends are fixed relative to it, so it needs no per-tick
    # update and stays attached if the base is repositioned.
    base = spec.body("base")
    for i in range(len(WIRE_BASE_TO_BOARD) - 1):
        a, b = WIRE_BASE_TO_BOARD[i], WIRE_BASE_TO_BOARD[i + 1]
        base.add_geom(
            name=f"wire_bus_to_board_{i}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[WIRE_RADIUS, 0, 0],
            fromto=[*a, *b],
            rgba=WIRE_RGBA,
            contype=0,
            conaffinity=0,
            group=2,
        )


def build_model(wires=True, centre=False):
    """Compile the scene. Shared by the viewer and the validator."""
    spec = mujoco.MjSpec.from_file(str(SCENE_PATH))

    apply_table_config(spec)
    apply_clamp_config(spec)
    add_driver_board(spec)

    # This arm folds ~10 deg past so101.xml's shoulder_lift limit and rests
    # the upper arm on the base. Scene-local; never touches the shared XML.
    widen_shoulder_lift(spec)

    if wires:
        add_wire_geoms(spec)

    model = spec.compile()
    data = mujoco.MjData(model)

    # Edge placement, applied post-compile: so101.xml cannot be wrapped in a
    # positioning <body> (see the scene XML header), but `base`'s parent is
    # the world body, so writing body_pos/body_quat before the first
    # mj_forward translates the whole kinematic chain rigidly. Same
    # mechanism as run_robot_corner.py and m_lerobot_teleop_sim.py.
    if not centre:
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        if base_id == -1:
            raise RuntimeError("body 'base' not found -- scene changed?")
        if model.body_parentid[base_id] != 0:
            raise RuntimeError(
                "'base' is no longer a direct child of the world body; the "
                "body_pos override assumes that. Re-check so101.xml.")
        model.body_pos[base_id] = ROBOT_BASE_POSITION
        model.body_quat[base_id] = (math.cos(ROBOT_BASE_YAW / 2), 0, 0,
                                    math.sin(ROBOT_BASE_YAW / 2))

    return model, data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose", choices=sorted(POSES), default="rest")
    parser.add_argument("--no-wires", action="store_true",
                        help="Omit the cosmetic cables.")
    parser.add_argument("--centre", action="store_true",
                        help="Leave the arm at the table centre instead of "
                             "the clamped edge position (the clamp stays "
                             "at the edge, so they will not line up -- for "
                             "comparison against the older centred scenes).")
    args = parser.parse_args()

    model, data = build_model(wires=not args.no_wires, centre=args.centre)
    qpos = POSES[args.pose]

    data.qpos[:6] = qpos
    data.ctrl[:6] = qpos
    mujoco.mj_forward(model, data)

    print(f"pose={args.pose}  wires={not args.no_wires}  "
          f"contacts at load: {data.ncon}")

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
