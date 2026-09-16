#!/usr/bin/env python3
"""Mark where the real cube is by putting the gripper around it, then keep
the sim cube in step with it during leader-arm teleop.

Used by m_lerobot_teleop_sim.py --bare --cube:
  1. Open jaws down around the real cube, not pressing it. Press M in the
     terminal: the sim cube is placed between the sim jaws and saved to disk.
  2. Lift straight up, go home. Record, and pick the cube.
  3. While the REAL gripper holds it, the sim gripper carries the sim cube;
     when the real jaws open, the sim cube drops.

The cube collides with the floor only, never the arm. The sim gripper sits
~1 cm off the real one, so simulated contact would knock the cube away on
approaches the real fingers never touched it on, and a simulated grasp on
the 0.75 mm fingertip spheres slips (replay_pick_place.py). The grasp is
read from the real gripper instead -- see GripDetector.
"""

import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

CUBE_HALF = 0.010
CUBE_MASS = 0.0032
CUBE_RGBA = (0.08, 0.20, 0.85, 1.0)
# The cube and the floor share this collision bit; every arm geom uses bit 1 only.
CUBE_BIT = 2
PARKED_XY = (-0.20, 0.20)

# Sim fingertips must be this close to the sim cube for a real grab to carry it.
ATTACH_RADIUS = 0.04
# A mark is refused with the fingertips higher than this above the floor, or
# closer together than the cube is wide. The folded rest pose also has its
# tips at the floor, but only 6 mm apart; a real grab stalls at 19 mm.
MAX_MARK_HEIGHT = 0.05
MIN_MARK_SPACING = 2 * CUBE_HALF

# Fitted on every real-follower recording on this bench (2026-09-16): 9/9 cube
# grabs caught (jaw stalled at 10.9-11.7 while the leader said 1.4-1.7) and 0
# false grabs across 11 empty closes (jaw runs on to ~1.8). Re-checked by
# validate_cube_mark.py.
HOLD_GAP = 3.0
HOLD_MIN = 5.0
STALL_S = 0.25
STALL_TOL = 0.6
OPEN_MARGIN = 2.0
# Replay: a real jaw closing on the cube again stops a little wider or
# narrower each run (10.8-12.1 across one hold in pick_place_3.csv), so a
# grab needs the shown jaw closed to within this of the recorded grip, not an
# exact match -- a +/-1.2 match made the sim grab only some of the time.
REPLAY_JAW_SLACK = 3.0
# After a grab the cube eases from where it sat into the jaws over this much
# lift of the fingertips, instead of jumping there.
LIFT_BLEND_M = 0.02

DEFAULT_MARK_FILE = (Path(__file__).resolve().parents[3]
                     / "recordings" / "cube_marked.json")


class GripDetector:
    """Is the REAL gripper holding something?

    Holding = the follower's jaw is at least HOLD_GAP more open than the
    leader commands, well above empty-closed, and has stopped closing for
    STALL_S. Released when the jaw opens past where it gripped, or closes on
    through (the object slipped out). Inputs are LeRobot-normalised units.
    """

    def __init__(self):
        self.holding = False
        self.jaw_at_grab = None
        self._since = None
        self._lo = self._hi = None

    def step(self, jaw, cmd, now):
        """Feed one tick. Returns "grab", "release" or None."""
        if self.holding:
            if jaw > self.jaw_at_grab + OPEN_MARGIN or jaw < HOLD_MIN:
                self.holding = False
                self._since = None
                return "release"
            return None

        if not (jaw - cmd >= HOLD_GAP and jaw >= HOLD_MIN):
            self._since = None
            return None
        if self._since is None:
            self._since, self._lo, self._hi = now, jaw, jaw
            return None
        self._lo, self._hi = min(self._lo, jaw), max(self._hi, jaw)
        if self._hi - self._lo > STALL_TOL:
            self._since, self._lo, self._hi = now, jaw, jaw
            return None
        if now - self._since >= STALL_S:
            self.holding, self.jaw_at_grab = True, jaw
            self._since = None
            return "grab"
        return None


def add_cube(spec):
    """Add the 2 cm cube to a scene spec with a geom named 'floor'."""
    floor = spec.geom("floor")
    if floor is None:
        raise RuntimeError("no geom named 'floor' in this scene -- the cube "
                           "needs the bare scene (robot on the groundplane)")
    floor.conaffinity |= CUBE_BIT
    body = spec.worldbody.add_body(name="cube", pos=[*PARKED_XY, CUBE_HALF])
    body.add_freejoint(name="cube_freejoint")
    body.add_geom(name="cube_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                  size=[CUBE_HALF] * 3, mass=CUBE_MASS, rgba=list(CUBE_RGBA),
                  contype=CUBE_BIT, conaffinity=CUBE_BIT)


class MarkedCube:
    """The sim cube: placed by mark_here(), carried while the real gripper holds it."""

    def __init__(self, model, data, mark_file=DEFAULT_MARK_FILE):
        self.model, self.data = model, data
        self.mark_file = Path(mark_file) if mark_file is not None else None
        self.grip = GripDetector()
        self.held = False
        self.mark = None
        self._kin = mujoco.MjData(model)

        def gid(name):
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

        self._body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
        if self._body == -1 or jid == -1:
            raise RuntimeError("no cube in this model -- call add_cube(spec) "
                               "before compiling")
        self._qadr = int(model.jnt_qposadr[jid])
        self._vadr = int(model.jnt_dofadr[jid])
        self._gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self._fixed_tips = [gid(f"fixed_jaw_sph_tip{i}") for i in (1, 2, 3)]
        self._moving_tips = [gid(f"moving_jaw_sph_tip{i}") for i in (1, 2, 3)]
        self._floor_z = float(model.geom_pos[gid("floor")][2])
        self._rel_start = self._rel_goal = self._rel_rot = None
        self._tips_z0 = None
        self._blend = 0.0
        self._jaw_min = None

        if self.mark_file is not None and self.mark_file.exists():
            self.mark = json.loads(self.mark_file.read_text())
            self._place(self.mark["x"], self.mark["y"],
                        math.radians(self.mark["yaw_deg"]))

    def startup_message(self):
        if self.mark is None:
            return ("  [cube] no cube marked yet -- put the OPEN jaws down "
                    "around the real cube,\n         click this terminal, "
                    "press M")
        return (f"  [cube] using the mark from {self.mark['marked_at']}: "
                f"x={self.mark['x']:+.3f} y={self.mark['y']:+.3f} m\n"
                "         If the real cube has moved since, re-mark it: open "
                "jaws around it, press M")

    def mark_here(self, arm_qpos):
        """Place the cube between the jaws of `arm_qpos` (the 6 mirrored joints,
        sim radians) and save it. Returns a message to print."""
        if self.grip.holding:
            return ("  [cube] NOT marked: the real gripper is holding "
                    "something -- open it first")
        fixed, moving = self._kinematic_tips(arm_qpos)
        mid = (fixed + moving) / 2
        height = float(mid[2] - self._floor_z)
        if height > MAX_MARK_HEIGHT:
            return (f"  [cube] NOT marked: the fingertips are "
                    f"{height * 100:.1f} cm above the floor. Put the open "
                    "jaws down around the cube, then press M")
        axis = fixed - moving
        spacing = float(np.linalg.norm(axis))
        if spacing < MIN_MARK_SPACING:
            return (f"  [cube] NOT marked: the jaws are only "
                    f"{spacing * 1000:.0f} mm apart, narrower than the "
                    f"{MIN_MARK_SPACING * 1000:.0f} mm cube. Open them a little "
                    "wider around it, then press M")

        yaw = (math.atan2(axis[1], axis[0])
               if math.hypot(axis[0], axis[1]) > 0.005 else 0.0)
        self._place(mid[0], mid[1], yaw)
        self.held = False
        self.mark = {
            # The arm pose AT THE MARK, sim radians -- lets goto_marked_cube.py
            # walk the real arm back to exactly this pose later, so the real
            # cube can be placed under the jaws again without IK: this is the
            # same forward-kinematics pose that put the sim cube here, just
            # replayed on request instead of continuously.
            "arm_qpos_sim_rad": [round(float(v), 6) for v in arm_qpos],
            "x": round(float(mid[0]), 4),
            "y": round(float(mid[1]), 4),
            "yaw_deg": round(math.degrees(yaw), 1),
            "tip_spacing_mm": round(spacing * 1000, 1),
            "tips_above_floor_mm": round(height * 1000, 1),
            "marked_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.mark_file.parent.mkdir(parents=True, exist_ok=True)
        self.mark_file.write_text(json.dumps(self.mark, indent=2))
        return (f"  [cube] MARKED at x={mid[0]:+.3f} y={mid[1]:+.3f} m "
                f"(yaw {math.degrees(yaw):+.0f} deg), saved to "
                f"{self.mark_file.name}")

    def update(self, jaw, cmd, arm_qpos, now):
        """Call once per tick, after stepping the sim. Returns a message or None."""
        event = self.grip.step(jaw, cmd, now)
        msg = None
        if event == "grab":
            if self.mark is None:
                msg = ("  [cube] the real gripper is holding something, but "
                       "no cube is marked -- not carrying anything")
            else:
                fixed, moving = self._kinematic_tips(arm_qpos)
                dist = float(np.linalg.norm((fixed + moving) / 2
                                            - self.data.xpos[self._body]))
                if dist <= ATTACH_RADIUS:
                    self._attach()
                    msg = (f"  [cube] GRABBED (sim jaws were {dist * 1000:.0f}"
                           " mm from the sim cube)")
                else:
                    msg = (f"  [cube] the real gripper is holding something, "
                           f"but the sim cube is {dist * 100:.1f} cm away -- "
                           "not carrying it. If the real cube moved, re-mark")
        elif event == "release" and self.held:
            self.held = False
            self.data.qvel[self._vadr:self._vadr + 6] = 0.0
            msg = "  [cube] RELEASED"

        if self.held:
            self._follow()
        return msg

    def place_mark(self, mark):
        """Put the cube at a saved mark dict (e.g. a recording's sidecar)
        without reading or writing mark_file."""
        self.mark = mark
        self.held = False
        self._place(mark["x"], mark["y"], math.radians(mark.get("yaw_deg", 0.0)))

    def follow_recording(self, recorded_held, jaw, recorded_jaw):
        """Replay: carry the cube with the arm ON SCREEN, not the positions
        the recording logged. Call once per tick after stepping the sim.

        A replayed real arm trails its recording (clamp, servo lag), so the
        logged cube positions run ahead of the jaws shown -- 31-55 mm off at
        0.4 s of lag on pick_place_2.csv. Grab when the recording says held
        AND the shown jaw has caught up to the recorded grip; release when
        the recording says let go AND the shown jaw has actually opened.
        `jaw` is the shown (measured) gripper, `recorded_jaw` the recording's,
        both LeRobot-normalised. Returns a message or None.
        """
        msg = None
        if not self.held:
            if recorded_held and jaw <= recorded_jaw + REPLAY_JAW_SLACK:
                tips = self.data.geom_xpos[self._fixed_tips + self._moving_tips].mean(axis=0)
                dist = float(np.linalg.norm(tips - self.data.xpos[self._body]))
                if dist <= ATTACH_RADIUS:
                    self._attach()
                    self._jaw_min = jaw
                    msg = f"  [cube] GRABBED (jaws {dist * 1000:.0f} mm from the cube)"
        else:
            self._jaw_min = min(self._jaw_min, jaw)
            if not recorded_held and jaw > self._jaw_min + OPEN_MARGIN:
                self.held = False
                self.data.qvel[self._vadr:self._vadr + 6] = 0.0
                msg = "  [cube] RELEASED"
        if self.held:
            self._follow()
        return msg

    def pose(self):
        """(x, y, z, held) of the sim cube, world metres."""
        x, y, z = self.data.xpos[self._body]
        return float(x), float(y), float(z), self.held

    def write_recording_note(self, csv_path):
        """Record next to a --record CSV where the cube was marked when it began."""
        Path(str(csv_path) + ".cube.json").write_text(json.dumps(
            {"cube_half_m": CUBE_HALF, "mark_at_start": self.mark}, indent=2))

    def _kinematic_tips(self, arm_qpos):
        # From the joint values alone, not the physics pose, which can settle
        # short of the real arm at a fold or against the floor.
        self._kin.qpos[:] = self.data.qpos
        self._kin.qpos[:6] = arm_qpos
        mujoco.mj_kinematics(self.model, self._kin)
        return (self._kin.geom_xpos[self._fixed_tips].mean(axis=0),
                self._kin.geom_xpos[self._moving_tips].mean(axis=0))

    def _place(self, x, y, yaw):
        a = self._qadr
        self.data.qpos[a:a + 3] = (x, y, self._floor_z + CUBE_HALF)
        self.data.qpos[a + 3:a + 7] = (math.cos(yaw / 2), 0.0, 0.0,
                                       math.sin(yaw / 2))
        self.data.qvel[self._vadr:self._vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _attach(self):
        # The cube belongs in the displayed jaws, height included: the real
        # fingers often close partway up it (lowest sim fingertip 8-20 mm above
        # the floor at today's grabs, against 0-3 mm at the 11 Sep floor
        # touches). Snapping it there at the grab looked like the cube jumping
        # into the gripper, so it does not move at the grab; it eases into the
        # jaws over the first LIFT_BLEND_M of lifting instead.
        d = self.data
        rg = d.xmat[self._gripper].reshape(3, 3)
        pg = d.xpos[self._gripper]
        tips = d.geom_xpos[self._fixed_tips + self._moving_tips].mean(axis=0)
        goal = np.array([tips[0], tips[1],
                         max(float(tips[2]), self._floor_z + CUBE_HALF)])
        self._rel_start = rg.T @ (d.xpos[self._body] - pg)
        self._rel_goal = rg.T @ (goal - pg)
        self._rel_rot = rg.T @ d.xmat[self._body].reshape(3, 3)
        self._tips_z0 = float(tips[2])
        self._blend = 0.0
        self.held = True

    def _follow(self):
        d = self.data
        rg = d.xmat[self._gripper].reshape(3, 3)
        tips_z = float(d.geom_xpos[self._fixed_tips + self._moving_tips][:, 2].mean())
        self._blend = max(self._blend,
                          min(1.0, max(0.0, (tips_z - self._tips_z0) / LIFT_BLEND_M)))
        rel = self._rel_start + self._blend * (self._rel_goal - self._rel_start)
        pos = d.xpos[self._gripper] + rg @ rel
        pos[2] = max(pos[2], self._floor_z + CUBE_HALF)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, (rg @ self._rel_rot).flatten())
        a = self._qadr
        d.qpos[a:a + 3] = pos
        d.qpos[a + 3:a + 7] = quat
        d.qvel[self._vadr:self._vadr + 6] = 0.0
        mujoco.mj_forward(self.model, d)
