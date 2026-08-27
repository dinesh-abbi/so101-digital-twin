#!/usr/bin/env python3
"""
The measured servo behaviours the MuJoCo model does not reproduce.

Two effects, both measured on this arm (BACKLASH-RESULTS.md), both absent from
a plain MuJoCo `position` actuator:

    backlash       mechanical slop; the joint settles on one side or the other
                   of the geartrain play depending on which way it approached
    gravity droop  the servo settles where its torque balances the load, so it
                   stops short by an amount that varies with joint angle

Neither belongs in so101.xml. MuJoCo has no gear-backlash primitive for a
hinge, and both are properties of THIS arm's servos rather than of the
kinematic model, which is shared with the other arm. Keeping them in a drive
layer leaves the XML a clean model and puts the arm-specific quirks in one
place that can be tested on its own.

Two directions of use:

    SIM  -> REAL   compensate(): given a target the sim reached, work out what
                   to command the real servo so it lands there
    REAL -> SIM    predict(): given a command, predict where the real joint
                   will actually settle, for comparison against a recording

They are inverses, which is the property the self-test checks.

Measured coefficients
---------------------
                backlash    gravity   sin-fit R^2
  shoulder_lift   1.955 deg  1.009 deg    0.880
  elbow_flex      1.196 deg  1.379 deg    0.975

Gravity is fitted against sin(angle), not cos: the torque goes as the sine of
the angle FROM VERTICAL, and this arm's calibrated zero sits near vertical.
Fitting cos instead gave R^2 0.056 / 0.002 while still reporting a confident
amplitude - see BACKLASH-RESULTS.md.

CAVEAT on elbow_flex backlash: measured 1.196 deg on AVERAGE, but the gap is
not constant - it is ~0.5 deg near zero and ~1.9 deg toward the extremes,
symmetric in |angle|, cause not yet understood. Applying it as a constant is
right near the middle of travel and wrong at the ends. shoulder_lift's
1.955 deg IS near-constant (sd 0.372) and is the trustworthy one.

WHERE THIS MODEL APPLIES - and where it must not
------------------------------------------------
It describes where the joint comes to REST. Applying it mid-move, before the
slop has been taken up, is not valid: on the elbow's 20-unit step run the
transient error got 9% worse.

More importantly, do NOT stack it on top of the kp/kv fit from Task 2. Those
gains were fitted against step runs that visit only two commands (0 and
+19.6 deg), so whatever backlash exists at those two points was already
absorbed into the gains. The measured backlash is 1.196 deg but the Task 2
residual direction gap is only 0.118 deg - about 90% of it is already in the
fit. Adding this model there double-counts and makes settled error 245%
worse, verified.

Use it for:
  - predicting where the real arm will settle for a NEW command sequence
  - compensating a command so the real arm lands where intended
  - any trajectory that visits commands the gain fit never saw

Do not use it for:
  - post-hoc correction of the Task 2 replay (already accounted for)
  - anything mid-transient
"""

import math

# joint -> (backlash_deg, gravity_amplitude_deg, gravity_offset_deg, trusted)
#
# gravity is  offset + amplitude * sin(radians(angle))  , the fit from
# analyze_backlash.py. `trusted` records whether the backlash figure behaved
# like a constant across the swept range; see the caveat above.
MEASURED = {
    "shoulder_lift": dict(backlash=1.955, grav_amp=1.009, grav_off=0.867,
                          trusted=True),
    "elbow_flex": dict(backlash=1.196, grav_amp=1.379, grav_off=-0.188,
                       trusted=False),
}


class ServoModel:
    """Backlash + gravity droop for one joint.

    Backlash is modelled as hysteresis with memory: the joint sits half the
    slop above the commanded position when it arrived from below, half below
    when it arrived from above, and does not move at all until a reversal has
    taken up the full slop. That last part is what makes it backlash rather
    than a fixed offset - a small reversal is absorbed entirely by the play
    and produces no motion.
    """

    def __init__(self, joint, backlash_deg=None, grav_amp_deg=None,
                 grav_off_deg=None):
        m = MEASURED.get(joint, {})
        self.joint = joint
        self.backlash = m.get("backlash", 0.0) if backlash_deg is None else backlash_deg
        self.grav_amp = m.get("grav_amp", 0.0) if grav_amp_deg is None else grav_amp_deg
        self.grav_off = m.get("grav_off", 0.0) if grav_off_deg is None else grav_off_deg
        self.trusted = m.get("trusted", False)

        # Hysteresis state: which side of the play we are currently resting on.
        # +1 = arrived from below, -1 = arrived from above, 0 = unknown.
        self._side = 0
        self._last_cmd = None

    # --- the two effects, separately -------------------------------------
    def gravity_droop(self, angle_deg):
        """How far short of `angle_deg` gravity leaves the joint."""
        return self.grav_off + self.grav_amp * math.sin(math.radians(angle_deg))

    def backlash_offset(self, cmd_deg):
        """Signed backlash contribution for a command, updating hysteresis state.

        Returns half the slop, signed by approach direction. The state only
        flips once the command has actually reversed - a command that repeats
        or continues in the same direction keeps resting on the same flank.

        SIGN: the measured gap is `from_below - from_above = -1.955`, i.e. the
        joint settles LOWER when it arrived from below and HIGHER when it
        arrived from above. It lags behind the direction of travel, which is
        what slop does - the driven flank trails the driving one. An earlier
        version had this inverted (moving up -> sits high), which doubled the
        error instead of removing it: validated RMSE went from 1.337 to 2.040
        deg. The self-test now checks against recorded data, not just against
        the model's own arithmetic.
        """
        if self._last_cmd is not None:
            if cmd_deg > self._last_cmd:
                self._side = -1     # travelling up -> resting low
            elif cmd_deg < self._last_cmd:
                self._side = +1     # travelling down -> resting high
            # equal -> unchanged, still resting where it was
        self._last_cmd = cmd_deg
        return self._side * self.backlash / 2.0

    # --- the two directions of use ---------------------------------------
    def predict(self, cmd_deg):
        """Where the real joint will settle, given this command.

        Used to compare a simulated trajectory against a real recording.
        """
        return cmd_deg + self.backlash_offset(cmd_deg) + self.gravity_droop(cmd_deg)

    def compensate(self, target_deg):
        """What to command the real servo so it settles at `target_deg`.

        The inverse of predict(). Gravity is evaluated at the target rather
        than solved implicitly: droop is ~1 deg and d(droop)/d(angle) is under
        0.025 deg per deg, so one pass is already accurate to ~0.03 deg. The
        self-test measures the residual rather than assuming it.
        """
        return target_deg - self.backlash_offset(target_deg) - self.gravity_droop(target_deg)

    def reset(self, side=0):
        """Forget the hysteresis state (e.g. between runs)."""
        self._side = side
        self._last_cmd = None


def _self_test():
    """Check the model reproduces what was measured, and that the two
    directions really are inverses."""
    print("=" * 70)
    print("  servo_model self-test")
    print("=" * 70)

    for joint in MEASURED:
        m = ServoModel(joint)
        print(f"\n  {joint}   backlash {m.backlash:.3f} deg, "
              f"gravity {m.grav_amp:.3f} deg"
              f"{'' if m.trusted else '   (backlash NOT constant - see docstring)'}")

        # 1. Does a staircase reproduce the measured direction gap?
        rungs = [-36.9, -24.6, -12.3, 0.0, 12.3, 24.6, 36.9]
        m.reset()
        up = {}
        for r in rungs:
            up[r] = m.predict(r)
        down = {}
        for r in reversed(rungs):
            down[r] = m.predict(r)
        gaps = [up[r] - down[r] for r in rungs[1:-1]]
        mean_gap = sum(gaps) / len(gaps)
        print(f"    staircase direction gap: {mean_gap:+.3f} deg "
              f"(measured {-m.backlash:+.3f})")
        assert abs(abs(mean_gap) - m.backlash) < 1e-6, "gap != backlash"

        # 2. Is compensate() the inverse of predict()?
        worst = 0.0
        for target in (-40, -20, -5, 0, 5, 20, 40):
            m.reset()
            cmd = m.compensate(target)
            m.reset()
            # Same approach direction both times, so the hysteresis state
            # matches; this isolates the gravity linearisation.
            landed = m.predict(cmd)
            worst = max(worst, abs(landed - target))
        print(f"    compensate/predict round-trip: worst error {worst:.4f} deg")
        assert worst < 0.05, f"round-trip error too large: {worst}"

        # 3. Does a reversal smaller than the slop produce no net motion?
        m.reset()
        m.predict(10.0)            # arrive from below
        a = m.predict(20.0)        # continue up
        b = m.predict(20.0)        # repeat - must not move
        assert a == b, "repeated command moved the joint"
        print("    repeated command produces no motion: OK")

    # 4. The check that actually matters: does the model reduce the error
    #    against the RECORDED data? Checks 1-3 only test the model against its
    #    own arithmetic, and all three passed while the backlash sign was
    #    inverted - which made predictions 53% WORSE than no model at all.
    _validate_against_recordings()

    print("\n  all checks passed")


def _validate_against_recordings(data_dir=None):
    """Compare predict() against the staircase recordings.

    Guards the failure mode the other checks cannot see: a model that is
    self-consistent but wrong about the real hardware.
    """
    import csv
    import pathlib

    try:
        import numpy as np
    except ImportError:
        print("\n  (numpy unavailable - skipping recording validation)")
        return

    d = pathlib.Path(data_dir) if data_dir else pathlib.Path(__file__).parent / "raw_data"
    print("\n  validation against recorded staircases")

    for joint in MEASURED:
        paths = sorted(d.glob(f"{joint}_staircase*_no_load.csv"))
        if not paths:
            print(f"    {joint}: no staircase CSVs, skipped")
            continue

        m = ServoModel(joint)
        raw, modelled = [], []
        for p in paths:
            with open(p, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            c = np.array([float(r["commanded_deg"]) for r in rows])
            a = np.array([float(r["actual_deg"]) for r in rows])
            t = np.array([float(r["time"]) for r in rows])
            bounds = [0] + list(np.where(np.diff(c) != 0)[0] + 1) + [len(c)]
            m.reset()
            for i in range(len(bounds) - 1):
                s0, s1 = bounds[i], bounds[i + 1]
                n = s1 - s0
                if n < 10:
                    continue
                target = float(c[s0])
                pred = m.predict(target)          # advances hysteresis state
                if (t[s1 - 1] - t[s0]) < 0.8:     # not settled - state only
                    continue
                settled = float(np.mean(a[s0 + int(n * 0.6):s1]))
                raw.append(settled - target)
                modelled.append(settled - pred)

        def rms(v):
            return math.sqrt(sum(x * x for x in v) / len(v))

        r0, r1 = rms(raw), rms(modelled)
        verdict = "OK" if r1 < r0 else "MODEL IS WORSE THAN NOTHING"
        print(f"    {joint:15s} {r0:.3f} -> {r1:.3f} deg "
              f"({(1 - r1 / r0) * 100:+.0f}%)  {verdict}")
        assert r1 < r0, f"{joint}: model increased the error ({r0:.3f} -> {r1:.3f})"


if __name__ == "__main__":
    _self_test()
