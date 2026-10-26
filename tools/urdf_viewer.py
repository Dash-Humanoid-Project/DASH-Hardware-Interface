#!/usr/bin/env python3
"""Interactive URDF pose viewer for DASH, with a slider per joint.

Loads the robotDataPackage URDF in pybullet's GUI and adds a debug slider
for every revolute/prismatic joint, letting you manually pose the virtual
model and compare it against the real robot — e.g. to check whether a
joint's absolute-encoder-calibrated zero (see calibrate_absolute_encoder)
actually matches the pose the URDF was modeled at, or whether a measured
joint limit (see record_joint_limits) looks geometrically sane once applied.

Pure kinematic pose viewer, not a physics sim — gravity is off and joints
are moved with resetJointState(), not motors/forces.

Also does live self-collision detection (2026-09-16): pybullet reports real
link-vs-link overlaps via URDF_USE_SELF_COLLISION, not just an eyeballed
check — useful for sanity-checking a candidate limit on a joint that has no
physical hard stop (see the record_joint_limits conversation).

Usage:
    /home/dvolpi/.venvs/dash-urdf-viz/bin/python tools/urdf_viewer.py
"""
import math
import os
import time

import pybullet as p
import pybullet_data

# robotDataPackage/urdf/Dash.urdf references meshes as "meshes/visual/x.stl"
# and "meshes/collision/x.stl" — paths relative to the PACKAGE root (the
# parent of urdf/), not the urdf/ directory itself, so PACKAGE_DIR (not
# URDF_DIR) is what gets chdir'd into below. Originally shipped with
# Windows-style backslash paths (filename="meshes\\collision\\x.stl"),
# which don't resolve on Linux — normalized to forward slashes in place
# (2026-09-16), with the original saved alongside as Dash.urdf.bak.
PACKAGE_DIR = os.path.expanduser("~/Downloads/robotDataPackage")
URDF_RELATIVE_PATH = os.path.join("urdf", "Dash.urdf")
URDF_PATH = os.path.join(PACKAGE_DIR, URDF_RELATIVE_PATH)

# Joints with no real <limit> in the URDF (most of them, currently — see the
# 2026-09-15 conversation on closing the loop between the URDF and hardware)
# get this fallback slider range instead of an unusable infinite one.
FALLBACK_RANGE_RAD = math.pi


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    os.chdir(PACKAGE_DIR)  # so the URDF's relative mesh paths ("meshes/visual/x.stl") resolve
    # EXCLUDE_ALL_PARENTS: without it, every link constantly reports a false
    # "collision" against everything nested near it in the kinematic chain —
    # not just its immediate parent (EXCLUDE_PARENT alone still let e.g.
    # upper_arm <-> prox_shoulder fire, since dist_shoulder sits between
    # them — found 2026-09-16). This keeps only meaningful overlaps between
    # links that aren't expected to be mechanically close, like a leg vs.
    # the torso.
    robot_id = p.loadURDF(
        URDF_RELATIVE_PATH,
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
    )

    link_names = {-1: p.getBodyInfo(robot_id)[0].decode("utf-8")}  # -1 = base link
    sliders = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_index = info[0]
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        lower, upper = info[8], info[9]
        link_names[joint_index] = info[12].decode("utf-8")  # child link name

        if joint_type not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            continue
        if not math.isfinite(lower) or not math.isfinite(upper) or (upper - lower) > 1e3:
            lower, upper = -FALLBACK_RANGE_RAD, FALLBACK_RANGE_RAD

        sliders[joint_index] = p.addUserDebugParameter(joint_name, lower, upper, 0.0)

    print(f"Loaded {URDF_PATH} with {len(sliders)} joint sliders.")
    print("Move sliders in the pybullet window to pose the model. Ctrl+C here to exit.")

    last_colliding_pairs = frozenset()
    while True:
        for joint_index, slider_id in sliders.items():
            value = p.readUserDebugParameter(slider_id)
            p.resetJointState(robot_id, joint_index, value)

        p.performCollisionDetection()
        contacts = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
        colliding_pairs = frozenset(
            frozenset((c[3], c[4])) for c in contacts  # (linkIndexA, linkIndexB)
        )
        if colliding_pairs != last_colliding_pairs:
            if colliding_pairs:
                # contactDistance (index 8) is negative when penetrating, in
                # meters — report the deepest point per pair so it's obvious
                # whether this is a trivial collision-mesh margin (~1mm) or a
                # real interpenetration worth caring about.
                depth_by_pair = {}
                for c in contacts:
                    pair = frozenset((c[3], c[4]))
                    depth_by_pair[pair] = min(depth_by_pair.get(pair, 0.0), c[8])
                parts = []
                for pair in colliding_pairs:
                    names = " <-> ".join(link_names[i] for i in pair)
                    parts.append(f"{names} ({depth_by_pair[pair] * 1000:.1f}mm)")
                print(f"SELF-COLLISION: {', '.join(parts)}")
            else:
                print("Self-collision cleared.")
            last_colliding_pairs = colliding_pairs

        # Deliberately not p.stepSimulation(): this is a pure kinematic pose
        # viewer, not a physics sim, and stepSimulation() runs the actual
        # dynamics/contact-resolution solver — with self-collision enabled,
        # that fights resetJointState()'s override every frame (the solver
        # pushing overlapping links apart, then getting overridden, then
        # doing it again), which looked like the model randomly popping
        # around (found 2026-09-16). performCollisionDetection() alone
        # updates collision queries without running that solver.
        time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
