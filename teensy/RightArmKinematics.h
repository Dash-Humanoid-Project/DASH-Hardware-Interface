#pragma once
#include <cmath>
#include "LimbKinematics.h"

// Forward kinematics and Jacobian for the DASH right arm.
// Derived from Dash_URDF/dash.urdf kinematic chain:
//   torso -> r_shoulder_pitch -> r_shoulder_row -> r_shoulder_yaw -> r_elbow
//
// These are the RIGHT arm's own URDF origin/rpy values, transcribed
// directly - NOT a sign-flip of LeftArmKinematics.h's numbers, same
// precedent as RightLegKinematics.h. (URDF spells the roll joint
// "r_shoulder_row" — a typo, same as the leg's "r_hip_row" — code uses
// the corrected "r_shoulder_roll" for the joint name.)
//
// Unlike the leg (5 motors, only 4 make it into the Cartesian chain — the
// ankle is excluded), the arm has exactly 4 motors and all 4 are the
// LimbKinematics chain: there's no wrist/hand joint beyond the elbow in the
// URDF, so the end-effector is the elbow joint's own output frame (the
// proximal end of r_forearm), not a further hand/tool offset.
//
// Assumed ODrive-to-joint mapping (Teensy 4):
//   odrv14 (CAN1, node 0) = r_shoulder_pitch
//   odrv15 (CAN1, node 1) = r_shoulder_roll
//   odrv16 (CAN2, node 0) = r_shoulder_yaw
//   odrv17 (CAN2, node 1) = r_elbow
//
// All joint angles are in radians. All positions are in meters.

namespace RightArm {

// Vec3/Mat4 and the generic transform builders/Jacobian math all live in
// LimbKin now (shared with LeftArmKinematics.h etc.) - aliased back in so
// every existing RightArm::Vec3/RightArm::translation/... call site keeps
// compiling unchanged. Only forwardKinematics() below (this limb's own URDF
// chain) and the computeJacobian() wrapper actually need to live here.
using LimbKin::Mat4;
using LimbKin::Vec3;
using LimbKin::translation;
using LimbKin::rotX;
using LimbKin::rotY;
using LimbKin::rotZ;
using LimbKin::rotRPY;
using LimbKin::jacobianTransposeMultiply;

// Forward kinematics: joint angles (radians) -> elbow-frame position in torso frame (meters)
// q[0] = shoulder_pitch, q[1] = shoulder_roll, q[2] = shoulder_yaw, q[3] = elbow
inline Vec3 forwardKinematics(const double q[4]) {
    // r_shoulder_pitch: torso -> r_prox_shoulder
    Mat4 T = translation(0.02, -0.183, 0.30)
           * rotRPY(1.57079633, 0.0, 0.0)
           * rotX(q[0]);

    // r_shoulder_row: r_prox_shoulder -> r_dist_shoulder
    T = T * translation(0.0, 0.0, 0.0)
          * rotRPY(0.0, -1.57079633, 0.0)
          * rotX(q[1]);

    // r_shoulder_yaw: r_dist_shoulder -> r_upper_arm
    T = T * translation(0.012, 0.0, -0.01)
          * rotRPY(1.57079633, 1.57079633, 0.0)
          * rotX(q[2]);

    // r_elbow: r_upper_arm -> r_forearm (end-effector; no further offset —
    // the URDF has no wrist/hand link beyond r_forearm)
    T = T * translation(0.0, 0.0, 0.20)
          * rotRPY(-1.57079633, 0.0, 3.14159265)
          * rotX(q[3]);

    return {T.m[0][3], T.m[1][3], T.m[2][3]};
}

// Binds LimbKin's generic central-difference Jacobian to this limb's own
// forwardKinematics - kept as a thin per-limb wrapper (rather than shared
// outright) because LimbKinematics::computeJacobian's function-pointer
// field has a fixed (q, J) signature with no room for an fk parameter.
inline void computeJacobian(const double q[4], double J[3][4]) {
    LimbKin::computeJacobian(&forwardKinematics, q, J);
}

// The kinematics-injection seam LegController binds to at construction -
// see LimbKinematics.h.
inline const LimbKinematics& kinematics() {
    static const LimbKinematics k{
        &forwardKinematics, &computeJacobian, &jacobianTransposeMultiply,
        {"r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow"}
    };
    return k;
}

} // namespace RightArm
