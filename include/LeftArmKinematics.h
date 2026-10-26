#pragma once
#include <cmath>
#include "LimbKinematics.h"

// Forward kinematics and Jacobian for the DASH left arm.
// Derived from Dash_URDF/dash.urdf kinematic chain:
//   torso -> l_shoulder_pitch -> l_shoulder_row -> l_shoulder_yaw -> l_elbow
// (URDF spells the roll joint "l_shoulder_row" — a typo, same as the leg's
// "l_hip_row" — code uses the corrected "l_shoulder_roll" for the joint
// name, matching LeftLegKinematics.h's precedent for l_hip_roll.)
//
// Unlike the leg (5 motors, only 4 make it into the Cartesian chain — the
// ankle is excluded), the arm has exactly 4 motors and all 4 are the
// LimbKinematics chain: there's no wrist/hand joint beyond the elbow in the
// URDF, so the end-effector is the elbow joint's own output frame (the
// proximal end of l_forearm), not a further hand/tool offset.
//
// Assumed ODrive-to-joint mapping (Teensy 3):
//   odrv10 (CAN1, node 0) = l_shoulder_pitch
//   odrv11 (CAN1, node 1) = l_shoulder_roll
//   odrv12 (CAN2, node 0) = l_shoulder_yaw
//   odrv13 (CAN2, node 1) = l_elbow
//
// All joint angles are in radians. All positions are in meters.

namespace LeftArm {

// Vec3/Mat4 and the generic transform builders/Jacobian math all live in
// LimbKin now (shared with RightArmKinematics.h etc.) - aliased back in so
// every existing LeftArm::Vec3/LeftArm::translation/... call site keeps
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
    // l_shoulder_pitch: torso -> l_prox_shoulder
    Mat4 T = translation(0.02, 0.183, 0.30)
           * rotRPY(1.57079633, 0.0, 0.0)
           * rotX(q[0]);

    // l_shoulder_row: l_prox_shoulder -> l_dist_shoulder
    T = T * translation(0.0, 0.0, 0.0)
          * rotRPY(0.0, 1.57079633, 0.0)
          * rotX(q[1]);

    // l_shoulder_yaw: l_dist_shoulder -> l_upper_arm
    T = T * translation(0.012, 0.0, 0.01)
          * rotRPY(-1.57079633, -1.57079633, 0.0)
          * rotX(q[2]);

    // l_elbow: l_upper_arm -> l_forearm (end-effector; no further offset —
    // the URDF has no wrist/hand link beyond l_forearm)
    T = T * translation(0.0, 0.0, -0.20)
          * rotRPY(1.57079633, 0.0, 3.14159265)
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
        {"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow"}
    };
    return k;
}

} // namespace LeftArm
