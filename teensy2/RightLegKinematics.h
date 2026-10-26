#pragma once
#include <cmath>
#include "LimbKinematics.h"

// Forward kinematics and Jacobian for the DASH right leg.
// Derived from Dash_URDF/dash.urdf kinematic chain (r_* joints):
//   torso -> r_hip_yaw -> r_hip_row -> r_hip_pitch -> r_knee -> (ankle)
//
// These are the RIGHT leg's own URDF origin/rpy values, transcribed
// directly - NOT a sign-flip of LeftLegKinematics.h's numbers. The two
// legs' frames are mirrored in the CAD, not simply negated (e.g. hip_yaw's
// rpy is -0.43633231 here vs -2.70526034 on the left; note
// pi - 2.70526034 = 0.43633231, consistent with a mirrored frame
// convention rather than a naive sign flip).
//
// Assumed ODrive-to-joint mapping (mirrors LeftLegKinematics.h's scheme):
//   node 0 = r_hip_yaw, node 1 = r_hip_row (CAN bus 0)
//   node 0 = r_hip_pitch, node 1 = r_knee (CAN bus 1)
//
// All joint angles are in radians. All positions are in meters.

namespace RightLeg {

// Vec3/Mat4 and the generic transform builders/Jacobian math all live in
// LimbKin now (shared with LeftLegKinematics.h etc.) - aliased back in so
// every existing RightLeg::Vec3/RightLeg::translation/... call site keeps
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

// Forward kinematics: joint angles (radians) -> ankle position in torso frame (meters)
// q[0] = hip_yaw, q[1] = hip_row, q[2] = hip_pitch, q[3] = knee
inline Vec3 forwardKinematics(const double q[4]) {
    // r_hip_yaw: torso -> r_prox_hip
    Mat4 T = translation(-0.0, -0.075, -0.0)
           * rotRPY(-0.43633231, -0.0, -0.0)
           * rotX(q[0]);

    // r_hip_row: r_prox_hip -> r_dist_hip
    T = T * translation(-0.05244740, 0.00280750, -0.08171350)
          * rotRPY(1.22171619, 0.34904440, -1.57079633)
          * rotX(q[1]);

    // r_hip_pitch: r_dist_hip -> r_upper_leg
    T = T * translation(-0.005, 0.0, -0.095)
          * rotRPY(-1.57079633, 1.22175193, 1.57079633)
          * rotX(q[2]);

    // r_knee: r_upper_leg -> r_lower_leg
    T = T * translation(0.0, 0.28, -0.0422)
          * rotX(q[3]);

    // End-effector: ankle (r_lower_leg -> r_foot offset)
    T = T * translation(0.0, 0.28, 0.0);

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
        {"r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee"}
    };
    return k;
}

} // namespace RightLeg
