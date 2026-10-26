#pragma once
#include <cmath>
#include <cstring>
#include "LimbKinematics.h"

// Forward kinematics and Jacobian for the DASH left leg.
// Derived from Dash_URDF/dash.urdf kinematic chain:
//   torso -> l_hip_yaw -> l_hip_row -> l_hip_pitch -> l_knee -> (ankle)
//
// Assumed ODrive-to-joint mapping:
//   odrv0 (CAN1, node 0) = l_hip_yaw
//   odrv1 (CAN1, node 1) = l_hip_row
//   odrv2 (CAN2, node 0) = l_hip_pitch
//   odrv3 (CAN2, node 1) = l_knee
//
// All joint angles are in radians. All positions are in meters.

namespace LeftLeg {

// Vec3/Mat4 and the generic transform builders/Jacobian math all live in
// LimbKin now (shared with RightLegKinematics.h etc.) - aliased back in so
// every existing LeftLeg::Vec3/LeftLeg::translation/... call site keeps
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
    // l_hip_yaw: torso -> l_prox_hip
    Mat4 T = translation(0.0, 0.075, 0.0)
           * rotRPY(-2.70526034, 0.0, 0.0)
           * rotX(q[0]);

    // l_hip_row: l_prox_hip -> l_dist_hip
    T = T * translation(-0.05244738, 0.00280747, 0.08171345)
          * rotRPY(-1.22173048, -0.34906585, -1.57079633)
          * rotX(q[1]);

    // l_hip_pitch: l_dist_hip -> l_upper_leg
    T = T * translation(-0.005, 0.0, 0.095)
          * rotRPY(1.57079633, -1.22173048, 1.57079633)
          * rotX(q[2]);

    // l_knee: l_upper_leg -> l_lower_leg
    T = T * translation(0.0, 0.28, 0.0422)
          * rotX(q[3]);

    // End-effector: ankle (l_lower_leg -> l_foot offset)
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
        {"l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee"}
    };
    return k;
}

} // namespace LeftLeg
