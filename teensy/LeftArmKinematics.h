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

using LimbKin::Mat4;
using LimbKin::Vec3;

inline Mat4 translation(double x, double y, double z) {
    Mat4 T = Mat4::identity();
    T.m[0][3] = x;
    T.m[1][3] = y;
    T.m[2][3] = z;
    return T;
}

inline Mat4 rotX(double a) {
    Mat4 R = Mat4::identity();
    double c = cos(a), s = sin(a);
    R.m[1][1] = c;  R.m[1][2] = -s;
    R.m[2][1] = s;  R.m[2][2] = c;
    return R;
}

inline Mat4 rotY(double a) {
    Mat4 R = Mat4::identity();
    double c = cos(a), s = sin(a);
    R.m[0][0] = c;  R.m[0][2] = s;
    R.m[2][0] = -s; R.m[2][2] = c;
    return R;
}

inline Mat4 rotZ(double a) {
    Mat4 R = Mat4::identity();
    double c = cos(a), s = sin(a);
    R.m[0][0] = c;  R.m[0][1] = -s;
    R.m[1][0] = s;  R.m[1][1] = c;
    return R;
}

// URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
// The rpy attribute is ordered as "roll pitch yaw"
inline Mat4 rotRPY(double roll, double pitch, double yaw) {
    return rotZ(yaw) * rotY(pitch) * rotX(roll);
}

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

// Numerical Jacobian (3x4) via central finite differences
// Maps joint velocities (rad/s) to end-effector velocity (m/s)
inline void computeJacobian(const double q[4], double J[3][4]) {
    const double dq = 1e-6;

    for (int i = 0; i < 4; i++) {
        double q_plus[4]  = {q[0], q[1], q[2], q[3]};
        double q_minus[4] = {q[0], q[1], q[2], q[3]};
        q_plus[i]  += dq;
        q_minus[i] -= dq;

        Vec3 p_plus  = forwardKinematics(q_plus);
        Vec3 p_minus = forwardKinematics(q_minus);

        J[0][i] = (p_plus.x - p_minus.x) / (2.0 * dq);
        J[1][i] = (p_plus.y - p_minus.y) / (2.0 * dq);
        J[2][i] = (p_plus.z - p_minus.z) / (2.0 * dq);
    }
}

// tau = J^T * F  (4x1 = 4x3 * 3x1)
inline void jacobianTransposeMultiply(const double J[3][4], const double F[3], double tau[4]) {
    for (int i = 0; i < 4; i++) {
        tau[i] = 0;
        for (int j = 0; j < 3; j++) {
            tau[i] += J[j][i] * F[j];
        }
    }
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
