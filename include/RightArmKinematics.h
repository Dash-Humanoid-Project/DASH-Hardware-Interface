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
        {"r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow"}
    };
    return k;
}

} // namespace RightArm
