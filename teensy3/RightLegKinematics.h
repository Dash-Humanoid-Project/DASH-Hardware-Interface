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
        {"r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee"}
    };
    return k;
}

} // namespace RightLeg
