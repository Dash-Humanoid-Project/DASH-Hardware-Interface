#pragma once
#include <cmath>
#include <cstring>

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

struct Mat4 {
    double m[4][4];

    static Mat4 identity() {
        Mat4 r;
        memset(r.m, 0, sizeof(r.m));
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0;
        return r;
    }

    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                r.m[i][j] = 0;
                for (int k = 0; k < 4; k++)
                    r.m[i][j] += m[i][k] * b.m[k][j];
            }
        return r;
    }
};

struct Vec3 {
    double x, y, z;

    Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

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

} // namespace LeftLeg
