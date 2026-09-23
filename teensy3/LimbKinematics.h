#pragma once
#include <array>
#include <cmath>
#include <cstring>
#include <string>

// Shared value types + the kinematics injection seam used by LegController
// to stay agnostic to which limb (left leg, right leg, eventually an arm)
// it's wrapping. A plain struct of function pointers rather than an
// abstract base class: this codebase has no runtime polymorphism anywhere
// else, and which kinematics a LegController uses is decided once at
// construction, never swapped at runtime - a vtable buys nothing here.

namespace LimbKin {

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

// Generic homogeneous-transform builders used by every limb's
// forwardKinematics() to walk its own URDF chain. Limb-agnostic — they take
// no URDF-specific data, so unlike forwardKinematics() itself these don't
// need a per-limb copy.
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

// Numerical Jacobian (3x4) via central finite differences. Maps joint
// velocities (rad/s) to end-effector velocity (m/s). Generic over which
// limb's forward kinematics to differentiate - fk is that limb's own
// forwardKinematics (see e.g. LeftLegKinematics.h's computeJacobian
// wrapper, which binds this to its own forwardKinematics so its function
// pointer still matches LimbKinematics::computeJacobian's fixed signature).
inline void computeJacobian(Vec3 (*fk)(const double q[4]), const double q[4], double J[3][4]) {
    const double dq = 1e-6;

    for (int i = 0; i < 4; i++) {
        double q_plus[4]  = {q[0], q[1], q[2], q[3]};
        double q_minus[4] = {q[0], q[1], q[2], q[3]};
        q_plus[i]  += dq;
        q_minus[i] -= dq;

        Vec3 p_plus  = fk(q_plus);
        Vec3 p_minus = fk(q_minus);

        J[0][i] = (p_plus.x - p_minus.x) / (2.0 * dq);
        J[1][i] = (p_plus.y - p_minus.y) / (2.0 * dq);
        J[2][i] = (p_plus.z - p_minus.z) / (2.0 * dq);
    }
}

// tau = J^T * F  (4x1 = 4x3 * 3x1). Pure linear algebra, no limb-specific
// data - usable directly as every limb's LimbKinematics::jacobianTransposeMultiply.
inline void jacobianTransposeMultiply(const double J[3][4], const double F[3], double tau[4]) {
    for (int i = 0; i < 4; i++) {
        tau[i] = 0;
        for (int j = 0; j < 3; j++) {
            tau[i] += J[j][i] * F[j];
        }
    }
}

} // namespace LimbKin

// Which limb-specific math + joint names a LegController instance is bound
// to. One instance per limb *type* (LeftLeg, RightLeg, ...), not per Leg
// object - see LeftLegKinematics::kinematics()/RightLegKinematics::kinematics().
struct LimbKinematics {
    LimbKin::Vec3 (*forwardKinematics)(const double q[4]);
    void (*computeJacobian)(const double q[4], double J[3][4]);
    void (*jacobianTransposeMultiply)(const double J[3][4], const double F[3], double tau[4]);
    std::array<std::string, 4> joint_names;
};
