#pragma once
#include <array>
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
