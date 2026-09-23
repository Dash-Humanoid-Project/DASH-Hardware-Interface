#pragma once
#include <string>

// Measured state of one joint in physical (SI) units after gear-ratio conversion.
struct JointState {
    std::string joint_name;
    float position_rad;    // [rad]
    float velocity_rad_s;  // [rad/s]
};
