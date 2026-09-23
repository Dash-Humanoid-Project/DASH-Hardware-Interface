#pragma once
#include <string>

// Describes one physical actuator within a Teensy's CAN bus network.
// bus_idx  : CAN bus index on this Teensy (0 = CAN1, 1 = CAN2)
// node_idx : ODrive slot within that bus (0-indexed, matching CAN_ORDER_ID in Param.h)
// turns_per_rad : gear ratio conversion factor; turns = rad * turns_per_rad
//                 turns_per_rad = gear_ratio / (2π); for a direct-drive joint
//                 (gear_ratio = 1), turns_per_rad = 1 / (2π)
// q_min_rad, q_max_rad : joint-space position safety limits (rad)
// tau_max_nm            : joint-space torque safety limit (Nm), symmetric [-tau_max, tau_max]
// vel_max_rad_s         : joint-space velocity safety limit (rad/s), symmetric [-vel_max, vel_max]
struct MotorConfig {
    std::string joint_name;
    int bus_idx;
    int node_idx;
    float turns_per_rad;
    float q_min_rad;
    float q_max_rad;
    float tau_max_nm;
    float vel_max_rad_s;
};
