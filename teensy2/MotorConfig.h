#pragma once
#include <string>

// Describes one physical actuator within a Teensy's CAN bus network.
// bus_idx  : CAN bus index on this Teensy (0 = CAN1, 1 = CAN2)
// node_idx : ODrive slot within that bus (0-indexed, matching CAN_ORDER_ID in Param.h)
// turns_per_rad : gear ratio conversion factor; turns = rad * turns_per_rad
//                 For a direct-drive joint: turns_per_rad = 1 / (2π)
struct MotorConfig {
    std::string joint_name;
    int bus_idx;
    int node_idx;
    float turns_per_rad;
};
