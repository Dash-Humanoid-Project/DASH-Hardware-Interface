#pragma once

#include <string>
#include "Param.h"

// index 0 = Teensy 1 (left leg), index 1 = Teensy 2 (right leg), index 2 =
// Teensy 3 (left arm), index 3 = Teensy 4 (right arm) — all real hardware,
// constructed as real UPXtreme connections in HardwareBridge, gated by
// sim_mode like any other real limb.
struct SystemConfig
{
    static constexpr int N_teensy = 4;
    const int N_CAN_bus_lines_per_teensy[N_teensy] = {3, 2, 2, 2};
    const int N_actuator_per_CAN_bus_line = 2;
    const std::string PC_network_interface_name = "enp2s0";
    const std::string teensy_IP[N_teensy] = {TEENSY1_IP, TEENSY2_IP, TEENSY3_IP, TEENSY4_IP};
    const int udp_port_PC_teensy[N_teensy] = {8000, 8001, 8002, 8003};
};
