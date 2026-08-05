#pragma once

#include <string>
#include "Param.h"

// index 0 = Teensy 1 (left leg), index 1 = Teensy 3 (left arm), index 2 =
// Teensy 4 (right arm) — all real hardware, constructed as real UPXtreme
// connections in HardwareBridge. The right leg (Teensy 2) is intentionally
// NOT listed here: it's still always SimUPXtreme-backed regardless of
// sim_mode (see HardwareBridge.cpp), since it isn't wired into this
// array-driven real-connection path yet.
struct SystemConfig
{
    static constexpr int N_teensy = 3;
    const int N_CAN_bus_lines_per_teensy[N_teensy] = {3, 2, 2};
    const int N_actuator_per_CAN_bus_line = 2;
    const std::string PC_network_interface_name = "enp2s0";
    const std::string teensy_IP[N_teensy] = {TEENSY1_IP, TEENSY3_IP, TEENSY4_IP};
    const int udp_port_PC_teensy[N_teensy] = {8000, 8002, 8003};
};
