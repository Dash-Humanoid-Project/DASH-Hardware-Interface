#pragma once

#include <string>
#include "Param.h"

struct SystemConfig
{
    static constexpr int N_teensy = 2;
    const int N_CAN_bus_lines_per_teensy[N_teensy] = {2, 2};
    const int N_actuator_per_CAN_bus_line = 2;
    const std::string PC_network_interface_name = "enp2s0";
    const std::string teensy_IP[N_teensy] = {TEENSY1_IP, TEENSY2_IP};
    const int udp_port_PC_teensy[N_teensy] = {8000, 8001};
};
