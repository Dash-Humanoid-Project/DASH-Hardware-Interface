#pragma once

#include <string>

struct SystemConfig
{
    static constexpr int N_teensy = 1;
    const int N_CAN_bus_lines_per_teensy[N_teensy] = {1};
    const int N_actuator_per_CAN_bus_line = 1;
    const std::string PC_network_interface_name = "enp2s0";
    const std::string teensy_IP[N_teensy] = {"10.176.32.33"};
    const int udp_port_PC_teensy[N_teensy] = {8000};
};
