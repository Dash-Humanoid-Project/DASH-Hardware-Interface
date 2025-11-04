#pragma once

#include <string>


struct SystemConfig
{
    static constexpr int N_teensy = 3;
    // Number of CAN bus lines per Teensy
    const int N_CAN_bus_lines_per_teensy[N_teensy] = {1, 1, 1};
    // Number of actuators per CAN bus line
    const int N_actuator_per_CAN_bus_line = 1;
    // PC network interface name
    const std::string PC_network_interface_name = "enp2s0";
    // Teensy IP addresses
    const std::string teensy_IP[N_teensy] = {
        "10.176.32.33",
        "10.176.32.34",
        "10.176.32.35"
    };
    // UDP port on the PC to listen to UDP messages from Teensy
    const int udp_port_PC_teensy = 8000;
};
