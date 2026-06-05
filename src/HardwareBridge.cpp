#include "HardwareBridge.h"
#include "MotorConfig.h"
#include <cmath>
#include <thread>
#include <chrono>

// Direct-drive conversion factor: 1 turn = 2π rad → turns_per_rad = 1/(2π)
static constexpr float TURNS_PER_RAD = static_cast<float>(1.0 / (2.0 * M_PI));

HardwareBridge::HardwareBridge()
{
    // ---- Create Teensy connections from SystemConfig ----
    for (int i = 0; i < config_.N_teensy; ++i) {
        teensys_.push_back(std::make_unique<UPXtreme>(
            config_.teensy_IP[i],
            config_.PC_network_interface_name,
            config_.udp_port_PC_teensy[i],
            config_.N_CAN_bus_lines_per_teensy[i],
            config_.N_actuator_per_CAN_bus_line,
            "Teensy" + std::to_string(i + 1)
        ));
    }

    // ---- Left leg (Teensy 1) ----
    // Bus 0: l_hip_yaw (slot 0), l_hip_roll (slot 1)
    std::vector<MotorConfig> left_motors = {
        {"l_hip_yaw",   0, 0, TURNS_PER_RAD},
        {"l_hip_roll",  0, 1, TURNS_PER_RAD},
    };
    left_leg_ = std::make_unique<Leg>(*teensys_[0], std::move(left_motors), "left_leg");
}

void HardwareBridge::start()
{
    started_ = true;
    for (auto& t : teensys_)
        t->start();
}

void HardwareBridge::stop()
{
    if (stopped_) return;
    stopped_ = true;
    // Only idle if control threads were running (not for --start / --idle one-shot commands)
    if (started_) {
        idle();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (auto& t : teensys_)
        t->end();
}

void HardwareBridge::startClosedLoop()
{
    for (auto& t : teensys_)
        t->sendStartCommand();
}

void HardwareBridge::idle()
{
    for (auto& t : teensys_)
        t->sendIdleCommand();
}
