#include "TestUtils.h"
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>

std::atomic<bool> shutdown_requested(false);

void sendKeepAliveFor(HardwareBridge& bridge, std::chrono::milliseconds duration)
{
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < duration && !shutdown_requested) {
        bridge.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void rampJointsToZero(HardwareBridge& bridge, Leg& leg, const std::vector<std::string>& joints,
                       float max_offset_rad, float ramp_duration_s, float hold_duration_s)
{
    std::cout << "Checking joints are close to zero before moving..." << std::endl;
    std::map<std::string, float> start_pos;
    for (const auto& joint : joints) {
        float pos = leg.getJointState(joint).position_rad;
        std::cout << "  " << joint << " = " << pos << " rad";
        if (std::isnan(pos) || std::fabs(pos) > max_offset_rad) {
            std::cout << "  TOO FAR FROM ZERO (limit " << max_offset_rad
                      << " rad) — refusing to move. Run calibrate_absolute_encoder's "
                      << "--check on this joint to diagnose." << std::endl;
            throw std::runtime_error(joint + " is not close enough to zero to auto-correct");
        }
        std::cout << "  OK" << std::endl;
        start_pos[joint] = pos;
    }

    std::cout << "Arming closed-loop control..." << std::endl;
    bridge.startClosedLoop();
    sendKeepAliveFor(bridge, std::chrono::milliseconds(500));

    std::cout << "Ramping to zero over " << ramp_duration_s << "s..." << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count() / 1000.0f;
        if (elapsed >= ramp_duration_s) break;
        float p = elapsed / ramp_duration_s;
        std::map<std::string, float> targets;
        for (const auto& joint : joints)
            targets[joint] = start_pos[joint] * (1.0f - p);
        leg.setPositions(targets);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Holding at zero for " << hold_duration_s << "s..." << std::endl;
    auto hold_t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - hold_t0).count() < hold_duration_s * 1000) {
        std::map<std::string, float> zero;
        for (const auto& joint : joints) zero[joint] = 0.0f;
        leg.setPositions(zero);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
