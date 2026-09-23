// Records a calibrated joint's true range of motion relative to its
// absolute-encoder zero. getJointState() reads the live calibrated position
// regardless of whether the axis is armed, and the ODrive already starts
// idle (backdrivable) on its own at boot — so this never arms closed-loop
// control at all, just samples live position feedback continuously over a
// fixed recording window while you move the joint through its full range by
// hand, tracking the min/max seen. No interactive per-limit step — just move
// it back and forth through both limits at some point during the window.
//
// This only gives a trustworthy measurement within ONE continuous power
// session — the boot-time disambiguation window (~0.3 rad, see
// StartupZeroMove.cpp) only matters at power-up, not during live tracking,
// so as long as the ODrive stays powered from process start through the
// whole recording window, the result is accurate no matter how far the real
// limits are from that window. If it loses power mid-session, re-run from
// the top.
//
// Only wired for joints that actually have absolute-encoder calibration
// today (right hip_roll, right hip_pitch).
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include "HardwareBridge.h"
#include "TestUtils.h"

namespace {
constexpr int kSampleHz = 50;
constexpr float kDefaultRecordDurationS = 15.0f;
} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <joint_name> [record_duration_s]" << std::endl;
        return 1;
    }
    std::string joint = argv[1];
    float record_duration_s = argc == 3 ? std::stof(argv[2]) : kDefaultRecordDurationS;
    if (joint.rfind("r_", 0) != 0) {
        std::cerr << "Only r_* joints are wired for absolute-encoder calibration so far." << std::endl;
        return 1;
    }

    HardwareBridge bridge(false);
    bridge.start();
    std::cout << "Sending heartbeats for 1s to let the Teensy learn our IP..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(1000));

    int result = 0;
    try {
        Leg& leg = bridge.rightLeg();

        std::cout << joint << " starting position (should be near its calibrated zero): "
                  << leg.getJointState(joint).position_rad << " rad" << std::endl;
        std::cout << "Never arming closed-loop control — the ODrive is already idle/backdrivable "
                  << "from boot. Do NOT power-cycle the board during this session, or this "
                  << "recording loses its reference to the calibrated zero." << std::endl;

        std::cout << "\nRecording for " << record_duration_s << "s — move " << joint
                  << " through its full range now (both directions)." << std::endl;

        float q_min = std::numeric_limits<float>::infinity();
        float q_max = -std::numeric_limits<float>::infinity();
        float first_pos = leg.getJointState(joint).position_rad;
        bool saw_any_change = false;
        auto t0 = std::chrono::steady_clock::now();
        auto next_print = t0 + std::chrono::seconds(1);
        while (true) {
            float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count() / 1000.0f;
            if (elapsed >= record_duration_s) break;

            float pos = leg.getJointState(joint).position_rad;
            if (!std::isnan(pos)) {
                q_min = std::min(q_min, pos);
                q_max = std::max(q_max, pos);
                if (pos != first_pos) saw_any_change = true;
            }

            if (std::chrono::steady_clock::now() >= next_print) {
                std::cout << "  t=" << static_cast<int>(elapsed) << "s  pos=" << pos
                          << " rad  [min=" << q_min << ", max=" << q_max << "]" << std::endl;
                next_print += std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kSampleHz));
        }

        if (!saw_any_change) {
            std::cout << "\nWARNING: position never changed during recording — this looks like "
                      << "frozen/stale feedback (board unpowered or disconnected?), not a real "
                      << "range measurement. Check power/connection and re-run." << std::endl;
        }

        std::cout << "\n" << joint << " range relative to calibrated zero:" << std::endl;
        std::cout << "  q_min_rad = " << q_min << std::endl;
        std::cout << "  q_max_rad = " << q_max << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        result = 1;
    }

    bridge.stop();
    return result;
}
