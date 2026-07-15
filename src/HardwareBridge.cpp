#include "HardwareBridge.h"
#include "SimUPXtreme.h"
#include "MotorConfig.h"
#include <cmath>
#include <thread>
#include <chrono>

// turns_per_rad = gear_ratio / (2π). ODrive encoders read at the gearbox
// center (motor-side), so commanding/reporting joint-space radians requires
// scaling by the actual reduction ratio, not treating every joint as direct-drive.
static constexpr float TWO_PI_F = static_cast<float>(2.0 * M_PI);
static constexpr float TURNS_PER_RAD_10_1 = 10.0f / TWO_PI_F;  // hip_yaw/roll/pitch, knee
static constexpr float TURNS_PER_RAD_36_1 = 36.0f / TWO_PI_F;  // ankle

HardwareBridge::HardwareBridge(bool sim_mode) : sim_mode_(sim_mode)
{
    // ---- Create Teensy connections (or simulators) from SystemConfig ----
    for (int i = 0; i < config_.N_teensy; ++i) {
        if (sim_mode_) {
            teensys_.push_back(std::make_unique<SimUPXtreme>(
                config_.N_CAN_bus_lines_per_teensy[i],
                config_.N_actuator_per_CAN_bus_line,
                0.05f,  // tau = 50 ms first-order lag (set to 0 for perfect tracking)
                "Sim" + std::to_string(i + 1)
            ));
        } else {
            teensys_.push_back(std::make_unique<UPXtreme>(
                config_.teensy_IP[i],
                config_.PC_network_interface_name,
                config_.udp_port_PC_teensy[i],
                config_.N_CAN_bus_lines_per_teensy[i],
                config_.N_actuator_per_CAN_bus_line,
                "Teensy" + std::to_string(i + 1)
            ));
        }
    }

    // ---- Left leg (Teensy 1) ----
    // Bus 0: l_hip_yaw (slot 0), l_hip_roll (slot 1)
    // Bus 1: l_hip_pitch (slot 0), l_knee (slot 1)
    // Bus 2: l_ankle (slot 0) — slot 1 unused, see teensy.ino setup()
    // Position/velocity limits are placeholders derived from the actual motor
    // motion already exercised in ClosedLoopControlTest.cpp's --position sweep
    // (back-converted through the correct gear ratio, not the old mislabeled
    // "rad" amplitudes). Torque limits use the joint-space bound already used
    // intentionally in --cartesian mode. l_ankle was never swept, so its
    // limits are an arbitrary conservative guess. All of these need real
    // verification before the full range of motion is exploited for
    // standing/walking.
    std::vector<MotorConfig> left_motors = {
        {"l_hip_yaw",   0, 0, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"l_hip_roll",  0, 1, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"l_hip_pitch", 1, 0, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
        {"l_knee",      1, 1, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
        {"l_ankle",     2, 0, TURNS_PER_RAD_36_1, -0.3f,  0.3f,  1.0f, 0.5f},
    };
    left_leg_ = std::make_unique<Leg>(*teensys_[0], std::move(left_motors), "left_leg");

    // ---- Right leg: always SimUPXtreme-backed, independent of sim_mode_ ----
    // Not physically wired up yet — Sim here means "no hardware attached",
    // not "simulating a test run" (that's what --sim/sim_mode_ means for the
    // left leg above). Bypasses SystemConfig entirely: SimUPXtreme never
    // opens real sockets, so there's no IP/port to configure, and adding a
    // real Teensy slot for a leg that isn't physically there would be
    // misleading. Bus/actuator counts (3, 2) mirror the left leg's own
    // topology. Gear ratios confirmed identical to the left leg by the user
    // directly. Joint-limit clamp values are mirrored from the left leg as a
    // starting point — since this leg only ever drives a simulator, getting
    // a sign wrong here risks nothing physically, but should still be sanity
    // -checked (does --cartesian's computed EE position look like a mirror
    // image of the left leg's, not a flipped-and-wrong one) once RightLeg
    // kinematics are exercised.
    teensys_.push_back(std::make_unique<SimUPXtreme>(3, 2, 0.05f, "SimRightLeg"));
    UPXtreme& right_teensy = *teensys_.back();

    std::vector<MotorConfig> right_motors = {
        {"r_hip_yaw",   0, 0, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"r_hip_roll",  0, 1, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"r_hip_pitch", 1, 0, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
        {"r_knee",      1, 1, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
        {"r_ankle",     2, 0, TURNS_PER_RAD_36_1, -0.3f,  0.3f,  1.0f, 0.5f},
    };
    right_leg_ = std::make_unique<Leg>(right_teensy, std::move(right_motors), "right_leg");
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

void HardwareBridge::sendHeartbeat()
{
    for (auto& t : teensys_)
        t->sendHeartbeat();
}
