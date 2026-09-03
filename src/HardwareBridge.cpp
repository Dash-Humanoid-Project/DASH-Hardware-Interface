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

    // ---- Right leg (Teensy 2) ----
    // Bus 0: r_hip_yaw (slot 0), r_hip_roll (slot 1)
    // Bus 1: r_hip_pitch (slot 0), r_knee (slot 1)
    // r_ankle (ODRV9, reserved) is not physically installed, so unlike the
    // left leg's l_ankle there's no 5th MotorConfig entry or 3rd CAN bus
    // here (SystemConfig gives Teensy 2 only 2 bus lines). Gear ratios
    // confirmed identical to the left leg by the user directly. Joint-limit
    // clamp values are mirrored from the left leg as a starting point (same
    // placeholder caveat as the left leg above) — should be sanity-checked
    // against real right-leg motion (does --cartesian's computed EE position
    // look like a correct mirror image of the left leg's, not a flipped-and-
    // wrong one) once exercised.
    std::vector<MotorConfig> right_motors = {
        {"r_hip_yaw",   0, 0, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"r_hip_roll",  0, 1, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 1.0f},
        {"r_hip_pitch", 1, 0, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
        {"r_knee",      1, 1, TURNS_PER_RAD_10_1, -1.89f, 0.0f,  2.0f, 1.5f},
    };
    right_leg_ = std::make_unique<Leg>(*teensys_[1], std::move(right_motors), "right_leg");

    // ---- Left arm (Teensy 3) ----
    // Bus 0: l_shoulder_pitch (slot 0), l_shoulder_roll (slot 1)
    // Bus 1: l_shoulder_yaw (slot 0), l_elbow (slot 1)
    // Gear ratio confirmed 10:1 for all four joints by the user (2026-07-28),
    // same as the leg's hip/knee.
    //
    // shoulder_roll/shoulder_yaw: position stays at the original l_hip_yaw/
    // l_hip_roll-mirrored placeholder (+/-0.63 rad) — a real hand-guided
    // recording (2026-07-28, logs/arm_trajectory.csv) stayed well inside
    // this range (roll: -0.31..0.34 rad, yaw: -0.13..0.25 rad), so there's
    // no evidence yet that position needs widening.
    //
    // shoulder_pitch/elbow: an early recording repeatedly hit +/-0.63 rad —
    // shoulder_pitch reached 1.99 rad, elbow reached -0.90 rad, both during
    // ordinary careful guided motion, not an extreme motion. A hip_yaw/roll
    // limit was simply the wrong reference joint to mirror: a shoulder/elbow
    // has a much larger natural range of motion than a robot leg's hip yaw/
    // roll. Widened to the observed range plus margin (still a placeholder,
    // not a verified true mechanical limit):
    //   shoulder_pitch: observed [-0.04, 1.99] rad -> [-0.2, 2.15] rad
    //   elbow: a LATER recording (still 2026-07-28) then reached +0.41 rad,
    //   clamping again against the first pass's [-1.0, 0.15] — elbow's
    //   observed range across both recordings is [-0.90, 0.41] rad. Rather
    //   than keep chasing each new recording's edge with a tight margin,
    //   widened generously this time: [-1.0, 0.6] rad.
    //
    // vel_max_rad_s raised to 3.0 rad/s on all four arm joints (was 1.0,
    // the hip-mirrored placeholder) — same recording's finite-difference
    // vel_ff repeatedly hit +/-1.0 rad/s on every joint, including roll
    // (peak observed ~2.47 rad/s) and yaw (~2.14 rad/s), not just
    // pitch/elbow — same "wrong reference joint" reasoning, not
    // independently re-derived per joint.
    // l_shoulder_yaw: a --both bimanual recording (2026-07-30) pushed this
    // to -1.21 rad, clamping against the never-before-hit +/-0.63 rad
    // placeholder — every prior (single-arm) recording stayed well inside
    // it. Widened the min side only (observed range plus margin); the
    // positive side has never been approached, so left unchanged.
    std::vector<MotorConfig> left_arm_motors = {
        {"l_shoulder_pitch", 0, 0, TURNS_PER_RAD_10_1, -0.2f,  2.15f, 2.0f, 3.0f},
        {"l_shoulder_roll",  0, 1, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 3.0f},
        {"l_shoulder_yaw",   1, 0, TURNS_PER_RAD_10_1, -1.3f,  0.63f, 2.0f, 3.0f},
        {"l_elbow",          1, 1, TURNS_PER_RAD_10_1, -1.0f,  0.6f,  2.0f, 3.0f},
    };
    left_arm_ = std::make_unique<Leg>(*teensys_[2], std::move(left_arm_motors), "left_arm");

    // ---- Right arm (Teensy 4) ----
    // Bus 0: r_shoulder_pitch (slot 0), r_shoulder_roll (slot 1)
    // Bus 1: r_shoulder_yaw (slot 0), r_elbow (slot 1)
    // Gear ratio confirmed 10:1 for all four joints by the user (2026-07-28),
    // same as the left arm. Limits seeded directly from the left arm's own
    // current, battle-tested values above (widened multiple times from real
    // hand-guided recordings) rather than the naive hip_yaw/roll placeholder
    // — same physical arm design mirrored, so this is a much better-informed
    // starting point than the left arm got initially. Still a placeholder:
    // not independently verified against the right arm's own real range of
    // motion — expect this to need adjusting once it's actually exercised.
    // Confirmed necessary immediately: the same --both bimanual recording
    // that widened l_shoulder_yaw above also pushed r_shoulder_pitch to
    // -0.97 rad (well past the seeded -0.2 min — the left arm's own
    // recording never went that negative) and r_shoulder_yaw to 1.02 rad
    // (past the seeded +0.63 max). Widened just the sides actually hit.
    std::vector<MotorConfig> right_arm_motors = {
        {"r_shoulder_pitch", 0, 0, TURNS_PER_RAD_10_1, -1.1f,  2.15f, 2.0f, 3.0f},
        {"r_shoulder_roll",  0, 1, TURNS_PER_RAD_10_1, -0.63f, 0.63f, 2.0f, 3.0f},
        {"r_shoulder_yaw",   1, 0, TURNS_PER_RAD_10_1, -0.63f, 1.15f, 2.0f, 3.0f},
        {"r_elbow",          1, 1, TURNS_PER_RAD_10_1, -1.0f,  0.6f,  2.0f, 3.0f},
    };
    right_arm_ = std::make_unique<Leg>(*teensys_[3], std::move(right_arm_motors), "right_arm");
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
