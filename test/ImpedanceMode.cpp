#include "ImpedanceMode.h"
#include <cmath>
#include <iostream>
#include <thread>
#include "TestUtils.h"

namespace {
// 1 turn = 2π rad. Commands expressed in turns (old code) become: rad = turns * TWO_PI
constexpr float TURNS_TO_RAD = static_cast<float>(2.0 * M_PI);

const std::string kLeftJoints[5]  = {"l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle"};
const std::string kRightJoints[5] = {"r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle"};

// Stiffness (Nm/rad) and damping (Nm*s/rad) per joint. Identical for both
// legs — gear ratios/limits are confirmed the same.
// hip_yaw/hip_roll: converted from tuned Nm/turn values, K_rad = K_turn / (2π).
// hip_pitch/knee/ankle: conservative placeholders pending hardware tuning.
//
// NOTE: these were tuned before HardwareBridge.cpp's turns_per_rad and
// Leg::setTorques gear-ratio fixes, AND before this mode routed through the
// ODrive's own local gains instead of a PC-computed open-loop torque. Needs
// fresh hardware tuning from a low starting point, not a straight port.
constexpr float K[5] = {5.0f/TURNS_TO_RAD, 7.5f/TURNS_TO_RAD, 2.0f, 2.0f, 2.0f};
constexpr float D[5] = {0.1f/TURNS_TO_RAD, 0.2f/TURNS_TO_RAD, 0.1f, 0.1f, 0.1f};
} // namespace

ImpedanceMode::ImpedanceMode(HardwareBridge& bridge, LegController& left_ctrl, LegController& right_ctrl,
                              PeriodicTimer& loop_timer)
    : bridge_(bridge), left_ctrl_(left_ctrl), right_ctrl_(right_ctrl), loop_timer_(loop_timer)
{}

void ImpedanceMode::onEnter()
{
    std::cout << "Waiting for encoder feedback..." << std::endl;
    sendKeepAliveFor(bridge_, std::chrono::milliseconds(500));

    std::map<std::string, float> kp_left, kd_left, qd_zero_left;
    std::map<std::string, float> kp_right, kd_right, qd_zero_right;
    for (int j = 0; j < 5; ++j) {
        kp_left[kLeftJoints[j]] = K[j];
        kd_left[kLeftJoints[j]] = D[j];
        qd_zero_left[kLeftJoints[j]] = 0.0f;
        kp_right[kRightJoints[j]] = K[j];
        kd_right[kRightJoints[j]] = D[j];
        qd_zero_right[kRightJoints[j]] = 0.0f;
    }

    left_ctrl_.updateData();
    q_d_left_ = left_ctrl_.q();  // hold at current position
    left_ctrl_.setJointGains(kp_left, kd_left);
    left_ctrl_.setJointTargets(q_d_left_, qd_zero_left);

    right_ctrl_.updateData();
    q_d_right_ = right_ctrl_.q();
    right_ctrl_.setJointGains(kp_right, kd_right);
    right_ctrl_.setJointTargets(q_d_right_, qd_zero_right);

    std::cout << "Impedance control active (both legs).\n"
              << "L Home (rad): [" << q_d_left_[kLeftJoints[0]] << ", " << q_d_left_[kLeftJoints[1]] << ", "
              << q_d_left_[kLeftJoints[2]] << ", " << q_d_left_[kLeftJoints[3]] << ", " << q_d_left_[kLeftJoints[4]] << "]\n"
              << "K (Nm/rad): [" << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3] << ", " << K[4] << "]\n"
              << "D (Nm*s/rad): [" << D[0] << ", " << D[1] << ", " << D[2] << ", " << D[3] << ", " << D[4] << "]\n"
              << "Push the leg to feel the virtual spring-damper. Ctrl+C or a mode-switch key to stop." << std::endl;

    next_print_ = std::chrono::steady_clock::now();
}

void ImpedanceMode::run()
{
    left_ctrl_.updateData();
    left_ctrl_.updateCommand();
    right_ctrl_.updateData();
    right_ctrl_.updateCommand();

    auto now = std::chrono::steady_clock::now();
    if (now >= next_print_) {
        auto ql = left_ctrl_.q();
        auto tau_l = left_ctrl_.tauEstimate();
        std::cout << "L pos_err (rad): ["
                  << (q_d_left_[kLeftJoints[0]]-ql[kLeftJoints[0]]) << ", " << (q_d_left_[kLeftJoints[1]]-ql[kLeftJoints[1]]) << ", "
                  << (q_d_left_[kLeftJoints[2]]-ql[kLeftJoints[2]]) << ", " << (q_d_left_[kLeftJoints[3]]-ql[kLeftJoints[3]]) << ", "
                  << (q_d_left_[kLeftJoints[4]]-ql[kLeftJoints[4]])
                  << "] | tau_est (Nm): ["
                  << tau_l[kLeftJoints[0]] << ", " << tau_l[kLeftJoints[1]] << ", " << tau_l[kLeftJoints[2]] << ", "
                  << tau_l[kLeftJoints[3]] << ", " << tau_l[kLeftJoints[4]] << "]"
                  << " | missed=" << loop_timer_.lastMissedTicks() << std::endl;
        next_print_ = now + std::chrono::seconds(1);
    }
}

void ImpedanceMode::onExit()
{
    std::cout << "\n=== IMPEDANCE MODE SHUTDOWN ===" << std::endl;
    // Zero gains rather than raw zero torque: local ODrive PD with kp=kd=0
    // always outputs zero regardless of qDes, so this is the
    // LegController-routed equivalent of the old "send zero torque and
    // stop" behavior.
    std::map<std::string, float> zero_left, zero_right;
    for (int j = 0; j < 5; ++j) {
        zero_left[kLeftJoints[j]] = 0.0f;
        zero_right[kRightJoints[j]] = 0.0f;
    }
    left_ctrl_.setJointGains(zero_left, zero_left);
    left_ctrl_.updateCommand();
    right_ctrl_.setJointGains(zero_right, zero_right);
    right_ctrl_.updateCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "Zero gains sent. Shutting down." << std::endl;
}
