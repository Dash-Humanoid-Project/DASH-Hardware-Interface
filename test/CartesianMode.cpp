#include "CartesianMode.h"
#include <iostream>
#include <thread>
#include "TestUtils.h"

namespace {
const std::string kLeftJoints[5]  = {"l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle"};
const std::string kRightJoints[5] = {"r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle"};

// NOTE: tuned before the gear-ratio fixes. Needs fresh hardware tuning, not
// a straight port of these numbers. Identical for both legs.
LimbKin::Vec3 Kx() { return {1.0f, 1.0f, 1.0f}; }  // N/m
LimbKin::Vec3 Dx() { return {0.0f, 0.0f, 0.0f}; }  // Ns/m — velocity noise diagnostic
} // namespace

CartesianMode::CartesianMode(HardwareBridge& bridge, LegController& left_ctrl, LegController& right_ctrl,
                              PeriodicTimer& loop_timer)
    : bridge_(bridge), left_ctrl_(left_ctrl), right_ctrl_(right_ctrl), loop_timer_(loop_timer)
{}

void CartesianMode::onEnter()
{
    std::cout << "Waiting for encoder feedback..." << std::endl;
    sendKeepAliveFor(bridge_, std::chrono::milliseconds(500));

    // No local joint-space tracking for this mode — all authority comes
    // from the Cartesian term. Every joint (including ankle, outside the
    // Cartesian chain) gets kp=kd=0 so the ODrive outputs pure feedforward.
    std::map<std::string, float> zero_left, qd_zero_left, zero_right, qd_zero_right;
    for (int j = 0; j < 5; ++j) {
        zero_left[kLeftJoints[j]] = 0.0f; qd_zero_left[kLeftJoints[j]] = 0.0f;
        zero_right[kRightJoints[j]] = 0.0f; qd_zero_right[kRightJoints[j]] = 0.0f;
    }

    left_ctrl_.updateData();
    x_desired_left_ = left_ctrl_.p();
    std::map<std::string, float> q_home_left = left_ctrl_.q();
    left_ctrl_.setJointGains(zero_left, zero_left);
    left_ctrl_.setJointTargets(q_home_left, qd_zero_left);
    left_ctrl_.setCartesianGains(Kx(), Dx());
    left_ctrl_.setCartesianTargets(x_desired_left_);

    right_ctrl_.updateData();
    x_desired_right_ = right_ctrl_.p();
    std::map<std::string, float> q_home_right = right_ctrl_.q();
    right_ctrl_.setJointGains(zero_right, zero_right);
    right_ctrl_.setJointTargets(q_home_right, qd_zero_right);
    right_ctrl_.setCartesianGains(Kx(), Dx());
    right_ctrl_.setCartesianTargets(x_desired_right_);

    std::cout << "Cartesian impedance control active (both legs, full 4-joint chain).\n"
              << "L Home EE position (m): [" << x_desired_left_.x << ", " << x_desired_left_.y << ", " << x_desired_left_.z << "]\n"
              << "R Home EE position (m): [" << x_desired_right_.x << ", " << x_desired_right_.y << ", " << x_desired_right_.z << "]\n"
              << "Kx (N/m): [" << Kx().x << ", " << Kx().y << ", " << Kx().z << "]\n"
              << "Dx (Ns/m): [" << Dx().x << ", " << Dx().y << ", " << Dx().z << "]\n"
              << "Push the leg to feel the Cartesian spring-damper. Ctrl+C or a mode-switch key to stop." << std::endl;

    next_print_ = std::chrono::steady_clock::now();
}

void CartesianMode::run()
{
    left_ctrl_.updateData();
    left_ctrl_.updateCommand();
    right_ctrl_.updateData();
    right_ctrl_.updateCommand();

    auto now = std::chrono::steady_clock::now();
    if (now >= next_print_) {
        LimbKin::Vec3 xl = left_ctrl_.p();
        auto tau_l = left_ctrl_.tauEstimate();
        std::cout << "L x: [" << xl.x << ", " << xl.y << ", " << xl.z
                  << "] | err: [" << (x_desired_left_.x-xl.x) << ", "
                  << (x_desired_left_.y-xl.y) << ", " << (x_desired_left_.z-xl.z) << "]"
                  << " | tau_est: [" << tau_l[kLeftJoints[0]] << ", " << tau_l[kLeftJoints[1]] << ", "
                  << tau_l[kLeftJoints[2]] << ", " << tau_l[kLeftJoints[3]] << "]"
                  << " | missed=" << loop_timer_.lastMissedTicks()
                  << std::endl;
        next_print_ = now + std::chrono::seconds(1);
    }
}

void CartesianMode::onExit()
{
    std::cout << "\n=== CARTESIAN MODE SHUTDOWN ===" << std::endl;
    // Gains are already zero; also zero the Cartesian gains and
    // feedforward so the additive torque term drops out too.
    left_ctrl_.setCartesianGains({0, 0, 0}, {0, 0, 0});
    left_ctrl_.setCartesianFeedforward({0, 0, 0});
    left_ctrl_.updateCommand();
    right_ctrl_.setCartesianGains({0, 0, 0}, {0, 0, 0});
    right_ctrl_.setCartesianFeedforward({0, 0, 0});
    right_ctrl_.updateCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "Zero gains sent. Shutting down." << std::endl;
}
