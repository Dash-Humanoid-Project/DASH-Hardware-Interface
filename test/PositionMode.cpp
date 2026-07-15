#include "PositionMode.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include "TestUtils.h"

namespace {
#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

constexpr float SINE_PERIOD = 5.0f;

// --position sweep amplitudes, in true joint-space radians now that
// HardwareBridge.cpp's turns_per_rad correctly accounts for each joint's
// gear ratio. Sized to ~85-90% of each joint's clamp (see the MotorConfig
// limits in HardwareBridge.cpp) so the sweep stays clear of the boundary.
// Identical for both legs — gear ratios/limits are confirmed the same.
constexpr float POS_AMP_HIP_RAD            = 0.55f;  // hip_yaw/hip_roll, limit +/-0.63 rad
constexpr float POS_AMP_HIP_PITCH_KNEE_RAD = 1.7f;   // hip_pitch/knee depth, limit -1.89 rad

const char* kJointSuffixes[5] = {"hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"};
} // namespace

PositionMode::PositionMode(HardwareBridge& bridge, Leg& left, Leg& right, PeriodicTimer& loop_timer)
    : bridge_(bridge), left_(left), right_(right), loop_timer_(loop_timer)
{
    for (double& v : prev_pos_) v = -999;
    for (double& v : prev_vel_) v = -999;
}

void PositionMode::sweepTargets(const std::string& prefix, const float hold[5],
                                float t, float phase,
                                std::map<std::string, float>& pos_rad,
                                std::map<std::string, float>& vel_ff_rad_s)
{
    if (t < HOLD_TIME) {
        pos_rad = {
            {prefix + "hip_yaw",   hold[0]},
            {prefix + "hip_roll",  hold[1]},
            {prefix + "hip_pitch", hold[2]},
            {prefix + "knee",      hold[3]},
            {prefix + "ankle",     hold[4]},
        };
        vel_ff_rad_s = {
            {prefix + "hip_yaw", 0}, {prefix + "hip_roll", 0},
            {prefix + "hip_pitch", 0}, {prefix + "knee", 0}, {prefix + "ankle", 0},
        };
        return;
    }

    // Positions in true joint-space rad
    float q0 =  sinf(phase) * POS_AMP_HIP_RAD;
    float q1 = -sinf(phase) * POS_AMP_HIP_RAD;

    // Velocity feedforward in rad/s
    float dphase_dt = static_cast<float>(TWO_PI / SINE_PERIOD);
    float vf0 =  cosf(phase) * dphase_dt * POS_AMP_HIP_RAD;
    float vf1 = -cosf(phase) * dphase_dt * POS_AMP_HIP_RAD;

    // hip_pitch/knee: cosine-based downward sweep (π/2 phase lag)
    float q2 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * (sinf(phase - static_cast<float>(M_PI/2)) + 1.0f);
    float q3 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * (sinf(phase - static_cast<float>(M_PI/2)) + 1.0f);
    float vf2 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * cosf(phase - static_cast<float>(M_PI/2)) * dphase_dt;
    float vf3 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * cosf(phase - static_cast<float>(M_PI/2)) * dphase_dt;

    // ankle: no prior motion profile exists in git history, so it is
    // held at its home position (vel_ff=0) rather than guessing a sweep.
    pos_rad = {
        {prefix + "hip_yaw",   q0},
        {prefix + "hip_roll",  q1},
        {prefix + "hip_pitch", q2},
        {prefix + "knee",      q3},
        {prefix + "ankle",     hold[4]},
    };
    vel_ff_rad_s = {
        {prefix + "hip_yaw",  vf0},
        {prefix + "hip_roll", vf1},
        {prefix + "hip_pitch", vf2},
        {prefix + "knee",      vf3},
        {prefix + "ankle",     0},
    };
}

void PositionMode::onEnter()
{
    i_ = 0;
    time_log_.clear();
    for (auto& v : pos_log_) v.clear();
    for (auto& v : vel_log_) v.clear();
    missed_log_.clear();
    for (double& v : prev_pos_) v = -999;
    for (double& v : prev_vel_) v = -999;

    // Wait for initial encoder feedback (left leg drives the wait; right
    // leg's SimUPXtreme reports position immediately regardless).
    std::cout << "Waiting for initial encoder feedback..." << std::endl;
    for (int wait_count = 0; wait_count <= 100 && !shutdown_requested; ++wait_count) {
        auto js = left_.getJointStates();
        double p0 = js["l_hip_yaw"].position_rad;
        double p1 = js["l_hip_roll"].position_rad;
        double p2 = js["l_hip_pitch"].position_rad;
        double p3 = js["l_knee"].position_rad;
        double p4 = js["l_ankle"].position_rad;

        if (p0 != 0 || p1 != 0 || p2 != 0 || p3 != 0 || p4 != 0 || wait_count == 100) {
            std::cout << "Initial positions (rad): ["
                      << p0 << ", " << p1 << ", " << p2 << ", " << p3 << ", " << p4 << "]" << std::endl;
            std::cout << "Starting control loop... (Ctrl+C to stop, live-switch keys to change mode)" << std::endl;
            break;
        }
        bridge_.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto home_left = left_.getJointStates();
    hold_left_[0] = home_left["l_hip_yaw"].position_rad;
    hold_left_[1] = home_left["l_hip_roll"].position_rad;
    hold_left_[2] = home_left["l_hip_pitch"].position_rad;
    hold_left_[3] = home_left["l_knee"].position_rad;
    hold_left_[4] = home_left["l_ankle"].position_rad;

    auto home_right = right_.getJointStates();
    hold_right_[0] = home_right["r_hip_yaw"].position_rad;
    hold_right_[1] = home_right["r_hip_roll"].position_rad;
    hold_right_[2] = home_right["r_hip_pitch"].position_rad;
    hold_right_[3] = home_right["r_knee"].position_rad;
    hold_right_[4] = home_right["r_ankle"].position_rad;

    start_ = std::chrono::steady_clock::now();
    next_print_ = start_;
}

void PositionMode::run()
{
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_).count();
    float t = 0.001f * duration;
    float phase = (t < HOLD_TIME) ? 0.0f : (t - HOLD_TIME) * static_cast<float>(TWO_PI / SINE_PERIOD);

    std::map<std::string, float> pos_left, vel_left, pos_right, vel_right;
    sweepTargets("l_", hold_left_,  t, phase, pos_left,  vel_left);
    sweepTargets("r_", hold_right_, t, phase, pos_right, vel_right);

    left_.setPositions(pos_left, vel_left);
    right_.setPositions(pos_right, vel_right);

#ifndef ENABLE_TIME_BENCHMARK
    auto jl = left_.getJointStates();
    auto jr = right_.getJointStates();
    double cp[10], cv[10];
    for (int j = 0; j < 5; ++j) {
        cp[j]     = jl[std::string("l_") + kJointSuffixes[j]].position_rad;
        cv[j]     = jl[std::string("l_") + kJointSuffixes[j]].velocity_rad_s;
        cp[j + 5] = jr[std::string("r_") + kJointSuffixes[j]].position_rad;
        cv[j + 5] = jr[std::string("r_") + kJointSuffixes[j]].velocity_rad_s;
    }

    // Log on every new feedback sample (either leg)
    bool changed = false;
    for (int j = 0; j < 10; ++j)
        if (cp[j] != prev_pos_[j] || cv[j] != prev_vel_[j]) changed = true;
    if (changed) {
        time_log_.push_back(std::chrono::steady_clock::now().time_since_epoch());
        for (int j = 0; j < 10; ++j) {
            pos_log_[j].push_back(cp[j]);
            vel_log_[j].push_back(cv[j]);
            prev_pos_[j] = cp[j];
            prev_vel_[j] = cv[j];
        }
        missed_log_.push_back(loop_timer_.lastMissedTicks());
        i_++;
    }

    auto now = std::chrono::steady_clock::now();
    if (now >= next_print_) {
        std::cout << "t=" << t << "s"
                  << " | L pos: [" << cp[0] << ", " << cp[1] << ", " << cp[2] << ", " << cp[3] << ", " << cp[4] << "]"
                  << " | R pos: [" << cp[5] << ", " << cp[6] << ", " << cp[7] << ", " << cp[8] << ", " << cp[9] << "]"
                  << " | missed=" << loop_timer_.lastMissedTicks() << std::endl;
        next_print_ = now + std::chrono::seconds(1);
    }
#endif
}

void PositionMode::onExit()
{
    std::cout << "\n=== POSITION MODE SHUTDOWN ===" << std::endl;
    std::cout << "Returning motors to home position..." << std::endl;

    auto jl = left_.getJointStates();
    auto jr = right_.getJointStates();
    float curr_left[5], curr_right[5];
    for (int j = 0; j < 5; ++j) {
        curr_left[j]  = jl[std::string("l_") + kJointSuffixes[j]].position_rad;
        curr_right[j] = jr[std::string("r_") + kJointSuffixes[j]].position_rad;
    }
    std::cout << "Current L positions (rad): ["
              << curr_left[0] << ", " << curr_left[1] << ", " << curr_left[2] << ", "
              << curr_left[3] << ", " << curr_left[4] << "]" << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    float dur = 2.0f;
    while (true) {
        float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count() / 1000.0f;
        if (elapsed >= dur) break;
        float p = elapsed / dur;
        left_.setPositions({
            {"l_hip_yaw",   curr_left[0] * (1.0f - p)},
            {"l_hip_roll",  curr_left[1] * (1.0f - p)},
            {"l_hip_pitch", curr_left[2] * (1.0f - p)},
            {"l_knee",      curr_left[3] * (1.0f - p)},
            {"l_ankle",     curr_left[4] * (1.0f - p)},
        });
        right_.setPositions({
            {"r_hip_yaw",   curr_right[0] * (1.0f - p)},
            {"r_hip_roll",  curr_right[1] * (1.0f - p)},
            {"r_hip_pitch", curr_right[2] * (1.0f - p)},
            {"r_knee",      curr_right[3] * (1.0f - p)},
            {"r_ankle",     curr_right[4] * (1.0f - p)},
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Motors returned to home position." << std::endl;

#ifndef ENABLE_TIME_BENCHMARK
    std::cout << "Logged " << i_ << " measurements." << std::endl;
    std::ofstream log_file("../logs/position_measurement_log.csv");
    if (log_file.is_open()) {
        log_file << "Time,"
                 << "l_hip_yaw_pos_rad,l_hip_yaw_vel_rad_s,"
                 << "l_hip_roll_pos_rad,l_hip_roll_vel_rad_s,"
                 << "l_hip_pitch_pos_rad,l_hip_pitch_vel_rad_s,"
                 << "l_knee_pos_rad,l_knee_vel_rad_s,"
                 << "l_ankle_pos_rad,l_ankle_vel_rad_s,"
                 << "r_hip_yaw_pos_rad,r_hip_yaw_vel_rad_s,"
                 << "r_hip_roll_pos_rad,r_hip_roll_vel_rad_s,"
                 << "r_hip_pitch_pos_rad,r_hip_pitch_vel_rad_s,"
                 << "r_knee_pos_rad,r_knee_vel_rad_s,"
                 << "r_ankle_pos_rad,r_ankle_vel_rad_s,"
                 << "missed_ticks\n";
        for (int j = 0; j < i_; ++j) {
            log_file << time_log_[j].count();
            for (int k = 0; k < 10; ++k)
                log_file << "," << pos_log_[k][j] << "," << vel_log_[k][j];
            log_file << "," << missed_log_[j] << "\n";
        }
        log_file.close();
    } else {
        std::cout << "Unable to open log file." << std::endl;
    }
#endif
}
