#pragma once
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include "Mode.h"
#include "HardwareBridge.h"
#include "PeriodicTimer.h"

// Ports the former cmd_flag_==0 sine-sweep block from ClosedLoopControlTest
// into the Mode contract, now driving both legs (real left + SimUPXtreme
// -backed right) instead of just the left leg. l_ankle/r_ankle have no
// precedent motion profile, so both are held at their home position (no
// active sweep) rather than guessing one — same reasoning as before.
class PositionMode : public Mode {
public:
    PositionMode(HardwareBridge& bridge, Leg& left, Leg& right, PeriodicTimer& loop_timer);

    const char* name() const override { return "position"; }

    // Captures a fresh start time + home position for both legs — every
    // entry into this mode (initial or a later live-switch back into it)
    // restarts the HOLD_TIME+sine cycle from t=0, not from process start.
    void onEnter() override;

    void run() override;

    // Smooth return-to-zero over 2s for both legs, then flushes the CSV log
    // — fires whether leaving via a live mode-switch or process shutdown.
    void onExit() override;

private:
    // Computes this tick's joint-space position/velocity-feedforward targets
    // for one leg, given its joint-name prefix ("l_" or "r_") and captured
    // home position. Both legs use the identical sweep formula/amplitudes.
    static void sweepTargets(const std::string& prefix, const float hold[5],
                              float t, float phase,
                              std::map<std::string, float>& pos_rad,
                              std::map<std::string, float>& vel_ff_rad_s);

    HardwareBridge& bridge_;
    Leg& left_;
    Leg& right_;
    PeriodicTimer& loop_timer_;

    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point next_print_;
    float hold_left_[5]  = {};
    float hold_right_[5] = {};

    int i_ = 0;
    std::vector<std::chrono::duration<double>> time_log_;
    // [0..4] = left leg (hip_yaw,hip_roll,hip_pitch,knee,ankle), [5..9] = right leg, same order
    std::vector<double> pos_log_[10], vel_log_[10];
    double prev_pos_[10];
    double prev_vel_[10];
    std::vector<uint64_t> missed_log_;

    static constexpr float HOLD_TIME = 0.5f;
};
