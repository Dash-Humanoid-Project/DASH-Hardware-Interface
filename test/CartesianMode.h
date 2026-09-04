#pragma once
#include <chrono>
#include <map>
#include <string>
#include "Mode.h"
#include "HardwareBridge.h"
#include "LegController.h"
#include "LimbKinematics.h"
#include "PeriodicTimer.h"

// Ports the former cmd_flag_==4 Cartesian impedance block from
// ClosedLoopControlTest into the Mode contract, now driving both legs (real
// left + SimUPXtreme-backed right) plus both arms (real, Teensy 3 / Teensy 4).
// Routed through LegController's Cartesian terms — the J^T*F torque is
// still computed on the PC each cycle, but joint-level gains are explicitly
// zeroed and sent via SetGainsCommand — see LegController.h.
class CartesianMode : public Mode {
public:
    CartesianMode(HardwareBridge& bridge, LegController& left_ctrl, LegController& right_ctrl,
                  LegController& arm_ctrl, LegController& right_arm_ctrl, PeriodicTimer& loop_timer);

    const char* name() const override { return "cartesian"; }
    void onEnter() override;
    void run() override;
    void onExit() override;

private:
    HardwareBridge& bridge_;
    LegController& left_ctrl_;
    LegController& right_ctrl_;
    LegController& arm_ctrl_;
    LegController& right_arm_ctrl_;
    PeriodicTimer& loop_timer_;

    LimbKin::Vec3 x_desired_left_{}, x_desired_right_{}, x_desired_arm_{}, x_desired_right_arm_{};
    std::chrono::steady_clock::time_point next_print_;
};
