#pragma once
#include <chrono>
#include <map>
#include <string>
#include "Mode.h"
#include "HardwareBridge.h"
#include "LegController.h"
#include "PeriodicTimer.h"

// Ports the former cmd_flag_==3 joint-space impedance block from
// ClosedLoopControlTest into the Mode contract, now driving both legs (real
// left + SimUPXtreme-backed right) plus both arms (real, Teensy 3 / Teensy 4).
// Routed through LegController: qDes/kpJoint/kdJoint are sent to the ODrive
// for LOCAL tracking rather than a PC-computed open-loop torque relay — see
// LegController.h.
class ImpedanceMode : public Mode {
public:
    ImpedanceMode(HardwareBridge& bridge, LegController& left_ctrl, LegController& right_ctrl,
                  LegController& arm_ctrl, LegController& right_arm_ctrl, PeriodicTimer& loop_timer);

    const char* name() const override { return "impedance"; }
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

    std::map<std::string, float> q_d_left_, q_d_right_, q_d_arm_, q_d_right_arm_;
    std::chrono::steady_clock::time_point next_print_;
};
