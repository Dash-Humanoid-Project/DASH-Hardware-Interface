#pragma once
#include <vector>
#include <memory>
#include "UPXtreme.h"
#include "Leg.h"
#include "SystemConfig.h"

// Top-level hardware abstraction that owns all Teensy connections and exposes
// limbs by name. Application code creates one HardwareBridge, calls start(),
// then interacts exclusively through leftLeg()/rightLeg()/leftArm()/rightArm().
//
// All four limbs (Teensy 1/2/3/4 = left leg/right leg/left arm/right arm) are
// real hardware, gated by sim_mode like any other real limb.
class HardwareBridge {
public:
    explicit HardwareBridge(bool sim_mode = false);
    ~HardwareBridge() { stop(); }

    void start();           // Start all UPXtreme receive/send threads
    void stop();            // Clean shutdown: idle all motors, join all threads

    void startClosedLoop(); // Send StartCommand to every Teensy
    void idle();            // Send IdleCommand to every Teensy
    void sendHeartbeat();   // Send no-op watchdog keep-alive to every Teensy

    Leg& leftLeg()  { return *left_leg_; }
    Leg& rightLeg() { return *right_leg_; }
    Leg& leftArm()  { return *left_arm_; }
    Leg& rightArm() { return *right_arm_; }

    const Leg& leftLeg()  const { return *left_leg_; }
    const Leg& rightLeg() const { return *right_leg_; }
    const Leg& leftArm()  const { return *left_arm_; }
    const Leg& rightArm() const { return *right_arm_; }

private:
    SystemConfig config_;
    std::vector<std::unique_ptr<UPXtreme>> teensys_;
    std::unique_ptr<Leg> left_leg_;
    std::unique_ptr<Leg> right_leg_;
    std::unique_ptr<Leg> left_arm_;
    std::unique_ptr<Leg> right_arm_;
    bool sim_mode_ = false;
    bool started_  = false;
    bool stopped_  = false;
};
