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
// rightLeg() is currently always SimUPXtreme-backed (see HardwareBridge.cpp)
// — the right leg isn't physically wired up yet, so this lets multi-limb
// code be built and validated in software ahead of that hardware landing.
// leftLeg(), leftArm(), and rightArm() are all real hardware (Teensy 1 / 3 / 4),
// gated by sim_mode like any other real limb.
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
