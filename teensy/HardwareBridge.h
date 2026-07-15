#pragma once
#include <vector>
#include <memory>
#include "UPXtreme.h"
#include "Leg.h"
#include "SystemConfig.h"

// Top-level hardware abstraction that owns all Teensy connections and exposes
// legs by name. Application code creates one HardwareBridge, calls start(),
// then interacts exclusively through leftLeg() and rightLeg().
//
// rightLeg() is currently always SimUPXtreme-backed (see HardwareBridge.cpp)
// — the right leg isn't physically wired up yet, so this lets multi-limb
// code be built and validated in software ahead of that hardware landing.
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

    const Leg& leftLeg()  const { return *left_leg_; }
    const Leg& rightLeg() const { return *right_leg_; }

private:
    SystemConfig config_;
    std::vector<std::unique_ptr<UPXtreme>> teensys_;
    std::unique_ptr<Leg> left_leg_;
    std::unique_ptr<Leg> right_leg_;
    bool sim_mode_ = false;
    bool started_  = false;
    bool stopped_  = false;
};
