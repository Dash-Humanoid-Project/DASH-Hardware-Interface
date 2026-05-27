#pragma once
#include <vector>
#include <memory>
#include "UPXtreme.h"
#include "Leg.h"
#include "SystemConfig.h"

// Top-level hardware abstraction that owns all Teensy connections and exposes
// legs by name. Application code creates one HardwareBridge, calls start(),
// then interacts exclusively through leftLeg() and rightLeg().
class HardwareBridge {
public:
    HardwareBridge();
    ~HardwareBridge() { stop(); }

    void start();           // Start all UPXtreme receive/send threads
    void stop();            // Clean shutdown: idle all motors, join all threads

    void startClosedLoop(); // Send StartCommand to every Teensy
    void idle();            // Send IdleCommand to every Teensy

    Leg& leftLeg()  { return *left_leg_; }
    Leg& rightLeg() { return *right_leg_; }

    const Leg& leftLeg()  const { return *left_leg_; }
    const Leg& rightLeg() const { return *right_leg_; }

private:
    SystemConfig config_;
    std::vector<std::unique_ptr<UPXtreme>> teensys_;
    std::unique_ptr<Leg> left_leg_;
    std::unique_ptr<Leg> right_leg_;
    bool started_ = false;  // true only after start() is called
    bool stopped_ = false;  // guard against double-stop (explicit call + destructor)
};
