// Startup sanity move for absolute-encoder-calibrated joints: on boot, an
// OA1/RS485 absolute encoder should already read "almost zero" (no homing
// needed — see calibrate_absolute_encoder.cpp). This tool commands a short,
// smooth position ramp from wherever each calibrated joint currently reads
// to exactly 0.0 rad, so the robot settles at a precise, known pose every
// startup rather than trusting the raw encoder reading (which can carry a
// small residual error, encoder noise, or minor drift) at face value.
//
// Deliberately refuses to move any joint that isn't already close to zero:
// this is a small correction for the expected "almost zero" case, not a
// general homing/rehoming routine. If a joint reads far from zero, something
// is unexpected (a bad calibration, a joint disturbed while powered off
// beyond the encoder's disambiguation window — see the 2026-09-11 findings
// on r_hip_pitch — or a real mechanical problem) and commanding a move
// based on that reading could be a large, surprising motion. Run
// calibrate_absolute_encoder's --check on the affected joint to diagnose
// before proceeding in that case.
//
// Only wired for joints that actually have absolute-encoder calibration
// today (right hip_roll, right hip_pitch) — extend kCalibratedJoints as
// more joints get calibrated.
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "HardwareBridge.h"
#include "TestUtils.h"

namespace {

// Right-leg-only for now, matching what's actually been calibrated.
const std::vector<std::string> kCalibratedJoints = {"r_hip_roll", "r_hip_pitch"};

// Beyond this, refuse to move — see the file header. ~17 degrees: sized from
// 2026-09-11 live testing on r_hip_pitch, where a move landing at 0.22 rad
// stayed safely inside the encoder's disambiguation window (no re-anchor),
// while a larger "dramatic" move triggered one. 0.3 rad sits comfortably
// above that confirmed-safe case without reaching the observed-unsafe one,
// so this only refuses when something's actually wrong (bad calibration, a
// real disturbance beyond the safe window) rather than ordinary handling.
constexpr float kMaxStartupOffsetRad = 0.3f;

constexpr float kRampDurationS = 1.5f;
constexpr float kHoldAfterS = 0.3f;

} // namespace

int main(int argc, char** argv) {
    HardwareBridge bridge(false);
    bridge.start();
    std::cout << "Sending heartbeats for 1s to let the Teensy learn our IP..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(1000));

    int result = 0;
    try {
        Leg& leg = bridge.rightLeg();

        rampJointsToZero(bridge, leg, kCalibratedJoints, kMaxStartupOffsetRad, kRampDurationS, kHoldAfterS);

        std::cout << "Final positions:" << std::endl;
        for (const auto& joint : kCalibratedJoints)
            std::cout << "  " << joint << " = " << leg.getJointState(joint).position_rad << " rad" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        result = 1;
    }

    bridge.stop();
    return result;
}
