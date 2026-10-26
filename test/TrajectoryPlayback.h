#pragma once
#include <string>
#include <vector>
#include "HardwareBridge.h"
#include "Leg.h"

// Hand-guided record/playback for either arm's 4-joint chain
// (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow) — works with either
// arm's Leg&, given that arm's joint-name prefix ("l_" or "r_") so it can
// look up the right joint names.
//
// Workflow: `--record` idles just the arm (backdrivable, other limbs
// untouched — see Leg::idle()), samples its joint positions while you guide
// it by hand, and trims the static lead/trail so only the guided motion is
// saved — positions are saved exactly as recorded, no smoothing, so
// playback reproduces the true recorded amplitude (a moving-average pass
// was tried and removed: it attenuated brief extremes/holds, not just
// noise). `--playback` re-arms the arm, ramps from wherever it currently is
// to the recording's first sample (duration scales with distance —
// negligible if the arm hasn't moved since recording, capped for large
// gaps), then streams the recorded motion back with velocity feedforward
// computed from consecutive samples.

struct TrajectorySample {
    double t;     // seconds since the trimmed recording started (first sample is t=0)
    float q[4];   // shoulder_pitch, shoulder_roll, shoulder_yaw, elbow (rad) — which
                  // arm's joints these are is implicit in whichever prefix was used
                  // to record/play them, not stored in the sample itself.
};

// Idles `arm`, records its position at a fixed rate until any key is
// pressed (or shutdown_requested is set), then trims leading/trailing
// frames where the arm hasn't moved past a small threshold from its
// starting pose — matching the assumed "guide it through a motion and
// return it near its start" workflow. No filtering beyond the trim.
// `prefix` is the arm's joint-name prefix ("l_" or "r_").
std::vector<TrajectorySample> recordArmTrajectory(HardwareBridge& bridge, Leg& arm, const std::string& prefix);

void saveTrajectoryCSV(const std::vector<TrajectorySample>& traj, const std::string& path, const std::string& prefix);
std::vector<TrajectorySample> loadTrajectoryCSV(const std::string& path);

// Re-arms `arm`'s closed-loop control, ramps from its current position to
// traj's first sample (duration scales with how far it actually has to
// travel), then streams the trajectory back via position + finite-
// difference velocity feedforward at the originally recorded timing.
// No-ops with a message if traj is empty. `prefix` is the arm's joint-name
// prefix ("l_" or "r_") — must match whichever arm `arm` actually is.
void playArmTrajectory(HardwareBridge& bridge, Leg& arm, const std::vector<TrajectorySample>& traj,
                       const std::string& prefix);

// ---- Both arms together, same recorded/played timeline ----
// A separate type/functions rather than generalizing TrajectorySample to N
// arms: keeps the single-arm path above (already hardware-validated)
// completely untouched, at the cost of some duplication.

struct DualArmSample {
    double t;          // seconds since the trimmed recording started (first sample is t=0)
    float q_left[4];   // left arm: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow (rad)
    float q_right[4];  // right arm: same order
};

// Idles both arms, records both simultaneously (one shared timestamp per
// tick) until any key is pressed, then trims leading/trailing frames where
// NEITHER arm has moved past the threshold from its starting pose — a
// single-arm gesture during an otherwise-bimanual recording still counts as
// motion, so the static trim only trims a segment where both arms are still.
std::vector<DualArmSample> recordBothArmsTrajectory(HardwareBridge& bridge, Leg& left_arm, Leg& right_arm);

void saveDualArmTrajectoryCSV(const std::vector<DualArmSample>& traj, const std::string& path);
std::vector<DualArmSample> loadDualArmTrajectoryCSV(const std::string& path);

// Same as playArmTrajectory but drives both arms in lockstep on one shared
// timeline/index — both ramp in and start streaming recorded motion
// together, not independently timed.
void playBothArmsTrajectory(HardwareBridge& bridge, Leg& left_arm, Leg& right_arm,
                            const std::vector<DualArmSample>& traj);
