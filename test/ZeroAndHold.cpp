// Ramps every joint on all four limbs smoothly from wherever it currently
// is to 0 rad, then holds there indefinitely with compliant (low-gain)
// impedance control via LegController — i.e. "settle to a static zero pose
// and stay there," as opposed to the two existing tools that each cover
// only half of that:
//   - startup_zero_move ramps to zero the same way, but only for the two
//     calibrated right-leg joints, and releases (idles) after a 0.3s hold
//     rather than holding indefinitely.
//   - closed_loop_test --impedance holds compliantly and indefinitely, but
//     captures whatever pose the robot is ALREADY in at entry — it never
//     drives toward a new target.
//
// GAINS BELOW ARE DELIBERATELY CONSERVATIVE PLACEHOLDERS WITH NO
// HARDWARE-TUNING PRECEDENT AT ALL — same caveat as every gain value in
// ImpedanceMode.cpp, just starting even lower since this tool is meant to
// be the first thing you try. Watch the real behavior (supported/unloaded
// if at all possible on the first run) and only raise gains after
// confirming it settles safely; override via CLI so tuning doesn't need a
// rebuild. The existing per-joint position clamps (HardwareBridge.cpp's
// MotorConfig, backstopped by each Teensy's own Param.h clamp) still apply
// underneath this regardless of gain choice — the ramp duration is the
// main safety lever here, not the clamps.
//
// Gains are grouped per joint rather than flat across all 17 (2026-10-22),
// originally along two considerations — gravity moment arm (hip_roll/knee/
// ankle carry more gravity load than hip_yaw/hip_pitch/the arms, arguing
// for more stiffness) and transmission confidence (hip_roll/hip_yaw/
// hip_pitch are direct gearboxes with a confirmed ratio; knee is a
// four-bar whose ratio bug just got fixed, see HardwareBridge.cpp's
// r_knee/l_knee comments, but whose true limits are still unswept; ankle
// is a four-bar whose ratio has never been independently confirmed and
// whose limits are also unswept).
//
// Real hardware testing (2026-10-22) then showed hip_yaw/hip_pitch/ankle
// all need more stiffness than the moment-arm/transmission reasoning
// alone suggested — hip_yaw/hip_pitch split out of the arms' "standard"
// group into their own tier (same confirmed-gearbox confidence as
// hip_roll, so pushed to match it), and ankle raised to knee's tier
// despite its remaining transmission uncertainty: observed hardware
// behavior overrides the a-priori caution once it's actually been watched.
// Each group is independently overridable via CLI so tuning any one of
// them doesn't require a rebuild.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include "HardwareBridge.h"
#include "LegController.h"
#include "LeftArmKinematics.h"
#include "LeftLegKinematics.h"
#include "PeriodicTimer.h"
#include "RightArmKinematics.h"
#include "RightLegKinematics.h"
#include "TestUtils.h"

namespace {

constexpr float kDefaultRampTimeS = 4.0f;   // "not too quickly" — user-adjustable via --ramp-time

// Per-group gain defaults — see the file header for the reasoning behind
// each group's starting point. All unverified placeholders with no
// hardware-tuning precedent, same as every gain value in ImpedanceMode.cpp.
struct Gains { float kp, kd; };
constexpr Gains kDefaultArms      = {2.0f, 0.1f};   // shoulder_pitch/roll/yaw, elbow — raised 1.0->2.0
                                                     // baseline (2026-10-22, effective 2.5->5.0 post-scale)
// hip_yaw and hip_pitch were one shared "hip" group through 1.0->1.5->2.0
// (2026-10-22), but at 2.0 they diverged: hip_yaw was "getting closer" while
// hip_pitch still "moves way too easily" — split into their own groups so
// each can keep escalating independently.
constexpr Gains kDefaultHipYaw    = {5.0f, 0.25f};  // "getting closer" through 2.0->2.5->3.0->4.0 -> still
                                                     // needs more (2026-10-22), pushed to 5.0
constexpr Gains kDefaultHipPitch  = {10.0f, 0.5f};  // still not there through 3.5->5.0->6.25 — upper body
                                                     // rocking while the lower half holds still (2026-10-22)
                                                     // -> matched to hip_roll/ankle's baseline. Confirmed via
                                                     // param_test that r_hip_pitch/r_hip_roll share identical
                                                     // current/torque limits (65A soft/85A hard current, torque
                                                     // unconstrained) so there's no hidden ODrive-side ceiling
                                                     // masking this — room to keep pushing Kp/Kd here without
                                                     // hitting that first.
constexpr Gains kDefaultHipRoll   = {10.0f, 0.5f};  // overshot through 5.0->20.0->25.0 baseline (2026-10-22,
                                                     // the last two scaled to 50/62.5 effective — way more
                                                     // than intended) -> set equal to kDefaultAnkle's baseline
                                                     // instead, landing both at the same 25/1.25 effective
                                                     // value post-scale.
constexpr Gains kDefaultKnee      = {3.0f, 0.15f};  // large moment arm, four-bar (ratio now fixed) — raised
                                                     // 1.3->3.0 baseline (2026-10-22, effective 3.25->7.5
                                                     // post-scale)
constexpr Gains kDefaultAnkle     = {10.0f, 0.5f};  // raised 1.0->...->5.0->7.0, still rocking at 7.0
                                                     // (2026-10-22) -> pushed further, now well above every
                                                     // other group. Ratio still unconfirmed — observed
                                                     // behavior overrides that caution.

// 2026-10-22: pushed every group higher by the same factor, preserving the
// relative proportions already tuned between joints, rather than
// escalating one group at a time. The kDefault* constants above stay as
// the per-group-tuned baseline; this scale is the uniform "push it all
// further" lever on top of that baseline.
constexpr float kGlobalGainScale = 2.5f;  // raised 1.5->2.5 (2026-10-22): standing load-bears through the
                                           // legs, a different regime than the hanging/pendulum tuning this
                                           // was originally escalated against — needs more across the board
constexpr Gains scaled(Gains g) { return {g.kp * kGlobalGainScale, g.kd * kGlobalGainScale}; }

struct GainConfig {
    Gains arms      = scaled(kDefaultArms);
    Gains hip_yaw   = scaled(kDefaultHipYaw);
    Gains hip_pitch = scaled(kDefaultHipPitch);
    Gains hip_roll  = scaled(kDefaultHipRoll);
    Gains knee      = scaled(kDefaultKnee);
    Gains ankle     = scaled(kDefaultAnkle);
};

// How long Ctrl+C takes to fade gains to zero before idling (see the
// shutdown sequence in main()) — long enough to be a genuine fade rather
// than another abrupt step, short enough not to leave you waiting once
// you've decided to stop.
constexpr float kShutdownRampS = 1.5f;

// Scales every group by the same factor, used to fade gains toward zero on
// shutdown without changing their relative proportions to each other.
GainConfig scaleGainConfig(const GainConfig& cfg, float factor) {
    auto s = [factor](Gains g) { return Gains{g.kp * factor, g.kd * factor}; };
    GainConfig out;
    out.arms = s(cfg.arms);
    out.hip_yaw = s(cfg.hip_yaw);
    out.hip_pitch = s(cfg.hip_pitch);
    out.hip_roll = s(cfg.hip_roll);
    out.knee = s(cfg.knee);
    out.ankle = s(cfg.ankle);
    return out;
}

std::map<std::string, float> zeroMapLike(const std::map<std::string, float>& q) {
    std::map<std::string, float> z;
    for (const auto& [joint, _] : q) z[joint] = 0.0f;
    return z;
}

// Classifies a joint name into its gain group by substring match — safe
// here since "hip_roll"/"hip_yaw"/"hip_pitch"/"knee"/"ankle" don't collide
// with any other joint name in this project.
const Gains& gainsForJoint(const GainConfig& cfg, const std::string& joint) {
    if (joint.find("hip_roll") != std::string::npos) return cfg.hip_roll;
    if (joint.find("hip_yaw") != std::string::npos) return cfg.hip_yaw;
    if (joint.find("hip_pitch") != std::string::npos) return cfg.hip_pitch;
    if (joint.find("knee") != std::string::npos) return cfg.knee;
    if (joint.find("ankle") != std::string::npos) return cfg.ankle;
    return cfg.arms;
}

void signalHandler(int) { shutdown_requested = true; }

// Every joint present in q (i.e. every joint LegController::updateData()
// found on that limb's Leg) gets its group's kp/kd — Leg::setGains()
// requires an entry for every joint on the leg or it throws, so this must
// be derived from q's own keys rather than a hardcoded per-limb list (the
// kind of hardcoded list that's already a latent bug elsewhere in this
// codebase — e.g. ImpedanceMode.cpp assumes the right leg has an ankle
// joint, which it doesn't).
void setGroupGains(LegController& ctrl, const std::map<std::string, float>& q, const GainConfig& cfg) {
    std::map<std::string, float> kp_map, kd_map;
    for (const auto& [joint, _] : q) {
        const Gains& g = gainsForJoint(cfg, joint);
        kp_map[joint] = g.kp;
        kd_map[joint] = g.kd;
    }
    ctrl.setJointGains(kp_map, kd_map);
}

// Linear ramp from each joint's captured starting position to 0, reaching
// (and then holding at) exactly 0 once elapsed_s >= ramp_time_s.
std::map<std::string, float> rampedTarget(const std::map<std::string, float>& start,
                                          float elapsed_s, float ramp_time_s) {
    float p = ramp_time_s > 0.0f ? std::min(1.0f, elapsed_s / ramp_time_s) : 1.0f;
    std::map<std::string, float> target;
    for (const auto& [joint, q0] : start) target[joint] = q0 * (1.0f - p);
    return target;
}

// Instantaneous velocity of rampedTarget()'s linear ramp above — constant
// and nonzero while ramping, exactly 0 once holding. Commanding this as
// qd_des (instead of a flat 0 the whole time) stops the local damping term
// from fighting the ramp's own intended motion: LegController::updateCommand()
// feeds qd_des into the ODrive's own Kd*(qd_des - qd_actual) term, so a flat
// qd_des=0 while a joint is actually moving nets a drag torque opposing that
// motion, not just damping overshoot once it arrives.
std::map<std::string, float> rampedVelocity(const std::map<std::string, float>& start,
                                            float elapsed_s, float ramp_time_s) {
    bool ramping = ramp_time_s > 0.0f && elapsed_s < ramp_time_s;
    std::map<std::string, float> vel;
    for (const auto& [joint, q0] : start) vel[joint] = ramping ? (-q0 / ramp_time_s) : 0.0f;
    return vel;
}

// Mirrors rampedTarget()'s own 0->1 shape exactly, so gains reach full
// strength at the same moment the position ramp finishes (2026-10-22) —
// setGroupGains() used to apply the full target gains in one step at t=0,
// a stiffness discontinuity symmetric to (and likely just as responsible
// for a felt jump as) the one the shutdown fade-out above was built to fix.
float rampedGainFactor(float elapsed_s, float ramp_time_s) {
    return ramp_time_s > 0.0f ? std::min(1.0f, elapsed_s / ramp_time_s) : 1.0f;
}

} // namespace

int main(int argc, char** argv) {
    bool sim_mode = false;
    float ramp_time_s = kDefaultRampTimeS;
    float shutdown_ramp_s = kShutdownRampS;
    GainConfig gains;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--sim") {
            sim_mode = true;
        } else if (arg == "--ramp-time" && i + 1 < argc) {
            ramp_time_s = std::stof(argv[++i]);
        } else if (arg == "--shutdown-ramp" && i + 1 < argc) {
            shutdown_ramp_s = std::stof(argv[++i]);
        } else if (arg == "--kp-arms" && i + 1 < argc) {
            gains.arms.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-arms" && i + 1 < argc) {
            gains.arms.kd = std::stof(argv[++i]);
        } else if (arg == "--kp-hip-yaw" && i + 1 < argc) {
            gains.hip_yaw.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-hip-yaw" && i + 1 < argc) {
            gains.hip_yaw.kd = std::stof(argv[++i]);
        } else if (arg == "--kp-hip-pitch" && i + 1 < argc) {
            gains.hip_pitch.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-hip-pitch" && i + 1 < argc) {
            gains.hip_pitch.kd = std::stof(argv[++i]);
        } else if (arg == "--kp-hip-roll" && i + 1 < argc) {
            gains.hip_roll.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-hip-roll" && i + 1 < argc) {
            gains.hip_roll.kd = std::stof(argv[++i]);
        } else if (arg == "--kp-knee" && i + 1 < argc) {
            gains.knee.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-knee" && i + 1 < argc) {
            gains.knee.kd = std::stof(argv[++i]);
        } else if (arg == "--kp-ankle" && i + 1 < argc) {
            gains.ankle.kp = std::stof(argv[++i]);
        } else if (arg == "--kd-ankle" && i + 1 < argc) {
            gains.ankle.kd = std::stof(argv[++i]);
        } else {
            std::cerr << "Usage: " << argv[0] << " [--sim] [--ramp-time <s>] [--shutdown-ramp <s>]\n"
                      << "  [--kp-arms <Nm/rad>]      [--kd-arms <Nm*s/rad>]      (shoulder_pitch/roll/yaw, elbow)\n"
                      << "  [--kp-hip-yaw <Nm/rad>]   [--kd-hip-yaw <Nm*s/rad>]\n"
                      << "  [--kp-hip-pitch <Nm/rad>] [--kd-hip-pitch <Nm*s/rad>]\n"
                      << "  [--kp-hip-roll <Nm/rad>]  [--kd-hip-roll <Nm*s/rad>]\n"
                      << "  [--kp-knee <Nm/rad>]      [--kd-knee <Nm*s/rad>]\n"
                      << "  [--kp-ankle <Nm/rad>]     [--kd-ankle <Nm*s/rad>]\n";
            return 1;
        }
    }

    std::signal(SIGINT, signalHandler);

    std::cout << "=== ZERO AND HOLD ===\n"
              << "Ramping every joint on all four limbs to 0 rad over " << ramp_time_s << "s,\n"
              << "then holding there indefinitely via compliant (low-gain) impedance control.\n"
              << "Gains (Nm/rad, Nm*s/rad) — UNVERIFIED PLACEHOLDERS, no hardware-tuning precedent:\n"
              << "  arms (shoulder/elbow): Kp=" << gains.arms.kp << " Kd=" << gains.arms.kd << "\n"
              << "  hip_yaw:               Kp=" << gains.hip_yaw.kp << " Kd=" << gains.hip_yaw.kd << "\n"
              << "  hip_pitch:             Kp=" << gains.hip_pitch.kp << " Kd=" << gains.hip_pitch.kd << "\n"
              << "  hip_roll:              Kp=" << gains.hip_roll.kp << " Kd=" << gains.hip_roll.kd << "\n"
              << "  knee:                  Kp=" << gains.knee.kp << " Kd=" << gains.knee.kd << "\n"
              << "  ankle:                 Kp=" << gains.ankle.kp << " Kd=" << gains.ankle.kd << "\n"
              << "Confirm this settles safely before trusting it; re-run with lower --kp-*/--kd-*\n"
              << "if the motion looks wrong. Ctrl+C to stop.\n" << std::endl;

    HardwareBridge bridge(sim_mode);
    bridge.start();
    std::cout << "Sending heartbeats for 1s to let the Teensys learn our IP..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(1000));

    LegController left_ctrl(bridge.leftLeg(), LeftLeg::kinematics());
    LegController right_ctrl(bridge.rightLeg(), RightLeg::kinematics());
    LegController arm_ctrl(bridge.leftArm(), LeftArm::kinematics());
    LegController right_arm_ctrl(bridge.rightArm(), RightArm::kinematics());
    PeriodicTimer loop_timer(0.002); // 500 Hz, matching every other mode

    int result = 0;
    try {
        std::cout << "Arming closed-loop control..." << std::endl;
        bridge.startClosedLoop();
        sendKeepAliveFor(bridge, std::chrono::milliseconds(500));

        left_ctrl.updateData();
        right_ctrl.updateData();
        arm_ctrl.updateData();
        right_arm_ctrl.updateData();

        const auto start_left = left_ctrl.q();
        const auto start_right = right_ctrl.q();
        const auto start_arm = arm_ctrl.q();
        const auto start_right_arm = right_arm_ctrl.q();

        std::cout << "Starting positions (rad):" << std::endl;
        for (const auto& [joint, pos] : start_left) std::cout << "  " << joint << " = " << pos << std::endl;
        for (const auto& [joint, pos] : start_right) std::cout << "  " << joint << " = " << pos << std::endl;
        for (const auto& [joint, pos] : start_arm) std::cout << "  " << joint << " = " << pos << std::endl;
        for (const auto& [joint, pos] : start_right_arm) std::cout << "  " << joint << " = " << pos << std::endl;

        auto t0 = std::chrono::steady_clock::now();
        auto next_print = t0;
        bool announced_hold = false;

        while (!shutdown_requested) {
            float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count() / 1000.0f;

            // Gains ramp in lockstep with the position ramp (0 at t=0, full
            // strength once elapsed >= ramp_time_s) rather than snapping to
            // full stiffness in one step — see rampedGainFactor()'s comment.
            GainConfig ramped_gains = scaleGainConfig(gains, rampedGainFactor(elapsed, ramp_time_s));
            setGroupGains(left_ctrl, start_left, ramped_gains);
            setGroupGains(right_ctrl, start_right, ramped_gains);
            setGroupGains(arm_ctrl, start_arm, ramped_gains);
            setGroupGains(right_arm_ctrl, start_right_arm, ramped_gains);

            left_ctrl.updateData();
            right_ctrl.updateData();
            arm_ctrl.updateData();
            right_arm_ctrl.updateData();

            left_ctrl.setJointTargets(rampedTarget(start_left, elapsed, ramp_time_s),
                                      rampedVelocity(start_left, elapsed, ramp_time_s));
            right_ctrl.setJointTargets(rampedTarget(start_right, elapsed, ramp_time_s),
                                       rampedVelocity(start_right, elapsed, ramp_time_s));
            arm_ctrl.setJointTargets(rampedTarget(start_arm, elapsed, ramp_time_s),
                                     rampedVelocity(start_arm, elapsed, ramp_time_s));
            right_arm_ctrl.setJointTargets(rampedTarget(start_right_arm, elapsed, ramp_time_s),
                                           rampedVelocity(start_right_arm, elapsed, ramp_time_s));

            left_ctrl.updateCommand();
            right_ctrl.updateCommand();
            arm_ctrl.updateCommand();
            right_arm_ctrl.updateCommand();

            if (!announced_hold && elapsed >= ramp_time_s) {
                std::cout << "Reached 0 rad — holding indefinitely now (Ctrl+C to stop)." << std::endl;
                announced_hold = true;
            }

            auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                std::cout << (elapsed < ramp_time_s ? "Ramping" : "Holding")
                          << " | t=" << elapsed << "s"
                          << " | missed=" << loop_timer.lastMissedTicks() << std::endl;
                next_print = now + std::chrono::seconds(1);
            }

            loop_timer.wait();
        }

        // Clean Ctrl+C exit (not an exception/fault — those skip straight to
        // bridge.stop() below, same as before): fade gains to zero over
        // shutdown_ramp_s instead of cutting straight from full stiffness to
        // fully passive. Holds each joint at wherever it currently is (not
        // necessarily 0 rad, if Ctrl+C landed mid-ramp) throughout, so this
        // is a stiffness fade-out, not a position jump — letting a standing
        // robot go limp instantly at high Kp risked a sudden collapse rather
        // than settling down gently. Printed progress every ~1s so a long
        // --shutdown-ramp makes it unambiguous (from the console, not just
        // by feel) whether gains are actually fading gradually versus
        // whatever is causing a jump happening independently of this loop.
        std::cout << "\nCtrl+C received — fading gains to zero over " << shutdown_ramp_s
                  << "s before idling..." << std::endl;

        left_ctrl.updateData();
        right_ctrl.updateData();
        arm_ctrl.updateData();
        right_arm_ctrl.updateData();
        const auto hold_left = left_ctrl.q();
        const auto hold_right = right_ctrl.q();
        const auto hold_arm = arm_ctrl.q();
        const auto hold_right_arm = right_arm_ctrl.q();
        const auto zero_left = zeroMapLike(hold_left);
        const auto zero_right = zeroMapLike(hold_right);
        const auto zero_arm = zeroMapLike(hold_arm);
        const auto zero_right_arm = zeroMapLike(hold_right_arm);

        auto shutdown_t0 = std::chrono::steady_clock::now();
        auto shutdown_next_print = shutdown_t0;
        while (true) {
            float t = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - shutdown_t0).count() / 1000.0f;
            if (t >= shutdown_ramp_s) break;
            float factor = 1.0f - (t / shutdown_ramp_s);
            GainConfig faded = scaleGainConfig(gains, factor);

            auto now = std::chrono::steady_clock::now();
            if (now >= shutdown_next_print) {
                std::cout << "  fading | t=" << t << "s | gain factor=" << factor << std::endl;
                shutdown_next_print = now + std::chrono::milliseconds(500);
            }

            setGroupGains(left_ctrl, hold_left, faded);
            setGroupGains(right_ctrl, hold_right, faded);
            setGroupGains(arm_ctrl, hold_arm, faded);
            setGroupGains(right_arm_ctrl, hold_right_arm, faded);

            left_ctrl.updateData();
            right_ctrl.updateData();
            arm_ctrl.updateData();
            right_arm_ctrl.updateData();

            left_ctrl.setJointTargets(hold_left, zero_left);
            right_ctrl.setJointTargets(hold_right, zero_right);
            arm_ctrl.setJointTargets(hold_arm, zero_arm);
            right_arm_ctrl.setJointTargets(hold_right_arm, zero_right_arm);

            left_ctrl.updateCommand();
            right_ctrl.updateCommand();
            arm_ctrl.updateCommand();
            right_arm_ctrl.updateCommand();

            loop_timer.wait();
        }
        std::cout << "Gains faded to zero." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        result = 1;
    }

    std::cout << "\nShutting down." << std::endl;
    bridge.stop(); // idles every Teensy (see HardwareBridge::stop())
    return result;
}
