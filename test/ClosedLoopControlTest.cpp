#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <iostream>
#include <fstream>
#include <csignal>
#include <atomic>
#include <thread>
#include "HardwareBridge.h"
#include "LeftLegKinematics.h"
#include "RightLegKinematics.h"
#include "LeftArmKinematics.h"
#include "RightArmKinematics.h"
#include "PeriodicTimer.h"
#include "LegController.h"
#include "Mode.h"
#include "ModeDispatcher.h"
#include "PositionMode.h"
#include "ImpedanceMode.h"
#include "CartesianMode.h"
#include "TestUtils.h"
#include "TrajectoryPlayback.h"

#define UPXTREME_i14

#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

void signalHandler(int signum) {
    std::cout << "\n\nCtrl+C detected! Initiating safe shutdown..." << std::endl;
    shutdown_requested = true;
}

/*
 * Closed-loop control test using the HardwareBridge + Leg abstraction.
 *
 * All joint positions and velocities are in SI units (radians, rad/s).
 * The Leg class converts to ODrive turns internally using turns_per_rad.
 *
 * Left leg joint names (Teensy 1):
 *   "l_hip_yaw"   – bus 0, node 0
 *   "l_hip_roll"  – bus 0, node 1
 *   "l_hip_pitch" – bus 1, node 0
 *   "l_knee"      – bus 1, node 1
 *   "l_ankle"     – bus 2, node 0
 *
 * l_ankle has no precedent motion profile from prior commits, so the
 * --position test holds it at its home position (no active sweep) rather
 * than guessing a profile. --cartesian intentionally excludes l_ankle: the
 * URDF places l_foot at zero offset beyond the ankle joint, so it has no
 * effect on the computed end-effector position with the current kinematics.
 */

// 1 turn = 2π rad. Commands expressed in turns (old code) become: rad = turns * TWO_PI
static constexpr float TURNS_TO_RAD = static_cast<float>(2.0 * M_PI);

// --position sweep amplitudes, in true joint-space radians now that
// HardwareBridge.cpp's turns_per_rad correctly accounts for each joint's
// gear ratio (previously these amplitudes were written in "turns * 2π",
// which under the old (incorrect, direct-drive) unit conversion happened to
// produce a modest physical sweep — under the corrected units that same
// numeric amplitude is a physically enormous joint angle and immediately
// hits the safety clamp in Leg.cpp). Sized to ~85-90% of each joint's clamp
// (see the MotorConfig limits in HardwareBridge.cpp) so the sweep stays
// clear of the boundary.
static constexpr float POS_AMP_HIP_RAD            = 0.55f;  // hip_yaw/hip_roll, limit +/-0.63 rad
static constexpr float POS_AMP_HIP_PITCH_KNEE_RAD = 1.7f;   // hip_pitch/knee depth, limit -1.89 rad

class ClosedLoopControl
{
public:
    explicit ClosedLoopControl(int cmd_flag = 0, bool sim_mode = false)
        : cmd_flag_(cmd_flag), bridge_(sim_mode)
    {
        bridge_.start();

        // Self-arm rather than requiring a separate prior `--start` run: the
        // Teensy's comms-loss watchdog idles the ODrives shortly after any
        // process that isn't actively streaming commands exits, so a
        // `--start` process that arms and then exits no longer leaves the
        // robot armed for a later, separate `--position`/etc. invocation.
        //
        // Deliberately no sleep here: WATCHDOG_TIMEOUT_MS (150ms) is shorter
        // than the Teensy's own StartCommand processing time (~355ms), so
        // any PC-side wait here would let the watchdog re-idle before run()
        // ever sends a real command. Commands sent while the Teensy is still
        // busy arming are simply dropped/ignored — harmless, since the
        // stream continues once it's free.
        std::cout << "Arming closed-loop control..." << std::endl;
        bridge_.startClosedLoop();
    }

    ~ClosedLoopControl()
    {
        // stop() idles all motors and joins all threads cleanly
        bridge_.stop();
    }

    void run()
    {
        std::cout << "Running closed-loop control" << std::endl;

        // Anchored-schedule rate limiter (~500 Hz) — see PeriodicTimer.h.
        // Shared across the remaining inline modes below (--position now
        // routes through PositionMode/ModeDispatcher instead, see main()).
        PeriodicTimer loop_timer(0.002);

        // ---------- velocity command ----------
        if (cmd_flag_ == 1)
        {
            while (!shutdown_requested)
            {
                bridge_.leftLeg().setVelocities({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0}, {"l_ankle", 0},
                });
                loop_timer.wait();
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Velocity mode stopped (motors at zero velocity)." << std::endl;
            }
        }

        // ---------- torque command ----------
        if (cmd_flag_ == 2)
        {
            while (!shutdown_requested)
            {
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0}, {"l_ankle", 0},
                });
                loop_timer.wait();
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Torque mode stopped (motors at zero torque)." << std::endl;
            }
        }
    }

private:
    int cmd_flag_;
    HardwareBridge bridge_;
};


int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    std::cout << "Press Ctrl+C to safely stop and return motors to home position." << std::endl;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0]
                  << " [--position | --velocity | --torque | --impedance | --cartesian"
                  << " | --start | --idle | --reset | --record | --playback] [--sim] [--right | --both]"
                  << " (--right selects the right arm, --both records/plays both arms together"
                  << " and simultaneously, for --record/--playback; left arm alone otherwise)"
                  << std::endl;
        return 1;
    }

    // Parse --sim/--right/--both from any argument position
    bool sim_mode = false;
    bool right_arm_flag = false;
    bool both_arms_flag = false;
    std::string flag;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--sim") sim_mode = true;
        else if (std::string(argv[i]) == "--right") right_arm_flag = true;
        else if (std::string(argv[i]) == "--both") both_arms_flag = true;
        else flag = argv[i];
    }
    if (flag.empty()) {
        std::cout << "Usage: " << argv[0]
                  << " [--position | --velocity | --torque | --impedance | --cartesian"
                  << " | --start | --idle | --reset | --record | --playback] [--sim] [--right | --both]"
                  << " (--right selects the right arm, --both records/plays both arms together"
                  << " and simultaneously, for --record/--playback; left arm alone otherwise)"
                  << std::endl;
        return 1;
    }
    if (sim_mode) std::cout << "[SIM MODE] Running without hardware." << std::endl;

    if (flag == "--reset") {
        std::cout << "Command type: reset (returning motors to position zero)" << std::endl;

        HardwareBridge bridge(sim_mode);
        bridge.start();

        // Self-arm — same reasoning as ClosedLoopControl's constructor: the
        // watchdog means a prior, separate --start invocation can no longer
        // be relied on to still be in effect by the time this runs.
        std::cout << "Arming closed-loop control..." << std::endl;
        bridge.startClosedLoop();
        std::cout << "Waiting for encoder feedback..." << std::endl;
        sendKeepAliveFor(bridge, std::chrono::milliseconds(500));

        auto js = bridge.leftLeg().getJointStates();
        float curr[5] = {
            js["l_hip_yaw"].position_rad,
            js["l_hip_roll"].position_rad,
            js["l_hip_pitch"].position_rad,
            js["l_knee"].position_rad,
            js["l_ankle"].position_rad,
        };
        std::cout << "Current positions (rad): ["
                  << curr[0] << ", " << curr[1] << ", " << curr[2] << ", " << curr[3] << ", " << curr[4] << "]"
                  << std::endl;
        std::cout << "Smoothly returning to zero..." << std::endl;

        auto t0 = std::chrono::steady_clock::now();
        float dur = 2.0f;
        while (true) {
            float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count() / 1000.0f;
            if (elapsed >= dur) break;
            float p = elapsed / dur;
            bridge.leftLeg().setPositions({
                {"l_hip_yaw",   curr[0] * (1.0f - p)},
                {"l_hip_roll",  curr[1] * (1.0f - p)},
                {"l_hip_pitch", curr[2] * (1.0f - p)},
                {"l_knee",      curr[3] * (1.0f - p)},
                {"l_ankle",     curr[4] * (1.0f - p)},
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "Reset complete." << std::endl;
        bridge.stop();
        return 0;
    }
    else if (flag == "--start") {
        std::cout << "Command type: start (enabling closed-loop control)" << std::endl;
        HardwareBridge bridge(sim_mode);
        bridge.startClosedLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Diagnostic only: this process sends nothing further and exits, so
        // the Teensy's comms-loss watchdog (WATCHDOG_TIMEOUT_MS) will idle
        // the ODrives again shortly after this — expected, not a bug. Use
        // one of the --position/--velocity/--torque/--impedance/--cartesian
        // modes for actual operation; they self-arm and keep streaming.
        std::cout << "ODrives were briefly put in CLOSED_LOOP_CONTROL to verify they respond. "
                  << "This process doesn't keep streaming commands, so the comms-loss watchdog "
                  << "will idle them again shortly." << std::endl;
        return 0;
    }
    else if (flag == "--idle") {
        std::cout << "Command type: idle (putting all ODrives into IDLE state)" << std::endl;
        HardwareBridge bridge(sim_mode);
        bridge.idle();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "All ODrives should now be in IDLE state." << std::endl;
        return 0;
    }
    else if (flag == "--record") {
        HardwareBridge bridge(sim_mode);
        bridge.start();
        // Left arm keeps the original unqualified filename (existing recordings from
        // before the right arm existed live there) — right arm and both-arms each
        // get their own file rather than sharing/overwriting it.
        if (both_arms_flag) {
            std::cout << "Command type: record (hand-guide both arms together, save the trajectory)" << std::endl;
            auto traj = recordBothArmsTrajectory(bridge, bridge.leftArm(), bridge.rightArm());
            saveDualArmTrajectoryCSV(traj, "../logs/both_arms_trajectory.csv");
        } else if (right_arm_flag) {
            std::cout << "Command type: record (hand-guide the right arm, save the trajectory)" << std::endl;
            auto traj = recordArmTrajectory(bridge, bridge.rightArm(), "r_");
            saveTrajectoryCSV(traj, "../logs/arm_trajectory_right.csv", "r_");
        } else {
            std::cout << "Command type: record (hand-guide the left arm, save the trajectory)" << std::endl;
            auto traj = recordArmTrajectory(bridge, bridge.leftArm(), "l_");
            saveTrajectoryCSV(traj, "../logs/arm_trajectory.csv", "l_");
        }
        bridge.stop();
        return 0;
    }
    else if (flag == "--playback") {
        HardwareBridge bridge(sim_mode);
        bridge.start();
        if (both_arms_flag) {
            std::cout << "Command type: playback (replay the recorded both-arms trajectory)" << std::endl;
            auto traj = loadDualArmTrajectoryCSV("../logs/both_arms_trajectory.csv");
            playBothArmsTrajectory(bridge, bridge.leftArm(), bridge.rightArm(), traj);
        } else if (right_arm_flag) {
            std::cout << "Command type: playback (replay the recorded right arm trajectory)" << std::endl;
            auto traj = loadTrajectoryCSV("../logs/arm_trajectory_right.csv");
            playArmTrajectory(bridge, bridge.rightArm(), traj, "r_");
        } else {
            std::cout << "Command type: playback (replay the recorded left arm trajectory)" << std::endl;
            auto traj = loadTrajectoryCSV("../logs/arm_trajectory.csv");
            playArmTrajectory(bridge, bridge.leftArm(), traj, "l_");
        }
        bridge.stop();
        return 0;
    }
    else if (flag == "--position" || flag == "--impedance" || flag == "--cartesian") {
        std::cout << "Command type: " << flag.substr(2) << "\n";
        HardwareBridge bridge(sim_mode);
        bridge.start();

        // Self-arm — same reasoning as the one-shot branches above: the
        // watchdog means a prior, separate --start invocation can no longer
        // be relied on to still be armed by the time this runs.
        std::cout << "Arming closed-loop control..." << std::endl;
        bridge.startClosedLoop();

        LegController left_ctrl(bridge.leftLeg(), LeftLeg::kinematics());
        LegController right_ctrl(bridge.rightLeg(), RightLeg::kinematics());
        LegController arm_ctrl(bridge.leftArm(), LeftArm::kinematics());
        LegController right_arm_ctrl(bridge.rightArm(), RightArm::kinematics());
        PeriodicTimer loop_timer(0.002);

        // All three modes are always constructed, regardless of which flag
        // started the process — that's what makes live keyboard switching
        // between them work. --velocity/--torque stay outside this
        // dispatcher entirely (see below): they exercise raw ODrive axis
        // modes the Leg/LegController abstraction structurally can't
        // express, same reasoning as Phase 2.
        PositionMode position_mode(bridge, bridge.leftLeg(), bridge.rightLeg(), bridge.leftArm(),
                                    bridge.rightArm(), loop_timer);
        ImpedanceMode impedance_mode(bridge, left_ctrl, right_ctrl, arm_ctrl, right_arm_ctrl, loop_timer);
        CartesianMode cartesian_mode(bridge, left_ctrl, right_ctrl, arm_ctrl, right_arm_ctrl, loop_timer);

        std::map<char, Mode*> key_to_mode = {
            {'p', &position_mode}, {'i', &impedance_mode}, {'c', &cartesian_mode},
        };
        Mode* initial_mode = (flag == "--position")  ? static_cast<Mode*>(&position_mode)
                            : (flag == "--impedance") ? static_cast<Mode*>(&impedance_mode)
                                                       : static_cast<Mode*>(&cartesian_mode);
        ModeDispatcher dispatcher(key_to_mode, initial_mode);

        std::cout << "Live-switch keys: 'p' position, 'i' impedance, 'c' cartesian. Ctrl+C to stop." << std::endl;
        while (!shutdown_requested) {
            dispatcher.tick();
            loop_timer.wait();
        }
        dispatcher.current().onExit();

        bridge.stop();
        return 0;
    }

    int cmd_flag = 0;
    if      (flag == "--velocity")  { std::cout << "Command type: velocity\n";           cmd_flag = 1; }
    else if (flag == "--torque")    { std::cout << "Command type: torque\n";             cmd_flag = 2; }
    else {
        std::cout << "Unknown flag: " << flag << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [--position | --velocity | --torque | --impedance | --cartesian"
                  << " | --start | --idle | --reset | --record | --playback] [--sim] [--right]" << std::endl;
        return 1;
    }

    ClosedLoopControl ctrl(cmd_flag, sim_mode);
    ctrl.run();
    return 0;
}
