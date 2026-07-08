#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <csignal>
#include <atomic>
#include <thread>
#include "HardwareBridge.h"
#include "LeftLegKinematics.h"
#include "PeriodicTimer.h"

#define UPXTREME_i14

#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

// Global flag for graceful shutdown
std::atomic<bool> shutdown_requested(false);

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

// Keeps the Teensy's comms-loss watchdog satisfied during a settling wait.
// Uses the no-op Heartbeat message specifically, not a zero-torque command:
// TorqueCommand forces the Teensy into TORQUE_CONTROL mode as a side effect,
// which caused a real bug when this preceded a --position sweep (armed but
// unresponsive, stuck having switched away from POSITION_CONTROL). Heartbeat
// resets the watchdog with zero effect on control mode or motor state.
void sendKeepAliveFor(HardwareBridge& bridge, std::chrono::milliseconds duration)
{
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < duration && !shutdown_requested) {
        bridge.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

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

        auto start = std::chrono::steady_clock::now();

        // Anchored-schedule rate limiter (~500 Hz) — see PeriodicTimer.h.
        // Shared across all modes below since only one of them runs per
        // invocation.
        PeriodicTimer loop_timer(0.002);

        // ---------- position command ----------
        if (cmd_flag_ == 0)
        {
            int i = 0;
            std::vector<std::chrono::duration<double>> time_log{};
            time_log.reserve(100000);

            // Logs for each joint (all in radians)
            std::vector<double> pos_log[5], vel_log[5];
            for (auto& v : pos_log) v.reserve(100000);
            for (auto& v : vel_log) v.reserve(100000);
            std::vector<uint64_t> missed_log{};
            missed_log.reserve(100000);

            double prev_pos[5] = {-999, -999, -999, -999, -999};
            double prev_vel[5] = {-999, -999, -999, -999, -999};

            // Wait for initial encoder feedback
            std::cout << "Waiting for initial encoder feedback..." << std::endl;
            for (int wait_count = 0; wait_count <= 100 && !shutdown_requested; ++wait_count) {
                auto js = bridge_.leftLeg().getJointStates();
                double p0 = js["l_hip_yaw"].position_rad;
                double p1 = js["l_hip_roll"].position_rad;
                double p2 = js["l_hip_pitch"].position_rad;
                double p3 = js["l_knee"].position_rad;
                double p4 = js["l_ankle"].position_rad;

                if (p0 != 0 || p1 != 0 || p2 != 0 || p3 != 0 || p4 != 0 || wait_count == 100) {
                    std::cout << "Initial positions (rad): ["
                              << p0 << ", " << p1 << ", " << p2 << ", " << p3 << ", " << p4 << "]" << std::endl;
                    std::cout << "Starting control loop... (Ctrl+C to stop)" << std::endl;
                    break;
                }
                // Heartbeat keep-alive: this loop sends nothing otherwise,
                // which would let the Teensy's comms-loss watchdog trip
                // before the real sweep ever starts. Heartbeat (not a
                // zero-torque command) so it doesn't force a switch to
                // TORQUE_CONTROL right before this mode needs POSITION_CONTROL.
                bridge_.sendHeartbeat();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Hold initial position for HOLD_TIME seconds before starting sine
            auto home = bridge_.leftLeg().getJointStates();
            float hold[5] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad,
                home["l_ankle"].position_rad,
            };
            const float HOLD_TIME = 0.5f;

            // Rate-limited control loop: ~500 Hz matching Teensy feedback rate.
            // Runs until Ctrl+C.
            auto next_print = std::chrono::steady_clock::now();

            while (!shutdown_requested)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
                float t = 0.001f * duration;
                float phase = (t < HOLD_TIME) ? 0.0f : (t - HOLD_TIME) * static_cast<float>(TWO_PI / SINE_PERIOD);

                // Position commands (true joint-space radians) — see
                // POS_AMP_HIP_RAD / POS_AMP_HIP_PITCH_KNEE_RAD above
                std::map<std::string, float> pos_rad, vel_ff_rad_s;
                float scale = 0.5f;

                if (t < HOLD_TIME) {
                    pos_rad = {
                        {"l_hip_yaw",   hold[0]},
                        {"l_hip_roll",  hold[1]},
                        {"l_hip_pitch", hold[2]},
                        {"l_knee",      hold[3]},
                        {"l_ankle",     hold[4]},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                        {"l_hip_pitch", 0}, {"l_knee", 0}, {"l_ankle", 0},
                    };
                } else {
                    // Positions in true joint-space rad
                    float q0 =  sinf(phase) * POS_AMP_HIP_RAD;
                    float q1 = -sinf(phase) * POS_AMP_HIP_RAD;

                    // Velocity feedforward in rad/s
                    float dphase_dt = static_cast<float>(TWO_PI / SINE_PERIOD);
                    float vf0 =  cosf(phase) * dphase_dt * POS_AMP_HIP_RAD;
                    float vf1 = -cosf(phase) * dphase_dt * POS_AMP_HIP_RAD;

                    // hip_pitch/knee: cosine-based downward sweep (π/2 phase lag)
                    float q2 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * (sinf(phase - static_cast<float>(M_PI/2)) + 1.0f);
                    float q3 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * (sinf(phase - static_cast<float>(M_PI/2)) + 1.0f);
                    float vf2 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * cosf(phase - static_cast<float>(M_PI/2)) * dphase_dt;
                    float vf3 = -0.5f * POS_AMP_HIP_PITCH_KNEE_RAD * cosf(phase - static_cast<float>(M_PI/2)) * dphase_dt;

                    // l_ankle: no prior motion profile exists in git history, so it is
                    // held at its home position (vel_ff=0) rather than guessing a sweep.
                    pos_rad = {
                        {"l_hip_yaw",   q0},
                        {"l_hip_roll",  q1},
                        {"l_hip_pitch", q2},
                        {"l_knee",      q3},
                        {"l_ankle",     hold[4]},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw",  scale * vf0},
                        {"l_hip_roll", scale * vf1},
                        {"l_hip_pitch", scale * vf2},
                        {"l_knee",      scale * vf3},
                        {"l_ankle",     0},
                    };
                }

                bridge_.leftLeg().setPositions(pos_rad, vel_ff_rad_s);

#ifndef ENABLE_TIME_BENCHMARK
                auto js = bridge_.leftLeg().getJointStates();
                double cp[5] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad,
                    js["l_ankle"].position_rad,
                };
                double cv[5] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s,
                    js["l_ankle"].velocity_rad_s,
                };

                // Log on every new feedback sample
                bool changed = false;
                for (int j = 0; j < 5; ++j)
                    if (cp[j] != prev_pos[j] || cv[j] != prev_vel[j]) changed = true;
                if (changed)
                {
                    time_log.push_back(std::chrono::steady_clock::now().time_since_epoch());
                    for (int j = 0; j < 5; ++j) {
                        pos_log[j].push_back(cp[j]);
                        vel_log[j].push_back(cv[j]);
                        prev_pos[j] = cp[j];
                        prev_vel[j] = cv[j];
                    }
                    missed_log.push_back(loop_timer.lastMissedTicks());
                    i++;
                }

                // Print status once per second
                auto now = std::chrono::steady_clock::now();
                if (now >= next_print) {
                    std::cout << "t=" << t << "s"
                              << " | pos: [" << cp[0] << ", " << cp[1] << ", " << cp[2] << ", " << cp[3] << ", " << cp[4] << "]"
                              << " | missed=" << loop_timer.lastMissedTicks() << std::endl;
                    next_print = now + std::chrono::seconds(1);
                }
#endif
                // Rate-limit to ~500 Hz on a fixed schedule (see PeriodicTimer.h)
                loop_timer.wait();
            }

            // Safe shutdown: smoothly return to zero over 2 seconds
            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN SEQUENCE ===" << std::endl;
                std::cout << "Returning motors to home position..." << std::endl;

                auto js = bridge_.leftLeg().getJointStates();
                float curr[5] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad,
                    js["l_ankle"].position_rad,
                };
                std::cout << "Current positions (rad): ["
                          << curr[0] << ", " << curr[1] << ", " << curr[2] << ", " << curr[3] << ", " << curr[4] << "]" << std::endl;

                auto t0 = std::chrono::steady_clock::now();
                float dur = 2.0f;
                while (true) {
                    float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0f;
                    if (elapsed >= dur) break;
                    float p = elapsed / dur;
                    bridge_.leftLeg().setPositions({
                        {"l_hip_yaw",   curr[0] * (1.0f - p)},
                        {"l_hip_roll",  curr[1] * (1.0f - p)},
                        {"l_hip_pitch", curr[2] * (1.0f - p)},
                        {"l_knee",      curr[3] * (1.0f - p)},
                        {"l_ankle",     curr[4] * (1.0f - p)},
                    });
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::cout << "Motors returned to home position." << std::endl;
            }

#ifndef ENABLE_TIME_BENCHMARK
            std::cout << "Logged " << i << " measurements." << std::endl;
            std::ofstream log_file("../logs/position_measurement_log.csv");
            if (log_file.is_open()) {
                log_file << "Time,"
                         << "l_hip_yaw_pos_rad,l_hip_yaw_vel_rad_s,"
                         << "l_hip_roll_pos_rad,l_hip_roll_vel_rad_s,"
                         << "l_hip_pitch_pos_rad,l_hip_pitch_vel_rad_s,"
                         << "l_knee_pos_rad,l_knee_vel_rad_s,"
                         << "l_ankle_pos_rad,l_ankle_vel_rad_s,"
                         << "missed_ticks\n";
                for (int j = 0; j < i; ++j) {
                    log_file << time_log[j].count() << ","
                             << pos_log[0][j] << "," << vel_log[0][j] << ","
                             << pos_log[1][j] << "," << vel_log[1][j] << ","
                             << pos_log[2][j] << "," << vel_log[2][j] << ","
                             << pos_log[3][j] << "," << vel_log[3][j] << ","
                             << pos_log[4][j] << "," << vel_log[4][j] << ","
                             << missed_log[j] << "\n";
                }
                log_file.close();
            } else {
                std::cout << "Unable to open log file." << std::endl;
            }
#endif
        }

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

        // ---------- joint-space impedance control ----------
        if (cmd_flag_ == 3)
        {
            // Stiffness (Nm/rad) and damping (Nm*s/rad) per joint.
            // hip_yaw/hip_roll: converted from tuned Nm/turn values, K_rad = K_turn / (2π).
            // hip_pitch/knee/ankle: conservative placeholders pending hardware tuning.
            //
            // NOTE: these were tuned before HardwareBridge.cpp's turns_per_rad and
            // Leg::setTorques gear-ratio fixes. Both q (position error) and tau
            // (torque delivery) are now correctly scaled by the joint's real gear
            // ratio, whereas before both were off by the gear ratio in opposite
            // directions and compounded — actual physical stiffness was roughly
            // K * gear_ratio^2 (~100x for the 10:1 joints), not K. Expect this to
            // feel dramatically softer, likely too soft to hold the leg's own
            // weight against gravity (there's no gravity feedforward term here).
            // Needs fresh hardware tuning from a low starting point, not a
            // straight port of these numbers.
            float K[5] = {5.0f/TURNS_TO_RAD, 7.5f/TURNS_TO_RAD, 2.0f, 2.0f, 2.0f};
            float D[5] = {0.1f/TURNS_TO_RAD, 0.2f/TURNS_TO_RAD, 0.1f, 0.1f, 0.1f};

            std::cout << "Waiting for encoder feedback..." << std::endl;
            sendKeepAliveFor(bridge_, std::chrono::milliseconds(500));

            auto home = bridge_.leftLeg().getJointStates();
            double q_d[5] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad,
                home["l_ankle"].position_rad,
            };

            std::cout << "Impedance control active.\n"
                      << "Home (rad): [" << q_d[0] << ", " << q_d[1] << ", " << q_d[2] << ", " << q_d[3] << ", " << q_d[4] << "]\n"
                      << "K (Nm/rad): [" << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3] << ", " << K[4] << "]\n"
                      << "D (Nm*s/rad): [" << D[0] << ", " << D[1] << ", " << D[2] << ", " << D[3] << ", " << D[4] << "]\n"
                      << "Push the leg to feel the virtual spring-damper. Ctrl+C to stop." << std::endl;

            auto imp_next_print = std::chrono::steady_clock::now();
            while (!shutdown_requested)
            {
                auto js = bridge_.leftLeg().getJointStates();
                double q[5] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad,
                    js["l_ankle"].position_rad,
                };
                double qd[5] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s,
                    js["l_ankle"].velocity_rad_s,
                };

                // τ = K*(q_d - q) + D*(0 - q̇)
                float tau[5];
                for (int j = 0; j < 5; ++j)
                    tau[j] = K[j] * static_cast<float>(q_d[j] - q[j])
                           + D[j] * static_cast<float>(0.0 - qd[j]);

                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw",   tau[0]},
                    {"l_hip_roll",  tau[1]},
                    {"l_hip_pitch", tau[2]},
                    {"l_knee",      tau[3]},
                    {"l_ankle",     tau[4]},
                });

                auto imp_now = std::chrono::steady_clock::now();
                if (imp_now >= imp_next_print) {
                    std::cout << "pos_err (rad): ["
                              << (q_d[0]-q[0]) << ", " << (q_d[1]-q[1]) << ", "
                              << (q_d[2]-q[2]) << ", " << (q_d[3]-q[3]) << ", " << (q_d[4]-q[4])
                              << "] | tau (Nm): ["
                              << tau[0] << ", " << tau[1] << ", " << tau[2] << ", " << tau[3] << ", " << tau[4] << "]"
                              << " | missed=" << loop_timer.lastMissedTicks() << std::endl;
                    imp_next_print = imp_now + std::chrono::seconds(1);
                }
                loop_timer.wait();
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0}, {"l_ankle", 0},
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << "Zero torque sent. Shutting down." << std::endl;
            }
        }

        // ---------- Cartesian impedance control ----------
        if (cmd_flag_ == 4)
        {
            // NOTE: tuned before the gear-ratio fixes. Two things changed here:
            // (1) forward kinematics/Jacobian consume joint-space q, which was
            // previously over-reported by the gear ratio — the computed EE
            // position/Jacobian were physically wrong before and are only now
            // correct; (2) torque delivery through setTorques is now divided by
            // gear ratio instead of passed straight through, so the realized
            // Cartesian stiffness is much softer than before at the same Kx.
            // Needs fresh hardware tuning, not a straight port of these numbers.
            float Kx[3] = {1.0f, 1.0f, 1.0f};  // N/m
            float Dx[3] = {0.0f, 0.0f, 0.0f};  // Ns/m — velocity noise diagnostic
            float tau_max = 2.0f;                  // Nm

            std::cout << "Waiting for encoder feedback..." << std::endl;
            sendKeepAliveFor(bridge_, std::chrono::milliseconds(500));

            auto home = bridge_.leftLeg().getJointStates();
            double q_home[4] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad,
            };

            LeftLeg::Vec3 x_desired = LeftLeg::forwardKinematics(q_home);

            double J_home[3][4];
            LeftLeg::computeJacobian(q_home, J_home);

            std::cout << "Cartesian impedance control active (full 4-joint chain).\n"
                      << "Home (rad): ["
                      << q_home[0] << ", " << q_home[1] << ", " << q_home[2] << ", " << q_home[3] << "]\n"
                      << "Home EE position (m): ["
                      << x_desired.x << ", " << x_desired.y << ", " << x_desired.z << "]\n"
                      << "Kx (N/m): [" << Kx[0] << ", " << Kx[1] << ", " << Kx[2] << "]\n"
                      << "Dx (Ns/m): [" << Dx[0] << ", " << Dx[1] << ", " << Dx[2] << "]\n"
                      << "tau_max = " << tau_max << " Nm\n"
                      << "Push the leg to feel the Cartesian spring-damper. Ctrl+C to stop." << std::endl;

            auto cart_next_print = std::chrono::steady_clock::now();
            while (!shutdown_requested)
            {
                auto js = bridge_.leftLeg().getJointStates();
                double q[4] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad,
                };
                double qd[4] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s,
                };

                LeftLeg::Vec3 x_actual = LeftLeg::forwardKinematics(q);

                double J[3][4];
                LeftLeg::computeJacobian(q, J);

                // xdot = J * qdot
                double xdot[3] = {0, 0, 0};
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 4; c++)
                        xdot[r] += J[r][c] * qd[c];

                // F = K*(x_d - x) + D*(0 - xdot)
                double F[3] = {
                    Kx[0] * (x_desired.x - x_actual.x) + Dx[0] * (0 - xdot[0]),
                    Kx[1] * (x_desired.y - x_actual.y) + Dx[1] * (0 - xdot[1]),
                    Kx[2] * (x_desired.z - x_actual.z) + Dx[2] * (0 - xdot[2])
                };

                // tau = J^T * F (all 4 joints)
                double tau_d[4];
                LeftLeg::jacobianTransposeMultiply(J, F, tau_d);

                auto clamp = [tau_max](double v) {
                    float f = static_cast<float>(v);
                    if (f >  tau_max) f =  tau_max;
                    if (f < -tau_max) f = -tau_max;
                    return f;
                };
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw",   clamp(tau_d[0])},
                    {"l_hip_roll",  clamp(tau_d[1])},
                    {"l_hip_pitch", clamp(tau_d[2])},
                    {"l_knee",      clamp(tau_d[3])},
                });

                auto cart_now = std::chrono::steady_clock::now();
                if (cart_now >= cart_next_print) {
                    std::cout << "x: [" << x_actual.x << ", " << x_actual.y << ", " << x_actual.z
                              << "] | err: [" << (x_desired.x-x_actual.x) << ", "
                              << (x_desired.y-x_actual.y) << ", " << (x_desired.z-x_actual.z) << "]"
                              << " | tau: [" << clamp(tau_d[0]) << ", " << clamp(tau_d[1]) << ", "
                              << clamp(tau_d[2]) << ", " << clamp(tau_d[3]) << "]"
                              << " | missed=" << loop_timer.lastMissedTicks()
                              << std::endl;
                    cart_next_print = cart_now + std::chrono::seconds(1);
                }
                loop_timer.wait();
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0},
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << "Zero torque sent. Shutting down." << std::endl;
            }
        }
    }

    float SINE_PERIOD = 5.0f; // seconds

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
                  << " | --start | --idle | --reset] [--sim]" << std::endl;
        return 1;
    }

    // Parse --sim from any argument position
    bool sim_mode = false;
    std::string flag;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--sim") sim_mode = true;
        else flag = argv[i];
    }
    if (flag.empty()) {
        std::cout << "Usage: " << argv[0]
                  << " [--position | --velocity | --torque | --impedance | --cartesian"
                  << " | --start | --idle | --reset] [--sim]" << std::endl;
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

    int cmd_flag = 0;
    if      (flag == "--position")  { std::cout << "Command type: position\n";           cmd_flag = 0; }
    else if (flag == "--velocity")  { std::cout << "Command type: velocity\n";           cmd_flag = 1; }
    else if (flag == "--torque")    { std::cout << "Command type: torque\n";             cmd_flag = 2; }
    else if (flag == "--impedance") { std::cout << "Command type: impedance\n";          cmd_flag = 3; }
    else if (flag == "--cartesian") { std::cout << "Command type: cartesian impedance\n";cmd_flag = 4; }
    else {
        std::cout << "Unknown flag: " << flag << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [--position | --velocity | --torque | --impedance | --cartesian"
                  << " | --start | --idle | --reset]" << std::endl;
        return 1;
    }

    ClosedLoopControl ctrl(cmd_flag, sim_mode);
    ctrl.run();
    return 0;
}
