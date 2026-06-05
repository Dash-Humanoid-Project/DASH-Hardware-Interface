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
 */

// 1 turn = 2π rad. Commands expressed in turns (old code) become: rad = turns * TWO_PI
static constexpr float TURNS_TO_RAD = static_cast<float>(2.0 * M_PI);

class ClosedLoopControl
{
public:
    explicit ClosedLoopControl(int cmd_flag = 0) : cmd_flag_(cmd_flag)
    {
        bridge_.start();
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

        // ---------- position command ----------
        if (cmd_flag_ == 0)
        {
            int i = 0;
            std::vector<std::chrono::duration<double>> time_log{};
            time_log.reserve(100000);

            // Logs for each joint (all in radians)
            std::vector<double> pos_log[2], vel_log[2];
            for (auto& v : pos_log) v.reserve(100000);
            for (auto& v : vel_log) v.reserve(100000);

            double prev_pos[2] = {-999, -999};
            double prev_vel[2] = {-999, -999};

            // Wait for initial encoder feedback
            std::cout << "Waiting for initial encoder feedback..." << std::endl;
            for (int wait_count = 0; wait_count <= 100 && !shutdown_requested; ++wait_count) {
                auto js = bridge_.leftLeg().getJointStates();
                double p0 = js["l_hip_yaw"].position_rad;
                double p1 = js["l_hip_roll"].position_rad;

                if (p0 != 0 || p1 != 0 || wait_count == 100) {
                    std::cout << "Initial positions (rad): ["
                              << p0 << ", " << p1 << "]" << std::endl;
                    std::cout << "Starting control loop... (Ctrl+C to stop)" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Hold initial position for HOLD_TIME seconds before starting sine
            auto home = bridge_.leftLeg().getJointStates();
            float hold[2] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
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

                // Position commands (radians): amplitude = 1 turn = 2π rad
                std::map<std::string, float> pos_rad, vel_ff_rad_s;
                float scale = 0.5f;

                if (t < HOLD_TIME) {
                    pos_rad = {
                        {"l_hip_yaw",  hold[0]},
                        {"l_hip_roll", hold[1]},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    };
                } else {
                    // Positions in rad (1 ODrive turn = 2π rad)
                    float q0 =  sinf(phase) * TURNS_TO_RAD;
                    float q1 = -sinf(phase) * TURNS_TO_RAD;

                    // Velocity feedforward in rad/s
                    float dphase_dt = static_cast<float>(TWO_PI / SINE_PERIOD);
                    float vf0 =  cosf(phase) * dphase_dt * TURNS_TO_RAD;
                    float vf1 = -cosf(phase) * dphase_dt * TURNS_TO_RAD;

                    pos_rad = {
                        {"l_hip_yaw",  q0},
                        {"l_hip_roll", q1},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw",  scale * vf0},
                        {"l_hip_roll", scale * vf1},
                    };
                }

                bridge_.leftLeg().setPositions(pos_rad, vel_ff_rad_s);

#ifndef ENABLE_TIME_BENCHMARK
                auto js = bridge_.leftLeg().getJointStates();
                double cp[2] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                };
                double cv[2] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                };

                // Log on every new feedback sample
                if (cp[0] != prev_pos[0] || cv[0] != prev_vel[0] ||
                    cp[1] != prev_pos[1] || cv[1] != prev_vel[1])
                {
                    time_log.push_back(std::chrono::steady_clock::now().time_since_epoch());
                    for (int j = 0; j < 2; ++j) {
                        pos_log[j].push_back(cp[j]);
                        vel_log[j].push_back(cv[j]);
                        prev_pos[j] = cp[j];
                        prev_vel[j] = cv[j];
                    }
                    i++;
                }

                // Print status once per second
                auto now = std::chrono::steady_clock::now();
                if (now >= next_print) {
                    std::cout << "t=" << t << "s"
                              << " | pos: [" << cp[0] << ", " << cp[1] << "]" << std::endl;
                    next_print = now + std::chrono::seconds(1);
                }
#endif
                // Rate-limit to ~500 Hz
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            // Safe shutdown: smoothly return to zero over 2 seconds
            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN SEQUENCE ===" << std::endl;
                std::cout << "Returning motors to home position..." << std::endl;

                auto js = bridge_.leftLeg().getJointStates();
                float curr[2] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                };
                std::cout << "Current positions (rad): ["
                          << curr[0] << ", " << curr[1] << "]" << std::endl;

                auto t0 = std::chrono::steady_clock::now();
                float dur = 2.0f;
                while (true) {
                    float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0f;
                    if (elapsed >= dur) break;
                    float p = elapsed / dur;
                    bridge_.leftLeg().setPositions({
                        {"l_hip_yaw",  curr[0] * (1.0f - p)},
                        {"l_hip_roll", curr[1] * (1.0f - p)},
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
                         << "l_hip_roll_pos_rad,l_hip_roll_vel_rad_s\n";
                for (int j = 0; j < i; ++j) {
                    log_file << time_log[j].count() << ","
                             << pos_log[0][j] << "," << vel_log[0][j] << ","
                             << pos_log[1][j] << "," << vel_log[1][j] << "\n";
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
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
            // Converted from tuned Nm/turn values: K_rad = K_turn / (2π).
            float K[2] = {5.0f/TURNS_TO_RAD, 7.5f/TURNS_TO_RAD};
            float D[2] = {0.1f/TURNS_TO_RAD, 0.2f/TURNS_TO_RAD};

            std::cout << "Waiting for encoder feedback..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            auto home = bridge_.leftLeg().getJointStates();
            double q_d[2] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
            };

            std::cout << "Impedance control active.\n"
                      << "Home (rad): [" << q_d[0] << ", " << q_d[1] << "]\n"
                      << "K (Nm/rad): [" << K[0] << ", " << K[1] << "]\n"
                      << "D (Nm*s/rad): [" << D[0] << ", " << D[1] << "]\n"
                      << "Push the leg to feel the virtual spring-damper. Ctrl+C to stop." << std::endl;

            auto imp_next_print = std::chrono::steady_clock::now();
            while (!shutdown_requested)
            {
                auto js = bridge_.leftLeg().getJointStates();
                double q[2] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                };
                double qd[2] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                };

                // τ = K*(q_d - q) + D*(0 - q̇)
                float tau[2];
                for (int j = 0; j < 2; ++j)
                    tau[j] = K[j] * static_cast<float>(q_d[j] - q[j])
                           + D[j] * static_cast<float>(0.0 - qd[j]);

                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw",  tau[0]},
                    {"l_hip_roll", tau[1]},
                });

                auto imp_now = std::chrono::steady_clock::now();
                if (imp_now >= imp_next_print) {
                    std::cout << "pos_err (rad): ["
                              << (q_d[0]-q[0]) << ", " << (q_d[1]-q[1])
                              << "] | tau (Nm): ["
                              << tau[0] << ", " << tau[1] << "]" << std::endl;
                    imp_next_print = imp_now + std::chrono::seconds(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << "Zero torque sent. Shutting down." << std::endl;
            }
        }

        // ---------- Cartesian impedance control ----------
        if (cmd_flag_ == 4)
        {
            float Kx[3] = {10.0f, 10.0f, 10.0f};  // N/m
            float Dx[3] = {0.5f, 0.5f, 0.5f};    // Ns/m
            float tau_max = 2.0f;                  // Nm

            std::cout << "Waiting for encoder feedback..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Read home position; pass zeros for missing joints so FK/Jacobian still compute
            auto home = bridge_.leftLeg().getJointStates();
            double q_home[4] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                0.0, 0.0  // l_hip_pitch and l_knee not attached
            };

            LeftLeg::Vec3 x_desired = LeftLeg::forwardKinematics(q_home);

            double J_home[3][4];
            LeftLeg::computeJacobian(q_home, J_home);

            std::cout << "Cartesian impedance control active (2-joint config — hip_pitch/knee fixed at 0).\n"
                      << "Home (rad): ["
                      << q_home[0] << ", " << q_home[1] << "]\n"
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
                    0.0, 0.0
                };
                double qd[4] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    0.0, 0.0
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

                // tau = J^T * F (only send yaw and roll)
                double tau_d[4];
                LeftLeg::jacobianTransposeMultiply(J, F, tau_d);

                auto clamp = [tau_max](double v) {
                    float f = static_cast<float>(v);
                    if (f >  tau_max) f =  tau_max;
                    if (f < -tau_max) f = -tau_max;
                    return f;
                };
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw",  clamp(tau_d[0])},
                    {"l_hip_roll", clamp(tau_d[1])},
                });

                auto cart_now = std::chrono::steady_clock::now();
                if (cart_now >= cart_next_print) {
                    std::cout << "x: [" << x_actual.x << ", " << x_actual.y << ", " << x_actual.z
                              << "] | err: [" << (x_desired.x-x_actual.x) << ", "
                              << (x_desired.y-x_actual.y) << ", " << (x_desired.z-x_actual.z) << "]"
                              << " | tau: [" << clamp(tau_d[0]) << ", " << clamp(tau_d[1]) << "]"
                              << std::endl;
                    cart_next_print = cart_now + std::chrono::seconds(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
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
                  << " | --start | --idle | --reset]" << std::endl;
        return 1;
    }

    std::string flag = argv[1];

    if (flag == "--reset") {
        std::cout << "Command type: reset (returning motors to position zero)" << std::endl;

        HardwareBridge bridge;
        bridge.start();

        std::cout << "Waiting for encoder feedback..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto js = bridge.leftLeg().getJointStates();
        float curr[2] = {
            js["l_hip_yaw"].position_rad,
            js["l_hip_roll"].position_rad,
        };
        std::cout << "Current positions (rad): ["
                  << curr[0] << ", " << curr[1] << "]"
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
                {"l_hip_yaw",  curr[0] * (1.0f - p)},
                {"l_hip_roll", curr[1] * (1.0f - p)},
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "Reset complete." << std::endl;
        bridge.stop();
        return 0;
    }
    else if (flag == "--start") {
        std::cout << "Command type: start (enabling closed-loop control)" << std::endl;
        HardwareBridge bridge;
        bridge.startClosedLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "All ODrives should now be in CLOSED_LOOP_CONTROL state." << std::endl;
        return 0;
    }
    else if (flag == "--idle") {
        std::cout << "Command type: idle (putting all ODrives into IDLE state)" << std::endl;
        HardwareBridge bridge;
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

    ClosedLoopControl ctrl(cmd_flag);
    ctrl.run();
    return 0;
}
