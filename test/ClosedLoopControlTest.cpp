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
            std::vector<double> pos_log[4], vel_log[4];
            for (auto& v : pos_log) v.reserve(100000);
            for (auto& v : vel_log) v.reserve(100000);

            double prev_pos[4] = {-999, -999, -999, -999};
            double prev_vel[4] = {-999, -999, -999, -999};

            // Wait for initial encoder feedback
            // Wait up to 1 s for the first non-zero encoder packet
            std::cout << "Waiting for initial encoder feedback..." << std::endl;
            for (int wait_count = 0; wait_count <= 100 && !shutdown_requested; ++wait_count) {
                auto js = bridge_.leftLeg().getJointStates();
                double p0 = js["l_hip_yaw"].position_rad;
                double p1 = js["l_hip_roll"].position_rad;
                double p2 = js["l_hip_pitch"].position_rad;
                double p3 = js["l_knee"].position_rad;

                if (p0 != 0 || p1 != 0 || p2 != 0 || p3 != 0 || wait_count == 100) {
                    std::cout << "Initial positions (rad): ["
                              << p0 << ", " << p1 << ", " << p2 << ", " << p3 << "]" << std::endl;
                    std::cout << "Starting control loop... (Ctrl+C to stop)" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Hold initial position for HOLD_TIME seconds before starting sine
            auto home = bridge_.leftLeg().getJointStates();
            float hold[4] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad
            };
            auto r_home = bridge_.rightLeg().getJointStates();
            float r_yaw_home   = r_home["r_hip_yaw"].position_rad;
            float r_roll_home  = r_home["r_hip_roll"].position_rad;
            float r_pitch_home = r_home["r_hip_pitch"].position_rad;
            float r_knee_home  = r_home["r_knee"].position_rad;
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

                float r_yaw_cmd, r_yaw_vf, r_roll_cmd, r_roll_vf, r_pitch_cmd, r_pitch_vf, r_knee_cmd, r_knee_vf;
                if (t < HOLD_TIME) {
                    pos_rad = {
                        {"l_hip_yaw",   hold[0]},
                        {"l_hip_roll",  hold[1]},
                        {"l_hip_pitch", hold[2]},
                        {"l_knee",      hold[3]},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                        {"l_hip_pitch", 0}, {"l_knee", 0}
                    };
                    r_yaw_cmd   = r_yaw_home;   r_yaw_vf   = 0.0f;
                    r_roll_cmd  = r_roll_home;  r_roll_vf  = 0.0f;
                    r_pitch_cmd = r_pitch_home; r_pitch_vf = 0.0f;
                    r_knee_cmd  = r_knee_home;  r_knee_vf  = 0.0f;
                } else {
                    // Positions in rad (1 ODrive turn = 2π rad)
                    float q0 =  sinf(phase) * TURNS_TO_RAD;
                    float q1 = -sinf(phase) * TURNS_TO_RAD;
                    float q2 = -1.5f * (sinf(phase - static_cast<float>(M_PI)/2) + 1) * TURNS_TO_RAD;
                    float q3 = -1.5f * (sinf(phase - static_cast<float>(M_PI)/2) + 1) * TURNS_TO_RAD;

                    // Velocity feedforward in rad/s (d/dt of each position)
                    float dphase_dt = static_cast<float>(TWO_PI / SINE_PERIOD);
                    float vf0 =  cosf(phase) * dphase_dt * TURNS_TO_RAD;
                    float vf1 = -cosf(phase) * dphase_dt * TURNS_TO_RAD;
                    float vf2 = -1.5f * cosf(phase - static_cast<float>(M_PI)/2) * dphase_dt * TURNS_TO_RAD;
                    float vf3 = vf2;

                    pos_rad = {
                        {"l_hip_yaw",   q0},
                        {"l_hip_roll",  q1},
                        {"l_hip_pitch", q2},
                        {"l_knee",      q3},
                    };
                    vel_ff_rad_s = {
                        {"l_hip_yaw",   scale * vf0},
                        {"l_hip_roll",  scale * vf1},
                        {"l_hip_pitch", scale * vf2},
                        {"l_knee",      scale * vf3},
                    };
                    r_yaw_cmd   = q0;   r_yaw_vf   = scale * vf0;   // mirror l_hip_yaw
                    r_roll_cmd  = q1;   r_roll_vf  = scale * vf1;   // mirror l_hip_roll
                    r_pitch_cmd = -q2;  r_pitch_vf = -scale * vf2;  // opposite l_hip_pitch
                    r_knee_cmd  = -q3;  r_knee_vf  = -scale * vf3;  // opposite l_knee
                }

                bridge_.leftLeg().setPositions(pos_rad, vel_ff_rad_s);
                bridge_.rightLeg().setPositions(
                    {{"r_hip_yaw", r_yaw_cmd}, {"r_hip_roll", r_roll_cmd}, {"r_hip_pitch", r_pitch_cmd}, {"r_knee", r_knee_cmd}},
                    {{"r_hip_yaw", r_yaw_vf},  {"r_hip_roll", r_roll_vf},  {"r_hip_pitch", r_pitch_vf},  {"r_knee", r_knee_vf}});

#ifndef ENABLE_TIME_BENCHMARK
                auto js = bridge_.leftLeg().getJointStates();
                double cp[4] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad
                };
                double cv[4] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s
                };

                // Log on every new feedback sample
                if (cp[0] != prev_pos[0] || cv[0] != prev_vel[0] ||
                    cp[1] != prev_pos[1] || cv[1] != prev_vel[1] ||
                    cp[2] != prev_pos[2] || cv[2] != prev_vel[2] ||
                    cp[3] != prev_pos[3] || cv[3] != prev_vel[3])
                {
                    time_log.push_back(std::chrono::steady_clock::now().time_since_epoch());
                    for (int j = 0; j < 4; ++j) {
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
                    auto rjs = bridge_.rightLeg().getJointStates();
                    std::cout << "t=" << t << "s"
                              << " | L pos: [" << cp[0] << ", " << cp[1] << ", " << cp[2] << ", " << cp[3] << "]"
                              << " | R pos: [" << rjs["r_hip_yaw"].position_rad
                              << ", " << rjs["r_hip_roll"].position_rad
                              << ", " << rjs["r_hip_pitch"].position_rad
                              << ", " << rjs["r_knee"].position_rad << "]" << std::endl;
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
                float curr[4] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad
                };
                std::cout << "Current positions (rad): ["
                          << curr[0] << ", " << curr[1] << ", " << curr[2] << ", " << curr[3] << "]" << std::endl;

                auto r_curr_js    = bridge_.rightLeg().getJointStates();
                float r_yaw_curr   = r_curr_js["r_hip_yaw"].position_rad;
                float r_roll_curr  = r_curr_js["r_hip_roll"].position_rad;
                float r_pitch_curr = r_curr_js["r_hip_pitch"].position_rad;
                float r_knee_curr  = r_curr_js["r_knee"].position_rad;
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
                    });
                    bridge_.rightLeg().setPositions({
                        {"r_hip_yaw",   r_yaw_curr   * (1.0f - p)},
                        {"r_hip_roll",  r_roll_curr  * (1.0f - p)},
                        {"r_hip_pitch", r_pitch_curr * (1.0f - p)},
                        {"r_knee",      r_knee_curr  * (1.0f - p)},
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
                         << "l_knee_pos_rad,l_knee_vel_rad_s\n";
                for (int j = 0; j < i; ++j) {
                    log_file << time_log[j].count() << ","
                             << pos_log[0][j] << "," << vel_log[0][j] << ","
                             << pos_log[1][j] << "," << vel_log[1][j] << ","
                             << pos_log[2][j] << "," << vel_log[2][j] << ","
                             << pos_log[3][j] << "," << vel_log[3][j] << "\n";
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
                    {"l_hip_pitch", 0}, {"l_knee", 0}
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
                    {"l_hip_pitch", 0}, {"l_knee", 0}
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
            float K[4] = {5.0f/TURNS_TO_RAD, 7.5f/TURNS_TO_RAD,
                          4.0f/TURNS_TO_RAD, 2.5f/TURNS_TO_RAD};
            float D[4] = {0.1f/TURNS_TO_RAD, 0.2f/TURNS_TO_RAD,
                          0.1f/TURNS_TO_RAD, 0.1f/TURNS_TO_RAD};

            std::cout << "Waiting for encoder feedback..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            auto home = bridge_.leftLeg().getJointStates();
            double q_d[4] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad
            };

            std::cout << "Impedance control active.\n"
                      << "Home (rad): [" << q_d[0] << ", " << q_d[1]
                      << ", " << q_d[2] << ", " << q_d[3] << "]\n"
                      << "K (Nm/rad): [" << K[0] << ", " << K[1]
                      << ", " << K[2] << ", " << K[3] << "]\n"
                      << "D (Nm*s/rad): [" << D[0] << ", " << D[1]
                      << ", " << D[2] << ", " << D[3] << "]\n"
                      << "Push the leg to feel the virtual spring-damper. Ctrl+C to stop." << std::endl;

            auto imp_next_print = std::chrono::steady_clock::now();
            while (!shutdown_requested)
            {
                auto js = bridge_.leftLeg().getJointStates();
                double q[4] = {
                    js["l_hip_yaw"].position_rad,
                    js["l_hip_roll"].position_rad,
                    js["l_hip_pitch"].position_rad,
                    js["l_knee"].position_rad
                };
                double qd[4] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s
                };

                // τ = K*(q_d - q) + D*(0 - q̇)
                float tau[4];
                for (int j = 0; j < 4; ++j)
                    tau[j] = K[j] * static_cast<float>(q_d[j] - q[j])
                           + D[j] * static_cast<float>(0.0 - qd[j]);

                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw",   tau[0]},
                    {"l_hip_roll",  tau[1]},
                    {"l_hip_pitch", tau[2]},
                    {"l_knee",      tau[3]},
                });

                auto imp_now = std::chrono::steady_clock::now();
                if (imp_now >= imp_next_print) {
                    std::cout << "pos_err (rad): ["
                              << (q_d[0]-q[0]) << ", " << (q_d[1]-q[1])
                              << ", " << (q_d[2]-q[2]) << ", " << (q_d[3]-q[3])
                              << "] | tau (Nm): ["
                              << tau[0] << ", " << tau[1] << ", "
                              << tau[2] << ", " << tau[3] << "]" << std::endl;
                    imp_next_print = imp_now + std::chrono::seconds(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0}
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

            // Read home position in rad (Leg already converts from turns)
            auto home = bridge_.leftLeg().getJointStates();
            double q_home[4] = {
                home["l_hip_yaw"].position_rad,
                home["l_hip_roll"].position_rad,
                home["l_hip_pitch"].position_rad,
                home["l_knee"].position_rad
            };

            LeftLeg::Vec3 x_desired = LeftLeg::forwardKinematics(q_home);

            double J_home[3][4];
            LeftLeg::computeJacobian(q_home, J_home);

            std::cout << "Cartesian impedance control active.\n"
                      << "Home (rad): ["
                      << q_home[0] << ", " << q_home[1] << ", "
                      << q_home[2] << ", " << q_home[3] << "]\n"
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
                    js["l_knee"].position_rad
                };
                double qd[4] = {
                    js["l_hip_yaw"].velocity_rad_s,
                    js["l_hip_roll"].velocity_rad_s,
                    js["l_hip_pitch"].velocity_rad_s,
                    js["l_knee"].velocity_rad_s
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

                // tau = J^T * F
                double tau_d[4];
                LeftLeg::jacobianTransposeMultiply(J, F, tau_d);

                // Clamp and send
                std::map<std::string, float> torques;
                const char* names[4] = {"l_hip_yaw","l_hip_roll","l_hip_pitch","l_knee"};
                for (int j = 0; j < 4; ++j) {
                    float t = static_cast<float>(tau_d[j]);
                    if (t >  tau_max) t =  tau_max;
                    if (t < -tau_max) t = -tau_max;
                    torques[names[j]] = t;
                }
                bridge_.leftLeg().setTorques(torques);

                auto cart_now = std::chrono::steady_clock::now();
                if (cart_now >= cart_next_print) {
                    std::cout << "x: [" << x_actual.x << ", " << x_actual.y << ", " << x_actual.z
                              << "] | err: [" << (x_desired.x-x_actual.x) << ", "
                              << (x_desired.y-x_actual.y) << ", " << (x_desired.z-x_actual.z) << "]"
                              << " | tau: [" << torques["l_hip_yaw"] << ", " << torques["l_hip_roll"]
                              << ", " << torques["l_hip_pitch"] << ", " << torques["l_knee"] << "]"
                              << std::endl;
                    cart_next_print = cart_now + std::chrono::seconds(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                bridge_.leftLeg().setTorques({
                    {"l_hip_yaw", 0}, {"l_hip_roll", 0},
                    {"l_hip_pitch", 0}, {"l_knee", 0}
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
        float curr[4] = {
            js["l_hip_yaw"].position_rad,
            js["l_hip_roll"].position_rad,
            js["l_hip_pitch"].position_rad,
            js["l_knee"].position_rad
        };
        std::cout << "Current positions (rad): ["
                  << curr[0] << ", " << curr[1] << ", " << curr[2] << ", " << curr[3] << "]"
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
