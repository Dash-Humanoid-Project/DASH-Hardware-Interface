#include <chrono>
#include <cmath>  // For M_PI if needed
#include <memory>
#include "Command.h"
#include "SystemConfig.h"
#include "UPXtreme.h"
#include "LeftLegKinematics.h"
#include <iostream>
#include <fstream>
#include <csignal>
#include <atomic>

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
 * 	This example does closed-loop control test for three different command inputs:
 *  * position command (with velocity and torque feedforward)
 *  * velocity command (with torque feedforward)
 *  * torque command
 *
 *  The position closed-loop control test is adapted from:
 * 	    Controlling ODrive from an Arudino via CAN
 * 	    https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
 */

class ClosedLoopControl
{
public:
    ClosedLoopControl(int cmd_flag = 0) : cmd_flag_(cmd_flag)
    {
        for (size_t i = 0; i < config.N_teensy; i++)
        {
            pc_.push_back(
                std::make_unique<UPXtreme>(config.teensy_IP[i],
                                           config.PC_network_interface_name,
                                           config.udp_port_PC_teensy[i],
									       config.N_CAN_bus_lines_per_teensy[i],
									       config.N_actuator_per_CAN_bus_line,
                                           "UPXtreme_" + std::to_string(i)));
        }

        std::cout << "=== DEBUG TEST: About to start UPXtreme boards ===" << std::endl;
        for (size_t id = 0; id < pc_.size(); ++id)
        {
            auto &board = pc_[id];
            std::cout << "=== DEBUG TEST: Starting board " << id << " ===" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            board->start();
            std::cout << "=== DEBUG TEST: Board " << id << " started ===" << std::endl;
        }
        std::cout << "=== DEBUG TEST: All boards started ===" << std::endl;
    }

    ~ClosedLoopControl()
    {
        for (size_t id = 0; id < pc_.size(); ++id)
        {
            auto &board = pc_[id];
            board->end();
        }
    }

    void run()
    {
        std::cout << "Running closed-loop control" << std::endl;

        auto start = std::chrono::steady_clock::now();

        // position command
        if (cmd_flag_ == 0)
        {

             //Initialize the position control measurement log (only log when values change)
            int i = 0;
            int loop_count = 0;
            int max_loop_count = 5e7;
            std::vector<std::chrono::duration<double>> time_log{};
            time_log.reserve(100000);  // Reserve reasonable size

            // Logs for ODRV0 (CAN bus 0, node 0)
            std::vector<double> position_measurement_log_0{};
            position_measurement_log_0.reserve(100000);
            std::vector<double> velocity_measurement_log_0{};
            velocity_measurement_log_0.reserve(100000);

            // Logs for ODRV1 (CAN bus 0, node 1)
            std::vector<double> position_measurement_log_1{};
            position_measurement_log_1.reserve(100000);
            std::vector<double> velocity_measurement_log_1{};
            velocity_measurement_log_1.reserve(100000);

            // Logs for ODRV2 (CAN bus 1, node 0)
            std::vector<double> position_measurement_log_2{};
            position_measurement_log_2.reserve(100000);
            std::vector<double> velocity_measurement_log_2{};
            velocity_measurement_log_2.reserve(100000);

            // Logs for ODRV3 (CAN bus 1, node 1)
            std::vector<double> position_measurement_log_3{};
            position_measurement_log_3.reserve(100000);
            std::vector<double> velocity_measurement_log_3{};
            velocity_measurement_log_3.reserve(100000);

            // Track previous values to only log when they change
            double prev_pos_0 = -999.0, prev_vel_0 = -999.0;
            double prev_pos_1 = -999.0, prev_vel_1 = -999.0;
            double prev_pos_2 = -999.0, prev_vel_2 = -999.0;
            double prev_pos_3 = -999.0, prev_vel_3 = -999.0;

            // Wait for initial encoder feedback before starting control
            std::cout << "Waiting for initial encoder feedback..." << std::endl;
            while (!shutdown_requested) {
                double pos_0 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0);
                double pos_1 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1);
                double pos_2 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0);
                double pos_3 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1);

                // Check if we got valid feedback (at least one non-zero value, or all zeros is valid too)
                if (pos_0 != 0 || pos_1 != 0 || pos_2 != 0 || pos_3 != 0 || loop_count > 100) {
                    std::cout << "Initial positions: [" << pos_0 << ", " << pos_1
                              << ", " << pos_2 << ", " << pos_3 << "]" << std::endl;
                    std::cout << "Starting control loop..." << std::endl;
                    break;
                }
                loop_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            loop_count = 0;  // Reset loop count

            // Get initial positions to hold for 0.5 seconds before starting sine wave
            double hold_pos_0 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0);
            double hold_pos_1 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1);
            double hold_pos_2 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0);
            double hold_pos_3 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1);

            const float HOLD_TIME = 0.5;  // Hold current position for 0.5 seconds

            while (loop_count < max_loop_count && !shutdown_requested)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                float t = 0.001 * duration;

                // Hold current position for first HOLD_TIME seconds
                float phase;
                if (t < HOLD_TIME) {
                    phase = 0;  // Will be overridden below with hold positions
                } else {
                    phase = (t - HOLD_TIME) * (TWO_PI / SINE_PERIOD);
                }

                // Position commands for each motor
                Input_Pos_TYPE q_1, q_2, q_3, q_4;
                float vel_ff_1, vel_ff_2, vel_ff_3, vel_ff_4;

                if (t < HOLD_TIME) {
                    // Hold current position for smooth startup
                    q_1 = hold_pos_0;
                    q_2 = hold_pos_1;
                    q_3 = hold_pos_2;
                    q_4 = hold_pos_3;
                    vel_ff_1 = vel_ff_2 = vel_ff_3 = vel_ff_4 = 0;
                } else {
                    // Sine wave control
                    q_1 = sin(phase);
                    q_2 = -sin(phase);
                    q_3 = -1.5*(sin(phase-M_PI/2)+1);
                    q_4 = -1.5*(sin(phase-M_PI/2)+1);

                    // Per-motor velocity feedforward (derivative of each position command)
                    vel_ff_1 = cos(phase) * (TWO_PI / SINE_PERIOD);
                    vel_ff_2 = -cos(phase) * (TWO_PI / SINE_PERIOD);
                    vel_ff_3 = -1.5 * cos(phase - M_PI/2) * (TWO_PI / SINE_PERIOD);
                    vel_ff_4 = -1.5 * cos(phase - M_PI/2) * (TWO_PI / SINE_PERIOD);
                }

                // Scale down velocity feedforward to reduce jumpy motion
                // TODO: Tune ODrive vel_gain and increase scale toward 1.0 for better tracking
                float scale = 0.5;

                std::vector<Vel_FF_TYPE> velocity_ff = {
                    static_cast<Vel_FF_TYPE>(scale * vel_ff_1),
                    static_cast<Vel_FF_TYPE>(scale * vel_ff_2),
                    static_cast<Vel_FF_TYPE>(scale * vel_ff_3),
                    static_cast<Vel_FF_TYPE>(scale * vel_ff_4)
                };

                cmd_ptr = std::make_shared<PositionCommand>(
                    std::vector<Input_Pos_TYPE>{q_1, q_2, q_3, q_4},
                    velocity_ff  // Per-motor velocity feedforward
                );

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setPositionCommand(cmd_ptr);
#ifndef ENABLE_TIME_BENCHMARK
                    //pc_[id]->sys_data_->printValue();  // Temporarily disabled for debugging
#endif
                }
#ifndef ENABLE_TIME_BENCHMARK
                // Get current measurements
                double curr_pos_0 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0);
                double curr_vel_0 = pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 0);
                double curr_pos_1 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1);
                double curr_vel_1 = pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 1);
                double curr_pos_2 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0);
                double curr_vel_2 = pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 0);
                double curr_pos_3 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1);
                double curr_vel_3 = pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 1);

                // Only log if any value has changed
                if (curr_pos_0 != prev_pos_0 || curr_vel_0 != prev_vel_0 ||
                    curr_pos_1 != prev_pos_1 || curr_vel_1 != prev_vel_1 ||
                    curr_pos_2 != prev_pos_2 || curr_vel_2 != prev_vel_2 ||
                    curr_pos_3 != prev_pos_3 || curr_vel_3 != prev_vel_3) {

                    time_log.push_back(std::chrono::steady_clock::now().time_since_epoch());
                    position_measurement_log_0.push_back(curr_pos_0);
                    velocity_measurement_log_0.push_back(curr_vel_0);
                    position_measurement_log_1.push_back(curr_pos_1);
                    velocity_measurement_log_1.push_back(curr_vel_1);
                    position_measurement_log_2.push_back(curr_pos_2);
                    velocity_measurement_log_2.push_back(curr_vel_2);
                    position_measurement_log_3.push_back(curr_pos_3);
                    velocity_measurement_log_3.push_back(curr_vel_3);

                    prev_pos_0 = curr_pos_0;
                    prev_vel_0 = curr_vel_0;
                    prev_pos_1 = curr_pos_1;
                    prev_vel_1 = curr_vel_1;
                    prev_pos_2 = curr_pos_2;
                    prev_vel_2 = curr_vel_2;
                    prev_pos_3 = curr_pos_3;
                    prev_vel_3 = curr_vel_3;

                    i++;
                }
                loop_count++;
#endif
            }

            // Safe shutdown sequence if Ctrl+C was pressed
            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN SEQUENCE ===" << std::endl;
                std::cout << "Returning motors to home position..." << std::endl;

                // Get current positions
                double curr_pos_0 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0);
                double curr_pos_1 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1);
                double curr_pos_2 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0);
                double curr_pos_3 = pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1);

                std::cout << "Current positions: [" << curr_pos_0 << ", " << curr_pos_1
                          << ", " << curr_pos_2 << ", " << curr_pos_3 << "]" << std::endl;

                // Smoothly return to zero over 2 seconds
                auto shutdown_start = std::chrono::steady_clock::now();
                float shutdown_duration = 2.0; // seconds

                while (true) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - shutdown_start).count() / 1000.0;

                    if (elapsed >= shutdown_duration) break;

                    // Linear interpolation from current position to zero
                    float progress = elapsed / shutdown_duration;
                    Input_Pos_TYPE q_1 = curr_pos_0 * (1.0 - progress);
                    Input_Pos_TYPE q_2 = curr_pos_1 * (1.0 - progress);
                    Input_Pos_TYPE q_3 = curr_pos_2 * (1.0 - progress);
                    Input_Pos_TYPE q_4 = curr_pos_3 * (1.0 - progress);

                    auto shutdown_cmd = std::make_shared<PositionCommand>(
                        std::vector<Input_Pos_TYPE>{q_1, q_2, q_3, q_4},
                        std::vector<Vel_FF_TYPE>{0, 0, 0, 0}
                    );

                    for (size_t id = 0; id < pc_.size(); ++id) {
                        pc_[id]->setPositionCommand(shutdown_cmd);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                std::cout << "Motors returned to home position." << std::endl;

                // Put ODrives into IDLE state
                std::cout << "Putting ODrives into IDLE state..." << std::endl;
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->sendIdleCommand();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give time for command to process
                std::cout << "Shutdown complete." << std::endl;
            }

#ifndef ENABLE_TIME_BENCHMARK
            //Log the position control measurements to a file
            std::cout << "Logged " << i << " measurements." << std::endl;
            std::ofstream send_log_file("../logs/position_measurement_log.csv");
            if (send_log_file.is_open()) {
                // Write header
                send_log_file << "Time,ODRV0_Pos,ODRV0_Vel,ODRV1_Pos,ODRV1_Vel,ODRV2_Pos,ODRV2_Vel,ODRV3_Pos,ODRV3_Vel\n";

                // Write data for all three motors
                for (int j = 0; j < i; ++j) {
                    send_log_file << time_log[j].count() << ","
                                  << position_measurement_log_0[j] << "," << velocity_measurement_log_0[j] << ","
                                  << position_measurement_log_1[j] << "," << velocity_measurement_log_1[j] << ","
                                  << position_measurement_log_2[j] << "," << velocity_measurement_log_2[j] << ","
                                  << position_measurement_log_3[j] << "," << velocity_measurement_log_3[j] << "\n";
                }
                send_log_file.close();
            }
            else {
                std::cout << "Unable to open file for logging position control measurements." << std::endl;
            }
#endif
        }
        // velocity command
        if (cmd_flag_ == 1)
        {
            while (!shutdown_requested)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                Input_Vel_TYPE v1 = 0;
                Input_Vel_TYPE v2 = 0;
                Input_Vel_TYPE v3 = 0;
                Input_Vel_TYPE v4 = 0;
                std::vector<Input_Vel_TYPE> desired_velocity = {v1, v2, v3, v4};
                vcmd_ptr = std::make_shared<VelocityCommand>(desired_velocity);

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setVelocityCommand(vcmd_ptr);
#ifndef ENABLE_TIME_BENCHMARK
                    //pc_[id]->sys_data_->printValue();  // Temporarily disabled for debugging
#endif
                }
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Velocity mode stopped (motors already at zero velocity)." << std::endl;

                // Put ODrives into IDLE state
                std::cout << "Putting ODrives into IDLE state..." << std::endl;
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->sendIdleCommand();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "Shutdown complete." << std::endl;
            }
        }
        // torque command
        if (cmd_flag_ == 2)
        {
            while (!shutdown_requested)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                Input_Torque_TYPE tau1 = 0;
                Input_Torque_TYPE tau2 = 0;
                Input_Torque_TYPE tau3 = 0;
                Input_Torque_TYPE tau4 = 0;
                std::vector<Input_Torque_TYPE> desired_torque = {tau1, tau2, tau3, tau4};
                tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setTorqueCommand(tcmd_ptr);
#ifndef ENABLE_TIME_BENCHMARK
                    //pc_[id]->sys_data_->printValue();  // Temporarily disabled for debugging
#endif
                }
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Torque mode stopped (motors already at zero torque)." << std::endl;

                // Put ODrives into IDLE state
                std::cout << "Putting ODrives into IDLE state..." << std::endl;
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->sendIdleCommand();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "Shutdown complete." << std::endl;
            }
        }

        // impedance control
        if (cmd_flag_ == 3)
        {
            // Virtual stiffness (Nm/turn) and damping (Nm*s/turn) per joint
            // Tune these to change how the leg feels
            float K[4] = {5.0f, 7.5f, 4.0f, 2.5f};  // stiffness
            float D[4] = {0.1f, 0.2f, 0.1f, 0.1f};  // damping

            // Wait for encoder feedback
            std::cout << "Waiting for encoder feedback..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Use current positions as desired "home" position
            double q_desired[4] = {
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1)
            };

            std::cout << "Impedance control active." << std::endl;
            std::cout << "Home positions: [" << q_desired[0] << ", " << q_desired[1]
                      << ", " << q_desired[2] << ", " << q_desired[3] << "]" << std::endl;
            std::cout << "K = [" << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3] << "] Nm/turn" << std::endl;
            std::cout << "D = [" << D[0] << ", " << D[1] << ", " << D[2] << ", " << D[3] << "] Nm*s/turn" << std::endl;
            std::cout << "Push the leg to feel the virtual spring-damper. Ctrl+C to stop." << std::endl;

            int loop_count = 0;

            while (!shutdown_requested)
            {
                // Read current joint positions and velocities
                double q_actual[4] = {
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1)
                };
                double qdot_actual[4] = {
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 0),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 1),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 0),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 1)
                };

                // Impedance control law: τ = K*(q_d - q) + D*(0 - q̇)
                Input_Torque_TYPE tau0 = K[0] * (q_desired[0] - q_actual[0]) + D[0] * (0 - qdot_actual[0]);
                Input_Torque_TYPE tau1 = K[1] * (q_desired[1] - q_actual[1]) + D[1] * (0 - qdot_actual[1]);
                Input_Torque_TYPE tau2 = K[2] * (q_desired[2] - q_actual[2]) + D[2] * (0 - qdot_actual[2]);
                Input_Torque_TYPE tau3 = K[3] * (q_desired[3] - q_actual[3]) + D[3] * (0 - qdot_actual[3]);

                std::vector<Input_Torque_TYPE> desired_torque = {tau0, tau1, tau2, tau3};
                tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setTorqueCommand(tcmd_ptr);
                }

                // Print status periodically
                if (loop_count % 10000 == 0) {
                    std::cout << "pos_err: ["
                              << (q_desired[0] - q_actual[0]) << ", "
                              << (q_desired[1] - q_actual[1]) << ", "
                              << (q_desired[2] - q_actual[2]) << ", "
                              << (q_desired[3] - q_actual[3]) << "] | tau: ["
                              << tau0 << ", " << tau1 << ", " << tau2 << ", " << tau3 << "]" << std::endl;
                }
                loop_count++;
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Impedance mode stopped. Sending zero torque..." << std::endl;

                // Send zero torque before idling
                auto zero_torque = std::make_shared<TorqueCommand>(
                    std::vector<Input_Torque_TYPE>{0, 0, 0, 0});
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->setTorqueCommand(zero_torque);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                // Put ODrives into IDLE state
                std::cout << "Putting ODrives into IDLE state..." << std::endl;
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->sendIdleCommand();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "Shutdown complete." << std::endl;
            }
        }

        // Cartesian impedance control
        if (cmd_flag_ == 4)
        {
            // Cartesian stiffness (N/m) and damping (Ns/m) for x, y, z
            // Note: effective joint stiffness ≈ J^T*Kx*J, with links ~0.28m
            // Kx=1 N/m ≈ 0.08 Nm/rad ≈ 0.5 Nm/turn effective joint stiffness
            float Kx[3] = {0.5f, 0.5f, 0.5f};
            float Dx[3] = {0.05f, 0.05f, 0.05f};

            // Max torque per joint (Nm) for safety
            float tau_max = 0.5f;

            // Wait for encoder feedback
            std::cout << "Waiting for encoder feedback..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Get current joint positions (turns) and convert to radians
            double q_home_turns[4] = {
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0),
                pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1)
            };
            double q_home_rad[4];
            for (int i = 0; i < 4; i++) q_home_rad[i] = q_home_turns[i] * TWO_PI;

            // Compute desired end-effector position (hold home position)
            LeftLeg::Vec3 x_desired = LeftLeg::forwardKinematics(q_home_rad);

            // Compute and print Jacobian at home configuration for diagnostics
            double J_home[3][4];
            LeftLeg::computeJacobian(q_home_rad, J_home);

            std::cout << "Cartesian impedance control active." << std::endl;
            std::cout << "Joint mapping: odrv0=hip_yaw, odrv1=hip_row, odrv2=hip_pitch, odrv3=knee" << std::endl;
            std::cout << "Home joint angles (turns): [" << q_home_turns[0] << ", " << q_home_turns[1]
                      << ", " << q_home_turns[2] << ", " << q_home_turns[3] << "]" << std::endl;
            std::cout << "Home joint angles (rad): [" << q_home_rad[0] << ", " << q_home_rad[1]
                      << ", " << q_home_rad[2] << ", " << q_home_rad[3] << "]" << std::endl;
            std::cout << "Home end-effector position: [" << x_desired.x << ", "
                      << x_desired.y << ", " << x_desired.z << "] m" << std::endl;
            std::cout << "Jacobian at home:" << std::endl;
            for (int r = 0; r < 3; r++) {
                std::cout << "  [" << J_home[r][0] << ", " << J_home[r][1]
                          << ", " << J_home[r][2] << ", " << J_home[r][3] << "]" << std::endl;
            }
            // Effective stiffness diagonal of J^T*Kx*J (approximate)
            std::cout << "Effective joint stiffness (J^T*Kx*J diagonal, Nm/rad): [";
            for (int i = 0; i < 4; i++) {
                double eff = 0;
                for (int r = 0; r < 3; r++) eff += Kx[r] * J_home[r][i] * J_home[r][i];
                std::cout << eff << (i < 3 ? ", " : "");
            }
            std::cout << "]" << std::endl;
            std::cout << "Kx = [" << Kx[0] << ", " << Kx[1] << ", " << Kx[2] << "] N/m" << std::endl;
            std::cout << "Dx = [" << Dx[0] << ", " << Dx[1] << ", " << Dx[2] << "] Ns/m" << std::endl;
            std::cout << "tau_max = " << tau_max << " Nm" << std::endl;
            std::cout << "Push the leg to feel the Cartesian spring-damper. Ctrl+C to stop." << std::endl;

            int loop_count = 0;

            while (!shutdown_requested)
            {
                // Read current joint positions (turns) and velocities (turns/s)
                double q_turns[4] = {
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0),
                    pc_[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1)
                };
                double qdot_turns[4] = {
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 0),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(0, 1),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 0),
                    pc_[0]->sys_data_->getVelEstimateAtBusAndNode(1, 1)
                };

                // Convert to radians
                double q_rad[4], qdot_rad[4];
                for (int i = 0; i < 4; i++) {
                    q_rad[i] = q_turns[i] * TWO_PI;
                    qdot_rad[i] = qdot_turns[i] * TWO_PI;
                }

                // Forward kinematics: current end-effector position
                LeftLeg::Vec3 x_actual = LeftLeg::forwardKinematics(q_rad);

                // Compute Jacobian at current configuration
                double J[3][4];
                LeftLeg::computeJacobian(q_rad, J);

                // End-effector velocity: xdot = J * qdot
                double xdot[3] = {0, 0, 0};
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 4; j++)
                        xdot[i] += J[i][j] * qdot_rad[j];

                // Cartesian impedance: F = K*(x_d - x) + D*(0 - xdot)
                double F[3] = {
                    Kx[0] * (x_desired.x - x_actual.x) + Dx[0] * (0 - xdot[0]),
                    Kx[1] * (x_desired.y - x_actual.y) + Dx[1] * (0 - xdot[1]),
                    Kx[2] * (x_desired.z - x_actual.z) + Dx[2] * (0 - xdot[2])
                };

                // Map to joint torques: tau = J^T * F
                double tau[4];
                LeftLeg::jacobianTransposeMultiply(J, F, tau);

                // Clamp torques for safety
                for (int i = 0; i < 4; i++) {
                    if (tau[i] > tau_max) tau[i] = tau_max;
                    if (tau[i] < -tau_max) tau[i] = -tau_max;
                }

                // Send torque command
                std::vector<Input_Torque_TYPE> desired_torque = {
                    static_cast<Input_Torque_TYPE>(tau[0]),
                    static_cast<Input_Torque_TYPE>(tau[1]),
                    static_cast<Input_Torque_TYPE>(tau[2]),
                    static_cast<Input_Torque_TYPE>(tau[3])
                };
                tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->setTorqueCommand(tcmd_ptr);
                }

                // Print status periodically
                if (loop_count % 10000 == 0) {
                    std::cout << "x: [" << x_actual.x << ", " << x_actual.y << ", " << x_actual.z
                              << "] | err: [" << (x_desired.x - x_actual.x) << ", "
                              << (x_desired.y - x_actual.y) << ", "
                              << (x_desired.z - x_actual.z)
                              << "] | tau: [" << tau[0] << ", " << tau[1] << ", "
                              << tau[2] << ", " << tau[3] << "]" << std::endl;
                }
                loop_count++;
            }

            if (shutdown_requested) {
                std::cout << "\n=== SAFE SHUTDOWN ===" << std::endl;
                std::cout << "Cartesian impedance mode stopped. Sending zero torque..." << std::endl;

                auto zero_torque = std::make_shared<TorqueCommand>(
                    std::vector<Input_Torque_TYPE>{0, 0, 0, 0});
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->setTorqueCommand(zero_torque);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                std::cout << "Putting ODrives into IDLE state..." << std::endl;
                for (size_t id = 0; id < pc_.size(); ++id) {
                    pc_[id]->sendIdleCommand();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "Shutdown complete." << std::endl;
            }
        }
    }

    SystemConfig config;
    std::shared_ptr<PositionCommand> cmd_ptr;
    std::shared_ptr<VelocityCommand> vcmd_ptr;
    std::shared_ptr<TorqueCommand> tcmd_ptr;

    float SINE_PERIOD = 5.0f; // Period of the position command sine wave in seconds

private:

    int cmd_flag_;
    std::vector<std::unique_ptr<UPXtreme>> pc_;

};

int main(int argc, char* argv[])
{
    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);
    std::cout << "Press Ctrl+C to safely stop and return motors to home position." << std::endl;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [--position | --velocity | --torque | --impedance | --cartesian | --start | --idle | --reset]" << std::endl;
        return 1;
    }

    std::string flag = argv[1];
    int cmd_flag = 0;

    if (flag == "--reset") {
        std::cout << "Command type: reset (returning motors to position zero)" << std::endl;

        // Initialize UPXtreme board with threads to get encoder feedback
        SystemConfig config;
        std::vector<std::unique_ptr<UPXtreme>> pc;
        for (int i = 0; i < config.N_teensy; ++i) {
            pc.emplace_back(std::make_unique<UPXtreme>(
                config.teensy_IP[i],
                config.PC_network_interface_name,
                config.udp_port_PC_teensy[i],
                config.N_CAN_bus_lines_per_teensy[i],
                config.N_actuator_per_CAN_bus_line));
            pc[i]->start();
        }

        // Wait for encoder feedback
        std::cout << "Waiting for encoder feedback..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Get current positions
        double curr_pos_0 = pc[0]->sys_data_->getPosEstimateAtBusAndNode(0, 0);
        double curr_pos_1 = pc[0]->sys_data_->getPosEstimateAtBusAndNode(0, 1);
        double curr_pos_2 = pc[0]->sys_data_->getPosEstimateAtBusAndNode(1, 0);
        double curr_pos_3 = pc[0]->sys_data_->getPosEstimateAtBusAndNode(1, 1);

        std::cout << "Current positions: [" << curr_pos_0 << ", " << curr_pos_1
                  << ", " << curr_pos_2 << ", " << curr_pos_3 << "]" << std::endl;
        std::cout << "Smoothly returning to zero..." << std::endl;

        // Smoothly return to zero over 2 seconds
        auto start = std::chrono::steady_clock::now();
        float duration = 2.0; // seconds

        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count() / 1000.0;

            if (elapsed >= duration) break;

            float progress = elapsed / duration;
            auto cmd = std::make_shared<PositionCommand>(
                std::vector<Input_Pos_TYPE>{
                    static_cast<Input_Pos_TYPE>(curr_pos_0 * (1.0 - progress)),
                    static_cast<Input_Pos_TYPE>(curr_pos_1 * (1.0 - progress)),
                    static_cast<Input_Pos_TYPE>(curr_pos_2 * (1.0 - progress)),
                    static_cast<Input_Pos_TYPE>(curr_pos_3 * (1.0 - progress))
                },
                std::vector<Vel_FF_TYPE>{0, 0, 0, 0}
            );

            for (size_t id = 0; id < pc.size(); ++id) {
                pc[id]->setPositionCommand(cmd);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Motors returned to home position." << std::endl;

        // Put ODrives into IDLE state
        std::cout << "Putting ODrives into IDLE state..." << std::endl;
        for (size_t id = 0; id < pc.size(); ++id) {
            pc[id]->sendIdleCommand();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Reset complete." << std::endl;

        for (size_t id = 0; id < pc.size(); ++id) {
            pc[id]->end();
        }

        return 0;
    }
    else if (flag == "--start") {
        std::cout << "Command type: start (enabling closed-loop control)" << std::endl;

        // Initialize UPXtreme board (no threads needed for one-shot commands)
        SystemConfig config;
        std::vector<std::unique_ptr<UPXtreme>> pc;
        for (int i = 0; i < config.N_teensy; ++i) {
            pc.emplace_back(std::make_unique<UPXtreme>(
                config.teensy_IP[i],
                config.PC_network_interface_name,
                config.udp_port_PC_teensy[i],
                config.N_CAN_bus_lines_per_teensy[i],
                config.N_actuator_per_CAN_bus_line));
        }

        // Send start command
        std::cout << "Sending START command..." << std::endl;
        for (size_t id = 0; id < pc.size(); ++id) {
            pc[id]->sendStartCommand();
        }

        // Wait for command to process
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "All ODrives should now be in CLOSED_LOOP_CONTROL state." << std::endl;
        std::cout << "You can now run position/velocity/torque control." << std::endl;

        return 0;
    }
    else if (flag == "--idle") {
        std::cout << "Command type: idle (putting all ODrives into IDLE state)" << std::endl;

        // Initialize UPXtreme board (no threads needed for one-shot commands)
        SystemConfig config;
        std::vector<std::unique_ptr<UPXtreme>> pc;
        for (int i = 0; i < config.N_teensy; ++i) {
            pc.emplace_back(std::make_unique<UPXtreme>(
                config.teensy_IP[i],
                config.PC_network_interface_name,
                config.udp_port_PC_teensy[i],
                config.N_CAN_bus_lines_per_teensy[i],
                config.N_actuator_per_CAN_bus_line));
        }

        // Send idle command
        std::cout << "Sending IDLE command..." << std::endl;
        for (size_t id = 0; id < pc.size(); ++id) {
            pc[id]->sendIdleCommand();
        }

        // Wait for command to process
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "All ODrives should now be in IDLE state." << std::endl;

        return 0;
    }
    else if (flag == "--position") {
        std::cout << "Command type: position" << std::endl;
        cmd_flag = 0;
    }
    else if (flag == "--velocity") {
        std::cout << "Command type: velocity" << std::endl;
        cmd_flag = 1;
    }
    else if (flag == "--torque") {
        std::cout << "Command type: torque" << std::endl;
        cmd_flag = 2;
    }
    else if (flag == "--impedance") {
        std::cout << "Command type: impedance" << std::endl;
        cmd_flag = 3;
    }
    else if (flag == "--cartesian") {
        std::cout << "Command type: cartesian impedance" << std::endl;
        cmd_flag = 4;
    }
    else {
        std::cout << "Unknown flag: " << flag << std::endl;
        std::cout << "Usage: " << argv[0] << " [--position | --velocity | --torque | --impedance | --cartesian | --start | --idle | --reset]" << std::endl;
        return 1;
    }

    ClosedLoopControl ctrl(cmd_flag);
    ctrl.run();
}
