#include <chrono>
#include <cmath>  // For M_PI if needed
#include <memory>
#include "Command.h"
#include "SystemConfig.h"
#include "UPXtreme.h"
#include <iostream>
#include <fstream>

#define UPXTREME_i14

#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

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

        for (size_t id = 0; id < pc_.size(); ++id)
        {
            auto &board = pc_[id];
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            board->start();
        }
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
            int max_loop_count = 5e6;
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

            while (loop_count < max_loop_count)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                float t = 0.001 * duration;

                float phase = t * (TWO_PI / SINE_PERIOD);

                // Position commands for each motor
                Input_Pos_TYPE q_1 = sin(phase);
                Input_Pos_TYPE q_2 = -sin(phase);
                Input_Pos_TYPE q_3 = -1.5*(sin(phase-M_PI/2)+1);
                Input_Pos_TYPE q_4 = -1.5*(sin(phase-M_PI/2)+1);

                // Per-motor velocity feedforward (derivative of each position command)
                // Motor 1: d/dt[sin(phase)] = cos(phase) * d(phase)/dt
                float vel_ff_1 = cos(phase) * (TWO_PI / SINE_PERIOD);

                // Motor 2: d/dt[-sin(phase)] = -cos(phase) * d(phase)/dt
                float vel_ff_2 = -cos(phase) * (TWO_PI / SINE_PERIOD);

                // Motor 3: d/dt[-1.5*sin(phase-π/2)] = -1.5*cos(phase-π/2) * d(phase)/dt
                // Note: sin(x-π/2) = -cos(x), so derivative is -1.5*(-sin(x)) = 1.5*sin(x)
                float vel_ff_3 = -1.5 * cos(phase - M_PI/2) * (TWO_PI / SINE_PERIOD);

                // Motor 4: d/dt[-1.5*sin(phase-π/2)] = -1.5*cos(phase-π/2) * d(phase)/dt
                float vel_ff_4 = -1.5 * cos(phase - M_PI/2) * (TWO_PI / SINE_PERIOD);

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
                    pc_[id]->sys_data_->printValue();
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
            while (true)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                Input_Vel_TYPE v1 = 0;
                Input_Vel_TYPE v2 = 0;
                Input_Vel_TYPE v3 = 0;
                std::vector<Input_Vel_TYPE> desired_velocity = {v1, v2, v3};
                vcmd_ptr = std::make_shared<VelocityCommand>(desired_velocity);

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setVelocityCommand(vcmd_ptr);
#ifndef ENABLE_TIME_BENCHMARK
                    pc_[id]->sys_data_->printValue();
#endif
                }
            }
        }
        // torque command
        if (cmd_flag_ == 2)
        {
            while (true)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                Input_Torque_TYPE tau1 = 0;
                Input_Torque_TYPE tau2 = 0;
                Input_Torque_TYPE tau3 = 0;
                std::vector<Input_Torque_TYPE> desired_torque = {tau1, tau2, tau3};
                tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);

                for (size_t id = 0; id < pc_.size(); ++id)
                {
                    pc_[id]->setTorqueCommand(tcmd_ptr);
#ifndef ENABLE_TIME_BENCHMARK
                    pc_[id]->sys_data_->printValue();
#endif
                }
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
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [--position | --velocity | --torque] <value>" << std::endl;
        return 1;
    }

    std::string flag = argv[1];
    int cmd_flag = 0;

    if (flag == "--position") {
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
    else {
        std::cout << "Unknown flag: " << flag << std::endl;
        std::cout << "Usage: " << argv[0] << " [--position | --velocity | --torque] <value>" << std::endl;
        return 1;
    }

    ClosedLoopControl ctrl(cmd_flag);
    ctrl.run();
}
