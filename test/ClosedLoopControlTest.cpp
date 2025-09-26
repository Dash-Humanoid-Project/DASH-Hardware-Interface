#include <chrono>
#include <cmath>  // For M_PI if needed
#include <memory>
#include "Command.h"
#include "SystemConfig.h"
#include "UPXtreme.h"

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
        upx_ = std::make_unique<UPXtreme>(
            std::vector<std::string>(config.teensy_IP, config.teensy_IP + config.N_teensy),
            config.PC_network_interface_name,
            config.udp_port_PC_teensy,
            config.N_CAN_bus_lines_per_teensy[0], // Assuming all have same bus config
            config.N_actuator_per_CAN_bus_line,
            "UPXtreme");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        upx_->start();
    }

    ~ClosedLoopControl()
    {
        upx_->end();
    }

    void run()
    {
        std::cout << "Running closed-loop control" << std::endl;

        auto start = std::chrono::steady_clock::now();

        // position command
        if (cmd_flag_ == 0)
        {
            while (true)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                float t = 0.001 * duration;

                float phase = t * (TWO_PI / SINE_PERIOD);

                Input_Pos_TYPE desired_position = sin(phase);
                // TODO: look into the proper scale of velocity_ff. Setting scale = 1 leads to jumpy motion currently
                float scale = 0.5;
                Vel_FF_TYPE velocity_ff = scale * cos(phase) * (TWO_PI / SINE_PERIOD);

                cmd_ptr = std::make_shared<PositionCommand>(desired_position, velocity_ff);
                upx_->setPositionCommand(cmd_ptr);
            #ifndef ENABLE_TIME_BENCHMARK
                for (const auto& sys_data : upx_->sys_data_vec_) {
                    sys_data->printValue();
                }
            #endif
            }
        }
        if (cmd_flag_ == 1)
        {
            while (true)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                Input_Vel_TYPE desired_velocity = 0.5;
                vcmd_ptr = std::make_shared<VelocityCommand>(desired_velocity);
                upx_->setVelocityCommand(vcmd_ptr);
            #ifndef ENABLE_TIME_BENCHMARK
                for (const auto& sys_data : upx_->sys_data_vec_) {
                    sys_data->printValue();
                }
            #endif
            }
        }
        if (cmd_flag_ == 2)
        {
            while (true)
            {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
                float desired_torque = 0.32;
                tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);
                upx_->setTorqueCommand(tcmd_ptr);
            #ifndef ENABLE_TIME_BENCHMARK
                for (const auto& sys_data : upx_->sys_data_vec_) {
                    sys_data->printValue();
                }
            #endif
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
    std::unique_ptr<UPXtreme> upx_;
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
