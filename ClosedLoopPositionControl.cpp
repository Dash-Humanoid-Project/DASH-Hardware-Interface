#include <chrono>
#include <cmath>  // For M_PI if needed
#include <memory>
#include "Command.h"
#include "SystemConfig.h"
#include "UPXtreme.h"

#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

/*	
 * 	This example does closed-loop control test following the article:
 * 	Controlling ODrive from an Arudino via CAN
 * 	https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
 */

class ClosedLoopPositionControl
{
public:
    ClosedLoopPositionControl()
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

    ~ClosedLoopPositionControl()
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

        while (true)
        {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
            float t = 0.001 * duration;

            float phase = t * (TWO_PI / SINE_PERIOD);

            Input_Pos_TYPE desired_position = sin(phase);
            // TODO: look into the proper scale of velocity_ff. Setting scale = 1 leads to jumpy motion currently
            float scale = 0.5;
            Vel_FF_TYPE velocity_ff = scale * cos(phase) * (TWO_PI / SINE_PERIOD);

            //std::cout << "pos: " << desired_position << " | vel_ff: " << velocity_ff << std::endl;
            cmd_ptr = std::make_shared<PositionCommand>(desired_position, velocity_ff);
            Input_Vel_TYPE desired_velocity = 0.5;
            vcmd_ptr = std::make_shared<VelocityCommand>(desired_velocity);
            float desired_torque = 0.1;
            tcmd_ptr = std::make_shared<TorqueCommand>(desired_torque);

            for (size_t id = 0; id < pc_.size(); ++id)
            {
                pc_[id]->setPositionCommand(cmd_ptr);
                //pc_[id]->setVelocityCommand(vcmd_ptr);
                //pc_[id]->setTorqueCommand(tcmd_ptr);
                std::cout << "[Pos_Estimate] : " << pc_[id]->sys_data_.encoder_Pos_Estimate << std::endl;
                //std::cout << "[Vel_Estimate] : " << pc_[id]->sys_data_.encoder_Vel_Estimate << std::endl;
            }
        }

    }

    SystemConfig config;
    std::shared_ptr<PositionCommand> cmd_ptr;
    std::shared_ptr<VelocityCommand> vcmd_ptr;
    std::shared_ptr<TorqueCommand> tcmd_ptr;

    float SINE_PERIOD = 5.0f; // Period of the position command sine wave in seconds

private:

    std::vector<std::unique_ptr<UPXtreme>> pc_;

};

int main()
{
    ClosedLoopPositionControl ctrl;
    ctrl.run();
}
