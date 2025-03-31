#include <chrono>
#include <cmath>  // For M_PI if needed
#include "SystemConfig.h"
#include "SystemCommand.h"
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

            cmd.position = sin(phase);
            cmd.velocity_ff = cos(phase) * (TWO_PI / SINE_PERIOD);

            for (size_t id = 0; id < pc_.size(); ++id)
            {
                pc_[id]->sys_command_ = cmd;
                std::cout << "[Pos_Estimate] : " << pc_[id]->sys_data_.encoder_Pos_Estimate << std::endl;
                std::cout << "[Vel_Estimate] : " << pc_[id]->sys_data_.encoder_Vel_Estimate << std::endl;
            }
        }

    }

    SystemConfig config;
    SystemCommand cmd;

    float SINE_PERIOD = 5.0f; // Period of the position command sine wave in seconds

private:

    std::vector<std::unique_ptr<UPXtreme>> pc_;

};

int main()
{
    ClosedLoopPositionControl ctrl;
    ctrl.run();
}
