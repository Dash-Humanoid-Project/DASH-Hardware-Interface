#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include <asio.hpp>
#include <cmath>
#include "SystemConfig.h"
#include "Command.h"
#include "DataContainer.h"
#include "Config.h"
#include "Utils.h"
#include "Param.h"


using asio::ip::udp;

class UPXtreme
{
private:
	std::string board_name_;
    std::string teensy_IP_;
	const int n_bus_line_;
    const int n_actuator_;
    int udp_port_;

    asio::io_context io_context;
    asio::ip::udp::socket send_socket;
    asio::ip::udp::socket receive_socket;

    std::thread receive_thread;
    std::thread send_thread;

public:
	// TODO(@nicholasadr): move to private
    std::shared_ptr<CommandBase> sys_command_; //
    std::shared_ptr<SystemDataContainer> sys_data_;
    std::mutex command_mutex;

    UPXtreme(const std::string &ip, const std::string &interface, int port, int N_bus_line, int N_actuator, std::string board_name = "UPXtreme_default");

    ~UPXtreme() {}

    // Accepts a shared pointer to PositionCommand and stores it as CommandBase
    void setPositionCommand(std::shared_ptr<PositionCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    // Accepts a shared pointer to VelocityCommand and stores it as CommandBase
    void setVelocityCommand(std::shared_ptr<VelocityCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    // Accepts a shared pointer to TorqueCommand and stores it as CommandBase
    void setTorqueCommand(std::shared_ptr<TorqueCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    void sendToTeensy(const std::vector<uint8_t> &data, const int data_size);

    void handleUDPPacket(const asio::ip::udp::endpoint &client_endpoint, const std::vector<uint8_t> &data);

    void start();

    void end()
    {
		// wait for threads to finish completely
        joinThreads();
		// now that the threads are done, we can close the sockets
        closeSockets();
    }

    void closeSockets()
    {
        try
        {
            send_socket.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing send_socket: " << e.what() << std::endl;
        }

        // Close the server socket
        try
        {
            receive_socket.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing receive_socket: " << e.what() << std::endl;
        }
    }

    void joinThreads()
    {
        if (receive_thread.joinable())
        {
            receive_thread.join();
        }
        if (send_thread.joinable())
        {
            send_thread.join();
        }
    }

};
