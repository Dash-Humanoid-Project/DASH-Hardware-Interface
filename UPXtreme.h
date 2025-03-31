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
#include "SystemCommand.h"
#include "SystemData.h"
#include "Utils.h"


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
	// TODO(@nicholasadr): move System* to private
    SystemCommand sys_command_; //
    SystemData sys_data_;

    UPXtreme(const std::string &ip, const std::string &interface, int port, int N_bus_line, int N_actuator, std::string board_name = "UPXtreme_default");

    ~UPXtreme() {}

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
