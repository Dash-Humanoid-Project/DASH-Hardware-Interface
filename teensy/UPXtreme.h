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
#include "Utils.h"
#include "Param.h"


using asio::ip::udp;

class UPXtreme
{
protected:
    asio::io_context io_context; // has to be listed before *_socket
    std::string teensy_IP_;
    asio::ip::udp::socket send_socket;
    asio::ip::udp::socket receive_socket;
    int udp_port_;

private:
    std::string board_name_;
    const int n_bus_line_;
    const int n_actuator_;
    
    std::thread receive_thread;
    std::thread send_thread;
    std::thread benchmark_thread;

    std::atomic<uint32_t> receive_counter{0};
    std::atomic<uint32_t> send_counter{0};
    std::atomic<bool> stop_threads{false};

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

    // Send idle command to put all ODrives into IDLE state
    void sendIdleCommand() {
        auto idle_cmd = std::make_shared<IdleCommand>();
        std::vector<uint8_t> serialized_data = idle_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
        std::cout << "Sent IDLE command to Teensy" << std::endl;
    }

    // Send start command to put all ODrives into CLOSED_LOOP_CONTROL state
    void sendStartCommand() {
        auto start_cmd = std::make_shared<StartCommand>();
        std::vector<uint8_t> serialized_data = start_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
        std::cout << "Sent START command to Teensy" << std::endl;
    }

    void sendToTeensy(const std::vector<uint8_t> &data, const int data_size);

    void handleUDPPacket(const asio::ip::udp::endpoint &client_endpoint, const std::vector<uint8_t> &data);

    virtual void start();

    void end()
    {
        // Signal threads to stop
        stop_threads = true;
        // Join the send thread (it checks stop_threads and exits quickly)
        if (send_thread.joinable()) {
            send_thread.join();
        }
        // Detach the receive thread since synchronous receive_from can't be
        // reliably interrupted. Process exit will clean it up.
        if (receive_thread.joinable()) {
            receive_thread.detach();
        }
        // Don't bother closing sockets - process is about to exit
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

};
