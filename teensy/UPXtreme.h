#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <mutex>
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
    asio::io_context io_context; // must be listed before *_socket
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

    // A9: sys_command_ moved from public to private.
    // Use set*Command() methods instead of direct access.
    std::shared_ptr<CommandBase> sys_command_;

    // A3: guards sys_data_ against concurrent read (control loop) / write (receive thread)
    mutable std::mutex data_mutex_;
    std::shared_ptr<SystemDataContainer> sys_data_;

public:
    std::mutex command_mutex;

    UPXtreme(const std::string &ip, const std::string &interface, int port,
             int N_bus_line, int N_actuator, std::string board_name = "UPXtreme_default");

    virtual ~UPXtreme() { end(); }

    // ----- Command setters -----

    virtual void setPositionCommand(std::shared_ptr<PositionCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    virtual void setVelocityCommand(std::shared_ptr<VelocityCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    virtual void setTorqueCommand(std::shared_ptr<TorqueCommand> cmd) {
        std::lock_guard<std::mutex> lock(command_mutex);
        sys_command_ = cmd;
    }

    virtual void sendIdleCommand() {
        auto idle_cmd = std::make_shared<IdleCommand>();
        std::vector<uint8_t> serialized_data = idle_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
        std::cout << "Sent IDLE command to Teensy" << std::endl;
    }

    virtual void sendStartCommand() {
        auto start_cmd = std::make_shared<StartCommand>();
        std::vector<uint8_t> serialized_data = start_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
        std::cout << "Sent START command to Teensy" << std::endl;
    }

    // No-op keep-alive: resets the Teensy's watchdog without implying any
    // control mode. Deliberately no console print — this is meant to be
    // sent repeatedly during settling waits, unlike the one-shot commands above.
    virtual void sendHeartbeat() {
        auto hb_cmd = std::make_shared<HeartbeatCommand>();
        std::vector<uint8_t> serialized_data = hb_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
    }

    // ----- A3: thread-safe feedback accessors -----
    // Callers use these instead of accessing sys_data_ directly.

    virtual float getPosEstimate(int bus, int node) const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return sys_data_->getPosEstimateAtBusAndNode(bus, node);
    }

    virtual float getVelEstimate(int bus, int node) const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return sys_data_->getVelEstimateAtBusAndNode(bus, node);
    }

    // ----- Networking -----

    void sendToTeensy(const std::vector<uint8_t> &data, const int data_size);
    void handleUDPPacket(const asio::ip::udp::endpoint &client_endpoint,
                         const std::vector<uint8_t> &data);

    virtual void start();

    // A6: end() joins the receive thread cleanly by setting stop_threads first,
    // then closing the socket to unblock the blocking receive_from() call.
    virtual void end()
    {
        stop_threads = true;
        // Close socket to unblock receive_from(). The catch block in the receive
        // thread checks stop_threads so it won't print a spurious error message.
        try { receive_socket.close(); } catch (...) {}
        try { send_socket.close(); }    catch (...) {}

        if (send_thread.joinable())    { send_thread.join(); }
        if (receive_thread.joinable()) { receive_thread.join(); }
    }

    void closeSockets()
    {
        try { send_socket.close(); }
        catch (const std::exception &e) {
            std::cerr << "Error closing send_socket: " << e.what() << std::endl;
        }
        try { receive_socket.close(); }
        catch (const std::exception &e) {
            std::cerr << "Error closing receive_socket: " << e.what() << std::endl;
        }
    }

protected:
    // Sim-only constructor: initialises sys_data_ but skips all networking.
    UPXtreme(int n_bus_line, int n_actuator, std::string board_name)
        : n_bus_line_(n_bus_line), n_actuator_(n_actuator),
          board_name_(std::move(board_name)), udp_port_(0),
          send_socket(io_context), receive_socket(io_context)
    {
        sys_data_ = std::make_shared<SystemDataContainer>();
        for (int i = 0; i < n_bus_line_; ++i)
            sys_data_->add(SystemData<2>());
    }
};
