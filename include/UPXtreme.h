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

    // Dedicated low-rate socket for GetSetParamCommand responses — separate
    // from receive_socket (the 500Hz, exact-size-gated SystemData path) so
    // that path is never touched by this diagnostic-only feature. Shares
    // io_context with the other sockets, which is safe here because no
    // socket on this class ever calls io_context.run() — every operation
    // (including this one) is a direct synchronous/blocking call.
    asio::ip::udp::socket param_response_socket;
    int udp_port_param_response_;

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
             int param_response_port, int N_bus_line, int N_actuator,
             std::string board_name = "UPXtreme_default");

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
        // The real bug (found 2026-09-11) isn't idle's own reliability so
        // much as what's left running afterward. teensy2.ino's
        // PositionCommand handler calls setControllerMode(3,1) whenever
        // current_mode != 3, and idleAllODrives() resets current_mode = 0 —
        // so if a stale PositionCommand from before idling is still the one
        // the send thread keeps repeating at 30us intervals (sys_command_
        // never otherwise gets superseded), the very next one after idling
        // sees current_mode==0 and calls setControllerMode() again, which
        // can re-arm the axis right back to active control within
        // microseconds. Net effect: a joint that should go slack for manual
        // range-of-motion recording stays rigidly held instead.
        //
        // Fix: permanently supersede that stale stream with a Heartbeat
        // command instead of re-sending IdleCommand itself — Heartbeat's
        // Teensy-side handler is a deliberate no-op (no Serial output, no
        // mode/state change, just resets the watchdog timer), safe to repeat
        // at 30us indefinitely. IdleCommand's own handler prints two
        // Serial.println() calls every time, which would block the Teensy's
        // main loop solid if repeated at that rate — tried that first, and
        // live position feedback froze the instant the motor idled.
        auto idle_cmd = std::make_shared<IdleCommand>();
        std::vector<uint8_t> serialized_data = idle_cmd->serializeWithHeader();
        for (int i = 0; i < 5; ++i) {
            sendToTeensy(serialized_data, serialized_data.size());
            if (i < 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        auto hb_cmd = std::make_shared<HeartbeatCommand>();
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            sys_command_ = hb_cmd;
        }

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

    // Sets each ODrive's own onboard position/velocity gains. Per-motor
    // vectors, positionally aligned with the Teensy's odrives[] order (same
    // convention as setPositionCommand/setTorqueCommand) — every motor on
    // this Teensy must be present, since there's no safe default gain value.
    virtual void sendSetGainsCommand(std::vector<float> pos_gains,
                                      std::vector<float> vel_gains,
                                      std::vector<float> vel_integrator_gains) {
        auto gains_cmd = std::make_shared<SetGainsCommand>(pos_gains, vel_gains, vel_integrator_gains);
        std::vector<uint8_t> serialized_data = gains_cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
        std::cout << "Sent SetGains command to Teensy" << std::endl;
    }

    // Reads or writes one ODrive-native CAN parameter ("endpoint_id" — see
    // the odrive Python package for IDs) on one motor of this Teensy.
    // Diagnostic/config use only, not for the control loop. `sendGetSetParamCommand`
    // just sends the request (fire-and-forget for op==SET); for op==GET, the
    // caller must follow up with receiveParamResponse() to get the value —
    // see Leg::getParam*/setParam* (src/Leg.cpp) for the higher-level API
    // that does both in one call.
    virtual void sendGetSetParamCommand(uint8_t motor_idx, ParamOp op, uint16_t endpoint_id,
                                         ParamType type, const uint8_t value[4]) {
        auto cmd = std::make_shared<GetSetParamCommand>(
            motor_idx, static_cast<uint8_t>(op), endpoint_id, static_cast<uint8_t>(type), value);
        std::vector<uint8_t> serialized_data = cmd->serializeWithHeader();
        sendToTeensy(serialized_data, serialized_data.size());
    }

    // Blocks (up to timeout_ms) for one ParamResponse on the dedicated
    // param_response_socket. Returns false on timeout or a malformed packet.
    // Synchronous, one-shot — not part of the 500Hz receive_thread.
    virtual bool receiveParamResponse(ParamResponse& out, int timeout_ms = 200);

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
        try { param_response_socket.close(); } catch (...) {}

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
          send_socket(io_context), receive_socket(io_context),
          param_response_socket(io_context), udp_port_param_response_(0)
    {
        sys_data_ = std::make_shared<SystemDataContainer>();
        for (int i = 0; i < n_bus_line_; ++i)
            sys_data_->add(SystemData<2>());
    }
};
