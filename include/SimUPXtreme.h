#pragma once
#include "UPXtreme.h"
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <chrono>
#include <iostream>

// Software-only UPXtreme substitute. Runs a 500 Hz sim thread in place of the
// UDP receive/send threads. Position tracks commanded values with a first-order
// lag (tau_s seconds). Set tau_s = 0 for perfect (instantaneous) tracking.
// Velocity and torque modes are also supported (unit-inertia dynamics for torque).
class SimUPXtreme : public UPXtreme {
public:
    SimUPXtreme(int n_bus_line, int n_actuator, float tau_s = 0.0f,
                std::string board_name = "Sim");

    ~SimUPXtreme() override {
        sim_stop_ = true;
        if (sim_thread_.joinable()) sim_thread_.join();
    }

    void start()    override;
    void end()      override;

    float getPosEstimate(int bus, int node) const override;
    float getVelEstimate(int bus, int node) const override;

    void setPositionCommand(std::shared_ptr<PositionCommand> cmd) override;
    void setVelocityCommand(std::shared_ptr<VelocityCommand> cmd) override;
    void setTorqueCommand(std::shared_ptr<TorqueCommand>   cmd) override;
    void sendIdleCommand()  override;
    void sendStartCommand() override;
    void sendHeartbeat()    override;
    void sendSetGainsCommand(std::vector<float> pos_gains,
                              std::vector<float> vel_gains,
                              std::vector<float> vel_integrator_gains) override;

private:
    enum class SimMode { IDLE, POSITION, VELOCITY, TORQUE };

    float tau_s_;
    int   n_bus_;
    int   n_act_;

    mutable std::mutex   sim_mutex_;
    std::vector<float>   sim_pos_;   // current simulated position (turns)
    std::vector<float>   sim_vel_;   // current simulated velocity (turns/s)
    std::vector<float>   cmd_pos_;   // commanded position target (turns)
    std::vector<float>   cmd_vel_;   // commanded velocity (turns/s)
    std::vector<float>   cmd_tau_;   // commanded torque (arbitrary units, for dynamics)
    SimMode              sim_mode_   = SimMode::IDLE;

    std::thread          sim_thread_;
    std::atomic<bool>    sim_stop_{false};

    int  idx(int bus, int node) const { return bus * n_act_ + node; }
    void simLoop();
};
