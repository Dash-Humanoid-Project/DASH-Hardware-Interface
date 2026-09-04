#include "SimUPXtreme.h"

SimUPXtreme::SimUPXtreme(int n_bus_line, int n_actuator, float tau_s, std::string board_name)
    : UPXtreme(n_bus_line, n_actuator, std::move(board_name)),
      tau_s_(tau_s), n_bus_(n_bus_line), n_act_(n_actuator)
{
    int n = n_bus_line * n_actuator;
    sim_pos_.assign(n, 0.0f);
    sim_vel_.assign(n, 0.0f);
    cmd_pos_.assign(n, 0.0f);
    cmd_vel_.assign(n, 0.0f);
    cmd_tau_.assign(n, 0.0f);
    std::cout << "[Sim] Created " << n << " simulated motor(s), tau=" << tau_s << "s" << std::endl;
}

void SimUPXtreme::start()
{
    std::cout << "[Sim] Starting simulation thread..." << std::endl;
    sim_thread_ = std::thread([this]() { simLoop(); });
}

void SimUPXtreme::end()
{
    sim_stop_ = true;
    if (sim_thread_.joinable()) sim_thread_.join();
    UPXtreme::end(); // closes sockets (no-op, never opened) and joins HW threads (never started)
}

float SimUPXtreme::getPosEstimate(int bus, int node) const
{
    std::lock_guard<std::mutex> lock(sim_mutex_);
    return sim_pos_[idx(bus, node)];
}

float SimUPXtreme::getVelEstimate(int bus, int node) const
{
    std::lock_guard<std::mutex> lock(sim_mutex_);
    return sim_vel_[idx(bus, node)];
}

void SimUPXtreme::setPositionCommand(std::shared_ptr<PositionCommand> cmd)
{
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sim_mode_ = SimMode::POSITION;
    for (int b = 0; b < n_bus_; ++b)
        for (int n = 0; n < n_act_; ++n) {
            int i = idx(b, n);
            if (i < static_cast<int>(cmd->Input_Pos.size()))
                cmd_pos_[i] = cmd->Input_Pos[i];
        }
}

void SimUPXtreme::setVelocityCommand(std::shared_ptr<VelocityCommand> cmd)
{
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sim_mode_ = SimMode::VELOCITY;
    for (int b = 0; b < n_bus_; ++b)
        for (int n = 0; n < n_act_; ++n) {
            int i = idx(b, n);
            if (i < static_cast<int>(cmd->Input_Vel.size()))
                cmd_vel_[i] = cmd->Input_Vel[i];
        }
}

void SimUPXtreme::setTorqueCommand(std::shared_ptr<TorqueCommand> cmd)
{
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sim_mode_ = SimMode::TORQUE;
    for (int b = 0; b < n_bus_; ++b)
        for (int n = 0; n < n_act_; ++n) {
            int i = idx(b, n);
            if (i < static_cast<int>(cmd->Input_Torque.size()))
                cmd_tau_[i] = cmd->Input_Torque[i];
        }
}

void SimUPXtreme::sendIdleCommand()
{
    std::cout << "[Sim] IDLE (no-op)" << std::endl;
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sim_mode_ = SimMode::IDLE;
    for (auto& v : sim_vel_) v = 0.0f;
}

void SimUPXtreme::sendStartCommand()
{
    std::cout << "[Sim] START (no-op)" << std::endl;
}

void SimUPXtreme::sendHeartbeat()
{
    // No watchdog concept in sim mode — genuinely nothing to do.
}

void SimUPXtreme::sendSetGainsCommand(std::vector<float> pos_gains,
                                       std::vector<float> vel_gains,
                                       std::vector<float> vel_integrator_gains)
{
    // No local ODrive PD to configure in sim mode — the sim's position
    // tracking is the fixed first-order lag set at construction, not a
    // gain-driven cascade. Logged so --sim runs still show the call happened.
    std::cout << "[Sim] SetGains (no-op)" << std::endl;
}

void SimUPXtreme::simLoop()
{
    using clock = std::chrono::steady_clock;
    auto last = clock::now();

    while (!sim_stop_) {
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;

        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            for (int i = 0; i < n_bus_ * n_act_; ++i) {
                switch (sim_mode_) {
                case SimMode::POSITION: {
                    float old_pos = sim_pos_[i];
                    if (tau_s_ > 0.0f) {
                        float alpha = 1.0f - expf(-dt / tau_s_);
                        sim_pos_[i] += alpha * (cmd_pos_[i] - sim_pos_[i]);
                    } else {
                        sim_pos_[i] = cmd_pos_[i];
                    }
                    sim_vel_[i] = (dt > 1e-6f) ? (sim_pos_[i] - old_pos) / dt : 0.0f;
                    break;
                }
                case SimMode::VELOCITY:
                    sim_pos_[i] += cmd_vel_[i] * dt;
                    sim_vel_[i]  = cmd_vel_[i];
                    break;
                case SimMode::TORQUE: {
                    // Unit-inertia model: a = tau / J, J = 0.1 kg·m²
                    // Sinusoidal disturbance simulates external forces so the
                    // impedance controller has something to actively reject.
                    static constexpr float J = 0.1f;
                    float t = std::chrono::duration<float>(now.time_since_epoch()).count();
                    float disturbance = 0.3f * sinf(2.0f * static_cast<float>(M_PI) * 0.25f * t);
                    sim_vel_[i] += (cmd_tau_[i] + disturbance) / J * dt;
                    sim_pos_[i] += sim_vel_[i] * dt;
                    break;
                }
                case SimMode::IDLE:
                    sim_vel_[i] = 0.0f;
                    break;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(2000)); // ~500 Hz
    }
    std::cout << "[Sim] Simulation thread exiting." << std::endl;
}
