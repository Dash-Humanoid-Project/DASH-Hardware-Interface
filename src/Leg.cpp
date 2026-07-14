#include "Leg.h"
#include "Utils.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace {
constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);

// A gearbox multiplies torque by the reduction ratio, so the motor-shaft
// command (what the ODrive's Input_Torque/Torque_FF expects, assuming true
// motor Kt is configured) must be the joint-space torque divided by that
// ratio, not sent through directly.
float gearRatio(const MotorConfig& m) { return m.turns_per_rad * kTwoPi; }
}

Leg::Leg(UPXtreme& teensy, std::vector<MotorConfig> motors, std::string name)
    : teensy_(teensy), motors_(std::move(motors)), name_(std::move(name))
{
    // Sort motors by (bus_idx, node_idx) to match the Teensy's odrives[] array order:
    // CAN bus 0 first (slots 0, 1, ...), then CAN bus 1 (slots 0, 1, ...).
    std::sort(motors_.begin(), motors_.end(), [](const MotorConfig& a, const MotorConfig& b) {
        if (a.bus_idx != b.bus_idx) return a.bus_idx < b.bus_idx;
        return a.node_idx < b.node_idx;
    });
}

int Leg::motorIndex(const std::string& joint_name) const
{
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i)
        if (motors_[i].joint_name == joint_name) return i;
    return -1;
}

std::map<std::string, JointState> Leg::getJointStates() const
{
    std::map<std::string, JointState> states;
    for (const auto& m : motors_) {
        float pos_turns = teensy_.getPosEstimate(m.bus_idx, m.node_idx);
        float vel_turns = teensy_.getVelEstimate(m.bus_idx, m.node_idx);
        states[m.joint_name] = {
            m.joint_name,
            pos_turns / m.turns_per_rad,
            vel_turns / m.turns_per_rad
        };
    }
    return states;
}

JointState Leg::getJointState(const std::string& joint_name) const
{
    int idx = motorIndex(joint_name);
    if (idx < 0) throw std::runtime_error("Leg::getJointState: unknown joint: " + joint_name);
    const auto& m = motors_[idx];
    float pos_turns = teensy_.getPosEstimate(m.bus_idx, m.node_idx);
    float vel_turns = teensy_.getVelEstimate(m.bus_idx, m.node_idx);
    return {m.joint_name, pos_turns / m.turns_per_rad, vel_turns / m.turns_per_rad};
}

void Leg::setPositions(const std::map<std::string, float>& positions_rad,
                       const std::map<std::string, float>& vel_ff_rad_s,
                       const std::map<std::string, float>& torque_ff_nm)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<Input_Pos_TYPE>  pos(n, 0.0f);
    std::vector<Vel_FF_TYPE>     vel(n, 0);
    std::vector<Torque_FF_TYPE>  tau_ff(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];
        auto pit = positions_rad.find(m.joint_name);
        if (pit != positions_rad.end()) {
            float q = pit->second;
            float q_clamped = clampf(q, m.q_min_rad, m.q_max_rad);
            if (q_clamped != q)
                std::cerr << "[Leg] CLAMPED position: " << m.joint_name
                          << " requested=" << q << " rad, applied=" << q_clamped
                          << " rad (limits [" << m.q_min_rad << ", " << m.q_max_rad << "])\n";
            pos[i] = static_cast<Input_Pos_TYPE>(q_clamped * m.turns_per_rad);
        }

        auto vit = vel_ff_rad_s.find(m.joint_name);
        if (vit != vel_ff_rad_s.end()) {
            float qd = vit->second;
            float qd_clamped = clampf(qd, -m.vel_max_rad_s, m.vel_max_rad_s);
            if (qd_clamped != qd)
                std::cerr << "[Leg] CLAMPED vel_ff: " << m.joint_name
                          << " requested=" << qd << " rad/s, applied=" << qd_clamped
                          << " rad/s (limit +/-" << m.vel_max_rad_s << ")\n";
            vel[i] = static_cast<Vel_FF_TYPE>(qd_clamped * m.turns_per_rad);
        }

        auto tit = torque_ff_nm.find(m.joint_name);
        if (tit != torque_ff_nm.end()) {
            float t = tit->second;
            float t_clamped = clampf(t, -m.tau_max_nm, m.tau_max_nm);
            if (t_clamped != t)
                std::cerr << "[Leg] CLAMPED torque_ff: " << m.joint_name
                          << " requested=" << t << " Nm, applied=" << t_clamped
                          << " Nm (limit +/-" << m.tau_max_nm << ")\n";
            tau_ff[i] = static_cast<Torque_FF_TYPE>(t_clamped / gearRatio(m));
        }
    }

    teensy_.setPositionCommand(std::make_shared<PositionCommand>(pos, vel, tau_ff));
}

void Leg::setVelocities(const std::map<std::string, float>& velocities_rad_s)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<Input_Vel_TYPE> vel(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];
        auto it = velocities_rad_s.find(m.joint_name);
        if (it != velocities_rad_s.end()) {
            float qd = it->second;
            float qd_clamped = clampf(qd, -m.vel_max_rad_s, m.vel_max_rad_s);
            if (qd_clamped != qd)
                std::cerr << "[Leg] CLAMPED velocity: " << m.joint_name
                          << " requested=" << qd << " rad/s, applied=" << qd_clamped
                          << " rad/s (limit +/-" << m.vel_max_rad_s << ")\n";
            vel[i] = static_cast<Input_Vel_TYPE>(qd_clamped * m.turns_per_rad);
        }
    }

    teensy_.setVelocityCommand(std::make_shared<VelocityCommand>(vel));
}

void Leg::setTorques(const std::map<std::string, float>& torques_nm)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<Input_Torque_TYPE> tau(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];
        auto it = torques_nm.find(m.joint_name);
        if (it != torques_nm.end()) {
            float t = it->second;
            float t_clamped = clampf(t, -m.tau_max_nm, m.tau_max_nm);
            if (t_clamped != t)
                std::cerr << "[Leg] CLAMPED torque: " << m.joint_name
                          << " requested=" << t << " Nm, applied=" << t_clamped
                          << " Nm (limit +/-" << m.tau_max_nm << ")\n";

            tau[i] = static_cast<Input_Torque_TYPE>(t_clamped / gearRatio(m));
        }
    }

    teensy_.setTorqueCommand(std::make_shared<TorqueCommand>(tau));
}

void Leg::setGains(const std::map<std::string, float>& pos_gain,
                   const std::map<std::string, float>& vel_gain,
                   const std::map<std::string, float>& vel_integrator_gain)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<float> pg(n), vg(n), vig(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];

        auto pit = pos_gain.find(m.joint_name);
        if (pit == pos_gain.end())
            throw std::runtime_error("Leg::setGains: missing pos_gain for joint: " + m.joint_name);
        pg[i] = pit->second;

        auto vit = vel_gain.find(m.joint_name);
        if (vit == vel_gain.end())
            throw std::runtime_error("Leg::setGains: missing vel_gain for joint: " + m.joint_name);
        vg[i] = vit->second;

        auto vigit = vel_integrator_gain.find(m.joint_name);
        if (vigit != vel_integrator_gain.end()) vig[i] = vigit->second;
    }

    teensy_.sendSetGainsCommand(pg, vg, vig);
}
