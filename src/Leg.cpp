#include "Leg.h"
#include <algorithm>
#include <stdexcept>

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
                       const std::map<std::string, float>& vel_ff_rad_s)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<Input_Pos_TYPE> pos(n, 0.0f);
    std::vector<Vel_FF_TYPE>    vel(n, 0);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];
        auto pit = positions_rad.find(m.joint_name);
        if (pit != positions_rad.end())
            pos[i] = static_cast<Input_Pos_TYPE>(pit->second * m.turns_per_rad);

        auto vit = vel_ff_rad_s.find(m.joint_name);
        if (vit != vel_ff_rad_s.end())
            vel[i] = static_cast<Vel_FF_TYPE>(vit->second * m.turns_per_rad);
    }

    teensy_.setPositionCommand(std::make_shared<PositionCommand>(pos, vel));
}

void Leg::setVelocities(const std::map<std::string, float>& velocities_rad_s)
{
    const int n = static_cast<int>(motors_.size());
    std::vector<Input_Vel_TYPE> vel(n, 0.0f);

    for (int i = 0; i < n; ++i) {
        const auto& m = motors_[i];
        auto it = velocities_rad_s.find(m.joint_name);
        if (it != velocities_rad_s.end())
            vel[i] = static_cast<Input_Vel_TYPE>(it->second * m.turns_per_rad);
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
        if (it != torques_nm.end())
            tau[i] = static_cast<Input_Torque_TYPE>(it->second);
    }

    teensy_.setTorqueCommand(std::make_shared<TorqueCommand>(tau));
}
