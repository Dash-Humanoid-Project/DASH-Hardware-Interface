#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "MotorConfig.h"
#include "JointState.h"
#include "UPXtreme.h"
#include "Command.h"

// Represents one leg (or any named group of motors driven by a single Teensy).
//
// All public interfaces work in SI units (radians, rad/s, Nm).
// Internal conversion to ODrive turns/s uses MotorConfig::turns_per_rad.
//
// Motor ordering in the command vector follows (bus_idx, node_idx) sort order,
// which matches the Teensy's odrives[] array layout (CAN bus 0 first, then bus 1,
// slots in ascending order within each bus).
class Leg {
public:
    Leg(UPXtreme& teensy, std::vector<MotorConfig> motors, std::string name);

    // Feedback — returns state in SI units (rad, rad/s) after gear-ratio conversion.
    std::map<std::string, JointState> getJointStates() const;
    JointState getJointState(const std::string& joint_name) const;

    // Commands — inputs in SI units; converted to ODrive turns internally.
    // Joints absent from the map are commanded to 0 (position) or 0 torque/velocity.
    void setPositions(const std::map<std::string, float>& positions_rad,
                      const std::map<std::string, float>& vel_ff_rad_s = {});
    void setVelocities(const std::map<std::string, float>& velocities_rad_s);
    void setTorques(const std::map<std::string, float>& torques_nm);

    const std::string& name() const { return name_; }
    int numJoints() const { return static_cast<int>(motors_.size()); }
    const std::vector<MotorConfig>& motorConfigs() const { return motors_; }

private:
    UPXtreme& teensy_;
    std::vector<MotorConfig> motors_;  // sorted by (bus_idx, node_idx)
    std::string name_;

    int motorIndex(const std::string& joint_name) const;
};
