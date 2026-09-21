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
    // torque_ff_nm is the per-joint feedforward torque added on top of the
    // ODrive's own local position/velocity PD (see LegController) — not a
    // replacement for it.
    void setPositions(const std::map<std::string, float>& positions_rad,
                      const std::map<std::string, float>& vel_ff_rad_s = {},
                      const std::map<std::string, float>& torque_ff_nm = {});
    void setVelocities(const std::map<std::string, float>& velocities_rad_s);
    void setTorques(const std::map<std::string, float>& torques_nm);

    // Sets each joint's local ODrive position/velocity gains. pos_gain and
    // vel_gain must include every joint on this leg — unlike position/
    // velocity/torque commands, there's no safe default gain value (0 would
    // disable local tracking, not "do nothing"), so missing entries throw
    // rather than silently applying a dangerous value. vel_integrator_gain
    // defaults to 0 per joint if omitted (a normal, safe operating choice).
    void setGains(const std::map<std::string, float>& pos_gain,
                 const std::map<std::string, float>& vel_gain,
                 const std::map<std::string, float>& vel_integrator_gain = {});

    const std::string& name() const { return name_; }
    int numJoints() const { return static_cast<int>(motors_.size()); }
    const std::vector<MotorConfig>& motorConfigs() const { return motors_; }

    // Idles / arms just this leg's own Teensy — lets you e.g. hand-guide one
    // limb (backdrivable) while others are left untouched, unlike
    // HardwareBridge::idle()/startClosedLoop() which act on every Teensy.
    void idle() { teensy_.sendIdleCommand(); }
    void startClosedLoop() { teensy_.sendStartCommand(); }

    // Reads/writes one arbitrary ODrive-native CAN parameter ("endpoint_id" —
    // ODrive's own numbering; get it via the odrive Python package:
    // type(obj).__dict__[name]._info.endpoint_id) on one joint's ODrive.
    // Diagnostic/config use only — not for the control loop, and not safe to
    // call while this leg is under active full-rate closed-loop control
    // (the Teensy blocks its main loop for up to ~10ms per get). Throws if
    // joint_name is unknown or the get times out / the response doesn't
    // match what was requested.
    float   getParamFloat(const std::string& joint_name, uint16_t endpoint_id, int timeout_ms = 200) const;
    bool    getParamBool (const std::string& joint_name, uint16_t endpoint_id, int timeout_ms = 200) const;
    uint8_t getParamUint8(const std::string& joint_name, uint16_t endpoint_id, int timeout_ms = 200) const;
    int32_t getParamInt32(const std::string& joint_name, uint16_t endpoint_id, int timeout_ms = 200) const;

    void setParamFloat(const std::string& joint_name, uint16_t endpoint_id, float value);
    void setParamBool (const std::string& joint_name, uint16_t endpoint_id, bool value);
    void setParamUint8(const std::string& joint_name, uint16_t endpoint_id, uint8_t value);
    void setParamInt32(const std::string& joint_name, uint16_t endpoint_id, int32_t value);

private:
    UPXtreme& teensy_;
    std::vector<MotorConfig> motors_;  // sorted by (bus_idx, node_idx)
    std::string name_;

    int motorIndex(const std::string& joint_name) const;

    // Shared implementation for the typed getParam*/setParam* wrappers above —
    // resolves joint_name to the Teensy's odrives[] index and does the
    // request (+ response wait, for get). Throws std::runtime_error on an
    // unknown joint, a timeout, or a response that doesn't match the request.
    void setParamRaw(const std::string& joint_name, uint16_t endpoint_id,
                     ParamType type, const uint8_t value[4]);
    void getParamRaw(const std::string& joint_name, uint16_t endpoint_id,
                     ParamType type, uint8_t out_value[4], int timeout_ms) const;
};
