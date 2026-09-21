#pragma once

#include <vector>
#include <cstring>
#include <array>
#include <iostream>
#include "MsgBase.h"

#define Input_Pos_TYPE       float
#define Vel_FF_TYPE          float
#define Torque_FF_TYPE       float
#define Input_Vel_TYPE       float
#define Input_Torque_FF_TYPE float
#define Input_Torque_TYPE    float


struct CommandBase : public MsgBase {};

struct PositionCommand : public CommandBase {
    std::vector<Input_Pos_TYPE> Input_Pos;
    std::vector<Vel_FF_TYPE> Vel_FF;      // per-motor velocity feedforward
    std::vector<Torque_FF_TYPE> Torque_FF; // per-motor torque feedforward

    // Custom constructor with per-motor velocity/torque feedforward
    PositionCommand(std::vector<Input_Pos_TYPE> p, std::vector<Vel_FF_TYPE> v = {}, std::vector<Torque_FF_TYPE> t = {})
        : Input_Pos(p), Vel_FF(v), Torque_FF(t) {
        // If Vel_FF/Torque_FF are empty, resize to match Input_Pos size with zeros
        if (Vel_FF.empty() && !Input_Pos.empty()) {
            Vel_FF.resize(Input_Pos.size(), 0);
        }
        if (Torque_FF.empty() && !Input_Pos.empty()) {
            Torque_FF.resize(Input_Pos.size(), 0);
        }
    }

    // Default constructor (needed for deserialize)
    PositionCommand() = default;

    size_t dataSize() const override {
        // Size byte + position data + velocity_ff data + torque_ff data
        size_t num_motors = Input_Pos.empty() ? 0 : Input_Pos.size();
        return sizeof(uint8_t) + num_motors * sizeof(Input_Pos_TYPE) + num_motors * sizeof(Vel_FF_TYPE) + num_motors * sizeof(Torque_FF_TYPE);
    }

    MsgType getType() const override {
        return MsgType::PositionCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        // Write number of motors first
        uint8_t num_motors = static_cast<uint8_t>(Input_Pos.size());
        std::memcpy(buffer, &num_motors, sizeof(uint8_t));

        // Write position data
        size_t pos_size = Input_Pos.size() * sizeof(Input_Pos_TYPE);
        std::memcpy(buffer + sizeof(uint8_t), Input_Pos.data(), pos_size);

        // Write velocity feedforward data
        size_t vel_ff_size = Vel_FF.size() * sizeof(Vel_FF_TYPE);
        size_t offset = sizeof(uint8_t) + pos_size;
        std::memcpy(buffer + offset, Vel_FF.data(), vel_ff_size);

        // Write torque feedforward data
        size_t torque_ff_size = Torque_FF.size() * sizeof(Torque_FF_TYPE);
        offset += vel_ff_size;
        std::memcpy(buffer + offset, Torque_FF.data(), torque_ff_size);
    }

    void readFromBuffer(const uint8_t* buffer) override {
        // Read number of motors
        uint8_t num_motors;
        std::memcpy(&num_motors, buffer, sizeof(uint8_t));

        // Resize vectors and read position data
        Input_Pos.resize(num_motors);
        Vel_FF.resize(num_motors);
        Torque_FF.resize(num_motors);

        size_t pos_size = Input_Pos.size() * sizeof(Input_Pos_TYPE);
        std::memcpy(Input_Pos.data(), buffer + sizeof(uint8_t), pos_size);

        // Read velocity feedforward data
        size_t vel_ff_size = Vel_FF.size() * sizeof(Vel_FF_TYPE);
        size_t offset = sizeof(uint8_t) + pos_size;
        std::memcpy(Vel_FF.data(), buffer + offset, vel_ff_size);

        // Read torque feedforward data
        size_t torque_ff_size = Torque_FF.size() * sizeof(Torque_FF_TYPE);
        offset += vel_ff_size;
        std::memcpy(Torque_FF.data(), buffer + offset, torque_ff_size);
    }

    std::tuple<std::vector<Input_Pos_TYPE>, std::vector<Vel_FF_TYPE>, std::vector<Torque_FF_TYPE>> getCommandValue() {
        return { Input_Pos, Vel_FF, Torque_FF };
    }

    void printValue() override {
        std::cout << "Input_Pos: [";
        for (size_t i = 0; i < Input_Pos.size(); ++i) {
            std::cout << Input_Pos[i];
            if (i < Input_Pos.size() - 1) std::cout << ", ";
        }
        std::cout << "] | Vel_FF: [";
        for (size_t i = 0; i < Vel_FF.size(); ++i) {
            std::cout << Vel_FF[i];
            if (i < Vel_FF.size() - 1) std::cout << ", ";
        }
        std::cout << "] | Torque_FF: [";
        for (size_t i = 0; i < Torque_FF.size(); ++i) {
            std::cout << Torque_FF[i];
            if (i < Torque_FF.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
};

struct VelocityCommand : public CommandBase {
    std::vector<Input_Vel_TYPE> Input_Vel;
    Input_Torque_FF_TYPE Input_Torque_FF;

    // Custom constructor
    VelocityCommand(std::vector<Input_Vel_TYPE> v, Input_Torque_FF_TYPE t = 0)
        : Input_Vel(v), Input_Torque_FF(t) {}

    // Default constructor (needed for deserialize)
    VelocityCommand() = default;

    size_t dataSize() const override {
        size_t num_motors = Input_Vel.empty() ? 0 : Input_Vel.size();
        return sizeof(uint8_t) + num_motors * sizeof(Input_Vel_TYPE) + sizeof(Input_Torque_FF_TYPE);
    }

    MsgType getType() const override {
        return MsgType::VelocityCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        uint8_t num_motors = Input_Vel.size();
        std::memcpy(buffer, &num_motors, sizeof(uint8_t));
        std::memcpy(buffer + sizeof(uint8_t), Input_Vel.data(), num_motors * sizeof(Input_Vel_TYPE));
        std::memcpy(buffer + sizeof(uint8_t) + num_motors * sizeof(Input_Vel_TYPE), &Input_Torque_FF, sizeof(Input_Torque_FF_TYPE));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        uint8_t num_motors;
        std::memcpy(&num_motors, buffer, sizeof(uint8_t));
        Input_Vel.resize(num_motors);
        std::memcpy(Input_Vel.data(), buffer + sizeof(uint8_t), num_motors * sizeof(Input_Vel_TYPE));
        std::memcpy(&Input_Torque_FF, buffer + sizeof(uint8_t) + num_motors * sizeof(Input_Vel_TYPE), sizeof(Input_Torque_FF_TYPE));
    }

    std::tuple<std::vector<Input_Vel_TYPE>, Input_Torque_FF_TYPE> getCommandValue() {
        return { Input_Vel, Input_Torque_FF };
    }

    void printValue() override {
        std::cout << "Input_Vel: [";
        for (size_t i = 0; i < Input_Vel.size(); ++i) {
            std::cout << Input_Vel[i];
            if (i < Input_Vel.size() - 1) std::cout << ", ";
        }
        std::cout << "] | Torque_FF: " << Input_Torque_FF << std::endl;
    }
};

struct TorqueCommand : public CommandBase {
    std::vector<Input_Torque_TYPE> Input_Torque;

    // Custom constructor
    TorqueCommand(std::vector<Input_Torque_TYPE> tau)
        : Input_Torque(tau) {}

    // Default constructor (needed for deserialize)
    TorqueCommand() = default;

    size_t dataSize() const override {
        size_t num_motors = Input_Torque.empty() ? 0 : Input_Torque.size();
        return sizeof(uint8_t) + num_motors * sizeof(Input_Torque_TYPE);
    }

    MsgType getType() const override {
        return MsgType::TorqueCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        uint8_t num_motors = Input_Torque.size();
        std::memcpy(buffer, &num_motors, sizeof(uint8_t));
        std::memcpy(buffer + sizeof(uint8_t), Input_Torque.data(), num_motors * sizeof(Input_Torque_TYPE));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        uint8_t num_motors;
        std::memcpy(&num_motors, buffer, sizeof(uint8_t));
        Input_Torque.resize(num_motors);
        std::memcpy(Input_Torque.data(), buffer + sizeof(uint8_t), num_motors * sizeof(Input_Torque_TYPE));
    }

    std::tuple<std::vector<Input_Torque_TYPE>> getCommandValue() {
        return { Input_Torque };
    }

    void printValue() override {
        std::cout << "Input_Torque: [";
        for (size_t i = 0; i < Input_Torque.size(); ++i) {
            std::cout << Input_Torque[i];
            if (i < Input_Torque.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
};

struct IdleCommand : public CommandBase {
    // No data needed - just a signal to idle all motors

    IdleCommand() = default;

    size_t dataSize() const override {
        return 0;  // No payload
    }

    MsgType getType() const override {
        return MsgType::IdleCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        // No data to write
    }

    void readFromBuffer(const uint8_t* buffer) override {
        // No data to read
    }

    void printValue() override {
        std::cout << "IdleCommand" << std::endl;
    }
};

struct StartCommand : public CommandBase {
    // No data needed - just a signal to enable closed-loop control

    StartCommand() = default;

    size_t dataSize() const override {
        return 0;  // No payload
    }

    MsgType getType() const override {
        return MsgType::StartCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        // No data to write
    }

    void readFromBuffer(const uint8_t* buffer) override {
        // No data to read
    }

    void printValue() override {
        std::cout << "StartCommand" << std::endl;
    }
};

// No-op keep-alive: resets the Teensy's comms-loss watchdog timer only.
// Unlike Position/Velocity/TorqueCommand, this must NOT trigger any
// setControllerMode()/setState() call on the Teensy — its whole purpose is
// to satisfy the watchdog during a settling wait without implying or
// switching to any particular control mode.
struct HeartbeatCommand : public CommandBase {
    HeartbeatCommand() = default;

    size_t dataSize() const override {
        return 0;  // No payload
    }

    MsgType getType() const override {
        return MsgType::Heartbeat;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        // No data to write
    }

    void readFromBuffer(const uint8_t* buffer) override {
        // No data to read
    }

    void printValue() override {
        std::cout << "HeartbeatCommand" << std::endl;
    }
};

#define Pos_Gain_TYPE float
#define Vel_Gain_TYPE float
#define Vel_Integrator_Gain_TYPE float

// Sets each ODrive's own onboard position/velocity gains (odrives[i]->setPosGain()/
// setVelGains()). Lets the PC vary local tracking stiffness without touching
// the joint-space PD math itself, which stays local to the ODrive — see
// LegController. Not sent every cycle; only when gains actually change.
struct SetGainsCommand : public CommandBase {
    std::vector<Pos_Gain_TYPE> Pos_Gain;
    std::vector<Vel_Gain_TYPE> Vel_Gain;
    std::vector<Vel_Integrator_Gain_TYPE> Vel_Integrator_Gain;

    SetGainsCommand(std::vector<Pos_Gain_TYPE> pg,
                     std::vector<Vel_Gain_TYPE> vg,
                     std::vector<Vel_Integrator_Gain_TYPE> vig)
        : Pos_Gain(pg), Vel_Gain(vg), Vel_Integrator_Gain(vig) {}

    SetGainsCommand() = default;

    size_t dataSize() const override {
        size_t num_motors = Pos_Gain.empty() ? 0 : Pos_Gain.size();
        return sizeof(uint8_t)
            + num_motors * sizeof(Pos_Gain_TYPE)
            + num_motors * sizeof(Vel_Gain_TYPE)
            + num_motors * sizeof(Vel_Integrator_Gain_TYPE);
    }

    MsgType getType() const override {
        return MsgType::SetGains;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        uint8_t num_motors = static_cast<uint8_t>(Pos_Gain.size());
        std::memcpy(buffer, &num_motors, sizeof(uint8_t));

        size_t pg_size = Pos_Gain.size() * sizeof(Pos_Gain_TYPE);
        size_t offset = sizeof(uint8_t);
        std::memcpy(buffer + offset, Pos_Gain.data(), pg_size);

        size_t vg_size = Vel_Gain.size() * sizeof(Vel_Gain_TYPE);
        offset += pg_size;
        std::memcpy(buffer + offset, Vel_Gain.data(), vg_size);

        size_t vig_size = Vel_Integrator_Gain.size() * sizeof(Vel_Integrator_Gain_TYPE);
        offset += vg_size;
        std::memcpy(buffer + offset, Vel_Integrator_Gain.data(), vig_size);
    }

    void readFromBuffer(const uint8_t* buffer) override {
        uint8_t num_motors;
        std::memcpy(&num_motors, buffer, sizeof(uint8_t));

        Pos_Gain.resize(num_motors);
        Vel_Gain.resize(num_motors);
        Vel_Integrator_Gain.resize(num_motors);

        size_t pg_size = Pos_Gain.size() * sizeof(Pos_Gain_TYPE);
        size_t offset = sizeof(uint8_t);
        std::memcpy(Pos_Gain.data(), buffer + offset, pg_size);

        size_t vg_size = Vel_Gain.size() * sizeof(Vel_Gain_TYPE);
        offset += pg_size;
        std::memcpy(Vel_Gain.data(), buffer + offset, vg_size);

        size_t vig_size = Vel_Integrator_Gain.size() * sizeof(Vel_Integrator_Gain_TYPE);
        offset += vg_size;
        std::memcpy(Vel_Integrator_Gain.data(), buffer + offset, vig_size);
    }

    void printValue() override {
        std::cout << "Pos_Gain: [";
        for (size_t i = 0; i < Pos_Gain.size(); ++i) {
            std::cout << Pos_Gain[i];
            if (i < Pos_Gain.size() - 1) std::cout << ", ";
        }
        std::cout << "] | Vel_Gain: [";
        for (size_t i = 0; i < Vel_Gain.size(); ++i) {
            std::cout << Vel_Gain[i];
            if (i < Vel_Gain.size() - 1) std::cout << ", ";
        }
        std::cout << "] | Vel_Integrator_Gain: [";
        for (size_t i = 0; i < Vel_Integrator_Gain.size(); ++i) {
            std::cout << Vel_Integrator_Gain[i];
            if (i < Vel_Integrator_Gain.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
};

// Encodes the C++ type of the value payload below (see GetSetParamCommand),
// since `value` is always a raw 4-byte slot regardless of the ODrive
// endpoint's real type.
enum class ParamType : uint8_t { FLOAT = 0, BOOL = 1, UINT8 = 2, INT32 = 3 };
enum class ParamOp   : uint8_t { GET = 0, SET = 1 };

// Reads or writes one arbitrary ODrive-native CAN parameter ("endpoint",
// ODrive's own numbering from flat_endpoints.json) on one motor. Unlike
// PositionCommand/SetGainsCommand this always targets exactly one motor
// (motor_idx, matching the Teensy's odrives[] index), so there's no
// num_motors-prefixed variable-length payload — the wire size is fixed.
// Diagnostic/config use only, not sent every control cycle. See Leg::getParam*/
// setParam* (src/Leg.cpp) for the PC-side entry point, and teensy2.ino's
// `case MsgType::GetSetParam` for how this gets relayed to
// ODriveCAN::getEndpoint<T>()/setEndpoint<T>().
struct GetSetParamCommand : public CommandBase {
    uint8_t  motor_idx   = 0;  // index into Teensy's odrives[] (post-sort bus/node order)
    uint8_t  op          = 0;  // ParamOp
    uint16_t endpoint_id = 0;  // ODrive flat_endpoints.json id
    uint8_t  type_tag    = 0;  // ParamType
    uint8_t  value[4]    = {}; // little-endian encoded value; only meaningful when op==SET

    GetSetParamCommand() = default;
    GetSetParamCommand(uint8_t m, uint8_t o, uint16_t ep, uint8_t t, const uint8_t v[4])
        : motor_idx(m), op(o), endpoint_id(ep), type_tag(t) {
        std::memcpy(value, v, 4);
    }

    size_t dataSize() const override {
        return sizeof(motor_idx) + sizeof(op) + sizeof(endpoint_id)
             + sizeof(type_tag) + sizeof(value);
    }

    MsgType getType() const override {
        return MsgType::GetSetParam;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        size_t offset = 0;
        std::memcpy(buffer + offset, &motor_idx, sizeof(motor_idx));       offset += sizeof(motor_idx);
        std::memcpy(buffer + offset, &op, sizeof(op));                     offset += sizeof(op);
        std::memcpy(buffer + offset, &endpoint_id, sizeof(endpoint_id));   offset += sizeof(endpoint_id);
        std::memcpy(buffer + offset, &type_tag, sizeof(type_tag));         offset += sizeof(type_tag);
        std::memcpy(buffer + offset, value, sizeof(value));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        size_t offset = 0;
        std::memcpy(&motor_idx, buffer + offset, sizeof(motor_idx));       offset += sizeof(motor_idx);
        std::memcpy(&op, buffer + offset, sizeof(op));                     offset += sizeof(op);
        std::memcpy(&endpoint_id, buffer + offset, sizeof(endpoint_id));   offset += sizeof(endpoint_id);
        std::memcpy(&type_tag, buffer + offset, sizeof(type_tag));         offset += sizeof(type_tag);
        std::memcpy(value, buffer + offset, sizeof(value));
    }

    void printValue() override {
        std::cout << "GetSetParamCommand: motor_idx=" << static_cast<int>(motor_idx)
                   << " op=" << static_cast<int>(op)
                   << " endpoint_id=" << endpoint_id
                   << " type_tag=" << static_cast<int>(type_tag) << std::endl;
    }
};

// Teensy -> PC reply to a GetSetParamCommand with op==GET. Rides a separate,
// dedicated low-rate UDP socket (see UPXtreme::receiveParamResponse) — not
// the 500Hz SystemData feedback path — so it carries no MsgBase/CRC framing
// of its own; a malformed/short packet just fails to parse on the PC side.
//
// wireSize()/pack()/unpack() do explicit field-by-field (un)packing rather
// than a raw memcpy of sizeof(ParamResponse) — the struct's natural layout
// may include compiler-inserted padding before `endpoint_id` to align it,
// and that padding isn't guaranteed identical between the Teensy's ARM
// compiler and the PC's x86 build. The wire format is always exactly 9
// bytes: motor_idx(1) + endpoint_id(2, LE) + type_tag(1) + value(4) + ok(1).
struct ParamResponse {
    uint8_t  motor_idx;
    uint16_t endpoint_id;
    uint8_t  type_tag;
    uint8_t  value[4];
    // Always 1 today: the vendored ODriveCAN::getEndpoint<T>() returns T{}
    // (zero) on both a genuine CAN timeout and a real value of zero, with
    // no way to tell those apart, so the Teensy has no real failure signal
    // to report here. Kept as a field (rather than removed) so a future,
    // more capable getEndpoint() can start reporting real failures without
    // another wire-format change.
    uint8_t  ok;

    static constexpr size_t wireSize() { return 1 + 2 + 1 + 4 + 1; }

    void pack(uint8_t* buffer) const {
        size_t offset = 0;
        std::memcpy(buffer + offset, &motor_idx, 1);    offset += 1;
        std::memcpy(buffer + offset, &endpoint_id, 2);  offset += 2;
        std::memcpy(buffer + offset, &type_tag, 1);     offset += 1;
        std::memcpy(buffer + offset, value, 4);         offset += 4;
        std::memcpy(buffer + offset, &ok, 1);
    }

    static ParamResponse unpack(const uint8_t* buffer) {
        ParamResponse r{};
        size_t offset = 0;
        std::memcpy(&r.motor_idx, buffer + offset, 1);    offset += 1;
        std::memcpy(&r.endpoint_id, buffer + offset, 2);  offset += 2;
        std::memcpy(&r.type_tag, buffer + offset, 1);     offset += 1;
        std::memcpy(r.value, buffer + offset, 4);         offset += 4;
        std::memcpy(&r.ok, buffer + offset, 1);
        return r;
    }
};
