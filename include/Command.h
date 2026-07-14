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
