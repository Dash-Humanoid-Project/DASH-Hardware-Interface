#pragma once

#include <vector>
#include <cstring>
#include <array>
#include <iostream>
#include "MsgBase.h"

#define Input_Pos_TYPE       float
#define Vel_FF_TYPE          int16_t
#define Torque_FF_TYPE       int16_t
#define Input_Vel_TYPE       float
#define Input_Torque_FF_TYPE float
#define Input_Torque_TYPE    float


struct CommandBase : public MsgBase {};

struct PositionCommand : public CommandBase {
    std::vector<Input_Pos_TYPE> Input_Pos;
    std::vector<Vel_FF_TYPE> Vel_FF;  // Changed to vector for per-motor feedforward
    Torque_FF_TYPE Torque_FF;

    // Custom constructor with per-motor velocity feedforward
    PositionCommand(std::vector<Input_Pos_TYPE> p, std::vector<Vel_FF_TYPE> v = {}, Torque_FF_TYPE t = 0)
        : Input_Pos(p), Vel_FF(v), Torque_FF(t) {
        // If Vel_FF is empty, resize to match Input_Pos size with zeros
        if (Vel_FF.empty() && !Input_Pos.empty()) {
            Vel_FF.resize(Input_Pos.size(), 0);
        }
    }

    // Default constructor (needed for deserialize)
    PositionCommand() = default;

    size_t dataSize() const override {
        // Size byte + position data + velocity_ff data + torque_ff
        size_t num_motors = Input_Pos.empty() ? 0 : Input_Pos.size();
        return sizeof(uint8_t) + num_motors * sizeof(Input_Pos_TYPE) + num_motors * sizeof(Vel_FF_TYPE) + sizeof(Torque_FF);
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

        // Write torque_ff
        offset += vel_ff_size;
        std::memcpy(buffer + offset, &Torque_FF, sizeof(Torque_FF));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        // Read number of motors
        uint8_t num_motors;
        std::memcpy(&num_motors, buffer, sizeof(uint8_t));

        // Resize vectors and read position data
        Input_Pos.resize(num_motors);
        Vel_FF.resize(num_motors);

        size_t pos_size = Input_Pos.size() * sizeof(Input_Pos_TYPE);
        std::memcpy(Input_Pos.data(), buffer + sizeof(uint8_t), pos_size);

        // Read velocity feedforward data
        size_t vel_ff_size = Vel_FF.size() * sizeof(Vel_FF_TYPE);
        size_t offset = sizeof(uint8_t) + pos_size;
        std::memcpy(Vel_FF.data(), buffer + offset, vel_ff_size);

        // Read torque_ff
        offset += vel_ff_size;
        std::memcpy(&Torque_FF, buffer + offset, sizeof(Torque_FF));
    }

    std::tuple<std::vector<Input_Pos_TYPE>, std::vector<Vel_FF_TYPE>, Torque_FF_TYPE> getCommandValue() {
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
        std::cout << "] | Torque_FF: " << Torque_FF << std::endl;
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
