#pragma once

#include <vector>
#include <cstring>
#include "MsgBase.h"

#define Input_Pos_TYPE       float
#define Vel_FF_TYPE          int16_t
#define Torque_FF_TYPE       int16_t
#define Input_Vel_TYPE       float
#define Input_Torque_FF_TYPE float
#define Input_Torque_TYPE    float


struct CommandBase : public MsgBase {};

struct PositionCommand : public CommandBase {
    Input_Pos_TYPE Input_Pos;
    Vel_FF_TYPE Vel_FF;
    Torque_FF_TYPE Torque_FF;

    // Custom constructor
    PositionCommand(Input_Pos_TYPE p, Vel_FF_TYPE v = 0, Torque_FF_TYPE t = 0)
        : Input_Pos(p), Vel_FF(v), Torque_FF(t) {}

    // Default constructor (needed for deserialize)
    PositionCommand() = default;

    size_t dataSize() const override {
        return sizeof(Input_Pos) + sizeof(Vel_FF) + sizeof(Torque_FF);
    }

    MsgType getType() const override {
        return MsgType::PositionCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        std::memcpy(buffer, &Input_Pos, sizeof(Input_Pos));
        std::memcpy(buffer + sizeof(Input_Pos), &Vel_FF, sizeof(Vel_FF));
        std::memcpy(buffer + sizeof(Input_Pos) + sizeof(Vel_FF), &Torque_FF, sizeof(Torque_FF));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        std::memcpy(&Input_Pos, buffer, sizeof(Input_Pos));
        std::memcpy(&Vel_FF, buffer + sizeof(Input_Pos), sizeof(Vel_FF));
        std::memcpy(&Torque_FF, buffer + sizeof(Input_Pos) + sizeof(Vel_FF), sizeof(Torque_FF));
    }

    std::tuple<Input_Pos_TYPE, Vel_FF_TYPE, Torque_FF_TYPE> getCommandValue() {
        return { Input_Pos, Vel_FF, Torque_FF };
    }

    void printValue() override {
        PRINTLN("Input_Pos: ", Input_Pos,  " | Vel_FF: ", Vel_FF, " | Torque_FF: ", Torque_FF);
    }
};

struct VelocityCommand : public CommandBase {
    Input_Vel_TYPE Input_Vel;
    Input_Torque_FF_TYPE Input_Torque_FF;

    // Custom constructor
    VelocityCommand(Input_Vel_TYPE v, Input_Torque_FF_TYPE t = 0)
        : Input_Vel(v), Input_Torque_FF(t) {}

    // Default constructor (needed for deserialize)
    VelocityCommand() = default;

    size_t dataSize() const override {
        return sizeof(Input_Vel) + sizeof(Input_Torque_FF);
    }

    MsgType getType() const override {
        return MsgType::VelocityCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        std::memcpy(buffer, &Input_Vel, sizeof(Input_Vel));
        std::memcpy(buffer + sizeof(Input_Vel), &Input_Torque_FF, sizeof(Input_Torque_FF));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        std::memcpy(&Input_Vel, buffer, sizeof(Input_Vel));
        std::memcpy(&Input_Torque_FF, buffer + sizeof(Input_Vel), sizeof(Input_Torque_FF));
    }

    std::tuple<Input_Vel_TYPE, Input_Torque_FF_TYPE> getCommandValue() {
        return { Input_Vel, Input_Torque_FF };
    }

    void printValue() override {
        PRINTLN("Input_Vel: ", Input_Vel, " | Input_Torque_FF: ", Input_Torque_FF);
    }
};

struct TorqueCommand : public CommandBase {
    Input_Torque_TYPE Input_Torque;

    // Custom constructor
    TorqueCommand(Input_Torque_TYPE torque)
        : Input_Torque(torque) {}

    // Default constructor (needed for deserialize)
    TorqueCommand() = default;

    size_t dataSize() const override {
        return sizeof(Input_Torque);
    }

    MsgType getType() const override {
        return MsgType::TorqueCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        std::memcpy(buffer, &Input_Torque, sizeof(Input_Torque));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        std::memcpy(&Input_Torque, buffer, sizeof(Input_Torque));
    }
    
    std::tuple<Input_Torque_TYPE> getCommandValue() {
        return { Input_Torque };
    }

    void printValue() override {
        PRINTLN("Input_Torque: ", Input_Torque);
    }
};
