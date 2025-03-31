#pragma once

#include <vector>
#include <cstring>
#include <iostream>
#include "CommandBase.h"

#define Input_Pos_TYPE float
#define Vel_FF_TYPE uint16_t
#define Torque_FF_TYPE uint16_t
#define Input_Torque_TYPE float


struct PositionCommand : public CommandBase {
    float position;
    int16_t velocity_ff;

    // Custom constructor
    PositionCommand(float pos, int16_t vel)
        : position(pos), velocity_ff(vel) {}

    // Default constructor (needed for deserialize)
    PositionCommand() = default;

    size_t dataSize() const override {
        return sizeof(position) + sizeof(velocity_ff);
    }

    MsgType getType() const override {
        return MsgType::PositionCommand;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        std::memcpy(buffer, &position, sizeof(position));
        std::memcpy(buffer + sizeof(position), &velocity_ff, sizeof(velocity_ff));
    }

    void readFromBuffer(const uint8_t* buffer, size_t size) override {
        std::memcpy(&position, buffer, sizeof(position));
        std::memcpy(&velocity_ff, buffer + sizeof(position), sizeof(velocity_ff));
    }

    void printValue() override {
        std::cout << "pos: " << position << " | velocity_ff: " << velocity_ff << std::endl;
    }
};

struct TorqueCommand : public CommandBase {
    float Input_Torque;

    // Custom constructor
    TorqueCommand(float torque)
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

    void readFromBuffer(const uint8_t* buffer, size_t size) override {
        std::memcpy(&Input_Torque, buffer, sizeof(Input_Torque));
    }
    
    void printValue() override {
        std::cout << "torque: " << Input_Torque << std::endl;
    }

};
