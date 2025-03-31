#pragma once

#include <vector>
#include <cstring>
#include <iostream>
#include <memory>

#define Input_Pos_TYPE float
#define Vel_FF_TYPE uint16_t
#define Torque_FF_TYPE uint16_t
#define Input_Torque_TYPE float

// Forward declarations
struct PositionCommand;
struct TorqueCommand; 

enum class MsgType : uint8_t {
    PositionCommand = 0x01,
    TorqueCommand   = 0x02,
};

struct CommandBase {
    virtual size_t dataSize() const = 0;
    virtual MsgType getType() const = 0;

    virtual void writeToBuffer(uint8_t* buffer) const = 0;
    virtual void readFromBuffer(const uint8_t* buffer, size_t size) = 0;
    virtual void printValue() = 0;

    std::vector<uint8_t> serialize();

    std::vector<uint8_t> serializeWithHeader();

    void deserialize(const std::vector<uint8_t>& buffer);

    std::unique_ptr<CommandBase> fromBuffer(const std::vector<uint8_t>& buffer);

    virtual ~CommandBase() = default;
};
