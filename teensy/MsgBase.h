#pragma once

#include <vector>
#include <memory>
#include "Log.h"

enum class MsgType : uint8_t {
    PositionCommand = 0x01,
    VelocityCommand = 0x02,
    TorqueCommand   = 0x03,
    SystemData      = 0x04,
    IdleCommand     = 0x05,
    StartCommand    = 0x06,
};

struct MsgBase {
    virtual size_t dataSize() const = 0;
    virtual MsgType getType() const = 0;

    virtual void writeToBuffer(uint8_t* buffer) const = 0;
    virtual void readFromBuffer(const uint8_t* buffer) = 0;
    virtual void printValue() = 0;

    std::vector<uint8_t> serialize();

    std::vector<uint8_t> serializeWithHeader();

    void deserialize(const std::vector<uint8_t>& buffer);

    virtual ~MsgBase() = default;
};
