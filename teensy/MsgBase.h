#pragma once

#include <vector>
#include <cstring>
#include <iostream>
#include <memory>

enum class MsgType : uint8_t {
    Config          = 0x01,
    PositionCommand = 0x02,
    VelocityCommand = 0x03,
    TorqueCommand   = 0x04,
    SystemData      = 0x05,
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
