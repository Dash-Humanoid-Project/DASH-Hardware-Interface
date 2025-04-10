#include <vector>
#include <cstring>
#include <iostream>
#include "CommandBase.h"
#include "Command.h"

#define Input_Pos_TYPE float
#define Vel_FF_TYPE uint16_t
#define Torque_FF_TYPE uint16_t
#define Input_Torque_TYPE float


std::vector<uint8_t> CommandBase::serialize()
{
    std::vector<uint8_t> buffer(dataSize());
    writeToBuffer(buffer.data());
    return buffer;
}

std::vector<uint8_t> CommandBase::serializeWithHeader()
{
    std::vector<uint8_t> data;

    // Add message type at the start
    data.push_back(static_cast<uint8_t>(getType()));

    // Add the actual serialized payload
    auto payload = this->serialize();  // derived class implementation
    data.insert(data.end(), payload.begin(), payload.end());

    return data;
}

void CommandBase::deserialize(const std::vector<uint8_t>& buffer)
{
    if (buffer.size() >= dataSize()) {
        readFromBuffer(buffer.data(), buffer.size());
    } else {
        std::cerr << "Buffer too small for deserialization\n";
    }
}

std::unique_ptr<CommandBase> CommandBase::fromBuffer(const std::vector<uint8_t>& buffer)
{
    if (buffer.empty()) {
        std::cerr << "Empty buffer!\n";
        return nullptr;
    }

    MsgType type = static_cast<MsgType>(buffer[0]);
    // TODO(@nicholasadr): unnecessary?
    std::vector<uint8_t> payload(buffer.begin() + 1, buffer.end());

    switch (type) {
        case MsgType::PositionCommand:
        {
            PositionCommand cmd;
            cmd.deserialize(payload);
            return std::make_unique<PositionCommand>(cmd);
        }
        case MsgType::TorqueCommand:
        {
            TorqueCommand cmd;
            cmd.deserialize(payload);
            return std::make_unique<TorqueCommand>(cmd);
        }
        default:
            std::cerr << "Unknown MsgType!\n";
            return nullptr;
    }
}
