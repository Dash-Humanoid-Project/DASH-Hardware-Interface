#include <vector>
#include "MsgBase.h"


std::vector<uint8_t> MsgBase::serialize()
{
    std::vector<uint8_t> buffer(dataSize());
    writeToBuffer(buffer.data());
    return buffer;
}

std::vector<uint8_t> MsgBase::serializeWithHeader()
{
    std::vector<uint8_t> data;

    // Add message type at the start
    data.push_back(static_cast<uint8_t>(getType()));

    // Add the actual serialized payload
    auto payload = this->serialize();  // derived class implementation
    data.insert(data.end(), payload.begin(), payload.end());

    return data;
}

void MsgBase::deserialize(const std::vector<uint8_t>& buffer)
{
    if (buffer.size() >= dataSize()) {
        readFromBuffer(buffer.data());
    } else {
        PRINTLN("Buffer too small for deserialization");
    }
}

