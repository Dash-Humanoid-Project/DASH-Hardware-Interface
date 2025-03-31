#pragma once

#include <cstring>

struct SystemData {
    float encoder_Pos_Estimate;  // [rev]
    float encoder_Vel_Estimate;  // [rev/s]

    // Constructor to initialize default values
    SystemData() : encoder_Pos_Estimate(0.0f), encoder_Vel_Estimate(0.0f) {}

    // Constructor from input arguments
    SystemData(float pos_estimate, float vel_estimate) : encoder_Pos_Estimate(pos_estimate), encoder_Vel_Estimate(vel_estimate) {}

    // Serialize the structure into a byte array
    void serialize(uint8_t* buffer) const {
        // Serialize Pos_Estimate and Vel_Estimate into the buffer
        std::memcpy(buffer, &encoder_Pos_Estimate, sizeof(encoder_Pos_Estimate));
        std::memcpy(buffer + sizeof(encoder_Pos_Estimate), &encoder_Vel_Estimate, sizeof(encoder_Vel_Estimate));
    }

    // Deserialize from byte array
    static SystemData deserialize(const std::vector<uint8_t>& buffer) {
        SystemData msg;

        // Deserialize Pos_Estimate and Vel_Estimate from the buffer
        std::memcpy(&msg.encoder_Pos_Estimate, buffer.data(), sizeof(encoder_Pos_Estimate));
        std::memcpy(&msg.encoder_Vel_Estimate, buffer.data() + sizeof(encoder_Pos_Estimate), sizeof(encoder_Vel_Estimate));

        return msg;
    }
};
