#pragma once

#include <cstring>
#include <iostream>
#include "MsgBase.h"


struct DataBase : public MsgBase {};

struct SystemData : public DataBase {
    float encoder_Pos_Estimate;  // [rev]
    float encoder_Vel_Estimate;  // [rev/s]

    // Constructor to initialize default values
    SystemData() : encoder_Pos_Estimate(0.0f), encoder_Vel_Estimate(0.0f) {}

    // Constructor from input arguments
    SystemData(float pos_estimate, float vel_estimate) : encoder_Pos_Estimate(pos_estimate), encoder_Vel_Estimate(vel_estimate) {}

    size_t dataSize() const override {
        return sizeof(encoder_Pos_Estimate) + sizeof(encoder_Vel_Estimate);
    }

    MsgType getType() const override {
        return MsgType::SystemData;
    }

    void writeToBuffer(uint8_t* buffer) const {
        std::memcpy(buffer, &encoder_Pos_Estimate, sizeof(encoder_Pos_Estimate));
        std::memcpy(buffer + sizeof(encoder_Pos_Estimate), &encoder_Vel_Estimate, sizeof(encoder_Vel_Estimate));
    }

    void readFromBuffer(const uint8_t* buffer) {
        std::memcpy(&encoder_Pos_Estimate, buffer, sizeof(encoder_Pos_Estimate));
        std::memcpy(&encoder_Vel_Estimate, buffer + sizeof(encoder_Pos_Estimate), sizeof(encoder_Vel_Estimate));
    }

    void printValue() override {
        std::cout << "encoder_Pos_Estimate: " << encoder_Pos_Estimate << " | encoder_Vel_Estimate: " << encoder_Vel_Estimate << std::endl;
    }
};
