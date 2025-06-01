#pragma once

#include <cstring>
#include <iostream>
#include "MsgBase.h"


struct DataBase : public MsgBase {};

struct SystemData : public DataBase {
    float encoder_Pos_Estimate[2];  // [rev]
    float encoder_Vel_Estimate[2];  // [rev/s]

    // Constructor to initialize default values
    SystemData() {
        encoder_Pos_Estimate[0] = 0.0f;
        encoder_Pos_Estimate[1] = 0.0f;
        encoder_Vel_Estimate[0] = 0.0f;
        encoder_Vel_Estimate[1] = 0.0f;
    }

    // Constructor from input arguments
    SystemData(float pos_estimate[2], float vel_estimate[2]) {
        encoder_Pos_Estimate[0] = pos_estimate[0];
        encoder_Pos_Estimate[1] = pos_estimate[1];
        encoder_Vel_Estimate[0] = vel_estimate[0];
        encoder_Vel_Estimate[1] = vel_estimate[1];
    }

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
        std::cout << "encoder_Pos_Estimate: [" << encoder_Pos_Estimate[0] << "," << encoder_Pos_Estimate[1] << "] | encoder_Vel_Estimate: [" << encoder_Vel_Estimate[0] << "," << encoder_Vel_Estimate[1] << "]" << std::endl;
    }
};
