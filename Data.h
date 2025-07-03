#pragma once

#include <cstring>
#include <iostream>
#include "MsgBase.h"


struct DataBase : public MsgBase {};

template <size_t N>
struct SystemData : public DataBase {
    std::array<float, N> encoder_Pos_Estimate{};  // [rev]
    std::array<float, N> encoder_Vel_Estimate{};  // [rev/s]

    SystemData() = default;

    // Constructor from input arguments
    SystemData(const std::array<float, N>& pos, const std::array<float, N>& vel)
        : encoder_Pos_Estimate(pos), encoder_Vel_Estimate(vel) {}

    size_t dataSize() const override {
        return sizeof(float) * N * 2;
    }

    MsgType getType() const override {
        return MsgType::SystemData;
    }

    void writeToBuffer(uint8_t* buffer) const {
        std::memcpy(buffer, encoder_Pos_Estimate.data(), sizeof(float) * N);
        std::memcpy(buffer + sizeof(float) * N, encoder_Vel_Estimate.data(), sizeof(float) * N);
    }

    void readFromBuffer(const uint8_t* buffer) {
        std::memcpy(encoder_Pos_Estimate.data(), buffer, sizeof(float) * N);
        std::memcpy(encoder_Vel_Estimate.data(), buffer + sizeof(float) * N, sizeof(float) * N);
    }

    void printValue() override {
        std::cout << "encoder_Pos_Estimate: [";
        for (size_t i = 0; i < N; ++i) std::cout << encoder_Pos_Estimate[i] << (i < N-1 ? "," : "");
        std::cout << "] | encoder_Vel_Estimate: [";
        for (size_t i = 0; i < N; ++i) std::cout << encoder_Vel_Estimate[i] << (i < N-1 ? "," : "");
        std::cout << "]" << std::endl;
    }
};
