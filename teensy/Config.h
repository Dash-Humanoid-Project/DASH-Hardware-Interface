#pragma once

#include <vector>
#include <cstring>
#include <iostream>
#include <limits>
#include "MsgBase.h"
#include "ODriveEnums.h"

struct ConfigBase : public MsgBase {};

struct Config : public ConfigBase {
    uint8_t num_config = 34;
    float dc_bus_overvoltage_trip_lvl=26;
    float dc_bus_undervoltage_trip_lvl=22;
    float dc_max_positive_current=std::numeric_limits<float>::infinity();
    float dc_max_negative_current=-std::numeric_limits<float>::infinity();
    uint8_t motor_type=ODriveMotorType::MOTOR_TYPE_HIGH_CURRENT;
    uint32_t motor_pole_pairs=21;
    float motor_torque_constant=0.1378333;
    float motor_current_soft_max=23;
    float motor_current_hard_max=39.9;
    float calibration_current=10;
    float resistance_calib_max_vol=2;
    float calibration_lockin_current=10;
    bool motor_thermistor_enabled=false;
    uint8_t controller_mode=ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
    uint8_t controller_input_mode=ODriveInputMode::INPUT_MODE_PASSTHROUGH;
    float controller_vel_limit=10;
    float controller_vel_limit_tolerance=1.2;
    float torque_soft_min=-0.4;
    float torque_soft_max=0.4;
    uint8_t can_protocol=ODriveProtocol::PROTOCOL_SIMPLE;
    uint32_t can_baud_rate=250000;
    uint32_t can_node_id=0;
    uint32_t can_heartbeat_msg_rate_ms=100;
    uint32_t can_encoder_msg_rate_ms=10;
    uint32_t can_iq_msg_rate_ms=10;
    uint32_t can_torques_msg_rate_ms=10;
    uint32_t can_error_msg_rate_ms=10;
    uint32_t can_temperature_msg_rate_msg=10;
    uint32_t can_bus_vol_msg_rate_ms=10;
    bool enable_watchdog=false;
    uint8_t load_encoder=ODriveEncoderId::ENCODER_ID_RS485_ENCODER0;
    uint8_t commutation_encoder=ODriveEncoderId::ENCODER_ID_RS485_ENCODER0;
    uint8_t encoder_group0_mode=ODriveRs485EncoderMode::RS485_ENCODER_MODE_ODRIVE_OA1;
    bool enable_uart_a=false;

    size_t dataSize() const override {
        //return 14*sizeof(bool) + 7*sizeof(uint8_t) + 10*sizeof(uint32_t) + 14*sizeof(float);
        return 4*sizeof(float) + sizeof(uint8_t);
    }

    MsgType getType() const override {
        return MsgType::Config;
    }

    void writeToBuffer(uint8_t* buffer) const override {
        std::memcpy(buffer, &dc_bus_overvoltage_trip_lvl, sizeof(float));
        std::memcpy(buffer + sizeof(float), &dc_bus_undervoltage_trip_lvl, sizeof(float));
        std::memcpy(buffer + 2*sizeof(float), &dc_max_positive_current, sizeof(float));
        std::memcpy(buffer + 3*sizeof(float), &dc_max_negative_current, sizeof(float));
        std::memcpy(buffer + 4*sizeof(float), &motor_type, sizeof(uint8_t));
    }

    void readFromBuffer(const uint8_t* buffer) override {
        std::memcpy(&dc_bus_overvoltage_trip_lvl, buffer, sizeof(float));
        std::memcpy(&dc_bus_undervoltage_trip_lvl, buffer + sizeof(float), sizeof(float)); 
        std::memcpy(&dc_max_positive_current, buffer + 2*sizeof(float), sizeof(float));
        std::memcpy(&dc_max_negative_current, buffer + 3*sizeof(float), sizeof(float));
        std::memcpy(&motor_type, buffer + 4*sizeof(float), sizeof(uint8_t));
    }

    void printValue() override {
        std::cout << "dc_bus_overvoltage: " << dc_bus_overvoltage_trip_lvl << " | undervoltage: " << dc_bus_undervoltage_trip_lvl << std::endl;
    }
};
