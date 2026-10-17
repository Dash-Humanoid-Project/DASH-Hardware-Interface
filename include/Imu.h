#pragma once
#include <string>
#include <cstdint>
#include <memory>

// Latest orientation/angular-rate sample from the body-mounted IMU.
// roll/pitch/yaw and gyro are all in the IMU's own body frame — no
// mounting-offset correction is applied here (TODO once the mounting
// transform relative to the torso/URDF frame is measured).
struct ImuState {
    bool  valid = false;   // false until yaw/pitch/roll has been received
    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad   = 0.0f;
    float gyro_x_rad_s = 0.0f;
    float gyro_y_rad_s = 0.0f;
    float gyro_z_rad_s = 0.0f;
};

// USB-serial wrapper around VectorNav SDK 1.2.0. The SDK reads in the
// background; getState() polls its queue and checks for temperature-triggered
// soft reboots. Call all methods from one application thread.
class Imu {
public:
    explicit Imu(std::string port, uint32_t baudrate = 115200,
                 std::string configPath = "imu.conf");
    ~Imu();

    void start();  // connects, restores factory settings, configures 100 Hz output
    void stop();   // disconnects

    ImuState getState() const; // cached if no new sample; blocks during soft reboot

private:
    std::string port_;
    uint32_t baudrate_;
    std::string configPath_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
