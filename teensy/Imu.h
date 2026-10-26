#pragma once
#include <string>
#include <cstdint>

namespace vn { namespace sensors { class EzAsyncData; } }

// Latest orientation/angular-rate sample from the body-mounted IMU.
// roll/pitch/yaw and gyro are all in the IMU's own body frame — no
// mounting-offset correction is applied here (TODO once the mounting
// transform relative to the torso/URDF frame is measured).
struct ImuState {
    bool  valid = false;   // false until at least one packet has been received
    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad   = 0.0f;
    float gyro_x_rad_s = 0.0f;
    float gyro_y_rad_s = 0.0f;
    float gyro_z_rad_s = 0.0f;
};

// Thin wrapper around the VectorNav EzAsyncData API (USB-serial connection).
// EzAsyncData runs its own background read thread and caches the latest
// packet internally (mutex-guarded), so getState() here is non-blocking —
// same access pattern as UPXtreme::getPosEstimate().
class Imu {
public:
    explicit Imu(std::string port, uint32_t baudrate = 115200);
    ~Imu();

    void start();  // connects and configures binary output (YPR + quaternion + angular rate)
    void stop();   // disconnects

    ImuState getState() const;

private:
    std::string port_;
    uint32_t baudrate_;
    vn::sensors::EzAsyncData* ez_ = nullptr;
};
