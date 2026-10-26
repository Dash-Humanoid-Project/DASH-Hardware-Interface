#include "Imu.h"
#include "vn/ezasyncdata.h"
#include "vn/conversions.h"
#include <iostream>
#include <stdexcept>

using namespace vn::sensors;
using namespace vn::math;
using namespace vn::protocol::uart;

Imu::Imu(std::string port, uint32_t baudrate)
    : port_(std::move(port)), baudrate_(baudrate)
{
}

Imu::~Imu()
{
    stop();
}

void Imu::start()
{
    if (ez_) return;

    try {
        ez_ = EzAsyncData::connect(port_, baudrate_);

        // Configure binary output: yaw/pitch/roll (deg) + quaternion + angular
        // rate (rad/s), at 100 Hz (divisor 8 assumes an 800 Hz IMU sample rate —
        // adjust if a different VectorNav model reports a different base rate).
        // Higher rates (e.g. divisor 4 / 200 Hz) overflow 115200 baud for this
        // field set and the sensor rejects the config with InsufficientBaudRate.
        BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            8,
            COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL
                | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_NONE,
            INSGROUP_NONE,
            GPSGROUP_NONE);
        ez_->sensor()->writeBinaryOutput1(bor);

        std::cout << "[Imu] Connected to " << port_ << " @ " << baudrate_ << " baud" << std::endl;
    } catch (const std::exception& e) {
        delete ez_;
        ez_ = nullptr;
        throw std::runtime_error("Imu::start failed on " + port_ + ": " + e.what());
    }
}

void Imu::stop()
{
    if (!ez_) return;
    ez_->disconnect();
    delete ez_;
    ez_ = nullptr;
}

ImuState Imu::getState() const
{
    ImuState state;
    if (!ez_) return state;

    CompositeData cd = ez_->currentData();

    if (cd.hasYawPitchRoll()) {
        vec3f ypr = cd.yawPitchRoll();  // degrees: x=yaw, y=pitch, z=roll
        state.yaw_rad   = deg2rad(ypr.x);
        state.pitch_rad = deg2rad(ypr.y);
        state.roll_rad  = deg2rad(ypr.z);
        state.valid = true;
    }

    if (cd.hasAngularRate()) {
        vec3f gyro = cd.angularRate();  // rad/s, body frame
        state.gyro_x_rad_s = gyro.x;
        state.gyro_y_rad_s = gyro.y;
        state.gyro_z_rad_s = gyro.z;
    }

    return state;
}
