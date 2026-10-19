#include "Imu.h"
#include "vectornav/Interface/Sensor.hpp"
#include "Math/vectornav/Conversions.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {
std::string defaultConfigPath()
{
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error) {
        throw std::runtime_error("cannot resolve IMU config from executable path: "
                                 + error.message());
    }
    return (executable.parent_path() / ".." / "config" / "imu.conf")
        .lexically_normal().string();
}

void checkError(VN::Error error, const char* operation)
{
    if (error == VN::Error::None) return;
    throw std::runtime_error(std::string(operation) + ": " + std::string(VN::errorCodeToString(error)));
}

float loadTemperatureThreshold(const std::string& path)
{
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open IMU config: " + path);
    float threshold = 0.0f;
    std::string line;
    while (std::getline(file, line)) {
        if (const auto comment = line.find('#'); comment != std::string::npos) line.erase(comment);
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        const auto equals = line.find('=');
        if (equals != std::string::npos) line.replace(equals, 1, " = ");
        std::istringstream entry(line);
        std::string key, extra;
        char separator;
        if (equals == std::string::npos || threshold != 0.0f
            || !(entry >> key >> separator >> threshold) || separator != '='
            || key != "temperature_reset_delta_deg_c"
            || (entry >> extra) || !std::isfinite(threshold) || threshold <= 0.0f) {
            throw std::runtime_error("invalid IMU temperature threshold in " + path);
        }
    }
    if (file.bad() || threshold == 0.0f) throw std::runtime_error("missing IMU temperature threshold in " + path);
    return threshold;
}

void configureOutput(VN::Sensor& sensor)
{
    // Keep ASCII packets from displacing binary temperature/gyro samples in the queue.
    VN::Registers::System::AsyncOutputType asciiOutput;
    asciiOutput.ador = VN::Registers::System::AsyncOutputType::Ador::OFF;
    asciiOutput.serialPort = VN::Registers::System::AsyncOutputType::SerialPort::Serial1;
    checkError(sensor.writeRegister(&asciiOutput), "disable ASCII output on serial port 1");

    // 100 Hz for an 800 Hz sensor, within the 115200-baud link capacity.
    VN::Registers::System::BinaryOutput1 output;
    output.asyncMode.emplace();
    output.asyncMode->serial1 = true;
    output.rateDivisor = 8;
    output.common.timeStartup = true;
    output.common.ypr = true;
    output.common.quaternion = true;
    output.common.angularRate = true;
    output.imu.temperature = true;
    checkError(sensor.writeRegister(&output), "write binary output 1");
    sensor.getMostRecentMeasurement(false); // Discard pre-configuration measurements.
}
} // namespace

struct Imu::Impl {
    VN::Sensor sensor;
    ImuState state;
    float temperatureThreshold = 0.0f;
    float lowest_temperature_deg_c = std::numeric_limits<float>::infinity();
    bool failed = false;

    ~Impl()
    {
        sensor.disconnect();
    }
};

Imu::Imu(std::string port, uint32_t baudrate, std::string configPath)
    : port_(std::move(port)), baudrate_(baudrate), configPath_(std::move(configPath))
{
}

Imu::~Imu()
{
    stop();
}

void Imu::start()
{
    if (impl_) return;

    auto impl = std::make_unique<Impl>();
    try {
        impl->temperatureThreshold = loadTemperatureThreshold(
            configPath_.empty() ? defaultConfigPath() : configPath_);
        checkError(impl->sensor.connect(port_, static_cast<VN::Sensor::BaudRate>(baudrate_)), "connect");
        if (!impl->sensor.verifySensorConnectivity()) {
            throw std::runtime_error("verifySensorConnectivity failed");
        }
        checkError(impl->sensor.restoreFactorySettings(), "restoreFactorySettings");

        configureOutput(impl->sensor);
        std::cout << "[Imu] Connected to " << port_ << " @ "
                  << static_cast<uint32_t>(impl->sensor.connectedBaudRate().value())
                  << " baud" << std::endl;
        impl_ = std::move(impl);
    } catch (const std::exception& e) {
        throw std::runtime_error("Imu::start failed on " + port_ + ": " + e.what());
    }
}

void Imu::stop()
{
    impl_.reset();
}

ImuState Imu::getState() const
{
    if (!impl_ || impl_->failed) return {};
    auto& state = impl_->state;
    auto data = impl_->sensor.getMostRecentMeasurement(false);
    if (!data) return state;

    if (data->attitude.ypr) {
        const auto ypr = VN::Math::deg2rad(*data->attitude.ypr);
        state.yaw_rad = ypr.yaw;
        state.pitch_rad = ypr.pitch;
        state.roll_rad = ypr.roll;
        state.valid = true;
    }
    if (data->imu.angularRate) {
        state.gyro_x_rad_s = (*data->imu.angularRate)[0];
        state.gyro_y_rad_s = (*data->imu.angularRate)[1];
        state.gyro_z_rad_s = (*data->imu.angularRate)[2];
    }
    if (data->imu.temperature && std::isfinite(*data->imu.temperature)) {
        const float temperature_deg_c = *data->imu.temperature;
        auto& lowest_temperature_deg_c = impl_->lowest_temperature_deg_c;
        lowest_temperature_deg_c = std::min(lowest_temperature_deg_c, temperature_deg_c);
        if (temperature_deg_c - lowest_temperature_deg_c > impl_->temperatureThreshold) {
            std::cout << "[Imu] Temperature rose from " << lowest_temperature_deg_c
                      << " to " << temperature_deg_c << " C; soft rebooting" << std::endl;
            state = {};
            data = {}; // Release the SDK queue entry before reset/reconfiguration.
            try {
                checkError(impl_->sensor.reset(), "temperature-triggered soft reboot");
                configureOutput(impl_->sensor);
                lowest_temperature_deg_c = std::numeric_limits<float>::infinity();
            } catch (const std::exception& error) {
                impl_->failed = true;
                std::cerr << "[Imu] Temperature monitor stopped: " << error.what()
                          << "; call stop()/start() to reconnect" << std::endl;
            }
        }
    }
    return state;
}
