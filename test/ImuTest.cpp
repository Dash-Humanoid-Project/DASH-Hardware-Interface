#include "Imu.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

std::atomic<bool> shutdown_requested(false);

void signalHandler(int) { shutdown_requested = true; }

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);

    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    std::cout << "Connecting to IMU on " << port << " (Ctrl+C to stop)..." << std::endl;

    Imu imu(port);
    imu.start();

    while (!shutdown_requested) {
        ImuState s = imu.getState();
        if (s.valid) {
            std::cout << "roll=" << s.roll_rad << " pitch=" << s.pitch_rad
                      << " yaw=" << s.yaw_rad
                      << " | gyro(rad/s)=[" << s.gyro_x_rad_s << ", "
                      << s.gyro_y_rad_s << ", " << s.gyro_z_rad_s << "]"
                      << std::endl;
        } else {
            std::cout << "No data yet..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    imu.stop();
    std::cout << "Disconnected." << std::endl;
    return 0;
}
