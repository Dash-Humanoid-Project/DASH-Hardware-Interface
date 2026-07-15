#include "TestUtils.h"
#include <thread>

std::atomic<bool> shutdown_requested(false);

void sendKeepAliveFor(HardwareBridge& bridge, std::chrono::milliseconds duration)
{
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < duration && !shutdown_requested) {
        bridge.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
