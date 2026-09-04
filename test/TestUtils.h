#pragma once
#include <atomic>
#include <chrono>
#include "HardwareBridge.h"

// Global flag for graceful shutdown, set by the SIGINT handler in
// ClosedLoopControlTest.cpp. Shared across all Mode implementations and the
// pre-dispatcher one-shot branches (--reset/--start/--idle).
extern std::atomic<bool> shutdown_requested;

// Keeps the Teensy's comms-loss watchdog satisfied during a settling wait.
// Uses the no-op Heartbeat message specifically, not a zero-torque command:
// TorqueCommand forces the Teensy into TORQUE_CONTROL mode as a side effect,
// which caused a real bug when this preceded a --position sweep (armed but
// unresponsive, stuck having switched away from POSITION_CONTROL). Heartbeat
// resets the watchdog with zero effect on control mode or motor state.
void sendKeepAliveFor(HardwareBridge& bridge, std::chrono::milliseconds duration);
