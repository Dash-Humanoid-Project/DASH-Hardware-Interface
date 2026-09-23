#pragma once
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
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

// Shared by startup_zero_move and any tool that needs a known-good zero
// starting point (e.g. recording joint limits relative to it). Refuses to
// move (throws) if any joint reads further than max_offset_rad from zero —
// see StartupZeroMove.cpp's file header for why that threshold is sized the
// way it is. Arms closed-loop control itself; leaves it armed on return
// (caller/bridge.stop() is responsible for idling afterward).
void rampJointsToZero(HardwareBridge& bridge, Leg& leg, const std::vector<std::string>& joints,
                       float max_offset_rad = 0.3f,
                       float ramp_duration_s = 1.5f,
                       float hold_duration_s = 0.3f);
