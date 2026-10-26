#pragma once
#include <cstdint>

// Fixed-rate loop scheduler anchored to an absolute clock grid, not to
// "now + period" the way std::this_thread::sleep_for(period) is. Ported
// from Cheetah-Software's PeriodicTask::loopFunction() (timerfd against
// CLOCK_MONOTONIC): wait() blocks until the next tick on the schedule set
// at construction, so a slow iteration causes a reported miss but does not
// permanently shift the phase of future ticks the way a plain sleep does.
//
// Usage: construct once, call wait() once per loop iteration at the point
// where a rate-limiting sleep_for() used to sit.
class PeriodicTimer {
public:
    explicit PeriodicTimer(double period_s);
    ~PeriodicTimer();

    PeriodicTimer(const PeriodicTimer&) = delete;
    PeriodicTimer& operator=(const PeriodicTimer&) = delete;

    // Blocks until the next scheduled tick.
    void wait();

    // Number of ticks skipped between the previous wait() and this one.
    // 0 means the loop kept up with the schedule; >0 means an iteration
    // ran long enough to miss one or more ticks.
    uint64_t lastMissedTicks() const { return last_missed_; }

private:
    int timer_fd_ = -1;
    uint64_t last_missed_ = 0;
};
