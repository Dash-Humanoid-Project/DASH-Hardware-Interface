#include "PeriodicTimer.h"

#ifdef __linux__
#include <sys/timerfd.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>

PeriodicTimer::PeriodicTimer(double period_s)
{
    timer_fd_ = timerfd_create(CLOCK_MONOTONIC, 0);
    if (timer_fd_ < 0) {
        throw std::runtime_error("PeriodicTimer: timerfd_create failed");
    }

    time_t seconds = static_cast<time_t>(period_s);
    long nanoseconds = static_cast<long>(std::fmod(period_s, 1.0) * 1e9);

    itimerspec spec{};
    spec.it_value.tv_sec = seconds;
    spec.it_value.tv_nsec = nanoseconds;
    spec.it_interval.tv_sec = seconds;
    spec.it_interval.tv_nsec = nanoseconds;

    if (timerfd_settime(timer_fd_, 0, &spec, nullptr) < 0) {
        close(timer_fd_);
        throw std::runtime_error("PeriodicTimer: timerfd_settime failed");
    }
}

PeriodicTimer::~PeriodicTimer()
{
    if (timer_fd_ >= 0) close(timer_fd_);
}

void PeriodicTimer::wait()
{
    uint64_t missed = 0;
    ssize_t n = read(timer_fd_, &missed, sizeof(missed));
    // A miss reads as (ticks_elapsed_since_last_read); "1" is the normal,
    // on-schedule case (this wait() consumed exactly one tick).
    last_missed_ = (n == sizeof(missed) && missed > 0) ? (missed - 1) : 0;
}

#else
#error "PeriodicTimer currently only supports Linux (timerfd)"
#endif
