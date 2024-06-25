#pragma once

#include "units/time.h"
#include <chrono>
#include <thread>
#include <stdint.h>

/**
 * Manages the UpLift System
 */
class UpLiftBase {
public:
    virtual int UpLiftInit() = 0;
    virtual void UpLiftPeriodic() = 0;
   // virtual void SetPositionFrom0To100(double cab, double tail) = 0;

    virtual bool IsEnabled() = 0;
    virtual void EnabledInit() = 0;
    virtual int EnabledPeriodic() = 0;

    virtual void DisabledInit() = 0;
    virtual void DisabledPeriodic() = 0;


   // virtual bool IsRunning() { return true; }

private:
    units::millisecond_t _loopTime = 20_ms;
    int _lastEnabled = -1;
    bool _isRunning = true;

public:
    /**
     * Sleeps for the specified amount of time.
     */
    static inline void SleepFor(units::microsecond_t us)
    {
        std::this_thread::sleep_for(std::chrono::microseconds{(uint64_t)us.value()});
    }

    /**
     * Sets the loop time for program periodic calls.
     */
    void SetLoopTime(units::millisecond_t loopTime = 20_ms)
    {
        _loopTime = loopTime;
    }

    bool IsRunning() { return _isRunning;}


    /**
     * Runs the  program.
     */
    int Run();

private:
    static constexpr auto kErrorTimeMs = 500;
    std::chrono::time_point<std::chrono::steady_clock> _lastErrorTime = std::chrono::steady_clock::now();

    /** Reports a loop overrun with debouncing. */
    void ReportLoopOverrun(units::millisecond_t measured);
};
