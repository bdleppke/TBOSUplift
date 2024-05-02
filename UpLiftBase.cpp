#include "UpLiftBase.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable

int UpLiftBase::Run()
{
    printf("Starting upLift program...\n");

    /* this is uplift startup, run  init */
    UpLiftInit();

    while (IsRunning()) {
        auto const start = std::chrono::steady_clock::now();

        /* run the  periodic function */
        UpLiftPeriodic();

        /* check if we're enabled */
        if (IsEnabled()) {
            /* enabled */
            if (_lastEnabled != 1) {
                /* just switched, run enabled init */
                printf("UpLift ENABLED\n");
                EnabledInit();
                _lastEnabled = 1;
            }

            /* enable for 100 ms */
            ctre::phoenix::unmanaged::FeedEnable(100);
            /* run enabled periodic */
            EnabledPeriodic();
        } else {
            /* disabled */
            if (_lastEnabled != 0) {
                /* just switched, run disabled init */
                printf("Uplift DISABLED\n");
                DisabledInit();
                _lastEnabled = 0;
            }

            /* run disabled periodic */
            DisabledPeriodic();
        }

        auto const end = std::chrono::steady_clock::now();

        /* yield for the remainder of the loop time */
        units::millisecond_t const dtMs{std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0};
        if (dtMs < _loopTime) {
            SleepFor(_loopTime - dtMs);
        } else {
            /* loop overrun */
            ReportLoopOverrun(dtMs);
            /* yield control of this thread */
            SleepFor(0_ms);
        }
    }

    /* program shutting down */
    printf("Stopping uplift program...\n");

    return 0;
}

void UpLiftBase::ReportLoopOverrun(units::millisecond_t measured)
{
    auto const now = std::chrono::steady_clock::now();
    auto const dtMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastErrorTime).count();

    if (dtMs > kErrorTimeMs) {
        fprintf(stderr, "Warning: %.1fms loop time overrun\n"
                "     loop took %.3fms\n",
                _loopTime.value(), measured.value());
        _lastErrorTime = now;
    }
}
