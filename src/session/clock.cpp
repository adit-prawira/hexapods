#include "session/clock.hpp"
#include <cstdint>

uint64_t Clock::advance(double seconds) {
    if (is_paused_) {
        unsimulatedTime_ = 0.0;
        return 0;
    }

    bool shouldClamp = seconds > 0.1;
    if (shouldClamp)
        seconds = 0.1;

    unsimulatedTime_ += seconds;
    uint64_t stepsDue = 0;
    while (unsimulatedTime_ >= timestep_) {
        unsimulatedTime_ -= timestep_;
        ++stepsDue;
    }

    return stepsDue;
}
