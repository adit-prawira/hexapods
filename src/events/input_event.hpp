#pragma once
#include <cstdint>
#include <string>

struct InputEvent {
    enum Type : char {
        Key = 'k',     // teleop/keyboard commands
        Reset = 'r',   // world reinitialized; starts new epoch
        Perturb = 'p', // mouse-grab forces
        Slider = 's',  // live parameter changes,
        Toggle = 't',  // toogle to enable/disable physics
    };

    // the step at which an event took effect
    uint64_t stepIndex;

    Type type;
    std::string payload;
};
