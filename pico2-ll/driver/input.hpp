#pragma once
#include <cstdint>

enum Button : unsigned int {
    A          = 1 << 0,
    B          = 1 << 1,
    SELECT     = 1 << 2,
    START      = 1 << 3,
    DPAD_RIGHT = 1 << 4,
    DPAD_LEFT  = 1 << 5,
    DPAD_UP    = 1 << 6,
    DPAD_DOWN  = 1 << 7,
    L          = 1 << 9,
    R          = 1 << 8,
};

struct InputDriver {
    void (*init)();
    void (*update)(uint32_t &);
};

extern const InputDriver tca9555_driver;