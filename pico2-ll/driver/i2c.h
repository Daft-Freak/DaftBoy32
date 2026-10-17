#pragma once

#include <cstdint>

struct I2CDriver
{
    void (*init)();
    void (*update)();
};
