#pragma once
#include <cstdint>

class DMGCPU;
class DMGMemory;

class DMGDisplay
{
public:
    DMGDisplay(DMGCPU &cpu);

    void update(int cycles);

    const uint8_t *getData() {return screenData;}

private:
    DMGCPU &cpu;
    DMGMemory &mem;

    static const int scanlineCycles = 456;
    int remainingScanlineCycles = scanlineCycles;
    uint8_t screenData[160 * 144]; // greyscale
};