#pragma once
#include <cstdint>

class AGBCPU;
class AGBMemory;

class AGBDisplay
{
public:
    AGBDisplay(AGBCPU &cpu);

    void update(int cycles);

    void setFramebuffer(uint16_t *data);

    uint16_t readReg(uint32_t addr, uint16_t val);
    bool writeReg(uint32_t addr, uint16_t data);

private:
    void drawScanLine(int y);

    AGBCPU &cpu;
    AGBMemory &mem;

    uint8_t y = 0;

    static const int scanlineDots = 308; // * 4 cpu cycles
    static const int screenWidth = 240, screenHeight = 160;

    int remainingScanlineDots = scanlineDots;
    uint16_t *screenData; // rgb555
};