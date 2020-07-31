#pragma once
#include <cstdint>

class DMGCPU;
class DMGMemory;

class DMGDisplay
{
public:
    DMGDisplay(DMGCPU &cpu);

    void update(int cycles);

    const uint16_t *getData() {return screenData;}

    uint8_t readReg(uint16_t addr, uint8_t val);
    bool writeReg(uint16_t addr, uint8_t data);

private:
    void drawScanLine(int y);

    DMGCPU &cpu;
    DMGMemory &mem;

    uint8_t y = 0;
    int statMode = 0;

    static const int scanlineCycles = 456;
    static const int screenWidth = 160, screenHeight = 144;

    int remainingScanlineCycles = scanlineCycles;
    uint16_t screenData[screenWidth * screenHeight]; // rgb555

    // GBC
    uint8_t bgPalette[8 * 8], objPalette[8 * 8];
};