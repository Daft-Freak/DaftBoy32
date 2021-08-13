#pragma once
#include <cstdint>

class DMGCPU;
class DMGMemory;

class DMGDisplay
{
public:
    DMGDisplay(DMGCPU &cpu);

    void reset();

    void update(int cycles);

#ifndef PICO_BUILD
    const uint16_t *getData() {return screenData;}
#endif

    uint8_t readReg(uint16_t addr, uint8_t val);
    bool writeReg(uint16_t addr, uint8_t data);

private:
    void drawScanLine(int y);
    void drawBackground(uint16_t *scanLine, uint8_t *bgRaw);
    void drawSprites(uint16_t *scanLine, uint8_t *bgRaw);

    void updateCompare(bool newVal);

    DMGCPU &cpu;
    DMGMemory &mem;

    bool enabled = true;
    uint8_t y = 0;
    uint8_t statMode = 0;
    bool compareMatch = false;
    int windowY = 0;

    static const int scanlineCycles = 456;
    static const int screenWidth = 160, screenHeight = 144;

    int remainingScanlineCycles = scanlineCycles;
#ifndef PICO_BUILD
    uint16_t screenData[screenWidth * screenHeight]; // rgb555
#endif

    // GBC
    uint16_t bgPalette[8 * 4], objPalette[8 * 4];
};