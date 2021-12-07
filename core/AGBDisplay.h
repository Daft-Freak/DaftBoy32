#pragma once
#include <cstdint>

class AGBCPU;
class AGBMemory;

class AGBDisplay
{
public:
    AGBDisplay(AGBCPU &cpu);

    void reset();

    void update();
    int getCyclesToNextUpdate(uint32_t cycleCount) const;

    void setFramebuffer(uint16_t *data);

    uint16_t readReg(uint32_t addr, uint16_t val);
    bool writeReg(uint32_t addr, uint16_t data);

private:
    void drawScanLine(int y);

    AGBCPU &cpu;
    AGBMemory &mem;

    uint32_t lastUpdateCycle = 0;

    uint8_t y = 0;
    bool yInWin0 = false, yInWin1 = false;

    // internal reference points for affine bg
    int32_t refPointX[2]{0}, refPointY[2]{0};

    static const int scanlineDots = 308; // * 4 cpu cycles
    static const int screenWidth = 240, screenHeight = 160;

    unsigned int remainingScanlineDots = scanlineDots;
    unsigned int remainingModeDots = screenWidth;
    uint16_t *screenData; // rgb555
};