#pragma once
#include <cstdint>
#include <functional>

struct BESSCore;
struct DaftState;
class DMGCPU;
class DMGMemory;

class DMGDisplay
{
public:
    using VBlankCallback = void(*)();

    DMGDisplay(DMGCPU &cpu);

    void reset();

    void loadSaveState(BESSCore &bess, DaftState &state, std::function<uint32_t(uint32_t, uint32_t, uint8_t *)> readFunc);
    void saveState(DaftState &state);
    void savePaletteState(BESSCore &bess, std::function<uint32_t(uint32_t, uint32_t, const uint8_t *)> writeFunc, uint32_t &offset);

    void update();
    void updateForInterrupts();
    int getCyclesToNextUpdate() const;

    void setFramebuffer(uint16_t *data);

    uint8_t readReg(uint16_t addr, uint8_t val);
    bool writeReg(uint16_t addr, uint8_t data);

    void setVBlankCallback(VBlankCallback callback){vBlankCallback = callback;}

private:
    void drawScanLine(int y);
    void drawBackground(uint16_t *scanLine, uint8_t *bgRaw);
    void drawSprites(uint16_t *scanLine, uint8_t *bgRaw);

    void updateCompare(bool newVal);

    DMGCPU &cpu;
    DMGMemory &mem;

    uint32_t lastUpdateCycle = 0;

    bool enabled = true;
    uint8_t y = 0;
    uint8_t statMode = 0;
    bool compareMatch = false;
    int windowY = 0;
    bool firstFrame = false; // first frame after enabling display

    bool interruptsEnabled = false;
    uint8_t statInterruptActive = 0;

    static const int scanlineCycles = 456;
    static const int screenWidth = 160, screenHeight = 144;

    int remainingScanlineCycles = scanlineCycles;
    uint32_t remainingModeCycles = 0;
    uint16_t *screenData = nullptr; // rgb555

    // GBC
    uint16_t bgPalette[8 * 4], objPalette[8 * 4];

#if defined(DISPLAY_RGB565) || defined(DISPLAY_RB_SWAP)
    uint16_t bgPaletteRaw[8 * 4], objPaletteRaw[8 * 4];
    bool bgPaletteDirty = false, objPaletteDirty = false;
#endif

    VBlankCallback vBlankCallback = nullptr;
};
