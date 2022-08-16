#pragma once
#include <cstdint>

struct BESSHeader
{
    char id[4];
    uint32_t len;
};

struct BESSCore
{
    uint16_t versionMajor;
    uint16_t versionMinor;

    char model[4];

    uint16_t pc;
    uint16_t af;
    uint16_t bc;
    uint16_t de;
    uint16_t hl;
    uint16_t sp;

    uint8_t ime;
    uint8_t ie;
    uint8_t execState;
    uint8_t reserved;
    uint8_t ioRegs[128];

    uint32_t ramSize;
    uint32_t ramOff;
    uint32_t vramSize;
    uint32_t vramOff;
    uint32_t cartRAMSize;
    uint32_t cartRAMOff;
    uint32_t oamSize;
    uint32_t oamOff;
    uint32_t hramSize;
    uint32_t hramOff;
    uint32_t bgPalSize;
    uint32_t bgPalOff;
    uint32_t objPalSize;
    uint32_t objPalOff;
};
static_assert(sizeof(BESSCore) == 208);

inline const int daftBoyStateVersion = 1;

enum StateFlags
{
    State_EnableInterruptsNextCycle = 1 << 0, // for EI
    State_HaltBug                   = 1 << 1,

    State_TimerReload               = 1 << 2,
    State_TimerReloaded             = 1 << 3,
    State_TimerOldVal = 1 << 4,

    State_GDMATriggered = 1 << 5,

    State_SerialStart = 1 << 6,

    // display
    State_DispFirstFrame = 1 << 7,

    // APU
    State_APUSkipNextFrameSeqUpdate = 1 << 8,
    State_APUCh1SweepCalcWithNeg = 1 << 9
};

struct DaftState
{
    uint8_t version;

    uint8_t reserved[2];

    uint8_t divLow;

    uint32_t flags;

    uint32_t cycleCount;

    uint8_t oamDMACount;
    uint8_t oamDMADelay;

    uint8_t serialBits;

    // display
    uint8_t windowY;
    uint8_t statInterruptActive;
    uint8_t padding;
    uint16_t remainingScalineCycles;
    uint16_t remainingModeCycles;

    // APU
    uint8_t frameSeqClock;
    uint8_t padding2;
    uint32_t enableCycle;

    uint8_t envVolume[3];
    uint8_t envTimer[3];
    uint8_t ch3Sample;
    uint8_t ch3SampleIndex;
    uint32_t freqTimer[4]; // could be smaller

    uint8_t ch1SweepTimer;
    uint8_t ch1DutyStep;

    uint8_t ch2DutyStep;

    uint8_t padding3;
    uint32_t ch3LastAccessCycle;

    uint16_t ch4LFSRBits;

    uint8_t padding4[2];
};

static_assert(sizeof(DaftState) == 64);
