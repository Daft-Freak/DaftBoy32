#pragma once
#include <cstdint>

struct DaftState;
class DMGCPU;

class DMGAPU
{
public:
    DMGAPU(DMGCPU &cpu);

    void reset();

    void loadSaveState(DaftState &state);
    void saveState(DaftState &state);

    void update();

    int16_t getSample();
    int getNumSamples() const;

    uint8_t readReg(uint16_t addr, uint8_t val);
    bool writeReg(uint16_t addr, uint8_t data);

private:
    void updateFrameSequencer();
    void updateFreq(int cyclesPassed);

    void sampleOutput();

    DMGCPU &cpu;

    bool enabled = true;
    uint8_t channelEnabled = 0;

    uint8_t frameSeqClock = 0;
    bool skipNextFrameSeqUpdate = false;

    // deferred updates
    uint32_t lastUpdateCycle = 0;
    uint16_t lastDivValue = 0;

    uint32_t enableCycle = 0; // used to align timers (only really need & 4)

    // channel 1
    bool ch1SweepEnable = false;
    bool ch1SweepCalcWithNeg = false; // for some wierdness
    uint8_t ch1SweepTimer = 0;
    uint16_t ch1SweepFreq = 0;
    uint8_t ch1Len = 0;
    uint8_t ch1EnvVolume, ch1EnvTimer;
    bool ch1Val = false;
    int ch1FreqTimer = 0;
    uint16_t ch1FreqTimerPeriod = 1;
    uint8_t ch1DutyStep = 0;
    uint8_t ch1DutyPattern = 0;

    // channel 2
    uint8_t ch2Len = 0;
    uint8_t ch2EnvVolume, ch2EnvTimer;
    bool ch2Val = false;
    int ch2FreqTimer = 0;
    uint16_t ch2FreqTimerPeriod = 1;
    uint8_t ch2DutyStep = 0;
    uint8_t ch2DutyPattern = 0;

    // channel 3
    uint16_t ch3Len = 0; // this one can be 256
    int ch3FreqTimer = 0;
    uint16_t ch3FreqTimerPeriod = 1;
    uint8_t ch3Sample = 0;
    uint8_t ch3SampleIndex = 0;
    uint32_t ch3LastAccessCycle = 0;

    // channel 4
    uint8_t ch4Len = 0;
    uint8_t ch4EnvVolume, ch4EnvTimer;
    int ch4FreqTimer = 0;
    int ch4FreqTimerPeriod = 8;
    uint16_t ch4LFSRBits = 0; // really 15 bit
    bool ch4Narrow = false;
    bool ch4Val = false;

    // output
    int sampleClock = 0;
    static const int bufferSize = 1024;
    volatile uint16_t readOff = 0, writeOff = 64;
    int16_t sampleData[bufferSize] = {0};
    int32_t filterVal[2]{};
};