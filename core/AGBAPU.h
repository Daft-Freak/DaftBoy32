#pragma once
#include <cstdint>

class AGBCPU;

class AGBAPU
{
public:
    AGBAPU(AGBCPU &cpu);

    void reset();

    void update();

    void timerOverflow(int timer, uint32_t cycle);

    int16_t getSample();
    int getNumSamples() const;

    uint16_t readReg(uint32_t addr, uint16_t val);
    bool writeReg(uint32_t addr, uint16_t data);

private:
    void updateFrameSequencer();
    void updateFreq(int cyclesPassed);

    void sampleOutput();

    AGBCPU &cpu;

    bool enabled = true;
    uint8_t channelEnabled = 0;

    uint8_t frameSeqClock = 0;
    // deferred updates
    uint32_t lastUpdateCycle = 0;

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
    uint8_t ch3BankIndex = 0;
    uint64_t ch3WaveBuf[4]{0}; // 2x 128-bit buffers

    // channel 4
    uint8_t ch4Len = 0;
    uint8_t ch4EnvVolume, ch4EnvTimer;
    int ch4FreqTimer = 0;
    int ch4FreqTimerPeriod = 8;
    uint16_t ch4LFSRBits = 0; // really 15 bit
    bool ch4Narrow = false;
    bool ch4Val = false;

    // "DMA" channels
    uint8_t dmaAFIFO[32], dmaBFIFO[32];
    uint8_t fifoARead = 0, fifoAWrite = 0, fifoBRead = 0, fifoBWrite = 0;
    uint8_t fifoAFilled = 0, fifoBFilled = 0;

    int8_t dmaAVal = 0, dmaBVal = 0;

    // output
    int sampleClock = 0;
    static const int bufferSize = 1024;
    volatile uint16_t readOff = 0, writeOff = 64;
    int16_t sampleData[bufferSize] = {0};
};