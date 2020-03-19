#pragma once
#include <cstdint>

class DMGCPU;

class DMGAPU
{
public:
    DMGAPU(DMGCPU &cpu);

    void update(int cycles);

    int16_t getSample();
    int getNumSamples() const;

    uint8_t readReg(uint16_t addr, uint8_t val);
    void writeReg(uint16_t addr, uint8_t data);

private:
    DMGCPU &cpu;

    uint8_t frameSeqClock = 0;

    uint8_t channelEnabled = 0;

    // channel 1
    int ch1FreqTimer = 0;
    int ch1FreqTimerPeriod = 1;
    int ch1DutyStep = 0;
    uint8_t ch1DutyPattern = 0;

    // channel 2
    int ch2FreqTimer = 0;
    int ch2FreqTimerPeriod = 1;
    int ch2DutyStep = 0;
    uint8_t ch2DutyPattern = 0;

    // output
    int sampleClock = 0;
    static const int bufferSize = 1024;
    int16_t sampleData[bufferSize] = {0};
    volatile int readOff = 0, writeOff = 128;
};