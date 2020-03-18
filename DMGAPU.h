#pragma once
#include <cstdint>

class DMGCPU;

class DMGAPU
{
public:
    void update(int cycles, DMGCPU &cpu);

    int16_t getSample();
    int getNumSamples() const;

    uint8_t readReg(uint16_t addr, uint8_t val);

private:
    uint8_t frameSeqClock = 0;

    uint8_t channelEnabled = 0;

    // channel 1
    int ch1FreqTimer = 0;
    int ch1DutyStep = 0;

    // output
    int sampleClock = 0;
    static const int bufferSize = 1024;
    int16_t sampleData[bufferSize] = {0};
    volatile int readOff = 0, writeOff = 128;
};