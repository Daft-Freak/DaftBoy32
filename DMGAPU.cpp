#include "DMGAPU.h"

#include "DMGCPU.h"
#include "DMGRegs.h"

void DMGAPU::update(int cycles, DMGCPU &cpu)
{
    // this gets called before the timer is incremented
    auto oldDiv = cpu.getInternalTimer();

    // regs
    auto ch1LenDuty = cpu.readMem(0xFF00 | IO_NR11);
    auto ch1EnvVol = cpu.readMem(0xFF00 | IO_NR12);

    bool ch1En = false;

    if(!(cpu.readMem(0xFF00 | IO_NR52) & NR52_Enable))
    {
        // disabled
        frameSeqClock = 0;
    }
    else
    {
        // update frame sequencer clock
        for(int c = cycles; c > 0; c -= 4)
        {
            auto wasSet = oldDiv & (1 << 13);
            oldDiv += 4;

            if(wasSet && !(oldDiv & (1 << 13)))
            {
                frameSeqClock = (frameSeqClock + 1) % 8;

                if((frameSeqClock & 1) == 0)
                {} // length

                if(frameSeqClock == 7)
                {} // env

                if((frameSeqClock & 0x3) == 2)
                {} // sweep
            }
        }

        // channel 1
        ch1FreqTimer += cycles;
        auto freq = cpu.readMem(0xFF00 | IO_NR13) | ((cpu.readMem(0xFF00 | IO_NR14) & 0x7) << 8);
        auto timerPeriod = (2048 - freq) * 4;

        ch1En = (ch1LenDuty & 0x1F) > 0;

        while(ch1FreqTimer > timerPeriod)
        {
            ch1FreqTimer -= timerPeriod;
            ch1DutyStep++;
            ch1DutyStep &= 7;
        }
    }

    // output clock - attempt to get close to 22050Hz
    const int clocksPerSample = (4194304u * 1000) / 22050;
    sampleClock += cycles * 1000;

    if(sampleClock >= clocksPerSample)
    {
        sampleClock -= clocksPerSample;

        // wait for the audio thread/interrupt to catch up
        while(writeOff == readOff - 1) {}

        auto outputSelect = cpu.readMem(0xFF00 | IO_NR51);

        auto vol = ch1EnvVol >> 4;

        auto ch1Val = ch1En && ch1DutyStep > 3 ? vol : -vol;

        int32_t sample = 0;
        
        if(outputSelect & 1)
            sample += ch1Val * 0x888;

        sampleData[writeOff] = sample;
        writeOff = (writeOff + 1) % bufferSize;
    }
}

int16_t DMGAPU::getSample()
{
    auto ret = sampleData[readOff];

    readOff = (readOff + 1) % bufferSize;

    return ret;
}

int DMGAPU::getNumSamples() const
{
    int avail = writeOff - readOff;
    if(avail < 0)
        avail += bufferSize;

    return avail;
}
