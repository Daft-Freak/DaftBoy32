#include <cstdio>

#include "DMGAPU.h"

#include "DMGCPU.h"
#include "DMGRegs.h"

void DMGAPU::update(int cycles, DMGCPU &cpu)
{
    // this gets called before the timer is incremented
    auto oldDiv = cpu.getInternalTimer();

    // regs
    auto ch1LenDuty = cpu.readIORegRaw(IO_NR11);
    auto ch1EnvVol = cpu.readIORegRaw(IO_NR12);
    auto ch1FreqHi = cpu.readIORegRaw(IO_NR14);

    auto ch2LenDuty = cpu.readIORegRaw(IO_NR21);
    auto ch2EnvVol = cpu.readIORegRaw(IO_NR22);
    auto ch2FreqHi = cpu.readIORegRaw(IO_NR24);

    auto ch3Len = cpu.readIORegRaw(IO_NR31);
    auto ch3FreqHi = cpu.readIORegRaw(IO_NR34);

    auto ch4Len = cpu.readIORegRaw(IO_NR41);
    auto ch4EnvVol = cpu.readIORegRaw(IO_NR42);
    auto ch4FreqHi = cpu.readIORegRaw(IO_NR44);

    auto sndOnOff = cpu.readIORegRaw(IO_NR52);

    if(!(sndOnOff & NR52_Enable))
    {
        // disabled
        for(int i = IO_NR10; i < IO_NR52; i++)
            cpu.writeMem(0xFF00 | i, 0);
            
        channelEnabled = 0;
        frameSeqClock = 0;
    }
    else
    {
        // update frame sequencer clock
        for(int c = cycles; c > 0; c -= 4)
        {
            auto wasSet = oldDiv & (1 << 12);
            oldDiv += 4;

            if(wasSet && !(oldDiv & (1 << 12)))
            {
                frameSeqClock = (frameSeqClock + 1) % 8;

                if((frameSeqClock & 1) == 0)
                {
                    // length
                    if(ch1FreqHi & NRx4_Counter)
                    {
                        int newLen = (64 - (ch1LenDuty & 0x3F)) - 1;
                        if(newLen == 0)
                            channelEnabled &= ~(1 << 0); // done, disable

                        // write back
                        ch1LenDuty = (ch1LenDuty & 0xC0) | ((64 - newLen) & 0x3F);
                    }

                    if(ch2FreqHi & NRx4_Counter)
                    {
                        int newLen = (64 - (ch2LenDuty & 0x3F)) - 1;
                        if(newLen == 0)
                            channelEnabled &= ~(1 << 1); // done, disable

                        // write back
                        ch2LenDuty = (ch2LenDuty & 0xC0) | ((64 - newLen) & 0x3F);
                    }

                    if(ch3FreqHi & NRx4_Counter)
                    {
                        int newLen = (256 - ch3Len) - 1;
                        if(newLen == 0)
                            channelEnabled &= ~(1 << 2); // done, disable

                        // write back
                        ch3Len = 256 - newLen;
                    }

                    if(ch4FreqHi & NRx4_Counter)
                    {
                        int newLen = (64 - (ch4Len & 0x3F)) - 1;
                        if(newLen == 0)
                            channelEnabled &= ~(1 << 3); // done, disable

                        // write back
                        ch4Len = ((64 - newLen) & 0x3F);
                    }
                } 

                if(frameSeqClock == 7)
                {} // env

                if((frameSeqClock & 0x3) == 2)
                {} // sweep
            }
        }

        // channel 1
        if(ch1FreqHi & NRx4_Trigger)
        {
            channelEnabled |= (1 << 0);
            ch1FreqHi &= ~NRx4_Trigger;
        }

        // disable if DAC off
        if((ch1EnvVol & 0xF8) == 0)
            channelEnabled &= ~(1 << 0);

        ch1FreqTimer += cycles;
        auto freq = cpu.readIORegRaw(IO_NR13) | ((cpu.readIORegRaw(IO_NR14) & 0x7) << 8);
        auto timerPeriod = (2048 - freq) * 4;

        while(ch1FreqTimer > timerPeriod)
        {
            ch1FreqTimer -= timerPeriod;
            ch1DutyStep++;
            ch1DutyStep &= 7;
        }

        // channel 2
        if(ch2FreqHi & NRx4_Trigger)
        {
            channelEnabled |= (1 << 1);
            ch2FreqHi &= ~NRx4_Trigger;
        }

        if((ch2EnvVol & 0xF8) == 0)
            channelEnabled &= ~(1 << 1);

        // channel 3
        if(ch3FreqHi & NRx4_Trigger)
        {
            channelEnabled |= (1 << 2);
            ch3FreqHi &= ~NRx4_Trigger;
        }

        if(!(cpu.readIORegRaw(IO_NR30) & 0x80))
            channelEnabled &= ~(1 << 2);

        // channel 4
        if(ch4FreqHi & NRx4_Trigger)
        {
            channelEnabled |= (1 << 3);
            ch4FreqHi &= ~NRx4_Trigger;
        }

        if((ch4EnvVol & 0xF8) == 0)
            channelEnabled &= ~(1 << 3);
    }

    // output clock - attempt to get close to 22050Hz
    const int clocksPerSample = (4194304u * 1000) / 22050;
    sampleClock += cycles * 1000;

    if(sampleClock >= clocksPerSample)
    {
        sampleClock -= clocksPerSample;

        // wait for the audio thread/interrupt to catch up
        while(writeOff == readOff - 1) {}

        auto outputSelect = cpu.readIORegRaw(IO_NR51);

        auto vol = ch1EnvVol >> 4;

        auto ch1Val = (channelEnabled & 1) && ch1DutyStep > 3 ? vol : -vol;

        int32_t sample = 0;
        
        if(outputSelect & 1)
            sample += ch1Val * 0x888;

        sampleData[writeOff] = sample;
        writeOff = (writeOff + 1) % bufferSize;
    }

    // update
    if(!(sndOnOff & NR52_Enable))
        return;

    cpu.writeMem(0xFF00 | IO_NR11, ch1LenDuty);
    cpu.writeMem(0xFF00 | IO_NR14, ch1FreqHi);
    cpu.writeMem(0xFF00 | IO_NR21, ch2LenDuty);
    cpu.writeMem(0xFF00 | IO_NR24, ch2FreqHi);
    cpu.writeMem(0xFF00 | IO_NR31, ch3Len);
    cpu.writeMem(0xFF00 | IO_NR34, ch3FreqHi);
    cpu.writeMem(0xFF00 | IO_NR41, ch4Len);
    cpu.writeMem(0xFF00 | IO_NR44, ch4FreqHi);
    cpu.writeMem(0xFF00 | IO_NR52, (sndOnOff & 0xF0) | channelEnabled);
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

uint8_t DMGAPU::readReg(uint16_t addr, uint8_t val)
{
    if(addr < 0xFF00)
        return val;

    // unsued bits/lengths/freqs read as 1
    switch(addr & 0xFF)
    {
        case IO_NR10:
            return val | 0x80;

        case IO_NR11:
        case IO_NR21:
            return val | 0x3F;

        case IO_NR13:
        case 0x15: // NR20, if that was a thing
        case IO_NR23:
        case IO_NR31:
        case IO_NR33:
        case 0x1F: // NR40
        case IO_NR41:
        case 0x27: // unused \/
        case 0x28:
        case 0x29:
        case 0x2A:
        case 0x2B:
        case 0x2C:
        case 0x2D:
        case 0x2E:
        case 0x2F:
            return 0xFF;

        case IO_NR14:
        case IO_NR24:
        case IO_NR34:
        case IO_NR44:
            return val | 0xBF;

        case IO_NR30:
            return val | 0x7F;

        case IO_NR32:
            return val | 0x9F;

        case IO_NR52:
            return val | 0x70;
    }


    return val;
}