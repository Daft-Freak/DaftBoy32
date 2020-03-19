#include <cstdio>

#include "DMGAPU.h"

#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

DMGAPU::DMGAPU(DMGCPU &cpu) : cpu(cpu)
{}

void DMGAPU::update(int cycles)
{
    auto &mem = cpu.getMem();

    // regs
    const auto ch1EnvVol = mem.readIOReg(IO_NR12);
    const auto ch1FreqHi = mem.readIOReg(IO_NR14);

    const auto ch2EnvVol = mem.readIOReg(IO_NR22);
    const auto ch2FreqHi = mem.readIOReg(IO_NR24);

    const auto ch3FreqHi = mem.readIOReg(IO_NR34);

    const auto ch4EnvVol = mem.readIOReg(IO_NR42);
    const auto ch4FreqHi = mem.readIOReg(IO_NR44);

    const auto sndOnOff = mem.readIOReg(IO_NR52);

    if(!(sndOnOff & NR52_Enable))
    {
        // disabled
        for(int i = IO_NR10; i < IO_NR52; i++)
            mem.writeIOReg(i, 0);

        channelEnabled = 0;
        frameSeqClock = 0;
    }
    else
    {
        // this gets called before the timer is incremented
        auto oldDiv = cpu.getInternalTimer();

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
                    auto ch1LenDuty = mem.readIOReg(IO_NR11);
                    auto ch2LenDuty = mem.readIOReg(IO_NR21);
                    auto ch3Len = mem.readIOReg(IO_NR31);
                    auto ch4Len = mem.readIOReg(IO_NR41);

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

                    mem.writeIOReg(IO_NR11, ch1LenDuty);
                    mem.writeIOReg(IO_NR21, ch2LenDuty);
                    mem.writeIOReg(IO_NR31, ch3Len);
                    mem.writeIOReg(IO_NR41, ch4Len);
                } 

                if(frameSeqClock == 7)
                {} // env

                if((frameSeqClock & 0x3) == 2)
                {} // sweep
            }
        }

        // channel 1
        ch1FreqTimer += cycles;

        while(ch1FreqTimer > ch1FreqTimerPeriod)
        {
            ch1FreqTimer -= ch1FreqTimerPeriod;
            ch1DutyStep++;
            ch1DutyStep &= 7;
        }

        // channel 2
        ch2FreqTimer += cycles;
    
        while(ch2FreqTimer > ch2FreqTimerPeriod)
        {
            ch2DutyStep += ch2FreqTimer / ch2FreqTimerPeriod;
            ch2FreqTimer -= ch2FreqTimerPeriod;
            ch2DutyStep++;
            ch2DutyStep &= 7;
        }

        // channel 3

        // channel 4

    }

    // output clock - attempt to get close to 22050Hz
    const int clocksPerSample = (4194304u * 1000) / 22050;
    sampleClock += cycles * 1000;

    if(sampleClock >= clocksPerSample)
    {
        sampleClock -= clocksPerSample;

        // wait for the audio thread/interrupt to catch up
        while(writeOff == readOff - 1) {}

        auto outputSelect = mem.readIOReg(IO_NR51);

        auto vol = ch1EnvVol >> 4;
        auto ch1Val = (channelEnabled & 1) && (ch1DutyPattern & (1 << ch1DutyStep)) ? vol : -vol;

        vol = ch2EnvVol >> 4;
        auto ch2Val = (channelEnabled & 2) && (ch2DutyPattern & (1 << ch2DutyStep)) ? vol : -vol;

        int32_t sample = 0;
        
        if(outputSelect & 1)
            sample += ch1Val;

        if(outputSelect & 2)
            sample += ch2Val;

        // TODO SO2?

        sampleData[writeOff] = sample * 0x444;
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
            return (val & 0xF0) | 0x70 | channelEnabled; // enabled channels is read only
    }


    return val;
}

void DMGAPU::writeReg(uint16_t addr, uint8_t data)
{
    const uint8_t dutyPatterns[]{0b00000001, 0b10000001, 0b10000111, 0b01111110};

    if(addr < 0xFF00)
        return;

    auto &mem = cpu.getMem();

    switch(addr & 0xFF)
    {
        case IO_NR11: // length/duty
            ch1DutyPattern = dutyPatterns[data >> 6];
            break;

        case IO_NR12: // envelope/volume
            // disable if DAC off
            if((data & 0xF8) == 0)
                channelEnabled &= ~(1 << 0);
            break;

        case IO_NR13: // freq lo
        {
            auto freq = data | ((mem.readIOReg(IO_NR14) & 0x7) << 8);
            ch1FreqTimerPeriod = (2048 - freq) * 4;
            break;
        }

        case IO_NR14: // freq hi/trigger/counter
        {
            auto freq = mem.readIOReg(IO_NR13) | ((data & 0x7) << 8);
            ch1FreqTimerPeriod = (2048 - freq) * 4;

            if(data & NRx4_Trigger)
            {
                if(mem.readIOReg(IO_NR12) & 0xF8)
                    channelEnabled |= (1 << 0);
            }
            break;
        }

        case IO_NR21: // length/duty
            ch2DutyPattern = dutyPatterns[data >> 6];
            break;

        case IO_NR22: // envelope/volume
            // disable if DAC off
            if((data & 0xF8) == 0)
                channelEnabled &= ~(1 << 1);
            break;

        case IO_NR23: // freq lo
        {
            auto freq = data | ((mem.readIOReg(IO_NR24) & 0x7) << 8);
            ch2FreqTimerPeriod = (2048 - freq) * 4;
            break;
        }

        case IO_NR24: // freq hi/trigger/counter
        {
            auto freq = mem.readIOReg(IO_NR23) | ((data & 0x7) << 8);
            ch2FreqTimerPeriod = (2048 - freq) * 4;

            if(data & NRx4_Trigger)
            {
                if(mem.readIOReg(IO_NR22) & 0xF8)
                    channelEnabled |= (1 << 1);
            }
            break;
        }

        case IO_NR30: // enable
            // disable if DAC off
            if(!(data & 0x80))
                channelEnabled &= ~(1 << 2);
            break;

        case IO_NR34: // freq hi/trigger/counter
            if(data & NRx4_Trigger)
            {
                if(mem.readIOReg(IO_NR30) & 0x80)
                    channelEnabled |= (1 << 2);
            }
            break;

        case IO_NR42: // envelope/volume
            // disable if DAC off
            if((data & 0xF8) == 0)
                channelEnabled &= ~(1 << 3);
            break;

        case IO_NR44: // freq hi/trigger/counter
            if(data & NRx4_Trigger)
            {
                if(mem.readIOReg(IO_NR42) & 0xF8)
                    channelEnabled |= (1 << 3);
            }
            break;
    }

    // make sure enabled flags are up to date
    mem.writeIOReg(IO_NR52, (mem.readIOReg(IO_NR52) & 0xF0) | channelEnabled);
}