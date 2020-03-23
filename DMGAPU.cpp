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

    if(enabled)
    {
        // this gets called before the timer is incremented
        auto oldDiv = cpu.getInternalTimer();

        // update frame sequencer clock
        auto wasSet = oldDiv & (1 << 12);
        oldDiv += cycles;

        if(wasSet && !(oldDiv & (1 << 12)))
        {
            frameSeqClock = (frameSeqClock + 1) % 8;

            const auto ch1FreqHi = mem.readIOReg(IO_NR14);
            const auto ch2FreqHi = mem.readIOReg(IO_NR24);
            const auto ch3FreqHi = mem.readIOReg(IO_NR34);
            const auto ch4FreqHi = mem.readIOReg(IO_NR44);

            if((frameSeqClock & 1) == 0)
            {
                // length

                if((ch1FreqHi & NRx4_Counter) && ch1Len)
                {
                    if(--ch1Len == 0)
                        channelEnabled &= ~(1 << 0); // done, disable
                }

                if((ch2FreqHi & NRx4_Counter) && ch2Len)
                {
                    if(--ch2Len == 0)
                        channelEnabled &= ~(1 << 1); // done, disable
                }

                if((ch3FreqHi & NRx4_Counter) && ch3Len)
                {
                    if(--ch3Len == 0)
                        channelEnabled &= ~(1 << 2); // done, disable
                }

                if((ch4FreqHi & NRx4_Counter) && ch4Len)
                {
                    if(--ch4Len == 0)
                        channelEnabled &= ~(1 << 3); // done, disable
                }
            } 

            if(frameSeqClock == 7)
            {
                // envelope
                const auto ch1EnvVol = mem.readIOReg(IO_NR12);
                const auto ch2EnvVol = mem.readIOReg(IO_NR22);
                const auto ch4EnvVol = mem.readIOReg(IO_NR42);

                ch1EnvTimer--;

                if(ch1EnvTimer == 0)
                {
                    if(ch1EnvVol & (1 << 3) && ch1EnvVolume < 15)
                        ch1EnvVolume++;
                    else if(ch1EnvVolume > 0)
                        ch1EnvVolume--;

                    ch1EnvTimer = ch1EnvVol & 0x7;
                }

                ch2EnvTimer--;

                if(ch2EnvTimer == 0)
                {
                    if(ch2EnvVol & (1 << 3) && ch2EnvVolume < 15)
                        ch2EnvVolume++;
                    else if(ch2EnvVolume > 0)
                        ch2EnvVolume--;

                    ch2EnvTimer = ch2EnvVol & 0x7;
                }

                ch4EnvTimer--;

                if(ch4EnvTimer == 0)
                {
                    if(ch4EnvVol & (1 << 3) && ch4EnvVolume < 15)
                        ch4EnvVolume++;
                    else if(ch4EnvVolume > 0)
                        ch4EnvVolume--;

                    ch4EnvTimer = ch4EnvVol & 0x7;
                }
            }

            if((frameSeqClock & 0x3) == 2)
            {
                // sweep
                const auto ch1Sweep = mem.readIOReg(IO_NR10);
                auto sweepPeriod = (ch1Sweep & NR10_Period) >> 4;
                ch1SweepTimer--;

                if(ch1SweepEnable && ch1SweepTimer == 0)
                {
                    if(sweepPeriod)
                    {
                        auto shift = ch1Sweep & NR10_Shift;
                        int newFreq = ch1SweepFreq >> shift;

                        if(ch1Sweep & NR10_Negate) // negate
                            newFreq = ch1SweepFreq - newFreq;
                        else
                            newFreq = ch1SweepFreq + newFreq;

                        if(shift && newFreq < 2048)
                        {
                            ch1SweepFreq = newFreq;
                            mem.writeIOReg(IO_NR13, newFreq & 0xFF);
                            mem.writeIOReg(IO_NR14, (mem.readIOReg(IO_NR14) & 0xF1) | (newFreq >> 8));
                            ch1FreqTimerPeriod = (2048 - newFreq) * 4;
                        }

                        ch1SweepTimer = (ch1Sweep >> 4) & 3;

                        // calculate again for overflow check
                        newFreq = ch1SweepFreq >> shift;

                        if(ch1Sweep & NR10_Negate) // negate
                        {
                            newFreq = ch1SweepFreq - newFreq;
                            ch1SweepCalcWithNeg = true;
                        }
                        else
                            newFreq = ch1SweepFreq + newFreq;

                        if(newFreq >= 2048)
                            channelEnabled &= ~(1 << 0);
                    }
                    else // period is 0, reset to 8
                        ch1SweepTimer = 8;
                }
            }
        }

        // channel 1
        ch1FreqTimer -= cycles;

        while(ch1FreqTimer <= 0)
        {
            ch1FreqTimer += ch1FreqTimerPeriod;
            ch1DutyStep++;
            ch1DutyStep &= 7;
        }

        // channel 2
        ch2FreqTimer -= cycles;
    
        while(ch2FreqTimer <= 0)
        {
            ch2FreqTimer += ch2FreqTimerPeriod;
            ch2DutyStep++;
            ch2DutyStep &= 7;
        }

        // channel 3
        ch3FreqTimer -= cycles;

        while(ch3FreqTimer <= 0)
        {
            ch3FreqTimer += ch3FreqTimerPeriod;
            ch3SampleIndex = (ch3SampleIndex + 1) % 32;

            auto sampleByte = mem.readIOReg(0x30 + (ch3SampleIndex / 2));

            if(ch3SampleIndex & 1)
                ch3Sample = sampleByte & 0xF;
            else
                ch3Sample = sampleByte >> 4;
        }

        // channel 4
        ch4FreqTimer -= cycles;
    
        while(ch4FreqTimer <= 0)
        {
            ch4FreqTimer += ch4FreqTimerPeriod;
            // make noise
            bool bit = ((ch4LFSRBits >> 1) ^ ch4LFSRBits) & 1;
            ch4LFSRBits >>= 1;
            ch4LFSRBits |= bit << 14; // bit 15

            if(ch4Narrow)
                ch4LFSRBits |= (bit << 6); // also set bit 7

            ch4Val = ch4LFSRBits & 1;
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

        auto outputSelect = mem.readIOReg(IO_NR51);

        auto vol = ch1EnvVolume;
        auto ch1Val = (channelEnabled & 1) && (ch1DutyPattern & (1 << ch1DutyStep)) ? vol : -vol;

        vol = ch2EnvVolume;
        auto ch2Val = (channelEnabled & 2) && (ch2DutyPattern & (1 << ch2DutyStep)) ? vol : -vol;

        vol = (mem.readIOReg(IO_NR32) >> 5) & 0x3;
        auto ch3Val = (channelEnabled & 4) && vol ? (ch3Sample * 2) - 0xF : 0;
        ch3Val /= (1 << (vol - 1));

        vol = ch4EnvVolume;
        auto ch4Val = (channelEnabled & 8) && this->ch4Val ? vol : -vol;

        int32_t sample = 0;
        
        if(outputSelect & 1)
            sample += ch1Val;

        if(outputSelect & 2)
            sample += ch2Val;

        if(outputSelect & 4)
            sample += ch3Val;

        if(outputSelect & 8)
            sample += ch4Val;

        // TODO SO2?

        sampleData[writeOff] = sample * 0x222;
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

        // wave RAM - reads the current byte when the channel is enabled
        case 0x30:
        case 0x31:
        case 0x32:
        case 0x33:
        case 0x34:
        case 0x35:
        case 0x36:
        case 0x37:
        case 0x38:
        case 0x39:
        case 0x3A:
        case 0x3B:
        case 0x3C:
        case 0x3D:
        case 0x3E:
        case 0x3F:
            if(channelEnabled & (1 << 2))
            {
                return cpu.getMem().readIOReg(0x30 + ch3SampleIndex / 2);
            }

            break;
    }


    return val;
}

bool DMGAPU::writeReg(uint16_t addr, uint8_t data)
{
    const uint8_t dutyPatterns[]{0b00000001, 0b10000001, 0b10000111, 0b01111110};

    if(addr < 0xFF00)
        return false;

    addr = addr & 0xFF;

    auto &mem = cpu.getMem();

    // ignore the write
    if(!enabled && addr >= IO_NR10 && addr < IO_NR52)
    {
        // DMG allows length writes
        if(addr != IO_NR11 && addr != IO_NR21 && addr != IO_NR31 && addr != IO_NR41)
            return true;
    }

    switch(addr)
    {
        case IO_NR10: // sweep

            // switching negate off after an update with it on kills the channel
            if(!(data & NR10_Negate) && (mem.readIOReg(IO_NR10) & NR10_Negate) && ch1SweepCalcWithNeg)
                channelEnabled &= ~1;
            break;

        case IO_NR11: // length/duty
            ch1DutyPattern = dutyPatterns[data >> 6];
            ch1Len = 64 - (data & 0x3F);
            if(!enabled)
                return true; // don't store it if disabled
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

            // enabling counter can cause an extra clock
            if((data & NRx4_Counter) && !(mem.readIOReg(IO_NR14) & NRx4_Counter) && !(frameSeqClock & 1))
            {
                if(ch1Len && --ch1Len == 0)
                    channelEnabled &= ~(1 << 0); // done, disable
            }

            if(data & NRx4_Trigger)
            {
                // init sweep
                auto sweepReg = mem.readIOReg(IO_NR10);
                auto sweepShift = sweepReg & NR10_Shift;
                ch1SweepTimer = (sweepReg & NR10_Period) >> 4;
                ch1SweepFreq = freq;
                ch1SweepEnable = ch1SweepTimer || sweepShift;

                if(ch1SweepTimer == 0)
                    ch1SweepTimer = 8;

                // reload envelope
                ch1EnvVolume = mem.readIOReg(IO_NR12) >> 4;
                ch1EnvTimer = mem.readIOReg(IO_NR12) & 0x7;

                ch1FreqTimer = ch1FreqTimerPeriod;

                if(ch1Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & NRx4_Counter) && !(frameSeqClock & 1))
                        ch1Len = 63;
                    else
                        ch1Len = 64;
                }

                if(mem.readIOReg(IO_NR12) & 0xF8)
                    channelEnabled |= (1 << 0);

                ch1SweepCalcWithNeg = false;

                // update sweep now if shift is set
                if(sweepShift)
                {
                    int newFreq = ch1SweepFreq >> sweepShift;

                    if(sweepReg & NR10_Negate) // negate
                    {
                        newFreq = ch1SweepFreq - newFreq;
                        ch1SweepCalcWithNeg = true;
                    }
                    else
                        newFreq = ch1SweepFreq + newFreq;

                    if(newFreq >= 2048)
                        channelEnabled &= ~(1 << 0);
                }
            }
            break;
        }

        case IO_NR21: // length/duty
            ch2DutyPattern = dutyPatterns[data >> 6];
            ch2Len = 64 - (data & 0x3F);
            if(!enabled)
                return true;
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

            // enabling counter can cause an extra clock
            if((data & NRx4_Counter) && !(mem.readIOReg(IO_NR24) & NRx4_Counter) && !(frameSeqClock & 1))
            {
                if(ch2Len && --ch2Len == 0)
                    channelEnabled &= ~(1 << 1); // done, disable
            }

            if(data & NRx4_Trigger)
            {
                // reload envelope
                ch2EnvVolume = mem.readIOReg(IO_NR22) >> 4;
                ch2EnvTimer = mem.readIOReg(IO_NR22) & 0x7;

                ch2FreqTimer = ch2FreqTimerPeriod;

                if(ch2Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & NRx4_Counter) && !(frameSeqClock & 1))
                        ch2Len = 63;
                    else
                        ch2Len = 64;
                }

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

        case IO_NR31: // length
            ch3Len = 256 - data;
            break;

        case IO_NR33: // freq lo
        {
            auto freq = data | ((mem.readIOReg(IO_NR34) & 0x7) << 8);
            ch3FreqTimerPeriod = (2048 - freq) * 2;
            break;
        }

        case IO_NR34: // freq hi/trigger/counter
        {
            auto freq = mem.readIOReg(IO_NR33) | ((data & 0x7) << 8);
            ch3FreqTimerPeriod = (2048 - freq) * 2;

            // enabling counter can cause an extra clock
            if((data & NRx4_Counter) && !(mem.readIOReg(IO_NR34) & NRx4_Counter) && !(frameSeqClock & 1))
            {
                if(ch3Len && --ch3Len == 0)
                    channelEnabled &= ~(1 << 2); // done, disable
            }

            if(data & NRx4_Trigger)
            {
                ch3SampleIndex = 0;
                ch3FreqTimer = ch3FreqTimerPeriod;

                if(ch3Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & NRx4_Counter) && !(frameSeqClock & 1))
                        ch3Len = 255;
                    else
                        ch3Len = 256;
                }

                if(mem.readIOReg(IO_NR30) & 0x80)
                    channelEnabled |= (1 << 2);
            }
            break;
        }

        case IO_NR41: // length
            ch4Len = 64 - (data & 0x3F);
            break;

        case IO_NR42: // envelope/volume
            // disable if DAC off
            if((data & 0xF8) == 0)
                channelEnabled &= ~(1 << 3);
            break;

        case IO_NR43: // clock/width
        {
            const int divisors[]{8, 16, 32, 48, 64, 80, 96, 112};
            auto shift = data >> 4;
            auto divCode = data & 0x7;
            ch4FreqTimerPeriod = divisors[divCode] << shift;
            ch4Narrow = data & (1 << 3);
            break;
        }

        case IO_NR44: // freq hi/trigger/counter
            // enabling counter can cause an extra clock
            if((data & NRx4_Counter) && !(mem.readIOReg(IO_NR44) & NRx4_Counter) && !(frameSeqClock & 1))
            {
                if(ch4Len && --ch4Len == 0)
                    channelEnabled &= ~(1 << 3); // done, disable
            }

            if(data & NRx4_Trigger)
            {
                // reload envelope
                ch4EnvVolume = mem.readIOReg(IO_NR42) >> 4;
                ch4EnvTimer = mem.readIOReg(IO_NR42) & 0x7;

                ch4FreqTimer = ch4FreqTimerPeriod;

                ch4LFSRBits = 0x7FFF;

                if(ch4Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & NRx4_Counter) && !(frameSeqClock & 1))
                        ch4Len = 63;
                    else
                        ch4Len = 64;
                }

                if(mem.readIOReg(IO_NR42) & 0xF8)
                    channelEnabled |= (1 << 3);
            }
            break;

        case IO_NR52:
            // disabling
            if(enabled && !(data & NR52_Enable))
            {
                // disabled
                for(int i = IO_NR10; i < IO_NR52; i++)
                    mem.writeIOReg(i, 0);

                channelEnabled = 0;
                frameSeqClock = 0;
            }
            else if(!enabled && (data & NR52_Enable))
                frameSeqClock = 7; // make the next frame be 0

            enabled = data & NR52_Enable;
            break;
    }

    return false;
}