#include <cstdio>

#include "DMGAPU.h"

#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"
#include "DMGSaveState.h"

// TODO: there is even more strangeness here
static void nrx2Write(uint8_t old, uint8_t data, uint8_t &envVol, uint8_t &envTimer)
{
    int vol = envVol;

    if((data & (1 << 3)) && (old & 7) == 0) // -> inc, old timer 0
        vol++;

    if((old ^ data) & (1 << 3)) // direction change
    {
        if(data & (1 << 3) && (old & 7)) // dec -> inc, old timer not 0
            vol += 2;

        vol = 16 - vol;
    }

    if(!(data & (1 << 3)) && (old & 7) == 0 && (data & 7)) // -> dec, old timer 0, new timer not 0
        vol--;

    envVol = vol & 0xF;

    // apply immediately if enabling/disabling
    if(!(old & 7) != !(data & 7))
        envTimer = data & 7;
}

DMGAPU::DMGAPU(DMGCPU &cpu) : cpu(cpu)
{}

void DMGAPU::reset()
{
    enabled = true;

    lastUpdateCycle = 0;
    lastDivValue = cpu.getInternalTimer();

    frameSeqClock = 0;
    channelEnabled = 1;

    ch4FreqTimerPeriod = 8;

    //... incomplete

    sampleClock = 0;
    readOff = 0, writeOff = 64;

    // init wave RAM if we're a CGB (even in DMG mode)
    if(cpu.getConsole() == DMGCPU::Console::CGB || cpu.getColourMode())
    {
        for(int i = 0x30; i < 0x40;)
        {
            cpu.getMem().writeIOReg(i++, 0x00);
            cpu.getMem().writeIOReg(i++, 0xFF);
        }
    }
}

void DMGAPU::loadSaveState(DaftState &state)
{
    // CPU already set all the regs
    channelEnabled = cpu.getMem().readIOReg(IO_NR52) & 0xF;

    if(state.version)
    {
        frameSeqClock = state.frameSeqClock;
        enableCycle = state.enableCycle;
        skipNextFrameSeqUpdate = state.flags & State_APUSkipNextFrameSeqUpdate;

        ch1EnvVolume = state.envVolume[0];
        ch2EnvVolume = state.envVolume[1];
        ch4EnvVolume = state.envVolume[2];
        ch1EnvTimer = state.envTimer[0];
        ch2EnvTimer = state.envTimer[1];
        ch4EnvTimer = state.envTimer[2];
        ch1FreqTimer = state.freqTimer[0];
        ch2FreqTimer = state.freqTimer[1];
        ch3FreqTimer = state.freqTimer[2];
        ch4FreqTimer = state.freqTimer[3];

        ch1SweepTimer = state.ch1SweepTimer;
        ch1DutyStep = state.ch1DutyStep;
        ch1SweepCalcWithNeg = state.flags & State_APUCh1SweepCalcWithNeg;

        ch2DutyStep = state.ch2DutyStep;

        ch3Sample = state.ch3Sample;
        ch3SampleIndex = state.ch3SampleIndex;

        ch4LFSRBits = state.ch4LFSRBits;
    }

    lastDivValue = cpu.getInternalTimer();
    lastUpdateCycle = cpu.getCycleCount();
}

void DMGAPU::saveState(DaftState &state)
{
    state.frameSeqClock = frameSeqClock;
    state.enableCycle = enableCycle;

    if(skipNextFrameSeqUpdate)
        state.flags |= State_APUSkipNextFrameSeqUpdate;

    state.envVolume[0] = ch1EnvVolume;
    state.envVolume[1] = ch2EnvVolume;
    state.envVolume[2] = ch4EnvVolume;
    state.envTimer[0] = ch1EnvTimer;
    state.envTimer[1] = ch2EnvTimer;
    state.envTimer[2] = ch4EnvTimer;
    state.freqTimer[0] = ch1FreqTimer;
    state.freqTimer[1] = ch2FreqTimer;
    state.freqTimer[2] = ch3FreqTimer;
    state.freqTimer[3] = ch4FreqTimer;

    state.ch1SweepTimer = ch1SweepTimer;
    state.ch1DutyStep = ch1DutyStep;

    if(ch1SweepCalcWithNeg)
        state.flags |= State_APUCh1SweepCalcWithNeg;

    state.ch2DutyStep = ch2DutyStep;

    state.ch3Sample = ch3Sample;
    state.ch3SampleIndex = ch3SampleIndex;

    state.ch4LFSRBits = ch4LFSRBits;
}

void DMGAPU::update()
{
    auto curCycle = cpu.getCycleCount();
    if(lastUpdateCycle == curCycle)
        return;

    auto passed = curCycle - lastUpdateCycle;

    auto oldDiv = lastDivValue;

    if(cpu.getDoubleSpeedMode())
    {
        passed >>= 1;
        oldDiv >>= 1;
    }

    // output clock - attempt to get close to 22050Hz
    const int clocksPerSample = (4194304ull * 1024) / 22050;

    while(passed)
    {
        // clamp update step (min of next sample and next seq update)
        uint32_t nextSample = ((clocksPerSample - sampleClock) + 1023) / 1024;
        uint32_t nextFrameSeqUpdate = 8192u - (oldDiv & 0x1FFF);
        auto step = std::min(nextSample, std::min(nextFrameSeqUpdate, passed));

        updateFreq(step);

        // update frame sequencer clock
        if((oldDiv & 0x1FFF) + step >= 8192)
            updateFrameSequencer();

        // output
        sampleClock += step * 1024;

        if(sampleClock >= clocksPerSample)
        {
            sampleClock -= clocksPerSample;
            sampleOutput();
        }

        passed -= step;
        oldDiv += step;
    }

    lastUpdateCycle = curCycle;
    lastDivValue = cpu.getDoubleSpeedMode() ? oldDiv << 1 : oldDiv;
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
    update();

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
                // if we didn't just read, it fails entirely on DMG
                if(!cpu.getColourMode() && cpu.getCycleCount() != ch3LastAccessCycle)
                    return 0xFF;

                return cpu.getMem().readIOReg(0x30 + ch3SampleIndex / 2);
            }

            break;

        // these allow reading the current output
        case IO_PCM12:
        {
            auto ch1Val = (channelEnabled & 1) && this->ch1Val ? ch1EnvVolume : 0;
            auto ch2Val = (channelEnabled & 2) && this->ch2Val ? ch2EnvVolume : 0;

            return ch1Val | (ch2Val << 4);
        }

        case IO_PCM34:
        {
            auto vol = (cpu.getMem().readIOReg(IO_NR32) >> 5) & 0x3;
            auto ch3Val = (channelEnabled & 4) && vol ? ch3Sample : 0;
            if(vol)
                ch3Val >>= vol - 1;

            vol = ch4EnvVolume;
            auto ch4Val = (channelEnabled & 8) && this->ch4Val ? vol : 0;

            return ch3Val | (ch4Val << 4);
        }
    }


    return val;
}

bool DMGAPU::writeReg(uint16_t addr, uint8_t data)
{
    const uint8_t dutyPatterns[]{0b01000000, 0b11000000, 0b11110000, 0b00111111};

    if(addr == (0xFF00 | IO_DIV))
    {
        // force update when DIV is reset
        update();
        
        // extra update caused by bit changing
        int bit = cpu.getDoubleSpeedMode() ? (1 << 13) : (1 << 12);
        if(cpu.getInternalTimer() & bit)
            updateFrameSequencer();

        lastDivValue = 0;
    }

    if(addr < (0xFF00 | IO_NR10) || addr > 0xFF3F/* end of wave ram*/)
        return false;

    update();

    addr = addr & 0xFF;

    auto &mem = cpu.getMem();

    // ignore the write
    if(!enabled && addr >= IO_NR10 && addr < IO_NR52)
    {
        // DMG allows length writes
        if(cpu.getColourMode() || (addr != IO_NR11 && addr != IO_NR21 && addr != IO_NR31 && addr != IO_NR41))
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

            nrx2Write(mem.readIOReg(IO_NR12), data, ch1EnvVolume, ch1EnvTimer);

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

                ch1FreqTimer = ch1FreqTimerPeriod + 8; // delay

                // make sure frequency timer is aligned to 1MHz clock
                // (outside double speed mode both writes will always be aligned)
                if(cpu.getDoubleSpeedMode() && (cpu.getCycleCount() - enableCycle) & 4)
                   ch1FreqTimer += 2;

                // slightly smaller delay on restart
                if(channelEnabled & (1 << 0))
                    ch1FreqTimer -= 4;

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

            nrx2Write(mem.readIOReg(IO_NR22), data, ch2EnvVolume, ch2EnvTimer);
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

                ch2FreqTimer = ch2FreqTimerPeriod + 8; // delay

                // make sure frequency timer is aligned to 1MHz clock
                // (outside double speed mode both writes will always be aligned)
                if(cpu.getDoubleSpeedMode() && (cpu.getCycleCount() - enableCycle) & 4)
                   ch2FreqTimer += 2;

                // slightly smaller delay on restart
                if(channelEnabled & (1 << 1))
                    ch2FreqTimer -= 4;

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
                // the -2/+1 here are suspicious...
                if((channelEnabled & (1 << 2)) && !cpu.getColourMode() && ch3LastAccessCycle == cpu.getCycleCount() - 2)
                {
                    // triggering while active on DMG corrupts wave RAM
                    int index = (ch3SampleIndex + 1) % 32;

                    if(index < 8)
                    {
                        auto curByte = mem.readIOReg(0x30 + (index / 2));
                        mem.writeIOReg(0x30, curByte); // replace first byte
                    }
                    else
                    {
                        mem.writeIOReg(0x30, mem.readIOReg(0x30 + ((index / 2) & ~3)));
                        mem.writeIOReg(0x31, mem.readIOReg(0x31 + ((index / 2) & ~3)));
                        mem.writeIOReg(0x32, mem.readIOReg(0x32 + ((index / 2) & ~3)));
                        mem.writeIOReg(0x33, mem.readIOReg(0x33 + ((index / 2) & ~3)));
                    }
                }

                ch3SampleIndex = 0;
                ch3FreqTimer = ch3FreqTimerPeriod + 6; // there is a small delay

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

            nrx2Write(mem.readIOReg(IO_NR42), data, ch4EnvVolume, ch4EnvTimer);
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

                ch4FreqTimer = ch4FreqTimerPeriod / 2 + 8; // delay

                // make sure frequency timer is aligned to 1MHz clock
                // (outside double speed mode both writes will always be aligned)
                if(cpu.getDoubleSpeedMode() && (cpu.getCycleCount() - enableCycle) & 4)
                   ch4FreqTimer += 2;

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
                {
                    writeReg(0xFF00 | i, 0);
                    mem.writeIOReg(i, 0);
                }

                channelEnabled = 0;
                frameSeqClock = 0;

                ch1DutyStep = ch2DutyStep = 0;
                ch1Val = ch2Val = false;
                ch3SampleIndex = 0;
                ch3Sample = 0;
                ch4Val = 0;
            }
            else if(!enabled && (data & NR52_Enable))
            {
                frameSeqClock = 7; // make the next frame be 0

                // might skip an update
                int bit = cpu.getDoubleSpeedMode() ? (1 << 13) : (1 << 12);
                if(cpu.getInternalTimer() & bit)
                {
                    skipNextFrameSeqUpdate = true;
                    frameSeqClock = 6; // adjust for the frame we skip (needs to be even to trigger length bugs)
                }

                enableCycle = cpu.getCycleCount();
            }

            enabled = data & NR52_Enable;
            break;

        // wave RAM - writes the current byte when the channel is enabled
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
                // ignore write if not reading
                // assuming DMG only like the read case
                if(!cpu.getColourMode() && cpu.getCycleCount() != ch3LastAccessCycle)
                    return true;

                mem.writeIOReg(0x30 + ch3SampleIndex / 2, data);
                return true;
            }

            break;
    }

    return false;
}

void DMGAPU::updateFrameSequencer()
{
    auto &mem = cpu.getMem();

    frameSeqClock = (frameSeqClock + 1) & 7;

    if(skipNextFrameSeqUpdate)
    {
        // need to do this after the increment as we adjusted the clock back one
        skipNextFrameSeqUpdate = false;
        return;
    }

    if((frameSeqClock & 1) == 0)
    {
        // length

        const auto ch1FreqHi = mem.readIOReg(IO_NR14);
        const auto ch2FreqHi = mem.readIOReg(IO_NR24);
        const auto ch3FreqHi = mem.readIOReg(IO_NR34);
        const auto ch4FreqHi = mem.readIOReg(IO_NR44);

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

        if(ch1EnvTimer == 0 && (ch1EnvVol & 0x7))
        {
            if((ch1EnvVol & (1 << 3)) && ch1EnvVolume < 15)
                ch1EnvVolume++;
            else if(!(ch1EnvVol & (1 << 3)) && ch1EnvVolume > 0)
                ch1EnvVolume--;

            ch1EnvTimer = ch1EnvVol & 0x7;
        }

        ch2EnvTimer--;

        if(ch2EnvTimer == 0 && (ch2EnvVol & 0x7))
        {
            if((ch2EnvVol & (1 << 3)) && ch2EnvVolume < 15)
                ch2EnvVolume++;
            else if(!(ch2EnvVol & (1 << 3)) && ch2EnvVolume > 0)
                ch2EnvVolume--;

            ch2EnvTimer = ch2EnvVol & 0x7;
        }

        ch4EnvTimer--;

        if(ch4EnvTimer == 0 && (ch4EnvVol & 0x7))
        {
            if((ch4EnvVol & (1 << 3)) && ch4EnvVolume < 15)
                ch4EnvVolume++;
            else if(!(ch4EnvVol & (1 << 3)) && ch4EnvVolume > 0)
                ch4EnvVolume--;

            ch4EnvTimer = ch4EnvVol & 0x7;
        }
    }

    if((frameSeqClock & 0x3) == 2)
    {
        // sweep
        ch1SweepTimer--;

        if(ch1SweepEnable && ch1SweepTimer == 0)
        {
            const auto ch1Sweep = mem.readIOReg(IO_NR10);
            auto sweepPeriod = (ch1Sweep & NR10_Period) >> 4;
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

                ch1SweepTimer = (ch1Sweep & NR10_Period) >> 4;

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

void DMGAPU::updateFreq(int cyclesPassed)
{
    // channel 1
    if(channelEnabled & (1 << 0))
    {
        int timer = ch1FreqTimer;
        timer -= cyclesPassed;
        while(timer <= 0)
        {
            timer += ch1FreqTimerPeriod;
            ch1Val = ch1DutyPattern & (1 << ch1DutyStep);
            ch1DutyStep++;
            ch1DutyStep &= 7;
        }
        ch1FreqTimer = timer;
    }

    // channel 2
    if(channelEnabled & (1 << 1))
    {
        int timer = ch2FreqTimer;
        timer -= cyclesPassed;
        while(timer <= 0)
        {
            timer += ch2FreqTimerPeriod;
            ch2Val = ch2DutyPattern & (1 << ch2DutyStep);
            ch2DutyStep++;
            ch2DutyStep &= 7;
        }
        ch2FreqTimer = timer;
    }

    // channel 3
    if(channelEnabled & (1 << 2))
    {
        int timer = ch3FreqTimer;
        timer -= cyclesPassed;
        while(timer <= 0)
        {
            timer += ch3FreqTimerPeriod;
            ch3SampleIndex = (ch3SampleIndex + 1) % 32;

            auto sampleByte = cpu.getMem().readIOReg(0x30 + (ch3SampleIndex / 2));

            ch3LastAccessCycle = cpu.getCycleCount() - (ch3FreqTimerPeriod - timer);

            if(ch3SampleIndex & 1)
                ch3Sample = sampleByte & 0xF;
            else
                ch3Sample = sampleByte >> 4;
        }

        // calculate when this happened for read/write behaviour
        // needs to be updated every time to get the correct value
        // (updateFreq may be called multiple times in an update, and only in the last one is this right)
        if(ch3SampleIndex) // TODO: this really wants to check if a sample has happened
            ch3LastAccessCycle = cpu.getCycleCount() - (ch3FreqTimerPeriod - timer);

        ch3FreqTimer = timer;
    }

    // channel 4
    if(channelEnabled & (1 << 3))
    {
        int timer = ch4FreqTimer;
        timer -= cyclesPassed;
        while(timer <= 0)
        {
            timer += ch4FreqTimerPeriod;
            // make noise
            int bit = ((ch4LFSRBits >> 1) ^ ch4LFSRBits) & 1;
            ch4LFSRBits >>= 1;
            ch4LFSRBits |= bit << 14; // bit 15

            if(ch4Narrow)
                ch4LFSRBits = (ch4LFSRBits & ~(1 << 6)) | (bit << 6); // also set bit 7

            ch4Val = !(ch4LFSRBits & 1);
        }
        ch4FreqTimer = timer;
    }
}

void DMGAPU::sampleOutput()
{
    auto &mem = cpu.getMem();

    // wait for the audio thread/interrupt to catch up
    while((writeOff + 1) % bufferSize == readOff) {}

    auto masterVol = mem.readIOReg(IO_NR50);
    auto outputSelect = mem.readIOReg(IO_NR51);

    int leftVol = ((masterVol >> 4) & 7) + 1; // SO2
    int rightVol = (masterVol & 7) + 1; // SO1

    auto vol = ch1EnvVolume;
    auto ch1Val = (channelEnabled & 1) && this->ch1Val ? vol : -vol;

    vol = ch2EnvVolume;
    auto ch2Val = (channelEnabled & 2) && this->ch2Val ? vol : -vol;

    vol = (mem.readIOReg(IO_NR32) >> 5) & 0x3;
    auto ch3Val = (channelEnabled & 4) && vol ? (ch3Sample * 2) - 0xF : 0;

    if(vol)
        ch3Val >>= vol - 1;

    vol = ch4EnvVolume;
    auto ch4Val = (channelEnabled & 8) && this->ch4Val ? vol : -vol;

    int32_t left = 0, right = 0;

    if(outputSelect & 0x01)
        right += ch1Val * rightVol;
    if(outputSelect & 0x10)
        left += ch1Val * leftVol;

    if(outputSelect & 0x02)
        right += ch2Val * rightVol;
    if(outputSelect & 0x20)
        left += ch2Val * leftVol;

    if(outputSelect & 0x04)
        right += ch3Val * rightVol;
    if(outputSelect & 0x40)
        left += ch3Val * leftVol;

    if(outputSelect & 0x08)
        right += ch4Val * rightVol;
    if(outputSelect & 0x80)
        left += ch4Val * leftVol;

    left *= 0x20;
    right *= 0x20;

    // filter
    int32_t outLeft = 0, outRight = 0;
    outLeft = left - (filterVal[0] >> 16);
    outRight = right - (filterVal[1] >> 16);
    filterVal[0] = (left << 16) - (outLeft * 65014); // 65014 ~= 0.9920 * 0x10000
    filterVal[1] = (right << 16) - (outRight * 65014);

    sampleData[writeOff] = outLeft + outRight;
    writeOff = (writeOff + 1) % bufferSize;
}