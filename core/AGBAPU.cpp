#include <algorithm>
#include <cstdio>

#include "AGBAPU.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

AGBAPU::AGBAPU(AGBCPU &cpu) : cpu(cpu)
{}

void AGBAPU::reset()
{
    enabled = true;

    lastUpdateCycle = 0;

    frameSeqClock = 0;
    channelEnabled = 1;

    ch4FreqTimerPeriod = 8;

    //... incomplete

    sampleClock = 0;
    readOff = 0, writeOff = 64;

    // init wave RAM
    /*for(int i = 0x30; i < 0x40;)
    {
        cpu.getMem().writeIOReg(i++, 0x00);
        cpu.getMem().writeIOReg(i++, 0xFF);
    }*/
}

void AGBAPU::update()
{
    auto curCycle = cpu.getCycleCount();
    if(lastUpdateCycle == curCycle)
        return;

    auto passed = curCycle - lastUpdateCycle;

    auto oldCycle = lastUpdateCycle >> 2;

    passed >>= 2;
    lastUpdateCycle += passed << 2;

    // output clock
    const int clocksPerSample = 4194304 / 32768;

    while(passed)
    {
        // clamp update step (min of next sample and next seq update)
        uint32_t nextSample = clocksPerSample - sampleClock;
        uint32_t nextFrameSeqUpdate = 8192u - (oldCycle & 0x1FFF);
        auto step = std::min(nextSample, std::min(nextFrameSeqUpdate, passed));

        updateFreq(step);

        // update frame sequencer clock
        if((oldCycle & 0x1FFF) + step >= 8192)
            updateFrameSequencer();

        // output
        sampleClock += step;

        if(sampleClock >= clocksPerSample)
        {
            sampleClock -= clocksPerSample;
            sampleOutput();
        }

        passed -= step;
        oldCycle += step;
    }
}

// timer 0 or 1 overflow, may need to update DMA channels
void AGBAPU::timerOverflow(int timer, uint32_t cycle)
{
    auto control = cpu.getMem().readIOReg(IO_SOUNDCNT_H);

    if(!(control & 0x3300))
        return; // no DMA channels enabled

    if(timer == 0 && ((control & 0x4400) == 0x4400))
        return; // both channels using timer 1

    if(timer == 1 && !(control & 0x4400))
        return; // both channels using timer 0

    update(); //FIXME: to passed in cycle?

    // enabled and using this timer
    if(control & 0x300 && ((control & (1 << 10)) != 0) == (timer == 1))
    {
        dmaAVal = dmaAFIFO[fifoARead];
        fifoARead = (fifoARead + 1) & 0x1F;
        fifoAFilled--;

        // trigger DMA for more data
        if(fifoAFilled <= 16)
            cpu.triggerDMA(AGBCPU::Trig_SoundA);
    }

    if(control & 0x3000 && ((control & (1 << 14)) != 0) == (timer == 1))
    {
        dmaBVal = dmaBFIFO[fifoBRead];
        fifoBRead = (fifoBRead + 1) & 0x1F;
        fifoBFilled--;

        if(fifoBFilled <= 16)
            cpu.triggerDMA(AGBCPU::Trig_SoundB);
    }
}

int16_t AGBAPU::getSample()
{
    auto ret = sampleData[readOff];

    readOff = (readOff + 1) % bufferSize;

    return ret;
}

int AGBAPU::getNumSamples() const
{
    int avail = writeOff - readOff;
    if(avail < 0)
        avail += bufferSize;

    return avail;
}

uint16_t AGBAPU::readReg(uint32_t addr, uint16_t val)
{
    update();

    switch(addr & 0xFFFFFF)
    {
        case IO_SOUNDCNT_X:
            return (val & 0xFFF0) | 0x70 | channelEnabled; // enabled channels is read only

        // TODO: wave RAM
    }


    return val;
}

bool AGBAPU::writeReg(uint32_t addr, uint16_t data)
{
    const uint8_t dutyPatterns[]{0b01000000, 0b11000000, 0b11110000, 0b00111111};

    if((addr & 0xFFFFFF) < IO_SOUND1CNT_L || (addr & 0xFFFFFF) > IO_FIFO_B + 2)
        return false;

    update();

    addr = addr & 0xFFFFFF;

    auto &mem = cpu.getMem();

    // ignore the write (?)
    if(!enabled && addr >= IO_SOUND1CNT_L && addr < IO_SOUNDCNT_X)
        return true;

    switch(addr)
    {
        case IO_SOUND1CNT_L: // sweep

            // switching negate off after an update with it on kills the channel
            if(!(data & SOUND1CNT_L_Negate) && (mem.readIOReg(IO_SOUND1CNT_L) & SOUND1CNT_L_Negate) && ch1SweepCalcWithNeg)
                channelEnabled &= ~1;
            break;

        case IO_SOUND1CNT_H: // length/duty/env
            ch1DutyPattern = dutyPatterns[(data >> 6) & 3];
            ch1Len = 64 - (data & 0x3F);

            // disable if DAC off
            if((data & 0xF800) == 0)
                channelEnabled &= ~(1 << 0);

            if(!enabled)
                return true; // don't store it if disabled
            break;

        case IO_SOUND1CNT_X: // freq/trigger/counter
        {
            auto freq = data & 0x7FF;
            ch1FreqTimerPeriod = (2048 - freq) * 4;

            // enabling counter can cause an extra clock
            if((data & SOUNDxCNT_Length) && !(mem.readIOReg(IO_SOUND1CNT_X) & SOUNDxCNT_Length) && !(frameSeqClock & 1))
            {
                if(ch1Len && --ch1Len == 0)
                    channelEnabled &= ~(1 << 0); // done, disable
            }

            if(data & SOUNDxCNT_Trigger)
            {
                // init sweep
                auto sweepReg = mem.readIOReg(IO_SOUND1CNT_L);
                auto sweepShift = sweepReg & SOUND1CNT_L_Shift;
                ch1SweepTimer = (sweepReg & SOUND1CNT_L_Period) >> 4;
                ch1SweepFreq = freq;
                ch1SweepEnable = ch1SweepTimer || sweepShift;

                if(ch1SweepTimer == 0)
                    ch1SweepTimer = 8;

                // reload envelope
                ch1EnvVolume = mem.readIOReg(IO_SOUND1CNT_H) >> 12;
                ch1EnvTimer = (mem.readIOReg(IO_SOUND1CNT_H) >> 8) & 0x7;

                ch1FreqTimer = ch1FreqTimerPeriod + 8; // delay

                // slightly smaller delay on restart
                if(channelEnabled & (1 << 0))
                    ch1FreqTimer -= 4;

                if(ch1Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & SOUNDxCNT_Length) && !(frameSeqClock & 1))
                        ch1Len = 63;
                    else
                        ch1Len = 64;
                }

                if(mem.readIOReg(IO_SOUND1CNT_H) & 0xF800)
                    channelEnabled |= (1 << 0);

                ch1SweepCalcWithNeg = false;

                // update sweep now if shift is set
                if(sweepShift)
                {
                    int newFreq = ch1SweepFreq >> sweepShift;

                    if(sweepReg & SOUND1CNT_L_Negate) // negate
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

        case IO_SOUND2CNT_L: // length/duty/env
            ch2DutyPattern = dutyPatterns[(data >> 6) & 3];
            ch2Len = 64 - (data & 0x3F);

            // disable if DAC off
            if((data & 0xF800) == 0)
                channelEnabled &= ~(1 << 1);

            if(!enabled)
                return true; // don't store it if disabled
            break;

        case IO_SOUND2CNT_H: // freq lo
        {
            auto freq = data & 0x7FF;
            ch2FreqTimerPeriod = (2048 - freq) * 4;

            // enabling counter can cause an extra clock
            if((data & SOUNDxCNT_Length) && !(mem.readIOReg(IO_SOUND2CNT_H) & SOUNDxCNT_Length) && !(frameSeqClock & 1))
            {
                if(ch2Len && --ch2Len == 0)
                    channelEnabled &= ~(1 << 1); // done, disable
            }

            if(data & SOUNDxCNT_Trigger)
            {
                // reload envelope
                ch2EnvVolume = mem.readIOReg(IO_SOUND2CNT_L) >> 12;
                ch2EnvTimer = (mem.readIOReg(IO_SOUND2CNT_L) >> 8) & 0x7;

                ch2FreqTimer = ch2FreqTimerPeriod + 8; // delay

                // slightly smaller delay on restart
                if(channelEnabled & (1 << 1))
                    ch2FreqTimer -= 4;

                if(ch2Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & SOUNDxCNT_Length) && !(frameSeqClock & 1))
                        ch2Len = 63;
                    else
                        ch2Len = 64;
                }

                if(mem.readIOReg(IO_SOUND2CNT_L) & 0xF800)
                    channelEnabled |= (1 << 1);
            }
            break;
        }

        case IO_SOUND3CNT_L: // enable
            // disable if DAC off
            if(!(data & 0x80))
                channelEnabled &= ~(1 << 2);

            if(!(channelEnabled & (1 << 2)))
                ch3BankIndex = (data >> 6) & 1;
            break;

        case IO_SOUND3CNT_H: // length
            ch3Len = 256 - (data & 0xFF);
            break;

        case IO_SOUND3CNT_X: // freq/trigger/counter
        {
            auto freq = data & 0x7FF;
            ch3FreqTimerPeriod = (2048 - freq) * 2;

            // enabling counter can cause an extra clock
            if((data & SOUNDxCNT_Length) && !(mem.readIOReg(IO_SOUND3CNT_X) & SOUNDxCNT_Length) && !(frameSeqClock & 1))
            {
                if(ch3Len && --ch3Len == 0)
                    channelEnabled &= ~(1 << 2); // done, disable
            }

            if(data & SOUNDxCNT_Trigger)
            {
                ch3SampleIndex = 0;
                ch3BankIndex = (mem.readIOReg(IO_SOUND3CNT_L) >> 6) & 1;
                ch3FreqTimer = ch3FreqTimerPeriod + 6; // there is a small delay

                if(ch3Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & SOUNDxCNT_Length) && !(frameSeqClock & 1))
                        ch3Len = 255;
                    else
                        ch3Len = 256;
                }

                if(mem.readIOReg(IO_SOUND3CNT_L) & 0x80)
                    channelEnabled |= (1 << 2);
            }
            break;
        }

        case IO_SOUND4CNT_L: // length/envelope/volume
            ch4Len = 64 - (data & 0x3F);

            // disable if DAC off
            if((data & 0xF800) == 0)
                channelEnabled &= ~(1 << 3);
            break;

        case IO_SOUND4CNT_H: // clock/width/trigger/counter
        {
            const int divisors[]{8, 16, 32, 48, 64, 80, 96, 112};
            auto shift = (data >> 4) & 0xF;
            auto divCode = data & 0x7;
            ch4FreqTimerPeriod = divisors[divCode] << shift;
            ch4Narrow = data & (1 << 3);

            // enabling counter can cause an extra clock
            if((data & SOUNDxCNT_Length) && !(mem.readIOReg(IO_SOUND4CNT_H) & SOUNDxCNT_Length) && !(frameSeqClock & 1))
            {
                if(ch4Len && --ch4Len == 0)
                    channelEnabled &= ~(1 << 3); // done, disable
            }

            if(data & SOUNDxCNT_Trigger)
            {
                // reload envelope
                ch4EnvVolume = mem.readIOReg(IO_SOUND4CNT_L) >> 12;
                ch4EnvTimer = (mem.readIOReg(IO_SOUND4CNT_L) >> 8) & 0x7;

                ch4FreqTimer = ch4FreqTimerPeriod;

                ch4LFSRBits = 0x7FFF;

                if(ch4Len == 0)
                {
                    // triggering resets length to max
                    // but also clocks if the counter is enabled
                    if((data & SOUNDxCNT_Length) && !(frameSeqClock & 1))
                        ch4Len = 63;
                    else
                        ch4Len = 64;
                }

                if(mem.readIOReg(IO_SOUND4CNT_L) & 0xF800)
                    channelEnabled |= (1 << 3);
            }
            break;
        }

        // ordering should work out for these...
        case IO_FIFO_A:
        case IO_FIFO_A + 2:
            if(fifoAFilled < 31)
            {
                dmaAFIFO[fifoAWrite++] = data & 0xFF;
                fifoAWrite &= 0x1F;
                dmaAFIFO[fifoAWrite++] = data >> 8;
                fifoAWrite &= 0x1F;
                fifoAFilled += 2;
            }
            break;

        case IO_FIFO_B:
        case IO_FIFO_B + 2:
            if(fifoBFilled < 31)
            {
                dmaBFIFO[fifoBWrite++] = data & 0xFF;
                fifoBWrite &= 0x1F;
                dmaBFIFO[fifoBWrite++] = data >> 8;
                fifoBWrite &= 0x1F;
                fifoBFilled += 2;
            }
            break;

        case IO_SOUNDCNT_H:

            // FIFO reset
            if(data & (1 << 11))
                fifoAFilled = fifoARead = fifoAWrite = 0;
            if(data & (1 << 15))
                fifoBFilled = fifoBRead = fifoBWrite = 0;
            break;

        case IO_SOUNDCNT_X:
            // disabling
            if(enabled && !(data & SOUNDCNT_X_Enable))
            {
                // disabled
                for(int i = IO_SOUND1CNT_L; i < IO_SOUNDCNT_X; i++)
                {
                    writeReg(i, 0);
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
            else if(!enabled && (data & SOUNDCNT_X_Enable))
            {
                frameSeqClock = 7; // make the next frame be 0
            }

            enabled = data & SOUNDCNT_X_Enable;
            break;

        // wave RAM
        case 0x90:
        case 0x92:
        case 0x94:
        case 0x96:
        case 0x98:
        case 0x9A:
        case 0x9C:
        case 0x9E:
        {
            // the bank that isn't currently in use
            int bank = 1 - ch3BankIndex;
            int off64 = bank * 2 + ((addr & 0xF) / 8);
            int byte = addr & 7;

            // swap everything around
            uint16_t swapped = data << 8 | data >> 8;

            ch3WaveBuf[off64] = (ch3WaveBuf[off64] & ~(0xFFFF00000000ull >> (byte * 8))) | (swapped << ((7 - byte) * 8));
            break;
        }
    }

    return false;
}

void AGBAPU::updateFrameSequencer()
{
    auto &mem = cpu.getMem();

    frameSeqClock = (frameSeqClock + 1) & 7;

    if((frameSeqClock & 1) == 0)
    {
        // length

        const auto ch1FreqHi = mem.readIOReg(IO_SOUND1CNT_X);
        const auto ch2FreqHi = mem.readIOReg(IO_SOUND2CNT_H);
        const auto ch3FreqHi = mem.readIOReg(IO_SOUND3CNT_X);
        const auto ch4FreqHi = mem.readIOReg(IO_SOUND4CNT_H);

        if((ch1FreqHi & SOUNDxCNT_Length) && ch1Len)
        {
            if(--ch1Len == 0)
                channelEnabled &= ~(1 << 0); // done, disable
        }

        if((ch2FreqHi & SOUNDxCNT_Length) && ch2Len)
        {
            if(--ch2Len == 0)
                channelEnabled &= ~(1 << 1); // done, disable
        }

        if((ch3FreqHi & SOUNDxCNT_Length) && ch3Len)
        {
            if(--ch3Len == 0)
                channelEnabled &= ~(1 << 2); // done, disable
        }

        if((ch4FreqHi & SOUNDxCNT_Length) && ch4Len)
        {
            if(--ch4Len == 0)
                channelEnabled &= ~(1 << 3); // done, disable
        }
    } 

    if(frameSeqClock == 7)
    {
        // envelope
        const auto ch1EnvVol = mem.readIOReg(IO_SOUND1CNT_H);
        const auto ch2EnvVol = mem.readIOReg(IO_SOUND2CNT_L);
        const auto ch4EnvVol = mem.readIOReg(IO_SOUND4CNT_L);

        ch1EnvTimer--;

        if(ch1EnvTimer == 0 && (ch1EnvVol & 0x700))
        {
            if(ch1EnvVol & (1 << 11) && ch1EnvVolume < 15)
                ch1EnvVolume++;
            else if(ch1EnvVolume > 0)
                ch1EnvVolume--;

            ch1EnvTimer = (ch1EnvVol >> 8) & 0x7;
        }

        ch2EnvTimer--;

        if(ch2EnvTimer == 0 && (ch2EnvVol & 0x700))
        {
            if(ch2EnvVol & (1 << 11) && ch2EnvVolume < 15)
                ch2EnvVolume++;
            else if(ch2EnvVolume > 0)
                ch2EnvVolume--;

            ch2EnvTimer = (ch2EnvVol >> 8) & 0x7;
        }

        ch4EnvTimer--;

        if(ch4EnvTimer == 0 && (ch4EnvVol & 0x700))
        {
            if(ch4EnvVol & (1 << 11) && ch4EnvVolume < 15)
                ch4EnvVolume++;
            else if(ch4EnvVolume > 0)
                ch4EnvVolume--;

            ch4EnvTimer = (ch4EnvVol >> 8) & 0x7;
        }
    }

    if((frameSeqClock & 0x3) == 2)
    {
        // sweep
        ch1SweepTimer--;

        if(ch1SweepEnable && ch1SweepTimer == 0)
        {
            const auto ch1Sweep = mem.readIOReg(IO_SOUND1CNT_L);
            auto sweepPeriod = (ch1Sweep & SOUND1CNT_L_Period) >> 4;
            if(sweepPeriod)
            {
                auto shift = ch1Sweep & SOUND1CNT_L_Shift;
                int newFreq = ch1SweepFreq >> shift;

                if(ch1Sweep & SOUND1CNT_L_Negate) // negate
                    newFreq = ch1SweepFreq - newFreq;
                else
                    newFreq = ch1SweepFreq + newFreq;

                if(shift && newFreq < 2048)
                {
                    ch1SweepFreq = newFreq;
                    mem.writeIOReg(IO_SOUND1CNT_L, (mem.readIOReg(IO_SOUND1CNT_L) & 0xF100) | newFreq);
                    ch1FreqTimerPeriod = (2048 - newFreq) * 4;
                }

                ch1SweepTimer = (ch1Sweep & SOUND1CNT_L_Period) >> 4;

                // calculate again for overflow check
                newFreq = ch1SweepFreq >> shift;

                if(ch1Sweep & SOUND1CNT_L_Negate) // negate
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

void AGBAPU::updateFreq(int cyclesPassed)
{
    // channel 1
    if(channelEnabled & (1 << 0) && ch1EnvVolume)
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
    if(channelEnabled & (1 << 1) && ch2EnvVolume)
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

            ch3SampleIndex++;

            if(ch3SampleIndex == 32)
            {
                // two bank mode - switch bank
                if(cpu.getMem().readIOReg(IO_SOUND3CNT_L) & (1 << 5))
                    ch3BankIndex ^= 1;
                ch3SampleIndex = 0;
            }

            int bank = ch3BankIndex;

            // rotate the sample
            auto tmp = ch3WaveBuf[bank * 2] >> 60;
            ch3WaveBuf[bank * 2 + 0] = ch3WaveBuf[bank * 2 + 0] << 4 | ch3WaveBuf[bank * 2 + 1] >> 60;
            ch3WaveBuf[bank * 2 + 1] = ch3WaveBuf[bank * 2 + 1] << 4 | tmp;

            ch3Sample = ch3WaveBuf[bank * 2] >> 60;
        }

        ch3FreqTimer = timer;
    }

    // channel 4
    if(channelEnabled & (1 << 3) && ch4EnvVolume)
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

void AGBAPU::sampleOutput()
{
    auto &mem = cpu.getMem();

    // wait for the audio thread/interrupt to catch up
    while(writeOff == readOff - 1) {}

    // TODO: master left/right volume in low byte
    auto outputSelect = mem.readIOReg(IO_SOUNDCNT_L) >> 8;
    // TODO sound 1-4 volume in bit 0-1
    auto dmaControl = mem.readIOReg(IO_SOUNDCNT_H);

    auto vol = ch1EnvVolume;
    auto ch1Val = (channelEnabled & 1) && this->ch1Val ? vol : -vol;

    vol = ch2EnvVolume;
    auto ch2Val = (channelEnabled & 2) && this->ch2Val ? vol : -vol;

    vol = (mem.readIOReg(IO_SOUND3CNT_H) >> 13) & 0x3;
    // TODO: bit 15 = 75% vol
    auto ch3Val = (channelEnabled & 4) && vol ? ((ch3Sample ^ 0xF) * 2) - 0xF : 0;
    ch3Val /= (1 << (vol - 1));

    vol = ch4EnvVolume;
    auto ch4Val = (channelEnabled & 8) && this->ch4Val ? vol : -vol;

    int32_t left = 0, right = 0;

    if(outputSelect & 0x01)
        right += ch1Val * 8;
    if(outputSelect & 0x10)
        left += ch1Val * 8;

    if(outputSelect & 0x02)
        right += ch2Val * 8;
    if(outputSelect & 0x20)
        left += ch2Val * 8;

    if(outputSelect & 0x04)
        right += ch3Val * 8;
    if(outputSelect & 0x40)
        left += ch3Val * 8;

    if(outputSelect & 0x08)
        right += ch4Val * 8;
    if(outputSelect & 0x80)
        left += ch4Val * 8;

    // shiny new DMA channels
    int dmaAVal = this->dmaAVal * 4;
    int dmaBVal = this->dmaBVal * 4;

    // 50% vol
    if(!(dmaControl & (1 << 2)))
        dmaAVal /= 2;
    if(!(dmaControl & (1 << 3)))
        dmaBVal /= 2;

    if(dmaControl & (1 << 8))
        right += dmaAVal;
    if(dmaControl & (1 << 9))
        left += dmaAVal;

    if(dmaControl & (1 << 12))
        right += dmaBVal;
    if(dmaControl & (1 << 13))
        left += dmaBVal;

    auto bias = mem.readIOReg(IO_SOUNDBIAS) & 0x3FE;

    // bias to unsigned and clamp to 10-bit
    left = std::min(0x3FF, std::max(0, left + bias));
    right = std::min(0x3FF, std::max(0, right + bias));

    // ... and go back to mono signed 16-bit for output...
    sampleData[writeOff] = ((left - 0x200) + (right - 0x200)) * 16;
    writeOff = (writeOff + 1) % bufferSize;
}