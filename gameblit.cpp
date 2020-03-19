#include <cstring>

#include "gameblit.hpp"
#include "assets.hpp"

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

//

DMGMemory mem;
DMGCPU cpu(mem);
DMGDisplay display(cpu);
DMGAPU apu(cpu);

uint8_t inputs = 0;

bool turbo = false;
bool awfulScale = false;

void onCyclesExeceuted(int cycles)
{
    apu.update(cycles);
    display.update(cycles);
}

uint8_t onRead(uint16_t addr, uint8_t val)
{
    if((addr & 0xFF) == IO_JOYP)
    {
        int ret = val & 0xF0;
        if(!(val & JOYP_SelectDir))
            ret |= (~inputs) & 0xF;
        if(!(val & JOYP_SelectButtons))
            ret |= ((~inputs) >> 4) & 0xF;
        return ret;
    }

    return apu.readReg(addr, val);
}

void onWrite(uint16_t addr, uint8_t val)
{
    apu.writeReg(addr, val);
}

void updateAudio(void *arg)
{
    if(apu.getNumSamples() < 64)
    {
        //underrun
        memset(blit::channels[0].wave_buffer, 0, 64 * 2);
        return;
    }

    for(int i = 0; i < 64; i++)
        blit::channels[0].wave_buffer[i] = apu.getSample();
}

void init()
{
    blit::set_screen_mode(blit::ScreenMode::hires);

    blit::channels[0].waveforms = blit::Waveform::WAVE;
    blit::channels[0].volume = 0xFF;
    blit::channels[0].callback_waveBufferRefresh = &updateAudio;

    if(!turbo)
    {
        blit::channels[0].adsr = 0xFFFF00;
        blit::channels[0].trigger_sustain();
    }

    mem.loadCartridge(test_rom, test_rom_length);
    cpu.setCycleCallback(onCyclesExeceuted);
    mem.setReadCallback(onRead);
    mem.setWriteCallback(onWrite);
    cpu.reset();
}

void render(uint32_t time_ms)
{
    blit::screen.pen = blit::Pen(20, 30, 40);
    blit::screen.clear();

    auto gbScreen = display.getData();

    if(awfulScale)
    {
        int oy = 0;

        auto copyLine = [gbScreen](int y, int y1, int oy)
        {
            auto ptr = blit::screen.ptr(40, oy++ + 12);
            for(int x = 0; x < 160; x += 2)
            {
                auto v1 = (gbScreen[x + y * 160] + gbScreen[x + y1 * 160]) / 2;
                auto v2 = (gbScreen[(x + 1) + y * 160] + gbScreen[(x + 1) + y1 * 160]) / 2;

                *ptr++ = v1; *ptr++ = v1; *ptr++ = v1;
                *ptr++ = (v1 + v2) / 2; *ptr++ = (v1 + v2) / 2; *ptr++ = (v1 + v2) / 2;
                *ptr++ = v2; *ptr++ = v2; *ptr++ = v2;
            }
        };
        for(int y = 0; y < 144; y += 2)
        {
            copyLine(y, y, oy++);
            copyLine(y, y + 1, oy++);
            copyLine(y + 1, y + 1, oy++);
        }
    }
    else
    {
        for(int y = 0; y < 144; y++)
        {
            auto ptr = blit::screen.ptr(80, y + 48);
            for(int x = 0; x < 160; x++)
            {
                *ptr++ = gbScreen[x + y * 160];
                *ptr++ = gbScreen[x + y * 160];
                *ptr++ = gbScreen[x + y * 160];
            }
        }
    }
}

void update(uint32_t time_ms)
{
    static uint32_t lastButtonState = 0;

    auto changedButtons = blit::buttons ^ lastButtonState;

    auto start = blit::now();

    // translate inputs
    // TODO: move input handling?
    auto oldInputs = inputs;
    inputs = (blit::buttons & 0x7C) | // UP/DOWN/A/B match, select -> X
             ((blit::buttons & blit::Button::DPAD_RIGHT) >> 1) |
             ((blit::buttons & blit::Button::DPAD_LEFT) << 1) |
             ((blit::buttons & blit::Button::HOME) >> 2); // start -> home

    if(oldInputs == 0 && inputs != 0)
        cpu.flagInterrupt(Int_Joypad);

    // toggle the awful 1.5x scale
    if((changedButtons & blit::Button::Y) && !(blit::buttons & blit::Button::Y))
        awfulScale = !awfulScale;

    if(apu.getNumSamples() < 1024 - 225) // single update generates ~220 samples
        cpu.run(10);
    else
        printf("CPU stalled, no audio room!\n");

    // SPEEEEEEEED
    while(turbo && blit::now() - start < 9)
    {
        // discard audio
        while(apu.getNumSamples())
            apu.getSample();
        cpu.run(1);
    }

    lastButtonState = blit::buttons;
}
