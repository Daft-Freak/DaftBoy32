#include <cstring>

#include "gameblit.hpp"
#include "assets.hpp"

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"
#include "file-browser.hpp"

const blit::Font tallFont(tall_font);
FileBrowser fileBrowser(tallFont);

DMGMemory mem;
DMGCPU cpu(mem);
DMGDisplay display(cpu);
DMGAPU apu(cpu);

uint8_t inputs = 0;

bool loaded = false;
std::string loadedFilename;
blit::File romFile;

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

bool onWrite(uint16_t addr, uint8_t val)
{
    apu.writeReg(addr, val);

    return false;
}


void getROMBank(uint8_t bank, uint8_t *ptr)
{
    printf("loading bank %i\n", bank);
    romFile.read(bank * 0x4000, 0x4000, (char *)ptr);
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

void openROM(std::string filename)
{
    romFile.open(filename);

    if(blit::file_exists(filename + ".ram"))
    {
        blit::File file(filename + ".ram");
        auto ramLen = file.get_length();

        if(file.get_ptr())
            mem.loadCartridgeRAM(file.get_ptr(), ramLen);
        else
        {
            auto ramData = new uint8_t[ramLen];
            file.read(0, ramLen, (char *)ramData);
            mem.loadCartridgeRAM(ramData, ramLen);
            delete[] ramData;
        }
    }

    cpu.reset();
    loaded = true;
    loadedFilename = filename;
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

    fileBrowser.setExtensions({".gb"});
    fileBrowser.setDisplayRect(blit::Rect(5, 5, blit::screen.bounds.w - 10, blit::screen.bounds.h - 10));
    fileBrowser.setOnFileOpen(openROM);

    // embed test ROM
#if 0
    blit::File::add_buffer_file("auto.gb", test_rom, test_rom_length);
    blit::File::add_buffer_file("auto.gb.ram", test_ram, test_ram_length);
#endif

    fileBrowser.init();

    cpu.setCycleCallback(onCyclesExeceuted);
    mem.setROMBankCallback(getROMBank);
    mem.setReadCallback(onRead);
    mem.setWriteCallback(onWrite);

    // autostart
    if(blit::file_exists("auto.gb"))
        openROM("auto.gb");
}

void render(uint32_t time_ms)
{
    blit::screen.pen = blit::Pen(20, 30, 40);
    blit::screen.clear();

    if(!loaded)
    {
        fileBrowser.render();
        return;
    }

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
    if(!loaded)
    {
        fileBrowser.update(time_ms);
        return;
    }

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

    // dump cart ram
    if((changedButtons & blit::Button::JOYSTICK) && !(blit::buttons & blit::Button::JOYSTICK))
    {
        blit::File f(loadedFilename + ".ram", blit::OpenMode::write);

        uint16_t addr = 0xA000;
        f.write(0, 0x8000, (const char *)mem.mapAddress(addr));
    }

    lastButtonState = blit::buttons;
}
