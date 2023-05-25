#include <cstring>

#include "gameblit.hpp"
#include "assets.hpp"

#ifdef PICO_BUILD
#include "st7789.hpp"
#endif

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"
#include "file-browser.hpp"
#include "menu.hpp"

#ifdef PROFILER
#include "engine/profiler.hpp"

blit::Profiler profiler;
blit::ProfilerProbe *profilerUpdateProbe, *profilerRenderProbe;
#endif

// catch running out of memory
#ifdef TARGET_32BLIT_HW
extern "C" void *_sbrk(ptrdiff_t incr)
{
    extern char end, __ltdc_start;
    static char *heap_end;

    if(!heap_end)
        heap_end = &end;

    // ltdc is at the end of the heap
    if(heap_end + incr > &__ltdc_start)
        return (void *)-1;

    char *ret = heap_end;
    heap_end += incr;

    return (void *)ret;
}
#endif

void addAppendedFiles()
{
#if defined(TARGET_32BLIT_HW)
    extern char _flash_end;
    auto appFilesPtr = &_flash_end;
#elif defined(PICO_BUILD)
    extern char __flash_binary_end;
    auto appFilesPtr = &__flash_binary_end;
    appFilesPtr = (char *)(((uintptr_t)appFilesPtr) + 0xFF & ~0xFF); // round up to 256 byte boundary
#else
    char *appFilesPtr = nullptr;
    return;
#endif

    if(memcmp(appFilesPtr, "APPFILES", 8) != 0)
        return;

    uint32_t numFiles = *reinterpret_cast<uint32_t *>(appFilesPtr + 8);

    const int headerSize = 12, fileHeaderSize = 8;

    auto dataPtr = appFilesPtr + headerSize + fileHeaderSize * numFiles;

    for(auto i = 0u; i < numFiles; i++)
    {
        auto filenameLength = *reinterpret_cast<uint16_t *>(appFilesPtr + headerSize + i * fileHeaderSize);
        auto fileLength = *reinterpret_cast<uint32_t *>(appFilesPtr + headerSize + i * fileHeaderSize + 4);

        blit::File::add_buffer_file("/" + std::string(dataPtr, filenameLength), reinterpret_cast<uint8_t *>(dataPtr + filenameLength), fileLength);

        dataPtr += filenameLength + fileLength;
    }
}

const blit::Font tallFont(tall_font);
duh::FileBrowser fileBrowser(tallFont);

DMGCPU cpu;

#ifndef PICO_BUILD
static uint16_t screenData[160 * 144];
#endif

// ROM cache
#ifdef PICO_BUILD
static const int romBankCacheSize = 0; // could fit 2 with GBC removed
#else
static const int romBankCacheSize = 11;
static const int extraROMBankCacheSize = 4;

static uint8_t extraROMBankCache[0x4000 * extraROMBankCacheSize]{1}; // sneakily steal some of DTCMRAM
#endif

static uint8_t romBankCache[0x4000 * romBankCacheSize];

bool loaded = false;
std::string loadedFilename;
blit::File romFile;

bool turbo = false;
bool awfulScale = false;

void updateCartRAM(uint8_t *cartRam, unsigned int size);

// menu
enum class MenuItem
{
    SaveRAM,
    LoadState,
    SaveState,
    Reset,
    SwitchGame,
};

Menu menu("Menu",
{
    {static_cast<int>(MenuItem::SaveRAM), "Save Cart RAM"},
    {static_cast<int>(MenuItem::LoadState), "Load State"},
    {static_cast<int>(MenuItem::SaveState), "Save State"},
    {static_cast<int>(MenuItem::Reset), "Reset"},
    {static_cast<int>(MenuItem::SwitchGame), "Switch Game"}
}, tallFont);

bool menuOpen = false;

bool redwawBG = true;
uint32_t lastUpdate = 0;

#ifdef _MSC_VER
#include <intrin.h>
static int log2i(unsigned int x)
{
    unsigned long idx = 0;
    _BitScanReverse(&idx, x);
    return idx;
}
#else
static int log2i(unsigned int x)
{
    return 8 * sizeof(unsigned int) - __builtin_clz(x) - 1;
}
#endif

// assuming RLE/palette and that data is the right size
void packedToRGB(const uint8_t *packed_data, uint8_t *data)
{
    auto image = *(const blit::packed_image *)packed_data;

    int palette_entry_count = image.palette_entry_count;
    if(palette_entry_count == 0)
      palette_entry_count = 256;

    uint8_t bit_depth = log2i(std::max(1, palette_entry_count - 1)) + 1;

    auto palette = (const blit::Pen *)(packed_data + sizeof(blit::packed_image));
    auto image_data = packed_data + sizeof(blit::packed_image) + palette_entry_count * 4;
    auto end = packed_data + image.byte_count;

    uint8_t *pdest = (uint8_t *)data;
    auto dest_end = pdest + blit::screen.row_stride * blit::screen.bounds.h;
    int parse_state = 0;
    uint8_t count = 0, col = 0, bit = 0;

    for (auto bytes = image_data; bytes < end; ++bytes) {
        uint8_t b = *bytes;

        for (auto j = 0; j < 8; j++)
        {
            switch (parse_state)
            {
                case 0: // flag
                    if(b & (0b10000000 >> j))
                        parse_state = 1;
                    else
                        parse_state = 2;
                    break;
                case 1: // repeat count
                    count <<= 1;
                    count |= ((0b10000000 >> j) & b) ? 1 : 0;
                    if(++bit == 8)
                    {
                        parse_state = 2;
                        bit = 0;
                    }
                    break;

                case 2: // value
                    col <<= 1;
                    col |= ((0b10000000 >> j) & b) ? 1 : 0;

                    if(++bit == bit_depth)
                    {
                        for (int c = 0; c <= count; c++)
                        {
#ifdef PICO_BUILD
                            uint16_t packed = (palette[col].r >> 3) | ((palette[col].g >> 2) << 5) | ((palette[col].b >> 3) << 11);
                            *pdest++ = packed & 0xFF;
                            *pdest++ = packed >> 8;
#else
                            *pdest++ = palette[col].r;
                            *pdest++ = palette[col].g;
                            *pdest++ = palette[col].b;
#endif
                        }

                        bit = 0; col = 0;
                        parse_state = 0;
                        count = 0;

                        // done, skip any remaining padding
                        if(pdest == dest_end)
                            parse_state = 3;
                    }
                    break;
            }
        }
    }
}

void onMenuItemPressed(const Menu::Item &item)
{
    switch(static_cast<MenuItem>(item.id))
    {
        case MenuItem::SaveRAM:
            updateCartRAM(cpu.getMem().getCartridgeRAM(), cpu.getMem().getCartridgeRAMSize());
            break;

        case MenuItem::LoadState:
        {
            auto filename = loadedFilename.substr(0, loadedFilename.find_last_of('.') + 1) + "s0";
            blit::File file(filename);

            if(file.is_open())
            {
                cpu.loadSaveState(file.get_length(), [&file](uint32_t off, uint32_t len, uint8_t *buf) -> uint32_t
                {
                    auto ret = file.read(off, len, reinterpret_cast<char *>(buf));
                    return ret < 0 ? 0 : ret;
                });
            }
            break;
        }

        case MenuItem::SaveState:
        {
            auto filename = loadedFilename.substr(0, loadedFilename.find_last_of('.') + 1) + "s0";
            blit::File file(filename, blit::OpenMode::write);

            cpu.saveSaveState([&file](uint32_t off, uint32_t len, const uint8_t *buf) -> uint32_t
            {
                auto ret = file.write(off, len, reinterpret_cast<const char *>(buf));
                return ret < 0 ? 0 : ret;
            });
            break;
        }

        case MenuItem::Reset:
            cpu.reset();
            break;

        case MenuItem::SwitchGame:
            loaded = false;
            break;
    }

    menuOpen = false;
}

int loadedBanks = 0;
int bankLoadTime = 0;

void getROMBank(uint8_t bank, uint8_t *ptr)
{
    //blit::debugf("loading bank %i\n", bank);
    loadedBanks++;
    auto start = blit::now_us();
    romFile.read(bank * 0x4000, 0x4000, (char *)ptr);
    bankLoadTime += blit::us_diff(start, blit::now_us());
}

void updateCartRAM(uint8_t *cartRam, unsigned int size)
{
    auto saveFile = loadedFilename.substr(0, loadedFilename.find_last_of('.') + 1) + "sav";

    blit::File f(saveFile + ".tmp", blit::OpenMode::write);

    if(f.write(0, size, (const char *)cartRam) != int(size))
        return;

    // save RTC
    if(cpu.getMem().hasRTC())
    {
        uint32_t rtc[12];
        cpu.getMem().getRTCData(rtc);

        if(f.write(size, sizeof(rtc), (const char *)rtc) != sizeof(rtc))
            return;
    }

    f.close();

    blit::remove_file(saveFile + ".old"); // remove old
    blit::rename_file(saveFile, saveFile + ".old"); // move current -> old

    if(!blit::rename_file(saveFile + ".tmp", saveFile)) // move new -> current
        return;

    // cleanup for .gb.ram -> .sav
    if(blit::file_exists(loadedFilename + ".ram"))
    {
        blit::remove_file(loadedFilename + ".ram");
        blit::remove_file(loadedFilename + ".ram.old");
    }
}

void updateAudio(blit::AudioChannel &channel)
{
    auto &apu = cpu.getAPU();
    if(apu.getNumSamples() < 64)
    {
        //underrun
        memset(channel.wave_buffer, 0, 64 * 2);
        return;
    }

    for(int i = 0; i < 64; i++)
        channel.wave_buffer[i] = apu.getSample();
}

void openROM(std::string filename)
{
    romFile.open(filename);

    // use flash cache for anything bigger than 256K
    if(romFile.get_length() > 256 * 1024)
        romFile.open(filename, blit::OpenMode::read | blit::OpenMode::cached);

    auto &mem = cpu.getMem();

    mem.setCartROM(romFile.get_ptr());

    cpu.reset();

    auto saveFile = filename.substr(0, filename.find_last_of('.') + 1) + "sav";

    // fallback
    if(!blit::file_exists(saveFile))
        saveFile = filename + ".ram";

    if(blit::file_exists(saveFile))
    {
        blit::File file(saveFile);
        int ramLen = file.get_length();

        if(file.get_ptr())
            mem.loadCartridgeRAM(file.get_ptr(), std::min(ramLen, mem.getCartridgeRAMSize()));
        else
            file.read(0, mem.getCartridgeRAMSize(), (char *)mem.getCartridgeRAM());

        // save has RTC
        const int rtcDataSize = 48;
        if(ramLen % 8192 == rtcDataSize)
        {
            uint32_t rtc[12];
            file.read(ramLen - rtcDataSize, rtcDataSize, reinterpret_cast<char *>(rtc));
            mem.setRTCData(rtc);
        }
    }

    loaded = true;
    loadedFilename = filename;
}

void init()
{
    blit::set_screen_mode(blit::ScreenMode::hires);

    cpu.getMem().addROMCache(romBankCache, romBankCacheSize * 0x4000);

#ifdef PICO_BUILD
    // force the background onto the screen
    packedToRGB(asset_background_square, blit::screen.data);
    st7789::update();

    // reduce the framebuffer to the size of the emulated screen
    auto origSize = blit::screen.row_stride * blit::screen.bounds.h;
    blit::screen.bounds = {160, 144};
    blit::screen.row_stride = blit::screen.bounds.w * blit::screen.pixel_stride;
    st7789::set_window(40, 48, 160, 144);

    // ... and steal that extra RAM
    auto newSize = blit::screen.row_stride * blit::screen.bounds.h;
    cpu.getMem().addROMCache(blit::screen.data + newSize, origSize - newSize);

    cpu.getDisplay().setFramebuffer(reinterpret_cast<uint16_t *>(blit::screen.data));
#else
    // 32blit extra cache
    cpu.getMem().addROMCache(extraROMBankCache, extraROMBankCacheSize * 0x4000);
    cpu.getDisplay().setFramebuffer(screenData);
#endif

    blit::channels[0].waveforms = blit::Waveform::WAVE;
    blit::channels[0].wave_buffer_callback = &updateAudio;

    if(!turbo)
    {
        blit::channels[0].adsr = 0xFFFF00;
        blit::channels[0].trigger_sustain();
    }

    fileBrowser.set_extensions({".gb", ".gbc"});
    fileBrowser.set_on_file_open(openROM);

    // embed test ROM
#if 0
    blit::File::add_buffer_file("auto.gb", test_rom, test_rom_length);
    blit::File::add_buffer_file("auto.sav", test_ram, test_ram_length);
#endif

    addAppendedFiles();

    fileBrowser.init();

    menu.set_display_rect(blit::Rect(0, 0, 100, blit::screen.bounds.h));
    menu.set_on_item_activated(onMenuItemPressed);

    auto &mem = cpu.getMem();

    mem.setROMBankCallback(getROMBank);
    mem.setCartRamUpdateCallback(updateCartRAM);

    // autostart
    auto launchPath = blit::get_launch_path();
    if(launchPath)
        openROM(launchPath);
    else if(blit::file_exists("auto.gb"))
        openROM("auto.gb");

#ifdef PROFILER
    profiler.set_display_size(blit::screen.bounds.w, blit::screen.bounds.h);
    profiler.set_rows(5);
    profiler.set_alpha(200);
    profiler.display_history(true);

    profiler.setup_graph_element(blit::Profiler::dmCur, true, true, blit::Pen(0, 255, 0));
    profiler.setup_graph_element(blit::Profiler::dmAvg, true, true, blit::Pen(0, 255, 255));
    profiler.setup_graph_element(blit::Profiler::dmMax, true, true, blit::Pen(255, 0, 0));
    profiler.setup_graph_element(blit::Profiler::dmMin, true, true, blit::Pen(255, 255, 0));

    profilerUpdateProbe = profiler.add_probe("Update", 300);
    profilerRenderProbe = profiler.add_probe("Render", 300);
#endif
}

void render(uint32_t time_ms)
{
#ifdef PROFILER
    profilerRenderProbe->start();
#endif

    if(!loaded)
    {
        fileBrowser.render();
#ifdef PROFILER
        profilerRenderProbe->store_elapsed_us();
#endif
        return;
    }

#ifndef PICO_BUILD
    bool updateRunning = time_ms - lastUpdate < 20;

    if(redwawBG || !updateRunning)
    {
        if(awfulScale)
        {
            blit::screen.pen = blit::Pen(145, 142, 147);
            blit::screen.clear();
        }
        else
            packedToRGB(asset_background, blit::screen.data); // unpack directly to the screen

        redwawBG = !updateRunning;
    }

    auto gbScreen = screenData;

    auto expandCol = [](uint16_t rgb555, uint8_t &r, uint8_t &g, uint8_t &b)
    {
        r = (rgb555 & 0x1F) << 3;
        g = (rgb555 & 0x3E0) >> 2;
        b = (rgb555 & 0x7C00) >> 7;
    };

    if(awfulScale)
    {
        int oy = 0;

        auto copyLine = [gbScreen, &expandCol](int y, int y1, int oy)
        {
            auto ptr = blit::screen.ptr(27, oy++);
            for(int x = 0; x < 158; x += 3)
            {
                uint8_t tmpR, tmpG, tmpB;

                uint8_t r1, r2, r3, g1, g2, g3, b1, b2, b3;
                expandCol(gbScreen[x + y * 160], r1, g1, b1);
                expandCol(gbScreen[x + y1 * 160], tmpR, tmpG, tmpB);
                r1 = (r1 + tmpR) / 2;
                g1 = (g1 + tmpG) / 2;
                b1 = (b1 + tmpB) / 2;

                expandCol(gbScreen[(x + 1) + y * 160], r2, g2, b2);
                expandCol(gbScreen[(x + 1) + y1 * 160], tmpR, tmpG, tmpB);
                r2 = (r2 + tmpR) / 2;
                g2 = (g2 + tmpG) / 2;
                b2 = (b2 + tmpB) / 2;

                expandCol(gbScreen[(x + 2) + y * 160], r3, g3, b3);
                expandCol(gbScreen[(x + 2) + y1 * 160], tmpR, tmpG, tmpB);
                r3 = (r3 + tmpR) / 2;
                g3 = (g3 + tmpG) / 2;
                b3 = (b3 + tmpB) / 2;

                *ptr++ = r1; *ptr++ = g1; *ptr++ = b1;
                *ptr++ = (r1 + r2) / 2; *ptr++ = (g1 + g2) / 2; *ptr++ = (b1 + b2) / 2;
                *ptr++ = r2; *ptr++ = g2; *ptr++ = b2;
                *ptr++ = (r2 + r3) / 2; *ptr++ = (g2 + g3) / 2; *ptr++ = (b2 + b3) / 2;
                *ptr++ = r3; *ptr++ = g3; *ptr++ = b3;
            }

            // one pixel left
            uint8_t r1, g1, b1, r2, g2, b2;
            expandCol(gbScreen[159 + y * 160], r1, g1, b1);
            expandCol(gbScreen[159 + y1 * 160], r2, g2, b2);

            *ptr++ = (r1 + r2) / 2; *ptr++ = (g1 + g2) / 2; *ptr++ = (b1 + b2) / 2;
        };
        for(int y = 0; y < 144; y += 3)
        {
            copyLine(y, y, oy++);
            copyLine(y, y + 1, oy++);
            copyLine(y + 1, y + 1, oy++);
            copyLine(y + 1, y + 2, oy++);
            copyLine(y + 2, y + 2, oy++);
        }
    }
    else
    {
        for(int y = 0; y < 144; y++)
        {
            auto ptr = blit::screen.ptr(80, y + 48);
            for(int x = 0; x < 160; x++)
            {
                expandCol(gbScreen[x + y * 160], *ptr, *(ptr + 1), *(ptr + 2));
                ptr += 3;
            }
        }
    }
#endif

    if(menuOpen)
    {
        menu.render();
        redwawBG = true;
    }

#ifdef PROFILER
    profilerRenderProbe->store_elapsed_us();

    profiler.set_graph_time(profilerRenderProbe->elapsed_metrics().uMaxElapsedUs);
#endif
}

void update(uint32_t time_ms)
{
#ifdef PROFILER
    blit::ScopedProfilerProbe scopedProbe(profilerUpdateProbe);
#endif

    lastUpdate = time_ms;

    if(!loaded)
    {
        fileBrowser.update(time_ms);
        return;
    }

    // menu
    if(blit::buttons.released & blit::Button::MENU)
        menuOpen = !menuOpen;

    if(menuOpen)
    {
        menu.update(time_ms);
        return;
    }

#if defined(TARGET_32BLIT_HW) || defined(PICO_BUILD)
    if(blit::now() - time_ms >= 20)
        return;
#endif

    auto start = blit::now();

    // translate inputs
    uint8_t inputs = (blit::buttons & 0xFC) | // UP/DOWN/A/B match, select -> X, start -> Y
                     ((blit::buttons & blit::Button::DPAD_RIGHT) >> 1) |
                     ((blit::buttons & blit::Button::DPAD_LEFT) << 1);

    cpu.setInputs(inputs);

    // toggle the awful 1.5x scale
    if(blit::buttons.released & blit::Button::JOYSTICK)
    {
        awfulScale = !awfulScale;
        redwawBG = true;
    }

    loadedBanks = 0;
    bankLoadTime = 0;

    if(cpu.getAPU().getNumSamples() < 1024 - 225) // single update generates ~220 samples
    {
        cpu.run(10);
        cpu.getAPU().update();
    }
    else
        printf("CPU stalled, no audio room!\n");

    auto end = blit::now();

    if(end - start > 10)
        blit::debugf("running slow! %ims %i banks/%ius\n", end-start, loadedBanks, bankLoadTime);

    // SPEEEEEEEED
    while(turbo && blit::now() - start < 9)
    {
        // discard audio
        auto &apu = cpu.getAPU();
        while(apu.getNumSamples())
            apu.getSample();
        cpu.run(1);
    }

#ifdef PROFILER
    static int lastLogTime = time_ms;

    if(time_ms - lastLogTime >= 1000)
    {
        profiler.log_probes();
        lastLogTime = time_ms;
    }
#endif
}
