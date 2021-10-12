#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>

#include "png.h"

#include "DMGAPU.h"
#include "DMGCPU.h"
#include "DMGDisplay.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

static std::ifstream romFile;
static size_t romSize;

static DMGCPU *cpu;

static bool takeScreenshot = false;

static void getROMBank(uint8_t bank, uint8_t *ptr)
{
    auto addr = (bank * 0x4000);

    romFile.seekg(addr);
    romFile.read((char *)ptr, 0x4000);
}

// PNG load/save
// assuming 160 * 144
// and very little error checking
static uint8_t *loadPNG(const std::string &filename)
{
    auto f = fopen(filename.c_str(), "rb");

    if(!f)
        return nullptr;

    png_byte header[8];
    if(fread(header, 1, 8, f) != 8 || png_sig_cmp(header, 0, 8))
        return nullptr;

    auto pngRead = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    auto info = png_create_info_struct(pngRead);
    auto endInfo = png_create_info_struct(pngRead);

    if(setjmp(png_jmpbuf(pngRead)))
    {
        png_destroy_read_struct(&pngRead, &info, &endInfo);
        return nullptr;
    }

    png_init_io(pngRead, f);
    png_set_sig_bytes(pngRead, 8);

    png_read_png(pngRead, info, PNG_TRANSFORM_STRIP_16 | PNG_TRANSFORM_EXPAND | PNG_TRANSFORM_STRIP_ALPHA | PNG_TRANSFORM_GRAY_TO_RGB, nullptr);

    auto rows = png_get_rows(pngRead, info);

    png_uint_32 width, height;

    int bitDepth, colourType;

    png_get_IHDR(pngRead, info, &width, &height, &bitDepth, &colourType, nullptr, nullptr, nullptr);

    if(width != 160 || height != 144 || colourType != PNG_COLOR_TYPE_RGB)
    {
        png_destroy_read_struct(&pngRead, &info, &endInfo);
        fclose(f);
        return nullptr;
    }

    auto data = new uint8_t[width * height * 3];

    for(unsigned int y = 0; y < height; y++)
        memcpy(data + (y * width * 3), rows[y], width * 3);

    png_destroy_read_struct(&pngRead, &info, &endInfo);
    fclose(f);
    return data;
}

static void savePNG(const std::string &filename, const uint8_t *data)
{
    auto f = fopen(filename.c_str(), "wb");

    if(!f)
        return;

    auto pngWrite = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    auto info = png_create_info_struct(pngWrite);

    png_init_io(pngWrite, f);

    int width = 160, height = 144;

    png_set_IHDR(pngWrite, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    auto rows = static_cast<png_bytepp>(png_malloc(pngWrite, height * sizeof(png_bytep)));
    for(int y = 0; y < height; y++)
        rows[y] = const_cast<png_bytep>(data + y * width * 3);

    png_set_rows(pngWrite, info, rows);
    png_write_png(pngWrite, info, PNG_TRANSFORM_IDENTITY, nullptr);

    png_free(pngWrite, rows);

    png_destroy_write_struct(&pngWrite, &info);

    fclose(f);
}

static void dumpImage(const std::string &filename, const uint8_t *data)
{
    savePNG("./test-results/" + filename + ".png", data);
}

static void screenToRGB(DMGDisplay &display, uint8_t *outRGB)
{
    auto displayData = cpu->getDisplay().getData();

    for(int i = 0; i < 160 * 144; i++)
    {
        auto pixel = displayData[i];
        outRGB[i * 3 + 0] = (pixel & 0x1F) << 3;
        outRGB[i * 3 + 1] = (pixel & 0x3E0) >> 2;
        outRGB[i * 3 + 2] = (pixel & 0x7C00) >> 7;
    }
}

static bool runTest(const std::string &rom, DMGCPU::Console console = DMGCPU::Console::Auto)
{
    // find/open ROM
    std::string paths[]{
        "",
    };

    std::string basePath;

    for(auto path : paths)
    {
        basePath = path;
        romFile.open(path + rom);
        if(romFile)
            break;
    }

    if(!romFile)
    {
        std::cerr << "Failed to load " << rom << "\n";
        return false;
    }

    romFile.seekg(0, std::ios::end);
    romSize = romFile.tellg();
    romFile.seekg(0, std::ios::beg);

    // clean instance
    cpu = new DMGCPU;

    auto &mem = cpu->getMem();
    mem.setROMBankCallback(getROMBank);

    cpu->setConsole(console);

    cpu->reset();

    uint8_t rgbDisplay[160 * 144 * 3];

    unsigned int time = 0;
    bool result = false;

    while(!result)
    {
        auto &apu = cpu->getAPU();
        while(apu.getNumSamples())
            apu.getSample();
        cpu->run(10);

        time += 10;

        bool convertDisplay = takeScreenshot || cpu->getBreakpointTriggered();

        if(convertDisplay)
            screenToRGB(cpu->getDisplay(), rgbDisplay);

        if(takeScreenshot)
        {
            dumpImage("output", rgbDisplay);
            takeScreenshot = false;
        }
    }

    result = true;

    delete cpu;

    romFile.close();
    return result;
}

static void handleSignal(int signal)
{
    takeScreenshot = true;
}

int main(int argc, char *argv[])
{
    std::filesystem::create_directory("./test-results");

    if(argc > 1)
    {
        // run a test (wrapper for an external test runner)
        int i = 1;
        auto console = DMGCPU::Console::Auto;
        if(argc > 2)
        {
            // options
            std::string_view arg = argv[i];
            if(arg == "--cgb")
            {
                i++;
                console = DMGCPU::Console::CGB;
            }
            else if(arg == "--dmg")
            {
                i++;
                console = DMGCPU::Console::DMG;
            }
        }

        signal(SIGUSR1, handleSignal);
        runTest(argv[i], console);
        return 0;
    }
    return 0;
}