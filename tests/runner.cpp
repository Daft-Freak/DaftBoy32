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
static uint16_t screenData[160 * 144];
static uint8_t romBankCache[0x4000]; // need at least one bank

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
    for(int i = 0; i < 160 * 144; i++)
    {
        auto pixel = screenData[i];
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
    cpu->getDisplay().setFramebuffer(screenData);

    auto &mem = cpu->getMem();
    mem.setROMBankCallback(getROMBank);
    mem.addROMCache(romBankCache, sizeof(romBankCache));

    cpu->setConsole(console);

    cpu->reset();

    uint8_t rgbDisplay[160 * 144 * 3];

    unsigned int time = 0;
    bool result = false;

    while(!result)
    {
        // flush apu
        auto &apu = cpu->getAPU();
        apu.update();
        while(apu.getNumSamples())
            apu.getSample();
        cpu->run(10);

        time += 10;

        bool convertDisplay = takeScreenshot || cpu->getBreakpointTriggered();

        if(convertDisplay)
        {
            cpu->getDisplay().update();
            screenToRGB(cpu->getDisplay(), rgbDisplay);
        }

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

static void replayLog(const std::string &logFilename, DMGCPU::Console console = DMGCPU::Console::Auto, bool record = false)
{
    // find/open ROM
    std::string rom, logPath;

    auto index = logFilename.find_last_of("\\/");
    if(index != std::string::npos)
        logPath = logFilename.substr(0, index + 1);


    std::ifstream logFile(logFilename);

    if(!logFile)
    {
        std::cerr << "Failed to open log " << logFilename << "!\n";
        return;
    }

    // get rom from first line
    std::getline(logFile, rom);

    romFile.open(logPath + rom);

    if(!romFile)
    {
        std::cerr << "Failed to load " << logPath + rom << "\n";
        return;
    }

    romFile.seekg(0, std::ios::end);
    romSize = romFile.tellg();
    romFile.seekg(0, std::ios::beg);

    // clean instance
    cpu = new DMGCPU;
    cpu->getDisplay().setFramebuffer(screenData);

    auto &mem = cpu->getMem();
    mem.setROMBankCallback(getROMBank);
    mem.addROMCache(romBankCache, sizeof(romBankCache));

    cpu->setConsole(console);

    cpu->reset();

    uint8_t rgbDisplay[160 * 144 * 3];

    unsigned int tick = 0, imageIndex = 0;
    unsigned int nextInputTick;
    int nextInputValue;

    logFile >> nextInputTick;
    logFile >> nextInputValue;

    auto start = std::chrono::steady_clock::now();

    while(!logFile.eof())
    {
        // flush apu
        auto &apu = cpu->getAPU();
        apu.update();
        while(apu.getNumSamples())
            apu.getSample();

        bool didInput = false;
        if(tick == nextInputTick)
        {
            cpu->setInputs(nextInputValue);
            didInput = true;
            logFile >> nextInputTick;
            logFile >> nextInputValue;
        }

        cpu->run(10);

        if(record && didInput)
        {
            cpu->getDisplay().update();
            screenToRGB(cpu->getDisplay(), rgbDisplay);
            char name[10];
            snprintf(name, 10, "f%06i", imageIndex++);
            dumpImage(name, rgbDisplay);
        }

        tick++;
    }

    auto end = std::chrono::steady_clock::now();

    auto realTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Ran " << rom << " for " << (tick * 10) << "ms in " << realTime << "us\n";

    delete cpu;

    romFile.close();
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
        // options
        auto console = DMGCPU::Console::Auto;
        bool recordReplay = false;

        for(; i < argc; i++)
        {
            std::string_view arg = argv[i];
            if(arg == "--cgb")
                console = DMGCPU::Console::CGB;
            else if(arg == "--dmg")
                console = DMGCPU::Console::DMG;
            else if(arg == "--record")
                recordReplay = true;
            else
                break;
        }

        std::string filename = argv[i];
        std::string ext = filename.substr(filename.find_last_of('.'));

        if(ext == ".gb" || ext == ".gbc")
        {
            signal(SIGUSR1, handleSignal);
            runTest(filename, console);
        }
        else if(ext == ".log")
        {
            // replay from a log file
            // assume all remaining args are log files
            for(; i < argc; i++)
                replayLog(argv[i], console, recordReplay);
        }
        return 0;
    }
    return 0;
}