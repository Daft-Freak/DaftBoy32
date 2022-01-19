#include <fstream>
#include <iostream>
#include <thread>
#include <unordered_map>

#include <SDL.h>

#include "AGBCPU.h"
#include "DMGCPU.h"

static bool quit = false;
static bool turbo = false;

static bool isAGB = false;

static DMGCPU dmgCPU;
static AGBCPU agbCPU;

static uint16_t inputs = 0;
static uint16_t screenData[240 * 160];
static uint8_t romBankCache[0x4000];

static uint8_t agbBIOSROM[0x4000];

static std::ifstream romFile;

static const std::unordered_map<SDL_Keycode, int> dmgKeyMap {
    {SDLK_RIGHT,  1 << 0},
	{SDLK_LEFT,   1 << 1},
	{SDLK_UP,     1 << 2},
    {SDLK_DOWN,   1 << 3},

    {SDLK_z,      1 << 4},
	{SDLK_x,      1 << 5},
	{SDLK_c,      1 << 6},
    {SDLK_v,      1 << 7},
};

// maybe swap dpad/buttons on DMG to share mappings?
static const std::unordered_map<SDL_Keycode, int> agbKeyMap {
    {SDLK_RIGHT,  1 << 4},
    {SDLK_LEFT,   1 << 5},
    {SDLK_UP,     1 << 6},
    {SDLK_DOWN,   1 << 7},

    {SDLK_z,      1 << 0},
    {SDLK_x,      1 << 1},
    {SDLK_c,      1 << 2},
    {SDLK_v,      1 << 3},

    {SDLK_q,      1 << 9},
    {SDLK_e,      1 << 8},
};

static void getROMBank(uint8_t bank, uint8_t *ptr)
{
    auto addr = bank * 0x4000;

    romFile.seekg(addr);
    romFile.read((char *)ptr, 0x4000);
}

static void audioCallback(void *userdata, Uint8 *stream, int len)
{
    if(isAGB)
    {
        auto &apu = agbCPU.getAPU();

        auto ptr = reinterpret_cast<int16_t *>(stream);
        for(int i = 0; i < len / 2; i++)
        {
            while(!quit && !apu.hasSample())
                std::this_thread::yield();

            *ptr++ = apu.getSample();
        }
    }
    else
    {
        auto &apu = dmgCPU.getAPU();

        auto ptr = reinterpret_cast<int16_t *>(stream);
        for(int i = 0; i < len / 2; i++)
        {
            while(!quit && !apu.getNumSamples())
                std::this_thread::yield();

            *ptr++ = apu.getSample();
        }
    }
}

static uint8_t *readSave(const std::string &savePath, size_t &saveSize)
{
    std::ifstream saveFile(savePath);

    if(!saveFile)
    {
        std::cout << "Could not find " << savePath << ", no save loaded.\n";
        return nullptr;
    }

    saveFile.seekg(0, std::ios::end);
    saveSize = saveFile.tellg();
    saveFile.seekg(0);

    auto saveData = new uint8_t[saveSize];
    saveFile.read(reinterpret_cast<char *>(saveData), saveSize);
    std::cout << "Read " << saveSize << " bytes from " << savePath << "\n";

    return saveData;
}

static void pollEvents()
{
    auto &keyMap = isAGB ? agbKeyMap : dmgKeyMap;

    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_KEYDOWN:
            {
                auto it = keyMap.find(event.key.keysym.sym);
                if(it != keyMap.end())
                    inputs |= it->second;
                break;
            }
            case SDL_KEYUP:
            {
                auto it = keyMap.find(event.key.keysym.sym);
                if(it != keyMap.end())
                    inputs &= ~it->second;
                break;
            }

            case SDL_QUIT:
                quit = true;
                break;
        }
    }
}

int main(int argc, char *argv[])
{
    int screenWidth = 160;
    int screenHeight = 144;
    int screenScale = 5;

    std::string romFilename;

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else if(arg == "--turbo")
            turbo = true;
        else
            break;
    }

    if(i == argc)
    {
        std::cerr << "No ROM specified!\n";
        return 1;
    }

    // get base path
    std::string basePath;
    auto tmp = SDL_GetBasePath();
    if(tmp)
    {
        basePath = tmp;
        SDL_free(tmp);
    }

    romFilename = argv[i];

    std::string ext = romFilename.substr(romFilename.find_last_of('.'));
    isAGB = ext == ".gba";

    romFile.open(romFilename);

    if(!romFile)
    {
        std::cerr << "Failed to open ROM \"" << romFilename << "\"\n";
        return 1;
    }

    // emu init
    if(isAGB)
    {
        screenWidth = 240;
        screenHeight = 160;

        auto &mem = agbCPU.getMem();

        // need the bios for now
        std::ifstream biosFile(basePath + "bios.gba");
        if(biosFile)
        {
            biosFile.read(reinterpret_cast<char *>(agbBIOSROM), sizeof(agbBIOSROM));

            mem.setBIOSROM(agbBIOSROM);
        }
        else
            std::cout << "BIOS emulation is unfinished and likely inaccurate!";
        
        // read the entire ROM, this one doesn't have the load callback/caching setup
        romFile.seekg(0, std::ios::end);
        auto romSize = romFile.tellg();
        romFile.seekg(0);

        auto romData = new uint8_t[romSize]; // FIXME: leaky
        romFile.read(reinterpret_cast<char *>(romData), romSize);

        agbCPU.getDisplay().setFramebuffer(screenData);

        mem.setCartROM(romData, romSize);

        agbCPU.reset();

        // attempt to read save
        if(!turbo)
        {
            size_t size;
            auto saveData = readSave(romFilename.substr(0, romFilename.length() - 3) + "sav", size);

            if(saveData)
            {
                mem.loadCartridgeSave(saveData, size);
                delete[] saveData;
            }
        }
    }
    else
    {
        dmgCPU.getDisplay().setFramebuffer(screenData);

        auto &mem = dmgCPU.getMem();
        mem.setROMBankCallback(getROMBank);
        mem.addROMCache(romBankCache, sizeof(romBankCache));

        dmgCPU.reset();

        // attempt to read save
        size_t size;
        auto saveData = readSave(romFilename + ".ram", size);
    
        if(saveData)
        {
            mem.loadCartridgeRAM(saveData, size);
            delete[] saveData;
        }
    }

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                   screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE);

    auto renderer = SDL_CreateRenderer(window, -1, turbo ? 0 : SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

    auto texture = SDL_CreateTexture(renderer, isAGB ? SDL_PIXELFORMAT_BGR565 : SDL_PIXELFORMAT_BGR555, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

    // audio
    SDL_AudioSpec spec{};

    spec.freq = isAGB ? 32768 : 22050;
    spec.format = AUDIO_S16;
    spec.channels = 1;
    spec.samples = 512;
    spec.callback = audioCallback;

    auto dev = SDL_OpenAudioDevice(nullptr, false, &spec, nullptr, 0);

    if(!dev)
    {
        std::cerr << "Failed to open audio: " << SDL_GetError() << "\n";
        quit = true;
    }

    if(!turbo)
        SDL_PauseAudioDevice(dev, 0);

    auto lastTick = SDL_GetTicks();

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        if(isAGB)
        {
            // clamp updates as this can still be really slow
            auto time = now - lastTick;
            if(time > 30)
            {
                std::cout << "slow " << time << "\n";
                time = 30;
            }

            agbCPU.setInputs(inputs);
            agbCPU.run(turbo ? 10 : time);
            agbCPU.getAPU().update();

            if(turbo)
            {
                while(agbCPU.getAPU().getNumSamples())
                    agbCPU.getAPU().getSample();
            }
        }
        else
        {
            dmgCPU.setInputs(inputs);
            dmgCPU.run(turbo ? 10 : now - lastTick);
            dmgCPU.getAPU().update();
            if(turbo)
            {
                while(dmgCPU.getAPU().getNumSamples())
                    dmgCPU.getAPU().getSample();
            }
        }

        lastTick = now;
    
        // TODO: sync
        SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    // write save
    if(isAGB)
    {
        int size = 0;
        switch(agbCPU.getMem().getCartridgeSaveType())
        {
            case AGBMemory::SaveType::Unknown:
                break;
            case AGBMemory::SaveType::EEPROM_512:
                size = 512;
                break;
            case AGBMemory::SaveType::EEPROM_8K:
                size = 8 * 1024;
                break;
            case AGBMemory::SaveType::RAM:
                size = 32 * 1024;
                break;
            case AGBMemory::SaveType::Flash:
                size = 128 * 1024; // TODO: possibly 64k
                break;
        }
        if(size && !turbo)
        {
            auto saveFilename = romFilename.substr(0, romFilename.length() - 3) + "sav";
            std::ofstream saveFile(saveFilename);
            saveFile.write(reinterpret_cast<char *>(agbCPU.getMem().getCartridgeSave()), size);

            if(!saveFile)
                std::cerr << "Failed to write " << saveFilename << "\n";
            else
                std::cout << "Wrote " << size << " bytes to " << saveFilename << "\n";
        }
    }
    else
    {
        std::ofstream saveFile(romFilename + ".ram");
        saveFile.write(reinterpret_cast<char *>(dmgCPU.getMem().getCartridgeRAM()), dmgCPU.getMem().getCartridgeRAMSize());
    }

    SDL_CloseAudioDevice(dev);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
