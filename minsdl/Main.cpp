#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include <SDL3/SDL.h>

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

static SDL_Texture *texture; // display texture

static const std::unordered_map<SDL_Keycode, int> dmgKeyMap {
    {SDLK_RIGHT,  1 << 0},
	{SDLK_LEFT,   1 << 1},
	{SDLK_UP,     1 << 2},
    {SDLK_DOWN,   1 << 3},

    {SDLK_Z,      1 << 4},
	{SDLK_X,      1 << 5},
	{SDLK_C,      1 << 6},
    {SDLK_V,      1 << 7},
};

// maybe swap dpad/buttons on DMG to share mappings?
static const std::unordered_map<SDL_Keycode, int> agbKeyMap {
    {SDLK_RIGHT,  1 << 4},
    {SDLK_LEFT,   1 << 5},
    {SDLK_UP,     1 << 6},
    {SDLK_DOWN,   1 << 7},

    {SDLK_Z,      1 << 0},
    {SDLK_X,      1 << 1},
    {SDLK_C,      1 << 2},
    {SDLK_V,      1 << 3},

    {SDLK_Q,      1 << 9},
    {SDLK_E,      1 << 8},
};

static void getROMBank(uint8_t bank, uint8_t *ptr)
{
    auto addr = bank * 0x4000;

    romFile.seekg(addr);
    romFile.read((char *)ptr, 0x4000);
}

static void audioCallback(void *userdata, SDL_AudioStream *stream, int additional_amount, int total_amount)
{
    // TODO: refactor core so we can push data directly
    if(isAGB)
    {
        auto &apu = agbCPU.getAPU();

        for(int i = 0; i < additional_amount; i++)
        {
            if(!apu.hasSample())
                break;

            int16_t v = apu.getSample();
            SDL_PutAudioStreamData(stream, &v, 2);
        }
    }
    else
    {
        auto &apu = dmgCPU.getAPU();

        for(int i = 0; i < additional_amount; i++)
        {
            if(!apu.getNumSamples())
                break;

            int16_t v = apu.getSample();
            SDL_PutAudioStreamData(stream, &v, 2);
        }
    }
}

static uint8_t *readSave(const std::string &savePath, size_t &saveSize)
{
    std::ifstream saveFile(savePath, std::ios::binary);

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

static void dmgVBlankCallback()
{
    SDL_UpdateTexture(texture, nullptr, screenData, 160 * 2);
}

static void pollEvents()
{
    auto &keyMap = isAGB ? agbKeyMap : dmgKeyMap;

    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_EVENT_KEY_DOWN:
            {
                auto it = keyMap.find(event.key.key);
                if(it != keyMap.end())
                    inputs |= it->second;
                break;
            }
            case SDL_EVENT_KEY_UP:
            {
                auto it = keyMap.find(event.key.key);
                if(it != keyMap.end())
                    inputs &= ~it->second;
                break;
            }

            case SDL_EVENT_QUIT:
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
    bool useBIOS = true;

    uint32_t timeToRun = 0;
    bool timeLimit = false;
    std::string romFilename;

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else if(arg == "--turbo")
            turbo = true;
        else if(arg == "--time" && i + 1 < argc)
        {
            timeLimit = true;
            timeToRun = std::stoi(argv[++i]) * 1000;
        }
        else if(arg == "--no-bios")
            useBIOS = false;
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
        basePath = tmp;

    romFilename = argv[i];

    std::string ext = romFilename.substr(romFilename.find_last_of('.'));
    isAGB = ext == ".gba";

    romFile.open(romFilename, std::ios::binary);

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
        if(useBIOS)
        {
            std::ifstream biosFile(basePath + "bios.gba", std::ios::binary);
            if(biosFile)
            {
                biosFile.read(reinterpret_cast<char *>(agbBIOSROM), sizeof(agbBIOSROM));

                mem.setBIOSROM(agbBIOSROM);
            }
            else
            {
                std::cerr << "bios.gba not found use --no-bios to run without\n";
                return 1;
            }
        }
        else
            std::cout << "BIOS emulation is unfinished and likely inaccurate!\n";
        
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
        dmgCPU.getDisplay().setVBlankCallback(dmgVBlankCallback);

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
    if(!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO))
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE);

    auto renderer = SDL_CreateRenderer(window, nullptr);
    if(!turbo)
        SDL_SetRenderVSync(renderer, 1);
    SDL_SetRenderLogicalPresentation(renderer, screenWidth, screenHeight, SDL_LOGICAL_PRESENTATION_INTEGER_SCALE);

    texture = SDL_CreateTexture(renderer, isAGB ? SDL_PIXELFORMAT_BGR565 : SDL_PIXELFORMAT_XBGR1555, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);
    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);

    // audio
    SDL_AudioSpec spec{};

    spec.freq = isAGB ? 32768 : 22050;
    spec.format = SDL_AUDIO_S16LE;
    spec.channels = 1;

    auto stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, audioCallback, nullptr);

    if(!stream)
    {
        std::cerr << "Failed to open audio: " << SDL_GetError() << "\n";
        quit = true;
    }

    if(!turbo)
        SDL_ResumeAudioDevice(SDL_GetAudioStreamDevice(stream));

    auto lastTick = SDL_GetTicks();
    auto startTime = SDL_GetTicks();

    auto checkTimeLimit = [timeLimit, &timeToRun]()
    {
        // fixed length benchmark
        if(timeLimit)
        {
            timeToRun -= 10;
            if(timeToRun == 0)
            {
                quit = true;
                return true;
            }
        }
        return false;
    };

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        if(isAGB)
        {
            agbCPU.setInputs(inputs);

            // run frames until there isn't any room left for audio
            while(true)
            {
                // 548-549 samples per update 
                if(agbCPU.getAPU().getNumSamples() > 2047 - 549)
                    break;

                if(turbo)
                {
                    agbCPU.run(10);

                    agbCPU.getAPU().update();
                    while(agbCPU.getAPU().getNumSamples())
                        agbCPU.getAPU().getSample();

                    if(now - lastTick >= 10 || checkTimeLimit())
                        break;

                    now = SDL_GetTicks();
                }
                else
                {
                    agbCPU.runFrame();
                    agbCPU.getAPU().update();
                    agbCPU.getDisplay().update();
                }
            }

            // should be at vblank if not turbo mode, otherwise we don't care
            SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        }
        else
        {
            if(turbo)
            {
                dmgCPU.setInputs(inputs);

                // push as fast as possible
                // avoid doing SDL stuff between updates
                while(now - lastTick < 10)
                {
                    dmgCPU.run(10);
                    dmgCPU.getAPU().update();

                    // discard audio
                    auto &apu = dmgCPU.getAPU();
                    while(apu.getNumSamples())
                        apu.getSample();

                    now = SDL_GetTicks();

                    if(checkTimeLimit())
                        break;
                }
            }
            else
            {
                dmgCPU.setInputs(inputs);
                dmgCPU.run(now - lastTick);
                dmgCPU.getAPU().update();
                dmgCPU.getDisplay().update();
            }

            // we'll update the texture in the vblank callback, so it's synced
        }

        lastTick = now;

        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
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
            std::ofstream saveFile(saveFilename, std::ios::out | std::ios::binary);
            saveFile.write(reinterpret_cast<char *>(agbCPU.getMem().getCartridgeSave()), size);

            if(!saveFile)
                std::cerr << "Failed to write " << saveFilename << "\n";
            else
                std::cout << "Wrote " << size << " bytes to " << saveFilename << "\n";
        }
    }
    else
    {
        std::ofstream saveFile(romFilename + ".ram", std::ios::out | std::ios::binary);
        saveFile.write(reinterpret_cast<char *>(dmgCPU.getMem().getCartridgeRAM()), dmgCPU.getMem().getCartridgeRAMSize());
    }

    if(timeLimit)
    {
        auto runTime = SDL_GetTicks() - startTime;
        printf("Ran for %ums\n", runTime);
    }

    SDL_DestroyAudioStream(stream);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
