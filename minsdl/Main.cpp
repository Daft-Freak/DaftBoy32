#include <fstream>
#include <iostream>
#include <unordered_map>

#include <SDL.h>

#include "AGBCPU.h"
#include "DMGCPU.h"

static bool quit = false;

static bool isAGB = false;

static DMGCPU dmgCPU;
static AGBCPU agbCPU;

static uint16_t inputs = 0;
static uint16_t screenData[240 * 160];
static uint8_t romBankCache[0x4000];

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
};

static void getROMBank(uint8_t bank, uint8_t *ptr)
{
    auto addr = bank * 0x4000;

    romFile.seekg(addr);
    romFile.read((char *)ptr, 0x4000);
}

static void audioCallback(void *userdata, Uint8 *stream, int len)
{
    while(!quit && dmgCPU.getAPU().getNumSamples() < 512); // ohno

    auto ptr = reinterpret_cast<int16_t *>(stream);
    for(int i = 0; i < len / 2; i++)
        *ptr++ = dmgCPU.getAPU().getSample();
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
        else
            break;
    }

    if(argc < i)
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
        uint8_t biosROM[0x4000];

        std::ifstream biosFile(basePath + "bios.gba");
        biosFile.read(reinterpret_cast<char *>(biosROM), sizeof(biosROM));

        mem.setBIOSROM(biosROM);
        
        // read the entire ROM, this one doesn't have the load callback/caching setup
        romFile.seekg(0, std::ios::end);
        auto romSize = romFile.tellg();
        romFile.seekg(0);

        auto romData = new uint8_t[romSize]; // FIXME: leaky
        romFile.read(reinterpret_cast<char *>(romData), romSize);

        agbCPU.getDisplay().setFramebuffer(screenData);

        mem.setCartROM(romData, romSize);

        agbCPU.reset();
    }
    else
    {
        dmgCPU.getDisplay().setFramebuffer(screenData);

        auto &mem = dmgCPU.getMem();
        mem.setROMBankCallback(getROMBank);
        mem.addROMCache(romBankCache, sizeof(romBankCache));

        dmgCPU.reset();
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

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR555, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

    // audio
    SDL_AudioSpec spec{};

    spec.freq = 22050;
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

    if(!isAGB) // no audio yet
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
            agbCPU.run(time);
        }
        else
        {
            dmgCPU.setInputs(inputs);
            dmgCPU.run(now - lastTick);
            dmgCPU.getAPU().update();
        }

        lastTick = now;
    
        // TODO: sync
        SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_CloseAudioDevice(dev);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
