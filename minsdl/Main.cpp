#include <fstream>
#include <iostream>
#include <unordered_map>

#include <SDL.h>

#include "DMGCPU.h"

static bool quit = false;

static DMGCPU cpu;

static uint8_t inputs = 0;
static uint16_t screenData[160 * 144];
static uint8_t romBankCache[0x4000];

static std::ifstream romFile;

static const std::unordered_map<SDL_Keycode, int> keyMap {
    {SDLK_RIGHT,  1 << 0},
	{SDLK_LEFT,   1 << 1},
	{SDLK_UP,     1 << 2},
    {SDLK_DOWN,   1 << 3},

    {SDLK_z,      1 << 4},
	{SDLK_x,      1 << 5},
	{SDLK_c,      1 << 6},
    {SDLK_v,      1 << 7},
};

static void getROMBank(uint8_t bank, uint8_t *ptr)
{
    auto addr = bank * 0x4000;

    romFile.seekg(addr);
    romFile.read((char *)ptr, 0x4000);
}

static void audioCallback(void *userdata, Uint8 *stream, int len)
{
    while(!quit && cpu.getAPU().getNumSamples() < 512); // ohno

    auto ptr = reinterpret_cast<int16_t *>(stream);
    for(int i = 0; i < len / 2; i++)
        *ptr++ = cpu.getAPU().getSample();
}

static void pollEvents()
{
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

    if(argc < 2)
    {
        std::cerr << "No ROM specified!\n";
        return 1;
    }

    romFilename = argv[1];

    romFile.open(romFilename);

    if(!romFile)
    {
        std::cerr << "Failed to open ROM \"" << romFilename << "\"\n";
        return 1;
    }

    // emu init
    cpu.getDisplay().setFramebuffer(screenData);

    auto &mem = cpu.getMem();
    mem.setROMBankCallback(getROMBank);
    mem.addROMCache(romBankCache, sizeof(romBankCache));

    cpu.reset();

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

    SDL_PauseAudioDevice(dev, 0);

    auto lastTick = SDL_GetTicks();

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        cpu.setInputs(inputs);
        cpu.run(now - lastTick);
        cpu.getAPU().update();
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