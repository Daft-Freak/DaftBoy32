#include <cstring>

#include "gameblit.hpp"
#include "assets.hpp"

#include "DMGCPU.h"
#include "DMGRegs.h"

//

DMGCPU cpu;

// tmp display

enum SpriteFlags
{
    Sprite_Palette    = (1 << 4),
    Sprite_XFlip      = (1 << 5),
    Sprite_YFlip      = (1 << 6),
    Sprite_BGPriority = (1 << 7),
};

const int scanlineCycles = 456;
int remainingScanlineCycles = scanlineCycles;
uint8_t gbScreen[160*144];

void onCyclesExeceuted(int cycles, uint8_t *ioRegs)
{
    // display
    const uint8_t lcdc = ioRegs[IO_LCDC];
    bool displayEnabled = lcdc & LCDC_DisplayEnable;

    if(!displayEnabled)
    {
        // TODO: set mode for off
        return;
    }

    // TODO: set mode/stat interrupt

    remainingScanlineCycles -= cycles;

    if(remainingScanlineCycles > 0)
        return;

    // next scanline
    ioRegs[IO_LY]++;
    remainingScanlineCycles += scanlineCycles;
    int y = ioRegs[IO_LY];

    if(y == 144)
        cpu.flagInterrupt(Int_VBlank);
    else if(ioRegs[IO_LY] > 153)
        y = ioRegs[IO_LY] = 0; // end vblank
    
    if(y < 144)
    {
        const uint8_t colMap[]{0xFF, 0xCC, 0x77, 0};

        auto tileDataAddr = (lcdc & LCDC_TileData8000) ? 0x8000 : 0x8800;
        auto bgMapAddr = (lcdc & LCDC_BGTileMap9C00) ? 0x9C00 : 0x9800;
        auto spriteDataAddr = 0x8000;

        const int tileDataSize = 16;
        const int screenSizeTiles = 32; // 32x32 tiles
        const int bgPal = ioRegs[IO_BGP];

        const int numSprites = 40;

        int tileY = y / 8;

        // active scanline
        if(lcdc & LCDC_BGDisp)
        {
            // bg
            // TODO: window
            for(int x = 0; x < 160; x++)
            {
                int tileId = (x / 8) + tileY * screenSizeTiles;

                // tile id is signed if addr == 0x8800
                tileId = tileDataAddr == 0x8800 ? (int8_t)cpu.readMem(bgMapAddr + tileId) + 128 : cpu.readMem(bgMapAddr + tileId);

                // TODO x/y pos

                auto tileAddr = tileDataAddr + tileId * tileDataSize;

                // get the two tile data bytes for this line
                uint8_t d1 = cpu.readMem(tileAddr + (y % 8) * 2);
                uint8_t d2 = cpu.readMem(tileAddr + (y % 8) * 2 + 1);

                //int xBit = 1 << (7 - (x % 8));
                int xShift = 7 - (x % 8);
                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                int col = (bgPal >> (2 * palIndex)) & 0x3;

                gbScreen[x + y * 160] = colMap[col];
            }
        }

        if(lcdc & LCDC_OBJDisp)
        {
            // sprites

            for(int i = 0; i < numSprites; i++)
            {
                const int spriteX = cpu.readMem(0xFE00/*OAM*/ + i * 4 + 1) - 8;
                const int spriteY = cpu.readMem(0xFE00/*OAM*/ + i * 4) - 16;
                const int tileId = cpu.readMem(0xFE00/*OAM*/ + i * 4 + 2);
                const int attrs = cpu.readMem(0xFE00/*OAM*/ + i * 4 + 3);

                const int spritePal = (attrs & Sprite_Palette) ? ioRegs[IO_OBP1] : ioRegs[IO_OBP0];

                // TODO: 8x16
                // TODO: priority

                // not on this line
                if(y < spriteY || y >= spriteY + 8)
                    continue;


                int ty = y - spriteY;

                //TODO: flip

                auto tileAddr = spriteDataAddr + tileId * tileDataSize;

                // get the two tile data bytes for this line
                uint8_t d1 = cpu.readMem(tileAddr + ty * 2);
                uint8_t d2 = cpu.readMem(tileAddr + ty * 2 + 1);

                for(int x = 0; x < 8; x++)
                {
                    int xShift = 7 - (x % 8); // TODO: flip
                    int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                    int col = (spritePal >> (2 * palIndex)) & 0x3;
                    gbScreen[(x + spriteX) + y * 160] = colMap[col];
                }
            }
        }
    }
}

void init()
{
    blit::set_screen_mode(blit::ScreenMode::hires);

    cpu.loadCartridge(test_rom, test_rom_length);
    cpu.setCycleCallback(onCyclesExeceuted);
    cpu.reset();
}

void render(uint32_t time_ms)
{
    blit::screen.pen = blit::Pen(20, 30, 40);
    blit::screen.clear();

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

void update(uint32_t time_ms)
{
    static uint32_t lastButtonState = 0;

    bool turbo = false;
    auto start = blit::now();

    // translate inputs
    uint8_t inputs = (blit::buttons & 0x7C) | // UP/DOWN/A/B match, select -> X
                     ((blit::buttons & blit::Button::DPAD_RIGHT) >> 1) |
                     ((blit::buttons & blit::Button::DPAD_LEFT) << 1) |
                     ((blit::buttons & blit::Button::HOME) >> 2); // start -> home
    cpu.setInputs(inputs);

    cpu.run(10);

    // SPEEEEEEEED
    while(turbo && blit::now() - start < 9)
        cpu.run(1);

    lastButtonState = blit::buttons;
}
