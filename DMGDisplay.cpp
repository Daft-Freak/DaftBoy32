#include "DMGDisplay.h"

#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

enum SpriteFlags
{
    Sprite_Palette    = (1 << 4),
    Sprite_XFlip      = (1 << 5),
    Sprite_YFlip      = (1 << 6),
    Sprite_BGPriority = (1 << 7),
};

DMGDisplay::DMGDisplay(DMGCPU &cpu) : cpu(cpu), mem(cpu.getMem())
{
}

void DMGDisplay::update(int cycles)
{
    auto ioRegs = mem.getIORegs();
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

    if(y == screenHeight)
        cpu.flagInterrupt(Int_VBlank);
    else if(ioRegs[IO_LY] > 153)
        y = ioRegs[IO_LY] = 0; // end vblank
    
    if(y < screenHeight)
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
            for(int x = 0; x < screenWidth; x++)
            {
                int tileId = (x / 8) + tileY * screenSizeTiles;

                // tile id is signed if addr == 0x8800
                tileId = tileDataAddr == 0x8800 ? (int8_t)mem.read(bgMapAddr + tileId) + 128 : mem.read(bgMapAddr + tileId);

                // TODO x/y pos

                auto tileAddr = tileDataAddr + tileId * tileDataSize;

                // get the two tile data bytes for this line
                uint8_t d1 = mem.read(tileAddr + (y % 8) * 2);
                uint8_t d2 = mem.read(tileAddr + (y % 8) * 2 + 1);

                //int xBit = 1 << (7 - (x % 8));
                int xShift = 7 - (x % 8);
                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                int col = (bgPal >> (2 * palIndex)) & 0x3;

                screenData[x + y * screenWidth] = colMap[col];
            }
        }

        if(lcdc & LCDC_OBJDisp)
        {
            // sprites

            for(int i = 0; i < numSprites; i++)
            {
                const int spriteX = mem.read(0xFE00/*OAM*/ + i * 4 + 1) - 8;
                const int spriteY = mem.read(0xFE00/*OAM*/ + i * 4) - 16;
                const int tileId = mem.read(0xFE00/*OAM*/ + i * 4 + 2);
                const int attrs = mem.read(0xFE00/*OAM*/ + i * 4 + 3);

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
                uint8_t d1 = mem.read(tileAddr + ty * 2);
                uint8_t d2 = mem.read(tileAddr + ty * 2 + 1);

                for(int x = 0; x < 8; x++)
                {
                    int xShift = 7 - (x % 8); // TODO: flip
                    int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                    int col = (spritePal >> (2 * palIndex)) & 0x3;
                    screenData[(x + spriteX) + y * screenWidth] = colMap[col];
                }
            }
        }
    }
}
