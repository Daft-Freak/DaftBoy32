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
    const uint8_t lcdc = mem.readIOReg(IO_LCDC);
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
    remainingScanlineCycles += scanlineCycles;
    int y = mem.readIOReg(IO_LY) + 1;

    if(y == screenHeight)
        cpu.flagInterrupt(Int_VBlank);
    else if(y > 153)
        y = 0; // end vblank

    mem.writeIOReg(IO_LY, y);
    
    if(y < screenHeight)
        drawScanLine(y);
}

void DMGDisplay::drawScanLine(int y)
{
    auto lcdc = mem.readIOReg(IO_LCDC);

    const uint8_t colMap[]{0xFF, 0xCC, 0x77, 0};

    uint16_t addr = (lcdc & LCDC_TileData8000) ? 0x8000 : 0x8800;
    auto tileDataPtr = mem.mapAddress(addr) + addr;
    addr = (lcdc & LCDC_BGTileMap9C00) ? 0x9C00 : 0x9800;
    auto bgMapPtr = mem.mapAddress(addr) + addr;

    addr = 0x8000;
    auto spriteDataPtr = mem.mapAddress(addr) + addr;

    const int tileDataSize = 16;
    const int screenSizeTiles = 32; // 32x32 tiles
    const int bgPal = mem.readIOReg(IO_BGP);

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
            tileId = (lcdc & LCDC_TileData8000) ? bgMapPtr[tileId] : (int8_t)bgMapPtr[tileId] + 128;

            // TODO x/y pos

            auto tileAddr = tileId * tileDataSize;

            // get the two tile data bytes for this line
            uint8_t d1 = tileDataPtr[tileAddr + (y % 8) * 2];
            uint8_t d2 = tileDataPtr[tileAddr + (y % 8) * 2 + 1];

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
        addr = 0xFE00;
        auto oam = mem.mapAddress(addr);

        for(int i = 0; i < numSprites; i++)
        {
            const int spriteX = oam[i * 4 + 1] - 8;
            const int spriteY = oam[i * 4] - 16;
            const int tileId = oam[i * 4 + 2];
            const int attrs = oam[i * 4 + 3];

            const int spritePal = mem.readIOReg((attrs & Sprite_Palette) ? IO_OBP1 : IO_OBP0);

            // TODO: 8x16
            // TODO: priority

            // not on this line
            if(y < spriteY || y >= spriteY + 8)
                continue;


            int ty = y - spriteY;

            //TODO: flip

            auto tileAddr = tileId * tileDataSize;

            // get the two tile data bytes for this line
            uint8_t d1 = spriteDataPtr[tileAddr + ty * 2];
            uint8_t d2 = spriteDataPtr[tileAddr + ty * 2 + 1];

            for(int x = 0; x < 8; x++)
            {
                if(x + spriteX < 0 || x + spriteX >= screenWidth)
                    continue;

                int xShift = 7 - (x % 8); // TODO: flip
                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                int col = (spritePal >> (2 * palIndex)) & 0x3;
                screenData[(x + spriteX) + y * screenWidth] = colMap[col];
            }
        }
    }
}