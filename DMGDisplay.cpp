#include <algorithm>

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
    bool displayEnabled = mem.readIOReg(IO_LCDC) & LCDC_DisplayEnable;

    if(!displayEnabled)
    {
        // TODO: set mode for off
        return;
    }

    remainingScanlineCycles -= cycles;

    int newMode;
    const int readTime = (168 + 291) / 2;

    if(y >= screenHeight)
        newMode = 1; // vblank
    else
    {
        // 80 cycles - mode 2 / oam search
        if(remainingScanlineCycles >= scanlineCycles - 80)
            newMode = 2;
        // 168-291 cycles - mode 3 / reading oam/vram
        else if(remainingScanlineCycles >= scanlineCycles - 80 - readTime)
            newMode = 3;
        else // hblank
            newMode = 0;
    }

    if(newMode != statMode)
    {
        auto stat = mem.readIOReg(IO_STAT);
        if(newMode == 2 && (stat & STAT_OAMInt))
            cpu.flagInterrupt(Int_LCDStat);
        else if(newMode == 0 && (stat & STAT_HBlankInt))
            cpu.flagInterrupt(Int_LCDStat);
    }

    statMode = newMode;

    if(remainingScanlineCycles > 0)
        return;

    // next scanline
    remainingScanlineCycles += scanlineCycles;

    if(y < screenHeight)
        drawScanLine(y);

    y++;

    // coincidince interrupt
    auto stat = mem.readIOReg(IO_STAT);

    if(stat & STAT_CoincidenceInt)
    {
        if((stat & STAT_Coincidence) && y == mem.readIOReg(IO_LYC))
            cpu.flagInterrupt(Int_LCDStat);
        else if(!(stat & STAT_Coincidence) && y - 1 == mem.readIOReg(IO_LYC)) // not-equal, trigger on change from equal
            cpu.flagInterrupt(Int_LCDStat);
    }

    if(y == screenHeight)
    {
        cpu.flagInterrupt(Int_VBlank);
        statMode = 1;
        if(mem.readIOReg(IO_STAT) & STAT_VBlankInt)
            cpu.flagInterrupt(Int_LCDStat);
    }
    else if(y > 153)
    {
        y = 0; // end vblank
        statMode = 2; // oam search

        // TODO: slightly duplicated
        if(mem.readIOReg(IO_STAT) & STAT_OAMInt)
            cpu.flagInterrupt(Int_LCDStat);
    }
}

uint8_t DMGDisplay::readReg(uint16_t addr, uint8_t val)
{
    if(addr < 0xFF00)
        return val;

    switch(addr & 0xFF)
    {
        case IO_STAT:
            return (val & 0xFC) | statMode;
        case IO_LY:
            return y;
    }

    return val;
}

bool DMGDisplay::writeReg(uint16_t addr, uint8_t data)
{
    if(addr < 0xFF00)
        return false;

    switch(addr & 0xFF)
    {
        case IO_LY:
            y = 0;
            return true;
    }
    
    return false;
}

void DMGDisplay::drawScanLine(int y)
{
    auto lcdc = mem.readIOReg(IO_LCDC);

    const uint8_t colMap[]{0xFF, 0xCC, 0x77, 0};

    uint16_t addr = (lcdc & LCDC_TileData8000) ? 0x8000 : 0x8800;
    auto tileDataPtr = mem.mapAddress(addr) + addr;
    addr = (lcdc & LCDC_BGTileMap9C00) ? 0x9C00 : 0x9800;
    auto bgMapPtr = mem.mapAddress(addr) + addr;
    addr = (lcdc & LCDC_WindowTileMap9C00) ? 0x9C00 : 0x9800;
    auto winMapPtr = mem.mapAddress(addr) + addr;

    addr = 0x8000;
    auto spriteDataPtr = mem.mapAddress(addr) + addr;

    const int tileDataSize = 16;
    const int screenSizeTiles = 32; // 32x32 tiles
    const int bgPal = mem.readIOReg(IO_BGP);

    const int numSprites = 40;

    uint8_t bgRaw[screenWidth]{0};

    auto windowY = mem.readIOReg(IO_WY);
    bool yIsWin = (lcdc & LCDC_WindowEnable) && y >= windowY;

    // active scanline
    if(lcdc & LCDC_BGDisp)
    {
        auto windowX = (lcdc & LCDC_WindowEnable) ? mem.readIOReg(IO_WX) - 7 : screenWidth;
        auto scrollX = mem.readIOReg(IO_SCX);
        auto scrollY = mem.readIOReg(IO_SCY);

        // bg/window
        for(int x = 0; x < screenWidth;)
        {
            int tileId;
            uint8_t tx;
            uint8_t ty;

            if(yIsWin && x >= windowX)
            {
                tx = x - windowX;
                ty = y - windowY;
                tileId = (tx / 8) + (ty / 8) * screenSizeTiles;
                tileId = (lcdc & LCDC_TileData8000) ? winMapPtr[tileId] : (int8_t)winMapPtr[tileId] + 128;
            }
            else
            {
                tx = x + scrollX;
                ty = y + scrollY;
                tileId = (tx / 8) + (ty / 8) * screenSizeTiles;

                // tile id is signed if addr == 0x8800
                tileId = (lcdc & LCDC_TileData8000) ? bgMapPtr[tileId] : (int8_t)bgMapPtr[tileId] + 128;
            }

            auto tileAddr = tileId * tileDataSize;

            // get the two tile data bytes for this line
            uint8_t d1 = tileDataPtr[tileAddr + (ty & 7) * 2];
            uint8_t d2 = tileDataPtr[tileAddr + (ty & 7) * 2 + 1];

            int xShift = 7 - (tx & 7);

            // attempt to copy as much of the tile as possible
            const int limit = x >= windowX ? screenWidth : windowX;
            for(; xShift >= 0 && x < limit; x++, xShift--)
            {
                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                bgRaw[x] = palIndex;

                int col = (bgPal >> (2 * palIndex)) & 0x3;
                screenData[x + y * screenWidth] = colMap[col];
            }
        }
    }

    if(lcdc & LCDC_OBJDisp)
    {
        // sprites
        addr = 0xFE00;
        auto oam = mem.mapAddress(addr);

        // 10 sprites per line limit
        uint8_t lineSprites[10];
        int numLineSprites = 0;

        for(int i = 0; i < numSprites && numLineSprites < 10; i++)
        {
            const int spriteY = oam[i * 4] - 16;

            // not on this line
            if(y < spriteY || y >= spriteY + 8)
                continue;

            lineSprites[numLineSprites++] = i;
        }

        for(int i = 0; i < numLineSprites; i++)
        {
            const int spriteId = lineSprites[i];
            const int spriteY = oam[spriteId * 4] - 16;
            const int spriteX = oam[spriteId * 4 + 1] - 8;
            const int tileId = oam[spriteId * 4 + 2];
            const int attrs = oam[spriteId * 4 + 3];

            const int spritePal = mem.readIOReg((attrs & Sprite_Palette) ? IO_OBP1 : IO_OBP0);

            // TODO: 8x16
            // TODO: priority

            int ty = y - spriteY;

            if(attrs & Sprite_YFlip)
                ty = 7 - ty;

            auto tileAddr = tileId * tileDataSize;

            // get the two tile data bytes for this line
            uint8_t d1 = spriteDataPtr[tileAddr + ty * 2];
            uint8_t d2 = spriteDataPtr[tileAddr + ty * 2 + 1];

            int end = std::min(8, screenWidth - spriteX);
            for(int x = std::max(0, -spriteX); x < end; x++)
            {
                // background has priority
                if((attrs & Sprite_BGPriority) && bgRaw[x + spriteX])
                    continue;

                int xShift = x;
                if(!(attrs & Sprite_XFlip))
                    xShift = 7 - xShift;

                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                if(!palIndex)
                    continue;

                int col = (spritePal >> (2 * palIndex)) & 0x3;
                screenData[(x + spriteX) + y * screenWidth] = colMap[col];
            }
        }
    }
}