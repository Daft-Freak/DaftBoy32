#include <algorithm>
#include <cstring>

#include "DMGDisplay.h"

#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

enum SpriteFlags
{
    // 0-2 is colour palette
    Sprite_Bank       = (1 << 3),
    Sprite_Palette    = (1 << 4), // non-colour
    Sprite_XFlip      = (1 << 5),
    Sprite_YFlip      = (1 << 6),
    Sprite_BGPriority = (1 << 7),
};

enum TileFlags
{
    // 0-2 is palette
    Tile_Bank       = (1 << 3),
    Tile_XFlip      = (1 << 5),
    Tile_YFlip      = (1 << 6),
    Tile_BGPriority = (1 << 7)
};

DMGDisplay::DMGDisplay(DMGCPU &cpu) : cpu(cpu), mem(cpu.getMem())
{
    memset(bgPalette, 0xFF, sizeof(bgPalette));
    memset(objPalette, 0xFF, sizeof(objPalette));
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

    const uint16_t colMap[]{0xFFFF, 0x6739, 0x39CE, 0};

    switch(addr & 0xFF)
    {
        case IO_LY:
            y = 0;
            return true;

        // grey palettes
        case IO_BGP:
        {
            for(int i = 0; i < 4; i++)
                bgPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP0:
        {
            for(int i = 0; i < 4; i++)
                objPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP1:
        {
            for(int i = 0; i < 4; i++)
                objPalette[i + 4] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }

        // colour palettes
        case IO_BCPD:
        {
            auto bcps = mem.readIOReg(IO_BCPS);
            reinterpret_cast<uint8_t *>(bgPalette)[bcps & 0x3F] = data;

            // auto inc
            if(bcps & 0x80)
                mem.writeIOReg(IO_BCPS, ((bcps & 0x3F) + 1) | 0x80);

            return true;
        }

        case IO_OCPD:
        {
            auto ocps = mem.readIOReg(IO_OCPS);
            reinterpret_cast<uint8_t *>(objPalette)[ocps & 0x3F] = data;

            // auto inc
            if(ocps & 0x80)
                mem.writeIOReg(IO_OCPS, ((ocps & 0x3F) + 1) | 0x80);

            return true;
        }
    }
    
    return false;
}

void DMGDisplay::drawScanLine(int y)
{
    auto lcdc = mem.readIOReg(IO_LCDC);

    const bool isColour = cpu.getColourMode();

    auto vram = mem.getVRAM();

    auto tileDataPtr = (lcdc & LCDC_TileData8000) ? vram : vram + 0x800;
    auto bgMapPtr = (lcdc & LCDC_BGTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;
    auto winMapPtr = (lcdc & LCDC_WindowTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;
    auto spriteDataPtr = vram;

    const int tileDataSize = 16;
    const int screenSizeTiles = 32; // 32x32 tiles

    const int numSprites = 40;
    const int spriteHeight = (lcdc & LCDC_Sprite8x16) ? 16 : 8;

    uint8_t bgRaw[screenWidth]{0};

    // active scanline
    if(lcdc & LCDC_BGDisp)
    {
        uint8_t windowX = screenWidth, windowY = 0;
        bool yIsWin = false;

        int x = 0;
        auto out = screenData + y * screenWidth;
        auto rawOut = bgRaw;

        auto copyTiles = [this, &x, y, lcdc, isColour, tileDataPtr, &out, &rawOut](uint8_t *mapPtr, int xLimit, int offsetX, int offsetY)
        {
            while(x < xLimit)
            {
                uint8_t tx = x + offsetX;
                uint8_t ty = y + offsetY;
                int tileId = (tx / 8) + (ty / 8) * screenSizeTiles;
                auto mapAttrs = mapPtr[tileId + 0x2000]; // GBC, bank 1

                // tile id is signed if addr == 0x8800
                tileId = (lcdc & LCDC_TileData8000) ? mapPtr[tileId] : (int8_t)mapPtr[tileId] + 128;

                // TODO: GBC h/v flip, bg priority

                auto tileAddr = tileId * tileDataSize;

                if(mapAttrs & Tile_Bank)
                    tileAddr += 0x2000;

                // get the two tile data bytes for this line
                uint8_t d1 = tileDataPtr[tileAddr + (ty & 7) * 2];
                uint8_t d2 = tileDataPtr[tileAddr + (ty & 7) * 2 + 1];

                // skip bits
                d1 <<= (tx & 7);
                d2 <<= (tx & 7);

                // palette
                const auto bgPal = bgPalette + (isColour ? (mapAttrs & 0x7) * 4 : 0);

                // attempt to copy as much of the tile as possible
                const int limit = std::min(x + 8 - (tx & 7), xLimit);

                for(; x < limit; x++, d1 <<= 1, d2 <<= 1)
                {
                    int palIndex = ((d1 & 0x80) >> 7) | ((d2 & 0x80) >> 6);

                    *(rawOut++) = palIndex;
                    *(out++) = bgPal[palIndex];
                }
            }
        };

        if(lcdc & LCDC_WindowEnable)
        {
            windowY = mem.readIOReg(IO_WY);
            if(y >= windowY)
            {
                yIsWin = true;
                windowX = mem.readIOReg(IO_WX) - 7;
            }
        }

        // background
        if(windowX > 0)
        {
            auto scrollX = mem.readIOReg(IO_SCX);
            auto scrollY = mem.readIOReg(IO_SCY);

            copyTiles(bgMapPtr, windowX, scrollX, scrollY);
        }

        // window
        if(yIsWin)
            copyTiles(winMapPtr, screenWidth, -windowX, -windowY);
    }

    if(lcdc & LCDC_OBJDisp)
    {
        // sprites
        uint16_t addr = 0xFE00;
        auto oam = mem.mapAddress(addr);

        // 10 sprites per line limit
        uint8_t lineSprites[10];
        int numLineSprites = 0;

        for(int i = 0; i < numSprites && numLineSprites < 10; i++)
        {
            const int spriteY = oam[i * 4] - 16;

            // not on this line
            if(y < spriteY || y >= spriteY + spriteHeight)
                continue;

            lineSprites[numLineSprites++] = i;
        }

        for(int i = 0; i < numLineSprites; i++)
        {
            const int spriteId = lineSprites[i];
            const int spriteY = oam[spriteId * 4] - 16;
            const int spriteX = oam[spriteId * 4 + 1] - 8;
            int tileId = oam[spriteId * 4 + 2];
            const int attrs = oam[spriteId * 4 + 3];

            // tall sprites
            if(spriteHeight != 8)
                tileId &= 0xFE;

            const uint16_t *spritePal;
            if(isColour)
                spritePal = objPalette + (attrs & 0x7) * 4;
            else // OBP0 or 1
                spritePal = objPalette + ((attrs & Sprite_Palette) ? 4 : 0);

            // TODO: priority

            int ty = y - spriteY;

            if(attrs & Sprite_YFlip)
                ty = (spriteHeight - 1) - ty;

            auto tileAddr = tileId * tileDataSize;

            if(attrs & Sprite_Bank)
                tileAddr += 0x2000;

            // get the two tile data bytes for this line
            uint8_t d1 = spriteDataPtr[tileAddr + ty * 2];
            uint8_t d2 = spriteDataPtr[tileAddr + ty * 2 + 1];

            int x = std::max(0, -spriteX);
            int end = std::min(8, screenWidth - spriteX);

            auto out = screenData + (x + spriteX) + y * screenWidth;
            auto bgIn = bgRaw + x + spriteX;
            for(; x < end; x++, out++, bgIn++)
            {
                // background has priority
                if((attrs & Sprite_BGPriority) && *bgIn)
                    continue;

                int xShift = x;
                if(!(attrs & Sprite_XFlip))
                    xShift = 7 - xShift;

                int palIndex = ((d1 >> xShift) & 1) | (((d2 >> xShift) & 1) << 1);

                if(!palIndex)
                    continue;

                *out = spritePal[palIndex];
            }
        }
    }
}