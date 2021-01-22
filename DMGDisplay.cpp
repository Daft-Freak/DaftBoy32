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

// reverses the bits in each byte, but not the bytes
static uint16_t reverseBitsPerByte(uint16_t v)
{
    v = (((v & 0xAAAA) >> 1) | ((v & 0x5555) << 1));
    v = (((v & 0xCCCC) >> 2) | ((v & 0x3333) << 2));
    v = (((v & 0xF0F0) >> 4) | ((v & 0x0F0F) << 4));
    return v;
}

DMGDisplay::DMGDisplay(DMGCPU &cpu) : cpu(cpu), mem(cpu.getMem())
{
    memset(bgPalette, 0xFF, sizeof(bgPalette));
    memset(objPalette, 0xFF, sizeof(objPalette)); // not initialised
}

void DMGDisplay::update(int cycles)
{
    bool displayEnabled = mem.readIOReg(IO_LCDC) & LCDC_DisplayEnable;

    if(!displayEnabled)
        return;

    remainingScanlineCycles -= cycles;

    // avg-ish time
    static const int readTime = (168 + 291) / 2;

    if(statMode > 1) // update STAT if not hblank/vblank
    {
        // 80 cycles - mode 2 / oam search
        if(remainingScanlineCycles >= scanlineCycles - 80)
        {} // transition handled below
        // 168-291 cycles - mode 3 / reading oam/vram
        else if(remainingScanlineCycles >= scanlineCycles - 80 - readTime)
            statMode = 3;
        else // hblank
        {
            if(statMode)
            {
                auto stat = mem.readIOReg(IO_STAT);
                if(stat & STAT_HBlankInt)
                    cpu.flagInterrupt(Int_LCDStat);
            }
            statMode = 0;
        }
    }

    if(remainingScanlineCycles > 0)
        return;

    // next scanline
    remainingScanlineCycles += scanlineCycles;

    if(y < screenHeight)
        drawScanLine(y);

    y++;

    // coincidince interrupt
    auto stat = mem.readIOReg(IO_STAT);

    if((stat & STAT_CoincidenceInt) && y == mem.readIOReg(IO_LYC))
        cpu.flagInterrupt(Int_LCDStat);

    if(y == screenHeight)
    {
        cpu.flagInterrupt(Int_VBlank);
        statMode = 1;
        if(stat & STAT_VBlankInt)
            cpu.flagInterrupt(Int_LCDStat);
    }
    else
    {
        if(y > 153)
        {
            y = windowY = 0; // end vblank

            // new scanline
            statMode = 2; // oam search

            if(stat & STAT_OAMInt)
                cpu.flagInterrupt(Int_LCDStat);
        }
    }
}

uint8_t DMGDisplay::readReg(uint16_t addr, uint8_t val)
{
    if(addr < 0xFF00)
        return val;

    switch(addr & 0xFF)
    {
        case IO_STAT:
            return (val & 0xF8) | (y == mem.readIOReg(IO_LYC) ? STAT_Coincidence : 0) | statMode | 0x80;
        case IO_LY:
            return y;
    }

    return val;
}

bool DMGDisplay::writeReg(uint16_t addr, uint8_t data)
{
    if(addr < 0xFF00)
        return false;

    const uint16_t colMap[]{0xFFFF, 0x56B5, 0x294A, 0};

    switch(addr & 0xFF)
    {
        case IO_LCDC:
        {
            if(!(data & LCDC_DisplayEnable))
            {
                // reset
                remainingScanlineCycles = scanlineCycles;
                statMode = 0;
                y = windowY = 0;
            }
            break;
        }
        case IO_LY:
            y = windowY = 0;
            return true;

        // grey palettes
        case IO_BGP:
        {
            if(cpu.getColourMode())
                break;

            for(int i = 0; i < 4; i++)
                bgPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP0:
        {
            if(cpu.getColourMode())
                break;

            for(int i = 0; i < 4; i++)
                objPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP1:
        {
            if(cpu.getColourMode())
                break;

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

    auto spriteDataPtr = vram;

    const int tileDataSize = 16;
    const int screenSizeTiles = 32; // 32x32 tiles

    const int numSprites = 40;
    const int spriteHeight = (lcdc & LCDC_Sprite8x16) ? 16 : 8;

    // contains palette index + a tile priority flag
    uint8_t bgRaw[screenWidth]{0};

    auto scanLine = screenData + y * screenWidth;

    // active scanline
    // this is reduced to a priority flag on GBC
    if(lcdc & LCDC_BGDisp || isColour)
    {
        auto tileDataPtr = (lcdc & LCDC_TileData8000) ? vram : vram + 0x800;
        auto bgMapPtr = (lcdc & LCDC_BGTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;
        auto winMapPtr = (lcdc & LCDC_WindowTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;

        int windowX = screenWidth;
        bool isWindow = false;

        int x = 0;
        auto out = scanLine;
        auto rawOut = bgRaw;

        auto copyTiles = [this, &x, lcdc, isColour, tileDataPtr, &out, &rawOut](uint8_t *mapPtr, int xLimit, int offsetX, uint8_t oy)
        {
            while(x < xLimit)
            {
                uint8_t ox = x + offsetX;

                int tileId = (ox / 8) + (oy / 8) * screenSizeTiles;
                auto mapAttrs = mapPtr[tileId + 0x2000]; // GBC, bank 1

                // tile id is signed if addr == 0x8800
                tileId = (lcdc & LCDC_TileData8000) ? mapPtr[tileId] : (int8_t)mapPtr[tileId] + 128;

                auto tileAddr = tileId * tileDataSize;

                if(mapAttrs & Tile_Bank)
                    tileAddr += 0x2000;

                int ty = (oy & 7);
                int tx = (ox & 7);

                if(mapAttrs & Tile_YFlip)
                    ty = 7 - ty;

                uint8_t tilePriority = (mapAttrs & Tile_BGPriority) ? 0x80 : 0;

                // get the two tile data bytes for this line
                //uint8_t d1 = tileDataPtr[tileAddr + ty * 2];
                //uint8_t d2 = tileDataPtr[tileAddr + ty * 2 + 1];
                uint16_t d = reinterpret_cast<uint16_t *>(tileDataPtr + tileAddr)[ty];

                if(mapAttrs & Tile_XFlip)
                    d = reverseBitsPerByte(d);

                // skip bits
                d <<= tx;

                // palette
                const auto bgPal = bgPalette + (mapAttrs & 0x7) * 4;

                auto bgPixel = [&]()
                {
                    int palIndex = ((d & 0x80) >> 7) | ((d & 0x8000) >> 14);

                    if(lcdc & LCDC_BGDisp)
                        *(rawOut++) = palIndex | tilePriority;
                    *(out++) = bgPal[palIndex];
                };

                if(!tx && x + 8 < xLimit)
                {
                    // copy entire row
                    for(int i = 0; i < 8; i++, d <<= 1)
                        bgPixel();
                    x += 8;
                }
                else
                {
                    // attempt to copy as much of the tile as possible
                    const int limit = std::min(x + 8 - tx, xLimit);

                    for(; x < limit; x++, d <<= 1)
                        bgPixel();
                }
            }
        };

        if(lcdc & LCDC_WindowEnable)
        {
            int windowY = mem.readIOReg(IO_WY);
            if(y >= windowY)
            {
                windowX = mem.readIOReg(IO_WX) - 7;

                if(windowX < screenWidth)
                    isWindow = true;
            }
        }

        // background
        if(windowX > 0)
        {
            auto scrollX = mem.readIOReg(IO_SCX);
            auto scrollY = mem.readIOReg(IO_SCY);

            copyTiles(bgMapPtr, windowX < screenWidth ? windowX : screenWidth, scrollX, y + scrollY);
        }

        // window
        if(isWindow)
        {
            copyTiles(winMapPtr, screenWidth, -windowX, windowY);
            windowY++;
        }
    }

    if(lcdc & LCDC_OBJDisp)
    {
        // sprites
        auto oam = mem.getOAM();

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

        // priority for lower address (GBC behaviour, TODO? sort by x for DMG)
        for(int i = numLineSprites - 1; i >= 0; i--)
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
            uint16_t d = reinterpret_cast<uint16_t *>(spriteDataPtr + tileAddr)[ty];

            if(attrs & Sprite_XFlip)
                d = reverseBitsPerByte(d);

            int x = std::max(0, -spriteX);
            int end = std::min(8, screenWidth - spriteX);

            d <<= x;

            auto out = scanLine + (x + spriteX);
            auto bgIn = bgRaw + x + spriteX;
            for(; x < end; x++, out++, bgIn++, d <<= 1)
            {
                // background has priority
                if(((attrs & Sprite_BGPriority) || (*bgIn & 0x80)/*tile has priority flag*/) && (*bgIn & 0x7F))
                    continue;

                int palIndex = ((d & 0x80) >> 7) | ((d & 0x8000) >> 14);

                if(!palIndex)
                    continue;

                *out = spritePal[palIndex];
            }
        }
    }
}