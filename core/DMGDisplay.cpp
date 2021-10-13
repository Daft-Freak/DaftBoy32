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

}

void DMGDisplay::reset()
{
    memset(bgPalette, 0xFF, sizeof(bgPalette));
    memset(objPalette, 0xFF, sizeof(objPalette)); // not initialised

    lastUpdateCycle = 0;

    enabled = true;
    y = 0;
    statMode = 0;
    compareMatch = false;
    windowY = 0;

    remainingScanlineCycles = scanlineCycles;

    // make sure the default palette gets set up for !GBC
    for(int i = IO_BGP; i <= IO_OBP1; i++)
        mem.write(0xFF00 | i, mem.readIOReg(i));
}

void DMGDisplay::update()
{
    auto curCycle = cpu.getCycleCount();
    if(enabled)
    {
        auto passed = curCycle - lastUpdateCycle;

        if(cpu.getDoubleSpeedMode())
            passed >>= 1;

        while(passed)
        {
            auto step = std::min(static_cast<unsigned>(remainingScanlineCycles), passed);
            passed -= step;

            remainingScanlineCycles -= step;

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
                        drawScanLine(y); // draw right before hblank

                        auto stat = mem.readIOReg(IO_STAT);
                        if(stat & STAT_HBlankInt)
                            cpu.flagInterrupt(Int_LCDStat);
                    }
                    statMode = 0;
                }

                continue;
            }

            if(remainingScanlineCycles > 0)
                continue;

            // next scanline
            remainingScanlineCycles += scanlineCycles;
            y++;

            // coincidince interrupt
            auto stat = mem.readIOReg(IO_STAT);

            bool old = compareMatch;
            compareMatch = y == mem.readIOReg(IO_LYC);

            if((stat & STAT_CoincidenceInt) && !old && compareMatch)
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
                    y = windowY = 0; // end vblank

                if(y < screenHeight)
                {
                    // new scanline
                    statMode = 2; // oam search

                    if(stat & STAT_OAMInt)
                        cpu.flagInterrupt(Int_LCDStat);
                }
            }
        }
    }

    lastUpdateCycle = curCycle;
}

void DMGDisplay::updateForInterrupts()
{
    if(!hblankInterruptEnabled && cpu.getCycleCount() - lastUpdateCycle < remainingScanlineCycles)
        return;

    if(!interruptsEnabled)
        return;

    update();
}

uint8_t DMGDisplay::readReg(uint16_t addr, uint8_t val)
{
    switch(addr & 0xFF)
    {
        case IO_STAT:
            update();
            return (val & 0xF8) | (compareMatch ? STAT_Coincidence : 0) | statMode | 0x80;
        case IO_LY:
            update();
            return y;
    }

    return val;
}

bool DMGDisplay::writeReg(uint16_t addr, uint8_t data)
{
    const uint16_t colMap[]{0xFFFF, 0x56B5, 0x294A, 0};

    switch(addr & 0xFF)
    {
        // update interrupt status
        case IO_STAT:
        case IO_IE:
        {
            update();
            uint8_t ie, stat;

            if((addr & 0xFF) == IO_IE)
            {
                ie = data;
                stat = mem.readIOReg(IO_STAT);
            }
            else
            {
                ie = mem.readIOReg(IO_IE);
                stat = data;
            }

            auto statInts = STAT_HBlankInt | STAT_VBlankInt | STAT_OAMInt | STAT_CoincidenceInt;
            interruptsEnabled = (ie & Int_VBlank) || ((ie & Int_LCDStat) && (stat & statInts));

            // only interrupt that doesn't happen at the end/start of a line
            hblankInterruptEnabled = (ie & Int_LCDStat) && (stat & STAT_HBlankInt);
            break;
        }

        case IO_LCDC:
        {
            update();
            if(!(data & LCDC_DisplayEnable))
            {
                // reset
                remainingScanlineCycles = scanlineCycles;
                statMode = 0;
                y = windowY = 0;
            }
            else if(!enabled) // enabling
                updateCompare(y == mem.readIOReg(IO_LYC));

            enabled = data & LCDC_DisplayEnable;
            break;
        }

        case IO_LY:
            return true;
        case IO_LYC:
            if(enabled)
            {
                update();
                updateCompare(y == data);
            }

            break;

        // grey palettes
        case IO_BGP:
        {
            if(cpu.getColourMode())
                break;

            update();

            for(int i = 0; i < 4; i++)
                bgPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP0:
        {
            if(cpu.getColourMode())
                break;

            update();

            for(int i = 0; i < 4; i++)
                objPalette[i] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }
        case IO_OBP1:
        {
            if(cpu.getColourMode())
                break;

            update();

            for(int i = 0; i < 4; i++)
                objPalette[i + 4] = colMap[(data >> (2 * i)) & 0x3];
            break;
        }

        case IO_BCPS:
        case IO_OCPS:
            return !cpu.getColourMode(); // ignore write on !GBC

        // colour palettes
        case IO_BCPD:
        {
            if(!cpu.getColourMode())
                return true;

            update();

            auto bcps = mem.readIOReg(IO_BCPS);
            reinterpret_cast<uint8_t *>(bgPalette)[bcps & 0x3F] = data;

            // auto inc
            if(bcps & 0x80)
                mem.writeIOReg(IO_BCPS, ((bcps & 0x3F) + 1) | 0x80);

            return true;
        }

        case IO_OCPD:
        {
            if(!cpu.getColourMode())
                return true;

            update();

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

// helpers/constants
static const int tileDataSize = 16;
static const int screenSizeTiles = 32; // 32x32 tiles

static const int numSprites = 40;

// get the two data bytes for a tile row
// handles x/y flips
static uint16_t getTileRow(uint8_t lcdc, uint8_t *mapPtr, uint8_t *tileDataPtr, int tileY, int &attrs)
{
    attrs = mapPtr[0x2000]; // GBC, bank 1

    // tile id is signed if addr == 0x8800
    int tileId = (lcdc & LCDC_TileData8000) ? *mapPtr : (int8_t)(*mapPtr) + 128;

    auto tileAddr = tileId * tileDataSize;

    if(attrs & Tile_Bank)
        tileAddr += 0x2000;

    if(attrs & Tile_YFlip)
        tileY = 7 - tileY;

    auto d = reinterpret_cast<uint16_t *>(tileDataPtr + tileAddr)[tileY];

    if(attrs & Tile_XFlip)
        d = reverseBitsPerByte(d);

    return d;
};

static uint16_t getTileRow(uint8_t lcdc, uint8_t *mapPtr, uint8_t *tileDataPtr, int x, int y, int tileY, int &attrs)
{
    int tileId = x + y * screenSizeTiles;

    return getTileRow(lcdc, mapPtr + tileId, tileDataPtr, tileY, attrs);
};

// gets the two bit index from the top of the two bytes
inline int getPalIndex(uint16_t d)
{
    return ((d & 0x80) >> 7) | ((d >> 15) << 1);
};

void DMGDisplay::drawScanLine(int y)
{
    // contains palette index + a tile priority flag
    uint8_t bgRaw[screenWidth]{0};

    auto scanLine = screenData + y * screenWidth;

    auto lcdc = mem.readIOReg(IO_LCDC);

    const bool isColour = cpu.getColourMode();

    // active scanline
    // this is reduced to a priority flag on GBC
    if(lcdc & LCDC_BGDisp || isColour)
        drawBackground(scanLine, bgRaw);

    if(lcdc & LCDC_OBJDisp)
        drawSprites(scanLine, bgRaw);
}

void DMGDisplay::drawBackground(uint16_t *scanLine, uint8_t *bgRaw)
{
    auto lcdc = mem.readIOReg(IO_LCDC);

    auto vram = mem.getVRAM();
    auto tileDataPtr = (lcdc & LCDC_TileData8000) ? vram : vram + 0x800;
    auto bgMapPtr = (lcdc & LCDC_BGTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;
    auto winMapPtr = (lcdc & LCDC_WindowTileMap9C00) ? vram + 0x1C00 : vram + 0x1800;

    int windowX = screenWidth;

    int x = 0;
    auto out = scanLine;
    auto rawOut = bgRaw;

    auto copyPartialTile = [this, &x, lcdc, &out, &rawOut](int endX, uint16_t d, int tileAttrs)
    {
        uint8_t tilePriority = (tileAttrs & Tile_BGPriority) ? 0x80 : 0;

        // palette
        const auto bgPal = bgPalette + (tileAttrs & 0x7) * 4;

        for(; x < endX; x++, d <<= 1)
        {
            int palIndex = getPalIndex(d);

            if(lcdc & LCDC_BGDisp)
                *(rawOut++) = palIndex | tilePriority;
            *(out++) = bgPal[palIndex];
        }
    };

    auto copyFullTile = [this, &x, lcdc, &out, &rawOut](uint16_t d, int tileAttrs)
    {
        uint8_t tilePriority = (tileAttrs & Tile_BGPriority) ? 0x80 : 0;

        // palette
        const auto bgPal = bgPalette + (tileAttrs & 0x7) * 4;

        for(int i = 0; i < 8; i++, d <<= 1)
        {
            int palIndex = getPalIndex(d);

            if(lcdc & LCDC_BGDisp)
                *(rawOut++) = palIndex | tilePriority;
            *(out++) = bgPal[palIndex];
        }
    };

    auto copyTiles = [this, &x, lcdc, tileDataPtr, &out, &rawOut, &copyPartialTile, &copyFullTile](uint8_t *mapPtr, int xLimit, int offsetX, uint8_t oy)
    {
        // full tiles
        uint8_t ox = x + offsetX; // this is a uint8 so that it wraps

        auto rowMapPtr = mapPtr + (oy / 8) * screenSizeTiles;

        while(x + 7 < xLimit)
        {
            int mapAttrs;
            auto d = getTileRow(lcdc, rowMapPtr + ox / 8, tileDataPtr, oy & 7, mapAttrs);

            copyFullTile(d, mapAttrs);
            x += 8;
            ox += 8;
        }

        if(x < xLimit)
        {
            int mapAttrs;
            auto d = getTileRow(lcdc, rowMapPtr + ox / 8, tileDataPtr, oy & 7, mapAttrs);

            copyPartialTile(xLimit, d, mapAttrs);
        }
    };

    if(lcdc & LCDC_WindowEnable)
    {
        int windowY = mem.readIOReg(IO_WY);
        if(y >= windowY)
            windowX = mem.readIOReg(IO_WX) - 7;
    }

    // background
    if(windowX > 0)
    {
        auto scrollX = mem.readIOReg(IO_SCX);
        auto scrollY = mem.readIOReg(IO_SCY);

        // partial tile at the start of the line
        if(scrollX & 7)
        {
            uint8_t oy = y + scrollY;
            int mapAttrs;
            auto d = getTileRow(lcdc, bgMapPtr, tileDataPtr, scrollX / 8, oy / 8, oy & 7, mapAttrs);

            // skip bits
            d <<= scrollX & 7;

            copyPartialTile(8 - (scrollX & 7), d, mapAttrs);
        }

        int xEnd = windowX < screenWidth ? windowX : screenWidth;
        copyTiles(bgMapPtr, xEnd, scrollX, y + scrollY);
    }

    // window
    if(x < screenWidth)
    {
        copyTiles(winMapPtr, screenWidth, -windowX, windowY);
        windowY++;
    }
}

void DMGDisplay::drawSprites(uint16_t *scanLine, uint8_t *bgRaw)
{
    auto lcdc = mem.readIOReg(IO_LCDC);
    const bool isColour = cpu.getColourMode();

    const int spriteHeight = (lcdc & LCDC_Sprite8x16) ? 16 : 8;

    // sprites
    auto oam = mem.getOAM();
    auto spriteDataPtr = mem.getVRAM();

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

void DMGDisplay::updateCompare(bool newVal)
{
    if((mem.readIOReg(IO_STAT) & STAT_CoincidenceInt) && !compareMatch && newVal)
        cpu.flagInterrupt(Int_LCDStat);

    compareMatch = newVal;
}
