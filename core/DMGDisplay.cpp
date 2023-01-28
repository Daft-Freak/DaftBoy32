#include <algorithm>
#include <cstring>

#include "DMGDisplay.h"

#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"
#include "DMGSaveState.h"

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

// constants
static const int tileDataSize = 16;
static const int screenSizeTiles = 32; // 32x32 tiles

static const int numSprites = 40;

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
    firstFrame = false;

    remainingScanlineCycles = scanlineCycles;
    remainingModeCycles = 4;

    // make sure the default palette gets set up for !GBC
    for(int i = IO_BGP; i <= IO_OBP1; i++)
        mem.write(0xFF00 | i, mem.readIOReg(i));
}

void DMGDisplay::loadSaveState(BESSCore &bess, DaftState &state, std::function<uint32_t(uint32_t, uint32_t, uint8_t *)> readFunc)
{
    readFunc(bess.bgPalOff, bess.bgPalSize, reinterpret_cast<uint8_t *>(bgPalette));
    readFunc(bess.objPalOff, bess.objPalSize, reinterpret_cast<uint8_t *>(objPalette));

    // sync with loaded regs
    enabled = bess.ioRegs[IO_LCDC] & LCDC_DisplayEnable;
    y = bess.ioRegs[IO_LY];
    statMode = bess.ioRegs[IO_STAT] & STAT_Mode;
    compareMatch = bess.ioRegs[IO_STAT] & STAT_Coincidence;

    if(state.version)
    {
        windowY = state.windowY;
        firstFrame = state.flags & State_DispFirstFrame;
        statInterruptActive = state.statInterruptActive;
        remainingScanlineCycles = state.remainingScalineCycles;
        remainingModeCycles = state.remainingModeCycles;
    }

    // interrupts
    const auto statInts = STAT_HBlankInt | STAT_VBlankInt | STAT_OAMInt | STAT_CoincidenceInt;
    interruptsEnabled = (bess.ie & Int_VBlank) || ((bess.ie & Int_LCDStat) && (bess.ioRegs[IO_STAT] & statInts));

    lastUpdateCycle = cpu.getCycleCount();
}

void DMGDisplay::saveState(DaftState &state)
{
    state.windowY = windowY;
    state.statInterruptActive = statInterruptActive;
    state.remainingScalineCycles = remainingScanlineCycles;
    state.remainingModeCycles = remainingModeCycles;

    if(firstFrame)
        state.flags |= State_DispFirstFrame;
}

void DMGDisplay::savePaletteState(BESSCore &bess, std::function<uint32_t(uint32_t, uint32_t, const uint8_t *)> writeFunc, uint32_t &offset)
{
    bool isColour = cpu.getColourMode();

    if(!isColour)
    {
        bess.bgPalOff = bess.bgPalSize = 0;
        bess.bgPalOff = bess.objPalOff = 0;
        return;
    }

    bess.bgPalOff = offset;
    bess.bgPalSize = sizeof(bgPalette);
    writeFunc(offset, bess.bgPalSize, reinterpret_cast<uint8_t *>(bgPalette));
    offset += bess.bgPalSize;

    bess.objPalOff = offset;
    bess.objPalSize = sizeof(objPalette);
    writeFunc(offset, bess.objPalSize, reinterpret_cast<uint8_t *>(objPalette));
    offset += bess.objPalSize;
}

void DMGDisplay::update()
{
    auto curCycle = cpu.getCycleCount();
    if(enabled)
    {
        auto passed = curCycle - lastUpdateCycle;

        // blank out the screen if we're stopped
        if(cpu.getStopped())
        {
            // check if CGB, even in DMG mode
            bool isCGB = cpu.getConsole() == DMGCPU::Console::CGB || cpu.getColourMode();

            // mode 3 on CGB keeps the old image
            if(!isCGB || statMode != 3)
                memset(screenData, isCGB ? 0 : 0xFF, screenWidth * screenHeight * 2);

            lastUpdateCycle = curCycle;
            return;
        }

        if(cpu.getDoubleSpeedMode())
            passed >>= 1;

        while(passed)
        {
            auto step = std::min(remainingModeCycles, passed);

            passed -= step;

            remainingScanlineCycles -= step;
            remainingModeCycles -= step;

            // update STAT if not vblank
            if(remainingModeCycles == 0)
            {
                // new mode
                if(statMode <= 1 && remainingScanlineCycles)
                {
                    // start of line - update compare
                    updateCompare(y == mem.readIOReg(IO_LYC));

                    auto stat = mem.readIOReg(IO_STAT);

                    // in vblank
                    if(y >= screenHeight)
                    {
                        // start of vblank
                        if(y == screenHeight)
                        {
                            cpu.flagInterrupt(Int_VBlank);
                            statMode = 1;
                            remainingModeCycles = scanlineCycles;

                            // line 144 still generates the OAM/mode 2 interrupt
                            // (do other vblank lines do this?)
                            if((stat & (STAT_VBlankInt | STAT_OAMInt)))
                            {
                                if(!statInterruptActive)
                                    cpu.flagInterrupt(Int_LCDStat);

                                statInterruptActive |= (stat & (STAT_VBlankInt | STAT_OAMInt));
                            }
                            statInterruptActive &= ~STAT_HBlankInt;
                        }

                        // line 153 has some additional wierdness... (switches to y = 0 early)

                        remainingModeCycles = remainingScanlineCycles;

                        continue;
                    }

                    // first line after enabling
                    if(firstFrame)
                    {
                        // skips mode 2
                        statMode = 3;
                        remainingModeCycles = 172; // minimal time (skipped the bit that reads sprites)
                        firstFrame = false;
                        continue;
                    }

                    // line start -> mode 2
                    statMode = 2; // oam search
                    remainingModeCycles = 80;

                    if(stat & STAT_OAMInt)
                    {
                        if(!statInterruptActive)
                            cpu.flagInterrupt(Int_LCDStat);

                        statInterruptActive |= STAT_OAMInt;
                    }
                    statInterruptActive &= ~(STAT_HBlankInt | STAT_VBlankInt);

                    continue;
                }
                else if(statMode == 2)
                {
                    // mode 2 -> 3
                    statMode = 3;
                    statInterruptActive &= ~(STAT_HBlankInt | STAT_OAMInt);

                    // guess something semi-accurate
                    remainingModeCycles = 172 + (mem.getIOReg(IO_SCX) & 7);

                    // more if sprites
                    const int spriteHeight = (mem.getIOReg(IO_LCDC) & LCDC_Sprite8x16) ? 16 : 8;
                    int numLineSprites = 0;
                    auto oam = mem.getOAM();

                    for(int i = 0; i < numSprites && numLineSprites < 10; i++)
                    {
                        const int spriteY = oam[i * 4] - 16;

                        // not on this line
                        if(y < spriteY || y >= spriteY + spriteHeight)
                            continue;

                        numLineSprites++;
                        remainingModeCycles += 11; // not really accurate, does result in the right range though
                    }

                    continue;
                }
                else if(statMode == 3)
                {
                    // mode 3 -> 0 (hblank)
                    statMode = 0;
                    drawScanLine(y); // draw right before hblank

                    auto stat = mem.readIOReg(IO_STAT);
                    if((stat & STAT_HBlankInt))
                    {
                        if(!statInterruptActive)
                            cpu.flagInterrupt(Int_LCDStat);

                        statInterruptActive |= STAT_HBlankInt;
                    }

                    remainingModeCycles = remainingScanlineCycles;
                    continue;
                }
                else if(statMode == 1)
                    remainingModeCycles = scanlineCycles;
            }
            else
                continue;

            // next scanline
            remainingScanlineCycles = scanlineCycles;
            remainingModeCycles = 4;
            y++;

            compareMatch = false; // cleared for one cycle

            if(y > 153)
            {
                y = windowY = 0; // end vblank
                statMode = 0;
            }
        }
    }

    lastUpdateCycle = curCycle;
}

void DMGDisplay::updateForInterrupts()
{
    if(!interruptsEnabled)
        return; // nothing to do

    auto passed = cpu.getCycleCount() - lastUpdateCycle;
    bool doubleSpeed = cpu.getDoubleSpeedMode();

    if(doubleSpeed)
        passed >>= 1;

    if(passed < remainingModeCycles)
        return;

    update();
}

int DMGDisplay::getCyclesToNextUpdate() const
{
    if(!interruptsEnabled)
        return 0x7FFFFFFF; // this is only used when halted and clamped to something reasonable in the CPU

    auto passed = cpu.getCycleCount() - lastUpdateCycle;
    bool doubleSpeed = cpu.getDoubleSpeedMode();

    if(doubleSpeed)
        passed >>= 1;

    return (remainingModeCycles - passed) * (doubleSpeed ? 2 : 1);
}

void DMGDisplay::setFramebuffer(uint16_t *data, int stride)
{
    screenData = data;
    screenStride = stride ? stride : screenWidth;
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
    // pre-converting the DMG palette is easy
#ifdef DISPLAY_RGB565
    const uint16_t colMap[]{0xFFFF, 0xAD55, 0x528A, 0};
#else
    const uint16_t colMap[]{0xFFFF, 0x56B5, 0x294A, 0};
#endif

    const auto statInts = STAT_HBlankInt | STAT_VBlankInt | STAT_OAMInt | STAT_CoincidenceInt;

    switch(addr & 0xFF)
    {
        case IO_STAT:
        {
            update();

            auto ie = mem.readIOReg(IO_IE);
            auto stat = data;

            // trigger vblank stat (and others?)
            if(statMode == 1 && (stat & STAT_VBlankInt) && !statInterruptActive)
            {
                cpu.flagInterrupt(Int_LCDStat);
                statInterruptActive |= STAT_VBlankInt;
            }

            statInterruptActive &= stat;

            interruptsEnabled = (ie & Int_VBlank) || ((ie & Int_LCDStat) && (stat & statInts));
            break;
        }

        case IO_LCDC:
        {
            update();
            if(!(data & LCDC_DisplayEnable))
            {
                // reset
                remainingScanlineCycles = scanlineCycles - 4; // running behind
                statMode = 0;
                remainingModeCycles = 80; // no mode 2
                y = windowY = 0;
                firstFrame = true;
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
             // ignore write on !GBC, force bit 6 otherwise
            if(cpu.getColourMode())
                mem.writeIOReg(addr & 0xFF, data | 0x40);
            return true;

        // colour palettes
        case IO_BCPD:
        {
            if(!cpu.getColourMode())
                return true;

            update();

            auto bcps = mem.readIOReg(IO_BCPS);

#ifdef DISPLAY_RGB565
            reinterpret_cast<uint8_t *>(bgPaletteRaw)[bcps & 0x3F] = data;
            bgPaletteDirty = true;
#else
            reinterpret_cast<uint8_t *>(bgPalette)[bcps & 0x3F] = data;
#endif
            // auto inc
            if(bcps & 0x80)
                mem.writeIOReg(IO_BCPS, ((bcps & 0x3F) + 1) | 0xC0);

            return true;
        }

        case IO_OCPD:
        {
            if(!cpu.getColourMode())
                return true;

            update();

            auto ocps = mem.readIOReg(IO_OCPS);

#ifdef DISPLAY_RGB565
            reinterpret_cast<uint8_t *>(objPaletteRaw)[ocps & 0x3F] = data;
            objPaletteDirty = true;
#else
            reinterpret_cast<uint8_t *>(objPalette)[ocps & 0x3F] = data;
#endif

            // auto inc
            if(ocps & 0x80)
                mem.writeIOReg(IO_OCPS, ((ocps & 0x3F) + 1) | 0xC0);

            return true;
        }

        case IO_OPRI:
        {
            if(!cpu.getColourMode())
                return true;
            // this is set by the bios
            break;
        }

        // update interrupt status
        case IO_IE:
        {
            update();
            auto ie = data;
            auto stat = mem.readIOReg(IO_STAT);
            interruptsEnabled = (ie & Int_VBlank) || ((ie & Int_LCDStat) && (stat & statInts));
            break;
        }

    }

    return false;
}

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

static void copyPartialTile(uint8_t lcdc, int &x, int endX, uint16_t d, int tileAttrs, uint16_t *bgPalette, uint16_t *&out, uint8_t *&rawOut)
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

static void copyFullTile(uint8_t lcdc, uint16_t d, int tileAttrs, uint16_t *bgPalette, uint16_t *&out, uint8_t *&rawOut)
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

static void copyTiles(uint8_t lcdc, uint8_t *tileDataPtr, uint8_t *mapPtr, uint16_t *bgPalette, int &x, int xLimit, int offsetX, uint8_t oy, uint16_t *&out, uint8_t *&rawOut)
{
    // full tiles
    uint8_t ox = x + offsetX; // this is a uint8 so that it wraps

    auto rowMapPtr = mapPtr + (oy / 8) * screenSizeTiles;

    while(x + 7 < xLimit)
    {
        int mapAttrs;
        auto d = getTileRow(lcdc, rowMapPtr + ox / 8, tileDataPtr, oy & 7, mapAttrs);

        copyFullTile(lcdc, d, mapAttrs, bgPalette, out, rawOut);
        x += 8;
        ox += 8;
    }

    if(x < xLimit)
    {
        int mapAttrs;
        auto d = getTileRow(lcdc, rowMapPtr + ox / 8, tileDataPtr, oy & 7, mapAttrs);

        copyPartialTile(lcdc, x, xLimit, d, mapAttrs, bgPalette, out, rawOut);
    }
};

void DMGDisplay::drawScanLine(int y)
{
    // contains palette index + a tile priority flag
    uint8_t bgRaw[screenWidth]{0};

#ifdef PICO_BUILD
    // need to avoid writing unfinished lines directly to the framebuffer as we're not synced
    uint16_t scanLine[screenWidth];
#else
    auto scanLine = screenData + y * screenStride;
#endif

    auto lcdc = mem.readIOReg(IO_LCDC);

    const bool isColour = cpu.getColourMode();

// sync palettes
#ifdef DISPLAY_RGB565
    if(bgPaletteDirty)
    {
        auto outCol = bgPalette;
        for(auto &col : bgPaletteRaw)
            *outCol++ = (col & 0x1F)  | (col & 0x7FE0) << 1;

        bgPaletteDirty = false;
    }

    if(objPaletteDirty)
    {
        auto outCol = objPalette;
        for(auto &col : objPaletteRaw)
            *outCol++ = (col & 0x1F)  | (col & 0x7FE0) << 1;

        objPaletteDirty = false;
    }

#endif

    // active scanline
    // this is reduced to a priority flag on GBC
    if(lcdc & LCDC_BGDisp || isColour)
        drawBackground(scanLine, bgRaw);

    if(lcdc & LCDC_OBJDisp)
        drawSprites(scanLine, bgRaw);

#ifdef PICO_BUILD
    memcpy(screenData + y * screenStride, scanLine, screenWidth * 2);
#endif
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

            copyPartialTile(lcdc, x, 8 - (scrollX & 7), d, mapAttrs, bgPalette, out, rawOut);
        }

        int xEnd = windowX < screenWidth ? windowX : screenWidth;
        copyTiles(lcdc, tileDataPtr, bgMapPtr, bgPalette, x, xEnd, scrollX, y + scrollY, out, rawOut);
    }

    // window
    if(x < screenWidth)
    {
        copyTiles(lcdc, tileDataPtr, winMapPtr, bgPalette, x, screenWidth, -windowX, windowY, out, rawOut);
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

    if(!isColour && numLineSprites > 1)
    {
        // sort sprites by x
        for(int i = 1; i < numLineSprites; i++)
        {
            int j = i;

            while(j && oam[lineSprites[j] * 4 + 1] < oam[lineSprites[j - 1] * 4 + 1])
            {
                std::swap(lineSprites[j], lineSprites[j - 1]);
                j--;
            }
        }
    }

    // TODO: the bg priority handling here isn't quite right
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
    {
        if(!statInterruptActive)
            cpu.flagInterrupt(Int_LCDStat);

        statInterruptActive |= STAT_CoincidenceInt;
    }
    else if(!newVal)
        statInterruptActive &= ~STAT_CoincidenceInt;

    compareMatch = newVal;
}
