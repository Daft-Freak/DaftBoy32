#include <algorithm>
#include <cstring>

#include "AGBDisplay.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

enum OAMAttr0Bits
{
    Attr0_Y         = 0xFF,
    Attr0_Mode      = 0x3 << 8, // normal, affine, disabled, affine double
    Attr0_Effect    = 0x3 << 10, // normal, blend, window, INVALID
    Attr0_Mosaic    = 1 << 12,
    Attr0_SinglePal = 1 << 13,
    Attr0_Shape     = 0x3 << 14 // square, wide tall
};

enum OAMAttr1Bits
{
    Attr1_X           = 0x1FF,
    Attr1_AffineIndex = 0x1F << 9, // this...
    Attr1_HFlip       = 1 << 12, // ... or these
    Attr1_VFlip       = 1 << 13, //
    Attr1_Size        = 0x3 << 14
};

enum OAMAttr2Bits
{
    Attr2_Index    = 0x3FF,
    Attr2_Priority = 0x3 << 10,
    Attr2_Pal      = 0xF << 12
};

// bg layer helpers
// hopefully this gets mostly inlined
static void drawTextBG(int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control, uint16_t xOffset, uint16_t yOffset)
{
    const int screenBlockTiles = 32; // 32x32

    int screenSize = (control & BGCNT_ScreenSize) >> 14;

    int ty = (y + yOffset) & 7;
    int by = (y + yOffset) >> 3;

    int blockIndex = 0;

    // inc index for second row
    if((screenSize & 2) && (by & screenBlockTiles)) // 1x2, 2x2
        blockIndex += screenSize == 2 ? 1 : 2;

    // wrap
    by &= (screenBlockTiles - 1);

    auto blockBase = (control & BGCNT_ScreenBase) << 3; // >> 8 * 0x800
    uint16_t *screenPtr = reinterpret_cast<uint16_t *>(vram + blockBase + blockIndex * 0x800);

    auto charBase = ((control & BGCNT_CharBase) >> 2) * 0x4000;
    auto charPtr = vram + charBase;

    for(int x = 0; x < 240;)
    {
        int tx = (x + xOffset) & 7;
        int bx = (x + xOffset) >> 3;

        auto tilePtr = screenPtr + by * screenBlockTiles;

        // second column
        if(screenSize & 1 && (bx & screenBlockTiles)) // 2x1, 2x2
            tilePtr += screenBlockTiles * screenBlockTiles;

        // wrap
        bx &= (screenBlockTiles - 1);

        uint16_t tileMeta = tilePtr[bx];

        // v flip
        int tty = ty;
        if(tileMeta & (1 << 11))
            tty = 7 - ty;

        if(control & BGCNT_SinglePal)
        {
            // 8bit tiles
            uint64_t tileRow = reinterpret_cast<uint64_t *>(charPtr + (tileMeta & 0x3FF) * 64)[tty];

            // h flip
            if(tileMeta & (1 << 10))
                tileRow = __builtin_bswap64(tileRow);

            auto tilePtr = reinterpret_cast<uint8_t *>(&tileRow) + tx;

            if(!tx && x + 8 < 240)
            {
                // full tile
                x += 8;
                for(; tx < 8; tx++, scanLine++)
                {
                    auto palIndex = *tilePtr++;
                    if(palIndex)
                        *scanLine = palRam[palIndex];
                }
            }
            else
            {
                // edges
                for(; tx < 8 && x < 240; tx++, x++, scanLine++)
                {
                    auto palIndex = *tilePtr++;
                    if(palIndex)
                        *scanLine = palRam[palIndex];
                }
            }
        }
        else
        {
            // 4bit tiles
            uint32_t tileRow = reinterpret_cast<uint32_t *>(charPtr + (tileMeta & 0x3FF) * 32)[tty];
            auto tilePal = palRam + ((tileMeta & 0xF000) >> 8);

            // h flip
            if(tileMeta & (1 << 10))
            {
                tileRow = __builtin_bswap32(tileRow);
                tileRow = ((tileRow & 0xF0F0F0F0) >> 4) | ((tileRow & 0x0F0F0F0F) << 4);
            }

            if(!tx && x + 8 < 240)
            {
                // full tile
                x += 8;
                for(; tx < 8; tx++, tileRow >>= 4, scanLine++)
                {
                    if(tileRow & 0xF)
                        *scanLine = tilePal[tileRow & 0xF];
                }
            }
            else
            {
                // edges
                tileRow >>= (tx * 4);

                for(; tx < 8 && x < 240; tx++, x++, tileRow >>= 4, scanLine++)
                {
                    if(tileRow & 0xF)
                        *scanLine = tilePal[tileRow & 0xF];
                }
            }
        }
    }
}

static void drawAffineBG(uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control, int32_t xOffset, int32_t yOffset, int16_t a, int16_t c)
{
    int screenSize = (control & BGCNT_ScreenSize) >> 14;

    int curX = xOffset;
    int curY = yOffset;

    int numTiles = 16 << screenSize; // 16-128

    auto blockBase = (control & BGCNT_ScreenBase) << 3; // >> 8 * 0x800
    auto screenPtr = reinterpret_cast<uint8_t *>(vram + blockBase);

    auto charBase = ((control & BGCNT_CharBase) >> 2) * 0x4000;
    auto charPtr = vram + charBase;

    bool wrap = control & BGCNT_Wrap;

    for(int x = 0; x < 240; x++, scanLine++, curX += a, curY += c)
    {
        int tx = (curX >> 8) & 7;
        int bx = curX >> 11;

        int ty = (curY >> 8) & 7;
        int by = curY >> 11;

        // wrap/clamp
        if(bx < 0 || bx >= numTiles)
        {
            if(wrap)
                bx &= numTiles - 1;
            else
                continue;
        }

        if(by < 0 || by >= numTiles)
        {
            if(wrap)
                by &= numTiles - 1;
            else
                continue;
        }

        auto tilePtr = screenPtr + by * numTiles;

        uint8_t tileIndex = tilePtr[bx];

        // 8bit tiles
        uint64_t tileRow = reinterpret_cast<uint64_t *>(charPtr + tileIndex * 64)[ty];

        auto tileDataPtr = reinterpret_cast<uint8_t *>(&tileRow) + tx;

        auto palIndex = *tileDataPtr++;
        if(palIndex)
            *scanLine = palRam[palIndex];
    }
}

// these two are always "text" mode
static void drawBG0(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    if((dispControl & DISPCNT_Mode) > 1)
        return;

    drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG0HOFS), mem.readIOReg(IO_BG0VOFS));
}

static void drawBG1(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    if((dispControl & DISPCNT_Mode) > 1)
        return;

    drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG1HOFS), mem.readIOReg(IO_BG1VOFS));
}

static void drawBG2(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control, int32_t refPointX, int32_t refPointY)
{
    switch(dispControl & DISPCNT_Mode)
    {
        case 0: // "text" mode
            drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG2HOFS), mem.readIOReg(IO_BG2VOFS));
            break;
        case 1: // affine mode
        case 2:
            drawAffineBG(scanLine, palRam, vram, dispControl, control, refPointX, refPointY, mem.readIOReg(IO_BG2PA), mem.readIOReg(IO_BG2PC));
            break;
        case 3: // 16-bit fullscreen bitmap
        {
            auto inPtr = reinterpret_cast<uint16_t *>(vram + y * 240 * 2);
            auto outPtr = scanLine;
            for(int x = 0; x < 240; x++)
                *outPtr++ = *inPtr++;
            break;
        }
        case 4: // paletted fullscreen bitmap
        {
            auto inPtr = vram + y * 240;
            if(dispControl & DISPCNT_Frame)
                inPtr += 0xA000;

            auto outPtr = scanLine;
            for(int x = 0; x < 240; x++)
                *outPtr++ = palRam[*inPtr++];
            break;
        }
        case 5: // 16-bit 160*128 bitmap
        {
            auto outPtr = scanLine;
            if(y < 128)
            {
                auto inPtr = reinterpret_cast<uint16_t *>(vram + y * 160 * 2);
                if(dispControl & DISPCNT_Frame)
                    inPtr += (0xA000 / 2);

                int x;
                for(x = 0; x < 160; x++)
                    *outPtr++ = *inPtr++;
            }

            break;
        }
        // else invalid
    }
}

static void drawBG3(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control, int32_t refPointX, int32_t refPointY)
{
    if((dispControl & DISPCNT_Mode) == 0)
        drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG3HOFS), mem.readIOReg(IO_BG3VOFS));
    else if((dispControl & DISPCNT_Mode) == 2)
        drawAffineBG(scanLine, palRam, vram, dispControl, control, refPointX, refPointY, mem.readIOReg(IO_BG3PA), mem.readIOReg(IO_BG3PC));
}

static void drawOBJs(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, int priority)
{
    auto oam = reinterpret_cast<uint16_t *>(mem.getOAM());
    const int entrySize = 4; // * 16 bit

    // width, flip the shape bits for heights
    static const int sizes[]
    {
         8, 16, 32, 64, // square
        16, 32, 32, 64, // wide
         8,  8, 16, 32  // tall
    };

    auto charPtr = vram + 0x10000;

    for(int i = 127; i >= 0; i--)
    {
        const uint16_t attr0 = oam[i * entrySize + 0];
        const uint16_t attr1 = oam[i * entrySize + 1];
        const uint16_t attr2 = oam[i * entrySize + 2];

        // not this priority
        if((attr2 & Attr2_Priority) >> 10 != priority)
            continue;

        auto mode = (attr0 & Attr0_Mode) >> 8;

        if(mode == 2/*disable*/)
            continue;

        // check y
        int spriteY = attr0 & Attr0_Y;

        auto shape = (attr0 & Attr0_Shape) >> 12;
        int sizeShape = (shape ? shape ^ 0xC : 0) | ((attr1 & Attr1_Size) >> 14);
        int spriteH = sizes[sizeShape];

        // possibly doubled size
        int doubledH = mode == 3 ? spriteH * 2 : spriteH;

        // wrap
        if(((spriteY + doubledH) & 0xFF) < spriteY)
            spriteY -= 256;

        if(spriteY > y)
            continue;

        if(spriteY + doubledH <= y)
            continue;

        // get X/W
        int spriteX = attr1 & Attr1_X;
        sizeShape = shape | ((attr1 & Attr1_Size) >> 14);
        const int spriteW = sizes[sizeShape];

        // wrap
        if(((spriteX + spriteW) & 0x1FF) < spriteX)
            spriteX -= 512;

        if(spriteX >= 240)
            continue;

        // calc offsets
        auto startTile = attr2 & Attr2_Index;
        int sx = std::max(0, -spriteX);
        int sy = y - spriteY;

        // palette
        auto spritePal = palRam + 256;

        if(mode != 0)
        {
            // affine sprite
            int affineIndex = (attr1 & Attr1_AffineIndex) >> 9;

            int16_t a, b, c, d;

            a = oam[(affineIndex * 4 + 0) * entrySize + 3];
            b = oam[(affineIndex * 4 + 1) * entrySize + 3];
            c = oam[(affineIndex * 4 + 2) * entrySize + 3];
            d = oam[(affineIndex * 4 + 3) * entrySize + 3];

            // offsets
            int halfW = spriteW;
            int halfH = spriteH;

            if(mode != 3)
            {
                halfW >>= 1;
                halfH >>= 1;
            }

            // transformed coords
            int tx = ((spriteW / 2) << 8) + (sx - halfW) * a + (sy - halfH) * b;
            int ty = ((spriteH / 2) << 8) + (sx - halfW) * c + (sy - halfH) * d;

            auto out = scanLine + (spriteX + sx);
            auto outEnd = scanLine + 240;

            if(!(attr0 & Attr0_SinglePal))
                spritePal += ((attr2 & Attr2_Pal) >> 8);

            for(int x = sx; x < halfW * 2 && out != outEnd; x++, out++, tx += a, ty += c)
            {
                if(tx < 0 || ty < 0 || tx >= (spriteW << 8) || ty >= (spriteH << 8))
                    continue;

                auto tile = startTile;

                if(dispControl & DISPCNT_OBJChar1D)
                    tile += (ty >> 11) * (spriteW >> 3) * (attr0 & Attr0_SinglePal ? 2 : 1);
                else
                    tile += (ty >> 11) * 32;

                if(attr0 & Attr0_SinglePal)
                {
                    auto tilePtr = charPtr + tile * 32 + (tx >> 11) * 64 + ((ty >> 8) & 7) * 8 + ((tx >> 8) & 7);

                    if(*tilePtr)
                        *out = spritePal[*tilePtr];
                }
                else
                {
                    uint32_t tileRow = reinterpret_cast<uint32_t *>(charPtr + (tile + (tx >> 11)) * 32)[(ty >> 8) & 7];
                    tileRow >>= ((tx >> 8) & 7) * 4;

                    if(tileRow & 0xF)
                        *out = spritePal[tileRow & 0xF];
                }
            }
        }
        else
        {
            // regular sprite
            bool hFlip = mode == 0 && (attr1 & Attr1_HFlip);

            // v flip
            if(mode == 0 && (attr1 & Attr1_VFlip))
                sy = (spriteH - 1) - sy;

            if(dispControl & DISPCNT_OBJChar1D)
                startTile += (sy >> 3) * (spriteW >> 3) * (attr0 & Attr0_SinglePal ? 2 : 1);
            else
                startTile += (sy >> 3) * 32;

            int xOff = sx & 7;
            auto out = scanLine + (spriteX + sx);
            auto outEnd = scanLine + 240;

            int tilesX = spriteW >> 3;

            if(attr0 & Attr0_SinglePal)
            {
                // 8bit tiles
                for(int stx = sx >> 3; stx < tilesX && out != outEnd; stx++)
                {
                    int tileX = hFlip ? (tilesX - 1) - stx : stx;
                    auto tilePtr = charPtr + startTile * 32 + tileX * 64 + (sy & 7) * 8;

                    if(hFlip)
                        tilePtr += 7 - xOff;
                    else
                        tilePtr += xOff;

                    // pixels in tile
                    for(int x = xOff; x < 8 && out != outEnd; x++, out++)
                    {
                        auto palIndex = *tilePtr;
                        if(palIndex)
                            *out = spritePal[palIndex];

                        if(hFlip)
                            --tilePtr;
                        else
                            ++tilePtr;
                    }

                    xOff = 0;
                }
            }
            else
            {
                // 4bit tiles
                spritePal += ((attr2 & Attr2_Pal) >> 8);

                for(int stx = sx >> 3; stx < tilesX && out != outEnd; stx++)
                {
                    int tileX = hFlip ? (tilesX - 1) - stx : stx;
                    uint32_t tileRow = reinterpret_cast<uint32_t *>(charPtr + (startTile + tileX) * 32)[sy & 7];

                    if(hFlip)
                    {
                        tileRow = __builtin_bswap32(tileRow);
                        tileRow = (tileRow & 0xF0F0F0F0) >> 4 | (tileRow & 0x0F0F0F0F) << 4;
                    }

                    // first tile
                    if(xOff)
                        tileRow >>= (xOff * 4);

                    for(int x = xOff; x < 8 && out != outEnd; x++, tileRow >>= 4, out++)
                    {
                        if(tileRow & 0xF)
                            *out = spritePal[tileRow & 0xF];
                    }

                    xOff = 0;
                }
            }
        }
    }
}

AGBDisplay::AGBDisplay(AGBCPU &cpu) : cpu(cpu), mem(cpu.getMem())
{
}

void AGBDisplay::reset()
{
    lastUpdateCycle = 0;
    y = 0;
}

void AGBDisplay::update()
{
    auto stat = mem.readIOReg(IO_DISPSTAT);

    auto passed = cpu.getCycleCount() - lastUpdateCycle;
    passed >>= 2; // 4MHz
    lastUpdateCycle += passed << 2;

    while(passed)
    {
        auto step = std::min(remainingModeDots, passed);
        passed -= step;

        remainingScanlineDots -= step;
        remainingModeDots -= step;

        if(remainingModeDots == 0)
        {
            if(remainingScanlineDots != 0)
            {
                // hblank
                remainingModeDots = remainingScanlineDots;

                if(y < screenHeight)
                {
                    if(stat & DISPSTAT_HBlankInt)
                        cpu.flagInterrupt(AGBCPU::Int_LCDHBlank);

                    cpu.triggerDMA(AGBCPU::Trig_HBlank);

                    drawScanLine(y);

                    // update affine
                    refPointX[0] += static_cast<int16_t>(mem.readIOReg(IO_BG2PB));
                    refPointY[0] += static_cast<int16_t>(mem.readIOReg(IO_BG2PD));
                    refPointX[1] += static_cast<int16_t>(mem.readIOReg(IO_BG3PB));
                    refPointY[1] += static_cast<int16_t>(mem.readIOReg(IO_BG3PD));
                }

                continue;
            }
        }
        else
            continue;
    
        // next scanline
        remainingScanlineDots = scanlineDots;
        remainingModeDots = screenWidth; // this is slightly off (hblank is a bit shorter than expected)

        y++;

        auto vCount = stat >> 8;

        if((stat & DISPSTAT_VCountInt) && y == vCount)
            cpu.flagInterrupt(AGBCPU::Int_LCDVCount);

        if(y == screenHeight)
        {
            if(stat & DISPSTAT_VBlankInt)
                cpu.flagInterrupt(AGBCPU::Int_LCDVBlank);
            cpu.triggerDMA(AGBCPU::Trig_VBlank);

            // reload affine ref
            refPointX[0] = mem.readIOReg(IO_BG2X_L) | (mem.readIOReg(IO_BG2X_H) & 0xFFF) << 16;
            refPointY[0] = mem.readIOReg(IO_BG2Y_L) | (mem.readIOReg(IO_BG2Y_H) & 0xFFF) << 16;
            refPointX[1] = mem.readIOReg(IO_BG3X_L) | (mem.readIOReg(IO_BG3X_H) & 0xFFF) << 16;
            refPointY[1] = mem.readIOReg(IO_BG3Y_L) | (mem.readIOReg(IO_BG3Y_H) & 0xFFF) << 16;

            // sign extend
            if(refPointX[0] & 0x8000000)
                refPointX[0] |= 0xF0000000;

            if(refPointY[0] & 0x8000000)
                refPointY[0] |= 0xF0000000;

            if(refPointX[1] & 0x8000000)
                refPointX[1] |= 0xF0000000;

            if(refPointY[1] & 0x8000000)
                refPointY[1] |= 0xF0000000;
        }
        else if(y >= 228)
            y = 0; // end vblank
    }
}

int AGBDisplay::getCyclesToNextUpdate() const
{
    auto passed = cpu.getCycleCount() - lastUpdateCycle;
    return (remainingModeDots * 4 - passed);
}

void AGBDisplay::setFramebuffer(uint16_t *data)
{
    screenData = data;
}

uint16_t AGBDisplay::readReg(uint32_t addr, uint16_t val)
{
    switch(addr)
    {
        case IO_DISPSTAT:
            update();
            return (y >= 160 ? DISPSTAT_VBlank : 0) | (remainingScanlineDots <= 68 ? DISPSTAT_HBlank : 0) | (mem.readIOReg(IO_DISPSTAT) & 0xFFF8); // TODO: vcount
        case IO_VCOUNT:
            update();
            return y;
    }

    return val;
}

bool AGBDisplay::writeReg(uint32_t addr, uint16_t data)
{
    if((addr & 0xFFFFFF) < 0x60/*SOUND1CNT_L*/)
        update();

    switch(addr)
    {

        // affine reference points
        case IO_BG2X_L:
            refPointX[0] = data | (refPointX[0] & 0xFFFF0000);
            break;
        case IO_BG2X_H:
        {
            auto tmp = (data & 0x800) ? (data | 0xF000) : (data & 0xFFF); // sign extend
            refPointX[0] = tmp << 16 | (refPointX[0] & 0xFFFF);
            break;
        }
        case IO_BG2Y_L:
            refPointY[0] = data | (refPointY[0] & 0xFFFF0000);
            break;
        case IO_BG2Y_H:
        {
            auto tmp = (data & 0x800) ? (data | 0xF000) : (data & 0xFFF); // sign extend
            refPointY[0] = tmp << 16 | (refPointY[0] & 0xFFFF);
            break;
        }
        case IO_BG3X_L:
            refPointX[1] = data | (refPointX[1] & 0xFFFF0000);
            break;
        case IO_BG3X_H:
        {
            auto tmp = (data & 0x800) ? (data | 0xF000) : (data & 0xFFF); // sign extend
            refPointX[1] = tmp << 16 | (refPointX[1] & 0xFFFF);
            break;
        }
        case IO_BG3Y_L:
            refPointY[1] = data | (refPointY[1] & 0xFFFF0000);
            break;
        case IO_BG3Y_H:
        {
            auto tmp = (data & 0x800) ? (data | 0xF000) : (data & 0xFFF); // sign extend
            refPointY[1] = tmp << 16 | (refPointY[1] & 0xFFFF);
            break;
        }
    }

    return false;
}

void AGBDisplay::drawScanLine(int y)
{
    auto dispControl = mem.readIOReg(IO_DISPCNT);
    auto bg0Control = mem.readIOReg(IO_BG0CNT);
    auto bg1Control = mem.readIOReg(IO_BG1CNT);
    auto bg2Control = mem.readIOReg(IO_BG2CNT);
    auto bg3Control = mem.readIOReg(IO_BG3CNT);

    auto palRAM = reinterpret_cast<uint16_t *>(mem.getPalRAM());
    auto vram = mem.getVRAM();

    auto scanLine = screenData + y * screenWidth;

    // fill with "transparent" (seems to work?)
    for(int i = 0; i < screenWidth; i++)
        scanLine[i] = palRAM[0];

    for(int priority = 3; priority >= 0; priority--)
    {
        if((dispControl & DISPCNT_BG3On) && (bg3Control & BGCNT_Priority) == priority)
            drawBG3(mem, y, scanLine, palRAM, vram, dispControl, bg3Control, refPointX[1], refPointY[1]);
        if((dispControl & DISPCNT_BG2On) && (bg2Control & BGCNT_Priority) == priority)
            drawBG2(mem, y, scanLine, palRAM, vram, dispControl, bg2Control, refPointX[0], refPointY[0]);
        if((dispControl & DISPCNT_BG1On) && (bg1Control & BGCNT_Priority) == priority)
            drawBG1(mem, y, scanLine, palRAM, vram, dispControl, bg1Control);
        if((dispControl & DISPCNT_BG0On) && (bg0Control & BGCNT_Priority) == priority)
            drawBG0(mem, y, scanLine, palRAM, vram, dispControl, bg0Control);

        if((dispControl & DISPCNT_OBJOn))
            drawOBJs(mem, y, scanLine, palRAM, vram, dispControl, priority);
    }
}
