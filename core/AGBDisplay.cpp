#include <algorithm>
#include <cstring>

#include "AGBDisplay.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

enum LayerEnabled
{
    Layer_BG0       = (1 << 0),
    Layer_BG1       = (1 << 1),
    Layer_BG2       = (1 << 2),
    Layer_BG3       = (1 << 3),
    Layer_OBJ       = (1 << 4),
};

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
    auto validDataEnd = vram + 0x10000;

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

        uint16_t tileMeta = 0;
    
        // screen base pointing past first 64k is invalid
        // reading as zero matches what the DS does
        if(reinterpret_cast<uint8_t *>(tilePtr) < validDataEnd)
            tileMeta = tilePtr[bx];

        // v flip
        int tty = ty;
        if(tileMeta & (1 << 11))
            tty = 7 - ty;

        if(control & BGCNT_SinglePal)
        {
            // 8bit tiles
            uint64_t tileRow = 0;
            auto thisTilePtr = charPtr + (tileMeta & 0x3FF) * 64;

            if(thisTilePtr < validDataEnd)
                tileRow = reinterpret_cast<uint64_t *>(thisTilePtr)[tty];

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
                        *scanLine = palRam[palIndex] | 0x8000;
                    else
                        *scanLine = 0;
                }
            }
            else
            {
                // edges
                for(; tx < 8 && x < 240; tx++, x++, scanLine++)
                {
                    auto palIndex = *tilePtr++;
                    if(palIndex)
                        *scanLine = palRam[palIndex] | 0x8000;
                    else
                        *scanLine = 0;
                }
            }
        }
        else
        {
            // 4bit tiles
            uint32_t tileRow = 0;
            auto thisTilePtr = charPtr + (tileMeta & 0x3FF) * 32;
            // char base pointing past first 64k is invalid
            if(thisTilePtr < validDataEnd)
                tileRow = reinterpret_cast<uint32_t *>(thisTilePtr)[tty];
            // this is what the GBA would do, but the DS just reads 0 so let's avoid some code here
            //else
            //    tileRow = (tileMeta & 0x3FF) | (tileMeta & 0x3FF) << 16;

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
                        *scanLine = tilePal[tileRow & 0xF] | 0x8000;
                    else
                        *scanLine = 0;
                }
            }
            else
            {
                // edges
                tileRow >>= (tx * 4);

                for(; tx < 8 && x < 240; tx++, x++, tileRow >>= 4, scanLine++)
                {
                    if(tileRow & 0xF)
                        *scanLine = tilePal[tileRow & 0xF] | 0x8000;
                    else
                        *scanLine = 0;
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
    auto validDataEnd = vram + 0x10000;

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
            {
                *scanLine = 0;
                continue;
            }
        }

        if(by < 0 || by >= numTiles)
        {
            if(wrap)
                by &= numTiles - 1;
            else
            {
                *scanLine = 0;
                continue;
            }
        }

        auto tilePtr = screenPtr + by * numTiles;

        uint8_t tileIndex = 0;
        
        if(tilePtr < validDataEnd)
            tileIndex = tilePtr[bx];

        // 8bit tiles
        uint64_t tileRow = 0;
        auto thisTilePtr = charPtr + tileIndex * 64;

        if(thisTilePtr < validDataEnd)
            tileRow = reinterpret_cast<uint64_t *>(thisTilePtr)[ty];

        auto tileDataPtr = reinterpret_cast<uint8_t *>(&tileRow) + tx;

        auto palIndex = *tileDataPtr++;
        if(palIndex)
            *scanLine = palRam[palIndex] | 0x8000;
        else
            *scanLine = 0;
    }
}

// these two are always "text" mode
static void drawBG0(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    if((dispControl & DISPCNT_Mode) > 1)
    {
        memset(scanLine, 0, 240 * 2);
        return;
    }

    drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG0HOFS), mem.readIOReg(IO_BG0VOFS));
}

static void drawBG1(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    if((dispControl & DISPCNT_Mode) > 1)
    {
        memset(scanLine, 0, 240 * 2);
        return;
    }

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
            auto inPtr = reinterpret_cast<uint16_t *>(vram);
            auto outPtr = scanLine;

            int curX = refPointX;
            int curY = refPointY;
            int16_t a = mem.readIOReg(IO_BG2PA), c = mem.readIOReg(IO_BG2PC);

            for(int x = 0; x < 240; x++, outPtr++, curX += a, curY += c)
            {
                int px = curX >> 8;
                int py = curY >> 8;
                if(px < 0 || px >= 240 || py < 0 || py >= 160)
                    *outPtr = 0;
                else
                    *outPtr = inPtr[px + py * 240] | 0x8000;
            }

            break;
        }
        case 4: // paletted fullscreen bitmap
        {
            auto inPtr = vram;
            auto outPtr = scanLine;

            if(dispControl & DISPCNT_Frame)
                inPtr += 0xA000;

            int curX = refPointX;
            int curY = refPointY;
            int16_t a = mem.readIOReg(IO_BG2PA), c = mem.readIOReg(IO_BG2PC);

            for(int x = 0; x < 240; x++, outPtr++, curX += a, curY += c)
            {
                int px = curX >> 8;
                int py = curY >> 8;
                if(px < 0 || px >= 240 || py < 0 || py >= 160)
                    *outPtr = 0;
                else
                    *outPtr = palRam[inPtr[px + py * 240]] | 0x8000;
            }
            break;
        }
        case 5: // 16-bit 160*128 bitmap
        {
            auto inPtr = reinterpret_cast<uint16_t *>(vram);
            auto outPtr = scanLine;

            if(dispControl & DISPCNT_Frame)
                inPtr += (0xA000 / 2);

            int curX = refPointX;
            int curY = refPointY;
            int16_t a = mem.readIOReg(IO_BG2PA), c = mem.readIOReg(IO_BG2PC);

            for(int x = 0; x < 240; x++, outPtr++, curX += a, curY += c)
            {
                int px = curX >> 8;
                int py = curY >> 8;
                if(px < 0 || px >= 160 || py < 0 || py >= 128)
                    *outPtr = 0;
                else
                    *outPtr = inPtr[px + py * 160] | 0x8000;
            }
            break;
        }
        default:
            memset(scanLine, 0, 240 * 2);
            return;
    }
}

static void drawBG3(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control, int32_t refPointX, int32_t refPointY)
{
    if((dispControl & DISPCNT_Mode) == 0)
        drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG3HOFS), mem.readIOReg(IO_BG3VOFS));
    else if((dispControl & DISPCNT_Mode) == 2)
        drawAffineBG(scanLine, palRam, vram, dispControl, control, refPointX, refPointY, mem.readIOReg(IO_BG3PA), mem.readIOReg(IO_BG3PC));
    else
        memset(scanLine, 0, 240 * 2);
}

static void drawOBJs(AGBMemory &mem, int y, uint16_t scanLine[5][240], uint16_t *palRam, uint8_t *vram, uint16_t dispControl)
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

    bool isBitmapMode = (dispControl & DISPCNT_Mode) > 2;

    // entire scanline, or just visible if "h-blank free"
    int cyclesRemaining = ((dispControl & DISPCNT_HBlankFree) ? 240 : 308) * 4 - 6;

    // seems to be the bit missing from the gbatek description to make the DUST tests pass
    // TODO: the offscreen test is still slightly off
    const int inactiveCycles = 2;

    for(int i = 0; i < 128 && cyclesRemaining > 0; i++)
    {
        const uint16_t attr0 = oam[i * entrySize + 0];
        const uint16_t attr1 = oam[i * entrySize + 1];
        const uint16_t attr2 = oam[i * entrySize + 2];

        auto mode = (attr0 & Attr0_Mode) >> 8;

        if(mode == 2/*disable*/)
        {
            cyclesRemaining -= inactiveCycles;
            continue;
        }

        // check y
        int spriteY = attr0 & Attr0_Y;

        auto shape = (attr0 & Attr0_Shape) >> 12;
        int sizeShape = (shape ? shape ^ 0xC : 0) | ((attr1 & Attr1_Size) >> 14);
        int spriteH = sizes[sizeShape];

        // possibly doubled size
        int doubledH = mode == 3 ? spriteH * 2 : spriteH;

        // wrap
        if(((spriteY + doubledH - 1) & 0xFF) < spriteY)
            spriteY -= 256;

        if(spriteY > y)
        {
            cyclesRemaining -= inactiveCycles;
            continue;
        }

        if(spriteY + doubledH <= y)
        {
            cyclesRemaining -= inactiveCycles;
            continue;
        }

        // get X/W
        int spriteX = attr1 & Attr1_X;
        sizeShape = shape | ((attr1 & Attr1_Size) >> 14);
        const int spriteW = sizes[sizeShape];

        // wrap
        if(((spriteX + spriteW) & 0x1FF) < spriteX)
            spriteX -= 512;

        if(spriteX >= 240)
        {
            cyclesRemaining -= inactiveCycles;
            continue;
        }

        // calc offsets
        auto startTile = attr2 & Attr2_Index;
        int sx = std::max(0, -spriteX);
        int sy = y - spriteY;

        // first half of "object" VRAM is not usable for objects in bitmap modes
        // TODO: what if part of the sprite is in valid RAM?
        if(isBitmapMode && startTile < 512)
        {
            cyclesRemaining -= inactiveCycles;
            // TODO: still takes time to draw?
            continue;
        }

        int priority = (attr2 & Attr2_Priority) >> 10;
        int effect = (attr0 & Attr0_Effect) >> 10;

        if(effect == 2 /*window*/)
            priority = 4; // fake extra priority level

        // palette
        auto spritePal = palRam + 256;

        if(mode != 0)
        {
            // overhead for affine + offscreen pixels
            cyclesRemaining -= 10;// + sx * 2;
            if(cyclesRemaining <= 0)
                break;

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

            auto out = scanLine[priority] + (spriteX + sx);
            auto outEnd = scanLine[priority] + 240;

            if(!(attr0 & Attr0_SinglePal))
                spritePal += ((attr2 & Attr2_Pal) >> 8);

            for(int x = sx; x < halfW * 2 && out != outEnd && cyclesRemaining > 1; x++, out++, tx += a, ty += c)
            {
                cyclesRemaining -= 2;

                if(tx < 0 || ty < 0 || tx >= (spriteW << 8) || ty >= (spriteH << 8))
                    continue;

                if(*out)
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
                        *out = spritePal[*tilePtr] | 0x8000;
                }
                else
                {
                    uint32_t tileRow = reinterpret_cast<uint32_t *>(charPtr + (tile + (tx >> 11)) * 32)[(ty >> 8) & 7];
                    tileRow >>= ((tx >> 8) & 7) * 4;

                    if(tileRow & 0xF)
                        *out = spritePal[tileRow & 0xF] | 0x8000;
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
            auto out = scanLine[priority] + (spriteX + sx);
            auto outEnd = scanLine[priority] + 240;

            int tilesX = spriteW >> 3;

            if(attr0 & Attr0_SinglePal)
            {
                // 8bit tiles
                for(int stx = sx >> 3; stx < tilesX && out != outEnd && cyclesRemaining; stx++)
                {
                    int tileX = hFlip ? (tilesX - 1) - stx : stx;
                    auto tilePtr = charPtr + startTile * 32 + tileX * 64 + (sy & 7) * 8;

                    if(hFlip)
                        tilePtr += 7 - xOff;
                    else
                        tilePtr += xOff;

                    // pixels in tile
                    for(int x = xOff; x < 8 && out != outEnd && cyclesRemaining; x++, out++)
                    {
                        cyclesRemaining--;

                        auto palIndex = *tilePtr;
                        if(!*out && palIndex)
                            *out = spritePal[palIndex] | 0x8000;

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

                for(int stx = sx >> 3; stx < tilesX && out != outEnd && cyclesRemaining; stx++)
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

                    for(int x = xOff; x < 8 && out != outEnd && cyclesRemaining; x++, tileRow >>= 4, out++)
                    {
                        cyclesRemaining--;
                        if(!*out && (tileRow & 0xF))
                            *out = spritePal[tileRow & 0xF] | 0x8000;
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
    auto passed = cpu.getCycleCount() - lastUpdateCycle;
    passed >>= 2; // 4MHz

    if(passed < remainingModeDots)
        return;

    lastUpdateCycle += passed << 2;

    auto stat = mem.readIOReg(IO_DISPSTAT);

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

                if(stat & DISPSTAT_HBlankInt)
                    cpu.flagInterrupt(AGBCPU::Int_LCDHBlank);

                // window update
                auto control = mem.readIOReg(IO_DISPCNT);
                if(control & DISPCNT_Window0On)
                {
                    auto winV = mem.readIOReg(IO_WIN0V);
                    if(y == (winV & 0xFF))
                        yInWin0 = false;
                    else if(y == winV >> 8)
                        yInWin0 = true;
                }

                if(control & DISPCNT_Window1On)
                {
                    auto winV = mem.readIOReg(IO_WIN1V);
                    if(y == (winV & 0xFF))
                        yInWin1 = false;
                    else if(y == winV >> 8)
                        yInWin1 = true;
                }

                if(y < screenHeight)
                {
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

int AGBDisplay::getCyclesToNextUpdate(uint32_t cycleCount) const
{
    auto passed = cycleCount - lastUpdateCycle;
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
        case IO_DISPCNT:
            return val &~DISPCNT_CGBMode; // really only writable from BIOS
        case IO_DISPSTAT:
            update();
            return (y >= 160 && y != 227 ? DISPSTAT_VBlank : 0) | 
                   (remainingScanlineDots <= 68 ? DISPSTAT_HBlank : 0) |
                   (y == (mem.readIOReg(IO_DISPSTAT) >> 8) ? DISPSTAT_VCount : 0) |
                   (mem.readIOReg(IO_DISPSTAT) & 0xFFF8);
        case IO_VCOUNT:
            update();
            return y;

        case IO_BG0CNT:
        case IO_BG1CNT:
            return val & ~BGCNT_Wrap;
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

    // forced blank, display white
    if(dispControl & DISPCNT_ForceBlank)
    {
        for(int i = 0; i < screenWidth; i++)
            scanLine[i] = 0x7FFF;
        return;
    }

    uint16_t bgData[4][screenWidth];
    uint16_t objData[5][screenWidth]{0}; // four priority levels for objs +1 for obj window

    int layerEnables = dispControl >> 8;
    bool windowEnabled = false;

    // get enabled layers for window
    // (avoid some work for lines outside the window)
    const int anyWindowEnabled = DISPCNT_Window0On | DISPCNT_Window1On | DISPCNT_OBJWindowOn;
    uint16_t winIn, winOut;
    uint16_t win0h = 0, win1h = 0;
    if(dispControl & anyWindowEnabled)
    {
        windowEnabled = true;
        winIn = mem.readIOReg(IO_WININ);
        winOut = mem.readIOReg(IO_WINOUT);

        win0h = mem.readIOReg(IO_WIN0H);
        win1h = mem.readIOReg(IO_WIN1H);

        int winLayers = winOut; // outside

        if((dispControl & DISPCNT_Window0On) && yInWin0)
            winLayers |= winIn;

        if((dispControl & DISPCNT_Window1On) && yInWin1)
            winLayers |= winIn >> 8;

        if((dispControl & DISPCNT_OBJWindowOn)) // assume inside for now
            winLayers |= winOut >> 8 | Layer_OBJ; // need to draw objects for the window even if they're not displayed anywhere

        layerEnables &= winLayers;
    }

    // get priority/active layers
    int layerPriority[] {
        bg0Control & BGCNT_Priority,
        bg1Control & BGCNT_Priority,
        bg2Control & BGCNT_Priority,
        bg3Control & BGCNT_Priority
    };

    int numActiveLayers = 0;
    std::tuple<uint16_t *, int> layers[9];

    for(int priority = 0; priority < 4; priority++)
    {
        // objects
        if(layerEnables & Layer_OBJ)
            layers[numActiveLayers++] = {objData[priority], Layer_OBJ};

        // background layers
        for(int l = 0; l < 4; l++)
        {
            if((layerEnables & (1 << l)) && layerPriority[l] == priority)
                layers[numActiveLayers++] = {bgData[l], 1 << l};
        }
    }

    layers[numActiveLayers++] = {nullptr, 0};

    // draw enabled layers
    if(layerEnables & Layer_BG0)
        drawBG0(mem, y, bgData[0], palRAM, vram, dispControl, bg0Control);

    if(layerEnables & Layer_BG1)
        drawBG1(mem, y, bgData[1], palRAM, vram, dispControl, bg1Control);

    if(layerEnables & Layer_BG2)
        drawBG2(mem, y, bgData[2], palRAM, vram, dispControl, bg2Control, refPointX[0], refPointY[0]);

    if(layerEnables & Layer_BG3)
        drawBG3(mem, y, bgData[3], palRAM, vram, dispControl, bg3Control, refPointX[1], refPointY[1]);

    if(layerEnables & Layer_OBJ)
        drawOBJs(mem, y, objData, palRAM, vram, dispControl);

    // start with window enabled if start/end are flipped
    bool xInWin0 = (win0h & 0xFF) < (win0h >> 8), xInWin1 = (win1h & 0xFF) < (win1h >> 8);

    // blending setup
    auto blendControl = mem.readIOReg(IO_BLDCNT);
    uint16_t blendAlpha;
    uint16_t blendY;

    if(blendControl & BLDCNT_Effect)
    {
        blendAlpha = mem.readIOReg(IO_BLDALPHA);
        blendY = mem.readIOReg(IO_BLDY);
    }

    // merge layers
    for(int x = 0; x < screenWidth; x++)
    {
        // TODO: maybe don't do this every pixel
        int curLayerEnables = layerEnables;

        int blendMode = (blendControl & BLDCNT_Effect) >> 6;

        if(windowEnabled)
        {
            if(x == (win0h & 0xFF))
                xInWin0 = false;
            else if(x == win0h >> 8)
                xInWin0 = true;

            if(x == (win1h & 0xFF))
                xInWin1 = false;
            else if(x == win1h >> 8)
                xInWin1 = true;    

            if((dispControl & DISPCNT_Window0On) && yInWin0 && xInWin0)
            {
                curLayerEnables &= winIn;
                if(!(winIn & WININ_Win0Effect))
                    blendMode = 0;
            }
            else if((dispControl & DISPCNT_Window1On) && yInWin1 && xInWin1)
            {
                curLayerEnables &= (winIn >> 8);
                if(!(winIn & WININ_Win1Effect))
                    blendMode = 0;
            }
            else if((dispControl & DISPCNT_OBJWindowOn) && objData[4][x])
            {
                curLayerEnables &= (winOut >> 8);
                if(!(winOut & WINOUT_ObjWinEffect))
                    blendMode = 0;
            }
            else //outside
            {
                curLayerEnables &= winOut;
                if(!(winOut & WINOUT_OutsideEffect))
                    blendMode = 0;
            }
        }

        for(int i = 0; i < 9; i++)
        {
            uint16_t *data;
            int mask;
            std::tie(data, mask) = layers[i];

            // backdrop
            if(!mask)
            {
                // TODO: light/dark blend
                scanLine[x] = palRAM[0];
                break;
            }

            // check if disabled by window
            if(!(curLayerEnables & mask))
                continue;
            
            if(data[x])
            {
                // blend enabled and layer is src (masks conveniently line up)
                if(blendMode && (blendControl & mask))
                {
                    // TODO: sprite blend override

                    int srcR = (data[x] >> 10) & 0x1F;
                    int srcG = (data[x] >> 5) & 0x1F;
                    int srcB = data[x] & 0x1F;

                    if(blendMode == 1) // alpha
                    {
                        // get next layer
                        uint16_t *nextData;
                        int nextMask;
                        int nextLayer = i + 1;

                        // find next non-transparent layer (also not the smae layer (objects))
                        do
                        {
                            std::tie(nextData, nextMask) = layers[nextLayer++];
                        } while(nextMask && (nextMask == mask || !(curLayerEnables & nextMask) || !nextData[x]));

                        nextMask = nextMask ? nextMask << 8 : (1 << 13); // shift up, handle 0 for backdrop

                        // check if its a dst target
                        if(!(blendControl & nextMask))
                        {
                            scanLine[x] = data[x];
                            break;
                        }

                        // do blend
                        int srcA = std::min(16, blendAlpha & 0x1F);

                        auto dstCol = nextData ? nextData[x] : palRAM[0]; // handle backdrop

                        int dstR = (dstCol >> 10) & 0x1F;
                        int dstG = (dstCol >> 5) & 0x1F;
                        int dstB = dstCol & 0x1F;
                        int dstA = std::min(16, (blendAlpha >> 8) & 0x1F);

                        int r = std::min(31, (srcR * srcA + dstR * dstA) / 16);
                        int g = std::min(31, (srcG * srcA + dstG * dstA) / 16);
                        int b = std::min(31, (srcB * srcA + dstB * dstA) / 16);

                        scanLine[x] = r << 10 | g << 5 | b;
                    }
                    else if(blendMode == 2) // lighten
                    {
                        int evy = std::min(16, blendY & 0x1F);

                        int r = srcR + ((31 - srcR) * evy) / 16;
                        int g = srcG + ((31 - srcG) * evy) / 16;
                        int b = srcB + ((31 - srcB) * evy) / 16;

                        scanLine[x] = r << 10 | g << 5 | b;
                    }
                    else if(blendMode == 3) // darken
                    {
                        int evy = std::min(16, blendY & 0x1F);

                        int r = srcR - (srcR * evy) / 16;
                        int g = srcG - (srcG * evy) / 16;
                        int b = srcB - (srcB * evy) / 16;

                        scanLine[x] = r << 10 | g << 5 | b;
                    }
                    else
                        scanLine[x] = data[x];
                }
                else
                    scanLine[x] = data[x];

                break;
            }
        }
    }
}
