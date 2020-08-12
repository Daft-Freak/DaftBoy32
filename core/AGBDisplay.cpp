#include <algorithm>
#include <cstring>

#include "AGBDisplay.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

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

    if(control & BGCNT_SinglePal)
        return; // TODO

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

        // TODO: x/y flip

        // 4bit tiles
        uint32_t tileRow = reinterpret_cast<uint32_t *>(charPtr + (tileMeta & 0x3FF) * 32)[ty];

        tileRow >>= (tx * 4);

        for(; tx < 8 && x < 240; tx++, x++, tileRow >>= 4)
        {
            auto palIndex = ((tileMeta & 0xF000) >> 8) | (tileRow & 0xF);
            *scanLine++ = palRam[palIndex];
        }
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

static void drawBG2(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    switch(dispControl & DISPCNT_Mode)
    {
        case 0: // "text" mode
            drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG2HOFS), mem.readIOReg(IO_BG2VOFS));
            break;
        // 1-2...
        case 3: // 16-bit fullscreen bitmap
        {
            // TODO: rot/scale
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
            // TODO: rot/scale
            auto outPtr = scanLine;
            if(y < 128)
            {
                auto inPtr = reinterpret_cast<uint16_t *>(vram + y * 160 * 2);
                if(dispControl & DISPCNT_Frame)
                    inPtr += (0xA000 / 2);

                int x;
                for(x = 0; x < 160; x++)
                    *outPtr++ = *inPtr++;

                // fill rest with pal 0? (works for tonc demo)
                for(;x < 240; x++)
                    *outPtr++ = palRam[0];
            }
            else
                for(int x = 0; x < 240; x++)
                    *outPtr++ = palRam[0];
            break;
        }
        default:
            memset(scanLine, 0, 240 * 2);
    }
}

static void drawBG3(AGBMemory &mem, int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    if((dispControl & DISPCNT_Mode) == 0)
        drawTextBG(y, scanLine, palRam, vram, dispControl, control, mem.readIOReg(IO_BG3HOFS), mem.readIOReg(IO_BG3VOFS));
    // else 2
}

AGBDisplay::AGBDisplay(AGBCPU &cpu) : cpu(cpu), mem(cpu.getMem())
{
}

void AGBDisplay::update(int cycles)
{
    auto stat = mem.readIOReg(IO_DISPSTAT);

    remainingScanlineDots -= cycles;

    if(y < screenHeight)
    {
        if(remainingScanlineDots == 68)
        {
            // hblank
            // TODO? could miss this if incrementing by > 1...
            if(stat & DISPSTAT_HBlankInt)
                cpu.flagInterrupt(AGBCPU::Int_LCDHBlank);
        }
    }

    if(remainingScanlineDots > 0)
        return;

    // next scanline
    remainingScanlineDots += scanlineDots;

    if(y < screenHeight)
        drawScanLine(y);

    y++;

    auto vCount = stat >> 8;

    if((stat & DISPSTAT_VCountInt) && y == vCount)
        cpu.flagInterrupt(AGBCPU::Int_LCDVCount);

    if(y == screenHeight)
        cpu.flagInterrupt(AGBCPU::Int_LCDVBlank);
    else if(y >= 228)
        y = 0; // end vblank
}

uint16_t AGBDisplay::readReg(uint32_t addr, uint16_t val)
{
    switch(addr)
    {
        case IO_DISPSTAT:
            return (y >= 160 ? DISPSTAT_VBlank : 0) | (remainingScanlineDots <= 68 ? DISPSTAT_HBlank : 0); // TODO: vcount
        case IO_VCOUNT:
            return y;
    }

    return val;
}

bool AGBDisplay::writeReg(uint32_t addr, uint8_t data)
{
    switch(addr)
    {

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

    for(int priority = 3; priority >= 0; priority--)
    {
        if((dispControl & DISPCNT_BG3On) && (bg3Control & BGCNT_Priority) == priority)
            drawBG3(mem, y, scanLine, palRAM, vram, dispControl, bg3Control);
        if((dispControl & DISPCNT_BG2On) && (bg2Control & BGCNT_Priority) == priority)
            drawBG2(mem, y, scanLine, palRAM, vram, dispControl, bg2Control);
        if((dispControl & DISPCNT_BG1On) && (bg1Control & BGCNT_Priority) == priority)
            drawBG1(mem, y, scanLine, palRAM, vram, dispControl, bg1Control);
        if((dispControl & DISPCNT_BG0On) && (bg0Control & BGCNT_Priority) == priority)
            drawBG0(mem, y, scanLine, palRAM, vram, dispControl, bg0Control);
    }
}
