#include <algorithm>
#include <cstring>

#include "AGBDisplay.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

static void drawBG2(int y, uint16_t *scanLine, uint16_t *palRam, uint8_t *vram, uint16_t dispControl, uint16_t control)
{
    switch(dispControl & DISPCNT_Mode)
    {
        // 0-2...
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
    auto bg2Control = mem.readIOReg(IO_BG2CNT);

    auto palRAM = reinterpret_cast<uint16_t *>(mem.getPalRAM());
    auto vram = mem.getVRAM();

    auto scanLine = screenData + y * screenWidth;

    for(int priority = 3; priority >= 0; priority--)
    {
        if((dispControl & DISPCNT_BG2On) && (bg2Control & BGCNT_Priority) == priority)
            drawBG2(y, scanLine, palRAM, vram, dispControl, bg2Control);
    }
}
