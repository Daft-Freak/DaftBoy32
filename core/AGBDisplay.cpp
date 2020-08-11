#include <algorithm>
#include <cstring>

#include "AGBDisplay.h"

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

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
    auto scanLine = screenData + y * screenWidth;
}