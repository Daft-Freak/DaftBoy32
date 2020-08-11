#include <cstdio>
#include <cstring>

#include "AGBMemory.h"

void AGBMemory::loadBIOSROM(const uint8_t *rom)
{
    memcpy(biosROM, rom, 0x4000);
}

void AGBMemory::reset()
{
}

void AGBMemory::setReadCallback(ReadCallback readCallback)
{
    this->readCallback = readCallback;
}

void AGBMemory::setWriteCallback(WriteCallback writeCallback)
{
    this->writeCallback = writeCallback;
}

uint8_t AGBMemory::read(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case 0x0:
            return biosROM[addr & 0x3FFF];

        case 0x2:
            return ewram[addr & 0x3FFFF];
        case 0x3:
            return iwram[addr & 0x7FFF];

        case 0x4:
        {
            auto val = ioRegs[addr & 0x3FF];
            if(readCallback)
                val = readCallback(addr, val);
            return val;
        }

        case 0x5:
            return palRAM[addr & 0x3FF];
        case 0x6:
            return vram[addr & 0x1FFFF]; // 18000-1FFFF is bad! 
        case 0x7:
            return oam[addr & 0x3FF];

        case 0x8: // wait state 0
        case 0x9:
        case 0xA: // wait state 1
        case 0xB:
        case 0xC: // wait state 2
        case 0xD:
            if((addr & 0x1FFFFFF) >= sizeof(cartROM))
                return 0;
            return cartROM[addr & 0x1FFFFFF];
    }

    //printf("read %x\n", addr);
    return 0;
}

void AGBMemory::write(uint32_t addr, uint8_t data)
{
    // io
    if((addr >> 24) == 0x4 && writeCallback && writeCallback(addr, data))
        return;

    auto ptr = mapAddress(addr);
    if(ptr)
    {
        ptr[addr] = data;
        return;
    }

    //printf("write %x = %x\n", addr, data);
}

uint8_t *AGBMemory::mapAddress(uint32_t &addr)
{
    switch(addr >> 24)
    {
        case 0x0:
            addr &= 0x3FFF;
            return nullptr; // bios rom

        case 0x2:
            addr &= 0x3FFFF;
            return ewram;
        case 0x3:
            addr &= 0x7FFF;
            return iwram;
        case 0x4:
            addr &= 0x3FF;
            return ioRegs;
        case 0x5:
            addr &= 0x3FF;
            return palRAM;
        case 0x6:
            addr &= 0x1FFFF; //? too much
            return vram;
        case 0x7:
            addr &= 0x3FF;
            return oam;

    }

    return nullptr;
}