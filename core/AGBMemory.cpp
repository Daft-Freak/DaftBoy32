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

void AGBMemory::setIOReadCallback(ReadCallback readCallback)
{
    this->readCallback = readCallback;
}

void AGBMemory::setIOWriteCallback(WriteCallback writeCallback)
{
    this->writeCallback = writeCallback;
}

uint8_t AGBMemory::read8(uint32_t addr) const
{
    // handle IO reads as 16-bit
    if((addr >> 24) == 0x4)
    {
        auto tmp = read16(addr & ~1);
        return (addr & 1) ? tmp >> 8 : tmp;
    }

    auto ptr = mapAddress(addr);

    if(ptr)
        return ptr[addr];

    return 0;
}

uint16_t AGBMemory::read16(uint32_t addr) const
{
    bool isIO = (addr >> 24) == 0x4;

    auto ptr = mapAddress(addr);
    uint16_t ret = 0;
    if(ptr)
        ret = *reinterpret_cast<const uint16_t *>(ptr + addr);

    // io
    if(isIO && readCallback)
        ret = readCallback(addr, ret);

    return ret;
}

void AGBMemory::write8(uint32_t addr, uint8_t data)
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

const uint8_t *AGBMemory::mapAddress(uint32_t &addr) const
{
    switch(addr >> 24)
    {
        case 0x0:
            addr &= 0x3FFF;
            return biosROM;

        case 0x2:
            addr &= 0x3FFFF;
            return ewram;
        case 0x3:
            addr &= 0x7FFF;
            return iwram;
        case 0x4:
            if(addr >= 0x4000400)
                return nullptr; // IO regs don't mirror
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
        case 0x8: // wait state 0
        case 0x9:
        case 0xA: // wait state 1
        case 0xB:
        case 0xC: // wait state 2
        case 0xD:
            addr &= 0x1FFFFFF;
            if(addr >= sizeof(cartROM))
                return nullptr;
            return cartROM;
    }

    return nullptr;
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
            if(addr >= 0x4000400)
                return nullptr; // IO regs don't mirror
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