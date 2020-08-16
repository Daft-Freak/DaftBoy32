#include <cstdio>
#include <cstring>

#include "AGBMemory.h"

void AGBMemory::setBIOSROM(const uint8_t *rom)
{
    biosROM = rom;
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

    return *mapAddress(addr);
}

uint16_t AGBMemory::read16(uint32_t addr) const
{
    auto ptr = mapAddress(addr);
    uint16_t ret = *reinterpret_cast<const uint16_t *>(ptr);

    // io
    if((addr >> 24) == 0x4 && readCallback)
        ret = readCallback(addr & 0xFFFFFF, ret);

    return ret;
}

void AGBMemory::write8(uint32_t addr, uint8_t data)
{
    // handle IO writes as 16-bit
    if((addr >> 24) == 0x4)
    {
        auto tmp = read8(addr ^ 1);
        addr & 1 ? write16(addr, tmp | (data << 8)) : write16(addr, (tmp << 8) | data);
        return;
    }

    if((addr >> 24) == 0x7 || ((addr >> 24) == 0x6 && (addr &  0x1FFFF) >= 0x10000)) // OAM / OBJ VRAM ignores byte writes
        return;

    if((addr >> 24) == 0x5 || ((addr >> 24) == 0x6 && (addr &  0x1FFFF) < 0x10000)) // pal / BG VRAM
    {
        write16(addr & ~1, data | (data << 8));
        return;
    }

    auto ptr = mapAddress(addr);
    if(ptr)
    {
        *ptr = data;
        return;
    }

    //printf("write %x = %x\n", addr, data);
}

void AGBMemory::write16(uint32_t addr, uint16_t data)
{
    // io
    if((addr >> 24) == 0x4 && writeCallback && writeCallback(addr & 0xFFFFFF, data))
        return;

    auto ptr = mapAddress(addr);
    if(ptr)
    {
        *reinterpret_cast<uint16_t *>(ptr) = data;
        return;
    }

    //printf("write %x = %x\n", addr, data);
}

const uint8_t *AGBMemory::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case 0x0:
            return biosROM + (addr & 0x3FFF);

        case 0x2:
            return ewram + (addr & 0x3FFFF);
        case 0x3:
            return iwram + (addr & 0x7FFF);
        case 0x4:
            if(addr >= 0x4000400)
                return reinterpret_cast<const uint8_t *>(&dummy); // IO regs don't mirror
            return ioRegs + (addr & 0x3FF);
        case 0x5:
            return palRAM + (addr & 0x3FF);
        case 0x6:
            addr &= 0x1FFFF;
            if(addr >= 0x18000)
                addr &= ~0x8000; // last 32K is the previous 32K
            return vram + addr;
        case 0x7:
            return oam + (addr & 0x3FF);
        case 0x8: // wait state 0
        case 0x9:
        case 0xA: // wait state 1
        case 0xB:
        case 0xC: // wait state 2
        case 0xD:
            addr &= 0x1FFFFFF;
            if(addr >= sizeof(cartROM))
                return reinterpret_cast<const uint8_t *>(&dummy);
            return cartROM + addr;
    }

    return reinterpret_cast<const uint8_t *>(&dummy);
}

uint8_t *AGBMemory::mapAddress(uint32_t addr)
{
    switch(addr >> 24)
    {
        case 0x0:
            return nullptr; // bios rom

        case 0x2:
            return ewram + (addr & 0x3FFFF);
        case 0x3:
            return iwram + (addr & 0x7FFF);
        case 0x4:
            if(addr >= 0x4000400)
                return nullptr; // IO regs don't mirror
            return ioRegs + (addr & 0x3FF);
        case 0x5:
            return palRAM + (addr & 0x3FF);
        case 0x6:
            addr &= 0x1FFFF;
            if(addr >= 0x18000)
                addr &= ~0x8000; // last 32K is the previous 32K
            return vram + addr;
        case 0x7:
            return oam + (addr & 0x3FF);
    }

    return nullptr;
}

int AGBMemory::getAccessCycles(uint32_t addr, int width, bool sequential) const
{
    switch(addr >> 24)
    {
        case 0x0: // BIOS
        case 0x3: // IWRAM
        case 0x4: // IO
        case 0x7: // OAM
            return 1;

        case 0x2: // EWRAM
            return width == 4 ? 6 : 3;

        case 0x5: // pal
        case 0x6: // VRAM
            return width == 4 ? 2 : 1;

        // defaults TODO: WAITCNT
        case 0x8: // wait state 0
        case 0x9:
            return (sequential ? 3 : 5) * (width == 4 ? 2 : 1);
        case 0xA: // wait state 1
        case 0xB:
            return (sequential ? 5 : 5) * (width == 4 ? 2 : 1);
        case 0xC: // wait state 2
        case 0xD:
            return (sequential ? 9 : 5) * (width == 4 ? 2 : 1);
    }

    return 1;
}