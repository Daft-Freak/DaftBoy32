#include <cstdio>
#include <cstring>

#include "AGBMemory.h"

#include "AGBCPU.h"

AGBMemory::AGBMemory(AGBCPU &cpu) : cpu(cpu){}

void AGBMemory::setBIOSROM(const uint8_t *rom)
{
    biosROM = rom;
}

void AGBMemory::setCartROM(const uint8_t *rom, uint32_t size)
{
    cartROM = rom;
    cartROMSize = size;
}

void AGBMemory::reset()
{
}

uint8_t AGBMemory::read8(uint32_t addr) const
{
    return *mapAddress(addr);
}

uint16_t AGBMemory::read16(uint32_t addr) const
{
    auto ptr = mapAddress(addr);
    uint16_t ret = *reinterpret_cast<const uint16_t *>(ptr);

    // EEPROM (or could just be a big cart...)
    if((addr >> 24) == 0xD)
        return eepromOutBits[(addr & 0xFF) >> 1];

    // io
    if((addr >> 24) == 0x4)
        ret = cpu.readReg(addr & 0xFFFFFF, ret);

    return ret;
}

uint32_t AGBMemory::read32(uint32_t addr) const
{
    return *reinterpret_cast<const uint32_t *>(mapAddress(addr));
}

void AGBMemory::write8(uint32_t addr, uint8_t data)
{
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
    if((addr >> 24) == 0x4 && cpu.writeReg(addr & 0xFFFFFF, data))
        return;

    // EEPROM
    if((addr >> 24) == 0xD)
    {
        eepromInBits[(addr & 0xFF) >> 1] = data;

        if((addr & 0xFF) == 0x10 && eepromInBits[0] && eepromInBits[1]) // end of read request for 512b
        {
            uint16_t eepromAddr = (eepromInBits[2] << 5) | (eepromInBits[3] << 4) | (eepromInBits[4] << 3)
                                | (eepromInBits[5] << 2) | (eepromInBits[6] << 1) | eepromInBits[7];

            uint64_t data = reinterpret_cast<uint64_t *>(eepromData)[eepromAddr];

            for(int i = 0; i < 64; i++)
                eepromOutBits[i + 4] = (data & (1ull << (63 - i))) ? 1 : 0;
        }
        else if((addr & 0xFF) == 0x90 && eepromInBits[0] && !eepromInBits[1])
        {
            uint16_t eepromAddr = (eepromInBits[2] << 5) | (eepromInBits[3] << 4) | (eepromInBits[4] << 3)
                                | (eepromInBits[5] << 2) | (eepromInBits[6] << 1) | eepromInBits[7];

            uint64_t data = 0;

            for(int i = 0; i < 64; i++)
                data |= static_cast<uint64_t>(eepromInBits[i + 8]) << (63 - i);

            reinterpret_cast<uint64_t *>(eepromData)[eepromAddr] = data;

            eepromOutBits[0] = 1;
        }
        return;
    }

    auto ptr = mapAddress(addr);
    if(ptr)
    {
        *reinterpret_cast<uint16_t *>(ptr) = data;
        return;
    }

    //printf("write %x = %x\n", addr, data);
}

void AGBMemory::write32(uint32_t addr, uint32_t data)
{
    auto ptr = mapAddress(addr);
    if(ptr)
    {
        *reinterpret_cast<uint32_t *>(ptr) = data;
        return;
    }
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
            if(addr >= cartROMSize)
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