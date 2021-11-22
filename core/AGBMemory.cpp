#include <cstdio>
#include <cstring>

#include "AGBMemory.h"

#include "AGBCPU.h"
#include "AGBRegs.h"

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

void AGBMemory::loadCartridgeSave(const uint8_t *data, uint32_t len)
{
    memcpy(cartSaveData, data, len);

    // determine the type of save from the size
    if(len == 512 || len == 4 * 1024)
        saveType = SaveType::EEPROM; // TODO: 4k
    else if(len == 32 * 1024)
        saveType = SaveType::RAM;
    else if(len == 64 * 1024 || len == 128 * 1024)
        saveType = SaveType::Flash;
}

void AGBMemory::reset()
{
    saveType = SaveType::Unknown;
    flashState = FlashState::Read;
    flashBank = 0;

    memset(cartSaveData, 0xFF, sizeof(cartSaveData));

    cartAccessN[0] = 5;
    cartAccessS[0] = 3;
    cartAccessN[1] = 5;
    cartAccessS[1] = 5;
    cartAccessN[2] = 5;
    cartAccessS[2] = 9;

    cartAccessN[3] = cartAccessS[3] = 5;
}

uint8_t AGBMemory::read8(uint32_t addr) const
{
    auto ptr = mapAddress(addr);
    // out of bounds rom access returns low bits of address
    if(!ptr && (addr & 0x8000000))
        return addr >> (1 + (addr & 1) * 8);

    return *ptr;
}

uint16_t AGBMemory::read16(uint32_t addr) const
{
    auto ptr = mapAddress(addr);

    // out of bounds rom access returns low bits of address
    if(!ptr && (addr & 0x8000000))
        return addr >> 1;

    uint16_t ret = *reinterpret_cast<const uint16_t *>(ptr);

    // EEPROM
    if(saveType == SaveType::EEPROM && (addr >> 24) == 0xD)
        return eepromOutBits[(addr & 0xFF) >> 1];

    // io
    if((addr >> 24) == 0x4)
        ret = cpu.readReg(addr & 0xFFFFFF, ret);

    return ret;
}

uint32_t AGBMemory::read32(uint32_t addr) const
{
    auto ptr = mapAddress(addr);

    // out of bounds rom access returns low bits of address
    if(!ptr && (addr & 0x8000000))
    {
        auto addrLow = (addr >> 1) & 0xFFFF;
        return addrLow | (addrLow + 1) << 16;
    }

    return *reinterpret_cast<const uint32_t *>(ptr);
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

    // flash/ram detection and flash handling
    if((addr >> 24) == 0xE && (saveType == SaveType::Flash || saveType == SaveType::Unknown))
    {
        // TODO: size

        // auto-detect
        if(saveType == SaveType::Unknown && addr == 0xE005555 && data == 0xAA)
            saveType = SaveType::Flash;
        else if(saveType == SaveType::Unknown)
            saveType = SaveType::RAM;

        if(saveType == SaveType::Flash)
        {
            writeFlash(addr, data);
            return;
        }
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
    if((addr >> 24) == 0x4)
    {
        if(cpu.writeReg(addr & 0xFFFFFF, data))
            return;

        if((addr & 0xFFFFFF) == IO_WAITCNT)
        {
            // update ROM access times
            static const int nTimings[]{4, 3, 2, 8};
            cartAccessN[0] = nTimings[(data & WAITCNT_ROMWS0N) >> 2] + 1;
            cartAccessN[1] = nTimings[(data & WAITCNT_ROMWS1N) >> 5] + 1;
            cartAccessN[2] = nTimings[(data & WAITCNT_ROMWS1N) >> 8] + 1;

            cartAccessS[0] = (data & WAITCNT_ROMWS0S) ? 2 : 3;
            cartAccessS[1] = (data & WAITCNT_ROMWS1S) ? 2 : 5;
            cartAccessS[2] = (data & WAITCNT_ROMWS2S) ? 2 : 9;

            // ... and SRAM/flash
            cartAccessN[3] = cartAccessS[3] = nTimings[data & WAITCNT_SRAM] + 1;
        }
    }

    // EEPROM
    if((addr >> 24) == 0xD && (saveType == SaveType::EEPROM || saveType == SaveType::Unknown))
    {
        if(saveType == SaveType::Unknown)
            saveType = SaveType::EEPROM;

        eepromInBits[(addr & 0xFF) >> 1] = data;

        if((addr & 0xFF) == 0x10 && eepromInBits[0] && eepromInBits[1]) // end of read request for 512b
        {
            uint16_t eepromAddr = (eepromInBits[2] << 5) | (eepromInBits[3] << 4) | (eepromInBits[4] << 3)
                                | (eepromInBits[5] << 2) | (eepromInBits[6] << 1) | eepromInBits[7];

            uint64_t data = reinterpret_cast<uint64_t *>(cartSaveData)[eepromAddr];

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

            reinterpret_cast<uint64_t *>(cartSaveData)[eepromAddr] = data;

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
                return nullptr;
            return cartROM + addr;

        case 0xE:
        case 0xF:
        {
            if(saveType != SaveType::EEPROM)
            {
                if(flashState == FlashState::ID)
                    return flashID + (addr & 1);

                addr &= (saveType == SaveType::RAM ? 0x7FFF : 0xFFFF); // RAM is always 32K, flash has 1-2 64k banks
                return cartSaveData + addr + (flashBank << 16);
            }
        }
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

        case 0xE:
        case 0xF:
        {
            if(saveType != SaveType::EEPROM)
            {
                // flash never makes it here
                addr &= 0x7FFF;
                return cartSaveData + addr;
            }
        }
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

        case 0x8: // wait state 0
        case 0x9:
        case 0xA: // wait state 1
        case 0xB:
        case 0xC: // wait state 2
        case 0xD:
        case 0xE: // SRAM/flash
        case 0xF:
            return (sequential ? cartAccessS[(addr >> 25) - 4] : cartAccessN[(addr >> 25) - 4])
                   + (width == 4 ? cartAccessS[(addr >> 25) - 4] : 0); // extra time for reading 32bit value is always sequential

    }

    return 1;
}

void AGBMemory::writeFlash(uint32_t addr, uint8_t data)
{
    // bank switch
    if(flashState == FlashState::Bank && addr == 0xE000000)
    {
        flashBank = data;
        flashState = FlashState::Read;
        return;
    }
    // write a byte
    else if(flashState == FlashState::Write)
    {
        cartSaveData[(addr & 0xFFFF) + (flashBank << 16)] = data;
        flashState = FlashState::Read;
        return;
    }

    // parse commands
    if(flashCmdState == 0 && addr == 0xE005555 && data == 0xAA)
        flashCmdState = 1;
    else if(flashCmdState == 1 && addr == 0xE002AAA && data == 0x55)
        flashCmdState = 2;
    else if(flashCmdState == 2)
    {
        if(data == 0x10 && addr == 0xE005555 && flashState == FlashState::Erase)
        {
            // erase all
            memset(cartSaveData, 0xFF, sizeof(cartSaveData));
            flashState = FlashState::Read;
        }
        else if(data == 0x30 && flashState == FlashState::Erase)
        {
            // erase 4k sector
            memset(cartSaveData + (addr & 0xF000) + (flashBank << 16), 0xFF, 0xFFF);
            flashState = FlashState::Read;
        }
        else if(data == 0x80 && addr == 0xE005555)
            flashState = FlashState::Erase; // actual erase happens later
        else if(data == 0x90 && addr == 0xE005555)
        {
            // TODO: ids - this is the 128k sanyo one
            flashID[0] = 0x62;
            flashID[1] = 0x13;
            flashState = FlashState::ID;
        }
        else if(data == 0xA0 && addr == 0xE005555)
            flashState = FlashState::Write;
        else if(data == 0xB0 && addr == 0xE005555)
            flashState = FlashState::Bank;
        else if(data == 0xF0 && addr == 0xE005555)
            flashState = FlashState::Read;
        else
            printf("Flash CMD %02X\n", data);

        flashCmdState = 0;
    }
    else
        flashCmdState = 0;
}