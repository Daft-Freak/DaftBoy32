#include <cstdio>
#include <cstring>

#include "AGBMemory.h"

#include "AGBCPU.h"
#include "AGBRegs.h"

enum MemoryRegion
{
    Region_BIOS      = 0x0,
    Region_Unused    = 0x1,
    Region_EWRAM     = 0x2,
    Region_IWRAM     = 0x3,
    Region_IO        = 0x4,
    Region_Palette   = 0x5,
    Region_VRAM      = 0x6,
    Region_OAM       = 0x7,
    Region_ROMWait0L = 0x8,
    Region_ROMWait0H = 0x9,
    Region_ROMWait1L = 0xA,
    Region_ROMWait1H = 0xB,
    Region_ROMWait2L = 0xC,
    Region_ROMWait2H = 0xD,
    Region_SaveL     = 0xE,
    Region_SaveH     = 0xF,
};

template uint8_t AGBMemory::read(uint32_t addr, int &cycles, bool sequential) const;
template uint16_t AGBMemory::read(uint32_t addr, int &cycles, bool sequential) const;
template uint32_t AGBMemory::read(uint32_t addr, int &cycles, bool sequential) const;
template void AGBMemory::write(uint32_t addr, uint8_t val, int &cycles, bool sequential);
template void AGBMemory::write(uint32_t addr, uint16_t val, int &cycles, bool sequential);
template void AGBMemory::write(uint32_t addr, uint32_t val, int &cycles, bool sequential);

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

template<class T>
T AGBMemory::read(uint32_t addr, int &cycles, bool sequential) const
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
        prefetchCycles -= c;
    };

    switch(addr >> 24)
    {
        case Region_BIOS:
            accessCycles(1);
            return doBIOSRead<T>(addr);
        case Region_Unused:
            accessCycles(1);
            return doOpenRead<T>(addr);
        case Region_EWRAM:
            accessCycles(sizeof(T) == 4 ? 6 : 3);
            return doRead<T>(ewram, addr);
        case Region_IWRAM:
            accessCycles(1);
            return doRead<T>(iwram, addr);
        case Region_IO:
            accessCycles(1);
            return doIORead<T>(addr);
        case Region_Palette:
            accessCycles(sizeof(T) == 4 ? 2 : 1);
            return doRead<T>(palRAM, addr);
        case Region_VRAM:
            accessCycles(sizeof(T) == 4 ? 2 : 1);
            return doVRAMRead<T>(addr);
        case Region_OAM:
            accessCycles(1);
            return doRead<T>(oam, addr);

        case Region_ROMWait0L:
        case Region_ROMWait0H:
        case Region_ROMWait1L:
        case Region_ROMWait1H:
        case Region_ROMWait2L:
            cycles += (sequential ? cartAccessS[(addr >> 25) - 4] : cartAccessN[(addr >> 25) - 4])
                   + (sizeof(T) == 4 ? cartAccessS[(addr >> 25) - 4] : 0);

            prefetchCycles = cartAccessN[(addr >> 25) - 4] + 1; // cart bus active, interrupt prefetch (+1 seems hacky but improves results?)
            return doROMRead<T>(addr);
        case Region_ROMWait2H:
            cycles += (sequential ? cartAccessS[2] : cartAccessN[2]) + (sizeof(T) == 4 ? cartAccessS[2] : 0);

            prefetchCycles = cartAccessN[2] + 1;
            return doROMOrEEPROMRead<T>(addr);

        case Region_SaveL:
        case Region_SaveH:
            cycles += (sequential ? cartAccessS[3] : cartAccessN[3]) + (sizeof(T) == 4 ? cartAccessS[3] : 0);
            return doSRAMRead<T>(addr);
    }

    return doOpenRead<T>(addr);
}

template<class T>
void AGBMemory::write(uint32_t addr, T data, int &cycles, bool sequential)
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
        prefetchCycles -= c;
    };

    switch(addr >> 24)
    {
        case Region_BIOS:
        case Region_Unused:
            accessCycles(1);
            return;
        case Region_EWRAM:
            accessCycles(sizeof(T) == 4 ? 6 : 3);
            doWrite(ewram, addr, data);
            return;
        case Region_IWRAM:
            accessCycles(1);
            doWrite(iwram, addr, data);
            return;
        case Region_IO:
            accessCycles(1);
            doIOWrite(addr, data);
            return;
        case Region_Palette:
            accessCycles(sizeof(T) == 4 ? 2 : 1);
            doPalRAMWrite(addr, data);
            return;
        case Region_VRAM:
            accessCycles(sizeof(T) == 4 ? 2 : 1);
            doVRAMWrite(addr, data);
            return;
        case Region_OAM:
            accessCycles(1);
            doOAMWrite(addr, data);
            return;

        case Region_ROMWait0L:
        case Region_ROMWait0H:
        case Region_ROMWait1L:
        case Region_ROMWait1H:
        case Region_ROMWait2L:
            cycles += (sequential ? cartAccessS[(addr >> 25) - 4] : cartAccessN[(addr >> 25) - 4])
                   + (sizeof(T) == 4 ? cartAccessS[(addr >> 25) - 4] : 0);
            return;
        case Region_ROMWait2H:
            cycles += (sequential ? cartAccessS[2] : cartAccessN[2]) + (sizeof(T) == 4 ? cartAccessS[2] : 0);
            return doEEPROMWrite(addr, data);

        case Region_SaveL:
        case Region_SaveH:
            cycles += (sequential ? cartAccessS[3] : cartAccessN[3]) + (sizeof(T) == 4 ? cartAccessS[3] : 0);
            doSRAMWrite(addr, data);
            return;
    }
}

const uint8_t *AGBMemory::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case Region_BIOS:
            return biosROM + (addr & 0x3FFF);

        case Region_EWRAM:
            return ewram + (addr & 0x3FFFF);
        case Region_IWRAM:
            return iwram + (addr & 0x7FFF);

        case Region_Palette:
            return palRAM + (addr & 0x3FF);
        case Region_VRAM:
            addr &= 0x1FFFF;
            if(addr >= 0x18000)
                addr &= ~0x8000; // last 32K is the previous 32K
            return vram + addr;
        case Region_OAM:
            return oam + (addr & 0x3FF);
        case Region_ROMWait0L:
        case Region_ROMWait0H:
        case Region_ROMWait1L:
        case Region_ROMWait1H:
        case Region_ROMWait2L:
        case Region_ROMWait2H:
            addr &= 0x1FFFFFF;
            if(addr >= cartROMSize)
                return nullptr;
            return cartROM + addr;
    }

    return reinterpret_cast<const uint8_t *>(&dummy);
}

uint8_t *AGBMemory::mapAddress(uint32_t addr)
{
    switch(addr >> 24)
    {
        case Region_EWRAM:
            return ewram + (addr & 0x3FFFF);
        case Region_IWRAM:
            return iwram + (addr & 0x7FFF);

        case Region_Palette:
            return palRAM + (addr & 0x3FF);
        case Region_VRAM:
            addr &= 0x1FFFF;
            if(addr >= 0x18000)
                addr &= ~0x8000; // last 32K is the previous 32K
            return vram + addr;
        case Region_OAM:
            return oam + (addr & 0x3FF);
    }

    return nullptr;
}

int AGBMemory::getAccessCycles(uint32_t addr, int width, bool sequential) const
{
    switch(addr >> 24)
    {
        case Region_BIOS:
        case Region_IWRAM:
        case Region_IO:
        case Region_OAM:
            return 1;

        case Region_EWRAM:
            return width == 4 ? 6 : 3;

        case Region_Palette:
        case Region_VRAM:
            return width == 4 ? 2 : 1;

        case Region_ROMWait0L:
        case Region_ROMWait0H:
        case Region_ROMWait1L:
        case Region_ROMWait1H:
        case Region_ROMWait2L:
        case Region_ROMWait2H:
        case Region_SaveL:
        case Region_SaveH:
            return (sequential ? cartAccessS[(addr >> 25) - 4] : cartAccessN[(addr >> 25) - 4])
                   + (width == 4 ? cartAccessS[(addr >> 25) - 4] : 0); // extra time for reading 32bit value is always sequential

    }

    return 1;
}

void AGBMemory::updateWaitControl(uint16_t waitcnt)
{
    // update ROM access times
    static const int nTimings[]{4, 3, 2, 8};
    cartAccessN[0] = nTimings[(waitcnt & WAITCNT_ROMWS0N) >> 2] + 1;
    cartAccessN[1] = nTimings[(waitcnt & WAITCNT_ROMWS1N) >> 5] + 1;
    cartAccessN[2] = nTimings[(waitcnt & WAITCNT_ROMWS1N) >> 8] + 1;

    cartAccessS[0] = (waitcnt & WAITCNT_ROMWS0S) ? 2 : 3;
    cartAccessS[1] = (waitcnt & WAITCNT_ROMWS1S) ? 2 : 5;
    cartAccessS[2] = (waitcnt & WAITCNT_ROMWS2S) ? 2 : 9;

    // ... and SRAM/flash
    cartAccessN[3] = cartAccessS[3] = nTimings[waitcnt & WAITCNT_SRAM] + 1;

    cartPrefetchEnabled = waitcnt & WAITCNT_Prefetch;
}

void AGBMemory::updatePC(uint32_t pc)
{
    // can't execure from save area anyway...
    pcInROM = (pc >> 24) >= Region_ROMWait0L;

    if(pcInROM)
    {
        prefetchCycles = prefetchSCycles = cartAccessS[(pc >> 25) - 4]; // pipeline refill makes next access S
        prefetchedHalfWords = 0;
    }
}

template<class T, size_t size>
T AGBMemory::doRead(const uint8_t (&mem)[size], uint32_t addr) const
{
    // use size of type for alignment
    return *reinterpret_cast<const T *>(mem + (addr & (size - sizeof(T))));
}

template<class T, size_t size>
void AGBMemory::doWrite(uint8_t (&mem)[size], uint32_t addr, T data)
{
    // use size of type for alignment
    *reinterpret_cast< T *>(mem + (addr & (size - sizeof(T)))) = data;
}

template<class T>
T AGBMemory::doBIOSRead(uint32_t addr) const
{
    // TODO: reading from outside BIOS
    const size_t size = 0x4000;
    return *reinterpret_cast<const T *>(biosROM + (addr & (size - 1)));
}

template<class T>
T AGBMemory::doIORead(uint32_t addr) const
{
    // IO regs don't mirror
    if(addr >= 0x4000400)
        return doOpenRead<T>(addr);

    return cpu.readReg(addr & 0xFFFFFF, doRead<T>(ioRegs, addr));
}

template<>
uint8_t AGBMemory::doIORead(uint32_t addr) const
{
    // promote to 16-bit
    return doIORead<uint16_t>(addr) >> (addr & 1) * 8;
}

template<>
[[gnu::noinline]]
uint32_t AGBMemory::doIORead(uint32_t addr) const
{
    // split for the callback
    return doIORead<uint16_t>(addr & ~3) | doIORead<uint16_t>((addr & ~3) + 2) << 16;
}

template<class T>
void AGBMemory::doIOWrite(uint32_t addr, T data)
{
    if(addr >= 0x4000400)
        return;

    if(cpu.writeReg(addr & 0x3FE, data, 0xFFFF))
        return;

    doWrite(ioRegs, addr, data);
}

template<>
void AGBMemory::doIOWrite(uint32_t addr, uint8_t data)
{
    if(addr >= 0x4000400)
        return;

    // promote to 16-bit
    auto tmp = readIOReg(addr & 0x3FE);
    uint16_t data16 = addr & 1 ? (tmp & 0xFF) | data << 8 : (tmp & 0xFF00) | data;

    // set mask to the byte actually getting written (important in some cases)
    if(cpu.writeReg(addr & 0x3FE, data16, addr & 1 ? 0xFF00 : 0xFF))
        return;

    doWrite(ioRegs, addr, data);
}

template<>
[[gnu::noinline]]
void AGBMemory::doIOWrite(uint32_t addr, uint32_t data)
{
    // split
    doIOWrite<uint16_t>(addr, data);
    doIOWrite<uint16_t>(addr + 2, data >> 16);
}

template<class T>
void AGBMemory::doPalRAMWrite(uint32_t addr, T data)
{
    doWrite(palRAM, addr, data);
}

template<>
void AGBMemory::doPalRAMWrite(uint32_t addr, uint8_t data)
{
    // writes byte value to halfword
    doWrite<uint16_t>(palRAM, addr, data | data << 8);
}

template<class T>
T AGBMemory::doVRAMRead(uint32_t addr) const
{
    addr &= (0x20000 - sizeof(T));
    if(addr >= sizeof(vram))
        addr &= ~0x8000; // last 32K is the previous 32K

    return *reinterpret_cast<const T *>(vram + addr);
}

template<class T>
void AGBMemory::doVRAMWrite(uint32_t addr, T data)
{
    addr &= (0x20000 - sizeof(T));
    if(addr >= sizeof(vram))
        addr &= ~0x8000; // last 32K is the previous 32K

    *reinterpret_cast<T *>(vram + addr) = data;
}

template<>
void AGBMemory::doVRAMWrite(uint32_t addr, uint8_t data)
{
    if((addr & 0x1FFFF) < 0x10000) // "background" VRAM, same as palette ram
        *reinterpret_cast<uint16_t *>(vram + (addr & 0xFFFE)) = data | data << 8;
    // else ignored
}

template<class T>
void AGBMemory::doOAMWrite(uint32_t addr, T data)
{
    doWrite(oam, addr, data);
}

template<>
void AGBMemory::doOAMWrite(uint32_t addr, uint8_t data)
{
    // ignored
}

template<class T>
T AGBMemory::doROMRead(uint32_t addr) const
{
    addr &= (0x2000000 - sizeof(T));
    if(addr >= cartROMSize)
    {
        // out of bounds rom access returns low bits of address
        auto addrLow = (addr >> 1) & 0xFFFF;

        if(sizeof(T) == 1)
            return addrLow >> (addr & 1) * 8;

        return addrLow | (addrLow + 1) << 16;
    }

    return *reinterpret_cast<const T *>(cartROM + addr);
}

template<class T>
T AGBMemory::doROMOrEEPROMRead(uint32_t addr) const
{
    // usually just ROM
    return doROMRead<T>(addr);
}

template<>
uint16_t AGBMemory::doROMOrEEPROMRead(uint32_t addr) const
{
    // 16-bit read from high ROM addr could be EEPROM
    if(saveType == SaveType::EEPROM)
        return eepromOutBits[(addr & 0xFF) >> 1];

    return doROMRead<uint16_t>(addr);
}

template<class T>
void AGBMemory::doEEPROMWrite(uint32_t addr, T data)
{
    // 16-bit only
}

template<>
void AGBMemory::doEEPROMWrite(uint32_t addr, uint16_t data)
{
    if(saveType == SaveType::Unknown)
        saveType = SaveType::EEPROM;

    if(saveType != SaveType::EEPROM)
        return;

    eepromInBits[(addr & 0xFF) >> 1] = data;

    // TODO: 4k

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
}

template<class T>
T AGBMemory::doSRAMRead(uint32_t addr) const
{
    uint8_t byte = doSRAMRead<uint8_t>(addr);

    return byte | byte << 8 | byte << 16 | byte << 24;
}

template<>
uint8_t AGBMemory::doSRAMRead(uint32_t addr) const
{
    // the only valid width
    switch(saveType)
    {
        case SaveType::Unknown:
        case SaveType::EEPROM:
            return 0xFF;
        case SaveType::RAM:
            return cartSaveData[addr & 0x7FFF]; // SRAM is 32k
        case SaveType::Flash:
            if(flashState == FlashState::ID)
                return flashID[addr & 1];
            
            return cartSaveData[(addr & 0xFFFF) + (flashBank << 16)]; // 1-2 64k banks
    }

    __builtin_unreachable();
}

template<class T>
void AGBMemory::doSRAMWrite(uint32_t addr, T data)
{
    int shift = (addr & (sizeof(T) - 1)) * 8;
    doSRAMWrite<uint8_t>(addr, data >> shift);
}

template<>
void AGBMemory::doSRAMWrite(uint32_t addr, uint8_t data)
{
    if(saveType == SaveType::Unknown)
    {
        if(addr == 0xE005555 && data == 0xAA)
            saveType = SaveType::Flash;
        else
            saveType = SaveType::RAM;
    }

    if(saveType == SaveType::Flash)
        writeFlash(addr, data);
    else if(saveType == SaveType::RAM)
        cartSaveData[addr & 0x7FFF] = data;
}

template<class T>
T AGBMemory::doOpenRead(uint32_t addr) const
{
    return static_cast<T>(0xBADADD55); // TODO
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
            memset(cartSaveData + (addr & 0xF000) + (flashBank << 16), 0xFF, 0x1000);
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