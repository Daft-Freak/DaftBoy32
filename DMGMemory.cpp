#include <cstdio>
#include <cstring>

#include "DMGMemory.h"
#include "DMGRegs.h"

void DMGMemory::loadCartridge(const uint8_t *rom, uint32_t romLen)
{
    cartROM = rom;
    cartROMLen = romLen;

    switch(cartROM[0x147])
    {
        case 0:
            mbcType = MBCType::None;
            break;
        case 1:
        case 2: // + RAM
        case 3: // + RAM + Battery
            mbcType = MBCType::MBC1;
            break;

        default:
            printf("unhandled cartridge type %x\n", cartROM[0x147]);
            mbcType = MBCType::None;
    }

    mbcRAMEnabled = false;
    mbcROMBank = 1;
    mbcRAMBank = 0;
    mbcRAMBankMode = false;
}

void DMGMemory::reset()
{
    // io regs
    memset(iohram, 0xFF, 0x80);

    iohram[IO_JOYP] = 0xC0;
    iohram[0x01] = 0x00; // SB
    iohram[0x02] = 0x7E; // SC
    iohram[IO_TIMA] = 0x00; // TIMA
    iohram[IO_TMA] = 0x00; // TMA
    iohram[IO_TAC] = 0xF8; // TAC
    iohram[IO_IF] = 0xE1;
    iohram[0x10] = 0x80; // NR10
    iohram[0x11] = 0xBF; // NR11
    iohram[0x12] = 0xF3; // NR12
    iohram[0x14] = 0xBF; // NR14
    iohram[0x16] = 0x3F; // NR21
    iohram[0x17] = 0x00; // NR22
    iohram[0x19] = 0xBF; // NR24
    iohram[0x1A] = 0x7F; // NR30
    iohram[0x1B] = 0xFF; // NR31
    iohram[0x1C] = 0x9F; // NR32
    iohram[0x1E] = 0xBF; // NR33
    iohram[0x20] = 0xFF; // NR41
    iohram[0x21] = 0x00; // NR42
    iohram[0x22] = 0x00; // NR43
    iohram[0x23] = 0xBF; // NR30
    iohram[0x24] = 0x77; // NR50
    iohram[0x25] = 0xF3; // NR51
    iohram[0x26] = 0xF1; // NR52
    iohram[IO_LCDC] = 0x91; // LCDC
    iohram[IO_STAT] = 0x80;
    iohram[IO_SCY] = 0x00; // SCY
    iohram[IO_SCX] = 0x00; // SCX
    iohram[IO_LY] = 0x00;
    iohram[IO_LYC] = 0x00; // LYC
    iohram[IO_BGP] = 0xFC; // BGP
    iohram[IO_OBP0] = 0xFF; // OBP0
    iohram[IO_OBP1] = 0xFF; // OBP1
    iohram[IO_WY] = 0x00; // WY
    iohram[IO_WX] = 0x00; // WX
    iohram[IO_IE] = 0x00; // IE
}

void DMGMemory::setReadCallback(ReadCallback readCallback)
{
    this->readCallback = readCallback;
}

void DMGMemory::setWriteCallback(WriteCallback writeCallback)
{
    this->writeCallback = writeCallback;
}

uint8_t DMGMemory::read(uint16_t addr) const
{
    if(addr < 0x8000)
    {
        int mappedAddr = addr;
        if(addr > 0x4000) // handle banking
            mappedAddr += (mbcROMBank - 1) * 0x4000;

        if(mappedAddr < cartROMLen)
            return cartROM[mappedAddr];
    }
    else if(addr < 0xA000)
        return vram[addr - 0x8000];
    else if(addr < 0xC000)
        return cartRam[(addr - 0xA000) + mbcRAMBank * 0x2000];
    else if(addr < 0xE000)
        return wram[addr - 0xC000];
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
        return oam[addr - 0xFE00];
    else if(addr < 0xFF00)
    {} //unusable
    else
    {
        auto val = iohram[addr & 0xFF];

        if(readCallback)
            val = readCallback(addr, val);

        return val;
    }

    printf("read %x\n", addr);
    return 0;
}

uint8_t DMGMemory::readIOReg(uint8_t addr) const
{
    return iohram[addr];
}

void DMGMemory::write(uint16_t addr, uint8_t data)
{
    if(addr < 0x8000)
    {
        writeMBC(addr, data); // cart rom
        return;
    }
    else if(addr >= 0xFF00)
    {
        if(writeCallback)
            writeCallback(addr, data);
        iohram[addr & 0xFF] = data;
        return;
    }
    else if(addr >= 0xFEA0 && addr < 0xFF00)
        return; // unusable
    else
    {
        auto ptr = mapAddress(addr);
        if(ptr)
        {
            ptr[addr] = data;
            return;
        }
    }

    printf("write %x = %x\n", addr, data);
}

void DMGMemory::writeIOReg(uint8_t addr, uint8_t val)
{
    iohram[addr] = val;
}

uint8_t *DMGMemory::mapAddress(uint16_t &addr)
{
    if(addr < 0x8000)
    {
        // since cartROM is const, can't return it here
        return nullptr; 
    }
    else if(addr < 0xA000)
    {
        addr -= 0x8000;
        return vram;
    }
    else if(addr < 0xC000)
    {
        addr = (addr - 0xA000) + mbcRAMBank * 0x2000;
        return cartRam;
    }
    else if(addr < 0xE000)
    {
        addr -= 0xC000;
        return wram;
    }
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
    {
        addr -= 0xFE00;
        return oam;
    }
    else if(addr < 0xFF00)
    {} //unusable
    else
    {
        addr = addr & 0xFF;
        return iohram;
    }

    return nullptr;
}

void DMGMemory::writeMBC(uint16_t addr, uint8_t data)
{
    if(mbcType == MBCType::None)
        return;

    // MBC1

    if(addr < 0x2000)
        mbcRAMEnabled = (data & 0xF) == 0xA;
    else if(addr < 0x4000)
    {
        // low 5 bits of rom bank
        mbcROMBank = (mbcROMBank & 0xE0) | (data & 0x1F);

        if((mbcROMBank & 0x1F) == 0)
            mbcROMBank++; // bank 0 is handled as bank 1
    }
    else if(addr < 0x6000)
    {
        // high 2 bits of rom bank / ram bank
        if(mbcRAMBankMode)
            mbcRAMBank = data & 0x3;
        else
            mbcROMBank = (mbcROMBank & 0x1F) | (data & 0xE0);
    }
    else // < 0x8000
    {
        mbcRAMBankMode = data == 1;
        if(mbcRAMBankMode)
            mbcROMBank &= 0x1F;
        else
            mbcRAMBank = 0;
    }
}