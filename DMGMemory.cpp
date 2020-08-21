#include <cstdio>
#include <cstring>

#include "DMGMemory.h"
#include "DMGRegs.h"

static const int extraROMBankCacheSize = 4;
static uint8_t extraROMBankCache[0x4000 * extraROMBankCacheSize]{1}; // sneakily steal some of DTCMRAM

void DMGMemory::setROMBankCallback(ROMBankCallback callback)
{
    this->romBankCallback = callback;
}

void DMGMemory::setCartROM(const uint8_t *rom)
{
    cartROM = rom;
}

void DMGMemory::loadCartridgeRAM(const uint8_t *ram, uint32_t len)
{
    memcpy(cartRam, ram, len);
}

void DMGMemory::reset()
{
    vramBank = 0;
    wramBank = 1;

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

    // clear vram
    memset(vram, 0, sizeof(vram));

    // load some ROM
    cartROMCurBank = cartROMBankCache;
    romBankCallback(0, cartROMBank0);
    romBankCallback(1, cartROMBankCache);

    cachedROMBanks.clear();
    for(int i = 0; i < romBankCacheSize; i++)
        cachedROMBanks.emplace_back(ROMCacheEntry{cartROMBankCache + i * 0x4000, 0});

    for(int i = 0; i < extraROMBankCacheSize; i++)
        cachedROMBanks.emplace_back(ROMCacheEntry{extraROMBankCache + i * 0x4000, 0});

    // check cart ram size
    static const unsigned int ramSizes[]{
        0, 2048, 8 * 1024, 32 * 1024, 128 * 1024, 64 * 1024
    };

    cartRamSize = ramSizes[cartROMBank0[0x149]];

    if(cartRamSize > sizeof(cartRam))
        printf("Too much cart RAM! (%i > %i)\n", cartRamSize, sizeof(cartRam));

    // use spare RAM as rom cache
    if(cartRamSize == 0)
        cachedROMBanks.emplace_back(ROMCacheEntry{cartRam, 0});
    if(cartRamSize <= 0x4000)
        cachedROMBanks.emplace_back(ROMCacheEntry{cartRam + 0x4000, 0});

    switch(cartROMBank0[0x147])
    {
        case 0:
            mbcType = MBCType::None;
            break;
        case 1:
        case 2: // + RAM
        case 3: // + RAM + Battery
            mbcType = MBCType::MBC1;
            break;

        case 0x0F: // + Timer + Battery
        case 0x10: // + Timer + RAM + Battery
        case 0x11:
        case 0x12: // + RAM
        case 0x13: // + RAM + Battery
            mbcType = MBCType::MBC3;
            break;

        case 0x19:
        case 0x1A: // + RAM
        case 0x1B: // + RAM + Battery
        case 0x1C: // + Rumble
        case 0x1D: // + Rumble + RAM
        case 0x1E: // + Rumble + RAM + Battery 
            mbcType = MBCType::MBC5;
            break;

        default:
            printf("unhandled cartridge type %x\n", cartROMBank0[0x147]);
            mbcType = MBCType::None;
    }

    mbcRAMEnabled = false;
    mbcROMBank = 1;
    mbcRAMBank = 0;
    mbcRAMBankMode = false;
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
    if(addr < 0x4000)
        return cartROMBank0[addr];
    else if(addr < 0x8000)
        return cartROMCurBank[addr & 0x3FFF];
    else if(addr < 0xA000)
        return vram[(addr & 0x1FFF) + (vramBank << 13) /* * 0x2000*/];
    else if(addr < 0xC000)
        return cartRam[(addr & 0x1FFF) + (mbcRAMBank << 13)];
    else if(addr < 0xD000)
        return wram[addr & 0xFFF];
    else if(addr < 0xE000)
        return wram[(addr & 0xFFF) + (wramBank << 12) /* * 0x1000*/];
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
        return oam[addr & 0xFF];
    else if(addr < 0xFF00)
    {} //unusable
    else
    {
        auto val = iohram[addr & 0xFF];

        if((addr & 0xFF) == IO_VBK)
            return vramBank;

        if(readCallback)
            val = readCallback(addr, val);

        return val;
    }

    printf("read %x\n", addr);
    return 0;
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
        if((addr & 0xFF) == IO_VBK)
            vramBank = data & 1;
        else if((addr & 0xFF) == IO_SVBK)
            wramBank = (data & 0x7) ? (data & 0x7) : 1; // 0 is also 1
        else if(writeCallback && writeCallback(addr, data))
            return;
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
            if(ptr == cartRam)
                cartRamWritten = true;

            ptr[addr] = data;
            return;
        }
    }

    printf("write %x = %x\n", addr, data);
}

void DMGMemory::setCartRamUpdateCallback(CartRamUpdateCallback callback)
{
    cartRamUpdateCallback = callback;
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
        addr = (addr & 0x1FFF) + (vramBank << 13) /* * 0x2000*/;
        return vram;
    }
    else if(addr < 0xC000)
    {
        addr = (addr & 0x1FFF) + (mbcRAMBank << 13);
        return mbcRAMEnabled ? cartRam : nullptr;
    }
    else if(addr < 0xD000)
    {
        addr &= 0xFFF;
        return wram;
    }
    else if(addr < 0xE000)
    {
        addr = (addr & 0xFFF) + (wramBank << 12) /* * 0x1000*/;
        return wram;
    }
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
    {
        addr &= 0xFF;
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

    // MBC1/3/5

    if(addr < 0x2000)
    {
        mbcRAMEnabled = (data & 0xF) == 0xA;

        // on disable sync the ram if changed
        if(!mbcRAMEnabled)
        {
            if(cartRamWritten && cartRamUpdateCallback)
                cartRamUpdateCallback(cartRam, cartRamSize);

            cartRamWritten = false;
        }
    }
    else if(addr < 0x4000)
    {
        if(mbcType == MBCType::MBC1) // low 5 bits of rom bank
        {
            mbcROMBank = (mbcROMBank & 0xE0) | (data & 0x1F);

            if((mbcROMBank & 0x1F) == 0)
                mbcROMBank++; // bank 0 is handled as bank 1 (+ some bugs)
        }
        else if(mbcType == MBCType::MBC3)
        {
            mbcROMBank = data & 0x7F; // 7 bit rom bank
            if(mbcROMBank == 0)
                mbcROMBank = 1; // bank 0 is handled as bank 1
        }
        else if(mbcType == MBCType::MBC5)
        {
            if(addr < 0x3000) // low 8
                mbcROMBank = (mbcROMBank & 0x100) | data;
            else // high 1
                mbcROMBank = (mbcROMBank & 0xFF) | ((data & 1) << 8);
        }

        updateCurrentROMBank();
    }
    else if(addr < 0x6000)
    {
        if(mbcType == MBCType::MBC5)
            mbcRAMBank = data & 0xF;
        // high 2 bits of rom bank / ram bank
        else if(mbcRAMBankMode || mbcType != MBCType::MBC3)
            mbcRAMBank = data & 0x3;
        else
        {
            mbcROMBank = (mbcROMBank & 0x1F) | (data & 0xE0);
            updateCurrentROMBank();
        }

        // TODO: MBC3 bank >= 8 maps RTC regs
    }
    else // < 0x8000
    {
        // TODO: MBC3 clock latch
        if(mbcType == MBCType::MBC1)
        {
            mbcRAMBankMode = data == 1;
            if(mbcRAMBankMode)
                mbcROMBank &= 0x1F;
            else
                mbcRAMBank = 0;
        }
    }
}

void DMGMemory::updateCurrentROMBank()
{
    if(mbcROMBank == 0)
    {
        cartROMCurBank = cartROMBank0;
        return;
    }

    // entire ROM is loaded
    if(cartROM)
    {
        cartROMCurBank = cartROM + mbcROMBank * 0x4000;
        return;
    }

    for(auto it = cachedROMBanks.begin(); it != cachedROMBanks.end(); ++it)
    {
        if(it->bank == mbcROMBank)
        {
            cartROMCurBank = it->ptr;
            cachedROMBanks.splice(cachedROMBanks.begin(), cachedROMBanks, it); // move it to the top
            return;
        }
    }

    // reuse the last (least recently used) bank
    auto it = std::prev(cachedROMBanks.end());

    cartROMCurBank = it->ptr;
    romBankCallback(mbcROMBank, it->ptr);
    it->bank = mbcROMBank;
    cachedROMBanks.splice(cachedROMBanks.begin(), cachedROMBanks, it); // move it to the top
}