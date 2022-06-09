#include <cstdio>
#include <cstring>

#include "DMGMemory.h"
#include "DMGCPU.h"
#include "DMGRegs.h"

DMGMemory::DMGMemory(DMGCPU &cpu) : cpu(cpu)
{
}

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

void DMGMemory::addROMCache(uint8_t *ptr, uint32_t size)
{
    auto end = ptr + size;

    while(ptr + 0x4000 <= end)
    {
        cachedROMBanks.emplace_back(ROMCacheEntry{ptr, 0});
        ptr += 0x4000;
    }
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

    // load first ROM bank for reading headers
    romBankCallback(0, cartROMBank0);

    // reset cache
    for(auto it = cachedROMBanks.begin(); it != cachedROMBanks.end();)
    {
        // remove cart ram/wram, will re-add later if possible
        if(it->ptr == cartRam || it->ptr == cartRam + 0x4000 || it->ptr == wram + 0x4000)
        {
            it = cachedROMBanks.erase(it);
            continue;
        }

        it->bank = 0;
        ++it;
    }

    // get ROM size
    int size = cartROMBank0[0x148];
    if(size <= 8)
        cartROMBanks = 2 << size;
    else if(size == 0x52)
        cartROMBanks = 72;
    else if(size == 0x53)
        cartROMBanks = 80;
    else if(size == 0x54)
        cartROMBanks = 96;
    else
        cartROMBanks = 0; // uhoh

    // check cart ram size
    static const unsigned int ramSizes[]{
        0, 2048, 8 * 1024, 32 * 1024, 128 * 1024, 64 * 1024
    };

    cartRamSize = ramSizes[cartROMBank0[0x149]];

    if(cartRamSize > sizeof(cartRam))
        printf("Too much cart RAM! (%i > %i)\n", cartRamSize, sizeof(cartRam));

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

        case 5:
        case 6: // + Battery
            mbcType = MBCType::MBC2;
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

    // use spare RAM as rom cache
    if(cartRamSize == 0 && mbcType != MBCType::MBC2)
        cachedROMBanks.emplace_back(ROMCacheEntry{cartRam, 0});
    if(cartRamSize <= 0x4000)
        cachedROMBanks.emplace_back(ROMCacheEntry{cartRam + 0x4000, 0});

    if(!(cartROMBank0[0x143] & 0x80))// CGB flag
        cachedROMBanks.emplace_back(ROMCacheEntry{wram + 0x4000, 0}); // spare WRAM (really 0x6000)

    // grab the first bank to use for bank 1
    auto cartROMBank1 = cachedROMBanks.front().ptr;

    if(mbcType == MBCType::MBC1 && cartROMBanks == 64)
    {
        // 1M MBC1 may be a multicart
        for(int i = 1; i < 4; i++)
        {
            romBankCallback(i << 4, cartROMBank1);

            // compare logos
            if(memcmp(cartROMBank0 + 0x104, cartROMBank1 + 0x104, 48) == 0)
            {
                mbcType = MBCType::MBC1M;
                break;
            }
        }
    }

    // load the second bank too
    romBankCallback(1, cartROMBank1);

    mbcRAMEnabled = false;
    mbcROMBank = 1;
    mbcRAMBank = 0;
    mbcRAMBankMode = false;

    regions[0x0] =
    regions[0x1] =
    regions[0x2] =
    regions[0x3] = cartROMBank0;
    regions[0x4] =
    regions[0x5] =
    regions[0x6] =
    regions[0x7] = cartROMBank1 - 0x4000;
    regions[0x8] = vram - 0x8000; // banked
    regions[0x9] = vram - 0x8000; // banked
    regions[0xA] = nullptr; // cart RAM - banked
    regions[0xB] = nullptr; // cart RAM - banked
    regions[0xC] = wram - 0xC000;
    regions[0xD] = wram - 0xC000; // banked
    regions[0xE] = wram - 0xE000;
    regions[0xF] = nullptr;
}

uint8_t DMGMemory::read(uint16_t addr) const
{
    int region = addr >> 12;

    if(regions[region])
        return regions[region][addr];

    if(region != 0xF)
    {
        // handle disabled RAM
        if(!mbcRAMEnabled)
            return 0xFF;

        // only other way to get here is MBC2
        return cartRam[addr & 0x1FF];
    }

    // must be Fxxx

    if(addr >= 0xFF00)
    {
        auto val = iohram[addr & 0xFF];
        val = cpu.readReg(addr, val);

        return val;
    }

    if(addr < 0xFE00)
        return regions[0xD][addr - 0x2000]; // echo of D
    if(addr < 0xFEA0)
        return oam[addr & 0xFF];

    return 0; // FEA0 - FF00 = unusable
}

const uint8_t *DMGMemory::mapAddress(uint16_t addr) const
{
    int region = addr >> 12;

    if(regions[region])
        return regions[region] + addr;

    if(addr < 0xFE00)
        return regions[0xD] + addr - 0x2000; //echo (?)
    if(addr < 0xFEA0)
        return oam;
    if(addr < 0xFF00)
        return nullptr;

    return iohram;
}

void DMGMemory::write(uint16_t addr, uint8_t data)
{
    int region = addr >> 12;
    if(region < 8)
        writeMBC(addr, data); // cart rom
    else if(region == 0xA || region == 0xB)
    {
        if(mbcRAMEnabled)
        {
            // 512 4-bit values
            if(mbcType == MBCType::MBC2)
                cartRam[addr & 0x1FF] = data | 0xF0;
            else
                const_cast<uint8_t *>(regions[region])[addr] = data;
            cartRamWritten = true;
        }
    }
    else if(regions[region])
        const_cast<uint8_t *>(regions[region])[addr] = data; // these are the non-const ones...
    else 
    {
        // must be Fxxx

        if(addr < 0xFE00) // echo
        {
            const_cast<uint8_t *>(regions[0xD])[addr - 0x2000] = data;
            return;
        }

        if(addr < 0xFEA0)
        {
            oam[addr & 0xFF] = data;
            return;
        }
        if(addr < 0xFF00)
            return; //unusable

        if(addr == 0xFF50) // boot flag, not writable
            return;
        else if((addr & 0xFF) == IO_VBK)
        {
            if(!isGBC) return;

            vramBank = data & 1;
            data |= 0xFE; // make sure only the low bit reads back
            regions[0x8] = regions[0x9] = vram + (vramBank * 0x2000) - 0x8000;
        }
        else if((addr & 0xFF) == IO_SVBK)
        {
            if(!isGBC) return;

            wramBank = (data & 0x7) ? (data & 0x7) : 1; // 0 is also 1
            regions[0xD] = wram + (wramBank * 0x1000) - 0xD000;
        }
        else if(cpu.writeReg(addr, data))
            return;

        iohram[addr & 0xFF] = data;
    }
}

void DMGMemory::setCartRamUpdateCallback(CartRamUpdateCallback callback)
{
    cartRamUpdateCallback = callback;
}

void DMGMemory::writeMBC(uint16_t addr, uint8_t data)
{
    if(mbcType == MBCType::None)
        return;

    auto updateRAMBank = [this]()
    {
        if(!mbcRAMEnabled)
            regions[0xA] = regions[0xB] = nullptr; // RAM disabled
        else if((mbcType == MBCType::MBC1 || mbcType == MBCType::MBC1M) && !mbcRAMBankMode) // no banking, use bank 0
            regions[0xA] = regions[0xB] = cartRam - 0xA000;
        else
        {
            int off = (mbcRAMBank * 0x2000) & (cartRamSize - 1); // limit to RAM size
            regions[0xA] = regions[0xB] = cartRam + off - 0xA000;
        }
    };

    // MBC2 is a bit different (and simpler)
    if(mbcType == MBCType::MBC2)
    {
        if(addr < 0x4000)
        {
            if(addr & 0x100) // ROM bank
            {
                mbcROMBank = data & 0xF; // 4 bit rom bank
                if(mbcROMBank == 0)
                    mbcROMBank = 1; // bank 0 is handled as bank 1

                updateCurrentROMBank(mbcROMBank, 4);
            }
            else // RAM enable
            {
                mbcRAMEnabled = (data & 0xF) == 0xA;
                // don't set the pointers as the RAM is tiny and weird
            }
        }
        return;
    }

    // MBC1/3/5

    if(addr < 0x2000)
    {
        if(mbcType == MBCType::MBC5) // MBC5 doesn't ignore the top bits
            mbcRAMEnabled = data == 0xA;
        else
            mbcRAMEnabled = (data & 0xF) == 0xA;

        updateRAMBank();

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
        else if(mbcType == MBCType::MBC1M) // low 4 bits of rom bank (game bank)
        {
            mbcROMBank = (mbcROMBank & 0xF0) | (data & 0xF);

            if((data & 0x1F) == 0)
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

        updateCurrentROMBank(mbcROMBank, 4);
    }
    else if(addr < 0x6000)
    {
        // MBC3/5 RAM banking
        if(mbcType == MBCType::MBC5 || mbcType == MBCType::MBC3)
        {
            mbcRAMBank = mbcType == MBCType::MBC5 ? data & 0xF : data & 0x3;
            updateRAMBank();

            // TODO: MBC3 bank >= 8 maps RTC regs
        }
        else // MBC1 high 2 bits of rom bank / ram bank
        {
            if(mbcRAMBankMode)
            {
                mbcRAMBank = data & 0x3;
                updateRAMBank();

                // also affetcts the bank at 0-3FFF
                if(mbcType == MBCType::MBC1M)
                    updateCurrentROMBank((data & 0x3) << 4, 0);
                else
                    updateCurrentROMBank((data & 0x3) << 5, 0);
            }

            // ROM is always banked
            if(mbcType == MBCType::MBC1M)
                mbcROMBank = (mbcROMBank & 0xF) | ((data & 0x3) << 4);
            else
                mbcROMBank = (mbcROMBank & 0x1F) | ((data & 0x3) << 5);
            updateCurrentROMBank(mbcROMBank, 4);
        }
    }
    else // < 0x8000
    {
        // TODO: MBC3 clock latch
        if(mbcType == MBCType::MBC1 || mbcType == MBCType::MBC1M)
        {
            mbcRAMBankMode = data == 1;
            updateRAMBank();

            // update bank at 0-3FFF
            if(mbcRAMBankMode)
                updateCurrentROMBank(mbcROMBank & 0x60, 0);
            else
                updateCurrentROMBank(0, 0);
        }
    }
}

void DMGMemory::updateCurrentROMBank(unsigned int bank, int region)
{
    // region is either 0 or 4
    int offset = region * 0x1000;

    bank %= cartROMBanks;

    if(bank == 0)
    {
        for(int i = 0; i < 4; i++)
            regions[region + i] = cartROMBank0 - offset;
        return;
    }

    // entire ROM is loaded
    if(cartROM)
    {
        for(int i = 0; i < 4; i++)
            regions[region + i] = cartROM + bank * 0x4000 - offset;
        return;
    }

    for(auto it = cachedROMBanks.begin(); it != cachedROMBanks.end(); ++it)
    {
        if(it->bank == bank)
        {
            for(int i = 0; i < 4; i++)
                regions[region + i] = it->ptr - offset;

            cachedROMBanks.splice(cachedROMBanks.begin(), cachedROMBanks, it); // move it to the top
            return;
        }
    }

    // reuse the last (least recently used) bank
    auto it = std::prev(cachedROMBanks.end());

    // make sure it isn't being used by the other region
    if(mbcType == MBCType::MBC1 && ((region == 0 && regions[4] == it->ptr - 0x4000) || (region == 4 && regions[0] == it->ptr)))
        --it; // use the next one instead

    for(int i = 0; i < 4; i++)
        regions[region + i] = it->ptr - offset;

    romBankCallback(bank, it->ptr);
    it->bank = bank;
    cachedROMBanks.splice(cachedROMBanks.begin(), cachedROMBanks, it); // move it to the top
}