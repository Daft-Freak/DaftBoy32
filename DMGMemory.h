#pragma once
#include <cstdint>
#include <list>

class DMGMemory
{
public:
    using ReadCallback = uint8_t(*)(uint16_t, uint8_t val);
    using WriteCallback = bool(*)(uint16_t, uint8_t val);

    using ROMBankCallback = void(*)(uint8_t, uint8_t *);

    using CartRamUpdateCallback = void(*)(uint8_t *);

    void setROMBankCallback(ROMBankCallback callback);
    void loadCartridgeRAM(const uint8_t *ram, uint32_t len);
    void reset();

    // only covers io registers
    void setReadCallback(ReadCallback readCallback);
    void setWriteCallback(WriteCallback writeCallback);

    uint8_t read(uint16_t addr) const;
    void write(uint16_t addr, uint8_t data);
    
    // fast access to IO regs
    uint8_t readIOReg(uint8_t addr) const {return iohram[addr];}
    uint8_t &getIOReg(uint8_t addr) {return iohram[addr];}
    void writeIOReg(uint8_t addr, uint8_t val) {iohram[addr] = val;}

    uint8_t *getCartridgeRAM() {return cartRam;}
    void setCartRamUpdateCallback(CartRamUpdateCallback callback);

    uint8_t *mapAddress(uint16_t &addr);

private:
    void writeMBC(uint16_t addr, uint8_t data);
    void updateCurrentROMBank();

    enum class MBCType
    {
        None = 0,
        MBC1,
        MBC3
    };

    struct ROMCacheEntry
    {
        uint8_t *ptr;
        uint8_t bank;
    };

    uint8_t vram[0x2000]; // 8k @ 0x8000
    uint8_t wram[0x2000]; // 8k @ 0xC000
    uint8_t oam[0xA0]; // @ 0xFE00
    uint8_t iohram[0x100]; // io @ 0xFF00, hram @ 0xFF80, ie & 0xFFFF

    // cartridge
    MBCType mbcType = MBCType::None;
    bool mbcRAMEnabled = false;
    int mbcROMBank = 1, mbcRAMBank = 0;
    bool mbcRAMBankMode = false;
    uint8_t cartRam[0x8000];
    bool cartRamWritten = false;

    uint8_t cartROMBank0[0x4000];
    uint8_t *cartROMCurBank;

    // cache as much as possible in RAM
    static const int romBankCacheSize = 7;
    uint8_t cartROMBankCache[0x4000 * romBankCacheSize];
    std::list<ROMCacheEntry> cachedROMBanks;

    ReadCallback readCallback;
    WriteCallback writeCallback;

    ROMBankCallback romBankCallback;

    CartRamUpdateCallback cartRamUpdateCallback;
};