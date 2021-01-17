#pragma once
#include <cstdint>
#include <list>

class DMGMemory
{
public:
    using ReadCallback = uint8_t(*)(uint16_t, uint8_t val);
    using WriteCallback = bool(*)(uint16_t, uint8_t val);

    using ROMBankCallback = void(*)(uint8_t, uint8_t *);

    using CartRamUpdateCallback = void(*)(uint8_t *, unsigned int);

    void setROMBankCallback(ROMBankCallback callback);
    void setCartROM(const uint8_t *rom);
    void loadCartridgeRAM(const uint8_t *ram, uint32_t len);
    void reset();

    // only covers io registers
    void setReadCallback(ReadCallback readCallback);
    void setWriteCallback(WriteCallback writeCallback);

    uint8_t read(uint16_t addr) const;
    void write(uint16_t addr, uint8_t data);

    const uint8_t *mapAddress(uint16_t addr) const;

    // fast access to IO regs
    uint8_t readIOReg(uint8_t addr) const {return iohram[addr];}
    uint8_t &getIOReg(uint8_t addr) {return iohram[addr];}
    void writeIOReg(uint8_t addr, uint8_t val) {iohram[addr] = val;}

    uint8_t *getVRAM() {return vram;}
    uint8_t *getOAM() {return oam;}

    uint8_t *getCartridgeRAM() {return cartRam;}
    int getCartridgeRAMSize() {return cartRamSize;}
    void setCartRamUpdateCallback(CartRamUpdateCallback callback);

private:
    void writeMBC(uint16_t addr, uint8_t data);
    void updateCurrentROMBank();

    enum class MBCType
    {
        None = 0,
        MBC1,
        MBC2,
        MBC3,
        MBC5
    };

    struct ROMCacheEntry
    {
        uint8_t *ptr;
        uint8_t bank;
    };

    // memory map with pointers offset so that regions[addr >> 12][addr] works
    const uint8_t *regions[16];

    uint8_t iohram[0x100]; // io @ 0xFF00, hram @ 0xFF80, ie & 0xFFFF

    uint8_t vram[0x2000 * 2]; // 8k @ 0x8000, two banks on GBC
    uint8_t wram[0x1000 * 8]; // 8k @ 0xC000, second half switchable on GBC
    uint8_t oam[0xA0]; // @ 0xFE00

    uint8_t vramBank = 0;
    uint8_t wramBank = 1;

    // cartridge
    MBCType mbcType = MBCType::None;
    bool mbcRAMEnabled = false;
    int mbcROMBank = 1, mbcRAMBank = 0;
    bool mbcRAMBankMode = false;
    uint8_t cartRam[0x8000];
    bool cartRamWritten = false;
    unsigned int cartRamSize = 0;

    uint8_t cartROMBank0[0x4000];
    const uint8_t *cartROM = nullptr; // used if entire rom is loaded somewhere

    // cache as much as possible in RAM
    static const int romBankCacheSize = 11;
    uint8_t cartROMBankCache[0x4000 * romBankCacheSize];
    std::list<ROMCacheEntry> cachedROMBanks;

    ReadCallback readCallback;
    WriteCallback writeCallback;

    ROMBankCallback romBankCallback;

    CartRamUpdateCallback cartRamUpdateCallback;
};