#pragma once
#include <cstdint>
#include <functional>
#include <list>

class DMGCPU;
class DMGMemory
{
public:
    DMGMemory(DMGCPU &cpu);

    using ROMBankCallback = void(*)(uint8_t, uint8_t *);

    using CartRamUpdateCallback = void(*)(uint8_t *, unsigned int);

    void setROMBankCallback(ROMBankCallback callback);
    void setCartROM(const uint8_t *rom);
    void loadCartridgeRAM(const uint8_t *ram, uint32_t len);

    void addROMCache(uint8_t *ptr, uint32_t size);

    void reset();

    void saveMBCState(std::function<uint32_t(uint32_t, uint32_t, const uint8_t *)> writeFunc, uint32_t &offset);

    void setGBC(bool gbc) {isGBC = gbc;}

    uint8_t read(uint16_t addr) const;
    void write(uint16_t addr, uint8_t data);

    const uint8_t *mapAddress(uint16_t addr) const;

    // fast access to IO regs
    uint8_t readIOReg(uint8_t addr) const {return iohram[addr];}
    uint8_t &getIOReg(uint8_t addr) {return iohram[addr];}
    void writeIOReg(uint8_t addr, uint8_t val) {iohram[addr] = val;}

    uint8_t *getWRAM() {return wram;}
    uint8_t *getVRAM() {return vram;}
    uint8_t *getOAM() {return oam;}

    uint8_t *getCartridgeRAM() {return cartRam;}
    int getCartridgeRAMSize() {return cartRamSize;}
    void setCartRamUpdateCallback(CartRamUpdateCallback callback);

    bool hasRTC() const;
    void getRTCData(uint32_t buf[12]);
    void setRTCData(uint32_t buf[12]);

private:
    void writeMBC(uint16_t addr, uint8_t data);
    void updateCurrentROMBank(unsigned int bank, int region);

    void updateRTC();

    enum class MBCType : uint8_t
    {
        None = 0,
        MBC1,
        MBC1M, // multicart - wired slightly differently
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

    static const int dmgWRAMSize = 0x2000; // 8k @ 0x8000
    static const int dmgVRAMSize = 0x2000; // 8k @ 0xC000
    static const int cgbWRAMSize = 0x2000 * 4; // 4k @ 0x8000 + 4k x7 @ 0xA000
    static const int cgbVRAMSize = 0x2000 * 2; // 8k x2 @ 0xC000

    uint8_t ram[cgbWRAMSize + cgbVRAMSize]; // wram + vram
    uint8_t *vram = nullptr;

    uint8_t oam[0xA0]; // @ 0xFE00

    bool isGBC = false;
    uint8_t vramBank = 0;
    uint8_t wramBank = 1;

    // cartridge
    MBCType mbcType = MBCType::None;
    bool mbcRAMEnabled = false;
    bool mbcRAMBankMode = false;
    bool cartRamWritten = false;

    int mbcROMBank = 1, mbcRAMBank = 0;
    uint8_t cartRam[0x8000];

    unsigned int cartRamSize = 0;

    // RTC
    uint8_t rtcRegs[10]; // internal / latched
    uint16_t rtcMilliseconds = 0;
    uint32_t rtcUpdateTime = 0;

    uint8_t cartROMBank0[0x4000];
    const uint8_t *cartROM = nullptr; // used if entire rom is loaded somewhere
    unsigned int cartROMBanks = 0; // read from the header

    // cache ROM banks in RAM
    std::list<ROMCacheEntry> cachedROMBanks;

    DMGCPU &cpu;

    ROMBankCallback romBankCallback;

    CartRamUpdateCallback cartRamUpdateCallback;
};