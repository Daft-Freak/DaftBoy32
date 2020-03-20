#pragma once
#include <cstdint>

class DMGMemory
{
public:
    using ReadCallback = uint8_t(*)(uint16_t, uint8_t val);
    using WriteCallback = void(*)(uint16_t, uint8_t val);

    void loadCartridge(const uint8_t *rom, uint32_t romLen);
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

    uint8_t *mapAddress(uint16_t &addr);

private:
    void writeMBC(uint16_t addr, uint8_t data);

    enum class MBCType
    {
        None = 0,
        MBC1,
        MBC3
    };

    uint8_t vram[0x2000]; // 8k @ 0x8000
    uint8_t wram[0x2000]; // 8k @ 0xC000
    uint8_t oam[0xA0]; // @ 0xFE00
    uint8_t iohram[0x100]; // io @ 0xFF00, hram @ 0xFF80, ie & 0xFFFF

    // cartridge
    const uint8_t *cartROM = nullptr;
    uint32_t cartROMLen = 0;
    MBCType mbcType = MBCType::None;
    bool mbcRAMEnabled = false;
    int mbcROMBank = 1, mbcRAMBank = 0;
    bool mbcRAMBankMode = false;
    uint8_t cartRam[0x8000];

    ReadCallback readCallback;
    WriteCallback writeCallback;
};