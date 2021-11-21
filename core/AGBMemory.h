#pragma once
#include <cassert>
#include <cstdint>

class AGBCPU;

class AGBMemory
{
public:
    enum class SaveType : uint8_t
    {
        Unknown,
        EEPROM,
        RAM,
        Flash
    };

    AGBMemory(AGBCPU &cpu);

    //using CartRamUpdateCallback = void(*)(uint8_t *, unsigned int);

    void setBIOSROM(const uint8_t *rom);
    void setCartROM(const uint8_t *rom, uint32_t size);
    void loadCartridgeSave(const uint8_t *data, uint32_t len);
    void reset();

    uint8_t read8(uint32_t addr) const;
    uint16_t read16(uint32_t addr) const;
    uint32_t read32(uint32_t addr) const;

    // used for fetching instructions, makes some assumptions
    uint16_t read16Fast(uint32_t addr) const
    {
        assert((addr & 1) == 0);
        assert((addr >> 24) != 0x4); // not io

        return *reinterpret_cast<const uint16_t *>(mapAddress(addr));
    }

    uint32_t read32Fast(uint32_t addr) const
    {
        assert((addr & 3) == 0);
        assert((addr >> 24) != 0x4);

        return *reinterpret_cast<const uint32_t *>(mapAddress(addr));
    }

    void write8(uint32_t addr, uint8_t data);
    void write16(uint32_t addr, uint16_t data);
    void write32(uint32_t addr, uint32_t data);

    // fast access to IO regs
    uint16_t readIOReg(uint16_t addr) const {return *reinterpret_cast<const uint16_t *>(ioRegs + addr);}
    uint16_t &getIOReg(uint16_t addr) {return *reinterpret_cast<uint16_t *>(ioRegs + addr);}
    void writeIOReg(uint16_t addr, uint16_t val) {*reinterpret_cast<uint16_t *>(ioRegs + addr) = val;}

    uint8_t *getPalRAM() {return palRAM;}
    uint8_t *getVRAM() {return vram;}
    uint8_t *getOAM() {return oam;}
  
    uint8_t *getCartridgeSave() {return cartSaveData;}
    SaveType getCartridgeSaveType() {return saveType;}
    /*void setCartRamUpdateCallback(CartRamUpdateCallback callback);*/

    const uint8_t *mapAddress(uint32_t addr) const;
    uint8_t *mapAddress(uint32_t addr);

    int getAccessCycles(uint32_t addr, int width, bool sequential) const;

private:

    enum class FlashState : uint8_t
    {
        Read,
        ID,
        Erase,
        Write,
        Bank,
    };

    void writeFlash(uint32_t addr, uint8_t data);

    AGBCPU &cpu;

    const uint8_t *biosROM = nullptr;
    uint8_t ewram[0x40000]; // external wram, two wait states, 16bit bus
    uint8_t iwram[0x8000]; // internal wram

    uint8_t ioRegs[0x400];
    
    uint8_t palRAM[0x400]; // 16bit bus
    uint8_t vram[0x18000]; // 16bit bus
    uint8_t oam[0x400];

    const uint8_t *cartROM = nullptr;
    uint32_t cartROMSize = 0;

    SaveType saveType = SaveType::Unknown;

    uint8_t eepromInBits[81]; // could be smaller if bits packed
    uint8_t eepromOutBits[68];
    uint8_t cartSaveData[128 * 1024]; // RAM/flash

    FlashState flashState = FlashState::Read;
    uint8_t flashCmdState = 0;
    uint8_t flashBank = 0;
    uint8_t flashID[2];

    uint32_t dummy = 0xBADADD55;

    //CartRamUpdateCallback cartRamUpdateCallback;
};