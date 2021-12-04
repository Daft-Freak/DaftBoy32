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

    template<class T>
    T read(uint32_t addr) const;
    template<class T>
    void write(uint32_t addr, T data);

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

    void updateWaitControl(uint16_t waitcnt);

private:

    enum class FlashState : uint8_t
    {
        Read,
        ID,
        Erase,
        Write,
        Bank,
    };

    template<class T, size_t size>
    T doRead(const uint8_t (&mem)[size], uint32_t addr) const;
    template<class T, size_t size>
    void doWrite(uint8_t (&mem)[size], uint32_t addr, T data);

    template<class T>
    T doBIOSRead(uint32_t addr) const;

    template<class T>
    T doIORead(uint32_t addr) const;
    template<class T>
    void doIOWrite(uint32_t addr, T data);

    template<class T>
    void doPalRAMWrite(uint32_t addr, T data);

    template<class T>
    T doVRAMRead(uint32_t addr) const;
    template<class T>
    void doVRAMWrite(uint32_t addr, T data);

    template<class T>
    void doOAMWrite(uint32_t addr, T data);

    template<class T>
    T doROMRead(uint32_t addr) const;
    template<class T>
    T doROMOrEEPROMRead(uint32_t addr) const;
    template<class T>
    void doEEPROMWrite(uint32_t addr, T data);

    template<class T>
    T doSRAMRead(uint32_t addr) const;
    template<class T>
    void doSRAMWrite(uint32_t addr, T data);

    template<class T>
    T doOpenRead(uint32_t addr) const;

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

    int8_t cartAccessN[4], cartAccessS[4]; // ROM and RAM

    //CartRamUpdateCallback cartRamUpdateCallback;
};