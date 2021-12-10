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
        EEPROM_512,
        EEPROM_8K,
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
    T read(uint32_t addr, int &cycles, bool sequential) const;
    template<class T>
    void write(uint32_t addr, T data, int &cycles, bool sequential);

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
    void updatePC(uint32_t pc);

    inline int iCycle(int i = 1)
    {
        prefetchCycles -= i;
        return i;
    }

    int prefetchTiming16(int cycles, int bugCycles = 0)
    {
        // not in ROM
        if(!pcInROM)
            return cycles;

        if(!cartPrefetchEnabled)
        {
            // prefetch disable bug, N cycle instead of S
            if(bugCycles)
                return bugCycles;
            
            return cycles;
        }

        prefetchCycles--;

        while(prefetchCycles <= 0)
        {
            if(prefetchedHalfWords < 8)
                prefetchedHalfWords++;
            prefetchCycles += prefetchSCycles;
        }

        if(prefetchedHalfWords)
        {
            prefetchedHalfWords--;
            return 1;
        }

        // prefetch isn't done... wait for it
        cycles = prefetchCycles + 1;
        prefetchCycles = prefetchSCycles;
        return cycles;
    }

    int prefetchTiming32(int cycles, int bugCycles = 0)
    {
        // not in ROM
        if(!pcInROM)
            return cycles;

        if(!cartPrefetchEnabled)
        {
            // prefetch disable bug, N cycle instead of S
            if(bugCycles)
                return bugCycles;
            
            return cycles;
        }

        prefetchCycles--;

        while(prefetchCycles <= 0)
        {
            if(prefetchedHalfWords < 8)
                prefetchedHalfWords++;
            prefetchCycles += prefetchSCycles;
        }

        if(prefetchedHalfWords > 1)
        {
            prefetchedHalfWords -= 2;
            return 1; // can fetch a word in one cycle
        }
        
        // prefetch isn't done... wait for it
        if(prefetchedHalfWords)
            cycles = 1 /*first half*/ + prefetchCycles/* + 1 */; // cycle for first half allows prefetch to continue?
        else
            cycles = prefetchCycles + 1 /*first half*/ + prefetchSCycles;

        prefetchedHalfWords = 0;
        prefetchCycles = prefetchSCycles;
        return cycles;
    }

    // verify that pointer returns the same as a regular read to the address
    // without affecting prefetch (used for asserts)
    template<class T>
    bool verifyPointer(const T *ptr, uint32_t addr)
    {
        auto tmpCycles = prefetchCycles;
        auto tmpHalfWords = prefetchedHalfWords;

        int tmp;
        bool ret = read<T>(addr, tmp, false) == *ptr;

        prefetchCycles = tmpCycles;
        prefetchedHalfWords = tmpHalfWords;
        
        return ret;
    }

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

    uint64_t eepromCommandData[2]; // max 81 bits
    uint64_t eepromReadData;
    uint8_t cartSaveData[128 * 1024]; // RAM/flash

    FlashState flashState = FlashState::Read;
    uint8_t flashCmdState = 0;
    uint8_t flashBank = 0;
    uint8_t flashID[2];

    uint32_t dummy = 0xBADADD55;

    int8_t cartAccessN[4], cartAccessS[4]; // ROM and RAM
    bool cartPrefetchEnabled = false, pcInROM = false;
    int prefetchSCycles = 0;
    mutable int prefetchCycles = 0;
    int prefetchedHalfWords = 0;

    //CartRamUpdateCallback cartRamUpdateCallback;
};