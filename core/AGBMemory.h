#pragma once
#include <cstdint>

class AGBMemory
{
public:
    using ReadCallback = uint8_t(*)(uint32_t, uint8_t val);
    using WriteCallback = bool(*)(uint32_t, uint8_t val);

    //using CartRamUpdateCallback = void(*)(uint8_t *, unsigned int);

    void loadBIOSROM(const uint8_t *rom);
    //void loadCartridgeRAM(const uint8_t *ram, uint32_t len);
    void reset();

    // only covers io registers
    void setIOReadCallback(ReadCallback readCallback);
    void setIOWriteCallback(WriteCallback writeCallback);

    uint8_t read8(uint32_t addr) const;
    uint16_t read16(uint32_t addr) const;

    void write8(uint32_t addr, uint8_t data);
  
    /*uint8_t *getCartridgeRAM() {return cartRam;}
    int getCartridgeRAMSize() {return cartRamSize;}
    void setCartRamUpdateCallback(CartRamUpdateCallback callback);*/

    const uint8_t *mapAddress(uint32_t &addr) const;
    uint8_t *mapAddress(uint32_t &addr);

private:
    uint8_t biosROM[0x4000];
    uint8_t ewram[0x40000]; // external wram, two wait states, 16bit bus
    uint8_t iwram[0x8000]; // internal wram

    uint8_t ioRegs[0x400];
    
    uint8_t palRAM[0x400]; // 16bit bus
    uint8_t vram[0x18000]; // 16bit bus
    uint8_t oam[0x400];

    uint8_t cartROM[0x80000]{}; // 16bit bus, much bigger than this (32MB)...

    ReadCallback readCallback;
    WriteCallback writeCallback;

    //CartRamUpdateCallback cartRamUpdateCallback;
};