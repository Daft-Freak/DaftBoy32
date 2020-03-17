#pragma once
#include <cstdint>

enum Interrupts
{
    Int_VBlank  = 1 << 0,
    Int_LCDStat = 1 << 1,
    Int_Timer   = 1 << 2,
    Int_Serial  = 1 << 3,
    Int_Joypad  = 1 << 4,
};

class DMGCPU final
{
public:
    using CycleCallback = void(*)(int, uint8_t *);

    void loadCartridge(const uint8_t *rom, uint32_t romLen);
    void reset();

    void run(int ms);

    void setCycleCallback(CycleCallback cycleCallback);

    void flagInterrupt(int interrupt);

    void setInputs(uint8_t inputs);

    uint8_t readMem(uint16_t addr) const;
    void writeMem(uint16_t addr, uint8_t data);

private:
    enum class Reg
    {
        A = 0,
        F,
        B,
        C,
        D,
        E,
        H,
        L
    };

    enum class WReg
    {
        AF = 0,
        BC,
        DE,
        HL
    };

    enum Flags
    {
        Flag_C = (1 << 4),
        Flag_H = (1 << 5),
        Flag_N = (1 << 6),
        Flag_Z = (1 << 7)
    };

    enum class MBCType
    {
        None = 0,
        MBC1
    };

    // this only works on little-endian...
    uint8_t reg(Reg r) const {return reinterpret_cast<const uint8_t *>(regs)[static_cast<int>(r) ^ 1];}
    uint8_t &reg(Reg r) {return reinterpret_cast<uint8_t *>(regs)[static_cast<int>(r) ^ 1];}
    uint16_t reg(WReg r) const {return regs[static_cast<int>(r)];}
    uint16_t &reg(WReg r) {return regs[static_cast<int>(r)];}

    uint16_t readMem16(uint16_t addr) const;
    void writeMem16(uint16_t addr, uint16_t data);

    int executeInstruction();
    int executeExInstruction();

    void writeMBC(uint16_t addr, uint8_t data);

    static const uint32_t clockSpeed = 4194304;

    // internal state
    bool stopped, halted;
    bool masterInterruptEnable;
    uint16_t divCounter = 0;
    int timerCounter = 0;

    // registers
    uint16_t regs[4];
    uint16_t pc, sp;

    // RAM
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

    // callbacks
    CycleCallback cycleCallback;

    // raw input data
    uint8_t rawInputs = 0;
};