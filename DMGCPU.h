#pragma once
#include <cstdint>

class DMGMemory;

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
    DMGCPU(DMGMemory &mem);

    using CycleCallback = void(*)(int);

    void reset();

    void run(int ms);

    void setCycleCallback(CycleCallback cycleCallback);

    void flagInterrupt(int interrupt);

    DMGMemory &getMem() {return mem;}

    uint16_t getInternalTimer() const {return divCounter;}

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

    // this only works on little-endian...
    uint8_t reg(Reg r) const {return reinterpret_cast<const uint8_t *>(regs)[static_cast<int>(r) ^ 1];}
    uint8_t &reg(Reg r) {return reinterpret_cast<uint8_t *>(regs)[static_cast<int>(r) ^ 1];}
    uint16_t reg(WReg r) const {return regs[static_cast<int>(r)];}
    uint16_t &reg(WReg r) {return regs[static_cast<int>(r)];}

    uint8_t readMem(uint16_t addr) const;
    uint16_t readMem16(uint16_t addr) const;
    void writeMem(uint16_t addr, uint8_t data);
    void writeMem16(uint16_t addr, uint16_t data);

    int executeInstruction();
    int executeExInstruction();

    void updateTimer(int cycles);
    void serviceInterrupts();

    static const uint32_t clockSpeed = 4194304;

    // internal state
    bool stopped, halted;
    bool masterInterruptEnable;
    uint16_t divCounter = 0;
    bool timerEnabled = false;
    int timerBit = 1 << 9;
    bool timerOldVal = false;

    // registers
    uint16_t regs[4];
    uint16_t pc, sp;

    // RAM
    DMGMemory &mem;

    // callbacks
    CycleCallback cycleCallback;
};