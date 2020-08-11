#pragma once
#include <cstdint>

class AGBMemory;

class AGBCPU final
{
public:
    enum Interrupts
    {
        Int_LCDVBlank = 1 << 0,
        Int_LCDHBlank = 1 << 1,
        Int_LCDVCount = 1 << 2,
        Int_Timer0    = 1 << 3,
        Int_Timer1    = 1 << 4,
        Int_Timer2    = 1 << 5,
        Int_Timer3    = 1 << 6,
        Int_Serial    = 1 << 7,
        Int_DMA0      = 1 << 8,
        Int_DMA1      = 1 << 9,
        Int_DMA2      = 1 << 10,
        Int_DMA3      = 1 << 11,
        Int_Keypad    = 1 << 12,
        Int_External  = 1 << 13
    };

    AGBCPU(AGBMemory &mem);

    using CycleCallback = void(*)(int);

    void reset();

    void run(int ms);

    void setCycleCallback(CycleCallback cycleCallback);

    void flagInterrupt(int interrupt);

    AGBMemory &getMem() {return mem;}

private:
    enum class Reg
    {
        R0 = 0,
        R1,
        R2,
        R3,
        R4,
        R5,
        R6,
        R7,
        // ARM mode/high
        R8,
        R9,
        R10,
        R11,
        R12,
        R13,
        R14,
        R15,

        // banked
        R8_fiq,
        R9_fiq,
        R10_fiq,
        R11_fiq,
        R12_fiq,
        R13_fiq,
        R14_fiq,

        R13_svc,
        R14_svc,

        R13_abt,
        R14_abt,

        R13_irq,
        R14_irq,

        R13_und,
        R14_und,

        // aliases
        SP = R13, // also banked aliases...
        LR = R14,
        PC = R15
    };

    enum Flags
    {
        // control
        Flag_T = (1 << 5), // thumb
        Flag_F = (1 << 6), // FIQ disable
        Flag_I = (1 << 7), // IRQ disable 

        // condition codes
        Flag_V = (1 << 28),
        Flag_C = (1 << 29),
        Flag_Z = (1 << 30),
        Flag_N = (1 << 31)
    };

    Reg mapReg(Reg r) const
    {
        int iReg = static_cast<int>(r);
        int mode = cpsr & 0x1F;
        if(mode == 0x10/*User*/ || mode == 0x1F /*System*/ || r == Reg::PC || iReg < 8 || (mode != 0x11/*FIQ*/ && iReg < 13))
            return r;

        if(mode == 0x11) // FIQ
            return static_cast<Reg>(iReg + 8);

        if(mode == 0x13) // SVC
            return static_cast<Reg>(iReg + 10);
        
        if(mode == 0x17) // ABT
            return static_cast<Reg>(iReg + 12);

        if(mode == 0x12) // IRQ
            return static_cast<Reg>(iReg + 14);

        if(mode == 0x1B) // UND
            return static_cast<Reg>(iReg + 16);

        return r; // invalid mode!
    }

    uint32_t reg(Reg r) const {return regs[static_cast<int>(mapReg(r))];}
    uint32_t &reg(Reg r) {return regs[static_cast<int>(mapReg(r))];}

    // THUMB, first 8 regs
    uint32_t loReg(Reg r) const {return regs[static_cast<int>(r)];}
    uint32_t &loReg(Reg r) {return regs[static_cast<int>(r)];}

    uint32_t &getSPSR()
    {
        switch(cpsr & 0x1F)
        {
            case 0x11: // FIQ
                return spsr[0];
            case 0x12: // IRQ
                return spsr[3];
            case 0x13: // SVC
                return spsr[1];
            case 0x17: // ABT
                return spsr[2];
            case 0x1B: // UND
                return spsr[4];
        }

        return spsr[5]; // invalid mode!
    }

    uint8_t readMem8(uint32_t addr) const;
    uint32_t readMem16(uint32_t addr) const;
    uint16_t readMem16Aligned(uint32_t addr) const;
    uint32_t readMem32(uint32_t addr) const;
    uint32_t readMem32Aligned(uint32_t addr) const;
    void writeMem8(uint32_t addr, uint8_t data);
    void writeMem16(uint32_t addr, uint16_t data);
    void writeMem32(uint32_t addr, uint32_t data);

    int executeARMInstruction();
    int executeTHUMBInstruction();

    int doALUOp(int op, Reg destReg, uint32_t op1, uint32_t op2, bool setCondCode, bool carry);

    bool serviceInterrupts();

    static const uint32_t clockSpeed = 16*1024*1024;
    static const uint32_t signBit = 0x80000000;

    // internal state
    //bool stopped, halted;

    // registers
    uint32_t regs[31]{};
    uint32_t cpsr;
    uint32_t spsr[6]; // fiq, svc, abt, irq, und

    AGBMemory &mem;

    // callbacks
    CycleCallback cycleCallback;
};