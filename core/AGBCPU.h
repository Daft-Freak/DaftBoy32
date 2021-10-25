#pragma once
#include <cstdint>

#include "AGBAPU.h"
#include "AGBDisplay.h"
#include "AGBMemory.h"

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

    enum DMATrigger
    {
        Trig_VBlank = 1,
        Trig_HBlank
        // sound...
    };

    AGBCPU();

    void reset();

    void run(int ms);

    void flagInterrupt(int interrupt);
    void triggerDMA(int trigger);

    uint16_t readReg(uint32_t addr, uint16_t val);
    bool writeReg(uint32_t addr, uint16_t data);

    AGBAPU &getAPU() {return apu;}
    AGBDisplay &getDisplay() {return display;}
    AGBMemory &getMem() {return mem;}

    uint32_t getCycleCount() const {return cycleCount;}

    void setInputs(uint16_t newInputs);

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
        if(!regBankOffset || iReg < 8 || r == Reg::PC)
            return r;

        if(regBankOffset != 8/*FIQ*/ && iReg < 13)
            return r;

        return static_cast<Reg>(iReg + regBankOffset);
    }

    uint32_t reg(Reg r) const {return regs[static_cast<int>(mapReg(r))];}
    uint32_t &reg(Reg r) {return regs[static_cast<int>(mapReg(r))];}

    // THUMB, first 8 regs, also used when we don't want to map
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

    void modeChanged() // possibly
    {
        switch(cpsr & 0x1F)
        {
            case 0x10: // User
            case 0x1F: // System
                regBankOffset = 0;
                break;
            case 0x11: // FIQ
                regBankOffset = static_cast<int>(Reg::R8_fiq) - static_cast<int>(Reg::R8);
                break;
            case 0x13: // SVC
                regBankOffset = static_cast<int>(Reg::R13_svc) - static_cast<int>(Reg::R13);
                break;
            case 0x17: // ABT
                regBankOffset = static_cast<int>(Reg::R13_abt) - static_cast<int>(Reg::R13);
                break;
            case 0x12: // IRQ
                regBankOffset = static_cast<int>(Reg::R13_irq) - static_cast<int>(Reg::R13);
                break;
            case 0x1B: // UND
                regBankOffset = static_cast<int>(Reg::R13_und) - static_cast<int>(Reg::R13);
                break;
        }

        curSP = mapReg(Reg::SP);
        curLR = mapReg(Reg::LR);
    }

    uint8_t readMem8(uint32_t addr) const;
    uint32_t readMem16(uint32_t addr);
    uint16_t readMem16Aligned(uint32_t addr);
    uint32_t readMem32(uint32_t addr);
    uint32_t readMem32Aligned(uint32_t addr);
    void writeMem8(uint32_t addr, uint8_t data);
    void writeMem16(uint32_t addr, uint16_t data);
    void writeMem32(uint32_t addr, uint32_t data);

    int executeARMInstruction();
    int executeTHUMBInstruction();

    int doALUOp(int op, Reg destReg, uint32_t op1, uint32_t op2, bool carry);
    int doALUOpNoCond(int op, Reg destReg, uint32_t op1, uint32_t op2);

    int doTHUMB01MoveShifted(uint16_t opcode, uint32_t &pc);
    int doTHUMB0102(uint16_t opcode, uint32_t &pc);
    int doTHUMB03(uint16_t opcode, uint32_t &pc);
    int doTHUMB040506(uint16_t opcode, uint32_t &pc);
    int doTHUMB04ALU(uint16_t opcode, uint32_t &pc);
    int doTHUMB05HiReg(uint16_t opcode, uint32_t &pc);
    int doTHUMB06PCRelLoad(uint16_t opcode, uint32_t &pc);
    int doTHUMB0708(uint16_t opcode, uint32_t &pc);
    int doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t &pc);
    int doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t &pc);
    int doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t &pc);
    int doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t &pc);
    int doTHUMB12LoadAddr(uint16_t opcode, uint32_t &pc);
    int doTHUMB1314(uint16_t opcode, uint32_t &pc);
    int doTHUMB13SPOffset(uint16_t opcode, uint32_t &pc);
    int doTHUMB14PushPop(uint16_t opcode, uint32_t &pc);
    int doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t &pc);
    int doTHUMB1617(uint16_t opcode, uint32_t &pc);
    int doTHUMB18UncondBranch(uint16_t opcode, uint32_t &pc);
    int doTHUMB19LongBranchLink(uint16_t opcode, uint32_t &pc);

    void updateARMPC();
    void updateTHUMBPC(uint32_t pc);

    bool serviceInterrupts();

    int dmaTransfer(int channel);

    void updateTimers();

    static const uint32_t clockSpeed = 16*1024*1024;
    static const uint32_t signBit = 0x80000000;

    const uint32_t *armPCPtr = nullptr;
    const uint16_t *thumbPCPtr = nullptr;
    int pcSCycles = 0, pcNCycles = 0;

    // internal state
    //bool stopped, halted;
    bool halted;

    uint16_t currentInterrupts = 0; // IME ? (IE & IF) : 0
    uint16_t enabledInterrutps = 0;

    // dma
    uint8_t dmaTriggered = 0;

    uint32_t cycleCount = 0;

    // timers
    uint32_t lastTimerUpdate = 0;
    uint8_t timerEnabled = 0, timerInterruptEnabled = 0;

    uint16_t timerCounters[4]{};
    int timerPrescalers[4]{};

    // registers
    uint32_t regs[31]{};
    uint32_t cpsr;
    uint32_t spsr[6]; // fiq, svc, abt, irq, und

    Reg curSP = Reg::SP, curLR = Reg::LR;
    int regBankOffset = 0;

    uint16_t inputs = 0;
    
    AGBAPU apu;
    AGBDisplay display;
    AGBMemory mem;
};