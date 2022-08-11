#pragma once
#include <cstdint>

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGMemory.h"


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
    enum class Console
    {
        Auto,
        DMG,
        CGB
    };

    DMGCPU();

    void reset();

    void run(int ms);

    Console getConsole() {return console;}
    void setConsole(Console c) {console = c;}

    void flagInterrupt(int interrupt);

    uint8_t readReg(uint16_t addr, uint8_t val);
    bool writeReg(uint16_t addr, uint8_t data);

    DMGMemory &getMem() {return mem;}
    DMGAPU &getAPU(){return apu;}
    DMGDisplay &getDisplay(){return display;}

    bool getStopped() const {return stopped;}
    bool getBreakpointTriggered() {return breakpoint;}

    uint32_t getCycleCount() const {return cycleCount;}
    uint16_t getInternalTimer() {updateTimer(); return divCounter;}

    bool getColourMode() const {return isGBC;} // CGB in CGB mode
    bool getDoubleSpeedMode() const {return doubleSpeed;}

    void setInputs(uint8_t newInputs);

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
    void writeMem(uint16_t addr, uint8_t data);

    void executeInstruction();
    void executeExInstruction();

    void cycleExecuted();

    void updateTimer();
    void incrementTimer();
    void caclulateNextTimerInterrupt(uint32_t cycleCount, uint16_t div);
    bool serviceInterrupts();

    void updateOAMDMA();
    void doGDMA();

    void updateSerial();
    void calculateNextSerialUpdate();

    static const uint32_t clockSpeed = 4194304;

    // internal state
    bool stopped, halted, breakpoint;
    bool masterInterruptEnable, enableInterruptsNextCycle;
    uint8_t serviceableInterrupts;
    bool haltBug = false;

    int cyclesToRun = 0;
    uint32_t cycleCount = 0;

    uint16_t divCounter = 0;
    bool timerEnabled = false;
    bool timerReload = false, timerReloaded = false;
    unsigned int timerBit = 1 << 9;
    bool timerOldVal = false;
    uint32_t lastTimerUpdate = 0;
    uint32_t nextTimerInterrupt = 0;

    bool isGBC = false;
    Console console = Console::Auto;
    bool doubleSpeed = false, speedSwitch = false;

    int oamDMACount, oamDMADelay;
    const uint8_t *oamDMASrc = nullptr;
    uint8_t *oamDMADest = nullptr;
    bool gdmaTriggered;

    bool serialStart = false, serialMaster = false;
    uint8_t serialBits = 0;
    uint32_t nextSerialBitCycle = 0;
    uint32_t lastSerialUpdate = 0;

    // registers
    uint16_t regs[4];
    uint16_t pc, sp;

    // RAM
    DMGMemory mem;
    
    DMGAPU apu;
    DMGDisplay display;
    uint8_t inputs = 0;
};
