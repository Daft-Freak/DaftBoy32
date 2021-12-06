#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib> //exit
#include <cstring>
#include <utility>

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

AGBCPU::AGBCPU() : apu(*this), display(*this), mem(*this)
{}

void AGBCPU::reset()
{
    cpsr = Flag_I | Flag_F | 0x13 /*supervisor mode*/;
    modeChanged();
    updateARMPC(0);
    halted = false;

    cycleCount = 0;
    lastTimerUpdate = 0;

    for(auto &c : timerCounters)
        c = 0;
    for(auto &p : timerPrescalers)
        p = 0;

    mem.reset();
    apu.reset();
    display.reset();
}

void AGBCPU::run(int ms)
{
    runCycles((clockSpeed * ms) / 1000);
}

void AGBCPU::runFrame()
{
    lastExtraCycles = runCycles(308 * 228 * 4 + lastExtraCycles);
}

void AGBCPU::flagInterrupt(int interrupt)
{
    mem.writeIOReg(IO_IF, mem.readIOReg(IO_IF) | interrupt);

    currentInterrupts = enabledInterrutps & mem.readIOReg(IO_IF);

    if(!interruptDelay)
        interruptDelay = 7; // unsure of this, but 7 seems to pass more tests than 6
}

void AGBCPU::triggerDMA(int trigger)
{
    for(int chan = 0; chan < 4; chan++)
    {
        auto control = mem.readIOReg(IO_DMA0CNT_H + chan * 12);
        if(!(control & DMACNTH_Enable))
            continue;

        if(((control & DMACNTH_Start) == 1 << 12 && (trigger == Trig_VBlank)) ||
            ((control & DMACNTH_Start) == 2 << 12 && (trigger == Trig_HBlank)))
        {
            dmaTriggered |= 1 << chan;
        }

        // special triggers
        else if((control & DMACNTH_Start) == 3 << 12)
        {
            if(chan == 1 && trigger == Trig_SoundA)
                dmaTriggered |= 1 << chan;
            else if(chan == 2 && trigger == Trig_SoundB)
                dmaTriggered |= 1 << chan;
        }
    }
}

uint16_t AGBCPU::readReg(uint32_t addr, uint16_t val)
{
    if((addr & 0xFFFFFF) < IO_SOUND1CNT_L)
        return display.readReg(addr, val);
    else if((addr & 0xFFFFFF) <= IO_FIFO_B)
        return apu.readReg(addr, val);

    switch(addr & 0xFFFFFF)
    {
        // not readable
        // TODO: src/dest regs are open bus
        case IO_DMA0CNT_L:
        case IO_DMA1CNT_L:
        case IO_DMA2CNT_L:
        case IO_DMA3CNT_L:
            return 0;

        case IO_DMA0CNT_H:
        case IO_DMA1CNT_H:
        case IO_DMA2CNT_H:
            return val & ~(DMACNTH_GamePak | 0x1F); // game pak bit is DMA3 only, low 5 bits are unused

        case IO_DMA3CNT_H:
            return val & ~0x1F; // game pak bit is DMA3 only, low 5 bits are unused

        case IO_TM0CNT_L:
        case IO_TM1CNT_L:
        case IO_TM2CNT_L:
        case IO_TM3CNT_L:
            // sync
            updateTimers();
            return timerCounters[((addr & 0xFFFFFF) - IO_TM0CNT_L) >> 2];

        case IO_KEYINPUT:
            return ~inputs;
    }
    return val;
}

bool AGBCPU::writeReg(uint32_t addr, uint16_t data, uint16_t mask)
{
    if(display.writeReg(addr, data) || apu.writeReg(addr, data, mask))
        return true;

    switch(addr & 0xFFFFFF)
    {
        case IO_DMA0CNT_H:
        case IO_DMA1CNT_H:
        case IO_DMA2CNT_H:
        case IO_DMA3CNT_H:
        {
            int index = ((addr & 0xFFFFFF) - IO_DMA0CNT_H) / 12;
            if(data & DMACNTH_Enable)
            {
                if((data & DMACNTH_Start) == 0)
                    dmaTriggered |= (1 << index);

                // reload on enable
                if(!(mem.readIOReg(addr) & DMACNTH_Enable))
                {
                    int regOffset = (addr & 0xFFFFFF) - IO_DMA0CNT_H;
                    dmaSrc[index] = (mem.readIOReg(IO_DMA0SAD + regOffset) | (mem.readIOReg(IO_DMA0SAD + regOffset + 2) << 16)) & 0xFFFFFFF;
                    dmaDst[index] = dmaCurDst[index] = (mem.readIOReg(IO_DMA0DAD + regOffset) | (mem.readIOReg(IO_DMA0DAD + regOffset + 2) << 16)) & 0xFFFFFFF;
                    dmaCount[index] = dmaCurCount[index] = mem.readIOReg(IO_DMA0CNT_L + regOffset);

                    // reading from ROM with DMA0 is invalid, remap it to something low so that the check later stops it
                    if(index == 0 && dmaSrc[index] >= 0x8000000)
                        dmaSrc[0] = 0;

                    // only DMA3 can write to the ROM area
                    if(index != 3 && dmaDst[index] >= 0x8000000)
                        dmaDst[index] = dmaCurDst[index] = 0;
                }
            }
            else
                dmaTriggered &= ~(1 << index);

            break;
        }

        case IO_TM0CNT_L:
        case IO_TM1CNT_L:
        case IO_TM2CNT_L:
        case IO_TM3CNT_L:
        {
            // sync
            updateTimers();
            break;
        }

        case IO_TM0CNT_H:
        case IO_TM1CNT_H:
        case IO_TM2CNT_H:
        case IO_TM3CNT_H:
        {
            int index = ((addr & 0xFFFFFF) - IO_TM0CNT_H) >> 2;
            static const int prescalers[]{1, 64, 256, 1024};

            // sync
            updateTimers();

            if(data & TMCNTH_Enable)
            {
                if(!(mem.readIOReg(IO_TM0CNT_H + index * 4) & TMCNTH_Enable))
                    timerCounters[index] = mem.readIOReg(IO_TM0CNT_L + index * 4); // reload counter

                if(data & TMCNTH_CountUp)
                    timerPrescalers[index] = -1; // magic value for count-up mode
                else
                    timerPrescalers[index] = prescalers[data & TMCNTH_Prescaler];

                timerEnabled |= (1 << index);

                if(data & TMCNTH_IRQEnable)
                    timerInterruptEnabled |= (1 << index);
                else
                    timerInterruptEnabled &= ~(1 << index);
            }
            else
            {
                timerEnabled &= ~(1 << index);
                timerInterruptEnabled &= ~(1 << index);
            }

            if(timerEnabled)
                calculateNextTimerOverflow(cycleCount);

            break;
        }

        case IO_IE:
            enabledInterrutps = (mem.readIOReg(IO_IME) & 1) ? data : 0;
            currentInterrupts = enabledInterrutps & mem.readIOReg(IO_IF);
            break;

        case IO_IF: // writing IF bits clears them internally
            data = mem.readIOReg(IO_IF) & ~data;
            mem.writeIOReg(IO_IF, data);

            currentInterrupts = (mem.readIOReg(IO_IME) & 1) ? mem.readIOReg(IO_IE) & data : 0;
            return true;

        case IO_WAITCNT:
            mem.updateWaitControl(data);
            pcSCycles = mem.getAccessCycles(loReg(Reg::PC), cpsr & Flag_T ? 2 : 4, true);
            pcNCycles = mem.getAccessCycles(loReg(Reg::PC), cpsr & Flag_T ? 2 : 4, false);
            break;

        case IO_IME:
            enabledInterrutps = (data & 1) ? mem.readIOReg(IO_IE) : 0;
            currentInterrupts = enabledInterrutps & mem.readIOReg(IO_IF);
            break;

        case IO_HALTCNT - 1: // the address of POSTFLG, but we're ignoring that
            if(mask >> 8) // ignore POSTFLG write
            {
                if(data & 0x8000)
                    printf("STOP\n");
                else
                    halted = true;
            }
            break;
    }

    return false;
}

void AGBCPU::setInputs(uint16_t newInputs)
{
    if(inputs == 0 && newInputs != 0)
        flagInterrupt(Int_Keypad);

    inputs = newInputs;
}

uint8_t AGBCPU::readMem8(uint32_t addr, int &cycles, bool sequential) const
{
    return mem.read<uint8_t>(addr, cycles, sequential);
}

uint32_t AGBCPU::readMem16(uint32_t addr, int &cycles, bool sequential)
{
    if(!(addr & 1))
        return readMem16Aligned(addr, cycles, sequential);

    // this returns the 32-bit result of an unaligned 16-bit read
    uint32_t val = mem.read<uint16_t>(addr, cycles, sequential);

    return (val >> 8) | (val << 24);
}

uint16_t AGBCPU::readMem16Aligned(uint32_t addr, int &cycles, bool sequential)
{
    assert((addr & 1) == 0);

    return mem.read<uint16_t>(addr, cycles, sequential);
}

uint32_t AGBCPU::readMem32(uint32_t addr, int &cycles, bool sequential)
{
    if(!(addr & 3))
        return readMem32Aligned(addr, cycles, sequential);

    uint32_t val = mem.read<uint32_t>(addr, cycles, sequential);

    int shift = (addr & 3) << 3;
    return (val >> shift) | (val << (32 - shift));
}

uint32_t AGBCPU::readMem32Aligned(uint32_t addr, int &cycles, bool sequential)
{
    assert((addr & 3) == 0);

    return mem.read<uint32_t>(addr, cycles, sequential);
}

void AGBCPU::writeMem8(uint32_t addr, uint8_t data, int &cycles, bool sequential)
{
    mem.write<uint8_t>(addr, data, cycles, sequential);
}

void AGBCPU::writeMem16(uint32_t addr, uint16_t data, int &cycles, bool sequential)
{
    mem.write<uint16_t>(addr, data, cycles, sequential);
}

void AGBCPU::writeMem32(uint32_t addr, uint32_t data, int &cycles, bool sequential)
{
    mem.write<uint32_t>(addr, data, cycles, sequential);
}

int AGBCPU::runCycles(int cycles)
{
    while(cycles > 0)
    {
        unsigned int exec = 1;

        // DMA
        if(dmaTriggered)
        {
            exec = 0;
            auto trig = dmaTriggered;
            for(int chan = 0; chan < 4 && trig; chan++, trig >>= 1)
            {
                if(trig & 1)
                    exec += dmaTransfer(chan);

                // channel still active, don't update next channel
                if(dmaActive & (1 << chan))
                    break;
            }
        }
        else if(!halted)
        {
            // CPU
            exec = (cpsr & Flag_T) ? executeTHUMBInstruction() : executeARMInstruction();
        }

        // loop until not halted or DMA was triggered
        do
        {
            bool shouldUpdateTimers = timerEnabled && nextTimerUpdate - cycleCount <= exec;

            cycles -= exec;
            cycleCount += exec;

            if(shouldUpdateTimers)
                updateTimers();

            bool displayInterruptsEnabled = enabledInterrutps & (Int_LCDVBlank | Int_LCDHBlank | Int_LCDVCount);

            if(displayInterruptsEnabled)
                display.update();

            if(currentInterrupts)
            {
                if(interruptDelay <= exec)
                {
                    serviceInterrupts(); // cycles?
                    interruptDelay = 0;
                }
                else
                    interruptDelay -= exec;
            }

            if(halted && cycles > 0)
            {
                // skip ahead
                exec = cycles;

                // limit to display interrupt
                if(displayInterruptsEnabled)
                    exec = std::min(cycles, display.getCyclesToNextUpdate());

                // limit to next timer overflow
                if(timerEnabled)
                    exec = std::min(exec, nextTimerUpdate - cycleCount);

                if(interruptDelay)
                    exec = std::min(exec, static_cast<unsigned int>(interruptDelay));

                assert(exec > 0);
            }
        }
        while(halted && !(dmaTriggered) && cycles > 0);
    }

    return cycles;
}

// returns cycle count
int AGBCPU::executeARMInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    uint32_t opcode = decodeOp;

    // decode stage - loads to do here... not
    decodeOp = fetchOp;

    // fetch next
    assert(*armPCPtr == mem.read32(pc));
    fetchOp = *++armPCPtr;

    // ... and execute

    pc += 4;

    // 0-3
    const auto doDataProcessing = [this](uint32_t opcode, uint32_t op2, bool carry, int pcInc = 0)
    {
        auto op1Reg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto op1 = reg(op1Reg);
        if(op1Reg == Reg::PC)
            op1 += pcInc;

        auto instOp = (opcode >> 21) & 0xF;
        bool setCondCode = (opcode & (1 << 20));
        auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
        return setCondCode ? doALUOp(instOp, destReg, op1, op2, carry) : doALUOpNoCond(instOp, destReg, op1, op2);
    };

    // condition
    auto cond = opcode >> 28;

    // the conditions here have to be backwards...
    switch(cond)
    {
        case 0x0: // equal
            if(!(cpsr & Flag_Z))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x1: // not equal
            if(cpsr & Flag_Z)
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x2: // carry set
            if(!(cpsr & Flag_C))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x3: // carry clear
            if(cpsr & Flag_C)
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x4: // negative
            if(!(cpsr & Flag_N))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x5: // positive or zero
            if((cpsr & Flag_N))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x6: // overflow
            if(!(cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x7: // no overflow
            if((cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x8: // unsigned higher
            if(!(cpsr & Flag_C) || (cpsr & Flag_Z))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0x9: // unsigned lower or same
            if((cpsr & Flag_C) && !(cpsr & Flag_Z))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0xA: // greater or equal
            if(!!(cpsr & Flag_N) != !!(cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0xB: // less than
            if(!!(cpsr & Flag_N) == !!(cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0xC: // greater than
            if((cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0xD: // less than or equal
            if(!(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V))
                return mem.prefetchTiming32(pcSCycles);
            break;
        case 0xE: // always
            break;
        // F is reserved
        default:
            assert(!"Invalid condition");
    }

    switch((opcode >> 24) & 0xF)
    {
        case 0x0: // data processing with register (and halfword transfer/multiply)
        {
            if(((opcode >> 4) & 9) == 9)
            {
                if((opcode >> 5) & 3) // halfword transfer
                    return doARMHalfwordTransfer(opcode, false);

                return doARMMultiply(opcode);
            }

            auto op2Shift = (opcode >> 4) & 0xFF;
            auto op2Reg = static_cast<Reg>(opcode & 0xF);

            // reg arg2
            bool carry;
            auto op2 = getARMShiftedReg(op2Reg, op2Shift, carry);

            return doDataProcessing(opcode, op2, carry, (op2Shift & 1) ? 4 : 0) + (op2Shift & 1); // +1I if shift by reg
        }
        case 0x1: // data processing with register (and branch exchange/swap)
        {
            if((opcode & 0x0FFFFF00) == 0x012FFF00) // Branch and Exchange (BX)
            {
                auto instOp = (opcode >> 4) & 0xF;
                assert(instOp == 1); //
                auto newPC = reg(static_cast<Reg>(opcode & 0xF));

                if(newPC & 1)
                {
                    cpsr |= Flag_T;
                    updateTHUMBPC(newPC & ~1);
                }
                else
                    updateARMPC(newPC);

                return pcSCycles * 2 + pcNCycles;
            }

            if(((opcode >> 4) & 9) == 9)
            {
                if((opcode >> 5) & 3) // halfword transfer
                    return doARMHalfwordTransfer(opcode, true);

                // SWP
                bool isByte = opcode & (1 << 22);
                auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
                auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
                auto srcReg = static_cast<Reg>(opcode & 0xF);

                auto addr = reg(baseReg);

                int cycles = 0;

                if(isByte)
                {
                    auto v = readMem8(addr, cycles);
                    writeMem8(addr, reg(srcReg), cycles);
                    reg(destReg) = v;
                }
                else
                {
                    auto v = readMem32(addr, cycles);
                    writeMem32(addr, reg(srcReg), cycles);
                    reg(destReg) = v;
                }

                return cycles + mem.iCycle() + mem.prefetchTiming32(pcSCycles);
            }

            auto instOp = (opcode >> 21) & 0xF;
            bool setCondCode = (opcode & (1 << 20));

            if(!setCondCode && (instOp >= 0x8/*TST*/ && instOp <= 0xB /*CMN*/)) // PSR Transfer
            {
                bool isSPSR = opcode & (1 << 22);
                if(opcode & (1 << 21)) // MSR
                {
                    assert((opcode & 0xFFF0) == 0xF000);

                    bool wF = opcode & (1 << 19);
                    // 17/18 should be 0
                    bool wC = opcode & (1 << 16);
                    auto val = reg(static_cast<Reg>(opcode & 0xF));

                    uint32_t mask = (wF ? 0xFF000000 : 0) |
                                    (wC ? 0x000000FF : 0);

                    if(isSPSR)
                    {
                        auto &spsr = getSPSR();
                        spsr = (spsr & ~mask) | (val & mask);
                    }
                    else
                    {
                        cpsr = (cpsr & ~mask) | (val & mask);
                        modeChanged();
                    }
                }
                else // MRS
                {
                    assert((opcode & 0xF0FFF) == 0xF0000);
                    auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
                    if(isSPSR)
                        reg(destReg) = getSPSR();
                    else
                        reg(destReg) = cpsr;
                }

                return mem.prefetchTiming32(pcSCycles);
            }

            auto op2Shift = (opcode >> 4) & 0xFF;
            auto op2Reg = static_cast<Reg>(opcode & 0xF);

            // reg arg2
            bool carry;
            auto op2 = getARMShiftedReg(op2Reg, op2Shift, carry);

            return doDataProcessing(opcode, op2, carry, (op2Shift & 1) ? 4 : 0) + (op2Shift & 1); // +1I if shift by reg
        }
        case 0x2: // data processing with immediate
        {
            // get the immediate value
            uint32_t op2 = opcode & 0xFF;
            int shift = ((opcode >> 8) & 0xF) * 2;
            op2 = (op2 >> shift) | (op2 << (32 - shift));
            bool carry = shift ? op2 & (1 << 31) : cpsr & Flag_C;

            return doDataProcessing(opcode, op2, carry);
        }
        case 0x3: // same as above, but also possibly MSR
        {
            auto instOp = (opcode >> 21) & 0xF;
            bool setCondCode = (opcode & (1 << 20));

            // get the immediate value
            uint32_t op2 = opcode & 0xFF;
            int shift = ((opcode >> 8) & 0xF) * 2;

            op2 = (op2 >> shift) | (op2 << (32 - shift));
            bool carry = shift ? op2 & (1 << 31) : cpsr & Flag_C;

            // TODO: a bit duplicated with 0/1
            if(!setCondCode && (instOp >= 0x8/*TST*/ && instOp <= 0xB /*CMN*/)) // MSR
            {
                bool isSPSR = opcode & (1 << 22);

                assert((opcode & 0xF000) == 0xF000);
                assert(opcode & (1 << 21));

                bool wF = opcode & (1 << 19);
                // 17/18 should be 0
                bool wC = opcode & (1 << 16);
                auto val = op2;

                uint32_t mask = (wF ? 0xFF000000 : 0) |
                                (wC ? 0x000000FF : 0);

                if(isSPSR)
                {
                    auto &spsr = getSPSR();
                    spsr = (spsr & ~mask) | (val & mask);
                }
                else
                {
                    cpsr = (cpsr & ~mask) | (val & mask);
                    modeChanged();
                }

                return mem.prefetchTiming32(pcSCycles);
            }

            return doDataProcessing(opcode, op2, carry);
        }
        case 0x4: // Single Data Transfer (I = 0, P = 0)
            return doARMSingleDataTransfer(opcode, false, false);
        case 0x5: // Single Data Transfer (I = 0, P = 1)
            return doARMSingleDataTransfer(opcode, false, true);
        case 0x6: // Single Data Transfer (I = 1, P = 0)
            return doARMSingleDataTransfer(opcode, true, false);
        case 0x7: // Single Data Transfer (I = 1, P = 1)
            return doARMSingleDataTransfer(opcode, true, true);
        case 0x8: // Block Data Transfer (P = 0)
            return doARMBlockDataTransfer(opcode, false);
        case 0x9: // Block Data Transfer (P = 1)
            return doARMBlockDataTransfer(opcode, true);
        case 0xA: // Branch (B)
        {
            auto offset = (static_cast<int32_t>(opcode & 0xFFFFFF) << 8) >> 6;
            updateARMPC(pc + offset);
            return pcSCycles * 2 + pcNCycles;
        }
        case 0xB: // Branch with Link (BL)
        {
            auto offset = (static_cast<int32_t>(opcode & 0xFFFFFF) << 8) >> 6;
            reg(Reg::LR) = pc - 4;
            updateARMPC(pc + offset);
            return pcSCycles * 2 + pcNCycles;
        }

        case 0xF: // SWI
        {
            auto ret = pc - 4;
            spsr[1/*svc*/] = cpsr;

            cpsr = (cpsr & ~0x1F) | Flag_I | 0x13; //supervisor mode
            modeChanged();
            updateARMPC(8);
            loReg(curLR) = ret;
            return pcSCycles * 2 + pcNCycles;
        }

        default:
            printf("ARM op %x @%x\n", opcode & 0xFFFFFFF, pc - 4);
            exit(0);
            break;
    }

    __builtin_unreachable();
}

int AGBCPU::executeTHUMBInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    uint16_t opcode = decodeOp;

    decodeOp = fetchOp;

    assert(!(pc & 1));
    assert(*thumbPCPtr == mem.read16(pc));
    fetchOp = *++thumbPCPtr;

    pc += 2;

    switch(opcode >> 12)
    {
        case 0x0: // format 1
            return doTHUMB01MoveShifted(opcode, pc);
        case 0x1: // formats 1-2
            return doTHUMB0102(opcode, pc);
        case 0x2: // format 3, mov/cmp immediate
        case 0x3: // format 3, add/sub immediate
            return doTHUMB03(opcode, pc);
        case 0x4: // formats 4-6
            return doTHUMB040506(opcode, pc);
        case 0x5: // formats 7-8
            return doTHUMB0708(opcode, pc);
        case 0x6: // format 9, load/store with imm offset (words)
            return doTHUMB09LoadStoreWord(opcode, pc);
        case 0x7: // ... (bytes)
            return doTHUMB09LoadStoreByte(opcode, pc);
        case 0x8: // format 10, load/store halfword
            return doTHUMB10LoadStoreHalf(opcode, pc);
        case 0x9: // format 11, SP-relative load/store
            return doTHUMB11SPRelLoadStore(opcode, pc);
        case 0xA: // format 12, load address
            return doTHUMB12LoadAddr(opcode, pc);
        case 0xB: // formats 13-14
            return doTHUMB1314(opcode, pc);
        case 0xC: // format 15, multiple load/store
            return doTHUMB15MultiLoadStore(opcode, pc);
        case 0xD: // formats 16-17
            return doTHUMB1617(opcode, pc);
        case 0xE: // format 18, unconditional branch
            return doTHUMB18UncondBranch(opcode, pc);
        case 0xF: // format 19, long branch with link
            return doTHUMB19LongBranchLink(opcode, pc);
    }

    __builtin_unreachable();
}

uint32_t AGBCPU::getARMShiftedReg(Reg r, uint8_t shift, bool &carry)
{
    auto ret = reg(r);

    // prefetch
    if(r == Reg::PC && (shift & 1))
        ret += 4;

    if(!shift) // left shift by immediate 0, do nothing and preserve carry
    {
        carry = cpsr & Flag_C;
        return ret;
    }

    int shiftType = (shift >> 1) & 3;
    int shiftAmount;
    if(shift & 1)
    {
        assert((shift & (1 << 3)) == 0);
        shiftAmount = reg(static_cast<Reg>(shift >> 4)) & 0xFF;

        if(!shiftAmount)
        {
            carry = cpsr & Flag_C;
            return ret;
        }
    }
    else
    {
        shiftAmount = shift >> 3;
        if(shiftAmount == 0) // lsr/asr shift by 32 instead of 0
            shiftAmount = 32;
    }

    switch(shiftType)
    {
        case 0: // LSL
            if(shiftAmount >= 32)
            {
                carry = shiftAmount == 32 ? (ret & 1) : 0;
                ret = 0;
            }
            else
            {
                carry = ret & (1 << (32 - shiftAmount));
                ret <<= shiftAmount;
            }
            break;
        case 1: // LSR
            if(shiftAmount >= 32)
            {
                carry = shiftAmount == 32 ? (ret & (1 << 31)) : 0;
                ret = 0;
            }
            else
            {
                carry = ret & (1 << (shiftAmount - 1));
                ret >>= shiftAmount;
            }
            break;
        case 2: // ASR
        {
            auto sign = ret & signBit;
            if(shiftAmount >= 32)
            {
                ret = sign ? 0xFFFFFFFF : 0;
                carry = sign;
            }
            else
            {
                carry = ret & (1 << (shiftAmount - 1));
                ret = static_cast<int32_t>(ret) >> shiftAmount;
            }
            break;
        }
        case 3:
            if(!(shift & 1) && shiftAmount == 32) // RRX (immediate 0)
            {
                carry = ret & 1; // carry out

                ret >>= 1;

                if(cpsr & Flag_C) // carry in
                    ret |= 0x80000000;
            }
            else // ROR
            {
                shiftAmount &= 0x1F;

                ret = (ret >> shiftAmount) | (ret << (32 - shiftAmount));
                carry = ret & (1 << 31);
            }
            break;

        default:
            assert(!"Invalid shift type!");
    }
    
    return ret;
}

int AGBCPU::doARMHalfwordTransfer(uint32_t opcode, bool isPre)
{
    //bool isUp = opcode & (1 << 23);
    //bool isImm = opcode & (1 << 22);
    //bool writeBack = opcode & (1 << 21);
    //bool isLoad = opcode & (1 << 20);
    auto baseReg = mapReg(static_cast<Reg>((opcode >> 16) & 0xF));
    auto srcDestReg = mapReg(static_cast<Reg>((opcode >> 12) & 0xF));

    int offset;

    if(opcode & (1 << 22)) // immediate
        offset = ((opcode >> 4) & 0xF0) | (opcode & 0xF);
    else
    {
        offset = reg(static_cast<Reg>(opcode & 0xF));
        assert((opcode & 0xF00) == 0);
    }

    if(!(opcode & (1 << 23))) // !up
        offset = -offset;

    auto addr = loReg(baseReg);

    // get value for store before write back
    auto val = loReg(srcDestReg);

    if(isPre)
    {
        addr += offset;
        if(opcode & (1 << 21)) // write back
            loReg(baseReg) = addr;
    }
    else
    {
        assert(!(opcode & (1 << 21))); // writeback should not be set
        loReg(baseReg) += offset; // always writes back
    }

    if(opcode & (1 << 20)) // load
    {
        bool sign = opcode & (1 << 6);
        bool halfWords = opcode & (1 << 5);

        int cycles = 0;

        if(halfWords && !sign)
            loReg(srcDestReg) = readMem16(addr, cycles); // LDRH
        else if(halfWords && !(addr & 1)) // LDRSH (aligned)
            loReg(srcDestReg) = static_cast<int16_t>(readMem16Aligned(addr, cycles)); // sign extend
        else // LDRSB ... or misaligned LDRSH
            loReg(srcDestReg) = static_cast<int8_t>(readMem8(addr, cycles)); // sign extend

        return cycles + mem.iCycle() + mem.prefetchTiming32(pcSCycles, pcNCycles);
    }
    else
    {
        // only unsigned halfword stores
        assert(opcode & (1 << 5)); // half
        assert(!(opcode & (1 << 6))); // sign

        if(srcDestReg == Reg::PC)
            val += 4;

        int cycles = 0;

        writeMem16(addr, val, cycles); // STRH

        return cycles + mem.prefetchTiming32(pcNCycles);
    }
}

int AGBCPU::doARMMultiply(uint32_t opcode)
{
    if(opcode & (1 << 23)) // MULL/MLAL
    {
        bool isSigned = opcode & (1 << 22);
        bool accumulate = opcode & (1 << 21);
        bool setCondCode = opcode & (1 << 20);
        auto destHiReg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto destLoReg = static_cast<Reg>((opcode >> 12) & 0xF);
        auto op2Reg = static_cast<Reg>((opcode >> 8) & 0xF);
        auto op1Reg = static_cast<Reg>(opcode & 0xF);

        auto op2 = reg(op2Reg);

        uint64_t res;

        if(isSigned) // SMULL
            res = static_cast<uint64_t>(static_cast<int64_t>(static_cast<int32_t>(reg(op1Reg))) * static_cast<int32_t>(op2));
        else // UMULL
            res = static_cast<uint64_t>(reg(op1Reg)) * op2;
        
        if(accumulate) // S/UMLAL
            res += (static_cast<uint64_t>(reg(destHiReg)) << 32) | reg(destLoReg);

        reg(destHiReg) = res >> 32;
        reg(destLoReg) = res & 0xFFFFFFFF;

        if(setCondCode)
        {
            // v and c are meaningless
            cpsr = (cpsr & ~(Flag_N | Flag_Z))
                    | (res & (1ull << 63) ? Flag_N : 0)
                    | (res == 0 ? Flag_Z : 0);
        }

        // leading 0s or 1s
        auto tmp = (isSigned && (op2 & (1 << 31))) ? ~op2 : op2;
        int prefix = tmp ? __builtin_clz(tmp) : 32;

        // more cycles the more bytes are non 0/ff (or just non 0 for unsigned)
        int iCycles = (prefix == 32 ? 1 : (4 - prefix / 8)) + (accumulate ? 1 : 0);
        return mem.iCycle(iCycles + 1) + mem.prefetchTiming32(pcSCycles, pcNCycles);
    }
    else // MUL/MLA
    {
        bool accumulate = opcode & (1 << 21);
        bool setCondCode = opcode & (1 << 20);
        auto destReg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto op3Reg = static_cast<Reg>((opcode >> 12) & 0xF);
        auto op2Reg = static_cast<Reg>((opcode >> 8) & 0xF);
        auto op1Reg = static_cast<Reg>(opcode & 0xF);

        auto op2 = reg(op2Reg);

        uint32_t res = reg(op1Reg) * op2;

        if(accumulate)
            res += reg(op3Reg);
        else
            assert(op3Reg == Reg::R0); // should be 0

        reg(destReg) = res;

        if(setCondCode)
        {
            // v is unaffected, c is meaningless
            cpsr = (cpsr & ~(Flag_N | Flag_Z))
                    | (res & signBit ? Flag_N : 0)
                    | (res == 0 ? Flag_Z : 0);
        }

        // leading 0s or 1s
        auto tmp = (op2 & (1 << 31)) ? ~op2 : op2;
        int prefix = tmp ? __builtin_clz(tmp) : 32;

        // more cycles the more bytes are non 0/ff
        int iCycles = (prefix == 32 ? 1 : (4 - prefix / 8)) + (accumulate ? 1 : 0);
        return mem.iCycle(iCycles) + mem.prefetchTiming32(pcSCycles, pcNCycles);
    }
}

int AGBCPU::doARMSingleDataTransfer(uint32_t opcode, bool isReg, bool isPre)
{
    //bool isUp = opcode & (1 << 23);
    //bool writeBack = opcode & (1 << 21);
    //bool isLoad = opcode & (1 << 20);
    auto baseReg = mapReg(static_cast<Reg>((opcode >> 16) & 0xF));
    auto srcDestReg = mapReg(static_cast<Reg>((opcode >> 12) & 0xF));
    int offset;

    if(!isReg) // immediate
        offset = opcode & 0xFFF;
    else
    {
        assert((opcode & (1 << 4)) == 0); // no reg shift
        bool carry;
        offset = getARMShiftedReg(static_cast<Reg>(opcode & 0xF), (opcode >> 4) & 0xFE, carry);
    }

    if(!(opcode & (1 << 23))) // !up
        offset = -offset;

    auto addr = loReg(baseReg);

    // get value for store before write back
    auto val = loReg(srcDestReg);

    if(isPre)
    {
        addr += offset;
        if(opcode & (1 << 21)) // write back
            loReg(baseReg) = addr;
    }
    else
    {
        assert(!(opcode & (1 << 21))); // non-privileged transfer
        loReg(baseReg) += offset; // always writes back
    }

    bool isByte = opcode & (1 << 22);
    if(opcode & (1 << 20)) // load
    {
        int cycles = 0;
        uint32_t val = isByte ? readMem8(addr, cycles) : readMem32(addr, cycles);

        if(srcDestReg == Reg::PC)
        {
            updateARMPC(val);
            return cycles + mem.iCycle() + pcSCycles * 2 + pcNCycles;
        }
        else
            loReg(srcDestReg) = val;

        return cycles + mem.iCycle() + mem.prefetchTiming32(pcSCycles, pcNCycles);
    }
    else
    {
        if(srcDestReg == Reg::PC)
            val += 4;

        int cycles = 0;

        if(isByte)
            writeMem8(addr, val, cycles);
        else
            writeMem32(addr, val, cycles);

        return cycles + mem.prefetchTiming32(pcNCycles);
    }
}

int AGBCPU::doARMBlockDataTransfer(uint32_t opcode, bool preIndex)
{
    bool isUp = opcode & (1 << 23);
    bool isLoadForce = opcode & (1 << 22);
    bool writeBack = opcode & (1 << 21);
    bool isLoad = opcode & (1 << 20);
    auto baseReg = mapReg(static_cast<Reg>((opcode >> 16) & 0xF));
    uint16_t regList = opcode;

    int cycles = 0;

    if(isLoadForce)
    {
        //assert(!writeBack); // "should not be used"
        assert(!isLoad || !(regList & (1 << 15))); // TODO: load with r15 (mode change)
    }

    auto addr = loReg(baseReg);

    // count regs
    int numRegs = 0;
    for(uint16_t t = regList; t; t >>= 1)
    {
        if(t & 1)
            numRegs++;
    }

    uint32_t lowAddr = 0;
    auto highAddr = addr + numRegs * 4;

    // flip decrement addressing around so that regs are stored in the right order    
    if(!isUp)
    {
        addr -= numRegs * 4;
        lowAddr = addr;

        if(!preIndex)
            addr += 4;
    }
    else if(preIndex)
        addr += 4;

    if(isLoad && regList & (1 << static_cast<int>(baseReg)))
        writeBack = false; // don't override

    // empty list loads/stores R15/PC
    if(regList == 0)
    {
        regList = 1 << 15;

        // ...ans inc/decrements all the way
        if(isUp)
            highAddr += 0x40;
        else
        {
            addr -= 0x40;
            lowAddr = addr - (preIndex ? 0 : 4);
        }
    }

    int i = 0;
    if(baseReg == curSP)
    {
        // stack push/pull, we can assume RAM
        // this is some annoying duplication...
        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));

        bool first = true;
        for(; regList && i < 16; regList >>= 1, i++)
        {
            if(!(regList & 1))
                continue;

            // directly index to force user
            auto reg = isLoadForce ? static_cast<Reg>(i): mapReg(static_cast<Reg>(i));

            if(isLoad)
            {
                if(reg == Reg::PC)
                    updateARMPC(*ptr++);
                else
                    loReg(reg) = *ptr++;
            }
            else
            {
                if(reg == Reg::PC)
                    *ptr++ = loReg(reg) + 4;
                else
                    *ptr++ = loReg(reg);
            }

            addr += 4;
            if(first && writeBack)
            {
                // write back after first store
                loReg(baseReg) = isUp ? highAddr : lowAddr;
            }
            first = false;
        }

        cycles = numRegs * mem.getAccessCycles(addr, 4, true); // it's RAM so N == S
        mem.iCycle(cycles); // not I cycles, but need to update prefetch...
    }
    else
    {
        // force alingment for everything but SRAM...
        if(addr < 0xE000000)
            addr &= ~3;

        bool first = true;
        for(; regList; regList >>= 1, i++)
        {
            if(!(regList & 1))
                continue;

            // directly index to force user
            auto reg = isLoadForce ? static_cast<Reg>(i) : mapReg(static_cast<Reg>(i));

            if(isLoad)
            {
                if(reg == Reg::PC)
                    updateARMPC(readMem32(addr, cycles, first));
                else
                    loReg(reg) = readMem32(addr, cycles, first);
            }
            else
            {
                if(reg == Reg::PC)
                    writeMem32(addr, loReg(reg) + 4, cycles);
                else
                    writeMem32(addr, loReg(reg), cycles);
            }

            addr += 4;
            if(first && writeBack)
            {
                // write back after first store
                loReg(baseReg) = isUp ? highAddr : lowAddr;
            }
            first = false;
        }
    }


    if(isLoad)
        return cycles + mem.iCycle() + mem.prefetchTiming32(pcSCycles, pcNCycles);
    else
        return cycles + mem.prefetchTiming32(pcNCycles);
}

int AGBCPU::doALUOp(int op, Reg destReg, uint32_t op1, uint32_t op2, bool carry)
{
    if(destReg == Reg::PC)
    {
        // don't attempt to restore in user/system mode as SPSR doesn't exist (a test ends up doing this...)
        if(regBankOffset)
            cpsr = getSPSR(); // restore

        int ret = doALUOpNoCond(op, destReg, op1, op2);

        modeChanged();

        return ret;
    }

    // output
    uint32_t res;

    auto doAdd = [this](uint32_t a, uint32_t b, int c = 0)
    {
        uint32_t res = a + b + c;
        bool carry = res < a || (res == a && c) /*for adc*/;
        bool overflow = !((a ^ b) & signBit) && ((a ^ res) & signBit);  // same sign and sign changed

        assert(carry == !!((static_cast<uint64_t>(a) + b + c) & (1ull << 32)));

        cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit ? Flag_N : 0) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow ? Flag_V : 0);
        return res;
    };

    auto doSub = [this](uint32_t a, uint32_t b, int c = 1)
    {
        uint32_t res = a - b + c - 1;
        bool carry = !(b > a || (b == a && !c) /*for sbc*/);
        bool overflow = ((a ^ b) & signBit) && ((a ^ res) & signBit);  // different sign and sign changed

        assert(carry == !((static_cast<uint64_t>(a) - b + c - 1) & (1ull << 32)));

        cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit ? Flag_N : 0) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow ? Flag_V : 0);
        return res;
    };

    switch(op)
    {
        case 0x0: // AND
            reg(destReg) = res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0x1: // EOR
            reg(destReg) = res = op1 ^ op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0x2: // SUB
            reg(destReg) = doSub(op1, op2);
            break;
        case 0x3: // RSB
            reg(destReg) = doSub(op2, op1);
            break;
        case 0x4: // ADD
            reg(destReg) = doAdd(op1, op2);
            break;
        case 0x5: // ADC
            reg(destReg) = doAdd(op1, op2, cpsr & Flag_C ? 1 : 0);
            break;
        case 0x6: // SBC
            reg(destReg) = doSub(op1, op2, cpsr & Flag_C ? 1 : 0);
            break;
        case 0x7: // RSC
            reg(destReg) = doSub(op2, op1, cpsr & Flag_C ? 1 : 0);
            break;
        case 0x8: // TST
            res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0x9: // TEQ
            res = op1 ^ op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0xA: // CMP
            doSub(op1, op2);
            break;
        case 0xB: // CMN
            doAdd(op1, op2);
            break;
        case 0xC: // ORR
            reg(destReg) = res = op1 | op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0xD: // MOV
            reg(destReg) = res = op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0xE: // BIC
            reg(destReg) = res = op1 & ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        case 0xF: // MVN
            reg(destReg) = res = ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0);
            break;
        default:
            __builtin_unreachable();
    }

    return mem.prefetchTiming32(pcSCycles);
}

int AGBCPU::doALUOpNoCond(int op, Reg destReg, uint32_t op1, uint32_t op2)
{
    uint32_t dest;

    switch(op)
    {
        case 0x0: // AND
            dest = op1 & op2;
            break;
        case 0x1: // EOR
            dest = op1 ^ op2;
            break;
        case 0x2: // SUB
            dest = op1 - op2;
            break;
        case 0x3: // RSB
            dest = op2 - op1;
            break;
        case 0x4: // ADD
            dest = op1 + op2;
            break;
        case 0x5: // ADC
            dest = op1 + op2 + (cpsr & Flag_C ? 1 : 0);
            break;
        case 0x6: // SBC
            dest = op1 - op2 + (cpsr & Flag_C ? 1 : 0) - 1;
            break;
        case 0x7: // RSC
            dest = op2 - op1 + (cpsr & Flag_C ? 1 : 0) - 1;
            break;
        // TST-CMN should not get here
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
            return pcSCycles;
        case 0xC: // ORR
            dest = op1 | op2;
            break;
        case 0xD: // MOV
            dest = op2;
            break;
        case 0xE: // BIC
            dest = op1 & ~op2;
            break;
        case 0xF: // MVN
            dest = ~op2;
            break;
        default:
            __builtin_unreachable();
    }

    if(destReg == Reg::PC)
    {
        // yes, this can happen
        if(cpsr & Flag_T)
            updateTHUMBPC(dest);
        else
            updateARMPC(dest);

        return pcNCycles + pcSCycles * 2;
    }
    else
        reg(destReg) = dest;

    return mem.prefetchTiming32(pcSCycles);
}

int AGBCPU::doTHUMB01MoveShifted(uint16_t opcode, uint32_t &pc)
{
    auto instOp = (opcode >> 11) & 0x1;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto offset = (opcode >> 6) & 0x1F;
    auto res = loReg(srcReg);

    uint32_t carry;
    switch(instOp)
    {
        case 0: // LSL
            if(offset != 0)
            {
                carry = res & (1 << (32 - offset)) ? Flag_C : 0;
                res <<= offset;
            }
            else
                carry = cpsr & Flag_C; // preserve
            break;
        case 1: // LSR
            if(!offset) offset = 32; // shift by 0 is really 32

            carry = res & (1 << (offset - 1)) ? Flag_C : 0;
            if(offset == 32)
                res = 0;
            else
                res >>= offset;
            break;
        default:
            assert(!"Invalid format 1 shift type");
    }

    loReg(dstReg) = res;

    cpsr = (cpsr & 0x1FFFFFFF)
         | (res & signBit ? Flag_N : 0)
         | (res == 0 ? Flag_Z : 0)
         | carry;

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB0102(uint16_t opcode, uint32_t &pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    if(instOp == 3) // format 2, add/sub
    {
        bool isImm = opcode & (1 << 10);
        bool isSub = opcode & (1 << 9);
        uint32_t op1 = loReg(srcReg), op2 = (opcode >> 6) & 7;

        uint32_t res;

        if(!isImm)
            op2 = loReg(static_cast<Reg>(op2));

        uint32_t carry, overflow;

        cpsr &= 0x0FFFFFFF;

        if(isSub)
        {
            res = op1 - op2;
            carry = !(res > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & (op1 ^ res)) & signBit;
        }
        else
        {
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = (~(op1 ^ op2) & (op1 ^ res)) & signBit;
        }

        loReg(dstReg) = res;

        cpsr |= (res & signBit ? Flag_N : 0)
             | (res == 0 ? Flag_Z : 0)
             | carry
             | (overflow >> 3);
    }
    else // format 1, move shifted register
    {
        auto offset = (opcode >> 6) & 0x1F;
        auto res = loReg(srcReg);

        assert(instOp == 2); // others are handled elsewhere

        uint32_t carry;

        if(!offset) offset = 32;

        auto sign = res & signBit;
        carry = res & (1 << (offset - 1)) ? Flag_C : 0;
        if(offset == 32)
            res = sign ? 0xFFFFFFFF : 0;
        else
            res = static_cast<int32_t>(res) >> offset;

        loReg(dstReg) = res;

        cpsr = (cpsr & 0x1FFFFFFF)
             | (res & signBit ? Flag_N : 0)
             | (res == 0 ? Flag_Z : 0)
             | carry;
    }

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB03(uint16_t opcode, uint32_t &pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t offset = opcode & 0xFF;

    auto dst = loReg(dstReg);

    uint32_t res;
    uint32_t carry, overflow;

    switch(instOp)
    {
        case 0: // MOV
            loReg(dstReg) = offset;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (offset == 0 ? Flag_Z : 0); // N not possible
            break;
        case 1: // CMP
            res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit; // offset cannot be negative, simplifies overflow checks
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3); // overflow is either 0 or 0x80000000, shift it down
            break;
        case 2: // ADD
            loReg(dstReg) = res = dst + offset;
            carry = res < dst ? Flag_C : 0;
            overflow = (~dst & res) & signBit;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 3: // SUB
            loReg(dstReg) = res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        default:
            __builtin_unreachable();
    }

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB040506(uint16_t opcode, uint32_t &pc)
{
    if(opcode & (1 << 11)) // format 6, PC-relative load
        return doTHUMB06PCRelLoad(opcode, pc);
    else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
        return doTHUMB05HiReg(opcode, pc);
    else // format 4, alu
        return doTHUMB04ALU(opcode, pc);
}

int AGBCPU::doTHUMB04ALU(uint16_t opcode, uint32_t &pc)
{
    auto instOp = (opcode >> 6) & 0xF;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto op1 = loReg(dstReg);
    auto op2 = loReg(srcReg);

    uint32_t res;
    uint32_t carry, overflow; // preserved if logical op

    switch(instOp)
    {
        case 0x0: // AND
            reg(dstReg) = res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x1: // EOR
            reg(dstReg) = res = op1 ^ op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x2: // LSL
            carry = cpsr & Flag_C;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & 1) : 0;
                carry = carry ? Flag_C : 0;
                reg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (32 - op2)) ? Flag_C : 0;
                reg(dstReg) = res = op1 << op2;
            }
            else
                reg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return mem.iCycle() + mem.prefetchTiming16(pcSCycles); // +1I for shift by register
        case 0x3: // LSR
            carry = cpsr & Flag_C;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & (1 << 31)) : 0;
                carry = carry ? Flag_C : 0;
                reg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                reg(dstReg) = res = op1 >> op2;
            }
            else
                reg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return mem.iCycle() + mem.prefetchTiming16(pcSCycles);
        case 0x4: // ASR
        {
            carry = cpsr & Flag_C;
            auto sign = op1 & signBit;
            if(op2 >= 32)
            {
                carry = sign ? Flag_C : 0;
                reg(dstReg) = res = sign ? 0xFFFFFFFF : 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                res = static_cast<int32_t>(op1) >> op2;

                reg(dstReg) = res;
            }
            else
                reg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return mem.iCycle() + mem.prefetchTiming16(pcSCycles);
        }
        case 0x5: // ADC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            reg(dstReg) = res = op1 + op2 + c;
            carry = res < op1 || (res == op1 && c) ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x6: // SBC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            reg(dstReg) = res = op1 - op2 + c - 1;
            carry = !(op2 > op1 || (op2 == op1 && !c)) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x7: // ROR
        {
            carry = cpsr & Flag_C;
            int shift = op2 & 0x1F;

            if(op2)
                carry = op1 & (1 << (shift - 1)) ? Flag_C : 0;

            reg(dstReg) = res = (op1 >> shift) | (op1 << (32 - shift));
            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return pcSCycles + 1;
        }
        case 0x8: // TST
            res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x9: // NEG
        {
            reg(dstReg) = res = 0 - op2;
            carry = !(op2 > 0) ? Flag_C : 0; //?
            overflow = (op2 & signBit) & (res & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0xA: // CMP
            res = op1 - op2;
            carry = !(op2 > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // different signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xB: // CMN
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // same signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xC: // ORR
            reg(dstReg) = res = op1 | op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0xD: // MUL
        {
            // carry is meaningless, v is unaffected
            reg(dstReg) = res = op1 * op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);

            // leading 0s or 1s
            int tmp = op1 & (1 << 31) ? ~op1 : op1;
            int prefix = tmp ? __builtin_clz(tmp) : 32;

            // more cycles the more bytes are non 0/ff
            int iCycles = prefix == 32 ? 1 : (4 - prefix / 8);
            return mem.iCycle(iCycles) + mem.prefetchTiming16(pcSCycles, pcNCycles);
        }
        case 0xE: // BIC
            reg(dstReg) = res = op1 & ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0xF: // MVN
            reg(dstReg) = res = ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
    }

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB05HiReg(uint16_t opcode, uint32_t &pc)
{
    auto op = (opcode >> 8) & 3;
    bool h1 = opcode & (1 << 7);
    bool h2 = opcode & (1 << 6);

    auto srcReg = static_cast<Reg>(((opcode >> 3) & 7) + (h2 ? 8 : 0));
    auto dstReg = static_cast<Reg>((opcode & 7) + (h1 ? 8 : 0));

    auto src = reg(srcReg);

    switch(op)
    {
        case 0: // ADD
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC((loReg(Reg::PC) + src) & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else
                reg(dstReg) += src;

            break;
        case 1: // CMP
        {
            auto dst = reg(dstReg);

            auto res = dst - src;
            bool carry = !(src > dst);

            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V))
                    | ((res & signBit) ? Flag_N : 0)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0)
                    | (((dst ^ src) & signBit) && ((dst ^ res) & signBit) ? Flag_V : 0);
            break;
        }
        case 2: // MOV
        {
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC(src & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else
                reg(dstReg) = src;

            break;
        }
        case 3: // BX
        {
            if(!(src & 1))
            {
                cpsr &= ~Flag_T;
                updateARMPC(src);
            }
            else
                updateTHUMBPC(src & ~1);

            return pcSCycles * 2 + pcNCycles;
        }

        default:
            assert(!"Invalid format 5 op!");
    }

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB06PCRelLoad(uint16_t opcode, uint32_t &pc)
{
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t word = opcode & 0xFF;

    // pc + 4, bit 1 forced to 0
    int cycles = 0;
    loReg(dstReg) = readMem32((pc & ~2) + (word << 2), cycles);

    return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
}

int AGBCPU::doTHUMB0708(uint16_t opcode, uint32_t &pc)
{
    auto offReg = static_cast<Reg>((opcode >> 6) & 7);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + loReg(offReg);

    if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
    {
        bool hFlag = opcode & (1 << 11);
        bool signEx = opcode & (1 << 10);

        if(signEx)
        {
            if(hFlag && !(addr & 1)) // LDRSH, (misaligned gets treated as a byte!)
            {
                int cycles = 0;
                auto val = readMem16(addr, cycles);
                if(val & 0x8000)
                    loReg(dstReg) = val | 0xFFFF0000;
                else
                    loReg(dstReg) = val;

                return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
            }
            else // LDRSB
            {
                int cycles = 0;
                auto val = readMem8(addr, cycles);
                if(val & 0x80)
                    loReg(dstReg) = val | 0xFFFFFF00;
                else
                    loReg(dstReg) = val;

                return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
            }
        }
        else
        {
            if(hFlag) // LDRH
            {
                int cycles = 0;
                loReg(dstReg) = readMem16(addr, cycles);
                return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
            }
            else // STRH
            {
                int cycles = 0;
                writeMem16(addr, loReg(dstReg), cycles);
                return cycles + mem.prefetchTiming16(pcNCycles);
            }
        }
    }
    else // format 7, load/store with reg offset
    {
        bool isLoad = opcode & (1 << 11);
        bool isByte = opcode & (1 << 10);

        if(isLoad)
        {
            int cycles = 0;
            if(isByte) // LDRB
                loReg(dstReg) = readMem8(addr, cycles);
            else // LDR
                loReg(dstReg) = readMem32(addr, cycles);

            return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
        }
        else
        {
            int cycles = 0;
            if(isByte) // STRB
                writeMem8(addr, loReg(dstReg), cycles);
            else // STR
                writeMem32(addr, loReg(dstReg), cycles);

            return cycles + mem.prefetchTiming16(pcNCycles);
        }
    }
}

int AGBCPU::doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t &pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + (offset << 2);
    if(isLoad) // LDR
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
    }
    else // STR
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + mem.prefetchTiming16(pcNCycles);
    }
}

int AGBCPU::doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t &pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRB
    {
        int cycles = 0;
        loReg(dstReg) = readMem8(addr, cycles);
        return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
    }
    else // STRB
    {
        int cycles = 0;
        writeMem8(addr, loReg(dstReg), cycles);
        return cycles + mem.prefetchTiming16(pcNCycles);
    }
}

int AGBCPU::doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t &pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F) << 1;
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRH
    {
        int cycles = 0;
        loReg(dstReg) = readMem16(addr, cycles);
        return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
    }
    else // STRH
    {
        int cycles = 0;
        writeMem16(addr, loReg(dstReg), cycles);
        return cycles + mem.prefetchTiming16(pcNCycles);
    }
}

int AGBCPU::doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t &pc)
{
    bool isLoad = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    auto addr = loReg(curSP) + word;

    if(isLoad)
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
    }
    else
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + mem.prefetchTiming16(pcNCycles);
    }
}

int AGBCPU::doTHUMB12LoadAddr(uint16_t opcode, uint32_t &pc)
{
    bool isSP = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    if(isSP)
        loReg(dstReg) = loReg(curSP) + word;
    else
        loReg(dstReg) = (pc & ~2) + word; // + 4, bit 1 forced to 0

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB1314(uint16_t opcode, uint32_t &pc)
{
    if(opcode & (1 << 10)) // format 14, push/pop
        return doTHUMB14PushPop(opcode, pc);
    else // format 13, add offset to SP
        return doTHUMB13SPOffset(opcode, pc);
}

int AGBCPU::doTHUMB13SPOffset(uint16_t opcode, uint32_t &pc)
{
    bool isNeg = opcode & (1 << 7);
    int off = (opcode & 0x7F) << 2;

    if(isNeg)
        loReg(curSP) -= off;
    else
        loReg(curSP) += off;

    return mem.prefetchTiming16(pcSCycles);
}

int AGBCPU::doTHUMB14PushPop(uint16_t opcode, uint32_t &pc)
{
    // timings here are very off

    bool isLoad = opcode & (1 << 11);
    bool pclr = opcode & (1 << 8); // store LR/load PC
    uint8_t regList = opcode & 0xFF;

    if(isLoad) // POP
    {
        auto addr = loReg(curSP);
        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
            {
                regs[i] = *ptr++;
                addr += 4;
            }
        }

        if(pclr)
        {
            updateTHUMBPC(*ptr++ & ~1); /*ignore thumb bit*/
            addr += 4;
        }

        loReg(curSP) = addr;
    }
    else // PUSH
    {
        auto addr = loReg(curSP) - (pclr ? 4 : 0);

        // offset
        for(uint8_t t = regList; t; t >>= 1)
        {
            if(t & 1)
                addr -= 4;
        }
        loReg(curSP) = addr;

        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
                *ptr++ = regs[i];
        }

        if(pclr)
            *ptr++ = loReg(curLR);
    }

    return pcNCycles;
}

int AGBCPU::doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t &pc)
{
    bool isLoad = opcode & (1 << 11);
    auto baseReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t regList = opcode & 0xFF;

    auto addr = loReg(baseReg);

    int cycles = 0;

    if(!regList)
    {
        // empty list loads/stores PC... even though it isn't usually possible here
        if(isLoad)
            updateTHUMBPC(readMem32(addr & ~3, cycles));
        else
            writeMem32(addr & ~3, pc + 2, cycles);

        reg(baseReg) = addr + 0x40;

        return cycles;
    }

    auto endAddr = addr;
    for(uint8_t t = regList; t; t >>=1)
    {
        if(t & 1)
            endAddr += 4;
    }

    // force alingment for everything but SRAM...
    if(addr < 0xE000000)
        addr &= ~3;

    int i = 0;
    bool first = true, seq = false;

    // prevent overriding base for loads
    // "A LDM will always overwrite the updated base if the base is in the list."
    if(isLoad && (regList & (1 << static_cast<int>(baseReg))))
        first = false;

    for(; regList; regList >>= 1, i++)
    {
        if(!(regList & 1))
            continue;

        if(isLoad)
            regs[i] = readMem32(addr, cycles, seq);
        else
            writeMem32(addr, regs[i], cycles, seq);

        // base write-back is on the second cycle of the instruction
        // which is when the first reg is written
        if(first)
            reg(baseReg) = endAddr;

        first = false;
        seq = true;

        addr += 4;
    }

    if(isLoad)
        cycles += mem.iCycle() + mem.prefetchTiming16(pcSCycles, pcNCycles);
    else
        cycles += mem.prefetchTiming16(pcNCycles);

    return cycles;
}

int AGBCPU::doTHUMB1617(uint16_t opcode, uint32_t &pc)
{
    auto cond = (opcode >> 8) & 0xF;
    if(cond == 0xF) // format 17, SWI
    {
        auto ret = (pc - 2) & ~1;
        spsr[1/*svc*/] = cpsr;

        cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x13; //supervisor mode
        modeChanged();
        updateARMPC(8);
        loReg(curLR) = ret;
    }
    else // format 16, conditional branch
    {
        int offset = static_cast<int8_t>(opcode & 0xFF);
        bool condVal = false;
        switch(cond)
        {
            case 0x0: // BEQ
                condVal = cpsr & Flag_Z;
                break;
            case 0x1: // BNE
                condVal = !(cpsr & Flag_Z);
                break;
            case 0x2: // BCS
                condVal = cpsr & Flag_C;
                break;
            case 0x3: // BCC
                condVal = !(cpsr & Flag_C);
                break;
            case 0x4: // BMI
                condVal = cpsr & Flag_N;
                break;
            case 0x5: // BPL
                condVal = !(cpsr & Flag_N);
                break;
            case 0x6: // BVS
                condVal = cpsr & Flag_V;
                break;
            case 0x7: // BVC
                condVal = !(cpsr & Flag_V);
                break;
            case 0x8: // BHI
                condVal = (cpsr & Flag_C) && !(cpsr & Flag_Z);
                break;
            case 0x9: // BLS
                condVal = !(cpsr & Flag_C) || (cpsr & Flag_Z);
                break;
            case 0xA: // BGE
                condVal = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                break;
            case 0xB: // BLT
                condVal = !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
                break;
            case 0xC: // BGT
                condVal = !(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
                break;
            case 0xD: // BLE
                condVal = (cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
                break;
            // E undefined
            // F is SWI

            default:
                assert(!"Invalid THUMB cond");
        }

        if(condVal)
            updateTHUMBPC(pc + offset * 2);
        else
            return pcSCycles; // no extra cycles if branch not taken
    }

    return pcSCycles * 2 + pcNCycles; // 2S + 1N, probably a bit wrong for SWI
}

int AGBCPU::doTHUMB18UncondBranch(uint16_t opcode, uint32_t &pc)
{
    uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2

    updateTHUMBPC(pc + offset);

    return pcSCycles * 2 + pcNCycles; // 2S + 1N
}

int AGBCPU::doTHUMB19LongBranchLink(uint16_t opcode, uint32_t &pc)
{
    bool high = opcode & (1 << 11);
    uint32_t offset = opcode & 0x7FF;

    if(!high) // first half
    {
        offset <<= 12;
        if(offset & (1 << 22))
            offset |= 0xFF800000; //sign extend
        loReg(curLR) = pc + offset;

        return pcSCycles;
    }
    else // second half
    {
        auto newPC = loReg(curLR) + (offset << 1);
        loReg(curLR) = (pc - 2) | 1; // magic switch to thumb bit...

        updateTHUMBPC(newPC);

        return pcNCycles + pcSCycles * 2;
    }
}

void AGBCPU::updateARMPC(uint32_t pc)
{
    assert(!(pc & 3));
    assert(pc < 0xE000000); // trying to execute save data would be bad

    thumbPCPtr = nullptr;

    if(armPCPtr && pc >> 24 == loReg(Reg::PC) >> 24)
    {
        // memory region didn't change, skip recaclculating ptr/cycles
        armPCPtr += static_cast<int32_t>(pc - loReg(Reg::PC)) / 4;
        assert(*armPCPtr == mem.read32(pc));
    }
    else
    {
        armPCPtr = reinterpret_cast<const uint32_t *>(std::as_const(mem).mapAddress(pc));
        pcSCycles = mem.getAccessCycles(pc, 4, true);
        pcNCycles = mem.getAccessCycles(pc, 4, false);
    }

    mem.updatePC(pc);

    // refill the pipeline
    decodeOp = *armPCPtr++;
    fetchOp = *armPCPtr;

    loReg(Reg::PC) = pc + 4; // pointing at last fetch
}

void AGBCPU::updateTHUMBPC(uint32_t pc)
{
    // called when PC is updated in THUMB mode (except for incrementing)
    assert(!(pc & 1));
    assert(pc < 0xE000000);

    armPCPtr = nullptr;

    if(thumbPCPtr && pc >> 24 == loReg(Reg::PC) >> 24)
    {
        // memory region didn't change, skip recaclculating ptr/cycles
        thumbPCPtr += static_cast<int32_t>(pc - loReg(Reg::PC)) / 2;
        assert(*thumbPCPtr == mem.read16(pc));
    }
    else
    {
        thumbPCPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc)); // force const mapAddress
        pcSCycles = mem.getAccessCycles(pc, 2, true);
        pcNCycles = mem.getAccessCycles(pc, 2, false);
    }

    mem.updatePC(pc);

    // refill the pipeline
    decodeOp = *thumbPCPtr++;
    fetchOp = *thumbPCPtr;

    loReg(Reg::PC) = pc + 2; // pointing at last fetch
}

bool AGBCPU::serviceInterrupts()
{
    if((cpsr & Flag_I))
        return false;

    halted = false;

    auto ret = loReg(Reg::PC);
    if(cpsr & Flag_T)
        ret += 2;

    spsr[3/*irq*/] = cpsr;

    cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x12; // irq mode
    modeChanged();
    updateARMPC(0x18);
    loReg(curLR) = ret;
    return true;
}

int AGBCPU::dmaTransfer(int channel)
{
    int regOffset = channel * 12;

    auto &dmaControl = mem.getIOReg(IO_DMA0CNT_H + regOffset);
    auto srcAddr = dmaSrc[channel];
    auto dstAddr = dmaCurDst[channel];
    int count = dmaCurCount[channel];

    bool is32Bit = dmaControl & DMACNTH_32Bit;
    int dstMode = (dmaControl & DMACNTH_DestMode) >> 5;
    int srcMode = (dmaControl & DMACNTH_SrcMode) >> 7;

    bool isValidSrc = srcAddr >= 0x2000000;

    bool started = dmaActive & (1 << channel);
    dmaActive |= 1 << channel;

    // sound DMA copies 4 to fixed dest
    if((channel == 1 || channel == 2) && (dmaControl & DMACNTH_Start) == 3 << 12)
    {
        if(!started)
            count = 4;
        dstMode = 2;
    }
    // reading from ROM always increments src
    else if(srcAddr > 0x8000000 && srcAddr < 0xE000000)
        srcMode = 0;

    int width = is32Bit ? 4 : 2;

    int cycles = 0;

    if(!started)
    {
        cycles = 2;

        // less cycles if both addresses are in the gamepak areas
        // gbatek says the opposite, but this makes more tests pass...
        // TODO: that probably doesn't include SRAM?
        if(srcAddr >= 0x8000000 && dstAddr >= 0x8000000)
            cycles -= 2;
    }

    srcAddr &= ~(width - 1);
    dstAddr &= ~(width - 1);

    uint32_t lastVal = dmaLastVal;

    while(count--)
    {
        if(is32Bit)
        {
            if(isValidSrc) // no read if source address is invalid
                lastVal = readMem32Aligned(srcAddr, cycles, started);

            writeMem32(dstAddr, lastVal, cycles, started);
        }
        else
        {
            if(isValidSrc)
                lastVal = readMem16Aligned(srcAddr, cycles, started);

            writeMem16(dstAddr, lastVal, cycles, started);
        }

        if(dstMode == 0 || dstMode == 3)
            dstAddr += width;
        else if(dstMode == 1)
            dstAddr -= width;

        started = true;

        // bad src, don't inc/dec
        if(!isValidSrc)
            continue;

        if(srcMode == 0)
            srcAddr += width;
        else if(srcMode == 1)
            srcAddr -= width;
    }

    // save last transferred value
    dmaLastVal = lastVal;
    if(isValidSrc && width == 2)
        dmaLastVal |= lastVal << 16;

    dmaSrc[channel] = srcAddr;

    // we're not done
    if(count > 0)
    {
        dmaCurCount[channel] = count;
        dmaCurDst[channel] = dstAddr;
        return cycles;
    }

    // done, maybe repeat
    if(!(dmaControl & DAMCNTH_Repeat))
    {
        dmaControl &= ~DMACNTH_Enable;
        dmaCount[channel] = dmaCurCount[channel] = 0;
    }

    // store unless we're reloading
    if(dstMode != 3)
        dmaDst[channel] = dmaCurDst[channel] = dstAddr;

    // flag interrupt
    if(dmaControl & DMACNTH_IRQEnable)
        flagInterrupt(Int_DMA0 << channel);

    dmaTriggered &= ~(1 << channel);
    dmaActive &= ~(1 << channel);

    return cycles;
}

void AGBCPU::updateTimers()
{
    // no timers, skip ahead
    if(!timerEnabled)
    {
        lastTimerUpdate = cycleCount;
        return;
    }

    auto passed = cycleCount - lastTimerUpdate;

    auto timer = lastTimerUpdate;
    lastTimerUpdate = cycleCount;

    while(passed)
    {
        auto step = std::min(nextTimerUpdate - timer, passed);
        passed -= step;

        assert(step > 0);

        uint8_t overflow = 0;
        auto enabled = timerEnabled;
        for(int i = 0; enabled; i++, enabled >>= 1)
        {
            if(!(enabled & 1))
                continue;

            // count-up
            if(timerPrescalers[i] == -1)
            {
                if(overflow & (1 << (i - 1)))
                    timerCounters[i]++;
            }
            else if(timerPrescalers[i] == 1)
                timerCounters[i] += step;
            else
            {
                int inc = ((timer & (timerPrescalers[i] - 1)) + step) / timerPrescalers[i];
                if(!inc)
                    continue;
                
                timerCounters[i] += inc;
            }

            // overflow
            if(timerCounters[i] == 0)
            {
                overflow |= (1 << i);
                timerCounters[i] = mem.readIOReg(IO_TM0CNT_L + i * 4);
                if(timerInterruptEnabled & (1 << i))
                    flagInterrupt(Int_Timer0 << i);

                if(i < 2)
                    apu.timerOverflow(i, timer);

                // overflow was where we expected
                assert(timer + step == nextTimerUpdate);
            }
        }

        // if we clamped the update there should've been an overflow
        assert(overflow || !passed);

        timer += step;
        
        // an overflow happend, recalculate next
        if(overflow)
            calculateNextTimerOverflow(timer);
    }
}

void AGBCPU::calculateNextTimerOverflow(uint32_t cycleCount)
{
    uint32_t nextOverflow = ~0;

    auto enabled = timerEnabled;
    for(int i = 0; enabled; i++, enabled >>= 1)
    {
        if(!(enabled & 1))
            continue;

        // count-up timer is updated when the previous timer overflows
        if(timerPrescalers[i] == -1)
            continue;

        // increments to overflow
        int incs = 0xFFFF - timerCounters[i];

        auto thisTimerOverflow = incs * timerPrescalers[i]
                               + timerPrescalers[i] - (cycleCount & (timerPrescalers[i] - 1));

        if(thisTimerOverflow < nextOverflow)
            nextOverflow = thisTimerOverflow;
    }

    nextTimerUpdate = cycleCount + nextOverflow;
}