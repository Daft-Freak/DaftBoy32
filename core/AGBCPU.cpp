#include <cassert>
#include <cstdio>
#include <cstdlib> //exit
#include <cstring>

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

AGBCPU::AGBCPU(AGBMemory &mem) : mem(mem)
{}

void AGBCPU::reset()
{
    cpsr = Flag_I | Flag_F | 0x13 /*supervisor mode*/;
    loReg(Reg::PC) = 0;

    timer = 0;
    for(auto &c : timerCounters)
        c = 0;
    for(auto &p : timerPrescalers)
        p = 0;

    mem.reset();
}

void AGBCPU::run(int ms)
{
    int cycles = (clockSpeed * ms) / 1000;

    while(cycles > 0)
    {
        int exec = 0;

        // DMA
        for(int chan = 0; chan < 4; chan++)
        {
            auto control = mem.readIOReg(IO_DMA0CNT_H + chan * 12);
            if(!(control & DMACNTH_Enable))
                continue;

            if((control & DMACNTH_Start) == 0 || // immediate
               ((control & DMACNTH_Start) == 1 << 12 && (dmaTriggers & Trig_VBlank)) ||
               ((control & DMACNTH_Start) == 2 << 12 && (dmaTriggers & Trig_HBlank)))
            {
                exec += dmaTransfer(chan);
            }
        }

        dmaTriggers = 0;

        if(!exec)
        {
            // CPU
            exec = (cpsr & Flag_T) ? executeTHUMBInstruction() : executeARMInstruction();

            serviceInterrupts(); // cycles?
        }

        updateTimers(exec);

        cycles -= exec;

        if(cycleCallback)
            cycleCallback(exec);
    }
}

void AGBCPU::setCycleCallback(CycleCallback cycleCallback)
{
    this->cycleCallback = cycleCallback;
}

void AGBCPU::flagInterrupt(int interrupt)
{
    mem.writeIOReg(IO_IF, mem.readIOReg(IO_IF) | interrupt);
}

void AGBCPU::triggerDMA(int trigger)
{
    dmaTriggers |= trigger;
}

uint8_t AGBCPU::readMem8(uint32_t addr) const
{
    return mem.read8(addr);
}

uint32_t AGBCPU::readMem16(uint32_t addr) const
{
    // this returns the 32-bit result of an unaligned 16-bit read
    auto aligned = addr & ~1;
    uint16_t val = readMem16Aligned(aligned);

    if(!(addr & 1))
        return val;

    return (val >> 8) | (val << 24);
}

uint16_t AGBCPU::readMem16Aligned(uint32_t addr) const
{
    assert((addr & 1) == 0);

    if((addr >> 24) == 0x4)
    {
        switch(addr & 0xFFFFFF)
        {
            case IO_TM0CNT_L:
                return timerCounters[0];
            case IO_TM1CNT_L:
                return timerCounters[1];
            case IO_TM2CNT_L:
                return timerCounters[2];
            case IO_TM3CNT_L:
                return timerCounters[3];
        }
    }

    return mem.read16(addr);
}

uint32_t AGBCPU::readMem32(uint32_t addr) const
{
    auto aligned = addr & ~3;
    uint32_t val = readMem32Aligned(aligned);

    if(!(addr & 3))
        return val;

    int shift = (addr & 3) << 3;
    return (val >> shift) | (val << (32 - shift));
}

uint32_t AGBCPU::readMem32Aligned(uint32_t addr) const
{
    assert((addr & 3) == 0);
    return readMem16Aligned(addr) | (readMem16Aligned(addr + 2) << 16);
}

void AGBCPU::writeMem8(uint32_t addr, uint8_t data)
{
    if((addr >> 24) == 0x4)
    {
        // need to modify these internally, promote to 16-bit
        if(addr == 0x4000202/*IF*/ || addr == 0x4000203 || (addr >= 0x4000100 && addr <= 0x400010E) /*timers*/)
        {
            auto tmp = readMem8(addr ^ 1);
            writeMem16(addr & ~1, addr & 1 ? tmp | (data << 8) : (tmp << 8) | data);
            return;
        }
    }

    mem.write8(addr, data);
}

void AGBCPU::writeMem16(uint32_t addr, uint16_t data)
{
    addr &= ~1;

    if((addr >> 24) == 0x4)
    {
        switch(addr & 0xFFFFFF)
        {
            case IO_TM0CNT_H:
            case IO_TM1CNT_H:
            case IO_TM2CNT_H:
            case IO_TM3CNT_H:
            {
                int index = ((addr & 0xFFFFFF) - IO_TM0CNT_H) >> 2;
                static const int prescalers[]{1, 64, 256, 1024};

                if(data & TMCNTH_Enable)
                {
                    if(!(mem.readIOReg(IO_TM0CNT_H + index * 4) & TMCNTH_Enable))
                        timerCounters[index] = mem.readIOReg(IO_TM0CNT_L + index * 4); // reload counter

                    if(data & TMCNTH_CountUp)
                        timerPrescalers[index] = -1; // magic value for count-up mode
                    else
                        timerPrescalers[index] = prescalers[data & TMCNTH_Prescaler];
                }
                else
                    timerPrescalers[index] = 0;

                break;
            }

            case IO_IF: // writing IF bits clears them internally
                data = mem.getIOReg(IO_IF) & ~data;
                break;
        }
    }

    mem.write16(addr, data);
}

void AGBCPU::writeMem32(uint32_t addr, uint32_t data)
{
    addr &= ~3;

    writeMem16(addr, data);
    writeMem16(addr + 2, data >> 16);
}

// returns cycle count
int AGBCPU::executeARMInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    auto opcode = readMem32Aligned(pc);
    auto timing = mem.getAccessCycles(pc, 4, true);

    pc += 4;

    // helpers
    auto getShiftedReg = [this, opcode, pc](Reg r, uint8_t shift, bool &carry)
    {
        auto ret = reg(r);

        int shiftType = (shift >> 1) & 3;
        int shiftAmount;
        if(shift & 1)
        {
            assert((shift & (1 << 3)) == 0);
            shiftAmount = reg(static_cast<Reg>(shift >> 4)) & 0xFF;
        }
        else
        {
            shiftAmount = shift >> 3;
            if(shiftType && shiftAmount == 0) // lsr/asr shift by 32 instead of 0 
                shiftAmount = 32;
        }

        // prefetch
        if(r == Reg::PC)
            ret += (shift & 1) ? 8 : 4;

        if(shiftAmount == 0) // do nothing, preserve carry if 0
        {
            carry = cpsr & Flag_C;
            return ret;
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
                    ret >>= shiftAmount;

                    if(sign)
                        ret |= ~(0xFFFFFFFF >> shiftAmount);
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
                                            
                    carry = ret & (1 << (shiftAmount - 1));
                    ret = (ret >> shiftAmount) | (ret << (32 - shiftAmount));
                }
                break;

            default:
                assert(!"Invalid shift type!");
        }
        
        return ret;
    };

    // condition
    auto cond = opcode >> 28;

    // the conditions here have to be backwards...
    switch(cond)
    {
        case 0x0: // equal
            if(!(cpsr & Flag_Z))
                return timing;
            break;
        case 0x1: // not equal
            if(cpsr & Flag_Z)
                return timing;
            break;
        case 0x2: // carry set
            if(!(cpsr & Flag_C))
                return timing;
            break;
        case 0x3: // carry clear
            if(cpsr & Flag_C)
                return timing;
            break;
        case 0x4: // negative
            if(!(cpsr & Flag_N))
                return timing;
            break;
        case 0x5: // positive or zero
            if((cpsr & Flag_N))
                return timing;
            break;
        case 0x6: // overflow
            if(!(cpsr & Flag_V))
                return timing;
            break;
        case 0x7: // no overflow
            if((cpsr & Flag_V))
                return timing;
            break;
        case 0x8: // unsigned higher
            if(!(cpsr & Flag_C) || (cpsr & Flag_Z))
                return timing;
            break;
        case 0x9: // unsigned lower or same
            if((cpsr & Flag_C) && !(cpsr & Flag_Z))
                return timing;
            break;
        case 0xA: // greater or equal
            if(!!(cpsr & Flag_N) != !!(cpsr & Flag_V))
                return timing;
            break;
        case 0xB: // less than
            if(!!(cpsr & Flag_N) == !!(cpsr & Flag_V))
                return timing;
            break;
        case 0xC: // greater than
            if((cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V))
                return timing;
            break;
        case 0xD: // less than or equal
            if(!(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V))
                return timing;
            break;
        case 0xE: // always
            break;
        // F is reserved
        default:
            assert(!"Invalid condition");
    }

    switch((opcode >> 24) & 0xF)
    {
        case 0x0: // data processing with register
        case 0x1: // and more...
        {
            if((opcode & 0x0FFFFF00) == 0x012FFF00) // Branch and Exchange (BX)
            {
                auto instOp = (opcode >> 4) & 0xF;
                assert(instOp == 1); //
                auto newPC = reg(static_cast<Reg>(opcode & 0xF));
                if(newPC & 1)
                    cpsr |= Flag_T;

                pc = newPC & 0xFFFFFFFE;
                return timing; // TODO: timing
            }

            if(((opcode >> 4) & 9) == 9)
            {
                if((opcode >> 5) & 3) // halfword transfer 
                {
                    bool isPre = opcode & (1 << 24);
                    bool isUp = opcode & (1 << 23);
                    bool isImm = opcode & (1 << 22);
                    bool writeBack = opcode & (1 << 21);
                    bool isLoad = opcode & (1 << 20);
                    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
                    auto srcDestReg = static_cast<Reg>((opcode >> 12) & 0xF);
                    bool sign = opcode & (1 << 6);
                    bool halfWords = opcode & (1 << 5);
                    auto offReg = static_cast<Reg>(opcode & 0xF);

                    int offset;
                    
                    if(isImm)
                        offset = ((opcode >> 4) & 0xF0) | (opcode & 0xF);
                    else
                    {
                        offset = reg(offReg);
                        assert((opcode & 0xF00) == 0);
                    }

                    if(!isUp)
                        offset = -offset;

                    auto addr = reg(baseReg);

                    // pc offset
                    if(baseReg == Reg::PC)
                        addr += 4;

                    if(isPre)
                        addr += offset;
                    else
                        assert(!writeBack); // bad

                    if(writeBack || !isPre)
                        reg(baseReg) += offset;

                    if(isLoad)
                    {
                        if(halfWords && (!sign || !(addr & 1))) // check alignment for signed load...
                        {
                            auto val = readMem16(addr); // LDRH/LDRSH

                            if(sign && (val & (1 << 15)))
                                reg(srcDestReg) = val | 0xFFFF0000; // sign extend
                            else
                                reg(srcDestReg) = val;
                        }
                        else // LDRSB ... or misaligned LDRSH
                        {
                            auto val = readMem8(addr);
                            if(val & (1 << 7))
                                reg(srcDestReg) = val | 0xFFFFFF00;
                            else
                                reg(srcDestReg) = val;
                        }
                    }
                    else
                    {
                        assert(halfWords);
                        assert(!sign);

                        // FIXME: PC is + 12
                        assert(srcDestReg != Reg::PC);
                        writeMem16(addr, reg(srcDestReg)); // STRH
                    }
                }
                else if(opcode & (1 << 24))
                {
                    bool isByte = opcode & (1 << 22);
                    auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
                    auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
                    auto srcReg = static_cast<Reg>(opcode & 0xF);

                    auto addr = reg(baseReg);

                    if(isByte)
                    {
                        auto v = readMem8(addr);
                        writeMem8(addr, reg(srcReg));
                        reg(destReg) = v;
                    }
                    else
                    {
                        auto v = readMem32(addr);
                        writeMem32(addr, reg(srcReg));
                        reg(destReg) = v;
                    }
                }
                else if(opcode & (1 << 23)) // MULL/MLAL
                {
                    bool isSigned = opcode & (1 << 22);
                    bool accumulate = opcode & (1 << 21);
                    bool setCondCode = opcode & (1 << 20);
                    auto destHiReg = static_cast<Reg>((opcode >> 16) & 0xF);
                    auto destLoReg = static_cast<Reg>((opcode >> 12) & 0xF);
                    auto op2Reg = static_cast<Reg>((opcode >> 8) & 0xF);
                    auto op1Reg = static_cast<Reg>(opcode & 0xF);

                    uint64_t res;

                    if(isSigned) // SMULL
                        res = static_cast<uint64_t>(static_cast<int64_t>(static_cast<int32_t>(reg(op1Reg))) * static_cast<int32_t>(reg(op2Reg)));
                    else // UMULL
                        res = static_cast<uint64_t>(reg(op1Reg)) * reg(op2Reg);
                    
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
                }
                else // MUL/MLA
                {
                    bool accumulate = opcode & (1 << 21);
                    bool setCondCode = opcode & (1 << 20);
                    auto destReg = static_cast<Reg>((opcode >> 16) & 0xF);
                    auto op3Reg = static_cast<Reg>((opcode >> 12) & 0xF);
                    auto op2Reg = static_cast<Reg>((opcode >> 8) & 0xF);
                    auto op1Reg = static_cast<Reg>(opcode & 0xF);

                    uint32_t res = reg(op1Reg) * reg(op2Reg);

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
                }

                return timing; // TODO: timing
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
                        cpsr = (cpsr & ~mask) | (val & mask);
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
            }
            else
            {
                auto op1Reg = static_cast<Reg>((opcode >> 16) & 0xF);
                auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
                auto op2Shift = (opcode >> 4) & 0xFF;
                auto op2Reg = static_cast<Reg>(opcode & 0xF);
                auto op1 = reg(op1Reg);

                // reg arg2
                bool carry;
                auto op2 = getShiftedReg(op2Reg, op2Shift, carry);

                if(op1Reg == Reg::PC)
                    op1 += (op2Shift & 1) ? 8 : 4;

                doALUOp(instOp, destReg, op1, op2, setCondCode, carry);
                return timing + (op2Shift & 1); // +1I if shift by reg
            }
            
            break;
        }
        case 0x2: // data processing with immediate
        case 0x3:
        {
            auto instOp = (opcode >> 21) & 0xF;
            bool setCondCode = (opcode & (1 << 20));
            auto op1Reg = static_cast<Reg>((opcode >> 16) & 0xF);
            auto op1 = reg(op1Reg);
            auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);

            if(op1Reg == Reg::PC)
                op1 += 4;

            // get the immediate value
            uint32_t op2 = opcode & 0xFF;
            int shift = ((opcode >> 8) & 0xF) * 2;

            bool carry = shift ? op2 & (1 << (shift - 1)) : cpsr & Flag_C;
            op2 = (op2 >> shift) | (op2 << (32 - shift));

            // TODO: a bit duplicated with above
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
                    cpsr = (cpsr & ~mask) | (val & mask);
            }

            doALUOp(instOp, destReg, op1, op2, setCondCode, carry);
            return timing;
        }
        case 0x4: // Single Data Transfer (I = 0, P = 0)
        case 0x5: // Single Data Transfer (I = 0, P = 1)
        case 0x6: // Single Data Transfer (I = 1, P = 0)
        case 0x7: // Single Data Transfer (I = 1, P = 1)
        {
            bool isReg = opcode & (1 << 25);
            bool isPre = opcode & (1 << 24);
            bool isUp = opcode & (1 << 23);
            bool isByte = opcode & (1 << 22);
            bool writeBack = opcode & (1 << 21);
            bool isLoad = opcode & (1 << 20);
            auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
            auto srcDestReg = static_cast<Reg>((opcode >> 12) & 0xF);
            int offset;
            
            if(!isReg) // immediate
                offset = opcode & 0xFFF;
            else
            {
                assert((opcode & (1 << 4)) == 0); // no reg shift
                bool carry;
                offset = getShiftedReg(static_cast<Reg>(opcode & 0xF), (opcode >> 4) & 0xFF, carry);
            }

            if(!isUp)
                offset = -offset;

            auto addr = reg(baseReg);

            // pc offset
            if(baseReg == Reg::PC)
                addr += 4;

            if(isPre)
                addr += offset;
            else
                assert(!writeBack); // non-privileged transfer

            if(writeBack || !isPre) // post inc always writes back
                reg(baseReg) += offset;

            if(isLoad)
            {
                if(isByte)
                    reg(srcDestReg) = readMem8(addr);
                else
                    reg(srcDestReg) = readMem32(addr);

                return timing + mem.getAccessCycles(addr, isByte ? 1 : 4, false) + 1; // 1S + 1N + 1I
                // TODO: +1S+1N if dst == PC
            }
            else
            {
                // FIXME: PC is + 12
                assert(srcDestReg != Reg::PC);
                if(isByte)
                    writeMem8(addr, reg(srcDestReg));
                else
                    writeMem32(addr, reg(srcDestReg));

                return mem.getAccessCycles(pc, 4, false) + mem.getAccessCycles(addr, isByte ? 1 : 4, false); // 2N
            }

            break;
        }
        case 0x8: // Block Data Transfer (P = 0)
        case 0x9: // Block Data Transfer (P = 1)
        {
            bool preIndex = opcode & (1 << 24);
            bool isUp = opcode & (1 << 23);
            bool isLoadForce = opcode & (1 << 22);
            bool writeBack = opcode & (1 << 21);
            bool isLoad = opcode & (1 << 20);
            auto baseReg = static_cast<Reg>((opcode >> 16) & 0xF);
            uint16_t regList = opcode;

            if(isLoadForce)
            {
                //assert(!writeBack); // "should not be used"
                assert(!isLoad || !(regList & (1 << 15))); // TODO: load with r15 (mode change)
            }

            auto addr = reg(baseReg) & ~3;

            // flip decrement addressing around so that regs are stored in the right order
            if(!isUp)
            {
                preIndex = !preIndex;
                for(uint16_t t = regList; t; t >>= 1)
                {
                    if(t & 1)
                        addr -= 4;
                }
            }

            uint32_t lowAddr = addr;

            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                if(preIndex)
                    addr += 4;

                if(isLoad && static_cast<Reg>(i) == baseReg)
                    writeBack = false; // don't override

                // directly index to force user
                auto &regRef = isLoadForce ? regs[i] : reg(static_cast<Reg>(i));

                if(isLoad)
                    regRef = readMem32(addr);
                else
                    writeMem32(addr, regRef);

                if(!preIndex)
                    addr += 4;
            }

            if(writeBack)
                reg(baseReg) = isUp ? addr : lowAddr;

            break;
        }
        case 0xA: // Branch (B)
        {
            auto offset = (opcode & 0xFFFFFF) << 2;
            if(offset & 0x2000000)
                offset |= 0xFC000000; // sign extend

            pc += static_cast<int32_t>(offset) + 4/*prefetch*/;
            break;
        }
        case 0xB: // Branch with Link (BL)
        {
            auto offset = (opcode & 0xFFFFFF) << 2;
            if(offset & 0x2000000)
                offset |= 0xFC000000; // sign extend

            reg(Reg::LR) = pc;
            pc += static_cast<int32_t>(offset) + 4/*prefetch*/;
            break;
        }

        case 0xF: // SWI
        {
            auto ret = pc;
            spsr[1/*svc*/] = cpsr;

            pc = 8;
            cpsr = (cpsr & ~0x1F) | Flag_I | 0x13; //supervisor mode
            reg(Reg::LR) = ret;
            break;
        }

        default:
            printf("ARM op %x @%x\n", opcode & 0xFFFFFFF, pc - 4);
            exit(0);
            break;
    }

    return timing; // TODO: timings
}

int AGBCPU::executeTHUMBInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    auto opcode = readMem16Aligned(pc & ~1);

    pc += 2;

    switch(opcode >> 12)
    {
        case 0x0: // format 1
        case 0x1: // formats 1-2
        {
            auto instOp = opcode >> 11;
            auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
            auto dstReg = static_cast<Reg>(opcode & 7);

            if(instOp == 3) // format 2, add/sub
            {
                bool isImm = opcode & (1 << 10);
                bool isSub = opcode & (1 << 9);
                uint32_t op1 = loReg(srcReg), op2;

                uint32_t res;

                if(isImm)
                    op2 = (opcode >> 6) & 7;
                else
                    op2 = loReg(static_cast<Reg>((opcode >> 6) & 7));

                bool carry, overflow;

                if(isSub)
                {
                    res = op1 - op2;
                    carry = !(op2 > op1);
                    overflow = ((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit);
                }
                else
                {
                    res = op1 + op2;
                    carry = res < op1;
                    overflow = !((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit);
                }

                loReg(dstReg) = res;

                cpsr = (cpsr & 0x0FFFFFFF)
                     | (res & signBit ? Flag_N : 0)
                     | (res == 0 ? Flag_Z : 0)
                     | (carry ? Flag_C : 0)
                     | (overflow ? Flag_V : 0);
            }
            else // format 1, move shifted register
            {
                auto offset = (opcode >> 6) & 0x1F;
                auto res = loReg(srcReg);

                if(!offset && instOp) offset = 32; // shift by 0 is really 32 (except for LSL)

                bool carry = cpsr & Flag_C;
                switch(instOp)
                {
                    case 0: // LSL
                        if(offset != 0)
                        {
                            carry = res & (1 << (32 - offset));
                            res <<= offset;
                        }
                        break;
                    case 1: // LSR
                        carry = res & (1 << (offset - 1));
                        if(offset == 32)
                            res = 0;
                        else
                            res >>= offset;
                        break;
                    case 2: // ASR
                    {
                        auto sign = res & signBit;
                        carry = res & (1 << (offset - 1));
                        if(offset == 32)
                            res = sign ? 0xFFFFFFFF : 0;
                        else
                        {
                            res >>= offset;
                            if(sign)
                                res |= ~(0xFFFFFFFF >> offset);
                        }
                        break;
                    }
                    default:
                        assert(!"Invalid format 1 shift type");
                }

                loReg(dstReg) = res;

                cpsr = (cpsr & 0x1FFFFFFF)
                        | (res & signBit ? Flag_N : 0)
                        | (res == 0 ? Flag_Z : 0)
                        | (carry ? Flag_C : 0);
            }

            break;
        }
        case 0x2: // format 3, mov/cmp immediate
        case 0x3: // format 3, add/sub immediate
        {
            auto instOp = (opcode >> 11) & 0x3;
            auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
            uint8_t offset = opcode & 0xFF;

            auto dst = loReg(dstReg);

            uint32_t res = 0;
            bool carry = false;

            switch(instOp)
            {
                case 0: // MOV
                    loReg(dstReg) = offset;
                    cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (offset == 0 ? Flag_Z : 0); // N not possible
                    break;
                case 1: // CMP
                    res = dst - offset;
                    carry = !(offset > dst);
                    break;
                case 2: // ADD
                    loReg(dstReg) = res = dst + offset;
                    carry = res < dst;
                    break;
                case 3: // SUB
                    loReg(dstReg) = res = dst - offset;
                    carry = !(offset > dst);
                    break;
            }

            if(instOp) // not MOV
            {
                cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V))
                     | ((res & signBit) ? Flag_V : 0)
                     | (res == 0 ? Flag_Z : 0)
                     | (carry ? Flag_C : 0)
                     | (((dst ^ offset) & signBit) && ((dst ^ res) & signBit) ? Flag_V : 0);
            }
            break;
        }
        case 0x4: // formats 4-6
        {
            if(opcode & (1 << 11)) // format 6, PC-relative load
            {
                auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
                uint8_t word = opcode & 0xFF;

                // pc + 4, bit 1 forced to 0
                auto base = (pc + 2) & ~2;
                loReg(dstReg) = readMem32(base + (word << 2));
            }
            else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
            {
                auto op = (opcode >> 8) & 3;
                bool h1 = opcode & (1 << 7);
                bool h2 = opcode & (1 << 6);

                auto srcReg = static_cast<Reg>(((opcode >> 3) & 7) + (h2 ? 8 : 0));
                auto dstReg = static_cast<Reg>((opcode & 7) + (h1 ? 8 : 0));

                auto src = reg(srcReg);

                if(srcReg == Reg::PC)
                    src += 2;

                switch(op)
                {
                    case 0: // ADD
                        reg(dstReg) += reg(srcReg);
                        break;
                    case 1: // CMP
                    {
                        auto dst = reg(dstReg);
                        auto res = dst - src;
                        bool carry = !(src > dst);

                        cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V))
                             | ((res & signBit) ? Flag_V : 0)
                             | (res == 0 ? Flag_Z : 0)
                             | (carry ? Flag_C : 0)
                             | (((dst ^ src) & signBit) && ((dst ^ res) & signBit) ? Flag_V : 0);
                        break;
                    }
                    case 2: // MOV
                        reg(dstReg) = src;
                        break;
                    case 3: // BX
                    {
                        auto newPC = src;
                        if(!(newPC & 1))
                            cpsr &= ~Flag_T;

                        pc = newPC & 0xFFFFFFFE;
                        break;
                    }

                    default:
                        assert(!"Invalid format 5 op!");
                }
            }
            else // format 4, alu
            {
                auto instOp = (opcode >> 6) & 0xF;
                auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
                auto dstReg = static_cast<Reg>(opcode & 7);

                auto op1 = loReg(dstReg);
                auto op2 = loReg(srcReg);

                uint32_t res;
                bool carry = cpsr & Flag_C, overflow = cpsr & Flag_V; // preserved if logical op

                switch(instOp)
                {
                    case 0x0: // AND
                        reg(dstReg) = res = op1 & op2;
                        break;
                    case 0x1: // EOR
                        reg(dstReg) = res = op1 ^ op2;
                        break;
                    case 0x2: // LSL
                        if(op2 >= 32)
                        {
                            carry = op2 == 32 ? (op1 & 1) : 0;
                            reg(dstReg) = res = 0;
                        }
                        else if(op2)
                        {
                            carry = op1 & (1 << (32 - op2));
                            reg(dstReg) = res = op1 << op2;
                        }
                        else
                            res = op1;
                        break;
                    case 0x3: // LSR
                        if(op2 >= 32)
                        {
                            carry = op2 == 32 ? (op1 & (1 << 31)) : 0;
                            reg(dstReg) = res = 0;
                        }
                        else if(op2)
                        {
                            carry = op1 & (1 << (op2 - 1));
                            reg(dstReg) = res = op1 >> op2;
                        }
                        else
                            res = op1;
                        break;
                    case 0x4: // ASR
                    {
                        auto sign = op1 & signBit;
                        if(op2 >= 32)
                        {
                            carry = sign;
                            reg(dstReg) = res = sign ? 0xFFFFFFFF : 0;
                        }
                        else if(op2)
                        {
                            carry = op1 & (1 << (op2 - 1));
                            res = op1 >> op2;

                            if(sign)
                                res |= ~(0xFFFFFFFF >> op2);

                            reg(dstReg) = res;
                        }
                        else
                            res = op1;

                        break;
                    }
                    case 0x5: // ADC
                    {
                        int c = carry ? 1 : 0;
                        reg(dstReg) = res = op1 + op2 + c;
                        carry = res < op1 || (res == op1 && c);
                        overflow = !((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit);

                        break;
                    }
                    case 0x6: // SBC
                    {
                        int c = carry ? 1 : 0;
                        res = op1 - op2 + c - 1;
                        carry = !(op2 > op1 || (op2 == op1 && !c));
                        overflow = ((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit);
                        break;
                    }
                    case 0x7: // ROR
                    {
                        int shift = op2 & 0x1F;
                                            
                        if(op2)
                            carry = op1 & (1 << (shift - 1));

                        reg(dstReg) = res = (op1 >> shift) | (op1 << (32 - shift));
                        break;
                    }
                    case 0x8: // TST
                        res = op1 & op2;
                        break;
                    case 0x9: // NEG
                    {
                        reg(dstReg) = res = 0 - op2;
                        carry = !(op2 > 0); //?
                        overflow = (op2 & signBit) && (res & signBit);
                        break;
                    }
                    case 0xA: // CMP
                        res = op1 - op2;
                        carry = !(op2 > op1);
                        overflow = ((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit); // different signs and sign changed
                        break;
                    case 0xB: // CMN
                        res = op1 + op2;
                        carry = res < op1;
                        overflow = !((op1 ^ op2) & signBit) && ((op1 ^ res) & signBit); // same signs and sign changed
                        break;
                    case 0xC: // ORR
                        reg(dstReg) = res = op1 | op2;
                        break;
                    case 0xD: // MUL
                        // carry is meaningless, v is unaffected
                        reg(dstReg) = res = op1 * op2;
                        break;
                    case 0xE: // BIC
                        reg(dstReg) = res = op1 & ~op2;
                        break;
                    case 0xF: // MVN
                        reg(dstReg) = res = ~op2;
                        break;

                    default:
                        assert(!"Invalid format 4 op!");
                }

                // cond code
                cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit ? Flag_N : 0) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow ? Flag_V : 0);
            }
            
            break;
        }
        case 0x5: // formats 7-8
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
                        auto val = readMem16(addr);
                        if(val & 0x8000)
                            loReg(dstReg) = val | 0xFFFF0000;
                        else
                            loReg(dstReg) = val;
                    }
                    else // LDRSB
                    {
                        auto val = readMem8(addr);
                        if(val & 0x80)
                            loReg(dstReg) = val | 0xFFFFFF00;
                        else
                            loReg(dstReg) = val;
                    }
                }
                else
                {
                    if(hFlag) // LDRH
                        loReg(dstReg) = readMem16(addr);
                    else // STRH
                        writeMem16(addr, loReg(dstReg));
                }
            }
            else // format 7, load/store with reg offset
            {
                bool isLoad = opcode & (1 << 11);
                bool isByte = opcode & (1 << 10);
                
                if(isLoad)
                {
                    if(isByte) // LDRB
                        loReg(dstReg) = readMem8(addr);
                    else // LDR
                        loReg(dstReg) = readMem32(addr);
                }
                else
                {
                    if(isByte) // STRB
                        writeMem8(addr, loReg(dstReg));
                    else // STR
                        writeMem32(addr, loReg(dstReg));
                }
            }
            
            break;
        }
        case 0x6: // format 9, load/store with imm offset
        case 0x7:
        {
            bool isByte = opcode & (1 << 12);
            bool isLoad = opcode & (1 << 11);
            auto offset = ((opcode >> 6) & 0x1F);
            auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
            auto dstReg = static_cast<Reg>(opcode & 7);

            auto base = loReg(baseReg);

            if(isLoad)
            {
                if(isByte) // LDRB
                    loReg(dstReg) = readMem8(base + offset);
                else // LDR
                    loReg(dstReg) = readMem32(base + (offset << 2));
            }
            else
            {
                if(isByte) // STRB
                    writeMem8(base + offset, loReg(dstReg));
                else // STR
                    writeMem32(base + (offset << 2), loReg(dstReg));
            }
            break;
        }
        case 0x8: // format 10, load/store halfword
        {
            bool isLoad = opcode & (1 << 11);
            auto offset = ((opcode >> 6) & 0x1F) << 1;
            auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
            auto dstReg = static_cast<Reg>(opcode & 7);

            if(isLoad) // LDRH
                loReg(dstReg) = readMem16(loReg(baseReg) + offset);
            else // STRH
                writeMem16(loReg(baseReg) + offset, loReg(dstReg));

            break;
        }
        case 0x9: // format 11, SP-relative load/store
        {
            bool isLoad = opcode & (1 << 11);
            auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
            auto word = (opcode & 0xFF) << 2;

            if(isLoad)
                loReg(dstReg) = readMem32(reg(Reg::SP) + word);
            else
                writeMem32(reg(Reg::SP) + word, loReg(dstReg));

            break;
        }
        case 0xA: // format 12, load address
        {
            bool isSP = opcode & (1 << 11);
            auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
            auto word = (opcode & 0xFF) << 2;

            if(isSP)
                loReg(dstReg) = reg(Reg::SP) + word;
            else
                loReg(dstReg) = ((pc + 2) & ~2) + word; // + 4, bit 1 forced to 0

            break;
        }
        case 0xB: // formats 13-14
        {
            if(opcode & (1 << 10)) // format 14, push/pop
            {
                bool isLoad = opcode & (1 << 11);
                bool pclr = opcode & (1 << 8); // store LR/load PC
                uint8_t regList = opcode & 0xFF;

                // count regs
                int numRegs = pclr ? 1 : 0;

                for(uint8_t t = regList; t; t >>= 1)
                {
                    if(t & 1)
                        numRegs++;
                }

                if(isLoad) // POP
                {
                    auto addr = reg(Reg::SP);

                    int i = 0;
                    for(; regList; regList >>= 1, i++)
                    {
                        if(regList & 1)
                        {
                            regs[i] = readMem32(addr);
                            addr += 4;
                        }
                    }
                    if(pclr)
                    {
                        pc = readMem32(addr) & ~1; /*ignore thumb bit*/
                        addr += 4;
                    }

                    reg(Reg::SP) = addr;
                }
                else // PUSH
                {
                    auto addr = reg(Reg::SP) - numRegs * 4;
                    reg(Reg::SP) = addr;

                    int i = 0;
                    for(; regList; regList >>= 1, i++)
                    {
                        if(regList & 1)
                        {
                            writeMem32(addr, regs[i]);
                            addr += 4;
                        }
                    }
                    
                    if(pclr)
                        writeMem32(addr, reg(Reg::LR));
                }
            }
            else // format 13, add offset to SP
            {
                bool isNeg = opcode & (1 << 7);
                int off = (opcode & 0x7F) << 2;

                if(isNeg)
                    reg(Reg::SP) -= off;
                else
                    reg(Reg::SP) += off;
            }
            break;
        }
        case 0xC: // format 15, multiple load/store
        {
            bool isLoad = opcode & (1 << 11);
            auto baseReg = static_cast<Reg>((opcode >> 8) & 7);
            uint8_t regList = opcode & 0xFF;

            auto addr = loReg(baseReg) & ~3;

            bool baseInList = regList & (1 << static_cast<int>(baseReg));

            int i = 0;
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                if(isLoad)
                    regs[i] = readMem32(addr);
                else
                    writeMem32(addr, regs[i]);

                addr += 4;
            }

            if(!isLoad || !baseInList)
                reg(baseReg) = addr;
            break;
        }
        case 0xD: // formats 16-17
        {
            auto cond = (opcode >> 8) & 0xF;
            if(cond == 0xF) // format 17, SWI
            {
                auto ret = pc & ~1;
                spsr[1/*svc*/] = cpsr;

                pc = 8;
                cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x13; //supervisor mode
                reg(Reg::LR) = ret;
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
                    // BVS BVC
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
                        printf("THUMB cond %x @%x\n", cond, pc - 2);
                        exit(0);
                }

                if(condVal)
                    pc += offset * 2 + 2 /*prefetch*/;
            }
            break;
        }
        case 0xE: // format 18, unconditional branch
        {
            uint32_t offset = (opcode & 0x7FF) << 1;
            if(offset & 0x800)
                offset |= 0xFFFFF000;

            pc += offset + 2 /*prefetch*/;

            break;
        }
        case 0xF: // format 19, long branch with link
        {
            bool high = opcode & (1 << 11);
            uint32_t offset = opcode & 0x7FF;

            if(!high) // first half
            {
                offset <<= 12;
                if(offset & (1 << 22))
                    offset |= 0xFF800000; //sign extend
                reg(Reg::LR) = pc + 2 + offset;
            }
            else // second half
            {
                auto temp = pc;
                pc = reg(Reg::LR) + (offset << 1);
                reg(Reg::LR) = temp | 1; // magic switch to thumb bit...
            }

            break;
        }
    }

    return mem.getAccessCycles(pc, 2, true);
}

int AGBCPU::doALUOp(int op, Reg destReg, uint32_t op1, uint32_t op2, bool setCondCode, bool carry)
{
    // output
    uint32_t res;
    bool overflow = cpsr & Flag_V; // preserved if logical op

    auto doAdd = [&res, &carry, &overflow](uint32_t a, uint32_t b, int c = 0)
    {
        res = a + b + c;
        carry = res < a || (res == a && c) /*for adc*/;
        overflow = !((a ^ b) & signBit) && ((a ^ res) & signBit);  // same sign and sign changed

        assert(carry == !!((static_cast<uint64_t>(a) + b + c) & (1ull << 32)));

        return res;
    };

    auto doSub = [&res, &carry, &overflow, &doAdd](uint32_t a, uint32_t b, int c = 1)
    {
        res = a - b + c - 1;
        carry = !(b > a || (b == a && !c) /*for sbc*/);
        overflow = ((a ^ b) & signBit) && ((a ^ res) & signBit);  // different sign and sign changed

        assert(carry == !((static_cast<uint64_t>(a) - b + c - 1) & (1ull << 32)));

        return res;
    };

    switch(op)
    {
        case 0x0: // AND
            reg(destReg) = res = op1 & op2;
            break;
        case 0x1: // EOR
            reg(destReg) = res = op1 ^ op2;
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
            break;
        case 0x9: // TEQ
            res = op1 ^ op2;
            break;
        case 0xA: // CMP
            doSub(op1, op2);
            break;
        case 0xB: // CMN
            doAdd(op1, op2);
            break;
        case 0xC: // ORR
            reg(destReg) = res = op1 | op2;
            break;
        case 0xD: // MOV
            reg(destReg) = res = op2;
            break;
        case 0xE: // BIC
            reg(destReg) = res = op1 & ~op2;
            break;
        case 0xF: // MVN
            reg(destReg) = res = ~op2;
            break;
        default:
            assert(!"Invalid ALU op!");
    }

    // cond code
    if(setCondCode)
    {
        if(destReg == Reg::PC)
            cpsr = getSPSR(); // restore
        else
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit ? Flag_N : 0) | (res == 0 ? Flag_Z : 0) | (carry ? Flag_C : 0) | (overflow ? Flag_V : 0);
    }

    return 1;
}

bool AGBCPU::serviceInterrupts()
{
    if((cpsr & Flag_I) || !(mem.readIOReg(IO_IME) & 1))
        return false;

    const auto flag = mem.readIOReg(IO_IF);
    const auto enabled = mem.readIOReg(IO_IE);

    if(enabled & flag)
    {
        //halted = false;

        auto ret = loReg(Reg::PC) + 4;
        spsr[3/*irq*/] = cpsr;

        loReg(Reg::PC) = 0x18;
        cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x12; // irq mode
        reg(Reg::LR) = ret;
        return true;
    }

    return false;
}

int AGBCPU::dmaTransfer(int channel)
{
    int regOffset = channel * 12;

    auto &dmaControl = mem.getIOReg(IO_DMA0CNT_H + regOffset);
    auto srcAddr = (mem.readIOReg(IO_DMA0SAD + regOffset) | (mem.readIOReg(IO_DMA0SAD + regOffset + 2) << 16)) & (channel ? 0xFFFFFFF : 0x7FFFFFF); // 1 bit less for DMA0
    auto dstAddr = (mem.readIOReg(IO_DMA0DAD + regOffset) | (mem.readIOReg(IO_DMA0DAD + regOffset + 2) << 16)) & (channel == 3 ? 0xFFFFFFF : 0x7FFFFFF); // 1 bit less for !DMA3
    auto count = mem.readIOReg(IO_DMA0CNT_L  + regOffset);

    bool is32Bit = dmaControl & DMACNTH_32Bit;
    int dstMode = (dmaControl & DMACNTH_DestMode) >> 5;
    int srcMode = (dmaControl & DMACNTH_SrcMode) >> 7;

    //printf("DMA%i %xx%i %x -> %x\n", channel, count, is32Bit ? 4 : 2, srcAddr, dstAddr);

    int cycles = count * 2 + 2; // FIXME: 1N + (n-1)S read + 1N + (n-1)S write + 2I (or 4I if both addrs in gamepak)

    while(count--)
    {
        if(is32Bit)
            writeMem32(dstAddr, readMem32(srcAddr & ~3));
        else
            writeMem16(dstAddr, readMem16(srcAddr & ~1));

        if(dstMode == 0 || dstMode == 3)
            dstAddr += is32Bit ? 4 : 2;
        else if(dstMode == 1)
            dstAddr -= is32Bit ? 4 : 2;

        if(srcMode == 0)
            srcAddr += is32Bit ? 4 : 2;
        else if(srcMode == 1)
            srcAddr -= is32Bit ? 4 : 2;
    }

    if(!(dmaControl & DAMCNTH_Repeat))
        dmaControl &= ~DMACNTH_Enable;

    return cycles;
}

void AGBCPU::updateTimers(int cycles)
{
    bool overflow = false;
    for(int i = 0; i < 4; i++)
    {
        if(!timerPrescalers[i])
        {
            overflow = false;
            continue;
        }

        auto oldCount = timerCounters[i];

        // count-up
        if(timerPrescalers[i] == -1)
        {
            if(overflow)
                timerCounters[i]++;
        }
        else
        {
            int count = (timer & (timerPrescalers[i] - 1)) + cycles;
            if(count >= timerPrescalers[i])
                timerCounters[i] += count / timerPrescalers[i];
        }

        overflow = timerCounters[i] < oldCount;

        if(overflow)
        {
            timerCounters[i] = mem.readIOReg(IO_TM0CNT_L + i * 4);
            flagInterrupt(Int_Timer0 << i);
        }
    }
    timer += cycles;
}