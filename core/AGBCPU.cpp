#include <cassert>
#include <cstdio>
#include <cstdlib> //exit
#include <cstring>
#include <utility>

#include "AGBCPU.h"
#include "AGBMemory.h"
#include "AGBRegs.h"

AGBCPU::AGBCPU(AGBMemory &mem) : mem(mem)
{}

void AGBCPU::reset()
{
    cpsr = Flag_I | Flag_F | 0x13 /*supervisor mode*/;
    loReg(Reg::PC) = 0;
    modeChanged();
    updateARMPC();
    halted = false;

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
        unsigned int exec;

        // DMA
        if(dmaTriggered)
        {
            exec = 0;
            auto trig = dmaTriggered;
            for(int chan = 0; chan < 4 && dmaTriggered; chan++, trig >>= 1)
            {
                if(trig & 1)
                    exec += dmaTransfer(chan);
            }
            dmaTriggered = trig;
        }
        else if(!halted)
        {
            // CPU
            exec = (cpsr & Flag_T) ? executeTHUMBInstruction() : executeARMInstruction();
        }
        else
            exec = 4; // higher = less overhead

        if(currentInterrupts)
            serviceInterrupts(); // cycles?

        if(timerInterruptEnabled)
        {
            updateTimers(timerDelayed + exec);
            timerDelayed = 0;
        }
        else
            timerDelayed += exec;

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

    currentInterrupts = (mem.readIOReg(IO_IME) & 1) ? mem.readIOReg(IO_IE) & mem.readIOReg(IO_IF) : 0;
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
    }
}

uint8_t AGBCPU::readMem8(uint32_t addr) const
{
    // handle IO reads as 16-bit
    if((addr >> 24) == 0x4)
    {
        auto tmp = mem.read16(addr & ~1);
        return (addr & 1) ? tmp >> 8 : tmp;
    }

    return mem.read8(addr);
}

uint32_t AGBCPU::readMem16(uint32_t addr)
{
    if(!(addr & 1))
        return readMem16Aligned(addr);

    // this returns the 32-bit result of an unaligned 16-bit read
    uint32_t val = readMem16Aligned(addr & ~1);

    return (val >> 8) | (val << 24);
}

uint16_t AGBCPU::readMem16Aligned(uint32_t addr)
{
    assert((addr & 1) == 0);

    if((addr >> 24) == 0x4)
    {
        switch(addr & 0xFFFFFF)
        {
            case IO_TM0CNT_L:
            case IO_TM1CNT_L:
            case IO_TM2CNT_L:
            case IO_TM3CNT_L:
                // sync
                updateTimers(timerDelayed);
                timerDelayed = 0;
                return timerCounters[((addr & 0xFFFFFF) - IO_TM0CNT_L) / 12];
        }
    }

    return mem.read16(addr);
}

uint32_t AGBCPU::readMem32(uint32_t addr)
{
    if(!(addr & 3))
        return readMem32Aligned(addr);

    uint32_t val = readMem32Aligned(addr & ~3);

    int shift = (addr & 3) << 3;
    return (val >> shift) | (val << (32 - shift));
}

uint32_t AGBCPU::readMem32Aligned(uint32_t addr)
{
    assert((addr & 3) == 0);
    if((addr >> 24) == 0x4) // IO
        return readMem16Aligned(addr) | (readMem16Aligned(addr + 2) << 16);

    return mem.read32(addr);
}

void AGBCPU::writeMem8(uint32_t addr, uint8_t data)
{
    if((addr >> 24) == 0x4)
    {
        if(addr == 0x4000301/*HALTCNT*/)
        {
            if(data & 0x80)
                printf("STOP\n");
            else
                halted = true;
        }
        else
        {
            // promote IO to 16-bit
            auto tmp = mem.readIOReg((addr & ~1) & 0x3FF);
            writeMem16(addr & ~1, addr & 1 ? (tmp & 0xFF) | (data << 8) : (tmp & 0xFF00) | data);
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
                updateTimers(timerDelayed);
                timerDelayed = 0;
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
                updateTimers(timerDelayed);
                timerDelayed = 0;

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

                break;
            }

            case IO_IE:
                currentInterrupts = (mem.readIOReg(IO_IME) & 1) ? data & mem.readIOReg(IO_IF) : 0;
                break;

            case IO_IF: // writing IF bits clears them internally
                data = mem.getIOReg(IO_IF) & ~data;

                currentInterrupts = (mem.readIOReg(IO_IME) & 1) ? mem.readIOReg(IO_IE) & data : 0;
                break;

            case IO_IME:
                currentInterrupts = (data & 1) ? mem.readIOReg(IO_IE) & mem.readIOReg(IO_IF) : 0;
                break;
        }
    }

    mem.write16(addr, data);
}

void AGBCPU::writeMem32(uint32_t addr, uint32_t data)
{
    addr &= ~3;

    if((addr >> 24) == 0x4)
    {
        writeMem16(addr, data);
        writeMem16(addr + 2, data >> 16);
    }
    else
        mem.write32(addr, data);
}

// returns cycle count
int AGBCPU::executeARMInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    assert(*armPCPtr == mem.read32Fast(pc));
    auto opcode = *armPCPtr++;

    auto timing = pcSCycles;

    pc += 4;

    // helpers
    const auto getShiftedReg = [this, opcode, pc](Reg r, uint8_t shift, bool &carry)
    {
        auto ret = reg(r);

        // prefetch
        if(r == Reg::PC)
            ret += (shift & 1) ? 8 : 4;

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
            if(shiftType && shiftAmount == 0) // lsr/asr shift by 32 instead of 0 
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
                                            
                    carry = ret & (1 << (shiftAmount - 1));
                    ret = (ret >> shiftAmount) | (ret << (32 - shiftAmount));
                }
                break;

            default:
                assert(!"Invalid shift type!");
        }
        
        return ret;
    };

    const auto halfwordTransfer = [this](uint32_t opcode, bool isPre)
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

        // pc offset
        if(baseReg == Reg::PC)
            addr += 4;

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

            if(halfWords && !sign)
                loReg(srcDestReg) = readMem16(addr); // LDRH
            else if(halfWords && !(addr & 1)) // LDRSH (aligned)
                loReg(srcDestReg) = static_cast<int16_t>(readMem16Aligned(addr)); // sign extend
            else // LDRSB ... or misaligned LDRSH
                loReg(srcDestReg) = static_cast<int8_t>(readMem8(addr)); // sign extend
        }
        else
        {
            // only unsigned halfword stores
            assert(opcode & (1 << 5)); // half
            assert(!(opcode & (1 << 6))); // sign

            // FIXME: PC is + 12
            assert(srcDestReg != Reg::PC);
            writeMem16(addr, loReg(srcDestReg)); // STRH
        }
    };

    // 0-3
    const auto doDataProcessing = [this](uint32_t opcode, uint32_t op2, bool carry, int pcInc = 4)
    {
        auto op1Reg = static_cast<Reg>((opcode >> 16) & 0xF);
        auto op1 = reg(op1Reg);
        if(op1Reg == Reg::PC)
            op1 += pcInc;

        auto instOp = (opcode >> 21) & 0xF;
        bool setCondCode = (opcode & (1 << 20));
        auto destReg = static_cast<Reg>((opcode >> 12) & 0xF);
        setCondCode ? doALUOp(instOp, destReg, op1, op2, carry) : doALUOpNoCond(instOp, destReg, op1, op2);
    };

    //4-7
    const auto doSingleDataTransfer = [this, timing, pc, &getShiftedReg](uint32_t opcode, bool isReg, bool isPre) __attribute__((always_inline))
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
            offset = getShiftedReg(static_cast<Reg>(opcode & 0xF), (opcode >> 4) & 0xFE, carry);
        }

        if(!(opcode & (1 << 23))) // !up
            offset = -offset;

        auto addr = loReg(baseReg);

        // pc offset
        if(baseReg == Reg::PC)
            addr += 4;

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
            if(isByte)
                loReg(srcDestReg) = readMem8(addr);
            else
                loReg(srcDestReg) = readMem32(addr);

            if(srcDestReg == Reg::PC)
                updateARMPC();

            return timing + mem.getAccessCycles(addr, isByte ? 1 : 4, false) + 1; // 1S + 1N + 1I
            // TODO: +1S+1N if dst == PC
        }
        else
        {
            // FIXME: PC is + 12
            assert(srcDestReg != Reg::PC);
            if(isByte)
                writeMem8(addr, loReg(srcDestReg));
            else
                writeMem32(addr, loReg(srcDestReg));

            return mem.getAccessCycles(pc, 4, false) + mem.getAccessCycles(addr, isByte ? 1 : 4, false); // 2N
        }
    };

    // 8-9
    const auto doBlockDataTransfer = [this](uint32_t opcode, bool preIndex)
    {
        bool isUp = opcode & (1 << 23);
        bool isLoadForce = opcode & (1 << 22);
        bool writeBack = opcode & (1 << 21);
        bool isLoad = opcode & (1 << 20);
        auto baseReg = mapReg(static_cast<Reg>((opcode >> 16) & 0xF));
        uint16_t regList = opcode;

        if(isLoadForce)
        {
            //assert(!writeBack); // "should not be used"
            assert(!isLoad || !(regList & (1 << 15))); // TODO: load with r15 (mode change)
        }

        auto addr = loReg(baseReg) & ~3;

        // flip decrement addressing around so that regs are stored in the right order
        uint32_t lowAddr;
        if(!isUp)
        {
            for(uint16_t t = regList; t; t >>= 1)
            {
                if(t & 1)
                    addr -= 4;
            }
            lowAddr = addr;

            if(!preIndex)
                addr += 4;
        }
        else if(preIndex)
            addr += 4;

        if(isLoad && regList & (1 << static_cast<int>(baseReg)))
            writeBack = false; // don't override

        int i = 0;
        if(baseReg == curSP)
        {
            // stack push/pull, we can assume RAM
            auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr));

            for(; regList && i < 16; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                // directly index to force user
                auto reg = isLoadForce ? static_cast<Reg>(i): mapReg(static_cast<Reg>(i));

                if(isLoad)
                    loReg(reg) = *ptr++;
                else
                    *ptr++ = loReg(reg);

                addr += 4;
            }
        }
        else
        {
            for(; regList; regList >>= 1, i++)
            {
                if(!(regList & 1))
                    continue;

                // directly index to force user
                auto reg = isLoadForce ? static_cast<Reg>(i) : mapReg(static_cast<Reg>(i));

                if(isLoad)
                    loReg(reg) = readMem32Aligned(addr);
                else
                    writeMem32(addr, loReg(reg));

                addr += 4;
            }
        }

        if(preIndex && writeBack)
            addr -= 4;

        if(writeBack)
            loReg(baseReg) = isUp ? addr : lowAddr;

        if(isLoad && (opcode & (1 << 15))) // load PC
            updateARMPC();
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
        case 0x0: // data processing with register (and halfword transfer/multiply)
        {
            if(((opcode >> 4) & 9) == 9)
            {
                if((opcode >> 5) & 3) // halfword transfer 
                {
                    halfwordTransfer(opcode, false);
                    break;
                }

                if(opcode & (1 << 23)) // MULL/MLAL
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

            auto op2Shift = (opcode >> 4) & 0xFF;
            auto op2Reg = static_cast<Reg>(opcode & 0xF);

            // reg arg2
            bool carry;
            auto op2 = getShiftedReg(op2Reg, op2Shift, carry);
            doDataProcessing(opcode, op2, carry, (op2Shift & 1) ? 8 : 4);

            return timing + (op2Shift & 1); // +1I if shift by reg
        }
        case 0x1: // data processing with register (and branch exchange/swap)
        {
            if((opcode & 0x0FFFFF00) == 0x012FFF00) // Branch and Exchange (BX)
            {
                auto instOp = (opcode >> 4) & 0xF;
                assert(instOp == 1); //
                auto newPC = reg(static_cast<Reg>(opcode & 0xF));
                pc = newPC & 0xFFFFFFFE;

                if(newPC & 1)
                {
                    cpsr |= Flag_T;
                    updateTHUMBPC(pc);
                }
                else
                    updateARMPC();

                return timing; // TODO: timing
            }

            if(((opcode >> 4) & 9) == 9)
            {
                if((opcode >> 5) & 3) // halfword transfer
                {
                    halfwordTransfer(opcode, true);
                    break;
                }
                else // SWP
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

                break;
            }

            auto op2Shift = (opcode >> 4) & 0xFF;
            auto op2Reg = static_cast<Reg>(opcode & 0xF);

            // reg arg2
            bool carry;
            auto op2 = getShiftedReg(op2Reg, op2Shift, carry);
            doDataProcessing(opcode, op2, carry, (op2Shift & 1) ? 8 : 4);

            return timing + (op2Shift & 1); // +1I if shift by reg
        }
        case 0x2: // data processing with immediate
        {
            // get the immediate value
            uint32_t op2 = opcode & 0xFF;
            int shift = ((opcode >> 8) & 0xF) * 2;
            op2 = (op2 >> shift) | (op2 << (32 - shift));
            bool carry = shift ? op2 & (1 << 31) : cpsr & Flag_C;

            doDataProcessing(opcode, op2, carry);
            return timing;
        }
        case 0x3: // same as above, but also possibly MSR
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

                return timing;
            }

            doDataProcessing(opcode, op2, carry);
            return timing;
        }
        case 0x4: // Single Data Transfer (I = 0, P = 0)
            return doSingleDataTransfer(opcode, false, false);
        case 0x5: // Single Data Transfer (I = 0, P = 1)
            return doSingleDataTransfer(opcode, false, true);
        case 0x6: // Single Data Transfer (I = 1, P = 0)
            return doSingleDataTransfer(opcode, true, false);
        case 0x7: // Single Data Transfer (I = 1, P = 1)
            return doSingleDataTransfer(opcode, true, true);
        case 0x8: // Block Data Transfer (P = 0)
            doBlockDataTransfer(opcode, false);
            break;
        case 0x9: // Block Data Transfer (P = 1)
            doBlockDataTransfer(opcode, true);
            break;
        case 0xA: // Branch (B)
        {
            auto offset = (static_cast<int32_t>(opcode & 0xFFFFFF) << 8) >> 6;
            pc += offset + 4/*prefetch*/;
            updateARMPC();
            break;
        }
        case 0xB: // Branch with Link (BL)
        {
            auto offset = (static_cast<int32_t>(opcode & 0xFFFFFF) << 8) >> 6;
            reg(Reg::LR) = pc;
            pc += offset + 4/*prefetch*/;
            updateARMPC();
            break;
        }

        case 0xF: // SWI
        {
            auto ret = pc;
            spsr[1/*svc*/] = cpsr;

            pc = 8;
            cpsr = (cpsr & ~0x1F) | Flag_I | 0x13; //supervisor mode
            modeChanged();
            updateARMPC();
            loReg(curLR) = ret;
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
    assert(!(pc & 1));
    assert(*thumbPCPtr == mem.read16Fast(pc));
    auto opcode = *thumbPCPtr++;

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

int AGBCPU::doALUOp(int op, Reg destReg, uint32_t op1, uint32_t op2, bool carry)
{
    if(destReg == Reg::PC)
    {
        doALUOpNoCond(op, destReg, op1, op2);

        cpsr = getSPSR(); // restore
        modeChanged();

        if(cpsr & Flag_T)
            updateTHUMBPC(reg(Reg::PC));

        return 1;
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

    return 1;
}

int AGBCPU::doALUOpNoCond(int op, Reg destReg, uint32_t op1, uint32_t op2)
{
    auto &dest = reg(destReg);

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
        updateARMPC();

    return 1;
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

    return pcSCycles;
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

    return pcSCycles;
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

    return pcSCycles;
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
            return pcSCycles + 1; // +1I for shift by register
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
            return pcSCycles + 1;
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
                carry = op1 & (1 << (op2 - 1));
                res = static_cast<int32_t>(op1) >> op2;

                reg(dstReg) = res;
            }
            else
                reg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return pcSCycles + 1;
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
            res = op1 - op2 + c - 1;
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
            int prefix = op1 & (1 << 31) ? __builtin_clz(~op1) : __builtin_clz(op1);

            // more cycles the more bytes are non 0/ff
            int iCycles = prefix == 32 ? 1 : (4 - prefix / 8);
            return pcSCycles + iCycles;
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

    return pcSCycles;
}

int AGBCPU::doTHUMB05HiReg(uint16_t opcode, uint32_t &pc)
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
                    | ((res & signBit) ? Flag_N : 0)
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
            pc = newPC & 0xFFFFFFFE;

            if(!(newPC & 1))
            {
                cpsr &= ~Flag_T;
                updateARMPC();
            }
            else
                updateTHUMBPC(pc);

            break;
        }

        default:
            assert(!"Invalid format 5 op!");
    }

    if(dstReg == Reg::PC)
    {
        pc &= ~1;
        updateTHUMBPC(pc);
    }

    return pcSCycles;
}

int AGBCPU::doTHUMB06PCRelLoad(uint16_t opcode, uint32_t &pc)
{
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t word = opcode & 0xFF;

    // pc + 4, bit 1 forced to 0
    auto base = (pc + 2) & ~2;
    loReg(dstReg) = mem.read32Fast(base + (word << 2));

    return pcSCycles + pcNCycles + 1;
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
                auto val = readMem16(addr);
                if(val & 0x8000)
                    loReg(dstReg) = val | 0xFFFF0000;
                else
                    loReg(dstReg) = val;

                return pcSCycles + mem.getAccessCycles(addr, 2, false) + 1;
            }
            else // LDRSB
            {
                auto val = readMem8(addr);
                if(val & 0x80)
                    loReg(dstReg) = val | 0xFFFFFF00;
                else
                    loReg(dstReg) = val;

                return pcSCycles + mem.getAccessCycles(addr, 1, false) + 1; // a little off if this was a misaligned LDRSH
            }
        }
        else
        {
            if(hFlag) // LDRH
            {
                loReg(dstReg) = readMem16(addr);
                return pcSCycles + mem.getAccessCycles(addr, 2, false) + 1;
            }
            else // STRH
            {
                writeMem16(addr, loReg(dstReg));
                return pcNCycles + mem.getAccessCycles(addr, 2, false);
            }
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

            return pcSCycles + mem.getAccessCycles(addr, isByte ? 1 : 4, false) + 1;
        }
        else
        {
            if(isByte) // STRB
                writeMem8(addr, loReg(dstReg));
            else // STR
                writeMem32(addr, loReg(dstReg));

            return pcNCycles + mem.getAccessCycles(addr, isByte ? 1 : 4, false);
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
        loReg(dstReg) = readMem32(addr);
        return pcSCycles + mem.getAccessCycles(addr, 4, false) + 1;
    }
    else // STR
    {
        writeMem32(addr, loReg(dstReg));
        return pcNCycles + mem.getAccessCycles(addr, 4, false);
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
        loReg(dstReg) = readMem8(addr);
        return pcSCycles + mem.getAccessCycles(addr, 1, false) + 1;
    }
    else // STRB
    {
        writeMem8(addr, loReg(dstReg));
        return pcNCycles + mem.getAccessCycles(addr, 1, false);
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
        loReg(dstReg) = readMem16(addr);
        return pcSCycles + mem.getAccessCycles(addr, 2, false) + 1;
    }
    else // STRH
    {
        writeMem16(addr, loReg(dstReg));
        return pcNCycles + mem.getAccessCycles(addr, 2, false);
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
        loReg(dstReg) = mem.read32Fast(addr);
        return pcSCycles + mem.getAccessCycles(addr, 4, false) + 1;
    }
    else
    {
        mem.write32(addr, loReg(dstReg));
        return pcNCycles + mem.getAccessCycles(addr, 4, false);
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
        loReg(dstReg) = ((pc + 2) & ~2) + word; // + 4, bit 1 forced to 0

    return pcSCycles;
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

    return pcSCycles;
}

int AGBCPU::doTHUMB14PushPop(uint16_t opcode, uint32_t &pc)
{
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
            pc = *ptr++ & ~1; /*ignore thumb bit*/
            updateTHUMBPC(pc);
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

    return pcSCycles;
}

int AGBCPU::doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t &pc)
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

    return pcSCycles;
}

int AGBCPU::doTHUMB1617(uint16_t opcode, uint32_t &pc)
{
    auto cond = (opcode >> 8) & 0xF;
    if(cond == 0xF) // format 17, SWI
    {
        auto ret = pc & ~1;
        spsr[1/*svc*/] = cpsr;

        pc = 8;
        cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x13; //supervisor mode
        modeChanged();
        updateARMPC();
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
        {
            pc += offset * 2 + 2 /*prefetch*/;
            updateTHUMBPC(pc);
        }
    }

    return pcSCycles * 2 + pcNCycles; // 2S + 1N, probably a bit wrong for SWI
}

int AGBCPU::doTHUMB18UncondBranch(uint16_t opcode, uint32_t &pc)
{
    uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2

    pc += offset + 2 /*prefetch*/;
    updateTHUMBPC(pc);

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
        loReg(curLR) = pc + 2 + offset;

        return pcSCycles;
    }
    else // second half
    {
        auto temp = pc;
        pc = loReg(curLR) + (offset << 1);
        loReg(curLR) = temp | 1; // magic switch to thumb bit...

        auto ret = pcNCycles;

        updateTHUMBPC(pc);

        return ret + pcSCycles * 2; //?
    }
}

void AGBCPU::updateARMPC()
{
    armPCPtr = reinterpret_cast<const uint32_t *>(std::as_const(mem).mapAddress(loReg(Reg::PC)));
    pcSCycles = mem.getAccessCycles(loReg(Reg::PC), 4, true);
    pcNCycles = mem.getAccessCycles(loReg(Reg::PC), 4, false);
}

void AGBCPU::updateTHUMBPC(uint32_t pc)
{
    // called when PC is updated in THUMB mode (except for incrementing)
    thumbPCPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc)); // force const mapAddress
    pcSCycles = mem.getAccessCycles(pc, 2, true);
    pcNCycles = mem.getAccessCycles(pc, 2, false);
}

bool AGBCPU::serviceInterrupts()
{
    if((cpsr & Flag_I))
        return false;

    halted = false;

    auto ret = loReg(Reg::PC) + 4;
    spsr[3/*irq*/] = cpsr;

    loReg(Reg::PC) = 0x18;
    cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x12; // irq mode
    modeChanged();
    updateARMPC();
    loReg(curLR) = ret;
    return true;
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

    int width = is32Bit ? 4 : 2;
    int cycles = mem.getAccessCycles(srcAddr, width, false) + mem.getAccessCycles(srcAddr, width, true) * (count - 1) // 1N + (n-1)S read
               + mem.getAccessCycles(dstAddr, width, false) + mem.getAccessCycles(dstAddr, width, true) * (count - 1) // 1N + (n-1)S write
               + 2; // TODO: 2I (or 4I if both addrs in gamepak)

    srcAddr &= ~(width - 1);

    while(count--)
    {
        if(is32Bit)
            writeMem32(dstAddr, readMem32Aligned(srcAddr));
        else
            writeMem16(dstAddr, readMem16Aligned(srcAddr));

        if(dstMode == 0 || dstMode == 3)
            dstAddr += width;
        else if(dstMode == 1)
            dstAddr -= width;

        if(srcMode == 0)
            srcAddr += width;
        else if(srcMode == 1)
            srcAddr -= width;
    }

    if(!(dmaControl & DAMCNTH_Repeat))
        dmaControl &= ~DMACNTH_Enable;

    return cycles;
}

void AGBCPU::updateTimers(int cycles)
{
    uint8_t overflow = 0;
    auto enabled = timerEnabled;
    for(int i = 0; enabled; i++, enabled >>= 1)
    {
        if(!(enabled & 1))
            continue;

        auto oldCount = timerCounters[i];

        // count-up
        if(timerPrescalers[i] == -1)
        {
            if(overflow & (1 << (i - 1)))
                timerCounters[i]++;
        }
        else if(timerPrescalers[i] == 1)
            timerCounters[i] += cycles;
        else
        {
            int count = (timer & (timerPrescalers[i] - 1)) + cycles;
            if(count >= timerPrescalers[i])
                timerCounters[i] += count / timerPrescalers[i];
        }

        if(timerCounters[i] < oldCount)
        {
            overflow |= (1 << i);
            timerCounters[i] = mem.readIOReg(IO_TM0CNT_L + i * 4);
            if(timerInterruptEnabled & (1 << i))
                flagInterrupt(Int_Timer0 << i);
        }
    }
    timer += cycles;
}