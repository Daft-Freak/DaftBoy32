#include <cassert>
#include <functional>
#include <cstdio>
#include <variant>

#include "ThumbTarget.h"
#include "ThumbBuilder.h"

const Reg cycleCountReg = Reg::R0;
const Reg pcReg = Reg::R1; // output, can be used as a temp as well
const Reg cpuPtrReg = Reg::R8;

static bool isLowReg(Reg r)
{
    return static_cast<int>(r) < 8;
}

// helpers for accessing 8/16-bit regs/values

// copies 8 bit value from upper/lower reg or imm to reg
static void get8BitValue(ThumbBuilder &builder, Reg dst, std::variant<std::monostate, ThumbTarget::RegInfo, uint8_t> b)
{
    assert(b.index());

    if(std::holds_alternative<uint8_t>(b))
        builder.mov(dst, std::get<uint8_t>(b));
    else
    {
        auto reg = std::get<ThumbTarget::RegInfo>(b);

        if(reg.mask == 0xFF00)
            builder.lsr(dst, reg.reg, 8); // shift it down
        else if(dst != reg.reg) // assume it's already masked if it's already there
        {
            if(isLowReg(reg.reg))
                builder.uxtb(dst, reg.reg); // clear the high half
            else
            {
                builder.mov(dst, reg.reg);
                // definitely need to mask if src wasn't 8-bit
                // otherwise assume this isn't an emulated reg and doesn't need masked
                if(reg.mask == 0)
                    builder.uxtb(dst, dst);
            }
        }
    }
}

// store to 8-bit reg, modifies src
static void write8BitReg(ThumbBuilder &builder, ThumbTarget::RegInfo dst, Reg src)
{
    // do nothing if already in the right place
    if(dst.reg == src)
    {
        assert(!dst.mask || dst.mask == 0xFF);
        return;
    }

    // just copy if not masked
    // or >= R8, mostly because the code below would fail
    if(!dst.mask || !isLowReg(dst.reg))
    {
        builder.mov(dst.reg, src);
        return;
    }

    if(dst.mask == 0xFF)
    {
        // shift out low byte
        builder.lsr(dst.reg, dst.reg, 8);
        builder.lsl(dst.reg, dst.reg, 8);
    }
    else if(dst.mask == 0xFF00)
    {
        builder.uxtb(dst.reg, dst.reg);
        builder.lsl(src, src, 8);
    }

    builder.orr(dst.reg, src);
}

static void load16BitValue(ThumbBuilder &builder, Reg dst, uint16_t value)
{
    if(value <= 0xFF || value >> __builtin_ctz(value) <= 0xFF)
    {
        int shift = 0;
        if(value > 0xFF)
            shift = __builtin_ctz(value);

        builder.mov(dst, value >> shift);
        if(shift)
            builder.lsl(dst, dst, shift);
    }
    else
    {
        // high << 8 + low
        builder.mov(dst, value >> 8);
        builder.lsl(dst, dst, 8);
        builder.add(dst, value & 0xFF);
    }
}

static void load32BitValue(ThumbBuilder &builder, Reg dst, uint32_t value)
{
    int trailingZeros = __builtin_ctz(value);

    if(value <= 0xFFFF || builder.isValidModifiedImmediate(value))
        builder.mov(dst, value);
    else if(value >> trailingZeros <= 0xFFFF)
    {
        builder.mov(dst, value >> trailingZeros);
        builder.lsl(dst, dst, trailingZeros);
    }
    else
    {
        builder.mov(dst, value & 0xFFFF);
        if(value >> 15 == 0x1FFFF)
            builder.sxth(dst, dst);
        else
            builder.movt(dst, value >> 16);
    }
}

static void branchOver(ThumbBuilder &builder, std::function<void(ThumbBuilder &)> func, Condition cond = Condition::AL)
{
    // do nothing if builder already in a bad state
    if(builder.getError())
        return;

    // add placeholder branch
    auto branchStart = builder.getPtr();
    builder.b(0);
    assert(builder.getPtr() - branchStart <= 1); // only going to be 0 if out of space

    // build code to branch over
    func(builder);

    // check we didn't run out of space
    if(builder.getError())
        return;

    // patch in the real branch
    int offset = (builder.getPtr() - branchStart - 1) * 2;

    builder.patch(branchStart, branchStart + 1);

    if(cond == Condition::AL)
        builder.b(offset);
    else
        builder.b(cond, offset);

    assert(!builder.getError()); // should be the same size as the placeholder, so no error can happen
    builder.endPatch();
}

void ThumbTarget::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg regList[]
    {
        Reg::R3, // only if 32bit mode

        Reg::R4,
        Reg::R5,
        Reg::R6,
        Reg::R7,
        Reg::R9,

        // temps
        Reg::R10, // not very optimal
        Reg::R2, // the same as temp/0, should only get used where temp is removed
    };

    // assume "32bit mode"/no 8/16 bit ops if both pc and temp are 32bit

    bool is32Bit = sourceInfo.pcSize == 32 && sourceInfo.registers[0].size == 32;

    // alloc registers
    unsigned int allocOff = 0;

    if(!is32Bit)
        allocOff++; // skip R3

    regAlloc.emplace(0, Reg::R2); // temp

    int i = 1;
    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it, i++)
    {
        if(it->type == SourceRegType::Flags)
            flagsReg = i;

        if(it->alias)
            continue;

        // TODO: make sure we allocate all temps
        if(allocOff == std::size(regList))
            continue;

        if(it->type != SourceRegType::Temp && (regList[allocOff] == Reg::R2 || regList[allocOff] == Reg::R10))
            continue;

        regAlloc.emplace(i, regList[allocOff]);

        allocOff++;
    }

    // map flags
    for(auto & f : flagMap)
        f = 0xFF;

    i = 0;
    for(auto it = sourceInfo.flags.begin(); it != sourceInfo.flags.end(); ++it, i++)
        flagMap[static_cast<int>(it->type)] = i;

    // allocate the flags register if it isn't an alias
    if(!sourceInfo.registers[flagsReg].alias && !regAlloc.count(flagsReg))
        regAlloc.emplace(flagsReg, Reg::R11);

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool ThumbTarget::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint32_t pc, GenBlockInfo &blockInfo)
{
    // don't handle HRAM for now
    if(blockInfo.flags & GenBlock_StrictSync)
        return false;

    auto codePtr16 = reinterpret_cast<uint16_t *>(codePtr);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBufEnd));

    auto startPC = pc;

    // state
    uint16_t *lastInstrCycleCheck = nullptr;
    std::map<uint16_t, uint16_t *> branchTargets;
    std::multimap<uint16_t, uint16_t *> forwardBranchesToPatch;

    // cycle executed sync
    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;
    bool stackCycleCount = false; // cycle count stored to stack

    auto cycleExecuted = [this, &builder, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount]()
    {
        assert(!stackCycleCount);

        cyclesThisInstr += sourceInfo.cycleMul;

        // only the optimised not-HRAM path
        delayedCyclesExecuted += sourceInfo.cycleMul;
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted, &stackCycleCount]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(!stackCycleCount);

        assert(delayedCyclesExecuted < 0xFF);
        auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

        uint8_t u8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        auto cycleCountOff = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - cpuPtrInt;
        assert(cycleCountOff <= 0xFF);

        // load to r1, add and store back
        builder.ldr(Reg::R1, cpuPtrReg, cycleCountOff);
        builder.add(Reg::R1, u8Cycles);
        builder.str(Reg::R1, cpuPtrReg, cycleCountOff);

        delayedCyclesExecuted = 0;
    };

    auto writeCycleCountToStack = [this, &builder, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount](int &instrCycles, int offset, Reg tempReg)
    {
        if(!stackCycleCount)
        {
            assert(cyclesThisInstr == delayedCyclesExecuted);
            assert(cyclesThisInstr + instrCycles <= 0xFF);

            builder.mov(tempReg, cyclesThisInstr + instrCycles);
            builder.str(tempReg, Reg::SP, offset);

            cyclesThisInstr = delayedCyclesExecuted = 0;
            instrCycles = 0;
            stackCycleCount = true;
        }
        else
            assert(!cyclesThisInstr && !delayedCyclesExecuted);
    };

    auto setupMemAddr = [this, &blockInfo, &builder, &syncCyclesExecuted](std::variant<std::monostate, Reg, uint32_t> addr, uint8_t addrIndex, bool skipMov = false)
    {
        assert(addr.index()); // caller should have checked checkRegOrImm result

        if(std::holds_alternative<Reg>(addr))
        {
            auto reg = std::get<Reg>(addr);

            if(!sourceInfo.shouldSyncForRegIndex || sourceInfo.shouldSyncForRegIndex(addrIndex, blockInfo))
            {
                syncCyclesExecuted(); // uses R1/3
                skipMov = false; // if it was in R1 it isn't now
            }

            if(!skipMov)
                builder.mov(Reg::R1, reg);
        }
        else
        {
            auto immAddr = std::get<uint32_t>(addr);
            if(!sourceInfo.shouldSyncForAddress || sourceInfo.shouldSyncForAddress(immAddr))
                syncCyclesExecuted();

            assert(immAddr <= 0xFFFF);
            load16BitValue(builder, Reg::R1, immAddr);
        }
    };

    // do instructions
    int numInstructions = 0;
    uint16_t *opStartPtr = nullptr;
    uint16_t *lastImmLoadStart = nullptr, *lastImmLoadEnd = nullptr;
    bool newEmuOp = true;
    bool lastWasEI = false;
    bool forceExitAfter = false;

    auto beginInstr = blockInfo.instructions.begin();
    auto endInstr = blockInfo.instructions.end();

    for(auto instIt = beginInstr; instIt != endInstr; ++instIt)
    {
        auto &instr = *instIt;

        // handle branch targets
        if(instr.flags & GenOp_BranchTarget)
        {
            // store for backwards jumps
            if(lastInstrCycleCheck)
                branchTargets.emplace(pc, lastInstrCycleCheck); // after adjusting cycle count but before the jump
            else
            {
                // this is the first instruction, so make a cycle check for the branch to go to
                lastInstrCycleCheck = builder.getPtr() + 1; // skip the branch over it

                branchOver(builder, [&](ThumbBuilder &builder)
                {
                    // if <= 0 exit
                    branchOver(builder, [this, pc](ThumbBuilder &builder)
                    {
                        loadPCValue(builder, pc);
                        builder.bl((saveAndExitPtr - builder.getPtr()) * 2);
                    }, Condition::GT);
                });

                branchTargets.emplace(pc, lastInstrCycleCheck);
            }

            // patch forwards jumps
            // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
            auto jumps = forwardBranchesToPatch.equal_range(pc);
            for(auto it = jumps.first; it != jumps.second; ++it)
            {
                auto off = (lastInstrCycleCheck - (it->second + 1)) * 2;

                if(off > 2048)
                    continue; // it's too far away

                // replace BL with B+NOP
                builder.patch(it->second, it->second + 2);
                builder.b(off);
                builder.nop();
                builder.endPatch();
            }
            forwardBranchesToPatch.erase(jumps.first, jumps.second);
        }

        // flag helpers
        // assumes value in R1 if needCompare is true
        const auto updateZ = [this, &instr, &builder](bool needCompare)
        {
            if(writesFlag(instr.flags, SourceFlagType::Zero))
            {
                auto f = *mapReg8(flagsReg);

                if(needCompare)
                    builder.cmp(Reg::R1, 0);

                builder.b(Condition::NE, 2);
                // assuming we started with 0 and never set the same flag twice, we can use add instead of orr
                builder.add(f.reg, 1 << getFlagInfo(SourceFlagType::Zero).bit);
            }
        };

        const auto updateC = [this, &instr, &builder](bool needCompare, bool inverted = false)
        {
            if(writesFlag(instr.flags, SourceFlagType::Carry))
            {
                auto f = *mapReg8(flagsReg);

                if(needCompare)
                {
                    builder.cmp(Reg::R1, 0xFF);
                    builder.b(Condition::LE, 2);
                }
                else
                    builder.b(inverted ? Condition::CS : Condition::CC, 2);

                builder.add(f.reg, 1 << getFlagInfo(SourceFlagType::Carry).bit);
            }
        };

        const auto setFlag = [this, &instr, &builder](SourceFlagType flag)
        {
            if(writesFlag(instr.flags, flag))
                builder.add(mapReg8(flagsReg)->reg, 1 << getFlagInfo(flag).bit);
        };

        const auto carryIn32 = [this, &builder]()
        {
            int carryBit = getFlagInfo(SourceFlagType::Carry).bit;

            auto f = mapReg(flagsReg);

            // we don't care about the result here, just using the shift to set carry
            if(f)
                builder.tst(*f, *f, ShiftType::LSL, 32 - carryBit);
        };

        // branch targets

        pc += instr.len;

        // update start pointer if the last op was the end of an emulated op
        if(newEmuOp)
            opStartPtr = builder.getPtr();

        // first cycle (fetch)
        int instrCycles = instr.cycles;
        if(instrCycles && (blockInfo.flags & GenBlock_FirstCycleEarly))
        {
            cycleExecuted();
            instrCycles--;
        }

        // previous op was EI
        if(lastWasEI)
        {
            // enable interrupts for EI
            auto enableInterruptsNextCycleOff = sourceInfo.extraCPUOffsets[1];
            auto masterInterruptEnableOff = sourceInfo.extraCPUOffsets[0];
            assert(enableInterruptsNextCycleOff < 32);
            assert(masterInterruptEnableOff < 32);        

            // if(enableInterruptsNextCycle)
            // probably don't need this check... might get a false positive in some extreme case though
            builder.mov(Reg::R3, cpuPtrReg);
            builder.ldrb(Reg::R1, Reg::R3, enableInterruptsNextCycleOff);

            builder.cmp(Reg::R1, 0);
            builder.b(Condition::EQ, 6);

            // masterInterruptEnable = true
            builder.strb(Reg::R1, Reg::R3, masterInterruptEnableOff); // R1 == 1 here

            // enableInterruptsNextCycle = false
            builder.mov(Reg::R1, 0);
            builder.strb(Reg::R1, Reg::R3, enableInterruptsNextCycleOff);

            forceExitAfter = true;
            lastWasEI = false;
        }

        bool err = false;

        // validating wrappers
        auto checkReg = [this, &builder, &err, &instr](uint8_t index, std::optional<Reg> loadStoreReg = {}, bool isDst = false)
        {
            auto reg = mapReg(index);
            if(!reg)
            {
                if(loadStoreReg)
                {
                    assert(sourceInfo.registers[index].cpuOffset != 0xFFFF || sourceInfo.getRegOffset);
                    if(!isDst)
                    {
                        // get offset and load
                        int offset = sourceInfo.registers[index].cpuOffset;
                        if(offset == 0xFFFF)
                            offset = sourceInfo.getRegOffset(cpuPtr, index);

                        builder.ldr(*loadStoreReg, cpuPtrReg, offset);
                    }
                    return loadStoreReg;
                }

                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }

            return reg;
        };

        auto checkReg8 = [this, &builder, &err, &instr](uint8_t index)
        {
            auto reg = mapReg8(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }

            return reg;
        };

        // immediate helpers
        auto getLastImmLoad = [&builder, &instIt, &beginInstr, &lastImmLoadStart, &lastImmLoadEnd]() -> std::optional<uint32_t>
        {
            if(instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::LoadImm)
            {
                // remove the load
                assert(lastImmLoadStart);
                builder.removeRange(lastImmLoadStart, lastImmLoadEnd);
                lastImmLoadStart = lastImmLoadEnd = nullptr;

                return (instIt - 1)->imm;
            }

            return {};
        };

        auto checkRegOrImm = [&checkReg, &getLastImmLoad](uint8_t index, std::optional<Reg> loadReg = {}) -> std::variant<std::monostate, Reg, uint32_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                    return *imm;
            }

            if(auto reg = checkReg(index, loadReg))
                return *reg;

            return {};
        };

        auto checkRegOrImm8 = [&checkReg8, &getLastImmLoad](uint8_t index) -> std::variant<std::monostate, RegInfo, uint8_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                {
                    assert(!(*imm & 0xFFFFFF00));
                    return static_cast<uint8_t>(*imm);
                }
            }

            if(auto reg = checkReg8(index))
                return *reg;

            return {};
        };

        auto storeUnmappedReg = [this, &builder](uint8_t index, Reg reg)
        {
            // make sure we actually need to do this
            if(regAlloc.count(index))
                return;

            // get offset and store
            int offset = sourceInfo.registers[index].cpuOffset;
            if(offset == 0xFFFF)
                offset = sourceInfo.getRegOffset(cpuPtr, index);

            builder.str(reg, cpuPtrReg, offset);
        };

        // error handling
        auto badRegSize = [&err, &instr](int size)
        {
            printf("unhandled reg size %i in op %i\n", size, int(instr.opcode));
            err = true;
        };

        auto unhandledFlags = [&err, &instr](uint16_t flags)
        {
            printf("unhandled flags %x in op %i\n", flags, int(instr.opcode));
            err = true;
        };

        // clear/preserve flags
        // needs to be after the reg helpers
        uint8_t preserveMask = 0;

        bool needCarry = instr.opcode == GenOpcode::AddWithCarry
                      || instr.opcode == GenOpcode::SubtractWithCarry
                      || instr.opcode == GenOpcode::RotateLeftCarry
                      || instr.opcode == GenOpcode::RotateRightCarry;

        if(sourceInfo.registers[flagsReg].size == 8 && ((instr.flags & GenOp_WriteFlags) || needCarry) && instr.opcode != GenOpcode::DMG_DAA /* needs most flags */)
        {
            preserveMask = 0;

            if((instr.flags & GenOp_PreserveFlags))
            {
                for(int i = 0; i < 4; i++)
                {
                    if(instr.flags & (1 << i))
                        preserveMask |= 1 << sourceInfo.flags[i].bit;
                }
            }

            auto f = checkReg8(flagsReg); // TODO: assumes 8-bit flags

            if(f)
            {
                // need to save carry if it's used later
                if(needCarry)
                {
                    builder.mov(Reg::R3, f->reg);
                    builder.mov(Reg::R1, 1 << getFlagInfo(SourceFlagType::Carry).bit);
                    builder.and_(Reg::R3, Reg::R1);
                }

                if(instr.flags & GenOp_WriteFlags)
                {
                    builder.mov(Reg::R1, ~preserveMask & 0xFF);
                    builder.bic(f->reg, Reg::R1); // clear flags
                }
            }
        }

        switch(instr.opcode)
        {
            case GenOpcode::NOP:
                break;

            case GenOpcode::LoadImm:
                lastImmLoadStart = builder.getPtr();

                if(sourceInfo.registers[0].size == 32)
                    load32BitValue(builder, *mapReg(0), instr.imm); // TODO: use a literal here? (would require extra work if we remove it)
                else
                    load16BitValue(builder, *mapReg(0), instr.imm);
 
                lastImmLoadEnd = builder.getPtr();
                break;

            case GenOpcode::Move:
            {
                auto regSize = sourceInfo.registers[instr.dst[0]].size;
                if(regSize == 32)
                {
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);
                    auto src = checkRegOrImm(instr.src[0], Reg::R2);
                
                    if(src.index() && dst)
                    {
                        bool setFlags = instr.flags & GenOp_WriteFlags;
    
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            // try to do a mov, fall back to literal load
                            auto value = std::get<uint32_t>(src);
                            if((value <= 0xFFFF && !setFlags) || builder.isValidModifiedImmediate(value))
                                builder.mov(*dst, value);
                            else
                            {
                                loadLiteral(builder, *dst, value);
                                // uhoh, no flags
                                assert(!(instr.flags & GenOp_WriteFlags));
                            }
                        }
                        else
                            builder.mov(*dst, std::get<Reg>(src), setFlags);

                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    assert(!(instr.flags & GenOp_WriteFlags));

                    auto dst = checkReg(instr.dst[0]);

                    if(sourceInfo.registers[instr.src[0]].size == 8)
                    {
                        // 8 -> 16 bit
                        auto src = checkRegOrImm8(instr.src[0]);
                        if(src.index() && dst)
                        {
                            if(isLowReg(*dst))
                                get8BitValue(builder, *dst, src);
                            else // LDH (C)
                            {
                                get8BitValue(builder, Reg::R1, src);
                                builder.mov(*dst, Reg::R1);
                            }
                        }
                    }
                    else
                    {
                        auto src = checkRegOrImm(instr.src[0]);
                        if(src.index() && dst)
                        {
                            if(std::holds_alternative<uint32_t>(src))
                            {
                                auto value = std::get<uint32_t>(src);
                                assert(!(value & 0xFFFF0000));
                                // avoid a mov if possible
                                if(isLowReg(*dst))
                                    load16BitValue(builder, *dst, value);
                                else
                                {
                                    // effectively re-emit the LoadImm we just removed, at least we tried....
                                    load16BitValue(builder, Reg::R2, value);
                                    builder.mov(*dst, Reg::R2);
                                }
                            }
                            else
                                builder.mov(*dst, std::get<Reg>(src));
                        }
                    }
                }
                else if(regSize == 8)
                {
                    assert(!(instr.flags & GenOp_WriteFlags));

                    auto src = checkRegOrImm8(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);
                    if(src.index() && dst)
                    {
                        auto tmp = mapReg(0);
                        get8BitValue(builder, *tmp, src);
                        write8BitReg(builder, *dst, *tmp);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Load:
            case GenOpcode::Load2:
            case GenOpcode::Load4:
            {
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16 || addrSize == 32)
                {
                    auto addr = checkRegOrImm(instr.src[0], Reg::R2);
                    auto dst = addrSize == 16 ? checkReg8(instr.dst[0]) : RegInfo{*checkReg(instr.dst[0], Reg::R2, true), 0};

                    if(addr.index() && dst)
                    {
                        int pushMask = 1 << 0; // R0

                        if(addrSize == 32)
                            pushMask |= 1 << 3 | 1 << 4; // assume that we're using R3 and save R4 so we have somewhere to put the func addr

                        builder.push(pushMask, false);

                        // if the address is in a high reg and the last op was an add to it, the value should still be in R1
                        // this allows us to skip the mov (for stack pops)
                        bool skipMov = false;
                        if(addrSize == 16)
                        {
                            bool addrIsHighReg = std::holds_alternative<Reg>(addr) && !isLowReg(std::get<Reg>(addr));
                            if(!newEmuOp && addrIsHighReg && instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::Add && (instIt - 1)->dst[0] == instr.src[0] && !((instIt - 1)->flags & GenOp_WriteFlags))                        
                                skipMov = true;
                        }

                        // the sized version have two additional args
                        bool needCyclesFlags = sourceInfo.readMem8 != nullptr;

                        setupMemAddr(addr, instr.src[0], skipMov);

                        if(needCyclesFlags)
                        {
                            // cycle count
                            assert(pushMask == 0b11001);
                            writeCycleCountToStack(instrCycles, 12, Reg::R2);
                            builder.add(Reg::R2, Reg::SP, 12, false);

                            builder.mov(Reg::R3, instr.flags >> 8);
                        }

                        builder.mov(Reg::R0, cpuPtrReg);

                        // get func ptr
                        uintptr_t func = 0;
                        auto funcAddrReg = needCyclesFlags ? Reg::R4 : Reg::R2;

                        if(instr.opcode == GenOpcode::Load)
                        {
                            if(sourceInfo.readMem)
                                func = reinterpret_cast<uintptr_t>(sourceInfo.readMem);
                            else
                                func = reinterpret_cast<uintptr_t>(sourceInfo.readMem8);
                        }
                        else if(instr.opcode == GenOpcode::Load2)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.readMem16);
                        else if(instr.opcode == GenOpcode::Load4)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.readMem32);

                        assert(func);

                        loadLiteral(builder, funcAddrReg, func);

                        builder.blx(funcAddrReg); // do call

                        // move to dest
                        if(dst->mask)
                            write8BitReg(builder, *dst, Reg::R0);
                        else if(instr.opcode == GenOpcode::Load && (instr.flags & GenOp_SignExtend))
                            builder.sxtb(dst->reg, Reg::R0);
                        else if(instr.opcode == GenOpcode::Load2 && (instr.flags & GenOp_SignExtend))
                            builder.sxth(dst->reg, Reg::R0);
                        else
                            builder.mov(dst->reg, Reg::R0);

                        // masked register can't possibly be unmapped
                        if(!dst->mask)
                            storeUnmappedReg(instr.dst[0], dst->reg);

                        // avoid popping reg the result is in
                        // assumes we never push R1-2 or 5+
                        if(dst->reg == Reg::R3 && (pushMask & (1 << 3)))
                        {
                            builder.pop(1 << 0);
                            builder.add(Reg::SP, Reg::SP, 4, false);
                            
                            pushMask &= ~0b1001;
                        }
                        else if(dst->reg == Reg::R4 && (pushMask & (1 << 4)))
                        {
                            builder.pop(pushMask & ~(1 << 4));
                            builder.add(Reg::SP, Reg::SP, 4, false);
                            pushMask = 0;
                        }

                        if(pushMask)
                            builder.pop(pushMask);
                    }
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Store:
            case GenOpcode::Store2:
            case GenOpcode::Store4:
            {
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16 || addrSize == 32)
                {
                    auto addr = checkRegOrImm(instr.src[0], Reg::R12);

                    if(addr.index())
                    {
                        int pushMask = 1 << 4; // R4

                        if(addrSize == 32)
                            pushMask |= 1 << 3 ; // assume that we're using R3

                        builder.push(pushMask, false);

                        // if the address is in a high reg and the last op was an sub to it, the value should still be in R1
                        // this allows us to skip the mov (for stack pushes)
                        bool skipMov = false;
                        if(addrSize == 16)
                        {
                            bool addrIsHighReg = std::holds_alternative<Reg>(addr) && !isLowReg(std::get<Reg>(addr));
                            if(!newEmuOp && addrIsHighReg && instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::Subtract && (instIt - 1)->dst[0] == instr.src[0])
                                skipMov = true;
                        }

                        // the sized version have two additional args
                        bool needCyclesFlags = sourceInfo.readMem8 != nullptr;

                        setupMemAddr(addr, instr.src[0], skipMov);
 
                        if(needCyclesFlags)
                        {
                            // setup 5th and 6th args
                            builder.push(1 << 0); // R0 (cycle count)
                            builder.mov(Reg::R0, instr.flags >> 8);
                            builder.push(1 << 0); // flags

                            // get data (not expecting immediates here)
                            // might get one if storing PC though
                            // TODO: convert that to a literal?
                            auto data = checkReg(instr.src[1], Reg::R2);

                            if(data != Reg::R2)
                                builder.mov(Reg::R2, *data);

                            // cycle count
                            assert(pushMask == 0b11000);
                            writeCycleCountToStack(instrCycles, 16, Reg::R0);
                            builder.add(Reg::R3, Reg::SP, 16, false);
                        }
                        else
                        {
                            auto data = checkRegOrImm8(instr.src[1]);
                            assert(data.index());
                            get8BitValue(builder, Reg::R2, data);
                        
                            builder.mov(Reg::R3, cycleCountReg);
                        }

                        builder.mov(Reg::R0, cpuPtrReg);

                        // get func ptr
                        uintptr_t func = 0;
                        if(instr.opcode == GenOpcode::Store)
                        {
                            if(sourceInfo.readMem)
                                func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem);
                            else
                                func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem8);
                        }
                        else if(instr.opcode == GenOpcode::Store2)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem16);
                        else if(instr.opcode == GenOpcode::Store4)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem32);

                        assert(func);

                        loadLiteral(builder, Reg::R4, func);

                        builder.blx(Reg::R4); // do call

                        if(needCyclesFlags)
                            builder.add(Reg::SP, Reg::SP, 8, false); // space for args

                        // new cycle count is already in R0

                        builder.pop(pushMask, false);
                    }
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Add:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;
            
                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.add(*dst, *src0, imm, true);
                        }
                        else
                            builder.add(*dst, *src0, std::get<Reg>(src1), true);

                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    auto src0 = checkReg(instr.src[0]);
                    auto src1 = checkRegOrImm(instr.src[1]);
                    auto dst = checkReg(instr.dst[0]);
                    auto f = checkReg8(flagsReg);

                    if(src0 && src1.index() && dst && f)
                    {
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.mov(Reg::R3, *dst); // save dst in

                        auto resultReg = *dst;

                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            assert(*src0 == *dst);

                            auto imm = std::get<uint32_t>(src1);
                            assert(imm <= 0xFF || imm >= 0xFF80);

                            if(isLowReg(*dst))
                            {
                                if(imm >= 0xFF80) // LDHL SP (signed 8 bit)
                                    builder.sub(*dst, 0x10000 - imm);
                                else
                                    builder.add(*dst, imm);
                            }
                            else
                            {
                                resultReg = Reg::R1;
                                builder.mov(resultReg, *dst);

                                if(imm >= 0xFF80) // ADD SP (signed 8 bit)
                                    builder.sub(resultReg, 0x10000 - imm);
                                else
                                    builder.add(resultReg, imm);
                            }
                        }
                        else if(*dst == *src0)
                            builder.add(*dst, std::get<Reg>(src1)); // can handle high regs
                        else
                            builder.add(*dst, *src0, std::get<Reg>(src1));

                        // flags
                        // can't use carry from op here
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                        {
                            // res > 0xFFFF == res >> 16 != 0
                            builder.lsr(Reg::R1, resultReg, 16);

                            builder.b(Condition::EQ, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::Carry).bit);
                        }

                        // mask and move
                        builder.uxth(resultReg, resultReg);
                        if(resultReg != *dst)
                            builder.mov(*dst, resultReg);

                        // H = (res & 0xFFF) < (dst & 0xFFFF)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            // & 0xFFF == & ~0xF000
                            load16BitValue(builder, Reg::R2, 0xF000);

                            if(resultReg != Reg::R1)
                                builder.mov(Reg::R1, resultReg);

                            builder.bic(Reg::R1, Reg::R2);
                            builder.bic(Reg::R3, Reg::R2);

                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::GE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        } 
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        get8BitValue(builder, Reg::R1, *dst);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.mov(Reg::R3, Reg::R1); // save dst in

                        if(std::holds_alternative<uint8_t>(src))
                            builder.add(Reg::R1, std::get<uint8_t>(src));
                        else
                        {
                            assert(dst->reg != Reg::R2);
                            get8BitValue(builder, Reg::R2, src);
                            builder.add(Reg::R1, Reg::R1, Reg::R2);
                        }

                        // flags
                        // can't use carry from op here
                        updateC(true);

                        builder.uxtb(Reg::R1, Reg::R1); // mask

                        write8BitReg(builder, *dst, Reg::R1); // may modify R1

                        if(!(instr.flags & GenOp_MagicAlt1)) // LDHL SP/ADD SP use an 8-bit add to set flags, but set Z=0
                            updateZ(true);

                        // H = (res & 0xF) < (dst & 0xF)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            builder.mov(Reg::R2, 0xF);

                            if(dst->mask == 0xFF00) // undo shift from write
                                builder.lsr(Reg::R1, Reg::R1, 8);

                            builder.and_(Reg::R1, Reg::R2);

                            builder.and_(Reg::R3, Reg::R2);
                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::GE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        } 
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::AddWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;
            
                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        carryIn32();

                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.adc(*dst, *src0, imm, true);
                        }
                        else
                            builder.adc(*dst, *src0, std::get<Reg>(src1), true);

                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src && dst && f)
                    {
                        get8BitValue(builder, Reg::R1, *dst);

                        assert(dst->reg != Reg::R2);
                        get8BitValue(builder, Reg::R2, *src);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.push(0b1010, false); // save dst in and carry

                        builder.cmp(Reg::R3, 1); // set C

                        builder.adc(Reg::R1, Reg::R2);

                        // flags
                        // can't use carry from op here
                        updateC(true);

                        builder.uxtb(Reg::R1, Reg::R1); // mask

                        write8BitReg(builder, *dst, Reg::R1); // may modify R1

                        updateZ(true);

                        // H = (res & 0xF) < ((dst & 0xF) + C)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            builder.mov(Reg::R2, 0xF);

                            if(dst->mask == 0xFF00) // undo shift from write
                                builder.lsr(Reg::R1, Reg::R1, 8);

                            builder.and_(Reg::R1, Reg::R2);

                            // restore dst -> R3
                            builder.pop(1 << 3, false);
                            builder.and_(Reg::R3, Reg::R2);

                            // restore C -> R2
                            builder.pop(1 << 2, false);

                            // add C
                            builder.lsr(Reg::R2, Reg::R2, getFlagInfo(SourceFlagType::Carry).bit);
                            builder.add(Reg::R3, Reg::R2);
                            
                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::GE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        } 
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::And:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            
                            if(builder.isValidModifiedImmediate(imm))
                                builder.and_(*dst, *src0, imm, true);
                            else
                            {
                                // handle a few more cases (usually big masks)
                                assert(builder.isValidModifiedImmediate(~imm));
                                builder.bic(*dst, *src0, ~imm, true);
                            }
                        }
                        else
                            builder.and_(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src1 && dst)
                    {
                        // TODO: can optimise a bit for A reg
                        assert(src0->reg != Reg::R2);
                        get8BitValue(builder, Reg::R1, *src0);
                        get8BitValue(builder, Reg::R2, *src1);

                        builder.and_(Reg::R1, Reg::R2);

                        write8BitReg(builder, *dst, Reg::R1);

                        updateZ(false);
                        setFlag(SourceFlagType::HalfCarry);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Compare:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);

                    if(src0 && src1.index())
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.cmp(*src0, imm);
                        }
                        else
                            builder.cmp(*src0, std::get<Reg>(src1));

                        setFlags32(builder, instr.flags);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        get8BitValue(builder, Reg::R1, *dst);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.mov(Reg::R3, Reg::R1); // save dst in

                        if(std::holds_alternative<uint8_t>(src))
                            builder.sub(Reg::R1, std::get<uint8_t>(src));
                        else
                        {
                            assert(dst->reg != Reg::R2);
                            get8BitValue(builder, Reg::R2, src);
                            builder.sub(Reg::R1, Reg::R1, Reg::R2);
                        }

                        builder.uxtb(Reg::R1, Reg::R1);

                        // flags
                        updateC(false, true);
                        updateZ(true);

                        // H = (res & 0xF) > (dst & 0xF)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            builder.mov(Reg::R2, 0xF);
                            builder.and_(Reg::R1, Reg::R2);

                            builder.and_(Reg::R3, Reg::R2);
                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::LE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        } 

                        setFlag(SourceFlagType::WasSub);
                    }
                }
                else
                    badRegSize(regSize);
    
                break;
            }

            case GenOpcode::Multiply:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkReg(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1 && dst)
                    {
                        builder.mul(*dst, *src0, *src1);

                        // thumb2 MUL doesn't set flags, so do a MOV to get Z/N
                        if(instr.flags & GenOp_WriteFlags)
                            builder.mov(*dst, *dst, true);

                        // TODO: technically does write C on AGB, but we don't handle this properly in AGBCPU either
                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else
                    badRegSize(regSize);
    
                break;
            }

            case GenOpcode::Or:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.orr(*dst, *src0, imm, true);
                        }
                        else
                            builder.orr(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    // used for LDH, RET
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src0 = checkReg(instr.src[0]);
                        auto src1 = checkReg(instr.src[1]);
                        auto dst = checkReg(instr.dst[0]);

                        if(src0 && src1 && dst)
                        {
                            auto origDest = *dst;

                            // the high temp ends up getting used as src and dst...
                            if(!isLowReg(*src1))
                            {
                                builder.mov(Reg::R1, *src1);
                                src1 = Reg::R1;
                            }
                            else if(!isLowReg(*dst))
                                *dst = Reg::R1;

                            if(*dst != *src0)
                                builder.mov(*dst, *src0);

                            builder.orr(*dst, *src1);

                            if(*dst != origDest) // we moved it
                                builder.mov(origDest, *dst);
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src1 && dst)
                    {
                        assert(src0->reg != Reg::R2);
                        get8BitValue(builder, Reg::R1, *src0);
                        get8BitValue(builder, Reg::R2, *src1);

                        builder.orr(Reg::R1, Reg::R2);

                        write8BitReg(builder, *dst, Reg::R1);

                        updateZ(false);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Subtract:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;
            
                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.sub(*dst, *src0, imm, true);
                        }
                        else
                            builder.sub(*dst, *src0, std::get<Reg>(src1), true);

                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    // should be DEC, which sets no flags
                    if(instr.flags & (GenOp_PreserveFlags | GenOp_WriteFlags))
                        unhandledFlags(instr.flags & (GenOp_PreserveFlags | GenOp_WriteFlags));
                    else
                    {
                        auto src0 = checkReg(instr.src[0]);
                        auto src1 = checkRegOrImm(instr.src[1]);
                        auto dst = checkReg(instr.dst[0]);

                        if(src0 && src1.index() && dst)
                        {
                            assert(std::holds_alternative<uint32_t>(src1) && *src0 == *dst);

                            auto imm = std::get<uint32_t>(src1);
                            assert(imm <= 0xFF);

                            if(isLowReg(*dst))
                            {
                                builder.sub(*dst, imm);
                                builder.uxth(*dst, *dst);
                            }
                            else
                            {
                                auto r = Reg::R1;
                                builder.mov(r, *dst);
                                builder.sub(r, 1);
                                builder.uxth(r, r);
                                builder.mov(*dst, r);
                            }
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        get8BitValue(builder, Reg::R1, *dst);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.mov(Reg::R3, Reg::R1); // save dst in

                        if(std::holds_alternative<uint8_t>(src))
                            builder.sub(Reg::R1, std::get<uint8_t>(src));
                        else
                        {
                            assert(dst->reg != Reg::R2);
                            get8BitValue(builder, Reg::R2, src);
                            builder.sub(Reg::R1, Reg::R1, Reg::R2);
                        }

                        builder.uxtb(Reg::R1, Reg::R1);

                        // flags
                        updateC(false, true);

                        write8BitReg(builder, *dst, Reg::R1); // may modify R1

                        updateZ(true);

                        // H = (res & 0xF) > (dst & 0xF)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            builder.mov(Reg::R2, 0xF);

                            if(dst->mask == 0xFF00) // undo shift from write
                                builder.lsr(Reg::R1, Reg::R1, 8);

                            builder.and_(Reg::R1, Reg::R2);

                            builder.and_(Reg::R3, Reg::R2);
                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::LE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        } 

                        setFlag(SourceFlagType::WasSub);
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::SubtractWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;
            
                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        // this is assuming arm "not borrow" logic for carry...
                        carryIn32();

                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.sbc(*dst, *src0, imm, true);
                        }
                        else
                            builder.sbc(*dst, *src0, std::get<Reg>(src1), true);

                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src && dst && f)
                    {
                        get8BitValue(builder, Reg::R1, *dst);

                        assert(dst->reg != Reg::R2);
                        get8BitValue(builder, Reg::R2, *src);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            builder.push(0b1010, false); // save dst in and carry

                        // need to invert C
                        builder.mvn(Reg::R3, Reg::R3);
                        // but now all the other bits are set, so use a shift
                        builder.lsr(Reg::R3, Reg::R3, getFlagInfo(SourceFlagType::Carry).bit + 1);

                        builder.sbc(Reg::R1, Reg::R2);
                        builder.uxtb(Reg::R1, Reg::R1);

                        // flags
                        updateC(false, true);

                        write8BitReg(builder, *dst, Reg::R1); // may modify R1

                        updateZ(true);

                        // H = (res & 0xF) > ((dst & 0xF) - C)
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            builder.mov(Reg::R2, 0xF);

                            if(dst->mask == 0xFF00) // undo shift from write
                                builder.lsr(Reg::R1, Reg::R1, 8);

                            builder.and_(Reg::R1, Reg::R2);

                            // restore dst -> R3
                            builder.pop(1 << 3, false);
                            builder.and_(Reg::R3, Reg::R2);

                            // restore C -> R2
                            builder.pop(1 << 2, false);

                            // sub C
                            builder.lsr(Reg::R2, Reg::R2, getFlagInfo(SourceFlagType::Carry).bit);
                            builder.sub(Reg::R3, Reg::R3, Reg::R2);
                            
                            builder.cmp(Reg::R1, Reg::R3);

                            builder.b(Condition::LE, 2);
                            builder.add(f->reg, 1 << getFlagInfo(SourceFlagType::HalfCarry).bit);
                        }

                        setFlag(SourceFlagType::WasSub);
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::Xor:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            assert(builder.isValidModifiedImmediate(imm));
                            builder.eor(*dst, *src0, imm, true);
                        }
                        else
                            builder.eor(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src1 && dst)
                    {
                        if(*src0 == *src1 && dst->reg == mapReg8(flagsReg)->reg)
                        {
                            // DMG XOR A special case, A = A ^ A with flags in the same 16-bit reg
                            // we don't need the flag clear code either, that's a little harder to avoid
                            // A = 0, F = Z
                            builder.mov(dst->reg, 1 << getFlagInfo(SourceFlagType::Zero).bit);
                        }
                        else
                        {
                            assert(src0->reg != Reg::R2);
                            get8BitValue(builder, Reg::R1, *src0);
                            get8BitValue(builder, Reg::R2, *src1);

                            builder.eor(Reg::R1, Reg::R2);

                            write8BitReg(builder, *dst, Reg::R1);

                            updateZ(false);
                        }
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Not:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    // shouldn't get these with immediate src
                    auto src = checkReg(instr.src[0], Reg::R2);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src && dst)
                        builder.mvn(*dst, *src, true);

                    assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                    assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                    setFlags32(builder, instr.flags);

                    storeUnmappedReg(instr.dst[0], *dst);
                }
                else if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);
                    if(src && dst)
                    {
                        assert(src->reg == dst->reg && src->mask == dst->mask);
                        assert(dst->mask);

                        // xor by reg mask
                        load16BitValue(builder, Reg::R1, dst->mask);
                        builder.eor(dst->reg, Reg::R1);

                        // flags (sets H and N to 1)
                        if((instr.flags & GenOp_WriteFlags))
                            builder.add(mapReg8(flagsReg)->reg, (1 << getFlagInfo(SourceFlagType::HalfCarry).bit) | (1 << getFlagInfo(SourceFlagType::WasSub).bit));
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1); // RLCA sets Z to 0, RLC A sets Z based on the result
                    bool setC = !(instr.flags & GenOp_MagicAlt2); // SWAP sets C to 0 (translated to rot by 4)

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);

                            // v << n | v >> (8 - n)
                            builder.lsr(Reg::R2, Reg::R1, 8 - std::get<uint8_t>(src1));
                            builder.lsl(Reg::R1, Reg::R1, std::get<uint8_t>(src1));
                            builder.orr(Reg::R1, Reg::R2);

                            // carry
                            if(setC)
                                updateC(true);

                            builder.uxtb(Reg::R1, Reg::R1);

                            // zero
                            if(setZ)
                                updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit rotate by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeftCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);

                            assert(std::get<uint8_t>(src1) == 1);

                            builder.lsl(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry in
                            builder.lsr(Reg::R3, Reg::R3, getFlagInfo(SourceFlagType::Carry).bit);
                            builder.orr(Reg::R1, Reg::R3);

                            // carry
                            updateC(true);

                            builder.uxtb(Reg::R1, Reg::R1);

                            // zero
                            if(setZ)
                                updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit rotate by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRight:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    // might need to preserve carry
                    if(writesFlag(instr.flags, SourceFlagType::Carry))
                        carryIn32();

                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            builder.ror(*dst, *src0, imm, true);
                        }
                        else
                            builder.ror(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);

                            // v << (8 - n) | v >> n
                            builder.lsl(Reg::R2, Reg::R1, 8 - std::get<uint8_t>(src1));
                            builder.lsr(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry (set before or)
                            updateC(false);

                            builder.orr(Reg::R1, Reg::R2);
                            builder.uxtb(Reg::R1, Reg::R1);

                            // zero
                            if(setZ)
                                updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit rotate by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRightCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);

                            assert(std::get<uint8_t>(src1) == 1);

                            builder.lsr(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry out
                            updateC(false);

                            // carry in
                            builder.lsl(Reg::R3, Reg::R3, 7 - getFlagInfo(SourceFlagType::Carry).bit);
                            builder.orr(Reg::R1, Reg::R3);

                            // zero
                            if(setZ)
                                updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit rotate by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        // might need to preserve carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            carryIn32();

                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            builder.lsl(*dst, *src0, imm, true);
                        }
                        else
                            builder.lsl(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    // used for RET
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src0 = checkReg(instr.src[0]);
                        auto src1 = checkRegOrImm8(instr.src[1]);
                        auto dst = checkReg(instr.dst[0]);

                        if(src0 && src0 && src1.index())
                        {
                            if(std::holds_alternative<uint8_t>(src1))
                                builder.lsl(*dst, *src0, std::get<uint8_t>(src1)); // should uxth, but the only users of this don't need it
                            else
                                assert(!"16-bit shift by reg");
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);
                            builder.lsl(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry
                            updateC(true);

                            builder.uxtb(Reg::R1, Reg::R1);

                            // zero
                            updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit shift by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightArith:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    // might need to preserve carry
                    if(writesFlag(instr.flags, SourceFlagType::Carry))
                        carryIn32();

                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            builder.asr(*dst, *src0, imm, true);
                        }
                        else
                            builder.asr(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }

                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);

                            builder.sxtb(Reg::R1, Reg::R1); // TODO: can avoid this in some cases be replacing the uxtb in get8BitValue
                            builder.asr(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry
                            updateC(false);

                            builder.uxtb(Reg::R1, Reg::R1);

                            // zero
                            updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit shift by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightLogic:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    // might need to preserve carry
                    if(writesFlag(instr.flags, SourceFlagType::Carry))
                        carryIn32();

                    auto src0 = checkReg(instr.src[0], Reg::R12);
                    auto src1 = checkRegOrImm(instr.src[1], Reg::R14);
                    auto dst = checkReg(instr.dst[0], Reg::R2, true);

                    if(src0 && src1.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src1))
                        {
                            auto imm = std::get<uint32_t>(src1);
                            builder.lsr(*dst, *src0, imm, true);
                        }
                        else
                            builder.lsr(*dst, *src0, std::get<Reg>(src1), true);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(builder, instr.flags);

                        storeUnmappedReg(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    // used for LD (nn), SP
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src0 = checkReg(instr.src[0]);
                        auto src1 = checkRegOrImm8(instr.src[1]);
                        auto dst = checkReg(instr.dst[0]);

                        if(src0 && src0 && src1.index())
                        {
                            auto resReg = *dst;
                            if(!isLowReg(*src0))
                            {
                                builder.mov(Reg::R1, *src0);
                                src0 = Reg::R1;
                            }

                            if(!isLowReg(*dst))
                                resReg = Reg::R1;

                            if(std::holds_alternative<uint8_t>(src1))
                                builder.lsr(resReg, *src0, std::get<uint8_t>(src1)); // should uxth, but the only users of this don't need it
                            else
                                assert(!"16-bit shift by reg");

                            if(resReg != *dst)
                                builder.mov(*dst, resReg);
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src0 = checkReg8(instr.src[0]);
                    auto src1 = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src0 && src0 && src1.index())
                    {
                        if(std::holds_alternative<uint8_t>(src1))
                        {
                            get8BitValue(builder, Reg::R1, *src0);
                            builder.lsr(Reg::R1, Reg::R1, std::get<uint8_t>(src1));

                            // carry
                            updateC(false);

                            // zero
                            updateZ(true);

                            // write back
                            write8BitReg(builder, *dst, Reg::R1);
                        }
                        else
                            assert(!"8-bit shift by reg");
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Jump:
            {
                auto condition = static_cast<GenCondition>(instr.src[0]);
                auto regSize = sourceInfo.registers[instr.src[1]].size;

                bool isExit = instr.flags & GenOp_Exit;

                if(regSize == 16)
                {
                    auto src = checkRegOrImm(instr.src[1]);
                    if(src.index())
                    {
                        assert(isExit || std::holds_alternative<uint32_t>(src)); // shouldn't be any non-exit jumps with unknown addr

                        uint16_t *branchPtr = nullptr;
                        Condition nativeCond = Condition::EQ;

                        // condition
                        if(condition != GenCondition::Always)
                        {
                            bool flagSet = false;

                            syncCyclesExecuted();

                            flagSet = !(static_cast<int>(condition) & 1); // even conds are set, odd are clear

                            auto f = regSize == 32 ? *checkReg(flagsReg) : checkReg8(flagsReg)->reg;

                            switch(condition)
                            {
                                case GenCondition::Equal:
                                case GenCondition::NotEqual:
                                    builder.tst(f, 1 << getFlagInfo(SourceFlagType::Zero).bit);
                                    break;
                                case GenCondition::CarrySet:
                                case GenCondition::CarryClear:
                                    builder.tst(f, 1 << getFlagInfo(SourceFlagType::Carry).bit);
                                    break;
                                default:
                                    printf("unhandled cond %i\n", static_cast<int>(condition));
                                    err = true;
                            }

                            branchPtr = builder.getPtr();
                            builder.b(Condition::EQ, 0); // patch later

                            nativeCond = flagSet ? Condition::EQ : Condition::NE;
                        }

                        // need to sync cycles *before* the jump out
                        if(instrCycles)
                        {
                            assert(instrCycles == 1 || !(blockInfo.flags & GenBlock_StrictSync));
                            while(instrCycles--)
                                cycleExecuted();
                        }
                        syncCyclesExecuted();


                        // set pc if we're exiting
                        // ... or always as we might not be able to patch the branch
                        if(std::holds_alternative<uint32_t>(src))
                            loadPCValue(builder, std::get<uint32_t>(src));
                        else if(sourceInfo.pcPrefetch)
                            builder.add(pcReg, std::get<Reg>(src), sourceInfo.pcPrefetch, true);
                        else if(std::get<Reg>(src) != pcReg)
                            builder.mov(pcReg, std::get<Reg>(src));
                    
                        if(!isExit)
                            builder.sub(cycleCountReg, cyclesThisInstr);

                        // don't update twice for unconditional branches
                        if(condition == GenCondition::Always)
                            cyclesThisInstr = 0;

                        auto it = std::holds_alternative<uint32_t>(src) ? branchTargets.find(std::get<uint32_t>(src)) : branchTargets.end();

                        if(it != branchTargets.end())
                        {
                            // backwards branch
                            int off = (it->second - builder.getPtr()) * 2;
                            if(off >= -2044)
                                builder.b(off);
                            else
                                builder.bl((exitPtr - builder.getPtr()) * 2); // too far, give up
                        }
                        else
                        {
                            if(!isExit)
                                forwardBranchesToPatch.emplace(std::get<uint32_t>(src), builder.getPtr());

                            // forwards branch or exit
                            if(isExit && (instr.flags & GenOp_Call) && instIt + 1 != endInstr)
                                builder.bl((exitForCallPtr - builder.getPtr()) * 2); // call, save when exiting
                            else
                                builder.bl((exitPtr - builder.getPtr()) * 2); // patched later if not exit
                        }

                        // patch the condition jump
                        if(branchPtr && !builder.getError())
                        {
                            int off = (builder.getPtr() - (branchPtr + 1)) * 2;
                            builder.patch(branchPtr, branchPtr + 2);
                            builder.b(nativeCond, off);
                            builder.endPatch();
                        }
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            // DMG specific
            case GenOpcode::DMG_DAA:
            {
                auto f = *mapReg8(flagsReg);
                auto a = Reg::R2;

                assert(f.reg == Reg::R4 && f.mask == 0xFF);

                auto cMask = 1 << getFlagInfo(SourceFlagType::Carry).bit;
                auto hMask = 1 << getFlagInfo(SourceFlagType::HalfCarry).bit;
                auto nMask = 1 << getFlagInfo(SourceFlagType::WasSub).bit;
                auto zMask = 1 << getFlagInfo(SourceFlagType::Zero).bit;

                builder.lsr(a, f.reg, 8); // get A

                // clear Z flag (will add back later)
                builder.mov(Reg::R1, zMask);
                builder.bic(f.reg, Reg::R1);

                builder.mov(Reg::R1, nMask);
                builder.and_(Reg::R1, f.reg); // tst?
                builder.b(Condition::EQ, 18); // N not set

                // negative

                // if (Flag_C) A -= 0x60
                builder.mov(Reg::R1, cMask);
                builder.and_(Reg::R1, f.reg); // tst?
                builder.b(Condition::EQ, 2); // C not set
                builder.sub(a, 0x60);

                // if (Flag_H) A -= 0x06
                builder.mov(Reg::R1, hMask);
                builder.and_(Reg::R1, f.reg); // tst?
                builder.b(Condition::EQ, 2); // C not set
                builder.sub(a, 0x06);

                builder.b(32); // skip

                // positive

                // if (Flag_C ...
                builder.mov(Reg::R1, cMask);
                builder.and_(Reg::R1, f.reg); // tst?
                builder.b(Condition::NE, 4); // C set
                // ... || A > 0x99) ...
                builder.cmp(a, 0x99);
                builder.b(Condition::LE, 6);
                // ... A += 0x60
                builder.add(a, 0x60);
                builder.mov(Reg::R1, cMask);
                builder.orr(f.reg, Reg::R1); // set C flag

                // if (Flag_H ...
                builder.mov(Reg::R1, hMask);
                builder.and_(Reg::R1, f.reg); // tst?
                builder.b(Condition::NE, 8); // H set
                // ... || A & (0x0F) > 0x09) ...
                builder.mov(Reg::R1, 0xF);
                builder.and_(Reg::R1, a);
                builder.cmp(Reg::R1, 0x09);
                builder.b(Condition::LE, 2);
                // .. A += 0x06
                builder.add(a, 0x06);

                builder.uxtb(a, a);

                // Z flag
                builder.cmp(a, 0);
                builder.b(Condition::NE, 4);
                builder.mov(Reg::R1, zMask);
                builder.orr(f.reg, Reg::R1);

                // clear H flag
                builder.mov(Reg::R1, hMask);
                builder.bic(f.reg, Reg::R1);

                // write A back
                write8BitReg(builder, *mapReg8(6/*A*/), a);

                break;
            }

            case GenOpcode::DMG_Halt:
            {
                auto haltedOff = sourceInfo.extraCPUOffsets[2];
                auto masterInterruptEnableOff = sourceInfo.extraCPUOffsets[0];
                auto serviceableInterruptsOff = sourceInfo.extraCPUOffsets[3];
                auto haltBugOff = sourceInfo.extraCPUOffsets[4];
                assert(haltedOff < 32);
                assert(masterInterruptEnableOff < 32);
                assert(serviceableInterruptsOff < 32);
                assert(haltBugOff < 32);

                builder.mov(Reg::R2, cpuPtrReg);

                // halted = true
                builder.mov(Reg::R1, 1);
                builder.strb(Reg::R1, Reg::R2, haltedOff);

                // if(!masterInterruptEnable && serviceableInterrupts)
                builder.ldrb(Reg::R1, Reg::R2, masterInterruptEnableOff);
                builder.cmp(Reg::R1, 0);
                builder.b(Condition::NE, 10);
                builder.ldrb(Reg::R1, Reg::R2, serviceableInterruptsOff);
                builder.cmp(Reg::R1, 0);
                builder.b(Condition::EQ, 4);
                // haltBug = true
                builder.mov(Reg::R1, 1);
                builder.strb(Reg::R1, Reg::R2, haltBugOff);

                // exit
                syncCyclesExecuted();
                load16BitValue(builder, Reg::R1, pc); // exits need to set PC themselves
                builder.bl((saveAndExitPtr - builder.getPtr()) * 2);

                break;
            }
        
            case GenOpcode::DMG_EnableIntrForRet:
                assert(sourceInfo.extraCPUOffsets[0] < 32);
                // masterInterruptEnable = true
                builder.mov(Reg::R2, cpuPtrReg);
                builder.mov(Reg::R1, 1);
                builder.strb(Reg::R1, Reg::R2, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/);
                break;

            case GenOpcode::DMG_DI:
                assert(sourceInfo.extraCPUOffsets[0] < 32);
                // masterInterruptEnable = false
                // TODO: after next instruction (DMGCPU also has this TODO)
                builder.mov(Reg::R2, cpuPtrReg);
                builder.mov(Reg::R1, 0);
                builder.strb(Reg::R1, Reg::R2, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/);
                break;

            case GenOpcode::DMG_EI:
                assert(sourceInfo.extraCPUOffsets[1] < 32);
                // enableInterruptsNextCycle = true
                builder.mov(Reg::R2, cpuPtrReg);
                builder.mov(Reg::R1, 1);
                builder.strb(Reg::R1, Reg::R2, sourceInfo.extraCPUOffsets[1]/*enableInterruptsNextCycle*/);
                lastWasEI = true;
                break;

            default:
                printf("unhandled gen op %i\n", static_cast<int>(instr.opcode));
                err = true;
        }

        if(builder.getError())
            break;

        // failed but builder still ok
        if(err)
        {
            builder.resetPtr(opStartPtr);

            // remove any literal loads we just un-wrote
            for(auto it = ldrLiteralInstrs.begin(); it != ldrLiteralInstrs.end();)
            {
                if(*it >= builder.getPtr())
                    it = ldrLiteralInstrs.erase(it);
                else
                    ++it;
            }

            loadPCValue(builder, pc - instr.len);
            builder.bl((exitPtr - builder.getPtr()) * 2);
            outputLiterals(builder, false);
            break;
        }

        // sync cycle count from stack at the end of the op, or if something updates cycles before that
        if(stackCycleCount && (instrCycles || instr.len || (instIt + 1)->opcode == GenOpcode::Jump))
        {
            // add/sub the cycles from the stack
            builder.ldr(Reg::R1, Reg::SP, 0); // should be at top of stack

            int offset = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - reinterpret_cast<uintptr_t>(cpuPtr);
            builder.ldr(Reg::R12, cpuPtrReg, offset);
            builder.add(Reg::R12, Reg::R1);
            builder.str(Reg::R12, cpuPtrReg, offset);

            builder.sub(Reg::R0, Reg::R0, Reg::R1);
            stackCycleCount = false;
        }

        // additional cycles
        if(instrCycles > 0)
        {
            assert(instrCycles == 1 || !(blockInfo.flags & GenBlock_StrictSync));
            while(instrCycles--)
                cycleExecuted();
        }

        // check if this is the end of the source instruction (pc incremented)
        newEmuOp = instr.len != 0;

        if(instIt + 1 == endInstr && (instr.opcode != GenOpcode::Jump || static_cast<GenCondition>(instr.src[0]) != GenCondition::Always))
        {
            // ending on a non-jump probably means this was an incomplete block
            // add an exit
            syncCyclesExecuted();
            loadPCValue(builder, pc);
            builder.bl((exitPtr - builder.getPtr()) * 2);
        }

        // check cycle count if this is the last part of en emulated op
        // ... but not on the last op, that should always exit anyway
        // ... or exits unless followed by a branch target
        auto nextInstr = instIt + 1;
        bool isCond = instr.opcode == GenOpcode::Jump && static_cast<GenCondition>(instr.src[0]) != GenCondition::Always;
        bool shouldSkip = nextInstr == endInstr || ((instr.flags & GenOp_Exit) && !isCond && !(nextInstr->flags & GenOp_BranchTarget));
        if(newEmuOp && !shouldSkip)
        {
            // might exit, sync
            syncCyclesExecuted();

            // exit may be forced after interrupts are enabled
            if(forceExitAfter)
               builder.b(cyclesThisInstr ? 4 : 2); // jump over the condition

            // cycles -= executed
            if(cyclesThisInstr) // 0 means we already did the sub
                builder.sub(cycleCountReg, cyclesThisInstr);

            lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

            // if <= 0 exit
            branchOver(builder, [this, pc](ThumbBuilder &builder)
            {
                loadPCValue(builder, pc);
                builder.bl((saveAndExitPtr - builder.getPtr()) * 2);
            }, Condition::GT);

            cyclesThisInstr = 0;
            forceExitAfter = false;
        }

        if(newEmuOp)
            numInstructions++;

        // output literals if ~close to the limit or this is the last instruction
        // this was definitely NOT set by decreasing until it didn't abort...
        // don't output after a LoadImm as it might get removed by the next op
        const int minLiterals = 3; // load/store can use 2 + possibly one from esit
        if(instr.opcode != GenOpcode::LoadImm && !ldrLiteralInstrs.empty() && (nextInstr == endInstr || builder.getPtr() - ldrLiteralInstrs[0] >= 440 || curLiteral + minLiterals > std::size(literals)))
        {
            // calls might return here
            bool exited = shouldSkip && !(instr.flags & GenOp_Call);
            outputLiterals(builder, !exited);
        }
    }

    if(builder.getError())
    {
        ldrLiteralInstrs.clear();
        printf("recompile @%04X failed due to error (out of space?)\n", startPC);
        return false;
    }

    if(numInstructions == 0)
    {
#ifdef RECOMPILER_DEBUG
        printf("recompile @%04X failed to handle any instructions\n", startPC);
#endif
        return false;
    }

    auto endPtr = builder.getPtr();

#ifdef RECOMPILER_DEBUG
    int len = endPtr - codePtr16;

    //debug
    printf("recompile @%04X generated %i halfwords (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr16; p != endPtr; p++)
        printf(" %02X %02X", *p & 0xFF, *p >> 8);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);
#endif

    // need to clear the cache
    __builtin___clear_cache(codePtr, endPtr);

    codePtr = reinterpret_cast<uint8_t *>(endPtr);

    return true;
}

uint8_t *ThumbTarget::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

    // push/reserve stack space
    int extraStack = 0;

    if(sourceInfo.readMem8)
        extraStack += 4;

    builder.push(0b0100111111110000); // R4-11, LR

    if(extraStack)
        builder.sub(Reg::SP, Reg::SP, extraStack, false);
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    auto cpuPtrLoadPtr = builder.getPtr();
    builder.ldr(Reg::R2, 64); // patched later
    builder.mov(cpuPtrReg, Reg::R2);

    // load emu regs
    uint16_t firstRegOff = 0xFFFF;

    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            if(auto mappedReg = mapReg(i))
            {
                // make offsets smaller
                // assumes first reg is at lowest addr
                if(reg.cpuOffset < firstRegOff)
                {
                    firstRegOff = reg.cpuOffset;
                    if(firstRegOff)
                        builder.add(Reg::R2, firstRegOff); // add to cpu ptr
                }

                if(reg.size == 32)
                    builder.ldr(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
                else
                    builder.ldrh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
            }
        }

        i++;
    }

    builder.bx(Reg::R1);

    // exit setting the call flag ... and saving LR
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg::R0, 1);

    auto callFlagPtrLoadPtr = builder.getPtr();
    builder.ldr(Reg::R2, 0); // patched later
    builder.strb(Reg::R0, Reg::R2, 0);

    // exit saving LR
    saveAndExitPtr = builder.getPtr();

    builder.mov(Reg::R0, Reg::LR);

    auto exitPtrLoadPtr = builder.getPtr();
    builder.ldr(Reg::R2, 0); // patched later
    builder.str(Reg::R0, Reg::R2, 0);

    // exit
    exitPtr = builder.getPtr();

    // store PC
    int pcOff = sourceInfo.pcOffset;
    assert(pcOff <= 0xFF);
    builder.mov(Reg::R2, cpuPtrReg); // cpu ptr
    builder.mov(Reg::R0, pcOff);

    if(sourceInfo.pcSize == 32)
        builder.str(pcReg, Reg::R2, Reg::R0);
    else
        builder.strh(pcReg, Reg::R2, Reg::R0);

    // save emu regs
    if(firstRegOff)
        builder.add(Reg::R2, firstRegOff); // add to cpu ptr

    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            if(auto mappedReg = mapReg(i))
            {
                if(reg.size == 32)
                    builder.str(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
                else
                    builder.strh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
            }
        }

        i++;
    }


    // restore regs and return
    if(extraStack)
        builder.add(Reg::SP, Reg::SP, extraStack, false);

    builder.pop(0b1000111111110000); // R4-11, PC

    // write cpu addr
    auto ptr = builder.getPtr();

    if((ptr - codePtr16) & 1)
        *ptr++ = 0; // align

    auto patchLoad = [&builder](uint16_t *curPtr, uint16_t *loadPtr)
    {
        auto aligned = (reinterpret_cast<uintptr_t>(loadPtr) + 4) & ~2;
        int offset = reinterpret_cast<uintptr_t>(curPtr) - aligned;
        builder.patch(loadPtr, loadPtr + 1);
        builder.ldr(Reg::R2, offset);
        builder.endPatch();
    };

    patchLoad(ptr, cpuPtrLoadPtr);
    *ptr++ = cpuPtrInt;
    *ptr++ = cpuPtrInt >> 16;

    // write addr of exitCallFlag
    patchLoad(ptr, callFlagPtrLoadPtr);
    auto addr = reinterpret_cast<uintptr_t>(sourceInfo.exitCallFlag);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    // write addr of tmpSavedPtr
    patchLoad(ptr, exitPtrLoadPtr);
    addr = reinterpret_cast<uintptr_t>(sourceInfo.savedExitPtr);
    *ptr++ = addr;
    *ptr++ = addr >> 16;


#ifdef RECOMPILER_DEBUG
    int len = ptr - codePtr16;

    //debug
    printf("generated %i halfwords for entry/exit\ncode:", len);

    for(auto p = codeBuf; p != codeBuf + len * 2; p++)
        printf(" %02X", *p);

    printf("\n");
#endif

    __builtin___clear_cache(codeBuf, ptr);

    auto ret = codeBuf + 1; // thumb bit

    codeBuf = reinterpret_cast<uint8_t *>(ptr);

    return ret;
}

void ThumbTarget::loadLiteral(ThumbBuilder &builder, Reg reg, uint32_t val)
{
    // TODO: AGBRecompiler may have improved literal code...
    unsigned int index;
    for(index = 0; index < curLiteral; index++)
    {
        if(literals[index] == val)
            break;
    }

    // use a MOV if the value will fit... and we would be using a 32bit instruction anyway
    if(!isLowReg(reg) && (builder.isValidModifiedImmediate(val) || val <= 0xFFFF))
    {
        builder.mov(reg, val);
        return;
    }

    if(index == curLiteral)
    {
        assert(curLiteral < std::size(literals));
        literals[curLiteral++] = val;
    }

    ldrLiteralInstrs.push_back(builder.getPtr());
    builder.ldr(reg, index << 2); //write literal index, patched later
}

void ThumbTarget::outputLiterals(ThumbBuilder &builder, bool reachable)
{
    if(!ldrLiteralInstrs.empty())
    {
        auto dataPtr = builder.getPtr();

        if(reachable)
        {
            // jump over the literals
            dataPtr++;
            if(reinterpret_cast<uintptr_t>(dataPtr) & 2)
            {
                // not aligned
                builder.b(curLiteral * 4 + 2);
                builder.data(0);
                dataPtr++;
            }
            else // aligned
                builder.b(curLiteral * 4);
        }
        else if(reinterpret_cast<uintptr_t>(dataPtr) & 2)
        {
            builder.nop(); // align
            dataPtr++;
        }

        for(unsigned int i = 0; i < curLiteral; i++)
        {
            builder.data(literals[i]);
            builder.data(literals[i] >> 16);
        }

        // ran out of space
        if(builder.getError())
            return;

        // patch
        for(auto &ptr : ldrLiteralInstrs)
        {
            bool t2 = (*ptr & 0xFF7F) == 0xF85F;

            auto start = ptr + 1;
            if(reinterpret_cast<uintptr_t>(start) & 2)
                start++;

            // get back the index (we encoded it in the immediate value)
            unsigned index;

            if(t2)
                index = (*(ptr + 1) & 0xFFF) >> 2;
            else
                index = *ptr & 0xFF;

            // update LDR imm
            auto off = ((dataPtr + index * 2) - start);
            assert(index < std::size(literals));

            if(t2)
            {
                off <<= 1;
                assert(off < 0xFFF);
                ptr++;
                *ptr = (*ptr & 0xF000) | off;
            }
            else
            {
                off >>= 1;
                assert(off <= 0xFF);
                *ptr = (*ptr & 0xFF00) | off;
            }
        }
    }

    // reset
    ldrLiteralInstrs.clear();
    curLiteral = 0;

    for(auto &literal : literals)
        literal = 0;
}

void ThumbTarget::loadPCValue(ThumbBuilder &builder, uint32_t val)
{
    val += sourceInfo.pcPrefetch;

    if(sourceInfo.pcSize == 16 || !(val >> 16))
        load16BitValue(builder, pcReg, val);
    else
    {
        // look for an old literal to reuse
        // an add is half the size of a new literal
        // also we don't want to be emitting a literal for every instruction 
        uint32_t oldLiteral = 0;
        for(unsigned index = 0; index < curLiteral; index++)
        {
            if(val - literals[index] <= 0xFF)
            {
                oldLiteral = literals[index];
                break;
            }
        }

        if(oldLiteral)
        {
            loadLiteral(builder, pcReg, oldLiteral);
            builder.add(pcReg, val - oldLiteral);
        }
        else
            loadLiteral(builder, pcReg, val);
    }
}

void ThumbTarget::setFlags32(ThumbBuilder &builder, uint8_t instrFlags)
{
    if(!(instrFlags & GenOp_WriteFlags))
        return;

    auto f = mapReg(flagsReg);
    
    if(!f)
        return;

    uint32_t writtenMask = 0;

    for(int i = 0; i < 4; i++)
    {
        if(instrFlags & (1 << (i + 4)))
            writtenMask |= 1 << sourceInfo.flags[i].bit;
    }
    // clear flags
    builder.bic(*f, *f, writtenMask, false);

    assert(!writesFlag(instrFlags, SourceFlagType::HalfCarry));
    assert(!writesFlag(instrFlags, SourceFlagType::WasSub));

    // TODO: for multiple flags we can cheat for AGBRecompiler

    if(writesFlag(instrFlags, SourceFlagType::Carry))
    {
        branchOver(builder, [this, &f](ThumbBuilder &builder)
        {
            builder.orr(*f, *f, 1 << getFlagInfo(SourceFlagType::Carry).bit, false);
        }, Condition::CC);
    }

    if(writesFlag(instrFlags, SourceFlagType::Zero))
    {
        branchOver(builder, [this, &f](ThumbBuilder &builder)
        {
            builder.orr(*f, *f, 1 << getFlagInfo(SourceFlagType::Zero).bit, false);
        }, Condition::NE);
    }

    if(writesFlag(instrFlags, SourceFlagType::Negative))
    {
        branchOver(builder, [this, &f](ThumbBuilder &builder)
        {
            builder.orr(*f, *f, 1 << getFlagInfo(SourceFlagType::Negative).bit, false);
        }, Condition::PL);
    }

    if(writesFlag(instrFlags, SourceFlagType::Overflow))
    {
        branchOver(builder, [this, &f](ThumbBuilder &builder)
        {
            builder.orr(*f, *f, 1 << getFlagInfo(SourceFlagType::Overflow).bit, false);
        }, Condition::VC);
    }
}

std::optional<Reg> ThumbTarget::mapReg(uint8_t index)
{
    if(sourceInfo.registers[index].alias)
        return {};

    auto alloc = regAlloc.find(index);
    if(alloc != regAlloc.end())
        return alloc->second;

    return {};
}

std::optional<ThumbTarget::RegInfo> ThumbTarget::mapReg8(uint8_t index)
{
    // remap aliases
    auto &reg = sourceInfo.registers[index];
    if(reg.alias)
        index = reg.alias;

    if(auto mapped = mapReg(index))
        return RegInfo{*mapped, reg.aliasMask};

    return {};
}

SourceFlagInfo &ThumbTarget::getFlagInfo(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    assert(flagMap[iFlag] != 0xFF);

    return sourceInfo.flags[flagMap[iFlag]];
}

uint8_t ThumbTarget::flagWriteMask(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    if(flagMap[iFlag] == 0xFF) // source does not have this flag
        return 0;

    return 1 << (flagMap[iFlag] + 4);
}

bool ThumbTarget::writesFlag(uint16_t opFlags, SourceFlagType flag)
{
    return opFlags & flagWriteMask(flag);
}
