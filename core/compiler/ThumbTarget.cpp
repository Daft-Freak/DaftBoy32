#include <cassert>
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

// TODO: improve branch handling
static int load16BitValueSize(uint16_t value)
{
    if(value <= 0xFF)
        return 2; // mov
    
    if(value >> __builtin_ctz(value) <= 0xFF)
        return 4; // mov + lsl

    return 6; // mov + lsl + add
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

void ThumbTarget::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg regList[]
    {
        // from DMGRecompilerThumb
        Reg::R4,
        Reg::R5,
        Reg::R6,
        Reg::R7,
        Reg::R9,

        // temps
        Reg::R10, // not very optimal
        Reg::R2, // the same as temp/0, should only get used where temp is removed
    };

    // alloc registers
    unsigned int allocOff = 0;

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

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool ThumbTarget::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint16_t pc, GenBlockInfo &blockInfo)
{
    // don't handle HRAM for now
    if(pc >= 0xFF00)
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

    auto cycleExecuted = [this, &builder, &cyclesThisInstr, &delayedCyclesExecuted]()
    {
        cyclesThisInstr += 4;

        // only the optimised not-HRAM path
        delayedCyclesExecuted += 4;
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(delayedCyclesExecuted < 0xFF);
        auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

        uint8_t u8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        auto cycleCountOff = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - cpuPtrInt;
        assert(cycleCountOff <= 124);

        // load to r3, add and store back
        builder.mov(Reg::R1, cpuPtrReg); // cpu ptr

        builder.ldr(Reg::R3, Reg::R1, cycleCountOff);
        builder.add(Reg::R3, u8Cycles);
        builder.str(Reg::R3, Reg::R1, cycleCountOff);

        delayedCyclesExecuted = 0;
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
                auto loadSize = load16BitValueSize(pc);
                builder.b(loadSize + 6);
                lastInstrCycleCheck = builder.getPtr();

                // if <= 0 exit
                builder.b(Condition::GT, loadSize + 4);
                load16BitValue(builder, Reg::R1, pc);
                builder.bl((saveAndExitPtr - builder.getPtr()) * 2);

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

        // branch targets

        pc += instr.len;

        // update start pointer if the last op was the end of an emulated op
        if(newEmuOp)
            opStartPtr = builder.getPtr();

        // first cycle (fetch)
        int instrCycles = instr.cycles;
        if(instrCycles)
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
        auto checkReg = [this, &builder, &err, &instr](uint8_t index)
        {
            auto reg = mapReg(index);
            if(!reg)
            {
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

        auto checkRegOrImm = [&checkReg, &getLastImmLoad](uint8_t index) -> std::variant<std::monostate, Reg, uint32_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                    return *imm;
            }

            if(auto reg = checkReg(index))
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

        if(((instr.flags & GenOp_WriteFlags) || needCarry) && instr.opcode != GenOpcode::DMG_DAA /* needs most flags */)
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
                    builder.mov(Reg::R1, ~preserveMask);
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
                assert(instr.imm <= 0xFFFF);
                load16BitValue(builder, *mapReg(0), instr.imm);
                lastImmLoadEnd = builder.getPtr();
                break;

            case GenOpcode::Move:
            {
                assert(!(instr.flags & GenOp_WriteFlags));

                auto regSize = sourceInfo.registers[instr.dst[0]].size;
                if(regSize == 16)
                {
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
            {
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16)
                {
                    auto addr = checkRegOrImm(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(addr.index() && dst)
                    {
                        int pushMask = 1 << 0; // R0

                        builder.push(pushMask, false);

                        // if the address is in a high reg and the last op was an add to it, the value should still be in R1
                        // this allows us to skip the mov (for stack pops)
                        bool skipMov = false;
                        bool addrIsHighReg = std::holds_alternative<Reg>(addr) && !isLowReg(std::get<Reg>(addr));
                        if(!newEmuOp && addrIsHighReg && instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::Add && (instIt - 1)->dst[0] == instr.src[0] && !((instIt - 1)->flags & GenOp_WriteFlags))                        
                            skipMov = true;

                        setupMemAddr(addr, instr.src[0], skipMov);

                        builder.mov(Reg::R0, cpuPtrReg);

                        // get func ptr
                        loadLiteral(builder, Reg::R2, reinterpret_cast<uintptr_t>(sourceInfo.readMem));

                        builder.blx(Reg::R2); // do call

                        // move to dest
                        write8BitReg(builder, *dst, Reg::R0);

                        builder.pop(pushMask, false);
                    }
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Store:
            {
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16)
                {
                    auto addr = checkRegOrImm(instr.src[0]);
                    auto data = checkRegOrImm8(instr.src[1]);

                    if(addr.index() && data.index())
                    {
                        int pushMask = 1 << 4; // R4

                        builder.push(pushMask, false);

                        // if the address is in a high reg and the last op was an sub to it, the value should still be in R1
                        // this allows us to skip the mov (for stack pushes)
                        bool skipMov = false;
                        bool addrIsHighReg = std::holds_alternative<Reg>(addr) && !isLowReg(std::get<Reg>(addr));
                        if(!newEmuOp && addrIsHighReg && instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::Subtract && (instIt - 1)->dst[0] == instr.src[0])
                            skipMov = true;

                        setupMemAddr(addr, instr.src[0], skipMov);
                        get8BitValue(builder, Reg::R2, data);
                        builder.mov(Reg::R3, cycleCountReg);

                        builder.mov(Reg::R0, cpuPtrReg);

                        // get func ptr
                        loadLiteral(builder, Reg::R4, reinterpret_cast<uintptr_t>(sourceInfo.writeMem));

                        builder.blx(Reg::R4); // do call

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
            
                if(regSize == 16)
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
            
                if(regSize == 8)
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

                if(regSize == 8)
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

                if(regSize == 8)
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

            case GenOpcode::Or:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 16)
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
            
                if(regSize == 16)
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
            
                if(regSize == 8)
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

                if(regSize == 8)
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

                if(regSize == 8)
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

                if(regSize == 16)
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

                if(regSize == 8)
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

                if(regSize == 16)
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
                        bool flagSet = false;

                        // condition
                        if(condition != GenCondition::Always)
                        {
                            auto f = checkReg8(flagsReg);

                            syncCyclesExecuted();

                            flagSet = condition == GenCondition::Equal || condition == GenCondition::CarrySet;
                            auto flag = (condition == GenCondition::Equal || condition == GenCondition::NotEqual) ? SourceFlagType::Zero : SourceFlagType::Carry;

                            if(f)
                            {
                                builder.mov(Reg::R1, 1 << getFlagInfo(flag).bit);
                                builder.and_(Reg::R1, f->reg);
                                branchPtr = builder.getPtr();
                                builder.b(Condition::EQ, 0); // patch later
                            }
                        }

                        // need to sync cycles *before* the jump out
                        if(instrCycles)
                        {
                            cycleExecuted();
                            instrCycles--;
                            assert(instrCycles == 0);
                        }
                        syncCyclesExecuted();


                        // set pc if we're exiting
                        // ... or always as we might not be able to patch the branch
                        if(std::holds_alternative<uint32_t>(src))
                            load16BitValue(builder, pcReg, std::get<uint32_t>(src));
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
                            if(isExit && (instr.flags & GenOp_Call))
                                builder.bl((exitForCallPtr - builder.getPtr()) * 2); // call, save when exiting
                            else
                                builder.bl((exitPtr - builder.getPtr()) * 2); // patched later if not exit
                        }

                        // patch the condition jump
                        if(branchPtr && !builder.getError())
                        {
                            int off = (builder.getPtr() - (branchPtr + 1)) * 2;
                            builder.patch(branchPtr, branchPtr + 2);
                            builder.b(flagSet ? Condition::EQ : Condition::NE, off);
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

            load16BitValue(builder, pcReg, pc - instr.len);
            builder.bl((exitPtr - builder.getPtr()) * 2);
            outputLiterals(builder, false);
            break;
        }

        // additional cycle
        if(instrCycles)
        {
            cycleExecuted();
            assert(instrCycles == 1);
        }

        // check if this is the end of the source instruction (pc incremented)
        newEmuOp = instr.len != 0;

        // check cycle count if this is the last part of en emulated op
        // ... but not on the last op, that should always exit anyway
        // ... or exits unless followed by a branch target
        auto nextInstr = instIt + 1;
        bool shouldSkip = nextInstr == endInstr || ((instr.flags & GenOp_Exit) && !(nextInstr->flags & GenOp_BranchTarget));
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
            auto loadSize = load16BitValueSize(pc);

            // if <= 0 exit
            builder.b(Condition::GT, loadSize + 4);

            load16BitValue(builder, pcReg, pc);
            builder.bl((saveAndExitPtr - builder.getPtr()) * 2);

            cyclesThisInstr = 0;
            forceExitAfter = false;
        }

        if(newEmuOp)
            numInstructions++;

        // output literals if ~close to the limit or this is the last instruction
        // this was definitely NOT set by decreasing until it didn't abort...
        // don't output after a LoadImm as it might get removed by the next op
        if(instr.opcode != GenOpcode::LoadImm && !ldrLiteralInstrs.empty() && (nextInstr == endInstr || builder.getPtr() - ldrLiteralInstrs[0] >= 460))
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

    builder.mov(Reg::R2, Reg::R8);
    builder.mov(Reg::R3, Reg::R9);
    builder.push(0b11111100, true); // R2-7, LR

    builder.mov(Reg::R2, Reg::R10);
    builder.push(0b100, false); // R2
    
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

                assert(reg.cpuOffset - firstRegOff <= 62);

                if(isLowReg(*mappedReg))
                {
                    if(reg.size == 32)
                        builder.ldr(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
                    else
                        builder.ldrh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
                }
                else
                {
                    if(reg.size == 32)
                        builder.ldr(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff);
                    else
                        builder.ldrh(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff);
                    builder.mov(*mappedReg, Reg::R3);
                }
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
                assert(reg.cpuOffset - firstRegOff <= 62);
                if(isLowReg(*mappedReg))
                {
                    if(reg.size == 32)
                        builder.str(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff);
                    else
                        builder.strh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
                }
                else
                {
                    builder.mov(Reg::R3, *mappedReg);
                    if(reg.size == 32)
                        builder.str(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff);
                    else
                        builder.strh(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
                }
            }
        }

        i++;
    }


    // restore regs and return
    builder.pop(0b100, false); // R2
    builder.mov(Reg::R10, Reg::R2);

    builder.pop(0b11111100, false); // R2-7
    builder.mov(Reg::R8, Reg::R2);
    builder.mov(Reg::R9, Reg::R3);

    builder.pop(0, true); // PC

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
            auto start = ptr + 1;
            if(reinterpret_cast<uintptr_t>(start) & 2)
                start++;

            // update LDR imm
            auto index = *ptr & 0xFF;
            auto off = ((dataPtr + index * 2) - start) >> 1;

            assert(index < 2);
            assert(off <= 0xFF);
            *ptr = (*ptr & 0xFF00) | off;
        }
    }

    // reset
    ldrLiteralInstrs.clear();
    curLiteral = 0;

    for(auto &literal : literals)
        literal = 0;
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
