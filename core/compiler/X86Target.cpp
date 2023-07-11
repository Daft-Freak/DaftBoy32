#include <cassert>
#include <cstdio>
#include <functional>
#include <variant>

#include "X86Target.h"
#include "X86Builder.h"

enum CheckValueFlags
{
    Value_Immediate = 1 << 0,
    Value_Memory    = 1 << 1,
};

// reg helpers
static const Reg64 cpuPtrReg = Reg64::R14;

// only an output at the end, use the same reg used for the cycle count
static const Reg32 pcReg32 = Reg32::EDI;
static const Reg16 pcReg16 = Reg16::DI;

#ifdef _WIN32
static const Reg64 argumentRegs64[]{Reg64::RCX, Reg64::RDX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::ECX, Reg32::EDX, Reg32::R8D, Reg32::R9D};
#else
static const Reg64 argumentRegs64[]{Reg64::RDI, Reg64::RSI, Reg64::RDX, Reg64::RCX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::EDI, Reg32::ESI, Reg32::EDX, Reg32::ECX, Reg32::R8D, Reg32::R9D};
#endif

static const Reg64 callSavedRegs[]{Reg64::RAX, Reg64::RCX, Reg64::RDX, Reg64::RDI, Reg64::RSI};

static bool isXHReg(Reg8 reg)
{
    return static_cast<int>(reg) >= 4 && static_cast<int>(reg) < 8;
}

static bool requiresREX(Reg8 reg)
{
    return static_cast<int>(reg) >= 8;
}

static Reg8 swapRegHalf(Reg8 reg)
{
    auto iReg = static_cast<int>(reg);
    assert(iReg < 8);

    return static_cast<Reg8>(iReg ^ 4);
}

// 8-bit op helpers
using ImmOp8 = void(Reg8, uint8_t);
using RegOp8 = void(Reg8, Reg8);
using RegUnOp8 = void(Reg8);

static bool doRegImmOp8(X86Builder &builder, std::optional<Reg8> dst, std::variant<std::monostate, Reg8, uint8_t> src, std::function<void(X86Builder &, Reg8, Reg8)> regOp, std::function<void(X86Builder &, Reg8, uint8_t)> immOp)
{
    if(!src.index() || !dst)
        return false;

    if(std::holds_alternative<uint8_t>(src))
        immOp(builder, *dst, std::get<uint8_t>(src));
    else
    {
        auto regSrc = std::get<Reg8>(src);

        // if there's a reg that requires REX and an xH reg, swap the halves over and read/write xL instead
        std::optional<Reg8> swapReg;
        if(isXHReg(*dst) && requiresREX(regSrc))
            swapReg = dst = swapRegHalf(*dst);
        else if(isXHReg(regSrc) && requiresREX(*dst))
            swapReg = regSrc = swapRegHalf(regSrc);

        if(swapReg)
            builder.xchg(*swapReg, swapRegHalf(*swapReg));

        regOp(builder, *dst, regSrc);

        if(swapReg)
            builder.xchg(*swapReg, swapRegHalf(*swapReg));
    }

    return true;
}

static bool doRegImmShift8(X86Builder &builder, std::optional<Reg8> dst, std::variant<std::monostate, Reg8, uint8_t> src, std::function<void(X86Builder &, Reg8)> regOp, std::function<void(X86Builder &, Reg8, uint8_t)> immOp)
{
    if(!src.index() || !dst)
        return false;

    if(std::holds_alternative<uint8_t>(src))
    {
        assert(std::get<uint8_t>(src) < 32);
        immOp(builder, *dst, std::get<uint8_t>(src));
    }
    else
    {
        auto srcReg = std::get<Reg8>(src);
        assert(srcReg != *dst);

        // can only shift by CL
        bool swap = srcReg != Reg8::CL;

        // swap with whatever is currently in CL
        if(swap)
            builder.xchg(srcReg, Reg8::CL);

        // if it was the dst swap the args around
        if(dst == Reg8::CL)
            regOp(builder, srcReg);
        else
            regOp(builder, *dst);

        // restore
        if(swap)
            builder.xchg(srcReg, Reg8::CL);
    }

    return true;
}

// more shift helpers
static bool doRegImmShift32(X86Builder &builder, std::optional<Reg32> dst, std::variant<std::monostate, Reg8, uint8_t> src, std::function<void(X86Builder &, Reg32)> regOp, std::function<void(X86Builder &, Reg32, uint8_t)> immOp, std::function<void(X86Builder &)> preOp, bool rotate = false)
{
    if(!src.index() || !dst)
        return false;

    if(std::holds_alternative<uint8_t>(src))
    {
        preOp(builder);

        auto imm = std::get<uint8_t>(src);
        assert(imm <= 32);
        if(imm == 32)
        {
            immOp(builder, *dst, 31);
            imm = 1;
        }
        immOp(builder, *dst, imm);
    }
    else
    {
        auto srcReg = std::get<Reg8>(src);
        auto srcReg32 = static_cast<Reg32>(srcReg);

        auto patchCondBranch = [&builder](uint8_t *branchPtr, Condition cond)
        {
            if(!branchPtr)
                return;

            auto off = builder.getPtr() - branchPtr - 2;
            builder.patch(branchPtr, branchPtr + 2);
            builder.jcc(cond, off);
            builder.endPatch();
        };

        auto patchBranch = [&builder](uint8_t *branchPtr)
        {
            if(!branchPtr)
                return;

            auto off = builder.getPtr() - branchPtr - 2;
            builder.patch(branchPtr, branchPtr + 2);
            builder.jmp(off);
            builder.endPatch();
        };

        // special cases
        uint8_t *ltBranch, *eqBranch = nullptr;
        uint8_t *gtDoneBranch = nullptr, *eqDoneBranch;

        builder.cmp(srcReg, 32);

        ltBranch = builder.getPtr();
        builder.jcc(Condition::B, 0); // < 32

        // skip the > case for rotates
        // in this case the condition above is != 32
        if(!rotate)
        {
            eqBranch = builder.getPtr();
            builder.jcc(Condition::E, 0); // == 32

            // > 32
            // need to set to 0 and set appropriate flags
            preOp(builder);
            immOp(builder, *dst, 31);
            immOp(builder, *dst, 2);
            gtDoneBranch = builder.getPtr();
            builder.jmp(0);
        }

        // == 32
        patchCondBranch(eqBranch, Condition::E);
        preOp(builder);
        immOp(builder, *dst, 31);
        immOp(builder, *dst, 1);
        eqDoneBranch = builder.getPtr();
        builder.jmp(0);

        // < 32 (!= 32 for rotates)
        patchCondBranch(ltBranch, rotate ? Condition::NE : Condition::B);

        bool swap = srcReg != Reg8::CL;

        if(swap)
            builder.xchg(srcReg32, Reg32::ECX);

        preOp(builder);

        // if src/dst were the same, they're now both ECX
        if(srcReg32 == *dst)
            regOp(builder, Reg32::ECX);
        // if it was the dst swap the args around
        else if(dst == Reg32::ECX)
            regOp(builder, srcReg32);
        else
            regOp(builder, *dst);

        if(swap)
            builder.xchg(srcReg32, Reg32::ECX);

        patchBranch(gtDoneBranch);
        patchBranch(eqDoneBranch);
    }

    return true;
}

void X86Target::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg32 regList[]
    {
        Reg32::EAX,
        Reg32::ECX,
        Reg32::EDX,
        Reg32::EBX,
        Reg32::EBP,
        Reg32::R12D,
        Reg32::R13D,
        Reg32::R15D,

        Reg32::R11D, // last resort temp
    };

    numSavedRegs = 4; // TODO: can skip RDI/RSI on windows

    // count regs
    int numRegs = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(!reg.alias)
            numRegs++;
    }

    // enable using R15 if needed
    saveR15 = numRegs > 8;

    // alloc registers
    unsigned int allocOff = 0;

    regAlloc.emplace(0, Reg32::R10D); // temp

    int i = 1;
    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it, i++)
    {
        if(it->type == SourceRegType::Flags)
        {
            flagsReg = i;
            flagsSize = it->size;
        }

        if(it->alias)
            continue;

        // TODO: make sure we allocate all temps
        if(allocOff == std::size(regList))
            continue;

        if(it->type != SourceRegType::Temp && regList[allocOff] == Reg32::R11D)
            continue;

        regAlloc.emplace(i, regList[allocOff]);

        allocOff++;
    }

    // map flags
    for(auto & f : flagMap)
        f = 0xFF;

    i = 0;
    for(auto it = sourceInfo.flags.begin(); it != sourceInfo.flags.end(); ++it, i++)
    {
        flagMap[static_cast<int>(it->type)] = i;
        flagsMask |= 1 << it->bit;
    }

    // allocate the flags register if it isn't an alias
    if(!sourceInfo.registers[flagsReg].alias && !regAlloc.count(flagsReg))
    {
        // assume this isn't something that has a half carry flag so ESI is free
        assert(!flagWriteMask(SourceFlagType::HalfCarry));

        regAlloc.emplace(flagsReg, Reg32::ESI);
        numSavedRegs++; // need to save it
    }

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool X86Target::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint32_t pc, GenBlockInfo &blockInfo)
{
    X86Builder builder(codePtr, codeBufEnd);

    auto startPC = pc;

    // state
    uint8_t *lastInstrCycleCheck = nullptr;
    std::map<uint32_t, uint8_t *> branchTargets;
    std::multimap<uint32_t, uint8_t *> forwardBranchesToPatch;
    int needCallRestore = 0;


    // cycle executed sync
    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;
    bool stackCycleCount = false; // cycle count stored to stack

    auto cycleExecuted = [this, &builder, &needCallRestore, &blockInfo, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount]()
    {
        assert(!stackCycleCount);

        cyclesThisInstr += sourceInfo.cycleMul;

        if(!(blockInfo.flags & GenBlock_StrictSync))
        {
            // delay/inline cycle updates if possible
            delayedCyclesExecuted += sourceInfo.cycleMul;
            return;
        }

        // FIXME: handle not having a cycleExecuted (AGBRecompiler)

        callSaveIfNeeded(builder, needCallRestore);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(sourceInfo.cycleExecuted)); // function ptr
        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr
        builder.call(Reg64::RAX); // do call
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted, &stackCycleCount]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(!stackCycleCount);

        assert(delayedCyclesExecuted < 127);
        auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

        int8_t i8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        int offset = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - cpuPtrInt;
        builder.addD({cpuPtrReg, offset}, i8Cycles);

        delayedCyclesExecuted = 0;
    };

    auto writeCycleCountToStack = [this, &builder, &needCallRestore, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount](int &instrCycles)
    {
        if(!stackCycleCount)
        {
            assert(cyclesThisInstr == delayedCyclesExecuted);
            builder.mov({Reg64::RSP, needCallRestore * 8}, uint32_t(cyclesThisInstr + instrCycles));

            cyclesThisInstr = delayedCyclesExecuted = 0;
            instrCycles = 0;
            stackCycleCount = true;
        }
        else
            assert(!cyclesThisInstr && !delayedCyclesExecuted);
    };

    // load/store helpers
    auto setupMemAddr = [this, &blockInfo, &builder, &syncCyclesExecuted](std::variant<std::monostate, Reg32, uint32_t> addr, int addrSize, uint8_t addrIndex)
    {
        if(!addr.index())
            return; // err should already be set

        if(std::holds_alternative<Reg32>(addr))
        {
            if(!sourceInfo.shouldSyncForRegIndex || sourceInfo.shouldSyncForRegIndex(addrIndex, blockInfo))
                syncCyclesExecuted();

            auto addrReg = std::get<Reg32>(addr);

            if(addrSize == 16)
                builder.movzx(argumentRegs32[1], static_cast<Reg16>(addrReg));
            else
                builder.mov(argumentRegs32[1], addrReg);
        }
        else
        {
            auto immAddr = std::get<uint32_t>(addr);
            assert(addrSize == 32 || !(immAddr & 0xFFFF0000));

            if(!sourceInfo.shouldSyncForAddress || sourceInfo.shouldSyncForAddress(immAddr))
                syncCyclesExecuted();

            builder.mov(argumentRegs32[1], immAddr);
        }
    };

    auto carryIn = [this, &builder](Reg8 f)
    {
        builder.test(f, 1 << getFlagInfo(SourceFlagType::Carry).bit); // sets CF to 0
        builder.jcc(Condition::E, 1); // not set
        builder.stc(); // CF = 1
    };

    // neededFlags is what we need to output
    // updateFlags is what we can output from the result
    // oneFlags is is what is alwyas set to 1
    // hasPreservedFlags is set if some flags were preserved (we can't write flags directly here)
    const auto setFlags = [this, &builder, &needCallRestore](std::optional<Reg8> result, uint8_t &neededFlags, uint8_t updateFlags, uint8_t oneFlags = 0, bool hasPreservedFlags = false, bool isRotate = false)
    {
        if(!flagsReg)
            return;

        if(!neededFlags)
            return; // nothing to set

        // FIXME: assuming 8bit flags reg
        auto f = *mapReg8(flagsReg);

        callRestoreIfNeeded(builder, f, needCallRestore);

        updateFlags &= neededFlags;  // mask out unneeded

        // remap "always one" flags to bits in reg
        uint8_t mappedOneFlags = 0;

        for(int i = 0; i < 4; i++)
        {
            if(oneFlags & (1 << (i + 4))) // +4 as it's the write flags
                mappedOneFlags |= 1 << sourceInfo.flags[i].bit;
        }

        // preserved flags should have been preserved already
        // so F should have all other bits zero

        bool setF = hasPreservedFlags;
        bool haveOpFlags = true; // assuming nothing has destroyed flags before this

        if(haveOpFlags && (updateFlags & flagWriteMask(SourceFlagType::Carry)))
        {
            auto &cFlag = getFlagInfo(SourceFlagType::Carry);

            // copy + shift if not set
            if(!setF)
            {
                builder.setcc(Condition::B, f);
                builder.shl(f, cFlag.bit); // shift to position
                setF = true;
            }
            else
            {
                builder.jcc(Condition::AE, 3); // if !carry
                builder.or_(f, 1 << cFlag.bit); // set C
            }

            haveOpFlags = false;
            neededFlags &= ~flagWriteMask(SourceFlagType::Carry);
        }

        if((updateFlags & flagWriteMask(SourceFlagType::Zero)))
        {
            auto &zFlag = getFlagInfo(SourceFlagType::Zero);

            bool haveZFlag = haveOpFlags && !isRotate;  // rotates don't set Z

            if(!haveZFlag && result)
            {
                // reconstruct from result
                builder.cmp(*result, 0);
                haveZFlag = true;
            }

            if(haveZFlag)
            {
                if(!setF)
                {
                    builder.setcc(Condition::E, f);
                    builder.shl(f, zFlag.bit); // shift to position
                    setF = true;
                }
                else
                {
                    builder.jcc(Condition::NE, 3); // if != 0
                    builder.or_(f, 1 << zFlag.bit); // set Z
                }

                haveOpFlags = false;
                neededFlags &= ~flagWriteMask(SourceFlagType::Zero);
            }
        }

        if(neededFlags && !setF) // some flags left and no write, make sure constant flags are set
        {
            builder.mov(f, mappedOneFlags);
            neededFlags &= ~oneFlags;
            setF = true;
        }

        // still haven't set always one flags
        if(neededFlags & oneFlags)
        {
            builder.or_(f, mappedOneFlags);
            neededFlags &= ~oneFlags;
        }
    };

    const auto calcHalfCarry8 = [this, &builder](GenOpcode op, Reg8 dst, std::variant<std::monostate, Reg8, uint8_t> src, std::optional<Reg8> f)
    {
        bool withCarry = op == GenOpcode::AddWithCarry || op == GenOpcode::SubtractWithCarry;
        bool isAdd = op == GenOpcode::Add || op == GenOpcode::AddWithCarry;

        assert(dst != Reg8::AL);
        builder.push(Reg64::RAX);

        auto halfDst = Reg8::AH;

        if(withCarry)
        {
            // check carry
            auto carryBit = getFlagInfo(SourceFlagType::Carry).bit;
            builder.test(*f, 1 << carryBit); // sets CF to 0
            builder.setcc(Condition::NE, Reg8::SIL);

            if(std::holds_alternative<uint8_t>(src))
                halfDst = Reg8::AL; // avoid conflict
        }
        
        if(dst != halfDst)
        {
            if(requiresREX(dst)) // INC (HL)
            {
                assert(std::holds_alternative<uint8_t>(src));
                // use the other half, we don't need it
                halfDst = Reg8::AL;
            }

            builder.mov(halfDst, dst);
        }

        if(std::holds_alternative<uint8_t>(src))
        {
            // imm (simplify a bit)
            builder.and_(halfDst, 0xF); // mask

            if(isAdd)
            {
                if(withCarry)
                    builder.add(halfDst, Reg8::SIL); // add carry

                builder.cmp(halfDst, 0xF - (std::get<uint8_t>(src) & 0xF));
            }
            else
            {
                if(withCarry)
                    builder.sub(halfDst, Reg8::SIL); // add carry

                builder.cmp(halfDst, std::get<uint8_t>(src) & 0xF);
            }
        }
        else
        {
            // reg
            builder.mov(Reg8::AL, std::get<Reg8>(src));

            builder.and_(Reg32::EAX, 0x0F0Fu); // mask both

            if(withCarry)
                builder.add(Reg8::AL, Reg8::SIL); // add carry

            if(isAdd)
            {
                builder.add(halfDst, Reg8::AL);
                builder.cmp(halfDst, 0xF);
            }
            else
                builder.cmp(halfDst, Reg8::AL);
        }

        builder.pop(Reg64::RAX);

        builder.setcc(isAdd ? Condition::A : Condition::L, Reg8::SIL);
        builder.shl(Reg8::SIL, getFlagInfo(SourceFlagType::HalfCarry).bit);
    };

    auto setFlags32 = [this, &builder](Reg32 dst, Reg32 carryFlagCopy, uint16_t flags, bool invCarry = false, uint8_t haveResFlags = 0xF0)
    {
        if(!(flags & GenOp_WriteFlags))
            return;

        assert(!writesFlag(flags, SourceFlagType::HalfCarry));
        assert(!writesFlag(flags, SourceFlagType::WasSub));
    
        auto f = mapReg32(flagsReg);

        // overflow
        if(writesFlag(flags, SourceFlagType::Overflow))
        {
            builder.jcc(Condition::NO, 6);
            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Overflow).bit);
            haveResFlags = false;
        }

        // carry
        auto cFlag = flagWriteMask(SourceFlagType::Carry);
        if(flags & cFlag)
        {
            if(!(haveResFlags & cFlag))
            {
                builder.shl(carryFlagCopy, getFlagInfo(SourceFlagType::Carry).bit);
                builder.or_(*f, carryFlagCopy);
            }
            else
            {
                builder.jcc(invCarry ? Condition::B : Condition::AE, 6);

                builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Carry).bit);
            }
            haveResFlags = 0;
        }

        // negative
        auto nFlag = flagWriteMask(SourceFlagType::Negative);
        if(flags & nFlag)
        {
            if(haveResFlags & nFlag)
                builder.jcc(Condition::NS, 6);
            else
            {
                // dst < 0
                builder.cmp(dst, int8_t(0));
                builder.jcc(Condition::GE, 6);
            }

            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Negative).bit);
            haveResFlags = 0;
        }

        // zero
        auto zFlag = flagWriteMask(SourceFlagType::Zero);
        if(flags & zFlag)
        {
            if(!(haveResFlags & zFlag))
                builder.cmp(dst, int8_t(0));

            builder.jcc(Condition::NE, 6);
            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Zero).bit);
            haveResFlags = 0;
        }
    };

    // do instructions
    int numInstructions = 0;
    uint8_t *opStartPtr = nullptr;
    uint8_t *lastImmLoadStart = nullptr, *lastImmLoadEnd = nullptr;
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
                builder.jmp(12);
                lastInstrCycleCheck = builder.getPtr();

                // if <= 0 exit
                builder.jcc(Condition::G, 10);
                builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
                builder.call(saveAndExitPtr - builder.getPtr());

                // TODO: this is the first instruction
                branchTargets.emplace(pc, lastInstrCycleCheck);
            }

            // patch forwards jumps
            // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
            auto jumps = forwardBranchesToPatch.equal_range(pc);
            for(auto it = jumps.first; it != jumps.second; ++it)
            {
                // overrite 4 byte disp
                auto off = lastInstrCycleCheck - (it->second + 5);

                it->second[1] = off;
                it->second[2] = off >> 8;
                it->second[3] = off >> 16;
                it->second[4] = off >> 24;
            }
            forwardBranchesToPatch.erase(jumps.first, jumps.second);
        }

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

            // if(enableInterruptsNextCycle)
            // probably don't need this check... might get a false positive in some extreme case though
            builder.cmp({cpuPtrReg, sourceInfo.extraCPUOffsets[1]/*enableInterruptsNextCycle*/}, 0);
            builder.jcc(Condition::E, 10);

            // masterInterruptEnable = true
            builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/}, uint8_t(1));

            // enableInterruptsNextCycle = false
            builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[1]/*enableInterruptsNextCycle*/}, uint8_t(0));

            forceExitAfter = true;
            lastWasEI = false;
        }

        bool err = false;

        // helpers to load/store registers we didn't allocate
        auto loadExtraReg32 = [this, &builder](uint8_t index, Reg32 dst)
        {
            auto offset = sourceInfo.getRegOffset(cpuPtr, index);
            builder.mov(dst, {cpuPtrReg, offset});
        };

        auto storeExtraReg32 = [this, &builder](uint8_t index, std::variant<std::monostate, Reg32, uint32_t> src)
        {
            assert(src.index());

            auto offset = sourceInfo.getRegOffset(cpuPtr, index);

            if(std::holds_alternative<uint32_t>(src))
                builder.mov({cpuPtrReg, offset}, std::get<uint32_t>(src));
            else
                builder.mov({cpuPtrReg, offset}, std::get<Reg32>(src));
        };

        // validating wrappers
        auto checkReg = [this, &builder, &needCallRestore, &err, &instr](uint8_t index, int size, bool allowExtra = false)
        {
            // this one doesn't return the reg, just validates and makes sure it's available
            bool found = false;
            int saveIndex = -1;

            if(size == 8)
            {
                if(auto reg = mapReg8(index))
                {
                    found = true;
                    saveIndex = callSaveIndex(*reg);
                }
            }
            else
            {
                // 16/32-bit registers are the same as far as this validation is concerned
                if(auto reg = mapReg32(index))
                {
                    found = true;
                    saveIndex = callSaveIndex(static_cast<Reg16>(*reg));
                }
            }

            if(!found)
            {
                if(allowExtra)
                    return true;

                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else if(saveIndex != -1)
                callRestore(builder, needCallRestore, saveIndex);

            return found;
        };

        auto checkReg32 = [this, &builder, &needCallRestore, &err, &instr, &loadExtraReg32](uint8_t index, std::optional<Reg32> loadReg = {}, bool allowExtra = false)
        {
            auto reg = mapReg32(index);
            if(!reg)
            {
                // if 
                if(sourceInfo.getRegOffset)
                {
                    // load extra reg if we have somewhere to put it
                    if(loadReg)
                    {
                        loadExtraReg32(index, *loadReg);
                        return loadReg;
                    }

                    // otherwise return nothing but don't fail if the caller has promised to handle it
                    if(allowExtra)
                        return reg;
                }

                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else
                callRestoreIfNeeded(builder, static_cast<Reg16>(*reg), needCallRestore);

            return reg;
        };

        auto checkReg16 = [this, &builder, &needCallRestore, &err, &instr](uint8_t index)
        {
            auto reg = mapReg16(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else
                callRestoreIfNeeded(builder, *reg, needCallRestore);

            return reg;
        };

        auto checkReg8 = [this, &builder, &needCallRestore, &err, &instr](uint8_t index)
        {
            auto reg = mapReg8(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else
                callRestoreIfNeeded(builder, *reg, needCallRestore);

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

        auto checkRegOrImm32 = [&checkReg32, &getLastImmLoad](uint8_t index, std::optional<Reg32> loadReg = {}, bool allowLoad = false) -> std::variant<std::monostate, Reg32, uint32_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                    return *imm;
            }

            if(auto reg = checkReg32(index, loadReg, allowLoad))
                return *reg;

            return {};
        };

        auto checkRegOrImm16 = [&checkReg16, &getLastImmLoad](uint8_t index) -> std::variant<std::monostate, Reg16, uint16_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                {
                    assert(!(*imm & 0xFFFF0000));
                    return static_cast<uint16_t>(*imm);
                }
            }

            if(auto reg = checkReg16(index))
                return *reg;

            return {};
        };

        auto checkRegOrImm8 = [&checkReg8, &getLastImmLoad](uint8_t index) -> std::variant<std::monostate, Reg8, uint8_t>
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

        auto checkValue32 = [this, &checkReg32, &getLastImmLoad, &err](uint8_t index, int flags = 0, std::optional<Reg32> loadReg = {}) -> std::variant<std::monostate, X86Builder::RMOperand, uint32_t>
        {
            if(index == 0 && (flags & Value_Immediate))
            {
                auto imm = getLastImmLoad();
                if(imm)
                    return *imm;
            }

            if(auto reg = checkReg32(index, loadReg, flags & Value_Memory))
                return X86Builder::RMOperand{*reg};

            if(!err) // must be memory, checkReg didn't set an error
            {
                auto offset = sourceInfo.getRegOffset(cpuPtr, index);
                return X86Builder::RMOperand{cpuPtrReg, offset};
            }

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

        // helpers to deal with restrictions
        auto checkSingleSource = [this, &builder, &err, &instr, &checkReg32](bool canSwapSrcs = false)
        {
            if(instr.src[0] != instr.dst[0])
            {
                if(instr.src[1] == instr.dst[0] && canSwapSrcs)
                    return true;

                if(!sourceInfo.registers[instr.dst[0]].aliasMask && !sourceInfo.registers[instr.src[0]].aliasMask)
                {
                    auto dst = checkReg32(instr.dst[0]);
                    if(dst && instr.src[1] == instr.dst[0] && instr.opcode != GenOpcode::Not)
                    {
                        // dest is the second source, save it and replace the source
                        assert(instr.dst[0]); // it's already a temp

                        auto tmp = mapReg32(0);

                        if(instr.src[0] == 0)
                        {
                            printf("unhandled src[0] != dst in op %i\n", int(instr.opcode));
                            err = true;
                        }
                        else
                        {
                            // save dest in temp and use as second source
                            builder.mov(*tmp, *dst);
                            instr.src[1] = 0;
                        }
                    }

                    // move src0 to dst
                    auto src = checkReg32(instr.src[0], Reg32::R8D);

                    if(src && dst)
                    {
                        builder.mov(*dst, *src);
                        return false;
                    }
                }

                printf("unhandled src[0] != dst in op %i\n", int(instr.opcode));
                err = true;
            }

            return false;
        };

        // preserve flags
        // needs to be after the reg helpers
        uint32_t preserveMask = 0;

        if((instr.flags & GenOp_WriteFlags))
        {
            preserveMask = 0;
            for(int i = 0; i < 4; i++)
            {
                if(instr.flags & (1 << i))
                    preserveMask |= 1 << sourceInfo.flags[i].bit;
            }

            // preserve/clear flags
            if(flagsSize == 8 && preserveMask)
            {
                // 8-bit flags code assumes there is nothing else in the reg and writes the whole reg if there are no preserved flags
                if(auto f = checkReg8(flagsReg))
                    builder.and_(*f, preserveMask);
            }
            else if(flagsSize == 32)
            {
                // preserve carry if op needs it
                bool needCarry = instr.opcode == GenOpcode::AddWithCarry
                              || instr.opcode == GenOpcode::SubtractWithCarry
                              || instr.opcode == GenOpcode::RotateLeftCarry
                              || instr.opcode == GenOpcode::RotateRightCarry;

                if(needCarry)
                    preserveMask |= 1 << getFlagInfo(SourceFlagType::Carry).bit;

                // also preserve non-flags bits
                if(auto f = checkReg32(flagsReg))
                    builder.and_(*f, preserveMask | (~flagsMask));
            }
        }

        switch(instr.opcode)
        {
            case GenOpcode::NOP:
                break;

            case GenOpcode::LoadImm:
                lastImmLoadStart = builder.getPtr();
                builder.mov(*mapReg32(0), instr.imm);
                lastImmLoadEnd = builder.getPtr();
                break;

            case GenOpcode::Move:
            {
                auto regSize = sourceInfo.registers[instr.dst[0]].size;

                if(regSize == 32)
                {
                    auto dst = checkReg32(instr.dst[0], {}, true);

                    // avoid a mov if we need to load src, but not dst
                    auto src = checkRegOrImm32(instr.src[0], dst ? *dst : Reg32::R8D);
                    
                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                            builder.mov(*dst, std::get<uint32_t>(src));
                        else
                            builder.mov(*dst, std::get<Reg32>(src));

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags, false, 0);
                    }
                    else if(src.index() && !err) 
                    {
                        // store extra reg
                        storeExtraReg32(instr.dst[0], src);
                    }
                }
                else if(regSize == 16)
                {
                    assert(!(instr.flags & GenOp_WriteFlags));

                    // use 32-bit mov to save some bytes
                    auto dst = checkReg32(instr.dst[0]);

                    if(sourceInfo.registers[instr.src[0]].size == 8)
                    {
                        // 8 -> 16 bit
                        auto src = checkReg8(instr.src[0]);

                        if(src && dst)
                            builder.movzx(*dst, *src);
                    }
                    else
                    {
                        auto src = checkRegOrImm32(instr.src[0]);

                        if(src.index() && dst)
                        {
                            if(std::holds_alternative<uint32_t>(src))
                                builder.mov(*dst, std::get<uint32_t>(src));
                            else
                                builder.mov(*dst, std::get<Reg32>(src));
                        }
                    }
                }
                else if(regSize == 8)
                {
                    assert(!(instr.flags & GenOp_WriteFlags));

                    auto src = checkRegOrImm8(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);

                    doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::mov), std::mem_fn<ImmOp8>(&X86Builder::mov));
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::Load:
            case GenOpcode::Load2:
            case GenOpcode::Load4:
            {
                // TODO: store data width somewhere
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16 || addrSize == 32)
                {
                    auto dst = checkReg8(instr.dst[0]);
                    auto addr = checkRegOrImm32(instr.src[0], Reg32::R11D);

                    if(addr.index() && dst)
                    {
                        // zero-extend if not 8 bit dest (a temp)
                        bool zeroExtend = instr.opcode == GenOpcode::Load && sourceInfo.registers[instr.dst[0]].size != 8;

                        // unless we want to sign extend instead
                        if(instr.flags & GenOp_SignExtend)
                            zeroExtend = false;
                        
                        // maybe push
                        callSaveIfNeeded(builder, needCallRestore);

                        bool needCyclesSeq = sourceInfo.readMem8;

                        setupMemAddr(addr, addrSize, instr.src[0]);

                        uintptr_t func = 0;

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

                        if(!func)
                        {
                            printf("unhandled load\n");
                            err = true;
                        }

                        builder.mov(Reg64::RAX, func); // function ptr

                        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr

                        if(needCyclesSeq)
                        {
                            // cycle count from stack
                            builder.lea(argumentRegs64[2], {Reg64::RSP, needCallRestore * 8});
                            writeCycleCountToStack(instrCycles);

                            builder.mov(argumentRegs32[3], instr.flags >> 8); // flags
                        }

                        builder.call(Reg64::RAX); // do call

                        if(instr.opcode == GenOpcode::Load)
                        {
                            // 8-bit dest
                            if(isCallSaved(*dst))
                            {
                                callRestore(builder, *dst, zeroExtend, instr.flags & GenOp_SignExtend);
                                needCallRestore = 0;
                            }
                            else if(instr.flags & GenOp_SignExtend)
                                builder.movsx(static_cast<Reg32>(*dst), Reg8::AL);
                            else if(zeroExtend)
                                builder.movzx(static_cast<Reg32>(*dst), Reg8::AL);
                            else
                                builder.mov(*dst, Reg8::AL);
                        }
                        else
                        {
                            // 16/32-bit dest
                            bool saved = isCallSaved(*dst);
                            if(saved) // restore everything except RAX
                                callRestoreIfNeeded(builder, static_cast<Reg16>(callSavedRegs[1]), needCallRestore);

                            auto dst32 = static_cast<Reg32>(static_cast<int>(*dst) & 0xF); // bit of a hack

                            if(instr.opcode == GenOpcode::Load2 && (instr.flags & GenOp_SignExtend))
                                builder.movsx(dst32, Reg16::AX);
                            else if(dst32 != Reg32::EAX)
                                builder.mov(dst32, Reg32::EAX);

                            if(dst32 == Reg32::EAX)
                            {
                                // discard old RAX
                                builder.pop(Reg64::R10);
                                needCallRestore = 0;
                            }
                        }
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
                int dataSize = 8;
                if(instr.opcode == GenOpcode::Store2)
                    dataSize = 16;
                else if(instr.opcode == GenOpcode::Store4)
                    dataSize = 32;

                if(addrSize == 16 || addrSize == 32)
                {
                    auto addr = checkRegOrImm32(instr.src[0], Reg32::R8D);

                    if(addr.index() && checkReg(instr.src[1], dataSize, true))
                    {
                        callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);
                        callSaveIfNeeded(builder, needCallRestore);

                        bool needCyclesSeq = sourceInfo.writeMem8;

#ifndef _WIN32
                        // setup addr first (data writes DX)
                        setupMemAddr(addr, addrSize, instr.src[0]);
#endif
                        
                        if(instr.opcode == GenOpcode::Store)
                        {
                            // TODO: this is mostly checkRegOrImm8, without the check
                            std::variant<std::monostate, Reg8, uint8_t> data;

                            if(instr.src[1] == 0)
                            {
                                if(auto imm = getLastImmLoad())
                                {
                                    assert(!(*imm & 0xFFFFFF00));
                                    data = static_cast<uint8_t>(*imm);
                                }
                            }

                            if(!data.index())
                                data = *mapReg8(instr.src[1]);

                            if(std::holds_alternative<Reg8>(data))
                            {
#ifdef _WIN32
                                // argumentRegs[2] is R8, can't mov from xH to there
                                auto reg8 = std::get<Reg8>(data);
                                if(isXHReg(reg8))
                                {
                                    builder.mov(Reg8::AL, reg8);
                                    data = Reg8::AL;
                                }
#endif
                                builder.movzx(argumentRegs32[2], std::get<Reg8>(data));
                            }
                            else
                                builder.mov(argumentRegs32[2], std::get<uint8_t>(data));
                        }
                        else
                        {
                            auto data32 = mapReg32(instr.src[1]); // already checked

                            if(!data32)
                                loadExtraReg32(instr.src[1], argumentRegs32[2]);
                            else if(instr.opcode == GenOpcode::Store2)
                                builder.movzx(argumentRegs32[2], static_cast<Reg16>(*data32));
                            else
                                builder.mov(argumentRegs32[2], *data32);
                        }

#ifdef _WIN32
                        // setup addr after data (addr writes DX)
                        setupMemAddr(addr, addrSize, instr.src[0]);
#endif

                        if(needCyclesSeq)
                        {
                            // FIXME: win32

                            // cycle count from stack
                            builder.lea(argumentRegs64[3], {Reg64::RSP, needCallRestore * 8});
                            writeCycleCountToStack(instrCycles);

                            builder.mov(argumentRegs32[4], instr.flags >> 8);

                            builder.mov(argumentRegs32[5], Reg32::EDI); // cycle count
                        }
                        else
                            builder.mov(argumentRegs32[3], Reg32::EDI); // cycle count

                        // select function to call
                        uintptr_t func = 0;

                        if(instr.opcode == GenOpcode::Store)
                        {
                            if(sourceInfo.writeMem)
                                func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem);
                            else
                                func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem8);
                        }
                        else if(instr.opcode == GenOpcode::Store2)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem16);
                        else if(instr.opcode == GenOpcode::Store4)
                            func = reinterpret_cast<uintptr_t>(sourceInfo.writeMem32);

                        if(!func)
                        {
                            printf("unhandled store\n");
                            err = true;
                        }

                        builder.mov(Reg64::RAX, func); // function ptr

                        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr
                        builder.call(Reg64::RAX); // do call

                        // just pop EDI... then overrwrite it
                        callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);
                        builder.mov(Reg32::EDI, Reg32::EAX);
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
                    if(instr.flags & GenOp_WriteFlags)
                        checkSingleSource();

                    auto src = checkRegOrImm32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D); // TODO: only load if src0 == dst

                    bool done = false;

                    if(!(instr.flags & GenOp_WriteFlags) && instr.src[0] != instr.dst[0])
                    {
                        // we don't need the flags, so use LEA to avoid moves
                        auto src0 = checkRegOrImm32(instr.src[0], Reg32::R9D);

                        if(src0.index() && src.index() && dst)
                        {
                            bool src0Imm = std::holds_alternative<uint32_t>(src0);
                            bool src1Imm =  std::holds_alternative<uint32_t>(src);
                            assert(!src0Imm || !src1Imm);

                            if(src0Imm || src1Imm)
                            {
                                auto imm = std::get<uint32_t>(src0Imm ? src0 : src);
                                auto reg = std::get<Reg32>(src0Imm ? src : src0);
                                builder.lea(*dst, {static_cast<Reg64>(reg), static_cast<int>(imm)});
                            }
                            else
                            {
                                auto reg0 = std::get<Reg32>(src0);
                                auto reg1 = std::get<Reg32>(src);
                                builder.lea(*dst, {static_cast<Reg64>(reg0), static_cast<Reg64>(reg1)});
                            }

                            // write back extra reg
                            if(*dst == Reg32::R9D)
                                storeExtraReg32(instr.dst[0], *dst);

                            done = true;
                        }
                    }

                    if(src.index() && dst && !done)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80)
                                builder.add(*dst, static_cast<int8_t>(imm));
                            else
                                builder.add(*dst, imm);
                        }
                        else
                            builder.add(*dst, std::get<Reg32>(src));

                        // copy C flag if we need it
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::B, Reg8::R11B);

                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags);

                        // write back extra reg
                        if(*dst == Reg32::R9D)
                            storeExtraReg32(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    checkSingleSource();

                    auto src = checkRegOrImm16(instr.src[1]);
                    auto dst = checkReg16(instr.dst[0]);
                    
                    if(src.index() && dst)
                    {
                        // calc half carry
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                        {
                            // dst & 0xFFF
                            builder.mov(Reg32::ESI, static_cast<Reg32>(*dst));
                            builder.and_(Reg32::ESI, 0xFFFu);

                            // src & 0xFFF
                            if(std::holds_alternative<uint16_t>(src))
                                builder.mov(Reg32::R11D, std::get<uint16_t>(src) & 0xFFF);
                            else
                            {
                                builder.mov(Reg32::R11D, static_cast<Reg32>(std::get<Reg16>(src)));
                                builder.and_(Reg32::R11D, 0xFFFu);
                            }

                            // "half" add
                            builder.add(Reg32::ESI, Reg32::R11D);

                            // compare and set h bit
                            builder.cmp(Reg32::ESI, uint32_t(0xFFF));
                            builder.setcc(Condition::A, Reg8::SIL);
                            builder.shl(Reg8::SIL, getFlagInfo(SourceFlagType::HalfCarry).bit);
                        }

                        if(std::holds_alternative<uint16_t>(src))
                        {
                            auto srcImm = std::get<uint16_t>(src);
                            if(srcImm == 1 && !(writesFlag(instr.flags, SourceFlagType::Carry)))
                                builder.inc(*dst);
                            else
                            {
                                // currently signed 8-bit only
                                assert(srcImm < 128 || srcImm >= 0xFF80);
                                builder.add(*dst, srcImm);
                            }
                        }
                        else
                            builder.add(*dst, std::get<Reg16>(src));

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags({}, flags, flagWriteMask(SourceFlagType::Carry), 0, preserveMask);

                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*mapReg8(flagsReg), Reg8::SIL);
                    }
                }
                else if(regSize == 8)
                {
                    checkSingleSource();

                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        // calc half carry
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            calcHalfCarry8(instr.opcode, *dst, src, {});

                        doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::add), [this, &instr](X86Builder &builder, Reg8 dst, uint8_t src)
                        {
                            if(src == 1 && !(writesFlag(instr.flags, SourceFlagType::Carry)))
                                builder.inc(dst);
                            else
                                builder.add(dst, src);
                        });

                        // flags
                        bool setZ = !(instr.flags & GenOp_MagicAlt1); // LDHL SP/ADD SP use an 8-bit add to set flags, but set Z=0
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, (setZ ? flagWriteMask(SourceFlagType::Zero) : 0) | flagWriteMask(SourceFlagType::Carry), 0, preserveMask);

                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*mapReg8(flagsReg), Reg8::SIL);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::AddWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D);

                    if(src && dst)
                    {
                        // carry in (and clear it)
                        builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);

                        builder.adc(*dst, *src);

                        // copy C flag if we need it
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::B, Reg8::R11B);
                        
                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        // calc half carry
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            calcHalfCarry8(instr.opcode, *dst, src, f);

                        carryIn(*f);
                        doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::adc), std::mem_fn<ImmOp8>(&X86Builder::adc));

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero) | flagWriteMask(SourceFlagType::Carry), 0, preserveMask);

                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*f, Reg8::SIL);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::And:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource(true);

                if(regSize == 32)
                {
                    auto src = checkRegOrImm32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80 || imm >= 0xFFFFFF80)
                                builder.and_(*dst, static_cast<int8_t>(imm));
                            else 
                                builder.and_(*dst, imm);
                        }
                        else
                            builder.and_(*dst, std::get<Reg32>(src));

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                    }
                }
                else if(regSize == 8)
                {
                    // d = s0 & d -> d = d & s0
                    int srcIndex = instr.src[1] == instr.dst[0] ? 0 : 1;

                    auto src = checkRegOrImm8(instr.src[srcIndex]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::and_), std::mem_fn<ImmOp8>(&X86Builder::and_)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags({}, flags, flagWriteMask(SourceFlagType::Zero), flagWriteMask(SourceFlagType::HalfCarry), preserveMask);
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
                    auto src = checkRegOrImm32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.src[0], Reg32::R9D);
                    if(src.index() && dst)
                    {
                        // move the dest and do a sub to make flags handling easier
                        // could simplify if only one flag is used?
                        builder.mov(Reg32::R11D, *dst);
                    
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80)
                                builder.sub(Reg32::R11D, static_cast<int8_t>(imm));
                            else
                                builder.sub(Reg32::R11D, imm);
                        }
                        else
                            builder.sub(Reg32::R11D, std::get<Reg32>(src));

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R10B);

                        // flags
                        setFlags32(Reg32::R11D, Reg32::R10D, instr.flags, true);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        auto cMask = flagWriteMask(SourceFlagType::Carry);
                        auto zMask = flagWriteMask(SourceFlagType::Zero);

                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            calcHalfCarry8(instr.opcode, *dst, src, {});

                        doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::cmp), std::mem_fn<ImmOp8>(&X86Builder::cmp));

                        if((instr.flags & (cMask | zMask)) == (cMask | zMask))
                            builder.setcc(Condition::E, Reg8::R10B); // save zero if carry also set, should be safe to use tmp here

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags({}, flags, cMask | zMask, flagWriteMask(SourceFlagType::WasSub), preserveMask);

                        // half carry
                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*f, Reg8::SIL);

                        if(flags & zMask)
                        {
                            builder.shl(Reg8::R10B, getFlagInfo(SourceFlagType::Zero).bit);
                            builder.or_(*f, Reg8::R10B);
                        }
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Multiply:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src && dst)
                    {
                        builder.imul(*dst, *src);
                    
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags, false, 0);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Or:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm32(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            // use sign extended if imm < 0x80? 
                            builder.or_(*dst, imm);
                        }
                        else
                            builder.or_(*dst, std::get<Reg32>(src));
                    
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                    }
                }
                else if(regSize == 16)
                {
                    // used for LDH
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src = checkRegOrImm16(instr.src[1]);
                        auto dst = checkReg16(instr.dst[0]);

                        if(src.index() && dst)
                        {
                            if(std::holds_alternative<uint16_t>(src))
                                builder.or_(static_cast<Reg32>(*dst), std::get<uint16_t>(src));
                            else
                                builder.or_(*dst, std::get<Reg16>(src));
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::or_), std::mem_fn<ImmOp8>(&X86Builder::or_)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags({}, flags, flagWriteMask(SourceFlagType::Zero), 0, preserveMask);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Subtract:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                bool swapped = checkSingleSource(regSize == 32);

                if(regSize == 32)
                {
                    auto src = checkRegOrImm32(instr.src[swapped ? 0 : 1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D);

                    if(src.index() && dst)
                    {
                        // we lied about being able to use swapped sources
                        if(swapped)
                        {
                            // src1 == dst
                            if(std::holds_alternative<uint32_t>(src))
                            {
                                // immediate src0
                                // do negate for 0 - x?
                                auto tmp = mapReg32(0);
                                builder.mov(*tmp, *dst);
                                builder.mov(*dst, std::get<uint32_t>(src));
                                src = *tmp;
                            }
                            else if(instr.src[1] == 0)
                            {
                                // immediate src1, move src to dest
                                builder.mov(*dst, std::get<Reg32>(src));
                                src = *getLastImmLoad(); 
                            }
                            else
                            {
                                // both reg sources
                                // save dst, move src to dst
                                auto tmp = mapReg32(0);
                                builder.mov(*tmp, *dst);
                                builder.mov(*dst, std::get<Reg32>(src));                                
                                src = *tmp;
                            }
                        }

                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80)
                                builder.sub(*dst, static_cast<int8_t>(imm));
                            else
                                builder.sub(*dst, imm);
                        }
                        else
                            builder.sub(*dst, std::get<Reg32>(src));

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R11B);

                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags, true);

                        // write back extra reg
                        if(*dst == Reg32::R9D)
                            storeExtraReg32(instr.dst[0], *dst);
                    }
                }
                else if(regSize == 16)
                {
                    // should be DEC, which sets no flags
                    if(instr.flags & (GenOp_PreserveFlags | GenOp_WriteFlags))
                        unhandledFlags(instr.flags & (GenOp_PreserveFlags | GenOp_WriteFlags));
                    else
                    {
                        auto src = checkRegOrImm16(instr.src[1]);
                        auto dst = checkReg16(instr.dst[0]);

                        if(src.index() && dst)
                        {
                            if(std::holds_alternative<uint16_t>(src))
                            {
                                assert(std::get<uint16_t>(src) == 1); // should be DEC
                                builder.dec(*dst);
                            }
                            else
                                builder.sub(*dst, std::get<Reg16>(src));
                        }
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        // calc half carry
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            calcHalfCarry8(instr.opcode, *dst, src, {});

                        doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::sub), [this, &instr](X86Builder &builder, Reg8 dst, uint8_t src)
                        {
                            if(src == 1 && !(writesFlag(instr.flags, SourceFlagType::Carry)))
                                builder.dec(dst);
                            else
                                builder.sub(dst, src);
                        });

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero) | flagWriteMask(SourceFlagType::Carry), flagWriteMask(SourceFlagType::WasSub), preserveMask);

                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*mapReg8(flagsReg), Reg8::SIL);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::SubtractWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D);

                    if(src && dst)
                    {
                        // carry in (and clear it)
                        builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                        builder.cmc(); // inverted

                        builder.sbb(*dst, *src);

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R11B);

                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags, true);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);
                    auto f = checkReg8(flagsReg);

                    if(src.index() && dst && f)
                    {
                        // calc half carry
                        if(writesFlag(instr.flags, SourceFlagType::HalfCarry))
                            calcHalfCarry8(instr.opcode, *dst, src, f);

                        carryIn(*f);
                        doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::sbb), std::mem_fn<ImmOp8>(&X86Builder::sbb));

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero) | flagWriteMask(SourceFlagType::Carry), flagWriteMask(SourceFlagType::WasSub), preserveMask);

                        if(writesFlag(flags, SourceFlagType::HalfCarry))
                            builder.or_(*f, Reg8::SIL);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Xor:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm32(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            // use sign extended if imm < 0x80? 
                            builder.xor_(*dst, imm);
                        }
                        else
                            builder.xor_(*dst, std::get<Reg32>(src));

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                        
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmOp8(builder, dst, src, std::mem_fn<RegOp8>(&X86Builder::xor_), std::mem_fn<ImmOp8>(&X86Builder::xor_)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        bool alwaysZero = instr.src[1] == instr.dst[0]; // x ^ x is always 0
                        auto zMask = flagWriteMask(SourceFlagType::Zero);

                        setFlags({}, flags, alwaysZero ? 0 : zMask, alwaysZero ? zMask : 0, preserveMask);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Not:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    if(auto dst = checkReg32(instr.dst[0]))
                    {
                        builder.not_(*dst);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        setFlags32(*dst, {}, instr.flags, false, 0); // doesn't affect flags
                    }
                }
                else if(regSize == 8)
                {
                    if(auto dst = checkReg8(instr.dst[0]))
                    {
                        builder.not_(*dst);

                        // flags (sets H and N to 1)
                        if((instr.flags & GenOp_WriteFlags))
                            builder.or_(*mapReg8(flagsReg), (1 << getFlagInfo(SourceFlagType::HalfCarry).bit) | (1 << getFlagInfo(SourceFlagType::WasSub).bit));
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1); // RLCA sets Z to 0, RLC A sets Z based on the result
                    bool setC = !(instr.flags & GenOp_MagicAlt2); // SWAP sets C to 0 (translated to rot by 4)

                    if(doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::rolCL), std::mem_fn<ImmOp8>(&X86Builder::rol)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, (setC ? flagWriteMask(SourceFlagType::Carry) : 0) | (setZ ? flagWriteMask(SourceFlagType::Zero) : 0), 0, preserveMask, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeftCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);
                    auto f = checkReg8(flagsReg);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(src.index() && dst && f)
                    {
                        carryIn(*f);
                        doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::rclCL), std::mem_fn<ImmOp8>(&X86Builder::rcl));

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | (setZ ? flagWriteMask(SourceFlagType::Zero) : 0), 0, preserveMask, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRight:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::rorCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::ror), preOp, true))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = flagWriteMask(SourceFlagType::Carry); // only have carry
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::rorCL), std::mem_fn<ImmOp8>(&X86Builder::ror)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | (setZ ? flagWriteMask(SourceFlagType::Zero) : 0), 0, preserveMask, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRightCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);
                    auto f = checkReg8(flagsReg);

                    bool setZ = !(instr.flags & GenOp_MagicAlt1);

                    if(src.index() && dst && f)
                    {
                        carryIn(*f);
                        doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::rcrCL), std::mem_fn<ImmOp8>(&X86Builder::rcr));

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | (setZ ? flagWriteMask(SourceFlagType::Zero) : 0), 0, preserveMask, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shlCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shl), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else if(regSize == 16)
                {
                    // used for RET
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src = checkRegOrImm8(instr.src[1]);
                        auto dst = checkReg32(instr.dst[0]);

                        doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shlCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shl), {});
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::shlCL), std::mem_fn<ImmOp8>(&X86Builder::shl)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero), 0, preserveMask);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightArith:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::sarCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::sar), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::sarCL), std::mem_fn<ImmOp8>(&X86Builder::sar)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero), 0, preserveMask);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightLogic:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shrCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shr), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else if(regSize == 16)
                {
                    // used for LD (nn), SP
                    if(instr.flags & GenOp_WriteFlags)
                        unhandledFlags(instr.flags & GenOp_WriteFlags);
                    else
                    {
                        auto src = checkRegOrImm8(instr.src[1]);
                        auto dst = checkReg32(instr.dst[0]);

                        doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shrCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shr), {});
                    }
                }
                else if(regSize == 8)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(doRegImmShift8(builder, dst, src, std::mem_fn<RegUnOp8>(&X86Builder::shrCL), std::mem_fn<ImmOp8>(&X86Builder::shr)))
                    {
                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero), 0, preserveMask);
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

                if(regSize == 16 || regSize == 32)
                {
                    auto src = checkRegOrImm32(instr.src[1]);
                    if(src.index())
                    {
                        callRestoreIfNeeded(builder, needCallRestore);

                        assert(isExit || std::holds_alternative<uint32_t>(src)); // shouldn't be any non-exit jumps with unknown addr

                        uint8_t *branchPtr = nullptr;
                        bool flagSet = false;

                        // condition
                        if(condition != GenCondition::Always)
                        {
                            syncCyclesExecuted();

                            std::variant<Reg8, Reg32> f;

                            if(regSize == 8)
                                f = *mapReg8(flagsReg); // really should not get here without a valid flags reg
                            else if(regSize == 32)
                                f = *mapReg32(flagsReg);

                            auto testFlag = [this, &builder, &f](SourceFlagType flag)
                            {
                                std::visit([this, &builder, flag](auto &&r){builder.test(r, 1 << getFlagInfo(flag).bit);}, f);
                            };

                            flagSet = !(static_cast<int>(condition) & 1); // even conds are set, odd are clear

                            switch(condition)
                            {
                                case GenCondition::Equal:
                                case GenCondition::NotEqual:
                                    testFlag(SourceFlagType::Zero);
                                    break;
                                case GenCondition::CarrySet:
                                case GenCondition::CarryClear:
                                    testFlag(SourceFlagType::Carry);
                                    break;
                                case GenCondition::Negative:
                                case GenCondition::Positive:
                                    testFlag(SourceFlagType::Negative);
                                    break;
                                case GenCondition::OverflowSet:
                                case GenCondition::OverflowClear:
                                    testFlag(SourceFlagType::Overflow);
                                    break;

                                case GenCondition::Higher:
                                case GenCondition::LowerSame:
                                {
                                    // C && !Z (higher)

                                    // (flags >> C.bit) & (Z.bit | C.bit) == 1
                                    assert(getFlagInfo(SourceFlagType::Zero).bit > getFlagInfo(SourceFlagType::Carry).bit);

                                    auto carryBit = getFlagInfo(SourceFlagType::Carry).bit;
                                    auto mask = (1 << (getFlagInfo(SourceFlagType::Zero).bit - carryBit)) | 1;

                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, carryBit);
                                    builder.and_(Reg8::R11B, mask);
                                    builder.cmp(Reg8::R11B, 1);

                                    flagSet = !flagSet; // oops, backwards
                                    break;
                                }

                                case GenCondition::GreaterEqual:
                                case GenCondition::LessThan:
                                {
                                    // N == V (greater equal)
                                    // N != V (less than)

                                    // shift N to where V is
                                    assert(getFlagInfo(SourceFlagType::Negative).bit > getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, getFlagInfo(SourceFlagType::Negative).bit - getFlagInfo(SourceFlagType::Overflow).bit);
                                    
                                    // xor
                                    builder.xor_(Reg32::R11D, std::get<Reg32>(f));
                                    
                                    flagSet = !flagSet; // oops, backwards
                                    f = Reg32::R11D;
                                    testFlag(SourceFlagType::Overflow); // test the result
                                    break;
                                }

                                case GenCondition::GreaterThan:
                                case GenCondition::LessThanEqual:
                                {
                                    // N == V && !Z (greater than)
                                    // N != V || Z (less than equal)

                                    // shift N to where V is
                                    assert(getFlagInfo(SourceFlagType::Negative).bit > getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, getFlagInfo(SourceFlagType::Negative).bit - getFlagInfo(SourceFlagType::Overflow).bit);
                                    
                                    // xor
                                    builder.xor_(Reg32::R11D, std::get<Reg32>(f));

                                    // test result and Z
                                    auto mask = (1 << getFlagInfo(SourceFlagType::Zero).bit) | (1 << getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.test(Reg32::R11D, mask);

                                    flagSet = !flagSet; // oops, backwards
                                    break;
                                }

                                default:
                                    printf("unhandled cond %i\n", static_cast<int>(condition));
                                    err = true;
                            }

                            branchPtr = builder.getPtr();
                            builder.jcc(Condition::E, 1); // patch later
                        }

                        // need to sync cycles *before* the jump out
                        if(instrCycles)
                        {
                            assert(instrCycles == 1 || !(blockInfo.flags & GenBlock_StrictSync));
                            while(instrCycles--)
                                cycleExecuted();
                        }
                        syncCyclesExecuted();

                        callRestoreIfNeeded(builder, needCallRestore); // we might have just done another cycleExecuted call

                        // set pc if we're exiting
                        if(isExit)
                        {
                            if(std::holds_alternative<uint32_t>(src))
                                builder.mov(pcReg32, std::get<uint32_t>(src) + sourceInfo.pcPrefetch);
                            else if(std::get<Reg32>(src) != pcReg32)
                            {
                                if(regSize == 16)
                                    builder.movzx(pcReg32, static_cast<Reg16>(std::get<Reg32>(src)));
                                else if(sourceInfo.pcPrefetch)
                                    builder.lea(pcReg32, {static_cast<Reg64>(std::get<Reg32>(src)), sourceInfo.pcPrefetch});
                                else
                                    builder.mov(pcReg32, std::get<Reg32>(src));
                            }
                        }
                        else // or sub cycles early (we jump past the usual code that does this)
                            builder.sub(Reg32::EDI, static_cast<int8_t>(cyclesThisInstr));

                        // don't update twice for unconditional branches
                        if(condition == GenCondition::Always)
                            cyclesThisInstr = 0;

                        auto it = std::holds_alternative<uint32_t>(src) ? branchTargets.find(std::get<uint32_t>(src)) : branchTargets.end();

                        if(it != branchTargets.end())
                        {
                            // backwards branch
                            builder.jmp(it->second - builder.getPtr());
                        }
                        else
                        {
                            if(!isExit)
                                forwardBranchesToPatch.emplace(std::get<uint32_t>(src), builder.getPtr());

                            // forwards branch or exit
                            if(isExit && (instr.flags & GenOp_Call) && (instIt + 1) != endInstr)
                                builder.call(exitForCallPtr - builder.getPtr()); // call, save when exiting
                            else
                                builder.jmp(exitPtr - builder.getPtr(), !isExit); // patched later if not exit
                        }

                        // patch the condition jump
                        if(branchPtr && !builder.getError())
                        {
                            auto off = builder.getPtr() - branchPtr - 2;
                            builder.patch(branchPtr, branchPtr + 2);
                            builder.jcc(flagSet ? Condition::E : Condition::NE, off);
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
                auto a = *mapReg8(6); // magic!

                assert(f == Reg8::AL && a == Reg8::AH);

                auto cMask = 1 << getFlagInfo(SourceFlagType::Carry).bit;
                auto hMask = 1 << getFlagInfo(SourceFlagType::HalfCarry).bit;
                auto nMask = 1 << getFlagInfo(SourceFlagType::WasSub).bit;
                auto zMask = 1 << getFlagInfo(SourceFlagType::Zero).bit;

                // clear Z flag (will add back later)
                builder.and_(f, zMask ^ 0xFF);

                builder.test(f, nMask);
                builder.jcc(Condition::E, 18); // N not set

                // negative

                // if (Flag_C) A -= 0x60
                builder.test(f, cMask);
                builder.jcc(Condition::E, 03); // C not set
                builder.sub(a, 0x60);

                // if (Flag_H) A -= 0x06
                builder.test(f, hMask);
                builder.jcc(Condition::E, 3); // C not set
                builder.sub(a, 0x06);

                builder.jmp(43); // skip

                // positive

                // if (Flag_C ...
                builder.test(f, cMask);
                builder.jcc(Condition::NE, 5); // C set
                // ... || A > 0x99) ...
                builder.cmp(a, 0x99);
                builder.jcc(Condition::BE, 6);
                // ... A += 0x60
                builder.add(a, 0x60);
                builder.or_(f, cMask); // set C flag

                // if (Flag_H ...
                builder.test(f, hMask);
                builder.jcc(Condition::NE, 19); // H set
                // ... || A & (0x0F) > 0x09) ...
                builder.mov(Reg32::R10D, static_cast<Reg32>(f)); // move the whole thing as we can't easily mov just the high byte
                builder.and_(Reg32::R10D, 0xF00u);
                builder.cmp(Reg32::R10D, uint32_t(0x0900));
                builder.jcc(Condition::BE, 3);
                // .. A += 0x06
                builder.add(a, 0x06);

                // Z flag
                builder.and_(a, a);
                builder.jcc(Condition::NE, 3);
                builder.or_(f, zMask);

                // clear H flag
                builder.and_(f, ~hMask);

                break;
            }

            case GenOpcode::DMG_Halt:
            {
                // halted = true
                builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[2]/*halted*/}, uint8_t(1));

                // if(!masterInterruptEnable && serviceableInterrupts)
                builder.cmp({cpuPtrReg, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/}, 0);
                builder.jcc(Condition::NE, 12);
                builder.cmp({cpuPtrReg, sourceInfo.extraCPUOffsets[3]/*serviceableInterrupts*/}, 0);
                builder.jcc(Condition::E, 5);
                // haltBug = true
                builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[4]/*haltBug*/}, uint8_t(1));

                // exit
                syncCyclesExecuted();
                builder.mov(pcReg32, pc); // exits need to set PC themselves
                builder.call(saveAndExitPtr - builder.getPtr());

                break;
            }

            case GenOpcode::DMG_EnableIntrForRet:
                // masterInterruptEnable = true
                builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/}, uint8_t(1));
                break;

            case GenOpcode::DMG_DI:
                // masterInterruptEnable = false
                // TODO: after next instruction (DMGCPU also has this TODO)
                builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[0]/*masterInterruptEnable*/}, uint8_t(0));
                break;

            case GenOpcode::DMG_EI:
                // enableInterruptsNextCycle = true
                builder.mov({cpuPtrReg, sourceInfo.extraCPUOffsets[1]/*enableInterruptsNextCycle*/}, uint8_t(1));
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
            builder.mov(pcReg32, pc - instr.len + sourceInfo.pcPrefetch);
            builder.jmp(exitPtr - builder.getPtr());
            break;
        }

        // sync cycle count from stack at the end of the op, or if something updates cycles before that
        if(stackCycleCount && (instrCycles || instr.len || (instIt + 1)->opcode == GenOpcode::Jump))
        {
            callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);

            // add/sub the cycles from the stack
            builder.mov(Reg32::R8D, {Reg64::RSP, needCallRestore * 8});

            int offset = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - reinterpret_cast<uintptr_t>(cpuPtr);
            builder.add({cpuPtrReg, offset}, Reg32::R8D);

            builder.sub(Reg32::EDI, Reg32::R8D);
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
            callRestoreIfNeeded(builder, needCallRestore);
            syncCyclesExecuted();
            builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
            builder.jmp(exitPtr - builder.getPtr());
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
            callRestoreIfNeeded(builder, needCallRestore);
            syncCyclesExecuted();

            // exit may be forced after interrupts are enabled
            if(forceExitAfter)
                builder.jmp(cyclesThisInstr ? 5 : 2);

            // cycles -= executed
            if(cyclesThisInstr) // 0 means we already did the sub
                builder.sub(Reg32::EDI, static_cast<int8_t>(cyclesThisInstr));

            lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

            // if <= 0 exit
            builder.jcc(Condition::G, 10);
            builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
            builder.call(saveAndExitPtr - builder.getPtr());

            cyclesThisInstr = 0;
            forceExitAfter = false;
        }

        if(newEmuOp)
            numInstructions++;
    }

    if(builder.getError())
    {
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
    int len = endPtr - codePtr;

    //debug
    printf("recompile @%04X generated %i bytes (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr; p != endPtr; p++)
        printf(" %02X", *p);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);
#endif

    codePtr = endPtr;

    return true;
}

uint8_t *X86Target::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
{
    X86Builder builder(codeBuf, codeBuf + codeBufSize);

    // prologue
    builder.push(Reg64::RBP);

    // save
    builder.push(Reg64::R12);
    builder.push(Reg64::R13);
    builder.push(Reg64::R14);
    if(saveR15)
        builder.push(Reg64::R15);

    builder.push(Reg64::RBX);

#ifdef _WIN32
    builder.push(Reg64::RSI);
    builder.push(Reg64::RDI);

    builder.mov(Reg64::RDI, argumentRegs64[0]); // move cycle count
#endif

    if(sourceInfo.readMem8)
        builder.sub(Reg64::RSP, 16); // save some space if we're using the more advanced memory access

    builder.mov(Reg64::R10, argumentRegs64[1]); // code ptr

    // store pointer to CPU
    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);
    builder.mov(cpuPtrReg, cpuPtrInt);


    // load emu regs
    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16 || reg.size == 32);

             if(auto reg32 = mapReg32(i))
             {
                if(reg.size == 16)
                    builder.movzxW(*reg32, {cpuPtrReg, reg.cpuOffset});
                else if(reg.size == 32)
                    builder.mov(*reg32, {cpuPtrReg, reg.cpuOffset});
             }
        }

        i++;
    }

    // jump to code
    builder.jmp(Reg64::R10);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.exitCallFlag));
    builder.mov({Reg64::R11, 0}, uint8_t(1));

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.savedExitPtr));
    builder.mov({Reg64::R11, 0}, Reg64::R10);

    // just exit
    exitPtr = builder.getPtr();

    // save emu regs
    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16 || reg.size == 32);

            if(reg.size == 16)
            {
                if(auto reg16 = mapReg16(i))
                    builder.mov({cpuPtrReg, reg.cpuOffset}, *reg16);
            }
            else if(reg.size == 32)
            {
                if(auto reg32 = mapReg32(i))
                    builder.mov({cpuPtrReg, reg.cpuOffset}, *reg32);
            }
        }

        i++;
    }

    // save emu pc
    if(sourceInfo.pcSize == 16)
        builder.mov({cpuPtrReg, sourceInfo.pcOffset}, pcReg16);
    else if(sourceInfo.pcSize == 32)
        builder.mov({cpuPtrReg, sourceInfo.pcOffset}, pcReg32);

    // restore

    if(sourceInfo.readMem8)
        builder.add(Reg64::RSP, 16);

#ifdef _WIN32
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RSI);
#endif

    builder.pop(Reg64::RBX);

    if(saveR15)
        builder.pop(Reg64::R15);
    builder.pop(Reg64::R14);
    builder.pop(Reg64::R13);
    builder.pop(Reg64::R12);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codeBuf;

    //debug
    printf("generated %i bytes for entry/exit\ncode:", len);

    for(auto p = codeBuf; p != codeBuf + len; p++)
        printf(" %02X", *p);

    printf("\n");
#endif

    auto ret = codeBuf;

    codeBuf = builder.getPtr();

    return ret;
}

std::optional<Reg8> X86Target::mapReg8(uint8_t index)
{
    // remap aliases
    auto &reg = sourceInfo.registers[index];
    if(reg.alias)
        index = reg.alias;

    if(auto reg32 = mapReg32(index))
    {
        // default to low
        if(!reg.alias || reg.aliasMask == 0xFF)
        {
            // SPL/BPL/SIL/DIL (require REX)
            if(*reg32 == Reg32::ESP || *reg32 == Reg32::EBP || *reg32 == Reg32::ESI || *reg32 == Reg32::EDI)
                return static_cast<Reg8>(static_cast<int>(*reg32) + 0x10);
    
            return static_cast<Reg8>(*reg32);
        }

        // handle high aliases
        if(reg.aliasMask == 0xFF00 && (*reg32 == Reg32::EAX || *reg32 == Reg32::ECX || *reg32 == Reg32::EDX || *reg32 == Reg32::EBX))
            return static_cast<Reg8>(static_cast<int>(*reg32) + 4);

        return {};
    }

    return {};
}

std::optional<Reg16> X86Target::mapReg16(uint8_t index)
{
    if(auto reg32 = mapReg32(index))
        return static_cast<Reg16>(*reg32);

    return {};
}

std::optional<Reg32> X86Target::mapReg32(uint8_t index)
{
    if(sourceInfo.registers[index].alias)
        return {};

    auto alloc = regAlloc.find(index);
    if(alloc != regAlloc.end())
        return alloc->second;

    return {};
}

std::optional<Reg64> X86Target::mapReg64(uint8_t index)
{
    if(auto reg32 = mapReg32(index))
        return static_cast<Reg64>(*reg32);

    return {};
}

SourceFlagInfo &X86Target::getFlagInfo(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    assert(flagMap[iFlag] != 0xFF);

    return sourceInfo.flags[flagMap[iFlag]];
}

uint8_t X86Target::flagWriteMask(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    if(flagMap[iFlag] == 0xFF) // source does not have this flag
        return 0;

    return 1 << (flagMap[iFlag] + 4);
}

bool X86Target::writesFlag(uint16_t opFlags, SourceFlagType flag)
{
    return opFlags & flagWriteMask(flag);
}


bool X86Target::needStackAlign() const
{
    // if R15 is saved the stack is misaligned before saving anything
    // otherwise if we're saving an odd number of registiers it's misaligned
    return !!(numSavedRegs % 2) != saveR15;
}

bool X86Target::isCallSaved(Reg16 reg) const
{
    return callSaveIndex(reg) != -1;
}

bool X86Target::isCallSaved(Reg8 reg) const
{
    return callSaveIndex(reg) != -1;
}

int X86Target::callSaveIndex(Reg16 reg) const
{
    for(size_t i = 0; i < numSavedRegs; i++)
    {
        if(reg == static_cast<Reg16>(callSavedRegs[i]))
            return static_cast<int>(i);
    }
    
    return -1;
}

int X86Target::callSaveIndex(Reg8 reg) const
{
    Reg16 mappedReg = static_cast<Reg16>(reg);
    auto iReg = static_cast<int>(reg);

    if(iReg >= 16) // SPL/BPL/SIL/DIL
        mappedReg = static_cast<Reg16>(iReg - 16);
    else if(isXHReg(reg))
        mappedReg = static_cast<Reg16>(iReg - 4);

    return callSaveIndex(mappedReg);
}

void X86Target::callSaveIfNeeded(X86Builder &builder, int &saveState) const
{
    int numRegs = static_cast<int>(numSavedRegs);

    if(saveState == numRegs)
        return;

    for(int i = saveState; i < numRegs; i++)
        builder.push(callSavedRegs[i]);

    if(needStackAlign())
        builder.sub(Reg64::RSP, 8); // align stack

#ifdef _WIN32
    builder.sub(Reg64::RSP, 32); // shadow space
#endif

    saveState = numRegs;
}

void X86Target::callRestore(X86Builder &builder, int &saveState, int toIndex) const
{
    if(saveState <= toIndex)
        return;

    int numRegs = static_cast<int>(numSavedRegs);

    assert(toIndex < numRegs);

#ifdef _WIN32
    if(saveState == numRegs)
        builder.add(Reg64::RSP, 32); // shadow space
#endif

    if(saveState == numRegs && needStackAlign())
        builder.add(Reg64::RSP, 8); // alignment

    for(int i = saveState - 1; i >= toIndex; i--)
        builder.pop(callSavedRegs[i]);

    saveState = toIndex;
}

void X86Target::callRestore(X86Builder &builder, Reg8 dstReg, bool zeroExtend, bool signExtend) const
{
    assert(dstReg != Reg8::DIL); // no

    assert(isCallSaved(dstReg));

    assert(!zeroExtend || !signExtend); // both makes no sense

#ifdef _WIN32
    builder.add(Reg64::RSP, 32); // shadow space
#endif

    if(needStackAlign())
        builder.add(Reg64::RSP, 8); // alignment

    // pop everything except RAX
    for(unsigned i = numSavedRegs - 1; i > 0; i--)
        builder.pop(callSavedRegs[i]);

    assert(callSavedRegs[0] == Reg64::RAX);

    // mov ret val (if not going to RAX)
    if(dstReg != Reg8::AL && dstReg != Reg8::AH)
    {
        if(signExtend)
            builder.movsx(static_cast<Reg32>(dstReg), Reg8::AL);
        else if(zeroExtend)
            builder.movzx(static_cast<Reg32>(dstReg), Reg8::AL);
        else
            builder.mov(dstReg, Reg8::AL);
        builder.pop(Reg64::RAX);
    }
    else if(dstReg == Reg8::AH) // TODO: having a worse case for A is not great
    {
        assert(!zeroExtend && !signExtend);
        builder.mov(Reg8::AH, Reg8::AL);
        builder.pop(Reg64::R10);
        builder.mov(Reg8::AL, Reg8::R10B); // restore low byte
    }
    else if(dstReg == Reg8::AL)// ... though this is the worst case... (AL == F, so unlikely)
    {
        builder.pop(Reg64::R10);
        
        if(signExtend)
            builder.movsx(Reg32::EAX, Reg8::AL);
        else if(!zeroExtend)
        {
            // EAX = EAX + (R10D & 0xFF00)
            builder.and_(Reg32::R10D, 0xFF00u);
            builder.movzx(Reg32::EAX, Reg8::AL);
            builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
        }
    }
}

void X86Target::callRestoreIfNeeded(X86Builder &builder, int &saveState) const
{
    callRestore(builder, saveState, 0);
}

// restore if it would affect this register
// takes a variant so it can be passed the result of checkRegOrImm
void X86Target::callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg8, uint8_t> val, int &saveState) const
{
    if(!std::holds_alternative<Reg8>(val))
        return; // not a register

    auto index = callSaveIndex(std::get<Reg8>(val));

    if(index != -1)
        callRestore(builder, saveState, index);
}

void X86Target::callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg16, uint16_t> val, int &saveState) const
{
    if(!std::holds_alternative<Reg16>(val))
        return;

    auto index = callSaveIndex(std::get<Reg16>(val));

    if(index != -1)
        callRestore(builder, saveState, index);
}
