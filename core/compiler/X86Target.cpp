#include <cassert>
#include <cstdio>
#include <variant>

#include "X86Target.h"
#include "X86Builder.h"

// reg helpers
static const Reg64 cpuPtrReg = Reg64::R14;

// only an output at the end, could be anything
static const Reg32 pcReg32 = Reg32::R12D;
static const Reg16 pcReg16 = Reg16::R12W;

#ifdef _WIN32
static const Reg64 argumentRegs64[]{Reg64::RCX, Reg64::RDX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::ECX, Reg32::EDX, Reg32::R8D, Reg32::R9D};
#else
static const Reg64 argumentRegs64[]{Reg64::RDI, Reg64::RSI, Reg64::RDX, Reg64::RCX}; // + R8/9 but we don't use > 4 args
static const Reg32 argumentRegs32[]{Reg32::EDI, Reg32::ESI, Reg32::EDX, Reg32::ECX};
#endif

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

// call helpers
// FIXME: DMG specific bits
static bool isCallSaved(Reg16 reg)
{
    return reg == Reg16::AX || reg == Reg16::CX || reg == Reg16::DX || reg == Reg16::DI;
}

static void callSave(X86Builder &builder)
{
    builder.push(Reg64::RAX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RDI);
    // builder.sub(Reg64::RSP, 8); // align stack

#ifdef _WIN32
    builder.sub(Reg64::RSP, 32); // shadow space
#endif
}

static void callSaveOrSkip(X86Builder &builder)
{
    auto tmpPtr = builder.getPtr() - 4;

    // skip pop/push if we just popped
    // only safe if values from RAX, RCX, RDX, RDI are not moved to args
    if(tmpPtr[0] == 0x5F/*pop rdi*/ && tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
        builder.resetPtr(tmpPtr);
    else if(tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
    {
        // skip 3/4 of the pops
        // (should be after a writeMem)
        builder.resetPtr(tmpPtr + 1);
        builder.push(Reg64::RDI);
    }
    else
    {
        callSave(builder);
        return;
    }

#ifdef _WIN32
    builder.sub(Reg64::RSP, 32); // shadow space
#endif
}

static void callRestore(X86Builder &builder)
{
#ifdef _WIN32
    builder.add(Reg64::RSP, 32); // shadow space
#endif

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg32 dstReg)
{
    // mov ret val (this is never used for the emulated regs)
    assert(dstReg != Reg32::EAX && dstReg != Reg32::EDX && dstReg != Reg32::ECX);

    builder.mov(dstReg, Reg32::EAX);

#ifdef _WIN32
    builder.add(Reg64::RSP, 32); // shadow space
#endif

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(dstReg == Reg32::EDI ? Reg64::RSI : Reg64::RDI); // use RSI to pop the unneeded value
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg8 dstReg)
{
    assert(dstReg != Reg8::DIL); // no

    // move before popping if possible
    bool isPoppedReg = dstReg == Reg8::AL || dstReg == Reg8::CL || dstReg == Reg8::DL || dstReg == Reg8::AH || dstReg == Reg8::CH || dstReg == Reg8::DH;

    if(!isPoppedReg)
        builder.mov(dstReg, Reg8::AL);

#ifdef _WIN32
    builder.add(Reg64::RSP, 32); // shadow space
#endif

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);

    // mov ret val (if not going to RAX)
    if(isPoppedReg && dstReg != Reg8::AL && dstReg != Reg8::AH)
    {
        builder.mov(dstReg, Reg8::AL);
        builder.pop(Reg64::RAX);
    }
    else if(dstReg == Reg8::AH) // TODO: having a worse case for A is not great
    {
        builder.mov(Reg8::AH, Reg8::AL);
        builder.pop(Reg64::R10);
        builder.mov(Reg8::AL, Reg8::R10B); // restore low byte
    }
    else if(dstReg == Reg8::AL)// ... though this is the worst case... (AL == F, so unlikely)
    {
        // EAX = EAX + (R10D & 0xFF00)
        builder.pop(Reg64::R10);
        builder.and_(Reg32::R10D, 0xFF00);
        builder.movzx(Reg32::EAX, Reg8::AL);
        builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
    }
    else // we already did it
        builder.pop(Reg64::RAX);
}

void X86Target::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg32 regList[]
    {
        // from DMGRecompilerX86
        Reg32::EAX,
        Reg32::ECX,
        Reg32::EDX,
        Reg32::EBX,
        Reg32::R13D,
    };
    // also free R11 R15

    // alloc registers
    unsigned int allocOff = 0;

    regAlloc.emplace(0, Reg32::R10D); // temp

    int i = 1;
    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it, i++)
    {
        if(it->type == SourceRegType::Flags)
            flagsReg = i;

        if(it->alias || it->type != SourceRegType::General)
            continue;

        if(allocOff == std::size(regList))
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

bool X86Target::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint16_t pc, GenBlockInfo &blockInfo)
{
    X86Builder builder(codePtr, codeBufEnd);

    auto startPC = pc;

    // state
    uint8_t *lastInstrCycleCheck = nullptr;

    // FIXME: DMG specific
    bool inHRAM = pc >= 0xFF00;

    // cycle executed sync
    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;

    auto cycleExecuted = [this, &builder, inHRAM, &cyclesThisInstr, &delayedCyclesExecuted]()
    {
        cyclesThisInstr += 4;

        // FIXME: DMG specific
        if(!inHRAM)
        {
            // since we refuse to compile when OAM DMA is active we can just inline cycleExecuted (-updateOAMDMA) when not running from HRAM
            // ...and we can also do it slightly later
            delayedCyclesExecuted += 4;
            return;
        }

        // FIXME: handle not having a cycleExecuted (AGBRecompiler)

        // safe to skip here as we take no args
        callSaveOrSkip(builder);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(sourceInfo.cycleExecuted)); // function ptr
        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(delayedCyclesExecuted < 127);
        auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

        int8_t i8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        builder.addD(i8Cycles, cpuPtrReg, reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - cpuPtrInt);

        delayedCyclesExecuted = 0;
    };

    // load/store helpers
    auto setupMemAddr = [this, &builder, &syncCyclesExecuted](std::variant<Reg16, uint16_t> addr)
    {
        //FIXME: DMG specific "should sync" logic
        if(std::holds_alternative<Reg16>(addr))
        {
            // skip sync for stack read/write
            //if(!isStack || spWrite) // TODO?
                syncCyclesExecuted();

            builder.movzx(argumentRegs32[1], std::get<Reg16>(addr));
        }
        else
        {
            // accessing most ram shouldn't cause anything to be updated, so we don't need an accurate cycle count
            auto immAddr = std::get<uint16_t>(addr);
            if(immAddr >= 0xFF00 && immAddr < 0xFF80/*HRAM start*/ && immAddr != 0xFFFF/*IE*/)
                syncCyclesExecuted();

            builder.mov(argumentRegs32[1], immAddr);
        }
    };

    auto readMem = [this, &builder, &setupMemAddr](std::variant<Reg16, uint16_t> addr, Reg8 dstReg)
    {
        // can skip push/pop if we don't need AX/CX/DX
        if(!std::holds_alternative<Reg16>(addr) || !isCallSaved(std::get<Reg16>(addr)))
            callSaveOrSkip(builder);
        else
            callSave(builder);

        setupMemAddr(addr);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(sourceInfo.readMem)); // function ptr
        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr

        builder.call(Reg64::RAX); // do call

        callRestore(builder, dstReg);
    };

    auto writeMem = [this, &builder, &setupMemAddr](std::variant<Reg16, uint16_t> addr, std::variant<Reg8, uint8_t> data)
    {
        // can skip push/pop if we don't need AX/CX/DX
        bool noPopAddr = !std::holds_alternative<Reg16>(addr) || !isCallSaved(std::get<Reg16>(addr));
        bool noPopData = !std::holds_alternative<Reg8>(data) || std::get<Reg8>(data) == Reg8::BL || std::get<Reg8>(data) == Reg8::BH;

        if(noPopAddr && noPopData)
            callSaveOrSkip(builder);
        else
            callSave(builder);

#ifndef _WIN32
        // setup addr first (data writes DX)
        setupMemAddr(addr);
#endif

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

#ifdef _WIN32
        // setup addr after data (addr writes DX)
        setupMemAddr(addr);
#endif

        builder.mov(argumentRegs32[3], Reg32::EDI); // cycle count

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(sourceInfo.writeMem)); // function ptr
        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder, Reg32::EDI);
    };

    // neededFlags is what we need to output
    // updateFlags is what we can output from the result
    // oneFlags is is what is alwyas set to 1
    // preservedFlags is what is... preserved
    const auto setFlags = [this, &builder](std::optional<Reg8> result, uint8_t &neededFlags, uint8_t updateFlags, uint8_t oneFlags = 0, uint8_t preservedFlags = 0)
    {
        if(!flagsReg)
            return;

        // FIXME: assuming 8bit flags reg
        auto f = *mapReg8(flagsReg);

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

        bool setF = preservedFlags != 0;
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

            if(haveOpFlags)
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
            else if(result)
            {
                // reconstruct from result
                builder.cmp(*result, 0);
                builder.jcc(Condition::NE, 3); // if != 0
                builder.or_(f, 1 << zFlag.bit); // set Z

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

    // do instructions
    int numInstructions = 0;
    uint8_t *opStartPtr = nullptr;
    bool newEmuOp = true;

    for(auto &instr : blockInfo.instructions)
    {
        bool forceExitAfter = false;

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

        // DMG EI check

        bool err = false;

        // validating wrappers
        auto checkReg32 = [this, &err, &instr](uint8_t index)
        {
            auto reg = mapReg32(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }

            return reg;
        };

        auto checkReg16 = [this, &err, &instr](uint8_t index)
        {
            auto reg = mapReg16(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }

            return reg;
        };

        auto checkReg8 = [this, &err, &instr](uint8_t index)
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

        auto badRegSize = [&err, &instr](int size)
        {
            printf("unhandled reg size %i in op %i\n", size, int(instr.opcode));
            err = true;
        };
    
        // helpers to deal with restrictions
        auto maybeSwapHReg = [&builder](std::optional<Reg8> &src, std::optional<Reg8> &dst)
        {
            // if there's a reg that requires REX and an xH reg, swap the halves over and read/write xL instead
            std::optional<Reg8> swapReg;
            if(isXHReg(*dst) && requiresREX(*src))
                swapReg = dst = swapRegHalf(*dst);
            else if(isXHReg(*src) && requiresREX(*dst))
                swapReg = src = swapRegHalf(*src);

            if(swapReg)
                builder.xchg(*swapReg, swapRegHalf(*swapReg));

            return swapReg;
        };

        auto unswapReg = [&builder](std::optional<Reg8> swapReg)
        {
            if(swapReg)
                builder.xchg(*swapReg, swapRegHalf(*swapReg));
        };

        auto checkSingleSource = [&err, &instr]
        {
            if(instr.src[0] != instr.dst[0])
            {
                // this just needs an extra mov to fix, but nothing generates these yet
                printf("unhandled src[0] != dst in op %i\n", int(instr.opcode));
                err = true;
            }
        };
    
        switch(instr.opcode)
        {
            case GenOpcode::NOP:
                break;

            case GenOpcode::LoadImm:
                // TODO: optimise (merge into user, smaller load?)
                builder.mov(*mapReg32(0), instr.imm);
                break;

            case GenOpcode::Move:
            {
                assert(!(instr.flags & GenOp_WriteFlags));

                auto regSize = sourceInfo.registers[instr.dst[0]].size;

                if(regSize == 16)
                {
                    auto src = checkReg32(instr.src[0]);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src && dst)
                        builder.mov(*dst, *src);
                }
                else if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        auto swapReg = maybeSwapHReg(src, dst);
                        builder.mov(*dst, *src);
                        unswapReg(swapReg);
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::Load:
            {
                // TODO: store data width somewhere
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16)
                {
                    auto addr = checkReg16(instr.src[0]);
                    auto dst = checkReg8(instr.dst[0]);
                    if(addr && dst)
                        readMem(*addr, *dst);
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Store:
            {
                // TODO: store data width somewhere
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 16)
                {
                    auto addr = checkReg16(instr.src[0]);
                    auto data = checkReg8(instr.src[1]);
                    if(addr && data)
                        writeMem(*addr, *data);
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::And:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        auto swapReg = maybeSwapHReg(src, dst);
                        builder.and_(*dst, *src);
                        unswapReg(swapReg);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero), flagWriteMask(SourceFlagType::HalfCarry));
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Compare:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.src[0]);
                    auto f = checkReg8(flagsReg);

                    if(src && dst && f)
                    {
                        auto cMask = flagWriteMask(SourceFlagType::Carry);
                        auto zMask = flagWriteMask(SourceFlagType::Zero);
                        auto hMask = flagWriteMask(SourceFlagType::HalfCarry);

                        if(instr.flags & hMask)
                        {
                            // half cmp
                            // this makes a lot of assumptions, but is only used for DMG anyway
                            if(isXHReg(*src))
                            {
                                builder.mov(*f, *src);
                                builder.mov(Reg8::SIL, *f);
                            }
                            else
                                builder.mov(Reg8::SIL, *src);

                            builder.mov(*f, *dst);

                            builder.and_(*f, 0xF);
                            builder.and_(Reg8::SIL, 0xF);

                            builder.cmp(*f, Reg8::SIL);
                            builder.setcc(Condition::B, Reg8::SIL);
                        }

                        auto swapReg = maybeSwapHReg(src, dst);
                        builder.cmp(*dst, *src);
                        unswapReg(swapReg);

                        if((instr.flags & (cMask | zMask)) == (cMask | zMask))
                            builder.setcc(Condition::E, Reg8::R10B); // save zero if carry also set, should be safe to use tmp here

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags({}, flags, cMask | zMask, flagWriteMask(SourceFlagType::WasSub));

                        // half carry
                        if(flags & flagWriteMask(SourceFlagType::HalfCarry))
                        {
                            builder.shl(Reg8::SIL, getFlagInfo(SourceFlagType::HalfCarry).bit);
                            builder.or_(*f, Reg8::SIL);
                        }

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

            case GenOpcode::Or:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        auto swapReg = maybeSwapHReg(src, dst);
                        builder.or_(*dst, *src);
                        unswapReg(swapReg);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero));
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
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        auto swapReg = maybeSwapHReg(src, dst);
                        builder.xor_(*dst, *src);
                        unswapReg(swapReg);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Zero));
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
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        assert(*src != *dst);

                        // can only shift by CL
                        bool swap = *src != Reg8::CL;

                        // swap with whatever is currently in CL
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // if it was the dst swap the args around
                        if(dst == Reg8::CL)
                            builder.shlCL(*src);
                        else
                            builder.shlCL(*dst);

                        // restore
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero));
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
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        assert(*src != *dst);

                        // can only shift by CL
                        bool swap = *src != Reg8::CL;

                        // swap with whatever is currently in CL
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // if it was the dst swap the args around
                        if(dst == Reg8::CL)
                            builder.sarCL(*src);
                        else
                            builder.sarCL(*dst);

                        // restore
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero));
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
                assert(!(instr.flags & GenOp_ReadFlags));

                if(regSize == 8)
                {
                    auto src = checkReg8(instr.src[1]);
                    auto dst = checkReg8(instr.dst[0]);

                    if(src && dst)
                    {
                        assert(*src != *dst);

                        // can only shift by CL
                        bool swap = *src != Reg8::CL;

                        // swap with whatever is currently in CL
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // if it was the dst swap the args around
                        if(dst == Reg8::CL)
                            builder.shrCL(*src);
                        else
                            builder.shrCL(*dst);

                        // restore
                        if(swap)
                            builder.xchg(*src, Reg8::CL);

                        // flags
                        uint8_t flags = instr.flags & GenOp_WriteFlags;
                        setFlags(*dst, flags, flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Zero));
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
                if((instr.flags & GenOp_Exit))
                {
                    if(regSize == 16)
                    {
                        if(auto src = checkReg16(instr.src[1]))
                        {
                            uint8_t *branchPtr = nullptr;
                            bool flagSet = false;

                            // condition
                            if(condition != GenCondition::Always)
                            {
                                auto f = checkReg8(flagsReg);

                                syncCyclesExecuted();
                                // TODO: only if unhandled
                                builder.mov(pcReg32, pc);

                                flagSet = condition == GenCondition::Equal || condition == GenCondition::CarrySet;
                                auto flag = condition == GenCondition::Equal || condition == GenCondition::NotEqual ? SourceFlagType::Zero : SourceFlagType::Carry;

                                if(f)
                                {
                                    builder.test(*f, 1 << getFlagInfo(flag).bit);
                                    branchPtr = builder.getPtr();
                                    builder.jcc(flagSet ? Condition::E : Condition::NE, 1); // patch later
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

                            // if branch sub cycles else
                            builder.movzx(pcReg32, *src);
    
                            // TODO: handle branch
                            builder.jmp(exitPtr - builder.getPtr()); // exit

                            // patch the condition jump
                            if(branchPtr && !builder.getError())
                            {
                                builder.patch(branchPtr, branchPtr + 2);
                                builder.jcc(flagSet ? Condition::E : Condition::NE, builder.getPtr() - branchPtr);
                                builder.endPatch();
                            }
                        }
                    }
                    else
                        badRegSize(regSize);
                }
                else
                {
                    printf("unhandled branch\n");

                    err = true;
                }
                break;
            }

            default:
                printf("unhandled gen op %i\n", instr.opcode);
                err = true;
        }

        if(builder.getError())
            break;

        // failed but builder still ok
        if(err)
        {
            builder.resetPtr(opStartPtr);
            builder.mov(pcReg32, pc - instr.len);
            builder.jmp(exitPtr - builder.getPtr());
            break;
        }

        // additional cycle
        if(instrCycles)
        {
            cycleExecuted();
            assert(instrCycles == 1);
        }

        // TODO: avoid syncing in the middle of an emulated op
        syncCyclesExecuted();

        newEmuOp = instr.len != 0;

        // check cycle count if this is the last part of en emulated op
        // ... but not on the last op, that should always exit anyway
        if(!(instr.flags & GenOp_Last) && newEmuOp) // TODO: also safe to omit if there's an unconditional exit
        {
            // exit may be forced after interrupts are enabled
            if(forceExitAfter)
                builder.jmp(cyclesThisInstr ? 5 : 2);

            // cycles -= executed
            if(cyclesThisInstr) // 0 means we already did the sub
                builder.sub(Reg32::EDI, cyclesThisInstr);

            lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

            // if <= 0 exit
            builder.jcc(Condition::G, 11);
            builder.mov(pcReg32, pc);
            builder.call(saveAndExitPtr - builder.getPtr());
        }

        cyclesThisInstr = 0;

        if(newEmuOp)
            numInstructions++;
    }

    /*spWrite = false;
    branchTargets.clear();
    forwardBranchesToPatch.clear();*/

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

void X86Target::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
{
    X86Builder builder(codeBuf, codeBuf + codeBufSize);

    // prologue
    builder.push(Reg64::RBP);
    builder.mov(Reg64::RBP, Reg64::RSP);

    // save
    builder.push(Reg64::R12);
    builder.push(Reg64::R13);
    builder.push(Reg64::R14);
    builder.push(Reg64::RBX);

#ifdef _WIN32
    builder.push(Reg64::RSI);
    builder.push(Reg64::RDI);

    builder.mov(Reg64::RDI, argumentRegs64[0]); // move cycle count
    builder.mov(Reg64::RSI, argumentRegs64[1]); // code ptr
#endif

    // store pointer to CPU
    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);
    builder.mov(cpuPtrReg, cpuPtrInt);


    // load emu regs
    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

             if(auto reg32 = mapReg32(i))
                builder.movzxW(*reg32, cpuPtrReg, reg.cpuOffset); // FIXME: assumes 16 bit regs
        }

        i++;
    }

    // jump to code
    builder.jmp(Reg64::RSI);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.exitCallFlag));
    builder.mov(1, Reg64::R11);

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.savedExitPtr));
    builder.mov(Reg64::R10, Reg64::R11, true);

    // just exit
    exitPtr = builder.getPtr();

    // save emu regs
    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

            if(auto reg16 = mapReg16(i))
                builder.mov(*reg16, cpuPtrReg, true, reg.cpuOffset); // FIXME: assumes 16 bit regs
        }

        i++;
    }

    // save emu pc
    builder.mov(pcReg16, cpuPtrReg, true, sourceInfo.pcOffset);

    // restore

#ifdef _WIN32
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RSI);
#endif

    builder.pop(Reg64::RBX);
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

    codeBuf = builder.getPtr();
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
            {
                if(!reg.alias || reg.aliasMask == 0xFF)
                    return static_cast<Reg8>(static_cast<int>(*reg32) + 0x10);
            }
    
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