#include <cassert>
#include <cstdio>

#include "X86Target.h"
#include "X86Builder.h"

// reg helpers
static const Reg64 cpuPtrReg = Reg64::R14;

// only an output at the end, could be anything
static const Reg32 pcReg32 = Reg32::R12D;
static const Reg16 pcReg16 = Reg16::R12W;

static const Reg32 spReg32 = Reg32::R13D;
static const Reg16 spReg16 = Reg16::R13W;
static const Reg8 spReg8 = Reg8::R13B; // mostly for the adds with the strange flags

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
        if(it->alias || it->type != SourceRegType::General)
            continue;

        regAlloc.emplace(i, regList[allocOff]);

        allocOff++;

        if(allocOff == std::size(regList))
            break;
    }

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
                        // if there's a reg that requires REX and an xH reg, swap the halves over and read/write xL instead
                        std::optional<Reg8> swapReg;
                        if(isXHReg(*dst) && requiresREX(*src))
                            swapReg = dst = swapRegHalf(*dst);
                        else if(isXHReg(*src) && requiresREX(*dst))
                            swapReg = src = swapRegHalf(*src);

                        if(swapReg)
                            builder.xchg(*swapReg, swapRegHalf(*swapReg));
                        
                        builder.mov(*dst, *src);

                        if(swapReg)
                            builder.xchg(*swapReg, swapRegHalf(*swapReg));
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            // ...

            case GenOpcode::Jump:
            {
                auto condition = static_cast<GenCondition>(instr.src[0]);
                auto regSize = sourceInfo.registers[instr.src[1]].size;
                if((instr.flags & GenOp_Exit) && condition == GenCondition::Always)
                {
                    if(regSize == 16)
                    {
                        if(auto src = checkReg16(instr.src[1]))
                        {
                            // need to sync cycles *before* the jump out
                            if(instrCycles)
                            {
                                cycleExecuted();
                                instrCycles--;
                                assert(instrCycles == 0);
                            }
                            syncCyclesExecuted();

                            builder.movzx(pcReg32, *src);
                            builder.jmp(exitPtr - builder.getPtr()); // exit
                        }
                    }
                    else
                        badRegSize(regSize);
                }
                else
                {
                    printf("unhandled gen op %i\n", instr.opcode);
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