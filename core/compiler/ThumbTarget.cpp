#include <cassert>
#include <cstdio>

#include "ThumbTarget.h"
#include "ThumbBuilder.h"

const Reg pcReg = Reg::R1;
const Reg cpuPtrReg = Reg::R8;

static bool isLowReg(Reg r)
{
    return static_cast<int>(r) < 8;
}

// TODO: improve branch handling
static int load16BitValueSize(uint16_t value)
{
    if(value <= 0xFF)
        return 2; // mov
    
    if(value >> __builtin_ctz(value) <= 0xFF)
        return 4; // mov + lsl

    return 8; // mov + mov + lsl + orr
}

static void load16BitValue(ThumbBuilder &builder, Reg dst, uint16_t value, Reg tmp = Reg::R2)
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
        builder.mov(dst, value & 0xFF);
        builder.mov(tmp, value >> 8);
        builder.lsl(tmp, tmp, 8);
        builder.orr(dst, tmp);
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
    };

    // alloc registers
    unsigned int allocOff = 0;

    regAlloc.emplace(0, Reg::R1); // temp

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

    // do instructions
    int numInstructions = 0;
    uint16_t *opStartPtr = nullptr;
    bool newEmuOp = true;
    bool forceExitAfter = false;

    auto beginInstr = blockInfo.instructions.begin();
    auto endInstr = blockInfo.instructions.end();

    for(auto instIt = beginInstr; instIt != endInstr; ++instIt)
    {
        auto &instr = *instIt;

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

        // ei check

        bool err = false;

        // preserve flags

        switch(instr.opcode)
        {
            case GenOpcode::NOP:
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
            load16BitValue(builder, pcReg, pc - instr.len);
            builder.bl((exitPtr - builder.getPtr()) * 2);
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
                builder.sub(Reg::R0, cyclesThisInstr);

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
    }

    if(builder.getError())
    {
        //ldrLiteralInstrs.clear();
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
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    builder.ldr(Reg::R2, 60); // FIXME: hardcoded offset
    builder.mov(cpuPtrReg, Reg::R2);

    // load emu regs
    uint16_t firstRegOff = 0xFFFF;

    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

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
                printf("%i %i %i\n", i, firstRegOff, reg.cpuOffset);
                fflush(stdout);
                if(isLowReg(*mappedReg))
                    builder.ldrh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
                else
                {
                    builder.ldrh(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
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
    builder.ldr(Reg::R2, 44); // FIXME: hardcoded offset
    builder.strb(Reg::R0, Reg::R2, 0);

    // exit saving LR
    saveAndExitPtr = builder.getPtr();

    builder.mov(Reg::R0, Reg::LR);
    builder.ldr(Reg::R2, 40); // FIXME: hardcoded offset
    builder.str(Reg::R0, Reg::R2, 0);

    // exit
    exitPtr = builder.getPtr();

    // store PC
    int pcOff = sourceInfo.pcOffset;
    assert(pcOff <= 0xFF);
    builder.mov(Reg::R2, Reg::R8); // cpu ptr
    builder.mov(Reg::R0, pcOff);
    builder.strh(Reg::R1, Reg::R2, Reg::R0);

    // save emu regs
    if(firstRegOff)
        builder.add(Reg::R2, firstRegOff); // add to cpu ptr

    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

             if(auto mappedReg = mapReg(i))
             {
                assert(reg.cpuOffset - firstRegOff <= 62);
                if(isLowReg(*mappedReg))
                    builder.strh(*mappedReg, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
                else
                {
                    builder.mov(Reg::R3, *mappedReg);
                    builder.strh(Reg::R3, Reg::R2, reg.cpuOffset - firstRegOff); // FIXME: assumes 16 bit regs
                }
             }
        }

        i++;
    }


    // restore regs and return
    builder.pop(0b11111100, false); // R2-7
    builder.mov(Reg::R8, Reg::R2);
    builder.mov(Reg::R9, Reg::R3);

    builder.pop(0, true); // PC

    // write cpu addr
    auto ptr = builder.getPtr();

    if((ptr - codePtr16) & 1)
        *ptr++ = 0; // align

    *ptr++ = cpuPtrInt;
    *ptr++ = cpuPtrInt >> 16;

    // write addr of exitCallFlag
    auto addr = reinterpret_cast<uintptr_t>(&sourceInfo.exitCallFlag);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    // write addr of tmpSavedPtr
    addr = reinterpret_cast<uintptr_t>(&sourceInfo.savedExitPtr);
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

    auto ret = codeBuf + 1; // thumb bit

    codeBuf = reinterpret_cast<uint8_t *>(ptr);

    return ret;
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
