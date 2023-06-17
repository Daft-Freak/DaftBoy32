#include <cassert>
#include <cstdio>

#include "ThumbTarget.h"
#include "ThumbBuilder.h"

void ThumbTarget::init(SourceInfo sourceInfo, void *cpuPtr)
{
    // reg alloc

    // map flags
    for(auto & f : flagMap)
        f = 0xFF;

    int i = 0;
    for(auto it = sourceInfo.flags.begin(); it != sourceInfo.flags.end(); ++it, i++)
        flagMap[static_cast<int>(it->type)] = i;

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool ThumbTarget::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint16_t pc, GenBlockInfo &blockInfo)
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codePtr);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBufEnd));

    auto startPC = pc;

    // do instructions
    int numInstructions = 0;
    uint16_t *opStartPtr = nullptr;
    bool newEmuOp = true;

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
            //cycleExecuted();
            instrCycles--;
        }

        // ei check

        bool err = false;

        // preserve flags

        switch(instr.opcode)
        {
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
            //load16BitValue(builder, Reg::R1, pc - instr.len);
            builder.bl(exitPtr - builder.getPtr());
            break;
        }

        // additional cycle
        if(instrCycles)
        {
            //cycleExecuted();
            assert(instrCycles == 1);
        }

        // check if this is the end of the source instruction (pc incremented)
        newEmuOp = instr.len != 0;

        // exit check

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

    codePtr = reinterpret_cast<uint8_t *>(endPtr);

    // need to clear the cache
    // TODO: don't do the entire range
    //__builtin___clear_cache(codeBuf, codeBuf + codeBufSize);

    return true;
}

void ThumbTarget::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
{

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
