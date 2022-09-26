#include <cassert>
#include <cstdio>
#include <variant>

#include "AGBRecompilerThumb.h"

#include "AGBCPU.h"
#include "ThumbBuilder.h"

bool AGBRecompilerThumb::compileTHUMB(uint8_t *&codePtr, uint32_t pc, BlockInfo &blockInfo)
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codePtr);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto startPC = pc;

    // do instructions
    int numInstructions = 0;

    for(auto &instr : blockInfo.instructions)
    {
        if(!recompileInstruction(pc, instr, builder))
            break;

        if(builder.getError())
            break;

        numInstructions++;
    }

    if(builder.getError())
    {
        ldrLiteralInstrs.clear();
        printf("recompile @%08X failed due to error (out of space?)\n", startPC);
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
    __builtin___clear_cache(codeBuf, codeBuf + codeBufSize);

    return true;
}

bool AGBRecompilerThumb::recompileInstruction(uint32_t &pc, OpInfo &instr, ThumbBuilder &builder)
{
    return false;
}

void AGBRecompilerThumb::compileEntry()
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

    builder.push(0, true); // LR
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    builder.ldr(Reg::R2, 20);
    builder.mov(Reg::R8, Reg::R2);

    // TODO: load emu regs

    builder.bx(Reg::R1);

    // exit setting the call flag ... and saving LR
    exitForCallPtr = reinterpret_cast<uint8_t *>(builder.getPtr());
    builder.mov(Reg::R0, 1);
    builder.ldr(Reg::R2, 16);
    builder.strb(Reg::R0, Reg::R2, 0);

    // exit saving LR
    saveAndExitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    builder.mov(Reg::R0, Reg::LR);
    builder.ldr(Reg::R2, 12);
    builder.str(Reg::R0, Reg::R2, 0);

    // exit
    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // TODO:
    // store PC
    // store SP
    // save emu regs

    // restore regs and return
    builder.pop(0, true); // PC

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf + 1/*thumb*/);

    // write cpu addr
    auto ptr = builder.getPtr();
    *ptr++ = 0; // align
    *ptr++ = cpuPtr;
    *ptr++ = cpuPtr >> 16;

    // write addr of exitCallFlag
    auto addr = reinterpret_cast<uintptr_t>(&exitCallFlag);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    // write addr of tmpSavedPtr
    addr = reinterpret_cast<uintptr_t>(&tmpSavedPtr);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    curCodePtr = reinterpret_cast<uint8_t *>(ptr);

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codePtr16;

    //debug
    printf("generated %i halfwords for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
#endif
}
