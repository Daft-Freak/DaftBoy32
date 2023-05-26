#include <cstring>

#include "DMGRecompilerGeneric.h"

DMGRecompilerGeneric::DMGRecompilerGeneric(DMGCPU &cpu) : DMGRecompiler(cpu), fallback(cpu)
{
    fallback.codeBuf = codeBuf;
    fallback.curCodePtr = codeBuf;
}

bool DMGRecompilerGeneric::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) 
{
    return fallback.compile(codePtr, pc, blockInfo);
}

void DMGRecompilerGeneric::compileEntry()
{
    // hax
    fallback.compileEntry();

    auto len = fallback.curCodePtr - fallback.codeBuf;
    curCodePtr += len;

    // patch tmpSavedPtr/exitForCall
    auto ptr = fallback.saveAndExitPtr + 4;
    uint64_t addr = reinterpret_cast<uintptr_t>(&tmpSavedPtr);

    *ptr++ = addr;
    *ptr++ = addr >> 8;
    *ptr++ = addr >> 16;
    *ptr++ = addr >> 24;
    *ptr++ = addr >> 32;
    *ptr++ = addr >> 40;
    *ptr++ = addr >> 48;
    *ptr++ = addr >> 56;

    ptr = fallback.exitForCallPtr + 2;
    addr = reinterpret_cast<uintptr_t>(&exitCallFlag);

    *ptr++ = addr;
    *ptr++ = addr >> 8;
    *ptr++ = addr >> 16;
    *ptr++ = addr >> 24;
    *ptr++ = addr >> 32;
    *ptr++ = addr >> 40;
    *ptr++ = addr >> 48;
    *ptr++ = addr >> 56;

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf);
}