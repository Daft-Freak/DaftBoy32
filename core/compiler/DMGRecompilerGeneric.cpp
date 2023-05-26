#include <cstring>

#include "DMGRecompilerGeneric.h"

DMGRecompilerGeneric::DMGRecompilerGeneric(DMGCPU &cpu) : DMGRecompiler(cpu), fallback(cpu)
{
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
    memcpy(codeBuf, fallback.codeBuf, len);
    curCodePtr += len;

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf);
}