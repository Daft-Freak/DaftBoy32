#pragma once

#include "DMGRecompiler.h"

#include "RecompilerGeneric.h"

#if defined(RECOMPILER_X86)
#include "X86Target.h"
#elif defined(RECOMPILER_THUMB)
#include "ThumbTarget.h"
#else
#error No recompiler target!
#endif


class DMGRecompilerGeneric final : public DMGRecompiler
{
public:
    DMGRecompilerGeneric(DMGCPU &cpu);

private:
    bool compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) override;

    void compileEntry() override;

    bool convertToGeneric(uint16_t pc, BlockInfo &block, GenBlockInfo &genBlock);
    void printBlock(uint16_t pc, GenBlockInfo &block);

#if defined(RECOMPILER_X86)
    X86Target target;
#elif defined(RECOMPILER_THUMB)
    ThumbTarget target;
#endif
};