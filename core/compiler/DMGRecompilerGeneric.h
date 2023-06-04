#pragma once

#include "DMGRecompiler.h"

#include "DMGRecompilerX86.h"
#include "RecompilerGeneric.h"
#include "X86Target.h"

class DMGRecompilerGeneric final : public DMGRecompiler
{
public:
    DMGRecompilerGeneric(DMGCPU &cpu);

private:
    bool compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) override;

    void compileEntry() override;

    bool convertToGeneric(uint16_t pc, BlockInfo &block, GenBlockInfo &genBlock);
    void printBlock(uint16_t pc, GenBlockInfo &block);

    X86Target target;

    // wastes memory, but this is a temp fallback
    DMGRecompilerX86 fallback;
};