#pragma once

#include "DMGRecompiler.h"

#include "DMGRecompilerX86.h"

class DMGRecompilerGeneric final : public DMGRecompiler
{
public:
    DMGRecompilerGeneric(DMGCPU &cpu);

private:
    bool compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) override;

    void compileEntry() override;

    // wastes memory, but this is a temp fallback
    DMGRecompilerX86 fallback;
};