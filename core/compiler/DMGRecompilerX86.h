#pragma once

#include "DMGRecompiler.h"

class X86Builder;

class DMGRecompilerX86 final : public DMGRecompiler
{
public:
    using DMGRecompiler::DMGRecompiler;

private:
    bool compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) override;

    bool recompileInstruction(uint16_t &pc, OpInfo &instr, X86Builder &builder);
    void recompileExInstruction(OpInfo &instr, X86Builder &builder);

    void compileEntry() override;
};