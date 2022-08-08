#pragma once

#include "DMGRecompiler.h"

class ThumbBuilder;

class DMGRecompilerThumb final : public DMGRecompiler
{
public:
    using DMGRecompiler::DMGRecompiler;

private:
    bool compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) override;

    bool recompileInstruction(uint16_t &pc, OpInfo &instr, ThumbBuilder &builder);
    void recompileExInstruction(OpInfo &instr, ThumbBuilder &builder);

    void compileEntry() override;
};