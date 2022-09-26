#pragma once

#include "AGBRecompiler.h"

class ThumbBuilder;

class AGBRecompilerThumb final : public AGBRecompiler
{
public:
    using AGBRecompiler::AGBRecompiler;

private:
    bool compileTHUMB(uint8_t *&codePtr, uint32_t pc, BlockInfo &blockInfo) override;

    bool recompileInstruction(uint32_t &pc, OpInfo &instr, ThumbBuilder &builder);

    void compileEntry() override;

    // literals
    // (we only need two so far)
    uint32_t literals[2]{};

    unsigned int curLiteral = 0;
    std::vector<uint16_t *> ldrLiteralInstrs;
};