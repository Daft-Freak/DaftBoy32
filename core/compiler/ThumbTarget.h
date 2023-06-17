#pragma once

#include <map>
#include <optional>

#include "RecompilerGeneric.h"

enum class Reg;

class ThumbTarget final
{
public:
    ThumbTarget(){}

    void init(SourceInfo sourceInfo, void *cpuPtr);

    const SourceInfo &getSourceInfo() {return sourceInfo;}

    bool compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint16_t pc, GenBlockInfo &blockInfo);

    uint8_t *compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize);

private:
    std::optional<Reg> mapReg(uint8_t index);

    SourceFlagInfo &getFlagInfo(SourceFlagType flag);
    uint8_t flagWriteMask(SourceFlagType flag);
    bool writesFlag(uint16_t opFlags, SourceFlagType flag);

    SourceInfo sourceInfo;
    void *cpuPtr;

    std::map<uint8_t, Reg> regAlloc;

    uint8_t flagsReg = 0;
    uint8_t flagMap[4]; // map from SourceFlagType to flags bit

    uint16_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;
};