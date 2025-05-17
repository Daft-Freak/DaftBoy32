#pragma once

#include <map>
#include <optional>

#include "RecompilerGeneric.h"

enum class Reg;
class ThumbBuilder;

class ThumbTarget final
{
public:
    struct RegInfo
    {
        bool operator==(RegInfo &r)
        {
            return reg == r.reg && mask == r.mask;
        }

        Reg reg;
        uint32_t mask;
    };

    ThumbTarget(){}

    void init(SourceInfo sourceInfo, void *cpuPtr);

    const SourceInfo &getSourceInfo() {return sourceInfo;}

    bool compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint32_t pc, GenBlockInfo &blockInfo);

    uint8_t *compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize);

private:
    void loadLiteral(ThumbBuilder &builder, Reg reg, uint32_t val);
    void outputLiterals(ThumbBuilder &builder, bool reachable = true);

    void loadPCValue(ThumbBuilder &builder, uint32_t val);

    void setFlags32(ThumbBuilder &builder, uint8_t instrFlags);

    std::optional<Reg> mapReg(uint8_t index);
    std::optional<RegInfo> mapReg8(uint8_t index);

    SourceFlagInfo &getFlagInfo(SourceFlagType flag);
    uint8_t flagWriteMask(SourceFlagType flag);
    bool writesFlag(uint16_t opFlags, SourceFlagType flag);

    SourceInfo sourceInfo;
    void *cpuPtr;

    std::map<uint8_t, Reg> regAlloc;

    uint8_t flagsReg = 0;
    uint8_t flagMap[6]; // map from SourceFlagType to flags bit

    uint16_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;

    // literals
    uint32_t literals[8]{};

    unsigned int curLiteral = 0;
    std::vector<uint16_t *> ldrLiteralInstrs;
};