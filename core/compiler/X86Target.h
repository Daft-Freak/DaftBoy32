#pragma once

#include <map>
#include <optional>

#include "RecompilerGeneric.h"

enum class Reg8;
enum class Reg16;
enum class Reg32;
enum class Reg64;

class X86Target final
{
public:
    X86Target(){}

    void init(SourceInfo sourceInfo, void *cpuPtr);

    const SourceInfo &getSourceInfo() {return sourceInfo;}

    bool compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint16_t pc, GenBlockInfo &blockInfo);

    void compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize);

private:
    std::optional<Reg8> mapReg8(uint8_t index);
    std::optional<Reg16> mapReg16(uint8_t index);
    std::optional<Reg32> mapReg32(uint8_t index);
    std::optional<Reg64> mapReg64(uint8_t index);

    SourceInfo sourceInfo;
    void *cpuPtr;

    std::map<uint8_t, Reg32> regAlloc;

public: // FIXME
    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;
};