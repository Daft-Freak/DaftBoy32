#pragma once

#include <map>

#include "RecompilerGeneric.h"

class X86Target final
{
public:
    X86Target(){}

    void init(SourceInfo sourceInfo, void *cpuPtr);

    const SourceInfo &getSourceInfo() {return sourceInfo;}

    bool compile(uint8_t *&codePtr, uint16_t pc, GenBlockInfo &blockInfo);

    void compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize);

private:
    SourceInfo sourceInfo;
    void *cpuPtr;

    std::map<uint8_t, int> regAlloc;

public: // FIXME
    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;
};