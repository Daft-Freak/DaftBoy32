#pragma once
#include <cstdint>
#include <map>
#include <vector>

#include "RecompilerGeneric.h"

#if defined(RECOMPILER_X86)
#include "X86Target.h"
#elif defined(RECOMPILER_THUMB)
#include "ThumbTarget.h"
#else
#error No recompiler target!
#endif

class AGBCPU;

class AGBRecompiler
{
public:
    AGBRecompiler(AGBCPU &cpu);
    ~AGBRecompiler() = default;

    int run(int cyclesToRun);

protected:
    AGBCPU &cpu;

    bool attemptToRun(int cyclesToRun, int &cyclesExecuted);

    void convertTHUMBToGeneric(uint32_t &pc, GenBlockInfo &genBlock);

    void compileEntry();

    static uint8_t readMem8(AGBCPU *cpu, uint32_t addr, int &cycles, uint8_t flags);
    static uint32_t readMem16(AGBCPU *cpu, uint32_t addr, int &cycles, uint8_t flags);
    static uint32_t readMem32(AGBCPU *cpu, uint32_t addr, int &cycles, uint8_t flags);

    static int writeMem8(AGBCPU *cpu, uint32_t addr, uint8_t data, int &cycles, uint8_t flags, int cyclesToRun);
    static int writeMem16(AGBCPU *cpu, uint32_t addr, uint16_t data, int &cycles, uint8_t flags, int cyclesToRun);
    static int writeMem32(AGBCPU *cpu, uint32_t addr, uint32_t data, int &cycles, uint8_t flags, int cyclesToRun);

    static void invalidateCode(AGBCPU *cpu, uint32_t addr);
    static int updateCyclesForWrite(AGBCPU *cpu, int cyclesToRun);

    /*
    more callbacks
    static void updatePCTHUMB(AGBCPU *cpu, uint32_t addr);
    static void updatePCInterworked(AGBCPU *cpu, uint32_t addr);

    static void handleSWI(AGBCPU *cpu, uint32_t cpsrCond);
    */

    uint8_t *codeBuf = nullptr, *curCodePtr;
    unsigned int codeBufSize;

    // cycles, entryAddr
    using CompiledFunc = void(*)(int, uint8_t *);

    struct FuncInfo
    {
        uint8_t *startPtr;
        uint8_t *endPtr;
        uint32_t endPC;
        uint8_t cpsrMode;
    };

    std::map<uint32_t, FuncInfo> compiled;
    uint32_t minRAMCode = 0xFFFFFFFF, maxRAMCode = 0;
    std::map<uint32_t, FuncInfo>::iterator ramStartIt;

    // common code
    CompiledFunc entryFunc = nullptr;

    // saved pointer on exit
    uint8_t *tmpSavedPtr = nullptr;
    bool exitCallFlag = false; // if we exited because of a call

    static const int savedExitsSize = 16;
    int curSavedExit = 0;
    std::tuple<uint8_t *, uint32_t, uint32_t> savedExits[savedExitsSize];

#if defined(RECOMPILER_X86)
    X86Target target;
#elif defined(RECOMPILER_THUMB)
    ThumbTarget target;
#endif
};