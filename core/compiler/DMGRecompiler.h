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

class DMGCPU;

class DMGRecompiler
{
public:
    DMGRecompiler(DMGCPU &cpu);
    ~DMGRecompiler() = default;

    void handleBranch();

protected:
    DMGCPU &cpu;

    void analyse(uint16_t pc, uint16_t endPC, GenBlockInfo &blockInfo);

    void convertToGeneric(uint16_t &pc, GenBlockInfo &genBlock);

    void compileEntry();

    static void cycleExecuted(DMGCPU *cpu);
    static uint8_t readMem(DMGCPU *cpu, uint16_t addr);
    static int writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data, int cyclesToRun);

    uint8_t *codeBuf = nullptr, *curCodePtr;
    unsigned int codeBufSize;

    // cycles, entryAddr
    using CompiledFunc = void(*)(int, uint8_t *);

    struct FuncInfo
    {
        uint8_t *startPtr;
        uint8_t *endPtr;
        uint32_t endPC;
    };

    std::map<uint32_t, FuncInfo> compiled;
    uint16_t minRAMCode = 0xFFFF;

    // common code
    CompiledFunc entryFunc = nullptr;
    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;

    // saved pointer on exit
    uint8_t *tmpSavedPtr = nullptr;
    bool exitCallFlag = false; // if we exited because of a call

    static const int savedExitsSize = 16;
    int curSavedExit = 0;
    std::tuple<uint8_t *, uint32_t> savedExits[savedExitsSize];

#if defined(RECOMPILER_X86)
    X86Target target;
#elif defined(RECOMPILER_THUMB)
    ThumbTarget target;
#endif
};