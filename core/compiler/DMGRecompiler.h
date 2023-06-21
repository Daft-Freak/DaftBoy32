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
    enum OpFlags
    {
        Op_ReadC = 1 << 0,
        Op_ReadH = 1 << 1,
        Op_ReadN = 1 << 2,
        Op_ReadZ = 1 << 3,
        Op_ReadFlags = Op_ReadC | Op_ReadH | Op_ReadN | Op_ReadZ,

        // these match the bits in the F register
        Op_WriteC = 1 << 4,
        Op_WriteH = 1 << 5,
        Op_WriteN = 1 << 6,
        Op_WriteZ = 1 << 7,
        Op_WriteFlags = Op_WriteC | Op_WriteH | Op_WriteN | Op_WriteZ,

        Op_Branch = 1 << 8,
        Op_BranchTarget = 1 << 9,
        Op_Exit = 1 << 10,

        Op_Load = 1 << 11,
        Op_Store = 1 << 12,

        Op_Last = 1 << 13, // last instruction in compiled block, compiler can't see this itself
    };

    enum RegFlags
    {
        Reg_A = 1 << 0,
        Reg_SP = 1 << 1, // this would be F, but we track flags seperately
        Reg_B = 1 << 2,
        Reg_C = 1 << 3,
        Reg_D = 1 << 4,
        Reg_E = 1 << 5,
        Reg_H = 1 << 6,
        Reg_L = 1 << 7
    };

    struct OpInfo
    {
        uint8_t opcode[3];
        uint8_t len; // 1-3
        uint8_t regsRead, regsWritten;
        uint16_t flags;
    };

    struct BlockInfo
    {
        std::vector<OpInfo> instructions;
    };

    DMGCPU &cpu;

    void gatherBlock(uint16_t &pc, BlockInfo &blockInfo);
    void analyse(uint16_t pc, uint16_t endPC, BlockInfo &blockInfo);

    bool convertToGeneric(uint16_t pc, BlockInfo &block, GenBlockInfo &genBlock);

    void printInfo(BlockInfo &blockInfo);

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