#pragma once
#include <cstdint>
#include <map>
#include <vector>

class AGBCPU;

class AGBRecompiler
{
public:
    AGBRecompiler(AGBCPU &cpu);
    ~AGBRecompiler() = default;

    int handleBranch(int cyclesToRun);

protected:
    enum OpFlags
    {
        Op_ReadV = 1 << 0,
        Op_ReadC = 1 << 1,
        Op_ReadZ = 1 << 2,
        Op_ReadN = 1 << 3,
        Op_ReadFlags = Op_ReadV | Op_ReadC | Op_ReadZ | Op_ReadN,

        Op_WriteV = 1 << 4,
        Op_WriteC = 1 << 5,
        Op_WriteZ = 1 << 6,
        Op_WriteN = 1 << 7,
        Op_WriteFlags = Op_WriteV | Op_WriteC | Op_WriteZ | Op_WriteN,

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
        Reg_SP = 1 << 1,
        Reg_B = 1 << 2,
        Reg_C = 1 << 3,
        Reg_D = 1 << 4,
        Reg_E = 1 << 5,
        Reg_H = 1 << 6,
        Reg_L = 1 << 7
    };

    struct OpInfo
    {
        uint32_t opcode;
        uint16_t regsRead, regsWritten;
        uint16_t flags;
    };

    struct BlockInfo
    {
        std::vector<OpInfo> instructions;
    };

    AGBCPU &cpu;

    void analyseTHUMB(uint32_t &pc, BlockInfo &blockInfo);

    void printInfo(BlockInfo &blockInfo);

    virtual bool compileTHUMB(uint8_t *&codePtr, uint32_t pc, BlockInfo &blockInfo) = 0;

    virtual void compileEntry() = 0;

    static uint8_t readMem8(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential);
    static uint16_t readMem16(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential);
    static uint32_t readMem32(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential);
    static int writeMem8(AGBCPU *cpu, uint32_t addr, uint8_t data, int &cycles, bool sequential, int cyclesToRun);
    static int writeMem16(AGBCPU *cpu, uint32_t addr, uint16_t data, int &cycles, bool sequential, int cyclesToRun);
    static int writeMem32(AGBCPU *cpu, uint32_t addr, uint32_t data, int &cycles, bool sequential, int cyclesToRun);

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

    // common code
    CompiledFunc entryFunc = nullptr;
    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;

    // saved pointer on exit
    uint8_t *tmpSavedPtr = nullptr;
    bool exitCallFlag = false; // if we exited because of a call

    static const int savedExitsSize = 16;
    int curSavedExit = 0;
    std::tuple<uint8_t *, uint32_t> savedExits[savedExitsSize];

    // compile state
    // TODO: make these temps
    /*uint8_t *lastInstrCycleCheck = nullptr;
    std::map<uint32_t, uint8_t *> branchTargets;
    std::multimap<uint32_t, uint8_t *> forwardBranchesToPatch;
    bool spWrite = false;*/
};