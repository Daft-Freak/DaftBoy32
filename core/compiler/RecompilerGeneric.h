#pragma once
#include <cstdint>
#include <vector>

enum class GenOpcode : uint8_t
{
    NOP = 0,
    LoadImm, // always to reg 0
    Move,
    Load,
    Store,

    Add,
    AddWithCarry,
    And,
    Compare,
    Or,
    Subtract,
    SubtractWithCarry,
    Xor,

    Not,

    RotateLeft,
    RotateLeftCarry,
    RotateRight,
    RotateRightCarry,
    ShiftLeft,
    ShiftRightArith,
    ShiftRightLogic,

    Jump,

    // not-so-generic
    DMG_Stop = 0x80,
    DMG_DAA,
    DMG_Halt,
    DMG_EnableIntrForRet, // RETI
    DMG_DI,
    DMG_EI,
};

// 1st src of jump
enum class GenCondition : uint8_t
{
    Equal = 0,
    NotEqual,
    CarrySet,
    CarryClear,
    Negative,
    Positive,
    OverflowSet,
    OverflowClear,
    Higher, // C == 1 && Z == 0
    LowerSame, // C == 0 || Z == 1
    GreaterEqual, // N == V
    LessThan, // N != V
    GreaterThan, // Z == 0 && N == V
    LessThanEqual, // Z == 1 || N != V
    Always
};

enum GenOpFlags
{
    GenOp_PreserveFlags = 0xF,

    GenOp_WriteFlags = 0xF << 4,

    GenOp_Call = 1 << 8, // branch is a call, code will return here later
    GenOp_BranchTarget = 1 << 9,
    GenOp_Exit = 1 << 10,

    GenOp_MagicAlt1 = 1 << 11, // for when an op sometimes has slightly different behaviour
    GenOp_MagicAlt2 = 1 << 12,
};

struct GenOpInfo
{
    GenOpcode opcode;
    uint8_t cycles;
    uint8_t len;
    union
    {
        struct
        {
            uint8_t src[2], dst[1];
        };
        uint32_t imm;
    };
    uint16_t flags;
};

enum GenBlockFlags
{
    GenBlock_StrictSync = 1 << 0, // call cycleExecuted for each cycle
    GenBlock_FirstCycleEarly = 1 << 1, // "execute" first cycle before ops

    GenBlock_DMGSPWrite = 1 << 16, // SP is set to point at regs or unknown addr in this block
};

struct GenBlockInfo
{
    std::vector<GenOpInfo> instructions;
    uint32_t flags;
};

enum class SourceRegType : uint8_t
{
    General = 0,
    Flags,
    Temp, // internal
};

struct SourceRegInfo
{
    const char *label;
    uint8_t size; // bits
    SourceRegType type;
    uint8_t alias; // this reg is part of a larger reg
    uint32_t aliasMask;
    uint16_t cpuOffset; // for load/store
};

enum class SourceFlagType : uint8_t
{
    Carry = 0,
    HalfCarry,
    WasSub, // the Z80 N flag
    Zero,
    Negative,
    Overflow
};

struct SourceFlagInfo
{
    char label;
    uint8_t bit;
    SourceFlagType type;
};

struct SourceInfo
{
    std::vector<SourceRegInfo> registers;
    std::vector<SourceFlagInfo> flags;

    uint8_t pcSize;
    uint8_t pcPrefetch; // offset to add on exit for prefetch
    uint16_t pcOffset;

    uint8_t cycleMul; // cycles to add for each cycle in an op

    int extraCPUOffsets[5]; // source specific ops

    bool (*shouldSyncForAddress)(uint16_t addr); // return true to sync cycle count before accessing this address
    bool (*shouldSyncForRegIndex)(uint8_t reg, const GenBlockInfo &block);

    uint16_t (*getRegOffset)(void *cpu, uint8_t reg);

    // emulator interface
    bool *exitCallFlag;
    uint8_t **savedExitPtr;

    uint32_t *cycleCount;
    void (*cycleExecuted)(void *cpu);

    uint8_t (*readMem)(void *cpu, uint16_t addr);
    int (*writeMem)(void *cpu, uint16_t addr, uint8_t data, int cyclesToRun);

    // TODO: unify
    uint8_t (*readMem8)(void *cpu, uint32_t addr, int &cycles, bool sequential);
};

void analyseGenBlock(uint32_t pc, uint32_t endPC, GenBlockInfo &blockInfo, const SourceInfo &sourceInfo);

void printGenBlock(uint32_t pc, const GenBlockInfo &block, const SourceInfo &sourceInfo);