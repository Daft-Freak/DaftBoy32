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
    Always
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

struct GenBlockInfo
{
    std::vector<GenOpInfo> instructions;
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
    Negative,
    Zero
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
    uint16_t pcOffset;

    // emulator interface
    bool *exitCallFlag;
    uint8_t **savedExitPtr;
};