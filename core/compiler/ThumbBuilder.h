#pragma once
#include <cassert>
#include <cstdint>

enum class Reg
{
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15,

    // aliases
    SP = R13,
    LR = R14,
    PC = R15,
};

// wrapper that asserts reg is in range
class LowReg final
{
public:
    constexpr LowReg(Reg r): val(r)
    {
        assert(static_cast<int>(r) < 8);
    }

    Reg val;
};

class ThumbBuilder final
{
public:
    ThumbBuilder(uint16_t *ptr, uint16_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void bl(int32_t off);

    void bx(Reg r);

    void ldr(LowReg t, uint16_t imm);

    void lsl(LowReg d, LowReg m, uint8_t imm);

    void mov(LowReg r, uint8_t imm);
    void mov(Reg d, Reg m);

    void orr(LowReg d, LowReg m);

    void pop(uint8_t regList, bool pc);

    void push(uint8_t regList, bool lr);

    void strh(LowReg t, LowReg n, LowReg m);

    uint16_t *getPtr() const {return ptr;}

    void resetPtr(uint16_t *oldPtr);

    bool getError() const {return error;}

private:
    void write(uint16_t hw);

    uint16_t *ptr, *endPtr;

    bool error = false; // super basic error handling
};
