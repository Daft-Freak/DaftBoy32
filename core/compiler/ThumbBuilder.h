#pragma once
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

// first 8
enum class LowReg
{
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7
};

class ThumbBuilder final
{
public:
    ThumbBuilder(uint16_t *ptr, uint16_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void bx(Reg r);

    uint16_t *getPtr() const {return ptr;}

    void resetPtr(uint16_t *oldPtr);

    bool getError() const {return error;}

private:
    void write(uint16_t hw);

    uint16_t *ptr, *endPtr;

    bool error = false; // super basic error handling
};
